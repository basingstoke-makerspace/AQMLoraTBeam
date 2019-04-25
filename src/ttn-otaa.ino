/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy.
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 * 2019-04-18 - Hacked for TTGO T-beam and air quality monitoring by Mark de Roussier
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <cstdint>
#include <encoder.hpp>
#include <getreadings.hpp>
#include <sensors.hpp>

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t PROGMEM APPEUI[8]={ 0xDF, 0xAF, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
// 00 A1 3E 3F C6 41 4F F7
static const u1_t PROGMEM DEVEUI[8]={ 0xF7, 0x4F, 0x41, 0xC6, 0x3F, 0x3E, 0xA1, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xEA, 0x65, 0xE4, 0xE8, 0xDA, 0x84, 0x07, 0x02, 0x91, 0x6E, 0x8E, 0x8A, 0x3B, 0x1C, 0xB9, 0xF6 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// These variables are used to pass data between the sensorread job and the send job
// Assume this can be done without interlocks.
static uint8_t  mydata[ encoder::MAX_BUFFER_BYTES ];
static int      bytesUsed = 0;

// Two jobs are triggered by successful transmission, namely reading of the sensors and
// sending of the sensor readings.
static osjob_t  sensorreadjob;
static osjob_t  sendjob;

// Pin mapping
const lmic_pinmap lmic_pins =
{
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

void onEvent (ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev)
    {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            {
                Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

                if (LMIC.txrxFlags & TXRX_ACK)
                {
                  Serial.println(F("Received ack"));
                }
                if (LMIC.dataLen)
                {
                  Serial.println(F("Received "));
                  Serial.println(LMIC.dataLen);
                  Serial.println(F(" bytes of payload"));
                }

                // Schedule next attempt to read sensors
                // This must be far enough in advance of the send to allow time for the sensor(s) to
                // wake up and supply data.
                int readTimeOffset = ( sensors::SDS011_WARMUP + sensors::SDS011_MAX_READ_TIME )/1000 ;
                // The TX_INTERVAL must be bigger than the read time offset.
                // Preferably much bigger, or we will use a lot more power.
                assert( getreadings::TX_INTERVAL > readTimeOffset );
                os_setTimedCallback(&sensorreadjob, os_getTime()+sec2osticks(getreadings::TX_INTERVAL - readTimeOffset ), do_sensorread);

                // Schedule next transmission
                // No sensor reading is done at this time, only sending of latest readings.
                os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(getreadings::TX_INTERVAL), do_send);
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_sensorread(osjob_t* j)
{
    struct encoder::readings values;
    uint8_t validValuesMask = 0;

    // Interlock -
    // If there is a transmission in progress ( should not be, but hey this is radio...)
    // then skip this read - reading will be triggered again by the EV_TXCOMPLETE event.

    if ( ! ( LMIC.opmode & OP_TXRXPEND ) )
    {
        // clean the output buffer
        memset( mydata, 0, encoder::MAX_BUFFER_BYTES );

        // Attempt to read the sensors specified by encoder::datamask
        // Note that we may not get all the readings we expect if the sensor
        // is not responding.
        validValuesMask = getreadings::getReadings( &values );

        Serial.print(F("Valid readings "));
        Serial.println(validValuesMask);

        // Only encode the set of readings which were actually valid
        bytesUsed = encoder::encode( mydata, &values, validValuesMask );

        Serial.print(F("Bytes used "));
        Serial.println(bytesUsed);
    }
    else
    {
        Serial.println(F("READING SKIPPED"));
    }
}

void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, bytesUsed, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after EV_TXCOMPLETE event.
}

void setup()
{
    // Serial port for programming/debugging. Is serial 0, i.e. U0UXD, TX/GPIO1, RX/GPIO3
    // See https://circuits4you.com/2018/12/31/esp32-hardware-serial2-example/
    Serial.begin(115200);

    while( !Serial ){}

    Serial.println(F("Starting"));
    Serial.println(F("120"));

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Perform initialisation for sensors
    if ( sensors::initSensors() )
    {
        // Note that we have not scheduled any timed jobs yet.

        // get the first sensor readings.
        do_sensorread(&sensorreadjob);

        // Start job (sending automatically starts OTAA too)
        do_send(&sendjob);
    }
    else
    {
        Serial.println(F("Failed to init sensors"));
        exit(-1);
    }
}

void loop()
{
    os_runloop_once();
}