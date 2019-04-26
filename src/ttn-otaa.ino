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
#include <ttnotaa.hpp>

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
static const u1_t PROGMEM APPKEY[16] = { 0xEA, 0x65, 0xE4, 0xE8, 0xDA, 0x84, 0x07, 0x02, 0x91, 0x6E, 0x8E, 0x8A, 0x3B, 0x1C, 0xB9, 0xF6 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// These variables are used to pass data between the sensorread job and the send job
// Assume this can be done without interlocks.
static uint8_t  mydata[ encoder::MAX_BUFFER_BYTES ];
static int      bytesUsed = 0;

// Three jobs are triggered by successful transmission, two for sensor reading and
// then the sending of the sensor readings.
static osjob_t  sensorreadphase1job;
static osjob_t  sensorreadphase2job;
static osjob_t  sendjob;

// Pin mapping
const lmic_pinmap lmic_pins =
{
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

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

                // phase1Offset is the time we need to allow between waking up the sensors and performing the transmit
                int phase1Offset = (( sensors::sensorMaxWakeup() + sensors::sensorMaxReadtime() )/1000 ) ;

                // The TX_INTERVAL must be bigger than the read time offset.
                // Preferably much bigger, or we will use a lot more power.
                assert( ttnotaa::TX_INTERVAL > phase1Offset );

                // Schedule phase 1
                // We want to read the sensors at a time as close as possible to the following transmission, which
                // is why we don't just do it now. This time is calculated by applying the phase1Offset.
                os_setTimedCallback(&sensorreadphase1job, os_getTime()+sec2osticks(ttnotaa::TX_INTERVAL - phase1Offset ), do_sensorread_phase1);

                // Schedule phase 2, allowing time for wakeup to complete
                int phase2Offset = ( sensors::sensorMaxWakeup() / 1000 ) ;

                // Schedule phase 2 - this is where the readings are actually taken
                os_setTimedCallback(&sensorreadphase2job, os_getTime()+sec2osticks(ttnotaa::TX_INTERVAL - phase2Offset ), do_sensorread_phase2);

                // Schedule next transmission
                // No sensor reading is done at this time, only sending of latest readings.
                os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(ttnotaa::TX_INTERVAL), do_send);
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

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void do_sensorread_phase1(osjob_t* j)
{
    // This phase just exists to wake up any sensors that
    // need waking up.
    sensors::sensorWakeup();
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void do_sensorread_phase2(osjob_t* j)
{
    // This is the phase that reads from the sensors.

    // Interlock -
    // If there is a transmission in progress and hence a risk that the output buffer may be
    // being read from ( should not be, but hey this is radio...) then skip this read -
    // reading will be triggered again by the EV_TXCOMPLETE event.

    if ( ! ( LMIC.opmode & OP_TXRXPEND ) )
    {
        // Cause the sensors to provide readings
        if( sensors::sensorTrigger() )
        {
        struct encoder::readings values;
        uint8_t validValuesMask = 0;

            // Attempt to get the readings specified by encoder::datamask
            // Note that we may not get all the readings we expect if the sensor
            // did not respond to the trigger.
            validValuesMask = getreadings::getReadings( &values );

            Serial.print(F("Valid readings "));
            Serial.println(validValuesMask);

            // clean the output buffer
            memset( mydata, 0, encoder::MAX_BUFFER_BYTES );

            // Only encode the set of readings which were actually valid
            bytesUsed = encoder::encode( mydata, &values, validValuesMask );

            Serial.print(F("Bytes used "));
            Serial.println(bytesUsed);
        }
        else
        {
            Serial.println(F("READING SKIPPED 1"));
        }
    }
    else
    {
        Serial.println(F("READING SKIPPED 2"));
    }
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

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

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

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
    if ( sensors::sensorInitSensors() )
    {
        // Note that we have not scheduled any timed jobs yet.
        // Note that this first sensor read occurs before OTAA,
        // this will be triggered when the sensor read has
        // completed.

        do_sensorread_phase1(&sensorreadphase1job);
    }
    else
    {
        Serial.println(F("Failed to init sensors"));
        exit(-1);
    }
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void loop()
{
    os_runloop_once();
}
