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
 * Do not forget to define the radio type correctly in config.h ( i.e. in the
 * LMIC library config.h ).
 *
 * 2019-04-18 - Hacked for TTGO T-beam and air quality monitoring by Mark de Roussier
 *
 *******************************************************************************/

/**
* @file ttn-otaa.cpp
* @author MdeR
* @date 26 Apr 2019
* @copyright 2019 MdeR
* @brief This file contains the LMIC based task framework for managing reading of
* sensors and transmission of readings via Lorawan.
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <cstdint>
#include <WiFi.h>

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
static const u1_t PROGMEM DEVEUI[8]={ 0x9A, 0x7C, 0x0C, 0xBD, 0xB8, 0xF1, 0xE6, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x3D, 0x94, 0x0B, 0x5E, 0xFF, 0x2B, 0x2F, 0xF2, 0x7F, 0x31, 0x29, 0x0C, 0x67, 0xC0, 0xC5, 0xED };
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
  .rst = LMIC_UNUSED_PIN,
  .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};


/**
* @brief Generate a bitwise encoding of available sensor values
* @param [in] none
* @return none
* @details Note that this function does not cause sensors to provide
* readings. It discovers which readings are actually available at this
* point in time, and directs the encode function to encode those values,
* which are retrieved from cache.
* TBD - at the moment, the actual reading and the encoding are tightly
* coupled, but this design is intended to allow this constraint to be
* lifted.
*/

void do_encode(void)
{
struct encoder::readings values;
uint16_t validValuesMask = 0;

    // Attempt to get the readings specified by encoder::datamask
    // Note that we may not get all the readings we expect if the sensor
    // did not respond to the trigger.
    validValuesMask = getreadings::getReadings( &values );

    Serial.print(F("Valid readings 0x"));
    Serial.println(validValuesMask, HEX);

    Serial.print(F("MAX_BUFFER_BYTES : "));
    Serial.println( encoder::MAX_BUFFER_BYTES );

    // clean the output buffer
    memset( mydata, 0, encoder::MAX_BUFFER_BYTES );

    // Only encode the set of readings which were actually valid
    // NB this might be no readings at all.
    bytesUsed = encoder::encode( mydata, &values, validValuesMask );

    Serial.print(F("Bytes used : "));
    Serial.println(bytesUsed);
    Serial.print(F("Message ( hex bytes ) : "));

    int i;
    for( i=0; i<(bytesUsed-1) ; i++ )
    {
        Serial.print(mydata[i],HEX);
        Serial.print(",");
    }

    Serial.println(mydata[i],HEX);
}

/**
* @brief First actions for a sensor reading.
* @param [in] j - OS context info for this job
* @return N/A
* @details Although some sensors can be read immediately after being powered up, some
* require a 'wakeup period' before a valid reading can be obtained. So we deal with
* this by implementing a reading in two phases. This is the first phase, that does
* the wakeup of sensors.
*/

void do_sensorread_phase1(osjob_t* j)
{
    Serial.println(F("Job ReadPhase1"));
    // This phase just exists to wake up any sensors that
    // need waking up.
    sensors::sensorWakeup();
}

/**
* @brief Read sensor values
* @param [in] j - OS context info for this job
* @return N/A
* @details This job does two things - it reads from the sensors, and it encodes the
* available readings in the form that will actually be transmitted. It *does not* actually
* perform the transmit.
*
* The rationale for seperating the transmit is just to minimise any uncertainty about the
* interval between queuing packets, i.e. we do not want to have a variable amount of work to
* do at the point in time that transmission is required.
*
* The actual transmit interval is not entirely within our control, as LORA is a slotted
* protocol and transmissions by other parties may affect when we are actually allowed to transmit.
*/

void do_sensorread_phase2(osjob_t* j)
{
    // This is the phase that reads from the sensors.
    Serial.println(F("Job ReadPhase2"));

    // Interlock -
    // If there is a transmission in progress and hence a risk that the output buffer may be
    // being read from ( should not be, but hey this is radio...) then skip this read -
    // reading will be triggered again by the EV_TXCOMPLETE event.

    if ( ! ( LMIC.opmode & OP_TXRXPEND ) )
    {
        // Cause the sensors to provide readings where possible, and write them to cache.
        if( sensors::sensorTrigger() )
        {
            // encode the readings in the output buffer
            // Note that the readings used here are taken from a cache, not directly from the sensor.
            do_encode();
        }
        else
        {
            Serial.println(F("ReadPhase2 READING SKIPPED - trigger failure"));
        }
    }
    else
    {
        Serial.println(F("ReadPhase2 READING SKIPPED - TX in progress"));
    }
}

/**
* @brief Queue a set of readings for the next slot.
* @param [in] j - OS context info for this job
* @return N/A
* @details Verify that we are allowed to queue a new message, and if we are then
* queue a message for the next available slot.
*/

void do_send(osjob_t* j)
{
    Serial.println(F("Job Send"));

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
* @brief Obtain an address for DEVEUI construction
* @param [in] buffer - enough bytes to hold whatever form of address we are using
* @return none
* @details It's not necessarily the case that a MAC address must be used -
* the purpose of this function is simply to supply a set of bytes that will
* serve to uniquely identify the device it is running on, in the context of
* all the other devices that will be attached to the application.
*/

void getAddress( uint8_t *buffer )
{
    // Currently the address we will use is the WiFi MAC address.
    WiFi.macAddress( buffer );
}

/**
* @brief Arduino framework setup function.
* @param [in] None
* @return N/A
* @details Perform one - time power up initialisations. In particular, send a dummy
* packet, because the successful transmission of a packet is required in order to
* start the normal schedule of sensor reading/transmission, as well as initiating the
* OTAA exchange.
*/

void setup( void )
{
uint8_t macAddress[6] = { 0,0,0,0,0,0 };

    //find out our MAC address
    getAddress( macAddress );

    //Turn off WiFi and Bluetooth ( or at least, try to... TBD )
    WiFi.mode(WIFI_OFF);
    btStop();

    //TBD pause for log viewing convenience
    delay(5000);

    // Serial port for programming/debugging. Is serial 0, i.e. U0UXD, TX/GPIO1, RX/GPIO3
    // See https://circuits4you.com/2018/12/31/esp32-hardware-serial2-example/
    Serial.begin(115200);

    Serial.print(F("MAC Address : "));
    int i = 5;
    for( ; i>0; i-- )
    {
        Serial.print( macAddress[i], HEX );
        Serial.print(F(":"));
    }
    Serial.println( macAddress[i], HEX );

    Serial.print(F("Starting, LORA tx interval (s) : "));
    Serial.println(ttnotaa::TX_INTERVAL);
    Serial.print(F("MAX_CHANNELS : "));
    Serial.println( ::MAX_CHANNELS );
    Serial.print(F("MAX_BANDS : "));
    Serial.println( ::MAX_BANDS );
    Serial.print(F("OSTICKS_PER_SEC : "));
    Serial.println( OSTICKS_PER_SEC );

    // LMIC init
    os_init();

    // 17 is the max power supported by the LMIC MAC
    LMIC.txpow = 17;

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Perform initialisation for sensors
    if ( sensors::sensorInitSensors() )
    {
        // Note that we have not scheduled any timed jobs yet.
        // Further scheduling of jobs occurs on receipt of EV_TXCOMPLETE.

        // To kick off, encode a blank message - no sensor readings have yet been triggered
        do_encode();

        // Start job (sending automatically starts OTAA too)
        do_send(&sendjob);
    }
    else
    {
        Serial.println(F("Failed to init sensors"));
        exit(-1);
    }
}

/**
* @brief Handle events generated by LMIC about the status of the radio subsystem
* @param [in] ev - LMIC Event
* @return N/A
* @details The key event here is EV_TXCOMPLETE. Completion of one transmission triggers
* the scheduling of further activity, in particular the reading of sensors and a subsequent
* further transmission.
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

                // phase1Offset is the time in secs we need to allow between waking up the sensors and performing the transmit
                int phase1Offset = (( sensors::sensorMaxWakeup() + sensors::sensorMaxReadtime() )/1000 ) ;
                int wakeupTime = os_getTime()+sec2osticks(ttnotaa::TX_INTERVAL - phase1Offset );

                Serial.print(F("Wakeup time :"));
                Serial.println( wakeupTime );

                // read time is wakeup time plus the wakeup period. 1s leeway.
                int readTime = wakeupTime + sec2osticks( 1 + sensors::sensorMaxWakeup()/1000 );

                Serial.print(F("Read time :"));
                Serial.println( readTime );

                // send time is read time plus the maximum time to read. 1s leeway.
                int sendTime = readTime + sec2osticks( 1 + sensors::sensorMaxReadtime()/1000 );

                Serial.print(F("Send time :"));
                Serial.println( sendTime );

                // The TX_INTERVAL must be bigger than the read time offset.
                // Preferably much bigger, or we will use a lot more power.
                assert( ttnotaa::TX_INTERVAL > phase1Offset );

                // Schedule phase 1
                // We want to wake up the sensors at a time as close as possible to the following transmission, which
                // is why we don't just do it now. This time is calculated by applying the phase1Offset.
                os_setTimedCallback(&sensorreadphase1job, wakeupTime, do_sensorread_phase1);

                // Schedule phase 2 - this is where the readings are actually taken
                os_setTimedCallback(&sensorreadphase2job, readTime, do_sensorread_phase2);

                // Schedule next transmission
                // No sensor reading is done at this time, only sending of latest readings.
                os_setTimedCallback(&sendjob, sendTime, do_send);

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
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event 0x"));
            Serial.println( ev, HEX );
            break;
    }
}

/**
* @brief Arduino framework main loop
* @param [in] None
* @return N/A
* @details In effect, this starts the internal LMIC event/job handling loop.
*/

void loop( void )
{
    os_runloop_once();
}

