/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This file defines data and code for the purpose of configuring ( initialising )
 * and reading from the NEO6M GPS sensor.
 *
 *******************************************************************************/

/**
* @file sensorNEO6M.cpp
* @author MdeR
* @date 26 Apr 2019
* @copyright 2019 MdeR
* @brief This file defines data and code for the purpose of configuring ( initialising )
* and reading from the NEO6M GPS sensor.
*/

#include <TinyGPS++.h>

#include <sensors.hpp>

using namespace sensors;

static TinyGPSPlus gps;

// Cache for sensor readings. Read by encoder.
static struct sensors::sensorNEO6MReadings sensorNEO6MLatestReadings;

static HardwareSerial* NEO6MSerialPtr = &Serial1;

/**
* @brief Calculate fletcher CRC for UBX messages
* @param [in] dataPtr   - ( pointer to ) bytes to create checksum for
*             size      - number of bytes to process from dataPtr
*       [out] outputPtr - ( pointer to ) two bytes to receive checksum
* @return None
* @details
*/

void sensors::sensorNEO6MUBXCRC( const uint8_t* dataPtr, uint32_t size, uint8_t* outputPtr )
{
uint32_t crc_a = 0;
uint32_t crc_b = 0;

    assert( dataPtr != NULL && outputPtr != NULL );

    if (size > 0)
    {
        do
        {
            crc_a += *dataPtr++;
            crc_b += crc_a;
        }
        while (--size);

        crc_a &= 0xff;
        crc_b &= 0xff;
    }

    Serial.print( F("NEO6M ubx checksum : "));
    Serial.print( crc_a, HEX );
    Serial.println( crc_b, HEX);

    *outputPtr = crc_a;
    outputPtr++;
    *outputPtr = crc_b;
}

/**
* @brief Send a UBX command to the NEO6M
* @param [in] cmd - id of the command to send
* @return true if command written OK, else false
* @details <details>
*/

bool sensors::sensorNEO6MSendCommand( uint32_t cmd )
{
int     retVal;
uint8_t crc[2];

    //Using ESP32 hardware UART U0UXD

    sensors::sensorNEO6MUBXCRC( sensors::NEO6MCommands[ cmd ].command, sensors::NEO6MCommands[ cmd ].length, crc );

    //Use two writes, otherwise would need to copy command into a writeable
    //buffer to add CRC.
    retVal = NEO6MSerialPtr->write( sensors::NEO6MCommands[ cmd ].command, sensors::NEO6MCommands[ cmd ].length );
    retVal += NEO6MSerialPtr->write( crc, 2 );

    if( retVal != ( sensors::NEO6MCommands[ cmd ].length + 2 ) )
    {
        Serial.print(F("NEO6M Command NOT sent : "));
        Serial.print( NEO6M_COMMAND_NAME[ cmd ] );
        Serial.print( " " );
        Serial.println( retVal );

        return false;
    }
    else
    {
        Serial.print(F("NEO6M Command sent : "));
        Serial.println( NEO6M_COMMAND_NAME[cmd] );

        return true;
    }
}

/**
* @brief Configure hardware serial for GPS Module comms
* @param [in] None
* @return true if initialisation was successful, else false
* @details This sensor is built in to the TTGO board and defaults
* to 'on' state. Therefore,if it is not configured, assume that
* it should be switched off and remain switched off. Otherwise,
* put it into Power Save Mode.
*/

bool sensors::sensorNEO6MInit( void )
{

    NEO6MSerialPtr->begin(9600, SERIAL_8N1, 12, 15);

    if( !sensors::sensorPresent(sensors::SENSOR_ID_NEO6M) )
    {
        // Switch off. This overrides PSM settings. The device will stay OFF
        // until a wake event, which is a hardware signal that it will never get.
        sensors::sensorNEO6MSendCommand( sensors::NEO6M_CMDID_RXM_PMREQ_BACKUP );

        // Sensor is not active - cannot provide a reading
        sensors::sensorStatus[ sensors::SENSOR_ID_NEO6M ] = false;

        Serial.println(F("Sensor NEO6M switched to Backup ( OFF )"));

        // Don't need the serial port again. NB This port is currently an alias
        // for 'Serial1', Serial1 will not be usable after this call without
        // another 'begin'.
        NEO6MSerialPtr->end();
    }
    else
    {
        // Put the device into Power Save Mode
        sensors::sensorNEO6MSendCommand( sensors::NEO6M_CMDID_CFG_RXM_PSM );

        // Configure power save mode
        sensors::sensorNEO6MSendCommand( sensors::NEO6M_CMDID_CFG_PM2_PSM );

        Serial.println(F("Sensor NEO6M configured in PSM, mode ON/OFF"));
    }

    Serial.println(F("Sensor NEO6M Init OK"));

    return true;
}

/**
* @brief Return the current value of the specified reading
* @param [in]   readingMask - set bit specifies reading required
*        [out]  valuePtr    - pointer to var to hold reading
* @return true if reading was found, else false
* @details Get the reading value requested from the private
* data store which was filled by the GetReadings function.
*/

bool sensors::sensorNEO6MRead( uint16_t readingMask, int* valuePtr )
{
bool retVal = true;

    if( readingMask & encoder::DATA_CONTAINS_LAT )
    {
        *valuePtr = sensorNEO6MLatestReadings.lat;
    }
    else
    if( readingMask & encoder::DATA_CONTAINS_LON )
    {
        *valuePtr = sensorNEO6MLatestReadings.lon;
    }
    else
    if( readingMask & encoder::DATA_CONTAINS_ALT )
    {
        *valuePtr = sensorNEO6MLatestReadings.alt;
    }
    else
    {
        Serial.println(F("NEO6M Invalid reading requested"));
        retVal = false;
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorNEO6MWakeup( void )
{
    Serial.println("Waking up NEO6M - nothing to do");
}

/**
* @brief Communicate with the sensor to get new readings
* @param [in] None
* @return true if all readings OK, else false
* @details Read all readings that this sensor is configured
* to supply. Shutdown the sensor afterwards if possible.
*/

bool sensors::sensorNEO6MGetReadings( void )
{
bool retVal = false;
unsigned long start;

    // Because the Arduino/ESP32 uart handling offers no better way of doing this,
    // ditch old queued characters ( there's a queue on top of the rxfifo...) by
    // reading them, and flush the rx/tx buffers. We don't want old data here.
    // The flush() call flushes the rx buffer ( crudely, by repeated reads )
    // despite the Arduino documentation only referring to the tx buffer ( and
    // then only to say that it doesn't actually flush it ).

    while( NEO6MSerialPtr->available() > 0 ) NEO6MSerialPtr->read();
    NEO6MSerialPtr->flush();

    start = millis();
    do
    {
        while (NEO6MSerialPtr->available())
        {
            gps.encode(NEO6MSerialPtr->read());
        }

        if( gps.sentencesWithFix() > 0 )
        {
            break;
        }

    }
    while ( (millis() - start) < sensors::NEO6M_LOCATION_TIMEOUT);

    if( gps.sentencesWithFix() == 0 )
    {
        Serial.println(F("NEO6M GPS fix not found before timeout"));
    }
    else
    {
    struct RawDegrees lat = gps.location.rawLat();
    struct RawDegrees lon = gps.location.rawLng();

        // Form a 32bit quantity by shifting the degrees to the msb's, and using the remaining
        // bits to store millionths ( not billionths, so we lose some data here - we're down to
        // 11cm resolution, which is enough ). Note that degrees provided by RawDegrees are unsigned.

        sensorNEO6MLatestReadings.lat   = ( lat.deg * sensors::NEO6M_DEG_TO_INT ) + ( lat.billionths/1000 );
        sensorNEO6MLatestReadings.lon   = ( lon.deg * sensors::NEO6M_DEG_TO_INT ) + ( lon.billionths/1000 );

        sensorNEO6MLatestReadings.alt   = gps.altitude.meters();

        Serial.print(F("NEO6M GPS lat "));
        Serial.println(sensorNEO6MLatestReadings.lat);
        Serial.print(F("NEO6M GPS lon "));
        Serial.println(sensorNEO6MLatestReadings.lon);
        Serial.print(F("NEO6M GPS alt "));
        Serial.println(sensorNEO6MLatestReadings.alt);

        retVal = true;
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorNEO6MSleep( void )
{
    return; // TBD
}
