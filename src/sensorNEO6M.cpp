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

static struct sensors::sensorNEO6MReadings sensorNEO6MLatestReadings;

/**
* @brief Configure hardware serial for GPS Module comms
* @param [in] None
* @return true if initialisation was successful, else false
* @details Not much to do here. Set up the serial comms with the
* GPS module. Would like to put the module into low power mode
* inbetween reads, but don't know how to do that - TBD
*/

bool sensors::sensorNEO6MInit( void )
{
    Serial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX

    Serial1.setRxBufferSize( sensors::NEO6M_SERIAL_HARDWAREBUFFERSIZE );

    sensors::sensorStatus[ sensors::SENSOR_ID_NEO6M ] = true;

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
unsigned long start = millis();

    do
    {
        while (Serial1.available())
        {
            gps.encode(Serial1.read());
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
