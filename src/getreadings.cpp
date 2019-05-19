/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Get values for the readings ( assumed to be max 32 bit ) that we want to
 * return. Record the values that we can *actually* return, which may differ
 * from the values that we think we should be able to return.
 *
 *******************************************************************************/

/**
* @file getreadings.cpp
* @author MdeR
* @date 26 Apr 2019
* @copyright 2019 MdeR
* @brief This file provides an interface for obtaining readings. Ultimately
* the readings come from sensors, but this interface hides knowledge of
* sensor specifics and deals only in readings.
*/

#include <encoder.hpp>
#include <getreadings.hpp>
#include <sensors.hpp>
#include <ttnotaa.hpp>

using namespace getreadings;

/**
* @brief Get the current value of a particular reading
* @param [in] - readingMask- provides identity of the reading requested
*        [out]  valuePtr   - pointer to the value of the reading
* @return true if valid reading was obtained from sensor interface, else false.
* @details Use the readingMask to identify a sensor that can provide the requested
* reading, and then request this reading from the particular sensor. A sensor may
* supply several readings, so also provide the readingMask to the sensor.
*/

bool getreadings::getReading( int* valuePtr, uint16_t readingMask )
{
bool    retVal = false;
int     sensorId = 0xDEADBEEF;

    // is this a reading that we expect to return ?
    if( encoder::datamask & readingMask )
    {
        // do we have a working sensor that can provide this reading ?
        if( ( sensorId = sensors::sensorValid(readingMask) ) > -1 )
        {
            // get the latest reading

            // *******************************************************************************
            // *** NB this works only so long as a reading can be represented as an int :) ***
            // *******************************************************************************

            if( sensors::sensorReading( sensorId, readingMask, valuePtr ) )
            {
                retVal = true;
            }
            else
            {
                // reading error
                Serial.println(F("Reading Error"));
                retVal = false;
            }
        }
        else
        {
            // We are expecting to be able to return a value, but the
            // sensor has failed in some way.
            Serial.println(F("Sensor Failure"));
            retVal = false;
        }
    }
    else
    {
        // not configured to use this reading
        Serial.print(F("Reading 0x"));
        Serial.print(readingMask, HEX);
        Serial.println(F(" not configured"));

        retVal = false;
    }

    return retVal;
}

/**
* @brief Assemble available readings, recording the ones available.
* @param [in] readingsPtr - a convenience structure for passing available readings to the encoder.
* @return A bitmap of the readings which are actually valid.
* @details Retrieve current value of reading for all known readings, and set a flag for each
* reading that is valid. Stop any currently powered on sensors where possible. This function can
* only be called again after wakeup.
*/

uint16_t getreadings::getReadings( struct encoder::readings* readingsPtr )
{
uint16_t retMask = 0;

    readingsPtr->pm2v5 = 0;
    readingsPtr->pm10 = 0;
    readingsPtr->temp = 0;
    readingsPtr->relh = 0;
    readingsPtr->nox = 0;
    readingsPtr->co2 = 0;
    readingsPtr->lat = 0;
    readingsPtr->lon = 0;
    readingsPtr->alt = 0;

    if( getReading( &readingsPtr->pm2v5, encoder::DATA_CONTAINS_PM2V5 ) )
    {
        retMask |= encoder::DATA_CONTAINS_PM2V5;
    }

    if( getReading( &readingsPtr->pm10, encoder::DATA_CONTAINS_PM10 ) )
    {
        retMask |= encoder::DATA_CONTAINS_PM10;
    }

    if( getReading( &readingsPtr->temp, encoder::DATA_CONTAINS_TEMP ) )
    {
        retMask |= encoder::DATA_CONTAINS_TEMP;
    }

    if( getReading( &readingsPtr->relh, encoder::DATA_CONTAINS_RELH ) )
    {
        retMask |= encoder::DATA_CONTAINS_RELH;
    }

    if( getReading( &readingsPtr->nox, encoder::DATA_CONTAINS_NOX ) )
    {
        retMask |= encoder::DATA_CONTAINS_NOX;
    }

    if( getReading( &readingsPtr->co2, encoder::DATA_CONTAINS_CO2 ) )
    {
        retMask |= encoder::DATA_CONTAINS_CO2;
    }

    if( getReading( &readingsPtr->lat, encoder::DATA_CONTAINS_LAT ) )
    {
        retMask |= encoder::DATA_CONTAINS_LAT;
    }

    if( getReading( &readingsPtr->lon, encoder::DATA_CONTAINS_LON ) )
    {
        retMask |= encoder::DATA_CONTAINS_LON;
    }

    if( getReading( &readingsPtr->alt, encoder::DATA_CONTAINS_ALT ) )
    {
        retMask |= encoder::DATA_CONTAINS_ALT;
    }

    // If we can, then stop the sensors, i.e. enter a lower power state. The
    // decision is based on the sampling interval. If the sampling interval is
    // small, and the wakeup time is long, then we cannot afford to stop the sensor,
    // because it will take too long to spin up again before we can do a read.

    // We don't care when, within the sampling interval, the measurement actually
    // happens, only that it can happen before we need to perform another read.

    sensors::sensorSleep();

    // Readings will only be attempted as specified by encoder::datamask.
    // retMask specifies which readings were actually obtained, i.e. this will
    // be a subset of the readings in encoder::datamask.
    return retMask;
}
