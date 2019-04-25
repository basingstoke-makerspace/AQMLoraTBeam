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

#include <encoder.hpp>
#include <getreadings.hpp>
#include <sensors.hpp>

extern  const unsigned TX_INTERVAL;

using namespace getreadings;

bool getreadings::getReading( int* valuePtr, uint8_t readingMask )
{
bool    retVal;
int     sensorId;

    // is this a reading that we expect to return ?
    if( encoder::datamask & readingMask )
    {
        // do we have a working sensor that can provide this reading ?
        if( ( sensorId = sensors::sensorValid(readingMask) ) > -1 )
        {
            // get reading from the sensor

            // *******************************************************************************
            // *** NB this works only so long as a reading can be represented as an int :) ***
            // *******************************************************************************

            *valuePtr = sensors::sensorRead( sensorId, readingMask );

            if( *valuePtr == 0xDEADBEEF )
            {
                // uart error
                retVal = false;
            }
            else
            {
                retVal = true;
            }
        }
        else
        {
            // We are expecting to be able to return a value, but the
            // sensor has failed in some way.
            retVal = false;
        }
    }
    else
    {
        // not configured to use this reading
        retVal = false;
    }

    return retVal;
}

uint8_t getreadings::getReadings( struct encoder::readings* readingsPtr )
{
uint8_t retMask = 0;

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

    // If we can, then stop the sensors, i.e. enter a lower power state. The
    // decision is based on the sampling interval. If the sampling interval is
    // small, then we cannot afford to stop the sensor, because it will take too
    // long to spin up again before we can do a read.

    // We don't care when, within the sampling interval, the measurement actually
    // happens, only that it can happen before we need to perform another read.

    if( ( sensors::SDS011_WARMUP + sensors::SDS011_MAX_READ_TIME ) < ( TX_INTERVAL*1000 ) )
    {
        sensors::sensorSDS011SendCommand( sensors::SDS011_CMDID_STOP );
    }

    // Readings will only be attempted as specified by encoder::datamask.
    // retMask specifies which readings were actually obtained, i.e. this will
    // be a subset of the readings in encoder::datamask.
    return retMask;
}
