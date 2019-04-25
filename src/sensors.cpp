/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Sensors are treated as independent devices. In the case of failure of a
 * sensor, it is considered better to allow the node to continue to report
 * the data which is available, rather than just report nothing at all.
 * Therefore all initialisations will be attempted.
 *
 * Sensors do not always provide just one type of reading. Reading type and
 * sensor type are therefore different, and a mapping is needed between sensor
 * type and reading type.
 *
 * Adding an additional sensor will usually add additional readings, and
 * therefore this will also impact the encoder code.
 */

#include <sensors.hpp>
#include <encoder.hpp>

using namespace sensors;

bool sensors::sensorStatus[ sensors::NUM_SENSORS ];

int sensors::sensorValid( uint8_t readingRequired )
{
int retVal = -1;

    for( int i=0; i<sensors::NUM_SENSORS; i++)
    {
        // have we found the sensor that provides this reading ?
        // readingRequired is a bitmask with one bit set which corresponds
        // to the reading being requested.
        if(  sensors::sensorDescriptors[i].readings & readingRequired   )
        {
            if( sensors::sensorStatus[i] )
            {
                // Sensor is good and the reading can be used
                // NB : we do not establish validity of a particular reading here,
                // only the validity of the sensor

                // return the id of the sensor to use
                retVal = i;
                break;
            }
        }
    }

    // sensor is bad OR we didn't find that reading
    return retVal;
}

bool sensors::initSensors( void )
{
    // just make sure we start in a known state
    for( int i=0; i<sensors::NUM_SENSORS; i++ )
    {
        sensors::sensorStatus[i] = false;
    }

    // The idea is to initialise as many sensors as we can,
    // and use the ones that work.
    sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ]  = sensorSDS011Init();
    sensors::sensorStatus[ sensors::SENSOR_ID_DHT ]     = sensorDHTInit();

    // Tell the caller if something failed
    return (    sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ] &&
                sensors::sensorStatus[ sensors::SENSOR_ID_DHT ] ) ;
}

int sensors::sensorRead( int sensorId, uint8_t readingMask )
{
int retVal = 0xDEADBEEF;

    // assume that the combination of sensorId and readingMask is valid
    // NB allow for the possibility here that we may need to deal with a
    // negative quantity, hence the integer as opposed to uint.

    switch( sensorId )
    {
    case SENSOR_ID_SDS011:
        {
            retVal = sensors::sensorSDS011Read( readingMask );
            break;
        }
    case SENSOR_ID_DHT:
        {
            retVal = sensors::sensorDHTRead( readingMask );
            break;
        }
    }

    return retVal;
}
