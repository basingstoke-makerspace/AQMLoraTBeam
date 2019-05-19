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

 /**
 * @file sensors.cpp
 * @author MdeR
 * @date 26 Apr 2019
 * @copyright 2019 MdeR
 * @brief This file provides an interface for handling sensors.
 */

#include <sensors.hpp>
#include <encoder.hpp>

using namespace sensors;

bool sensors::sensorStatus[ sensors::NUM_SENSORS ];

/**
* @brief Test if a sensor is configured ( i.e. should be initialised )
* @param [in] <name> <parameter_description>
* @return false if sensor not configured OR bad sensor id, else true
* @details Test the sensorconfigured array to determine whether or not
* a given sensor has been configured. The envisaged use of this facility
* is mostly for testing, since sensors cannot be added/removed at runtime.
*/

bool sensors::sensorConfigured( uint8_t sensorId )
{
int i;

   for( i=0; i<sensors::NUM_SENSORS; i++)
   {
       if( sensorId == sensors::sensorPresence[i].sensorId )
       {
           return sensors::sensorPresence[i].present;
       }
   }

   Serial.print(F("Bad sensorId "));
   Serial.println( sensorId );

   return false;
}

/**
* @brief Find the maximum time any sensor needs to take a reading
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

uint32_t sensors::sensorMaxReadtime( void )
{
uint32_t maxVal = 0;
uint32_t val;

    for( int i=0; i<sensors::NUM_SENSORS; i++)
    {
        val = sensors::sensorDescriptors[i].maxReadtime;

        if( maxVal < val )
        {
            maxVal = val;
        }
    }

    return maxVal;
}

/**
* @brief Find the maximum time any sensor needs to wakeup
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

uint32_t sensors::sensorMaxWakeup( void )
{
uint32_t maxVal = 0;
uint32_t val;

    for( int i=0; i<sensors::NUM_SENSORS; i++)
    {
        val = sensors::sensorDescriptors[i].wakeupMilliseconds;

        if( maxVal < val )
        {
            maxVal = val;
        }
    }

    return maxVal;
}

/**
* @brief Answer the question 'what sensor do we use for this reading ?'
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

int sensors::sensorValid( uint16_t readingRequired )
{
int retVal = -1;

    for( int i=0; i<sensors::NUM_SENSORS; i++)
    {
        if( !sensors::sensorConfigured(i) )
        {
            // if the sensor is not configured, we cannot consider it as a
            // potential source of readings.

            Serial.print( F("Sensor not valid because not configured : ") );
            Serial.println( i );

            continue;
        }

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

    if( -1 == retVal )
    {
        // sensor is bad OR we didn't find that reading
        Serial.print(F("Sensor or reading not valid 0x"));
        Serial.println( readingRequired, HEX );
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorInitSensors( void )
{
    Serial.println(F("Initialising sensors"));

    // just make sure we start in a known state
    for( int i=0; i<sensors::NUM_SENSORS; i++ )
    {
        sensors::sensorStatus[i] = false;
    }

    // The idea is to initialise all the sensors that are configured, and use the ones that work.
    if( sensors::sensorConfigured(sensors::SENSOR_ID_SDS011) )
    {
        sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ]  = sensors::sensorSDS011Init();
    }
    if( sensors::sensorConfigured(sensors::SENSOR_ID_DHT) )
    {
        sensors::sensorStatus[ sensors::SENSOR_ID_DHT ]     = sensors::sensorDHTInit();
    }
    if( sensors::sensorConfigured(sensors::SENSOR_ID_NEO6M) )
    {
        sensors::sensorStatus[ sensors::SENSOR_ID_NEO6M ]     = sensors::sensorNEO6MInit();
    }

    // Tell the caller if one of the attempted init's failed - not being configured is not a failure
    return (    ( !sensors::sensorConfigured(sensors::SENSOR_ID_SDS011) ? true : sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ] ) &&
                ( !sensors::sensorConfigured(sensors::SENSOR_ID_DHT) ? true : sensors::sensorStatus[ sensors::SENSOR_ID_DHT ] ) &&
                ( !sensors::sensorConfigured(sensors::SENSOR_ID_NEO6M) ? true : sensors::sensorStatus[ sensors::SENSOR_ID_NEO6M ] ) ) ;
}

/**
* @brief Provide specific reading from the specified sensor.
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorReading( int sensorId, uint16_t readingMask, int* valuePtr )
{
int retVal = false;

    // assume that the combination of sensorId and readingMask is valid

    switch( sensorId )
    {
    case SENSOR_ID_SDS011:
        {
            retVal = sensors::sensorSDS011Read( readingMask, valuePtr );
            break;
        }
    case SENSOR_ID_DHT:
        {
            retVal = sensors::sensorDHTRead( readingMask, valuePtr );
            break;
        }
    case SENSOR_ID_NEO6M:
        {
            retVal = sensors::sensorNEO6MRead( readingMask, valuePtr );
            break;
        }
    default:
        {
            Serial.print(F("Requesting unknown sensor "));
            Serial.println( sensorId );
        }
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorTrigger( void )
{
bool retVal = true;

    // trigger all sensors to provide readings
    for( int i=0; i<sensors::NUM_SENSORS; i++ )
    {
        // Do not trigger sensors which are not configured
        if( sensors::sensorConfigured(i) )
        {
            Serial.print(F("Triggering read from sensor : "));
            Serial.println( i );

            if( !( sensors::sensorDescriptors[i].triggerFunc() ) )
            {
                // failed to trigger a sensor
                Serial.println(F("Sensor failed to trigger"));

                retVal = false;
                break;
            }
        }
        else
        {
            // it's not a trigger failure if the sensor is not configured
            Serial.println(F("Sensor not available to be triggered"));
        }
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorWakeup( void )
{
    sensors::sensorSDS011Wakeup();
    sensors::sensorDHTWakeup();
    sensors::sensorNEO6MWakeup();
}

/**
* @brief Put sensors into low power state if possible
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorSleep( void )
{
    sensors::sensorSDS011Sleep();
    sensors::sensorDHTSleep();
    sensors::sensorNEO6MSleep();
}
