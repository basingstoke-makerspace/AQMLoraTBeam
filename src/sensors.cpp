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
 * In cases where an error is encountered that affects one sensor, that
 * sensor is marked as 'not available'. This means that further attempts to
 * read from this sensor should not occur. This is a run-time setting.
 *
 * Sensors do not always provide just one type of reading. Reading type and
 * sensor type are therefore different, and a mapping is needed between sensor
 * type and reading type.
 *
 * Adding an additional sensor will usually add additional readings, and
 * therefore this will also impact the encoder code.
 *
 * Sensors may be 'present' or 'not present'. 'not present' means
 * that the code should behave as if that sensor was not physically present.
 * This is a compile time setting.
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
* @brief Test if a sensor is present ( i.e. should be initialised )
* @param [in] sensorId - identity of the sensor whose presence should be
* tested.
* @return false if sensor not present OR bad sensor id, else true
* @details Test the sensorPresence array to determine whether or not
* a given sensor is present. The envisaged use of this facility
* is mostly for testing, since sensors cannot be added/removed at runtime.
*/

bool sensors::sensorPresent( uint8_t sensorId )
{
int i;

   for( i=0; i<sensors::NUM_SENSORS; i++)
   {
       if( sensorId == sensors::sensorPresence[i].sensorId )
       {
           return sensors::sensorPresence[i].present;
       }
   }

   Serial.print(F("Bad sensorId : "));
   Serial.println( sensorId );

   return false;
}

/**
* @brief Find the maximum time any sensor needs to take a reading
* @param [in] None
* @return The maximum read time given for any single sensor which is
* present.
* @details <details>
*/

uint32_t sensors::sensorMaxReadtime( void )
{
uint32_t maxVal = 0;
uint32_t val;

    for( int i=0; i<sensors::NUM_SENSORS; i++)
    {
        if( sensors::sensorPresent(i) )
        {
            val = sensors::sensorDescriptors[i].maxReadtime;

            if( maxVal < val )
            {
                maxVal = val;
            }
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
        if( sensors::sensorPresent(i) )
        {
            val = sensors::sensorDescriptors[i].wakeupMilliseconds;

            if( maxVal < val )
            {
                maxVal = val;
            }
        }
    }

    return maxVal;
}

/**
* @brief Answer the question 'what sensor do we use for this reading ?'
* @param [in] readingRequired - the reading for which we need a mapping to sensor
* @return -1 if there is a reading/sensor mismatch, otherwise the id of the sensor
* providing this reading.
* @details Search through all the sensors for one that supports this reading. Trap
* the condition where the sensor is not present, but do not assume that the reading
* cannot be provided until all sensors have been tried.
*/

int sensors::sensorValid( uint16_t readingRequired )
{
int retVal = -1;

    for( int i=0; i<sensors::NUM_SENSORS; i++)
    {
        // have we found the sensor that provides this reading ?
        // readingRequired is a bitmask with one bit set which corresponds
        // to the reading being requested.
        if(  sensors::sensorDescriptors[i].readings & readingRequired   )
        {
            if( !sensors::sensorPresent(i) )
            {
                // if the sensor is not present, we cannot consider it as a
                // potential source of readings.

                Serial.print( F("Sensor not valid because not present : ") );
                Serial.println( i );

                continue;
            }
            else if( sensors::sensorStatus[i] )
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
* @brief Initialise the sensors. Handle non-present sensors and onboard sensors.
* @param [in] None
* @return true if no present sensor failed to initialise, else false
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

    // The idea is to initialise all the sensors that are present, and use the ones that work.
    // But must treat on-board sensors as a special case, since they may be 'on' by default, so
    // allow Init to sort this out.
    if( sensors::sensorPresent(sensors::SENSOR_ID_SDS011) )
    {
        sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ]  = sensors::sensorSDS011Init();
    }

    if( sensors::sensorPresent(sensors::SENSOR_ID_DHT) )
    {
        sensors::sensorStatus[ sensors::SENSOR_ID_DHT ]     = sensors::sensorDHTInit();
    }

    // On board sensors may require active disabling.
    sensors::sensorStatus[ sensors::SENSOR_ID_NEO6M ]       = sensors::sensorNEO6MInit();


    // Tell the caller if one of the attempted init's failed - not being present is not a failure
    return (    ( !sensors::sensorPresent(sensors::SENSOR_ID_SDS011) ? true : sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ] ) &&
                ( !sensors::sensorPresent(sensors::SENSOR_ID_DHT) ? true : sensors::sensorStatus[ sensors::SENSOR_ID_DHT ] ) &&
                ( !sensors::sensorPresent(sensors::SENSOR_ID_NEO6M) ? true : sensors::sensorStatus[ sensors::SENSOR_ID_NEO6M ] ) ) ;
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
* @brief Cause all present sensors to read their readings
* @param [in] None
* @return true if all present sensors report success, otherwise false
* @details <details>
*/

bool sensors::sensorTrigger( void )
{
bool retVal = true;

    // trigger all sensors to provide readings
    for( int i=0; i<sensors::NUM_SENSORS; i++ )
    {
        // Do not trigger sensors which are not present
        if( sensors::sensorPresent(i) )
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
            // it's not a trigger failure if the sensor is not present
            Serial.println(F("Sensor not available to be triggered"));
        }
    }

    return retVal;
}

/**
* @brief Attempt to wake up all present sensors
* @param [in] None
* @return None
* @details <details>
*/

void sensors::sensorWakeup( void )
{
    // Only wake up configured sensors
    if( sensors::sensorPresent( sensors::SENSOR_ID_SDS011 ) )
    {
        sensors::sensorSDS011Wakeup();
    }
    else
    {
        Serial.print(F("Sensor not present, not waking : "));
        Serial.println( sensors::SENSOR_ID_SDS011 );
    }
    if( sensors::sensorPresent( sensors::SENSOR_ID_DHT ) )
    {
        sensors::sensorDHTWakeup();
    }
    else
    {
        Serial.print(F("Sensor not present, not waking : "));
        Serial.println( sensors::SENSOR_ID_DHT );
    }
    if( sensors::sensorPresent( sensors::SENSOR_ID_NEO6M ) )
    {
        sensors::sensorNEO6MWakeup();
    }
    else
    {
        Serial.print(F("Sensor not present, not waking : "));
        Serial.println( sensors::SENSOR_ID_NEO6M );
    }
}

/**
* @brief Put sensors into low power state if possible
* @param [in] None
* @return None
* @details <details>
*/

void sensors::sensorSleep( void )
{
    // Only sleep configured sensors
    if( sensors::sensorPresent( sensors::SENSOR_ID_SDS011 ) )
    {
        sensors::sensorSDS011Sleep();
    }
    else
    {
        Serial.print(F("Sensor not present, not sleeping : "));
        Serial.println( sensors::SENSOR_ID_SDS011 );
    }
    if( sensors::sensorPresent( sensors::SENSOR_ID_DHT ) )
    {
        sensors::sensorDHTSleep();
    }
    else
    {
        Serial.print(F("Sensor not present, not sleeping : "));
        Serial.println( sensors::SENSOR_ID_DHT );
    }
    if( sensors::sensorPresent( sensors::SENSOR_ID_NEO6M ) )
    {
        sensors::sensorNEO6MSleep();
    }
    else
    {
        Serial.print(F("Sensor not present, not sleeping : "));
        Serial.println( sensors::SENSOR_ID_NEO6M );
    }
}
