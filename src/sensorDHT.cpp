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
 * and reading from the DHT temperature and humidity sensor.
 *
 *******************************************************************************/

/**
* @file sensorDHT.cpp
* @author MdeR
* @date 26 Apr 2019
* @copyright 2019 MdeR
* @brief This file defines data and code for the purpose of configuring ( initialising )
* and reading from the DHT temperature and humidity sensor.
*/

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <sensors.hpp>

using namespace sensors;

static DHT *dhtPtr = NULL;

static struct sensorDHTReadings sensorDHTLatestReadings;

/**
* @brief Perform one-time actions required prior to using this sensor
* @param [in] None
* @return unconditionally true
* @details Init for this sensor is mostly done in the DHT constructor,
* where pin assignment and configuration is performed. Because of this,
* avoid creating a DHT instance unless it's actually needed ( i.e.
* init is called ),
*/

bool sensors::sensorDHTInit( void )
{
    dhtPtr = new DHT( DHT_PIN, DHT_TYPE );

    sensors::sensorStatus[ sensors::SENSOR_ID_DHT ] = true;

    Serial.println(F("Sensor DHT Init OK"));

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

bool sensors::sensorDHTRead( uint16_t readingMask, int* valuePtr )
{
bool        retVal = true;

    if( readingMask & encoder::DATA_CONTAINS_TEMP )
    {
        *valuePtr = sensorDHTLatestReadings.temp;
    }
    else
    if( readingMask & encoder::DATA_CONTAINS_RELH )
    {
        *valuePtr = sensorDHTLatestReadings.relHum;
    }
    else
    {
        Serial.println(F("DHT Invalid reading requested"));
        retVal = false;
    }

    return retVal;
}

/**
* @brief Read values of all readings supported by the sensor
* @param [in] None
* @return true if all readings successfully obtained, else false.
* @details NOTE that the read functions are timing critical
* and they <b>disable interrupts</b> to ensure that they can read
* the pin voltage without being disturbed.
*/

bool sensors::sensorDHTGetReadings( void )
{
bool  retVal = false;
float temp;
float relHum;

    // dht.begin() sets up some DHT internal interval measurement variables
    // so that a reading can be taken immediately.
    dhtPtr->begin();

    // read from sensor - INTERRUPTS WILL BE DISABLED
    temp    = dhtPtr->readTemperature();

    if( NAN == temp )
    {
        Serial.println(F("DHT temp read failed"));
    }
    else
    {
        // read from sensor - INTERRUPTS WILL BE DISABLED
        relHum  = dhtPtr->readHumidity();

        if( NAN == relHum )
        {
            Serial.println(F("DHT relHum read failed"));
        }
        else
        {
            // Store tenths of a degree Celcius
            sensorDHTLatestReadings.temp    = static_cast<int>(10.0*temp);

            // Store tenths of a percent humidity
            sensorDHTLatestReadings.relHum  =  static_cast<int>(10.0*relHum);

            retVal = true;
        }

    }

    return retVal;
}

/**
* @brief Perform per-reading actions required prior to using this sensor
* @param [in] None
* @return N/A
* @details No wakeup required for DHT sensor.
*/

void sensors::sensorDHTWakeup( void )
{
    Serial.println("Waking up DHT - nothing to do"); // TBD Null operation
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorDHTSleep( void )
{
    return; // Null operation
}
