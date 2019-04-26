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

#include <sensors.hpp>

using namespace sensors;

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorDHTInit( void )
{
    sensors::sensorStatus[ sensors::SENSOR_ID_DHT ] = true;

    return true;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorDHTRead( uint8_t readingMask, int* valuePtr )
{
    return true;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorDHTWakeup( void )
{
    Serial.println("Waking up DHT");
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorDHTGetReadings( void )
{
    return true;
}
