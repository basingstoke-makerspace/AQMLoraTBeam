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

bool sensors::sensorDHTInit( void )
{
    sensors::sensorStatus[ sensors::SENSOR_ID_DHT ] = true;

    return true;
}

int sensors::sensorDHTRead( uint8_t readingMask )
{
    return 0;
}
