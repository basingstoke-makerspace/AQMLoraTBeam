/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * The sensors namespace contains a description of each sensor. This may include
 * sensor specific configuration, but must include a sensor Id and a corresponding
 * sensor descriptor, which maps a sensor Id to a bit map of the readings which
 * that sensor supports.
 *
 * The sensors namespace also includes general sensor functions for sensor initialisation,
 * and retrieval of current sensor validity ( status ). Sensor validity is used to infer
 * reading validity, i.e. we cannot obtain or use a reading from an invalid sensor.
 */
#pragma once

#include <cstdint>
#include <HardwareSerial.h>
#include <DHT.h>

#include <encoder.hpp>
#include <ttnotaa.hpp>

// The sensors namespace is shared by all sensors

namespace sensors
{
#include <sensorSDS011.hpp>
#include <sensorDHT.hpp>
#include <sensorNEO6M.hpp>

    /*****************************************************
    * Sensor Common
    ******************************************************/

    // Identification constants for the various sensors.
    // NB values are expected to be contiguous
    const uint32_t  SENSOR_ID_SDS011 = 0;
    const uint32_t  SENSOR_ID_DHT    = 1;
    const uint32_t  SENSOR_ID_NEO6M  = 2;

    // The number of different sensors attached to the system.
    const uint32_t  NUM_SENSORS = 3;

    // collect information about a sensor in one place
    struct sensordescriptor
    {
        uint8_t     id;                     // unique identity of the sensor
        uint16_t    readings;               // mask specifying readings provided by the sensor
        bool        needsWakeup;            // does the sensor need to be woken up before reading from it ?
        uint32_t    wakeupMilliseconds;     // if there is a wakeup, how long does it take ?
        uint32_t    maxReadtime;            // how long to allow for a reading ?
        bool        (*triggerFunc)(void);   // the function to perform the reading(s);
        bool        needsShutdown;          // does the sensor need to be explicitly shut down ?
    };

    const struct sensordescriptor sensorDescriptors[ NUM_SENSORS ] =
    {
        { SENSOR_ID_SDS011, SDS011_READINGS, true, SDS011_WAKEUP, SDS011_MAX_READ_TIME, sensorSDS011GetReadings, false },
        { SENSOR_ID_DHT, DHT_READINGS, false, 0, 0, sensorDHTGetReadings, false },
        { SENSOR_ID_NEO6M, NEO6M_READINGS, false, 0, 0, sensorNEO6MGetReadings, false }
    };

    // If you want to *not* use a sensor, change the relevant entry to 'false'. This affects all readings
    // that would be supplied by this sensor.
    struct sensorpresence
    {
        uint8_t sensorId;
        bool    present;
    };

    const struct sensorpresence sensorPresence[ NUM_SENSORS ] =
    {
        { SENSOR_ID_SDS011, false }, // FALSE - MdeR 15/05/19 debug
        { SENSOR_ID_DHT,    false }, // FALSE - MdeR 01/05/19 debug
        { SENSOR_ID_NEO6M,  false }, // FALSE - MdeR 15/05/19 debug
    };

    // non const - can only be worked out at runtime
    extern bool sensorStatus[ NUM_SENSORS ];

   // sensor interface
   //***********************************
    bool        sensorConfigured( uint8_t sensorId );
    bool        sensorInitSensors( void );
    int         sensorValid( uint16_t readingRequired );
    bool        sensorReading( int sensorId, uint16_t readingMask, int* valuePtr );
    uint32_t    sensorMaxWakeup( void );
    uint32_t    sensorMaxReadtime( void );
    void        sensorWakeup( void );
    bool        sensorTrigger( void );

    template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	       return N; }

}
