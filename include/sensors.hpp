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

    struct sensordescriptor
    {
        uint8_t     id;
        uint16_t    readings;
        bool        needsWakeup;
        uint32_t    wakeupMilliseconds;
        uint32_t    maxReadtime;
        bool        (*triggerFunc)(void);
    };

    const struct sensordescriptor sensorDescriptors[ NUM_SENSORS ] =
    {
        { SENSOR_ID_SDS011, SDS011_READINGS, true, SDS011_WAKEUP, SDS011_MAX_READ_TIME, sensorSDS011GetReadings },
        { SENSOR_ID_DHT, DHT_READINGS, false, 0, 0, sensorDHTGetReadings },
        { SENSOR_ID_NEO6M, NEO6M_READINGS, false, 0, 0, sensorNEO6MGetReadings }
    };

    // non const - can only be worked out at runtime
    extern bool sensorStatus[ NUM_SENSORS ];

   // sensor interface
   //***********************************
    bool        sensorInitSensors( void );
    int         sensorValid( uint16_t readingRequired );
    bool        sensorReading( int sensorId, uint8_t readingMask, int* valuePtr );
    uint32_t    sensorMaxWakeup( void );
    uint32_t    sensorMaxReadtime( void );
    void        sensorWakeup( void );
    bool        sensorTrigger( void );

    template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	       return N; }

}
