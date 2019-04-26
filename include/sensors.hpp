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
    // Identification constants for the various sensors.
    // NB values are expected to be contiguous
    const uint32_t  SENSOR_ID_SDS011 = 0;
    const uint32_t  SENSOR_ID_DHT    = 1;

    // The number of different sensors attached to the system.
    const uint32_t  NUM_SENSORS = 2;

    /*********************************
    * Sensor SDS011
    *
    * Provides PM2.5 and PM10 readings
    **********************************/

    // Settings to use ESP32 Serial2 ( U2UXD ) hardware UART
    const uint32_t  SDS011_SERIAL_BAUD   = 9600;
    const uint32_t  SDS011_SERIAL_FORMAT = SERIAL_8N1;
    const uint32_t  SDS011_SERIAL_RXGPIO = 16;
    const uint32_t  SDS011_SERIAL_TXGPIO = 17;

    const size_t    SDS011_SERIAL_HARDWAREBUFFERSIZE = 32;

    // There is a 'wake up' time, for which the sensor needs to be continuously powered before it is
    // safe to read from it. Time in ms.
    const uint32_t  SDS011_WAKEUP = 15000;

    // There is a maximum time allowed for a read to complete. Time in ms.
    const uint32_t  SDS011_MAX_READ_TIME = 5000;

    // Should we stop this sensor after a reading ?
    constexpr bool  SDS011_STOP_SENSOR = ( sensors::SDS011_WAKEUP + sensors::SDS011_MAX_READ_TIME ) < ( ttnotaa::TX_INTERVAL*1000 );

    // Which readings are given by this sensor ?
    const uint8_t   SDS011_READINGS = encoder::DATA_CONTAINS_PM2V5 | encoder::DATA_CONTAINS_PM10 ;

    // SDS011 Command message constants
    //*********************************

    // Number of commands
    const uint32_t  SDS011_NUM_CMDS              = 4;

    // Command ids
    const uint32_t  SDS011_CMDID_START           = 0;
    const uint32_t  SDS011_CMDID_STOP            = 1;
    const uint32_t  SDS011_CMDID_CONTINUOUSMODE  = 2;
    const uint32_t  SDS011_CMDID_VERSION         = 3;

    // Table of Strings for various commands
	const uint8_t  SDS011_CMD_START[] =
    {
        0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB
    };
	const uint8_t  SDS011_CMD_STOP[]  =
    {
        0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
    };
	const uint8_t  SDS011_CMD_CONTINUOUSMODE[] =
    {
        0xAA, 0xB4, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xAB
    };
	const uint8_t  SDS011_CMD_VERSION[] =
    {
        0xAA, 0xB4, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
    };

    const uint8_t* const SDS011_CMD_STRINGS[SDS011_NUM_CMDS] =  {   SDS011_CMD_START,
                                                                    SDS011_CMD_STOP,
                                                                    SDS011_CMD_CONTINUOUSMODE,
                                                                    SDS011_CMD_VERSION
                                                                };

    // The length of all SDS011 commands is the same
    const uint32_t  SDS011_CMD_SIZE = 19;

    // SDS011 response message constants
    //***********************************

    const int       SDS011_RESP_LEAD_POS    = 0;
    const int       SDS011_RESP_CMD_POS     = 1;
    const int       SDS011_RESP_PM2V5LO_POS = 2;
    const int       SDS011_RESP_PM2V5HI_POS = 3;
    const int       SDS011_RESP_PM10LO_POS  = 4;
    const int       SDS011_RESP_PM10HI_POS  = 5;
    const int       SDS011_RESP_ID0_POS     = 6;
    const int       SDS011_RESP_ID1_POS     = 7;
    const int       SDS011_RESP_CHK_POS     = 8;
    const int       SDS011_RESP_TAIL_POS    = 9;
    const int       SDS011_RESP_SIZE        = 10;

    const uint8_t   SDS011_RESP_LEAD_BYTE   = 0xAA;
    const uint8_t   SDS011_RESP_CMD_BYTE    = 0xC0;
    const uint8_t   SDS011_RESP_TAIL_BYTE   = 0xAB;

    // sensor interface
    //***********************************

    struct  sensorSDS011Readings
    {
        uint32_t    timestamp;
        int         pm2v5;
        int         pm10;
        int         id;
    };

    bool    sensorSDS011Init( void );
    bool    sensorSDS011Checkwait( void );
    int     sensorSDS011ReadByte( void );
    bool    sensorSDS011SkipUntilByte( int target );
    bool    sensorSDS011ValidateMsg( int* msgPtr );
    bool    sensorSDS011GetReadings( void );
    bool    sensorSDS011SendCommand( uint32_t cmd);
    bool    sensorSDS011Read( uint8_t readingMask, int* valuePtr );
    void    sensorSDS011Wakeup( void );

    /*****************************************************
    * Sensor DHT
    *
    * Provides temperature and relative humidity readings.
    ******************************************************/

    // Which readings are given by this sensor ?
    const uint8_t   DHT_READINGS = encoder::DATA_CONTAINS_TEMP | encoder::DATA_CONTAINS_RELH ;

   // sensor interface
   //***********************************
    bool sensorDHTInit( void );
    bool sensorDHTGetReadings( void );
    bool sensorDHTRead( uint8_t readingMask, int* valuePtr );
    void sensorDHTWakeup( void );

    /*****************************************************
    * Sensor Common
    ******************************************************/

    struct sensordescriptor
    {
        uint8_t     id;
        uint8_t     readings;
        bool        needsWakeup;
        uint32_t    wakeupMilliseconds;
        uint32_t    maxReadtime;
        bool        (*triggerFunc)(void);
    };

    const struct sensordescriptor sensorDescriptors[ NUM_SENSORS ] =
    {
        { SENSOR_ID_SDS011, SDS011_READINGS, true, SDS011_WAKEUP, SDS011_MAX_READ_TIME, sensorSDS011GetReadings },
        { SENSOR_ID_DHT, DHT_READINGS, false, 0, 0, sensorDHTGetReadings }
    };

    // non const - can only be worked out at runtime
    extern bool sensorStatus[ NUM_SENSORS ];

   // sensor interface
   //***********************************
    bool        sensorInitSensors( void );
    int         sensorValid( uint8_t readingRequired );
    bool        sensorReading( int sensorId, uint8_t readingMask, int* valuePtr );
    uint32_t    sensorMaxWakeup( void );
    uint32_t    sensorMaxReadtime( void );
    void        sensorWakeup( void );
    bool        sensorTrigger( void );

    template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	       return N; }

}
