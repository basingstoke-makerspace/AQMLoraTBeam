// NB included in sensors.hpp - namespace sensors will be used

    /*********************************
    * Sensor SDS011
    *
    * Provides PM2.5 and PM10 readings
    **********************************/

    // Settings to use ESP32 Serial2 ( U2UXD ) hardware UART

    const uint32_t  SDS011_SERIAL_UART = 2;

    const uint32_t  SDS011_SERIAL_BAUD   = 9600;
    const uint32_t  SDS011_SERIAL_FORMAT = SERIAL_8N1;
    const uint32_t  SDS011_SERIAL_RXGPIO = 4;
    const uint32_t  SDS011_SERIAL_TXGPIO = 13;

    const size_t    SDS011_SERIAL_HARDWAREBUFFERSIZE = 32;

    // There is a 'wake up' time, for which the sensor needs to be continuously powered before it is
    // safe to read from it. Time in ms.
    const uint32_t  SDS011_WAKEUP = 20000;

    // There is a maximum time allowed for a read to complete. Time in ms.
    const uint32_t  SDS011_MAX_READ_TIME = 5000;

    // Should we stop this sensor after a reading ?
    constexpr bool  SDS011_STOP_SENSOR = ( sensors::SDS011_WAKEUP + sensors::SDS011_MAX_READ_TIME ) < ( ttnotaa::TX_INTERVAL*1000 );

    // Which readings are given by this sensor ?
    const uint16_t  SDS011_READINGS = encoder::DATA_CONTAINS_PM2V5 | encoder::DATA_CONTAINS_PM10 ;

    // SDS011 Command message constants
    //*********************************

    // Number of commands
    const uint32_t  SDS011_NUM_CMDS              = 4;

    // Command ids
    const uint32_t  SDS011_CMDID_WORK                       = 0;
    const uint32_t  SDS011_CMDID_SLEEP                      = 1;
    const uint32_t  SDS011_CMDID_WORKINGPERIOD_CONTINUOUS   = 2;
    const uint32_t  SDS011_CMDID_FIRMWAREVERSION            = 3;

    // Byte sequences for commands
	const uint8_t  SDS011_CMD_WORK[] =
    {
        0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB
    };
	const uint8_t  SDS011_CMD_SLEEP[]  =
    {
        0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
    };
	const uint8_t  SDS011_CMD_WORKINGPERIOD_CONTINUOUS[] =
    {
        0xAA, 0xB4, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xAB
    };
	const uint8_t  SDS011_CMD_FIRMWAREVERSION[] =
    {
        0xAA, 0xB4, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
    };

    const uint8_t* const SDS011_CMD_STRINGS[SDS011_NUM_CMDS] =  {   SDS011_CMD_WORK,
                                                                    SDS011_CMD_SLEEP,
                                                                    SDS011_CMD_WORKINGPERIOD_CONTINUOUS,
                                                                    SDS011_CMD_FIRMWAREVERSION
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
    bool    sensorSDS011Read( uint16_t readingMask, int* valuePtr );
    void    sensorSDS011Wakeup( void );
    void    sensorSDS011Sleep( void );
