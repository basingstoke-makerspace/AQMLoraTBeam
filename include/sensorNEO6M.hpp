// NB included in sensors.hpp - namespace sensors will be used

    /*****************************************************
    * Sensor NEO6M
    *
    * Provides temperature and relative humidity readings.
    ******************************************************/

    // Set buffer size to hold an NMEA sentence.
    const uint32_t  NEO6M_SERIAL_HARDWAREBUFFERSIZE = 82;

    // Timeout for location reading ( milliseconds )
    const uint32_t  NEO6M_LOCATION_TIMEOUT = 5000;

    // Which readings are given by this sensor ?
    const uint16_t  NEO6M_READINGS = encoder::DATA_CONTAINS_LAT | encoder::DATA_CONTAINS_LON | encoder::DATA_CONTAINS_ALT;

    struct sensorNEO6MReadings
    {
        int lat;
        int lon;
        int alt;
    };

    // TinyGPS provides NMEA parsing, but does not deal with ublox protocol.
    // A small number of ublox protocol messages are defined explicitly here,
    // for power management in particular. The default configuration of the
    // NEO6M permits NMEA and ublox protocol to be used on the same port.

    // NB - any general use of ublox commands should use a protocol schema
    // and generator.

    // Command ids
    const uint32_t  NEO6M_CMDID_RXM_PMREQ_BACKUP    = 0;
    const uint32_t  NEO6M_CMDID_CFG_RXM_PSM         = 1;
    const uint32_t  NEO6M_CMDID_CFG_PM2_PSM         = 2;
    const uint32_t  NEO6M_NUM_COMMANDS              = 3;

    const char* const NEO6M_COMMAND_NAME[] = {   "RXM_PMREQ_BACKUP", "CFG_RXM_PSM", "CFG_PM2_PSM" };

    // Byte sequences for commands. Section references are to the document
    // "u-blox 6 Receiver Description", GPS.G6-SW-10018-F.

    // NB CRC IS ADDED AT RUNTIME to avoid hassle if something in
    // one of the messages needs to be altered.

    // 36.3.1 Requests a Power Management task
    // Use to put system into indefinite backup mode ( 'OFF' )
    const uint32_t NEO6M_CMD_RXM_PMREQ_BACKUP_LEN = 12;
	const uint8_t  NEO6M_CMD_RXM_PMREQ_BACKUP[NEO6M_CMD_RXM_PMREQ_BACKUP_LEN] =
    {
        0xB5, 0x62,
        0x02, 0x41,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01
    };

    // 31.20.2 RXM configuration
    // Set the power mode, in this case to 'Power Save Mode' i.e. PSM
    const uint32_t NEO6M_CMD_CFG_RXM_PSM_LEN = 6;
	const uint8_t  NEO6M_CMD_CFG_RXM_PSM[NEO6M_CMD_CFG_RXM_PSM_LEN] =
    {
        0xB5, 0x62,
        0x06, 0x11,
        0x00,
        0x01
    };

    // 31.14.2 Extended Power Management configuration
    // Configure the power mode, in this case configure PSM
    // PSM config flags are all zero apart from Limit Peak Current,
    // which is set to 0b01.
    const uint32_t NEO6M_CMD_CFG_PM2_PSM_LEN = 48;
	const uint8_t  NEO6M_CMD_CFG_PM2_PSM[NEO6M_CMD_CFG_PM2_PSM_LEN] =
    {
        0xB5, 0x62,
        0x06, 0x3B,
        0x01,                           //Message version (set to 1)
        0x00,                           //Reserved
        0x00,                           //Reserved
        0x00,                           //Reserved
        0x00, 0x00, 0x00, 0x80,         //PSM configuration flags
        0x00, 0x00, 0x03, 0xE8,         //Position update period.
        0x00, 0x00, 0x03, 0xE8,         //Acquisition retry period.
        0x00, 0x00, 0x00, 0x00,         //Grid offset relative to GPS start of week
        0x00, 0x00,                     //on time after first successful fix
        0x00, 0x00,                     //minimal search time
        0x00, 0x00,                     //Reserved
        0x00, 0x00,                     //Reserved
        0x00, 0x00, 0x00, 0x00,         //Reserved
        0x00, 0x00, 0x00, 0x00,         //Reserved
        0x00,                           //Reserved
        0x00,                           //Reserved
        0x00, 0x00,                     //Reserved
        0x00, 0x00, 0x00, 0x00          //Reserved
    };

    struct sensorNEO6MCommand
    {
        const uint32_t    length;
        const uint8_t*    command;
    };

    const struct sensorNEO6MCommand NEO6MCommands[NEO6M_NUM_COMMANDS] =
    {
        { NEO6M_CMD_RXM_PMREQ_BACKUP_LEN, NEO6M_CMD_RXM_PMREQ_BACKUP },
        { NEO6M_CMD_CFG_RXM_PSM_LEN, NEO6M_CMD_CFG_RXM_PSM },
        { NEO6M_CMD_CFG_PM2_PSM_LEN, NEO6M_CMD_CFG_PM2_PSM }
    };

    // Convert degrees into the msb's of a uint32_t
    const uint32_t  NEO6M_DEG_TO_INT = 10000000; // ten million

    // sensor interface
    //***********************************
    void sensorNEO6MUBXCRC( const uint8_t* dataPtr, uint32_t size, uint8_t* outputPtr );
    bool sensorNEO6MSendCommand( uint32_t cmd );
    bool sensorNEO6MInit( void );
    bool sensorNEO6MGetReadings( void );
    bool sensorNEO6MRead( uint16_t readingMask, int* valuePtr );
    void sensorNEO6MWakeup( void );
    void sensorNEO6MSleep( void );
