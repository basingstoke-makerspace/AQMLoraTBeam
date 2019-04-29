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

    // Convert degrees into the msb's of a uint32_t
    const uint32_t  NEO6M_DEG_TO_INT = 10000000; // ten million

   // sensor interface
   //***********************************
    bool sensorNEO6MInit( void );
    bool sensorNEO6MGetReadings( void );
    bool sensorNEO6MRead( uint8_t readingMask, int* valuePtr );
    void sensorNEO6MWakeup( void );
