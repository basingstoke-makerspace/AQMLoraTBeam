// NB included in sensors.hpp - namespace sensors will be used

    /*****************************************************
    * Sensor DHT
    *
    * Provides temperature and relative humidity readings.
    ******************************************************/

    // Which readings are given by this sensor ?
    const uint16_t  DHT_READINGS = encoder::DATA_CONTAINS_TEMP | encoder::DATA_CONTAINS_RELH ;

   // sensor interface
   //***********************************
    bool sensorDHTInit( void );
    bool sensorDHTGetReadings( void );
    bool sensorDHTRead( uint8_t readingMask, int* valuePtr );
    void sensorDHTWakeup( void );
