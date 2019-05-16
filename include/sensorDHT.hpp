// NB included in sensors.hpp - namespace sensors will be used

    /*****************************************************
    * Sensor DHT
    *
    * Provides temperature and relative humidity readings.
    ******************************************************/

    // Set sensor type required by Adafruit library initialisation
    const uint8_t   DHT_TYPE = DHT11;

    // Set the pin required by Adafruit library initialisation
    const uint8_t   DHT_PIN = 2; //TBD

    // Which readings are given by this sensor ?
    const uint16_t  DHT_READINGS = encoder::DATA_CONTAINS_TEMP | encoder::DATA_CONTAINS_RELH ;

    struct sensorDHTReadings
    {
        int temp;
        int relHum;
    };

   // sensor interface
   //***********************************
    bool sensorDHTInit( void );
    bool sensorDHTGetReadings( void );
    bool sensorDHTRead( uint16_t readingMask, int* valuePtr );
    void sensorDHTWakeup( void );
