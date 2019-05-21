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
 * and reading from the SDS011 particle sensor. This provides PM2.5 and PM10
 * readings.
 *
 *******************************************************************************/

/**
* @file sensorSDS011.cpp
* @author MdeR
* @date 26 Apr 2019
* @copyright 2019 MdeR
* @brief This file defines data and code for the purpose of configuring ( initialising )
* and reading from the SDS011 particle sensor.
*/

#include <stdlib.h>

#include <lmic.h>
#include <hal/hal.h>

#include <getreadings.hpp>
#include <sensors.hpp>
#include <ttnotaa.hpp>

using namespace sensors;

// Cache for sensor readings. Read by encoder.
static struct sensors::sensorSDS011Readings sensorSDS011LatestReadings;

static HardwareSerial* SDS011SerialPtr = &Serial2;

/**
* @brief Set up an ESP32 hardware serial port for SDS011 comms
* @param [in] None
* @return true
* @details Obtain a hardwareserial instance, configure the comms
* parameters, map the GPIO pins to the UART, configure the UART
* buffer.
*/

bool sensors::sensorSDS011Init( void )
{
    // set up the serial port. Use ESP32  hardware UART 2 ( U2UXD ),
    // This port is accessed via Serial2

    assert( SDS011SerialPtr != NULL );

    // begin() invokes end(), which invokes uartEnd(), which forces
    // the Freertos queue for this uart to be recreated.

    // begin() invokes flush(), which handles tx/rx fifo's. Note that this
    // call does empty the rx fifo and the tx fifo, although it doesn't do
    // it very nicely ( it deals with them character by character ). The
    // Arduino documentation is misleading, because it suggests it does not
    // deal with the rx fifo.

    SDS011SerialPtr->begin( sensors::SDS011_SERIAL_BAUD,
                            sensors::SDS011_SERIAL_FORMAT,
                            sensors::SDS011_SERIAL_RXGPIO,
                            sensors::SDS011_SERIAL_TXGPIO );

    Serial.print(F("SDS011 post flush bytes available : "));
    Serial.println( SDS011SerialPtr->available() );

    sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ] = true;

    Serial.println(F("Sensor SDS011 Init OK"));

    return true;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorSDS011SendCommand( uint32_t cmd )
{
int retVal;

    //Using ESP32 hardware UART U2UXD
    retVal = SDS011SerialPtr->write( sensors::SDS011_CMD_STRINGS[ cmd ], sensors::SDS011_CMD_SIZE);

    if( retVal != sensors::SDS011_CMD_SIZE )
    {
        Serial.print(F("SDS011 Command NOT sent : "));
        Serial.print( SDS011_COMMAND_NAME[cmd] );
        Serial.print( " " );
        Serial.println( retVal );

        return false;
    }
    else
    {
        Serial.print(F("SDS011 Command sent : "));
        Serial.println( SDS011_COMMAND_NAME[cmd] );

        return true;
    }
}

/**
* @brief Read a byte from the SDS011 serial queue/buffer
* @param [in] None
* @return 0xDEADBEEF if byte not read, else 0x00-0xFF
* @details Exits if UART error, otherwise returns value
*/

int sensors::sensorSDS011ReadByte( void )
{
int byte=0xDEADBEEF;

    if( SDS011SerialPtr->available() > 0 )
    {
        byte = SDS011SerialPtr->read();

        if( -1 == byte )
        {
            // we're doomed....
            Serial.println(F("SDS011 UART Error 3"));
            exit( EXIT_CODE_SDS011_UARTERROR );
        }
    }

    return byte;
}

/**
* @brief Check checksum for received SDS011 message
* @param [in] msgPtr ( pointer to ) buffer containing SDS011 message
* @return true if checksum OK, else false
* @details <details>
*/

bool sensors::sensorSDS011ValidateMsg( int* msgPtr )
{
bool    retVal = false;
int     chksum = 0;
int     msgChk = 0;

    chksum = msgPtr[sensors::SDS011_RESP_CHK_POS];

    for( int i=sensors::SDS011_RESP_PM2V5LO_POS; i<sensors::SDS011_RESP_CHK_POS; i++ )
    {
        msgChk+=msgPtr[i];
    }

    if( chksum == (msgChk & 0x000000FF) )
    {
        retVal = true;
    }
    else
    {
        Serial.println(F("SDS011 Checksum Invalid"));
    }

    return retVal;
}

/**
* @brief Wait until a whole SDS011 message is available
* @param [in] None
* @return true if buffer filled OK, else false
* @details To keep parsing quick and simple, we just want
* to deal with a buffer that we know contains a whole
* message somewhere. Trade off some time spent receiving
* in order to achieve this.
*/

bool sensors::sensorSDS011WaitForMessage( void )
{
int     start=0;
int     bytesAvailable = 0xDEADBEEF;

    start = millis();
    while( (bytesAvailable = SDS011SerialPtr->available()) < 2*sensors::SDS011_RESP_SIZE )
    {
        if( millis()-start > sensors::SDS011_MAX_READ_TIME )
        {
            Serial.print(F("SDS011 bytesAvailable at timeout : "));
            Serial.println( bytesAvailable );

            // This read has failed
            return false;
        }

        // SDS011_MAX_READ_TIME is at least 2s, prevent blocking behaviour
        delay(SDS011_CHECKWAIT_DELAY);
    }

    return true;
}

/**
* @brief Read a message from serial port.
* @param [in] rxBufferPtr - ( pointer to ) buffer for received bytes
* @return None
* @details This function is called when we believe there is a whole
* message in the serial port queue, but we do not yet know where in
* the queue it begins. Read enough bytes from the queue to be sure
* we have put the expected message in the buffer.
*/

void sensors::sensorSDS011GetMsgBytes( int* rxBufferPtr )
{
int i=0;

    for( i=0 ; i<(2*sensors::SDS011_RESP_SIZE) ; i++)
    {
        rxBufferPtr[i] = sensors::sensorSDS011ReadByte();

        if( 0xDEADBEEF == rxBufferPtr[i] )
        {
            // our bytes have disappeared, we're doomed...
            Serial.println(F("SDS011 UART buffer problem, exiting"));
            exit( EXIT_CODE_SDS011_UARTBUFFER );
        }
    }
}

/**
* @brief Communicate with the sensor to get new readings
* @param [in] None
* @return true if all readings OK, else false
* @details Read all readings that this sensor is configured
* to supply. Shutdown the sensor afterwards if possible.
*/

bool sensors::sensorSDS011GetReadings( void )
{
int     rxBuffer[2*SDS011_RESP_SIZE + 1];
int*    msgPtr = NULL;
bool    retVal         = false;
bool    messageRxed    = false;
int     i=0;

    // Because the Arduino/ESP32 uart handling offers no better way of doing this,
    // ditch old queued characters ( there's a queue on top of the rxfifo...) by
    // reading them, and flush the rx/tx buffers. We don't want old data here.
    // The flush() call flushes the rx buffer ( crudely, by repeated reads )
    // despite the Arduino documentation only referring to the tx buffer ( and
    // then only to say that it doesn't actually flush it ).

    while( SDS011SerialPtr->available() > 0 ) SDS011SerialPtr->read();
    SDS011SerialPtr->flush();

    // We need at least a whole message worth of bytes to be available -
    // we don't want just the tail end of one message and the start of another.

    if( !sensors::sensorSDS011WaitForMessage() )
    {
        return false;
    }

    // Messages are sent at 1 Hz, and we've received two messages worth of data.
    // Therefore there should be a contiguous message in here somewhere...
    // Read all the bytes that we have.

    sensors::sensorSDS011GetMsgBytes( rxBuffer );

    // find the message - it must start in the first half of the buffer
    i=0;
    while( i<sensors::SDS011_RESP_SIZE )
    {
        if( sensors::SDS011_RESP_LEAD_BYTE  == rxBuffer[i] &&
            sensors::SDS011_RESP_CMD_BYTE   == rxBuffer[i+sensors::SDS011_RESP_CMD_POS] &&
            sensors::SDS011_RESP_TAIL_BYTE  == rxBuffer[i+sensors::SDS011_RESP_TAIL_POS])
        {
            msgPtr = &rxBuffer[i];

            messageRxed = sensors::sensorSDS011ValidateMsg( msgPtr );

            break;
        }

        i++;
    }

    if( messageRxed )
    {
        Serial.print(F("SDS011 Message found"));

        // Extract the readings from the message
        sensorSDS011LatestReadings.timestamp = os_getTime();

        sensorSDS011LatestReadings.pm2v5 =  ( msgPtr[sensors::SDS011_RESP_PM2V5HI_POS]*256 +
                                              msgPtr[sensors::SDS011_RESP_PM2V5LO_POS] ) / 10;

        sensorSDS011LatestReadings.pm10  =  ( msgPtr[sensors::SDS011_RESP_PM10HI_POS]*256 +
                                              msgPtr[sensors::SDS011_RESP_PM10LO_POS] ) / 10;

        sensorSDS011LatestReadings.id    =    msgPtr[sensors::SDS011_RESP_ID1_POS]*256 +
                                              msgPtr[sensors::SDS011_RESP_ID0_POS] ;

        Serial.print(F("SDS011 Readings : "));
        Serial.print( sensorSDS011LatestReadings.pm2v5 );
        Serial.print(F(", "));
        Serial.print( sensorSDS011LatestReadings.pm10 );
        Serial.print(F(", "));
        Serial.println( sensorSDS011LatestReadings.id );

        retVal = true;

    }
    else
    {
        // there is no valid reading any more
        sensorSDS011LatestReadings.timestamp    = 0;
        sensorSDS011LatestReadings.pm2v5        = 0xDEADBEEF;
        sensorSDS011LatestReadings.pm10         = 0xDEADBEEF;
        sensorSDS011LatestReadings.id           = 0xDEADBEEF;

        Serial.println(F("SDS011 Failed to find message"));
    }

    return retVal;
}

/**
* @brief Return the current value of the specified reading
* @param [in]   readingMask - set bit specifies reading required
*        [out]  valuePtr    - pointer to var to hold reading
* @return true if reading was found, else false
* @details Get the reading value requested from the private
* data store which was filled by the GetReadings function.
*/

bool sensors::sensorSDS011Read( uint16_t readingMask, int* valuePtr )
{
bool        retVal = true;

    if( readingMask & encoder::DATA_CONTAINS_PM2V5 )
    {
        *valuePtr = sensorSDS011LatestReadings.pm2v5;
    }
    else
    if( readingMask & encoder::DATA_CONTAINS_PM10 )
    {
        *valuePtr = sensorSDS011LatestReadings.pm10;
    }
    else
    {
        Serial.println(F("SDS011 Invalid reading requested"));
        retVal = false;
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorSDS011Wakeup( void )
{
    Serial.print(F("SDS011 waking up at : "));
    Serial.println( millis() );

    if( !(sensorSDS011SendCommand( sensors::SDS011_CMDID_WORK ) ) )
    {
        // system not in a good state, uart/sensor state unknown
        Serial.println( F("Sensor comms failed, exiting"));
        exit( EXIT_CODE_SDS011_SENSORCOMMSFAILED );
    }
}

/**
* @brief Send the command which causes the sensor to sleep, if possible.
* @param [in] None
* @return None
* @details If we can, then stop the sensor. The decision is based on the sampling
* interval. If the sampling interval is small, then we cannot afford to
* stop the sensor, because it will take too long to spin up again before
* we can do a read. We don't care when, within the sampling interval, the
* measurement actually happens, only that it can happen before we need to
* perform another read.
*/

void sensors::sensorSDS011Sleep( void )
{
    
    if( ( sensors::SDS011_WAKEUP + sensors::SDS011_MAX_READ_TIME ) < ( ttnotaa::TX_INTERVAL*1000 ) )
    {
        Serial.println( F("SDS011 Sleeping"));

        if( !(sensorSDS011SendCommand( sensors::SDS011_CMDID_SLEEP ) ) )
        {
            // system not in a good state, uart/sensor state unknown
            Serial.println( F("Sensor comms failed, exiting"));
            exit( EXIT_CODE_SDS011_SENSORCOMMSFAILED );
        }
    }

    // else don't sleep because there would not be enough time to wake up
}
