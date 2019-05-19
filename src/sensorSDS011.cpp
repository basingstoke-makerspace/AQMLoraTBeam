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

bool sensors::sensorSDS011Init(void)
{
    // set up the serial port. Use ESP32  hardware UART 2 ( U2UXD ),
    // This port is accessed via Serial2

    assert( SDS011SerialPtr != NULL );

    delay(50);

    SDS011SerialPtr->begin( sensors::SDS011_SERIAL_BAUD,
                            sensors::SDS011_SERIAL_FORMAT,
                            sensors::SDS011_SERIAL_RXGPIO,
                            sensors::SDS011_SERIAL_TXGPIO );

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

bool sensors::sensorSDS011SendCommand( uint32_t cmd)
{
int retVal;

    //Using ESP32 hardware UART U2UXD
    retVal = SDS011SerialPtr->write( sensors::SDS011_CMD_STRINGS[ cmd ], sensors::SDS011_CMD_SIZE);

    Serial.print(F("SDS011 Bytes written :"));
    Serial.println( retVal );

    // if we have given the sensor any command other than 'stop', then it is running
	return cmd != sensors::SDS011_CMDID_SLEEP;
}

/**
* @brief Wait for a message, but only for a certain length of time
* @param [in] None
* @return true if we are still waiting, else false.
* @details Called repeatedly whilst waiting for the SDS011 UART to accumulate
* enough characters in it's rx buffer to be sure we have a complete message.
* When a certain length of time has expired, indicate that we are done with
* waiting.
*/

bool sensors::sensorSDS011Checkwait( void )
{
static unsigned int totalWaitMilliseconds=0;
bool                retVal;

    if( totalWaitMilliseconds > (sensors::SDS011_MAX_READ_TIME) )
    {
        // we can't wait any longer
        Serial.print(F("SDS011 Maximum wait exceeded : "));
        Serial.println( totalWaitMilliseconds );
        totalWaitMilliseconds = 0;
        retVal = false;
    }
    else
    {
        delay(30);
        totalWaitMilliseconds+=30;
        retVal = true;
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
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
            exit(-1);
        }
    }

    return byte;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

bool sensors::sensorSDS011SkipUntilByte( int target )
{
int  byte   = 0xDEADBEEF;
bool retVal = false;

  	while( SDS011SerialPtr->available() > 0 )
    {
        // Throw away bytes until we get the target byte

        byte = sensors::sensorSDS011ReadByte();

        if( byte == target )
        {
            retVal = true;
            break;
        }
        else
        {
            if( !sensors::sensorSDS011Checkwait() )
            {
                // This read has failed
                retVal = false;
                break;
            }
        }
    }

    return retVal;
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
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
* @brief Communicate with the sensor to get new readings
* @param [in] None
* @return true if all readings OK, else false
* @details Read all readings that this sensor is configured
* to supply. Shutdown the sensor afterwards if possible.
*/

bool sensors::sensorSDS011GetReadings( void )
{
int  rxBuffer[2*SDS011_RESP_SIZE + 1];
int  *msgPtr;
int  bytesAvailable = 0xDEADBEEF;
bool retVal         = false;
bool messageRxed    = false;
int  i=0;

    // Remove old data
    SDS011SerialPtr->flush();

    bytesAvailable = SDS011SerialPtr->available();

    Serial.print(F("Bytesavailable : "));
    Serial.println( bytesAvailable );

    // We need at least a whole message worth of bytes to be available -
    // we don't want the tail end of one message and the start of another.
    while( (bytesAvailable = SDS011SerialPtr->available()) < 2*sensors::SDS011_RESP_SIZE )
    {
        if( !sensors::sensorSDS011Checkwait() )
        {
            Serial.print(F("SDS011 bytesAvailable at timeout : "));
            Serial.println( bytesAvailable );

            // This read has failed
            return false;
        }
    }

    // Messages are sent at 1 Hz, and we've received two messages worth of data.
    // Therefore there should be a contiguous message in here somewhere...
    // Read all the bytes that we have.
    for( i=0 ; i<(2*sensors::SDS011_RESP_SIZE) ; i++)
    {
        rxBuffer[i] = sensors::sensorSDS011ReadByte();

        if( 0xDEADBEEF == rxBuffer[i] )
        {
            // our bytes have disappeared, we're doomed...
            Serial.println(F("SDS011 UART buffer problem, exiting"));
            exit(-1);
        }
    }

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
        // Extract the readings from the message
        sensorSDS011LatestReadings.timestamp = os_getTime();

        sensorSDS011LatestReadings.pm2v5 =  ( msgPtr[sensors::SDS011_RESP_PM2V5HI_POS]*256 +
                                              msgPtr[sensors::SDS011_RESP_PM2V5LO_POS] ) / 10;

        sensorSDS011LatestReadings.pm10  =  ( msgPtr[sensors::SDS011_RESP_PM10HI_POS]*256 +
                                              msgPtr[sensors::SDS011_RESP_PM10LO_POS] ) / 10;

        sensorSDS011LatestReadings.id    =    msgPtr[sensors::SDS011_RESP_ID1_POS]*256 +
                                              msgPtr[sensors::SDS011_RESP_ID0_POS] ;

        Serial.print(F(" SDS011 Readings : "));
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

    // If we can, then stop the sensor. The decision is based on the sampling
    // interval. If the sampling interval is small, then we cannot afford to
    // stop the sensor, because it will take too long to spin up again before
    // we can do a read. We don't care when, within the sampling interval, the
    // measurement actually happens, only that it can happen before we need to
    // perform another read.

    if( sensors::SDS011_STOP_SENSOR )
    {
        sensorSDS011SendCommand( sensors::SDS011_CMDID_SLEEP );
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
    Serial.print(F("Waking up SDS011 at : "));
    Serial.println( millis() );

    sensorSDS011SendCommand( sensors::SDS011_CMDID_WORK );
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

void sensors::sensorSDS011Sleep( void )
{
    if( ( sensors::SDS011_WAKEUP + sensors::SDS011_MAX_READ_TIME ) < ( ttnotaa::TX_INTERVAL*1000 ) )
    {
        Serial.println( F("SDS011 Sleeping"));

        sensors::sensorSDS011SendCommand( sensors::SDS011_CMDID_SLEEP );
    }
    // else don't sleep because there would not be enough time to wake up
}
