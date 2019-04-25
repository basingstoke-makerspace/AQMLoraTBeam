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

#include <getreadings.hpp>
#include <sensors.hpp>

using namespace sensors;

extern const unsigned TX_INTERVAL;

bool sensors::sensorSDS011Init(void)
{
    // set up the serial port. Use ESP32  hardware UART 2 ( U2UXD ), TX/GPIO17, RX/GPI16
    // This port is accessed via Serial2
    Serial2.begin(sensors::SDS011_SERIAL_BAUD, sensors::SDS011_SERIAL_FORMAT, sensors::SDS011_SERIAL_RXGPIO, sensors::SDS011_SERIAL_TXGPIO);

    Serial2.setRxBufferSize( sensors::SDS011_SERIAL_HARDWAREBUFFERSIZE );

    sensors::sensorStatus[ sensors::SENSOR_ID_SDS011 ] = true;

    return true;
}

bool sensors::sensorSDS011SendCommand( uint32_t cmd)
{
    //Serial2 is ESP32 hardware UART U2UXD
    Serial2.write( sensors::SDS011_CMD_STRINGS[ cmd ], sensors::SDS011_CMD_SIZE);

    // if we have given the sensor any command other than 'stop', then it is running
	return cmd != sensors::SDS011_CMDID_STOP;
}

int sensors::sensorSDS011GetMsg( uint8_t* bufferPtr )
{
int bytesAvailable = 0;

	while ( ( bytesAvailable = Serial2.available() ) > 0)
    {
        // Read from the UART until the message is complete

        // Throw away bytes until we get the Message Start

        int rxByte = Serial2.read();

        if( -1 == rxByte )
        {
            // uart error


        }
        else
        {

        }


    }

}

int sensors::sensorSDS011Read( uint8_t readingMask )
{
int         bytesAvailable = 0;
int         rxByte = 0;
int         retVal = 0xDEADBEEF;
uint8_t     rxBuffer[SDS011_RESP_SIZE];
const int   check = 0;

    retVal = sensors::sensorSDS011GetMsg( rxBuffer );

    // Extract readings from buffer



    // If we can, then stop the sensor. The decision is based on the sampling
    // interval. If the sampling interval is small, then we cannot afford to
    // stop the sensor, because it will take too long to spin up again before
    // we can do a read. We don't care when, within the sampling interval, the
    // measurement actually happens, only that it can happen before we need to
    // perform another read.

    if( ( sensors::SDS011_WARMUP + sensors::SDS011_MAX_READ_TIME ) < ( getreadings::TX_INTERVAL*1000 ) )
    {
        sensorSDS011SendCommand( sensors::SDS011_CMDID_STOP );
    }

    // If TX_INTERVAL is small, we will use considerably more power because the
    // sensor will be permanently 'on'.

    return retVal;
}

#if 0
/*****************************************************************
 * read SDS011 sensor values                                     *
 *****************************************************************/
String sensors::sensorSDS011Read( void )
{
	String s = "";
	char buffer;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is = 0;
	int checksum_ok = 0;

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_SDS011), DEBUG_MED_INFO, 1);

    // is it too soon after startup to send anything ? Not obvious why we must wait for sending_intervall as
    // opposed to warm up time
	if (msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
    {
		if (is_SDS_running)
        {
			is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		}
	}
    else
    {
		if (! is_SDS_running)
        {
			is_SDS_running = SDS_cmd(PmSensorCmd::Start);
		}

		while (Serial2.available() > 0)
        {
			buffer = Serial2.read();
			debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO, 1);
//			"aa" = 170, "ab" = 171, "c0" = 192
			value = int(buffer);
			switch (len)
            {
			case (0):
				if (value != 170)
                {
					len = -1;
				};
				break;
			case (1):
				if (value != 192)
                {
					len = -1;
				};
				break;
			case (2):
				pm25_serial = value;
				checksum_is = value;
				break;
			case (3):
				pm25_serial += (value << 8);
				break;
			case (4):
				pm10_serial = value;
				break;
			case (5):
				pm10_serial += (value << 8);
				break;
			case (8):
				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), DEBUG_MED_INFO, 0);
				debug_out(String(checksum_is % 256), DEBUG_MED_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), DEBUG_MED_INFO, 0);
				debug_out(String(value), DEBUG_MED_INFO, 1);
				if (value == (checksum_is % 256))
                {
					checksum_ok = 1;
				}
                else
                {
					len = -1;
				};
				break;
			case (9):
				if (value != 171)
                {
					len = -1;
				};
				break;
			}

			if (len > 2) { checksum_is += value; }
			len++;
			if (len == 10 && checksum_ok == 1 && (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS)))
            {
				if ((! isnan(pm10_serial)) && (! isnan(pm25_serial)))
                {

				}
				len = 0;
				checksum_ok = 0;
				pm10_serial = 0.0;
				pm25_serial = 0.0;
				checksum_is = 0;
			}
			yield();
		} // while

	}
	if (send_now)
    {
		last_value_SDS_P1 = -1;
		last_value_SDS_P2 = -1;
		if (sds_val_count > 2)
        {
			sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
			sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
			sds_val_count = sds_val_count - 2;
		}
		if (sds_val_count > 0)
        {
			last_value_SDS_P1 = double(sds_pm10_sum) / (sds_val_count * 10.0);
			last_value_SDS_P2 = double(sds_pm25_sum) / (sds_val_count * 10.0);
			debug_out("PM10:  " + Float2String(last_value_SDS_P1), DEBUG_MIN_INFO, 1);
			debug_out("PM2.5: " + Float2String(last_value_SDS_P2), DEBUG_MIN_INFO, 1);
			debug_out("----", DEBUG_MIN_INFO, 1);
			s += Value2Json("SDS_P1", Float2String(last_value_SDS_P1));
			s += Value2Json("SDS_P2", Float2String(last_value_SDS_P2));
		}
		sds_pm10_sum = 0;
		sds_pm25_sum = 0;
		sds_val_count = 0;
		sds_pm10_max = 0;
		sds_pm10_min = 20000;
		sds_pm25_max = 0;
		sds_pm25_min = 20000;

		if ((cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
        {
			is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		}
	}

	debug_out(String(FPSTR(DBG_TXT_END_READING)) + FPSTR(SENSORS_SDS011), DEBUG_MED_INFO, 1);

	return s;
}

}
#endif //0
