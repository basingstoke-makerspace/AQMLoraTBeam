/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * The basic idea of this namespace is that you change the 'datamask' depending on
 * the set of readings that your available sensors can provide. Then you call 'encode()',
 * specifying the values you actually have. This is all you should need to do,
 * unless you wish to add additional readings. In this case, you need to
 * define the 'UNUSED' values, or potentially even increase the size of datamask,
 * which will have several knock-on effects.
 *
 * Note that 'sensors' and 'readings' are different concepts, because a sensor may
 * provide readings for several different parameters.
 */
#pragma once

#include <cstdint>

namespace encoder
{
//datamask is a bitfield that specifies the readings ( not sensors ) that the code
//expects to be able to provide.
//Bitmask           0x0001 - PM2.5
//                  0x0002 - PM10
//                  0x0004 - Temp
//                  0x0008 - Rel. Humidity
//                  0x0010 - NOx
//                  0x0020 - CO2
//                  0x0040 - Latitude
//                  0x0080 - Longditude
//                  0x0100 - Altitude
const uint16_t DATA_CONTAINS_PM2V5    = 0x0001;
const uint16_t DATA_CONTAINS_PM10     = 0x0002;
const uint16_t DATA_CONTAINS_TEMP     = 0x0004;
const uint16_t DATA_CONTAINS_RELH     = 0x0008;
const uint16_t DATA_CONTAINS_NOX      = 0x0010;
const uint16_t DATA_CONTAINS_CO2      = 0x0020;
const uint16_t DATA_CONTAINS_LAT      = 0x0040;
const uint16_t DATA_CONTAINS_LON      = 0x0080;
const uint16_t DATA_CONTAINS_ALT      = 0x0100;

//These are the ranges of readings that are assumed when
//selecting the bitfield sizes. If you change a range,
//you should verify that the bitfield size is still OK.

//PM 2.5            ( 9 bits,   0 - 512 ug per cubic metre )
//PM 10             ( 9 bits    0 - 512 ug per cubic metre )
//Temp              ( 6 bits   -20 - +43 degrees Celcius )
//Rel. Humidity     ( 7 bits    0 - 100 percent )
//NOx               ( 8 bits    20 - 275 ug per cubic metre )
//CO2               ( 7 bits    370 - 528 ppm )
//Lat               (32 bits    +-180 degrees. 60mins, 60sec)
//Long              ()
//Alt               ()
const int   BITS_FOR_DATAMASK   =  sizeof( DATA_CONTAINS_PM2V5 ) * 8;
const int   BITS_FOR_PM2V5      =  9;
const int   BITS_FOR_PM10       =  9;
const int   BITS_FOR_TEMP       =  6;
const int   BITS_FOR_RELH       =  7;
const int   BITS_FOR_NOX        =  8;
const int   BITS_FOR_CO2        =  7;
const int   BITS_FOR_LAT        =  32;
const int   BITS_FOR_LON        =  32;
const int   BITS_FOR_ALT        =  13;

// Using a value offset helps avoid wasted bits. In effect, it
// bases the transmitted value at zero. This effect has to be
// undone at the receiving end.
// If you change a range ( see above ), you may need to change
// a value offset.
const int   VALUE_OFFSET_FOR_PM2V5      =  0;
const int   VALUE_OFFSET_FOR_PM10       =  0;
const int   VALUE_OFFSET_FOR_TEMP       =  20;
const int   VALUE_OFFSET_FOR_RELH       =  0;
const int   VALUE_OFFSET_FOR_NOX        =  -20;
const int   VALUE_OFFSET_FOR_CO2        =  -370;
const int   VALUE_OFFSET_FOR_LAT        =  0;
const int   VALUE_OFFSET_FOR_LON        =  0;
const int   VALUE_OFFSET_FOR_ALT        =  100;

// Max bits ( for mask 0x3F ), 8+9+9+6+7+8+7, i.e. 54 bits ( 7 bytes )
// For mask 0x0F, size is 8+9+9+6+7, i.e. 39 bits ( 5 bytes )
const int   MAX_BUFFER_BYTES    = ((BITS_FOR_DATAMASK+
                                    BITS_FOR_PM2V5+
                                    BITS_FOR_PM10+
                                    BITS_FOR_TEMP+
                                    BITS_FOR_RELH+
                                    BITS_FOR_NOX+
                                    BITS_FOR_CO2+
                                    BITS_FOR_LAT+
                                    BITS_FOR_LON+
                                    BITS_FOR_ALT)/8)+1;

// Adapt the datamask for your circumstances
/*
const uint16_t datamask =   DATA_CONTAINS_PM2V5 |
                            DATA_CONTAINS_PM10 |
                            DATA_CONTAINS_TEMP |
                            DATA_CONTAINS_RELH |
                            DATA_CONTAINS_LAT |
                            DATA_CONTAINS_LON |
                            DATA_CONTAINS_ALT ;
*/

const uint16_t datamask =   DATA_CONTAINS_PM2V5 |
                            DATA_CONTAINS_PM10 ;

// Metadata for bitfields to be encoded
struct BitfieldDescriptor
{
    uint16_t    datamaskBit;
    int         numberOfBits;
    int         valueOffset;
};

// Required number of bitfield descriptors is bounded by the number of bits
// in the datamask ( but may be less ).
const struct BitfieldDescriptor bitfields[ 8*sizeof(datamask) ] =
{
    {DATA_CONTAINS_PM2V5    ,  BITS_FOR_PM2V5   , VALUE_OFFSET_FOR_PM2V5 },
    {DATA_CONTAINS_PM10     ,  BITS_FOR_PM10    , VALUE_OFFSET_FOR_PM10 },
    {DATA_CONTAINS_TEMP     ,  BITS_FOR_TEMP    , VALUE_OFFSET_FOR_TEMP },
    {DATA_CONTAINS_RELH     ,  BITS_FOR_RELH    , VALUE_OFFSET_FOR_RELH },
    {DATA_CONTAINS_NOX      ,  BITS_FOR_NOX     , VALUE_OFFSET_FOR_NOX },
    {DATA_CONTAINS_CO2      ,  BITS_FOR_CO2     , VALUE_OFFSET_FOR_CO2 },
    {DATA_CONTAINS_LAT      ,  BITS_FOR_LAT     , VALUE_OFFSET_FOR_LAT },
    {DATA_CONTAINS_LON      ,  BITS_FOR_LON     , VALUE_OFFSET_FOR_LON },
    {DATA_CONTAINS_ALT      ,  BITS_FOR_ALT     , VALUE_OFFSET_FOR_ALT }
};

// A convenience struct for passing readings to encode()
struct readings
{
    int pm2v5;
    int pm10;
    int temp;
    int relh;
    int nox;
    int co2;
    int lat;
    int lon;
    int alt;
};

void getBit( int bitPosition, int* byteOffsetPtr, uint8_t* maskPtr );
int  writeBitfield( uint16_t maskBit, int value, int* currentBitPositionPtr, uint8_t *outputBufferPtr, uint16_t validValuesMask);
int  encode( uint8_t* outputBufferPtr, struct encoder::readings* valuesPtr, uint16_t validValuesMask );

}
