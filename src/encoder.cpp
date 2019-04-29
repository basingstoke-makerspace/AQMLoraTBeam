/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This file defines data and code for the purpose of creating a bitfield
 * representation of a set of sensor readings in a provided buffer. The assumption
 * is that this buffer will be transmitted by a LoraWAN node. The constant
 * MAX_BUFFER_BYTES is guaranteed to be sufficient to hold all available fields,
 * but depending on the datamask this may be larger than strictly required.
 *
 *******************************************************************************/

#include <cstdint>
#include <HardwareSerial.h>

#include <encoder.hpp>

using namespace encoder;

/**
* @brief Get the byte and bitmask for a particular bit in buffer.
* @param [in] bitPosition - the bit position ( 0 offset ) for which a byte and mask are required.
* @return N/A
* @details The model used here is that bits in a buffer are used 'MS first'.
* In other words, for two adjacent bytes in the buffer, bit 7 of
* byte N+1 is written after bit 0 of byte N, as opposed to bit 0 of byte
* N+1 being written after bit 7 of byte N. This is in order to
* minimise the number of bits that must be transmitted. However, it's
* not entirely clear if Lora actually deals in bits when sending.
*/

void encoder::getBit( int bitPosition, int* byteOffsetPtr, uint8_t* maskPtr )
{
    *byteOffsetPtr = bitPosition/8;
    *maskPtr = 0xF0 >> (bitPosition % 8);
}

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

int encoder::writeBitfield( uint16_t maskBit, int value, int* currentBitPositionPtr, uint8_t* outputBufferPtr, uint16_t validValuesMask)
{
int retVal;

    // no point in looking for descriptor if we don't have this data
    // NB validValuesMask is assumed to be the subset of encoder::datamask readings for which
    // the corresponding sensor has actually provided values, so we do not test encoder::datamask here.
    if( validValuesMask & maskBit )
    {
    int i;
    int n;
    int byteOffset = 0;
    uint8_t byteMask = 0;

        // find the relevant bitfield descriptor
        for( i=0; encoder::bitfields[i].datamaskBit != maskBit; i++ );

        // first apply the value offset
        value += encoder::bitfields[i].valueOffset;

        // write the specified number of bits in the direction MSB->LSB.
        // assume that currentBitPosition is position of first bit to write.

        // turn the current bit position into a byte offset and mask
        getBit( *currentBitPositionPtr, &byteOffset, &byteMask );

        // We want to write the value MS bit first
        for( n=encoder::bitfields[i].numberOfBits; n>0; n-- )
        {
        int valueMask = 1;

            valueMask <<= (n-1) ;

            if( value & valueMask )
            {
                outputBufferPtr[byteOffset] |= byteMask;
            }
            else
            {
                outputBufferPtr[byteOffset] &= ~byteMask;
            }

            *currentBitPositionPtr = *currentBitPositionPtr + 1;
            getBit( *currentBitPositionPtr, &byteOffset, &byteMask );
        }

        // NB currentBitPosition is now next bit to write

        // Return the number of bits written
        retVal = encoder::bitfields[i].numberOfBits;
    }
    else
    {
        Serial.print(F("Parameter 0x"));
        Serial.print( maskBit, HEX );
        Serial.println(F(" unavailable"));

        retVal = 0;
    }

    return retVal;
}

/**
* @brief Turn readings into a bitfield structured buffer
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

int encoder::encode( uint8_t* outputBufferPtr, struct encoder::readings* valuesPtr, uint16_t validValuesMask )
{
int currentBitPosition = 0;
int bitsWritten = 0;

    // Tell the remote end what fields it is actually receiving
    outputBufferPtr[0] = (validValuesMask & 0xFF00) >> 8;
    outputBufferPtr[1] = (validValuesMask & 0x00FF);

    // Assumption is that the validValuesMask cannot be larger than the datamask.
    currentBitPosition += encoder::BITS_FOR_DATAMASK;
    bitsWritten+=encoder::BITS_FOR_DATAMASK;

    // Move through the validValuesMask, adding fields that are present, and
    // keeping track of the current bit position in the output buffer.
    // writeBitfield will not write fields that are not specifed as valid by the validValuesMask.
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_PM2V5, valuesPtr->pm2v5, &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_PM10 , valuesPtr->pm10 , &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_TEMP , valuesPtr->temp , &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_RELH , valuesPtr->relh , &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_NOX  , valuesPtr->nox  , &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_CO2  , valuesPtr->co2  , &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_LAT  , valuesPtr->lat  , &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_LON  , valuesPtr->lon  , &currentBitPosition, outputBufferPtr, validValuesMask);
    bitsWritten += writeBitfield( encoder::DATA_CONTAINS_ALT  , valuesPtr->lon  , &currentBitPosition, outputBufferPtr, validValuesMask);

    // output buffer should now be ready
    // return number of bytes used in the output buffer
    return( 1 + (bitsWritten/8) );
}
