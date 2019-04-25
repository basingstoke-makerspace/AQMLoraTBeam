/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 */

#pragma once

#include <cstdint>
#include <encoder.hpp>

namespace getreadings
{
    // Schedule TX every this many seconds (might become longer due to duty
    // cycle limitations).
    const uint32_t  TX_INTERVAL = 120;

    bool getReading( int* valuePtr, uint8_t readingMask );
    uint8_t getReadings( struct encoder::readings* readingsPtr );
}
