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
    bool getReading( int* valuePtr, uint16_t readingMask );
    uint16_t getReadings( struct encoder::readings* readingsPtr );
}
