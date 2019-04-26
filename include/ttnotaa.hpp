/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Only need this namespace for sharing of const things originally in
 * ttn-otaa.ino.
 */

#pragma once

namespace ttnotaa
{
    // Schedule TX every this many seconds (might become longer due to duty
    // cycle limitations).
    const uint32_t  TX_INTERVAL = 120;
}
