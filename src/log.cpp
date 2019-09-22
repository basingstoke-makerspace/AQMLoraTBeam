/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This file defines a log filter class.
 *
 *******************************************************************************/

#include "log.hpp"

void log::writeLogMsg( const char* cstring )
{
    Serial.print( cstring );
}

void log::setLevel( log::tLogLevel level )
{
    log::logLevel = level;
}

void log::debug( std::string outputMsg )
{
    if( log::DEBUG == log::logLevel  )
    {
        writeLogMsg( outputMsg.c_str() );
    }
}

void log::info( std::string outputMsg )
{
    if( log::INFO >= log::logLevel  )
    {
        writeLogMsg( outputMsg.c_str() );
    }
}

void log::warn( std::string outputMsg )
{
    if( log::WARN >= log::logLevel  )
    {
        writeLogMsg( outputMsg.c_str() );
    }
}

void log::error( std::string outputMsg )
{
    if( log::ERROR >= log::logLevel  )
    {
        writeLogMsg( outputMsg.c_str() );
    }
}

void log::always( std::string outputMsg )
{
    writeLogMsg( outputMsg.c_str() );
}
