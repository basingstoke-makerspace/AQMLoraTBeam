/*******************************************************************************
 * Copyright (c) 2019 Mark de Roussier
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * log filter, because the TTGO board doesn't do JTAG and there's an uncomfortable
 * amount of serial i/o doing logging, so allow it to be controlled.
 *
 */
#pragma once

#include <string>

#include <HardwareSerial.h>

class log
{
    public:
    
    void debug( std::string outputString );
    void info( std::string outputString );
    void warn( std::string outputString );
    void error( std::string outputString );
    void always( std::string outputString );

    typedef unsigned int tLogLevel;
    
    const tLogLevel DEBUG = 0;
    const tLogLevel INFO = 1;
    const tLogLevel WARN = 2;
    const tLogLevel ERROR = 3;

    void setLevel( tLogLevel logLevel );

    private:

    void writeLogMsg( const char* cstring );
    
    tLogLevel logLevel;
};