function getBit( bytes, offset )
{
    // byte to use
    index = Math.floor( offset/8 );
    // find bit required in byte to use
    // NB an offset of 8 is the MS bit of the byte at index 1
    bitIndex = 8 -(offset%8);
    bitMask = 0x01 << (bitIndex-1);

    return (bytes[index] & bitMask) >> (bitIndex-1);
}

var DATA_CONTAINS_PM2V5     = 0x01;
var DATA_CONTAINS_PM10      = 0x02;
var DATA_CONTAINS_TEMP      = 0x04;
var DATA_CONTAINS_RELH      = 0x08;
var DATA_CONTAINS_NOX       = 0x10;
var DATA_CONTAINS_CO2       = 0x20;
var DATA_CONTAINS_UNUSED1   = 0x40;
var DATA_CONTAINS_UNUSED2   = 0x80;
var BITS_FOR_DATAMASK       =  8;
var BITS_FOR_PM2V5          =  9;
var BITS_FOR_PM10           =  9;
var BITS_FOR_TEMP           =  6;
var BITS_FOR_RELH           =  7;
var BITS_FOR_NOX            =  8;
var BITS_FOR_CO2            =  7;
var BITS_FOR_UNUSED1        =  0;
var BITS_FOR_UNUSED2        =  0;
var VALUE_OFFSET_FOR_PM2V5     =  0;
var VALUE_OFFSET_FOR_PM10      =  0;
var VALUE_OFFSET_FOR_TEMP      =  20;
var VALUE_OFFSET_FOR_RELH      =  0;
var VALUE_OFFSET_FOR_NOX       =  -20;
var VALUE_OFFSET_FOR_CO2       =  -370;
var VALUE_OFFSET_FOR_UNUSED1   =  0;
var VALUE_OFFSET_FOR_UNUSED2   =  0;

function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  // if (port === 1) decoded.led = bytes[0];

  var whatsThere = bytes[0];

  var currentBit = 8;

  var i;

  var bitValue;

  var setMask;

  if( whatsThere & DATA_CONTAINS_PM2V5 )
  {
      decoded.pm2V5 = 0;

      // This loop retrieves bits from MS to LS
      for( i=0; i<BITS_FOR_PM2V5; i++)
      {
          bitValue = getBit( bytes, currentBit+i );

          if( bitValue == 1 )
          {
              setMask = 0x00000001 << ((BITS_FOR_PM2V5 - i) - 1 );
              decoded.pm2V5 |= setMask;
          }
      }

      decoded.pm2V5 -= VALUE_OFFSET_FOR_PM2V5;

      currentBit += BITS_FOR_PM2V5;
  }

  if( whatsThere & DATA_CONTAINS_PM10 )
  {
      decoded.pm10 = 0;

      // This loop retrieves bits from MS to LS
      for( i=0; i<BITS_FOR_PM10; i++)
      {
          bitValue = getBit( bytes, currentBit+i );

          if( bitValue == 1 )
          {
              setMask = 0x00000001 << ((BITS_FOR_PM10 - i) - 1 );
              decoded.pm10 |= setMask;
          }
      }

      decoded.pm10 -= VALUE_OFFSET_FOR_PM10;

      currentBit += BITS_FOR_PM10;
  }

  if( whatsThere & DATA_CONTAINS_TEMP )
  {
      decoded.temp = 0;

      // This loop retrieves bits from MS to LS
      for( i=0; i<BITS_FOR_TEMP; i++)
      {
          bitValue = getBit( bytes, currentBit+i );

          if( bitValue == 1 )
          {
              setMask = 0x00000001 << ((BITS_FOR_TEMP - i) - 1 );
              decoded.temp |= setMask;
          }
      }

      decoded.temp -= VALUE_OFFSET_FOR_TEMP;

      currentBit += BITS_FOR_TEMP;
  }

  if( whatsThere & DATA_CONTAINS_RELH )
  {
      decoded.relh = 0;

      // This loop retrieves bits from MS to LS
      for( i=0; i<BITS_FOR_RELH; i++)
      {
          bitValue = getBit( bytes, currentBit+i );

          if( bitValue == 1 )
          {
              setMask = 0x00000001 << ((BITS_FOR_RELH - i) - 1 );
              decoded.relh |= setMask;
          }
      }

      decoded.relh -= VALUE_OFFSET_FOR_RELH;

      currentBit += BITS_FOR_RELH;
  }

  if( whatsThere & DATA_CONTAINS_NOX )
  {

  }

  if( whatsThere & DATA_CONTAINS_CO2 )
  {

  }

  return decoded;
}
