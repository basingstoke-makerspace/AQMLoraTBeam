# AQMLoraTBeam
Code for running AQM ( Air Quality Monitoring ) sensors on an ESP32 TBeam device using Lorawan.

Supported sensors currently - SDS011 PM sensor, DHT ( 11/22 ) temperature and humidity sensor.
Also supporting the NEO6M GPS sensor integrated on the T-Beam device.

Project set up in PlatformIO, based on IBM Zurich LMIC code for Lora as ported for Arduino framework ( https://www.arduinolibraries.info/libraries/ibm-lmic-framework ). Also using TinyGPS++ and HardwareSerial.

NB This project is not using FreeRTOS. At least, not as far as I'm aware. This is not a reflection on 
FreeRTOS, more on the simplicity of the functionality required here, and the fact that the LMIC 
framework appears sufficient at this time. 

Code is currently C-stylee, switch to C++-stylee may happen after core functionality is verified.

Although inspired by the Luftdaten project, this code ( with the exception of some LMIC structure ) is
a complete ground-up redesign. This is because (a) It was just more fun that way (b) I wanted to use a
permissive non-GPL licence (c) Adding Lorawan support to the Luftdaten codebase seemed like it would be
as much effort as starting with the LMIC code and adding sensors. 
