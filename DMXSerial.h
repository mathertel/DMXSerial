// - - - - -
// DMXSerial - A hardware supported interface to DMX.
// DMXSerial.h: Library header file
// 
// Copyright (c) 2011 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// 
// Documentation and samples are available at http://www.mathertel.de/Arduino
// 25.07.2011 creation of the DMXSerial library.
// 01.12.2011 include file changed to work with the Arduino 1.0 environment
// 10.05.2012 added method noDataSince to check how long no packet was received
// - - - - -

#ifndef DmxSerial_h
#define DmxSerial_h

#include <avr/io.h>

// ----- Constants -----

#define DMXSERIAL_MAX 512 // max. number of supported DMX data channels

#define DmxModePin 2     // Arduino pin 2 for controlling the data direction
#define DmxModeOut HIGH  // set the level to HIGH for outgoing data direction
#define DmxModeIn  LOW   // set the level to LOW  for incomming data direction

// ----- Enumerations -----

// Mode of Operation
typedef enum { 
  DMXNone, // unspecified
  DMXController, // always sending
  DMXReceiver    // always listening
} DMXMode;

// ----- Library Class -----

class DMXSerialClass
{
  public:
    // Initialize for specific mode.
    void    init       (int mode);

    // Set the maximum used channel for DMXController mode.
    void    maxChannel (int channel);

    // Read the last known value of a channel.
    uint8_t read       (int channel);

    // Write a new value of a channel.
    void    write      (int channel, uint8_t value);

    // Calculate how long no data backet was received
    unsigned long noDataSince();

    // Terminate operation.
    void    term       (void);
};

// Use the DMXSerial library through the DMXSerial object.
extern DMXSerialClass DMXSerial;

#endif
