// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial.h: Library header file
// 
// Copyright (c) 2011-2014 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// 
// Documentation and samples are available at http://www.mathertel.de/Arduino
// 25.07.2011 creation of the DMXSerial library.
// 01.12.2011 include file changed to work with the Arduino 1.0 environment
// 10.05.2012 added method noDataSince to check how long no packet was received
// 12.07.2014 added update flag
// 19.03.2015 DMXModePin as optional parameter
//  Until here the maxChannel feature only was used in DMXController mode.
//  Now it enables triggering the onUpdate function in DMX receiver mode.
// 
//  See the WebSite for more information.
//
// - - - - -

#ifndef DmxSerial_h
#define DmxSerial_h

#include <avr/io.h>

// ----- Constants -----

#define DMXSERIAL_MAX 512 // max. number of supported DMX data channels

#define DMXMODEPIN 2     // Arduino pin 2 for controlling the data direction is the default value.
#define DmxModeOut HIGH  // set the level to HIGH for outgoing data direction
#define DmxModeIn  LOW   // set the level to LOW  for incomming data direction

#define DMXPROBE_RECV_MAX 50 // standard maximum of waiting for a DMX packet in DMXPROBE mode.

// ----- Enumerations -----

// Mode of Operation
typedef enum { 
  DMXNone, // unspecified
  DMXController , // always sending
  DMXReceiver,   // always listening
  DMXProbe       // send and receive upon request
} DMXMode;

// ----- Library Class -----

extern "C" {
  typedef void (*dmxUpdateFunction)(void);
}

class DMXSerialClass
{
  public:
    // Initialize for specific mode.
    void    init (int mode);

    // Initialize for specific mode including a specific mode pin.
    void    init (int mode, int modePin);

    // Set the maximum used channel for DMXController mode.
    void    maxChannel (int channel);

    // Read the last known value of a channel.
    uint8_t read       (int channel);

    // Write a new value of a channel.
    void    write      (int channel, uint8_t value);

    uint8_t *getBuffer();
    
    // Calculate how long no data packet was received
    unsigned long noDataSince();

    // return true when some DMX data was updated since last resetUpdated() call or onUpdate callback.
    bool dataUpdated();

    // reset DMX data update flag.
    void resetUpdated();

    // actively wait for an incomming DMX packet.
    bool receive();

    // actively wait for an incomming DMX packet, specifying the maximal waiting time in msecs.
    bool receive(uint8_t wait);
    
    // Terminate operation.
    void    term       (void);
    
  private:
    // Not used.
    // all private information is in the global _dmxXXX variables for speed and code size optimization.
    // See DMXSerial.cpp.


};

// Use the DMXSerial library through the DMXSerial object.
// There is only one DMX port supported and DMXSerial is a static object.
extern DMXSerialClass DMXSerial;

#endif
