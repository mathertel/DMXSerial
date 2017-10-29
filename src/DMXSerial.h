// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial.h: Library header file
// 
// Copyright (c) 2011-2014 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// 
// Documentation and samples are available at http://www.mathertel.de/Arduino
// 25.07.2011 creation of the DMXSerial library.
// 10.09.2011 fully control the serial hardware register
//            without using the Arduino Serial (HardwareSerial) class to avoid ISR implementation conflicts.
// 01.12.2011 include file changed to work with the Arduino 1.0 environment
// 28.12.2011 unused variable DmxCount removed
// 10.05.2012 added method noDataSince to check how long no packet was received
// 04.06.2012: set UCSRnA = 0 to use normal speed operation
// 30.07.2012 corrected TX timings with UDRE and TX interrupts
//            fixed bug in 512-channel RX
// 26.03.2013 #defines for the interrupt vector names
//            auto-increase _dmxMaxChannel
// 15.05.2013 Arduino Leonard and Arduino MEGA compatibility
// 19.05.2013 ATmega8 compatibility (beta)
// 24.08.2013 Optimizations for speed and size.
//            Removed some "volatile" annotations. 
// 12.07.2014 added update flag
// 19.03.2015 DMXModePin as optional parameter
// 25.08.2016 SCOPEDEBUG removed.
// 04.06.2017 Serial Initialization consolidated into _DMXSerialInit,
//            _DMXStartSending and _DMXStartReceiving functions.
// 27.08.2017 DMXProbe mode finished.
// 29.10.2017 documentation.
// - - - - -

#ifndef DmxSerial_h
#define DmxSerial_h

#include <avr/io.h>

// ----- Constants -----

#define DMXSERIAL_MAX 512 ///< max. number of supported DMX data channels

#define DMXMODEPIN 2     ///< Arduino pin 2 for controlling the data direction is the default value.
#define DmxModeOut HIGH  ///< set the level to HIGH for outgoing data direction
#define DmxModeIn  LOW   ///< set the level to LOW  for incomming data direction

#define DMXPROBE_RECV_MAX 50 // standard maximum of waiting for a DMX packet in DMXPROBE mode.

// ----- Enumerations -----

/**
 * Mode of Operation
 */
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

/**
 * @brief Arduino library to send and receive DMXX.
 *  
 * The library works unchanged with the Arduino 2009, UNO, MGEA 2560 and Leonardo boards. <br />
 * The Arduino MGEA 2560 boards use the serial port 0 on pins 0 an 1. <br />
 * The Arduino Leonardo will use serial port 1, also on pins 0 an 1. (on the 32u4 boards the first USART is USART1) <br />
 * This is consistent with the Layout of the Arduino DMX Shield http://www.mathertel.de/Arduino/DMXShield.aspx.
 */
class DMXSerialClass
{
  public:
    /**
     * @brief Initialize or re-initialize the specified mode.
     * This function can be called to switch to restart or switch the operatin mode. 
     * @param [in] mode The mode of operation to be started. A value from enum DMXMode;
     * @return void    
     */
    void    init (int mode);


    /**
     * @brief Initialize the specified mode including a specific mode pin.
     * This function can be called to switch to restart or switch the operatin mode. 
     * @param [in] mode The mode of operation to be started. A value from enum DMXMode;
     * @param [in] modePin The pin number for switching communication direction.
     * @return void    
     */
    void    init (int mode, int modePin);

    /**
     * @brief Set the maximum used channel for DMXController mode.
     * @param [in] channel The highest channel that will be transferred. 
     * @return void    
     */
    void    maxChannel (int channel);

    /**
     * @brief Read the current value of a channel.
     * @param [in] channel The channel number.
     * @return uint8_t The current value.
     */
    uint8_t read       (int channel);

    /**
     * @brief Write a new value to a channel.
     * This function also can be called in DMXReceiver mode to set a channel value even when no data is received.
     * It will be overwritten by the next received package.
     * @param [in] channel The channel number.
     * @param [in] value The current value.
     * @return void
     */
    void    write      (int channel, uint8_t value);

    /**
     * @brief Get a pointer to DMX Buffer.
     * This is the internal byte-array where the current DMX values are stored. 
     * @return uint8_t DMX values buffer.
     */
    uint8_t *getBuffer();
    
    /**
     * @brief Return the duration since data was received.
     * On startup the internal timer is reset too.
     * @return long milliseconds since last pdata package.
     */
    unsigned long noDataSince();

    /**
     * @brief Check for changed data.
     * Check DMX data was updated by a received package since last resetUpdated() call or onUpdate callback.
     * @return true Some DMX data was updated.
     * @return false No updated data.
     */
    bool dataUpdated();

    /**
     * @brief Reset DMX data update flag.
     */
    void resetUpdated();

    /**
     * @brief Actively wait for an incomming DMX packet.
     * This function waits DMXPROBE_RECV_MAX milliseconds for a new package to receive.
     * @return true when a package was received.
     * @return false after timeout no package was received.
     */
    bool receive();

    /**
     * @brief Actively wait for an incomming DMX packet.
     * This function waits the specified milliseconds for a new package to receive.
     * @param wait Milliseconds to wait for a new package.
     * @return true when a package was received.
     * @return false after timeout no package was received.
     */
    bool receive(uint8_t wait);
    
    /**
     * @brief Terminate the current operation mode.
     */
    void    term();
    
  private:
    // Not used.
    // all private information is in the global _dmxXXX variables for speed and code size optimization.
    // @see DMXSerial.cpp.

};

// Use the DMXSerial library through the DMXSerial object.
// There is only one DMX port supported and DMXSerial is a static object.
extern DMXSerialClass DMXSerial;

#endif
