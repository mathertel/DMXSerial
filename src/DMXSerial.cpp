// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial.cpp: Library implementation file
//
// Copyright (c) 2011-2020 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
//
// Documentation and samples are available at http://www.mathertel.de/Arduino
// Changelog: See DMXSerial.h
// - - - - -

#include "Arduino.h"
#include "DMXSerial.h"


// ----- forwards -----

void _DMXStartSending();
void _DMXStartReceiving();

// register interrupt for receiving data and frameerrors that calls _DMXReceived()
void _DMXReceived(uint8_t data, uint8_t frameerror);
void _DMXTransmitted();


// These functions all exist in the processor specific implementations:
void _DMX_init();
void _DMX_setMode();
void _DMX_writeByte(uint8_t data);
void _DMX_flush();


// ----- Serial UART Modes -----

// There are 4 different modes required while receiving and sending DMX using the Serial

// State of receiving DMX Bytes
typedef enum {
  OFF = 0, // Turn off
  RONLY = 1, // Receive DMX data only
  RDATA = 2, // Receive DMX data + Interrupt
  TBREAK = 3, // Transmit DMX Break + Interrupt on Transmission completed
  TDATA = 4, // Transmit DMX Data + Interrupt on Register empty
  TDONE = 5 // Transmit DMX Data + Interrupt on Transmission completed
} __attribute__((packed)) DMXUARTMode;


// Baud rate for DMX protocol
#define DMXSPEED 250000L

// the break timing is 10 bits (start + 8 data + 1 (even) parity) of this speed
// the mark-after-break is 1 bit of this speed plus approx 6 usec
// 100000 bit/sec is good: gives 100 usec break and 16 usec MAB
// 1990 spec says transmitter must send >= 92 usec break and >= 12 usec MAB
// receiver must accept 88 us break and 8 us MAB
#define BREAKSPEED 100000L

#define BREAKFORMAT SERIAL_8E2
#define DMXFORMAT SERIAL_8N2
#define DMXREADFORMAT SERIAL_8N1


// ----- include processor specific definitions and functions.

#if defined(ARDUINO_ARCH_AVR)
#include "DMXSerial_avr.h"

#elif defined(ARDUINO_ARCH_MEGAAVR)
#include "DMXSerial_megaavr.h"

#endif


// ----- Enumerations -----

// State of receiving DMX Bytes
typedef enum {
  STARTUP = 1, // wait for any interrupt BEFORE starting analyzing the DMX protocoll.
  IDLE = 2, // wait for a BREAK condition.
  BREAK = 3, // BREAK was detected.
  DATA = 4, // DMX data.
  DONE = 5 // All channels received.
} __attribute__((packed)) DMXReceivingState;


// ----- DMXSerial Private variables -----
// These variables are not class members because they have to be reached by the interrupt implementations.
// don't use these variable from outside, use the appropriate methods.

DMXMode _dmxMode; // Mode of Operation
int _dmxModePin; // pin used for I/O direction.

uint8_t _dmxRecvState; // Current State of receiving DMX Bytes
int _dmxChannel; // the next channel byte to be sent.

volatile unsigned int _dmxMaxChannel = 32; // the last channel used for sending (1..32).
volatile unsigned long _dmxLastPacket = 0; // the last time (using the millis function) a packet was received.

bool _dmxUpdated = true; // is set to true when new data arrived.

// Array of DMX values (raw).
// Entry 0 will never be used for DMX data but will store the startbyte (0 for DMX mode).
uint8_t _dmxData[DMXSERIAL_MAX + 1];

// This pointer will point to the next byte in _dmxData;
uint8_t *_dmxDataPtr;

// This pointer will point to the last byte in _dmxData;
uint8_t *_dmxDataLastPtr;

// Create a single class instance. Multiple class instances (multiple simultaneous DMX ports) are not supported.
DMXSerialClass DMXSerial;


// ----- Class implementation -----

// Initialize the specified mode.
void DMXSerialClass::init(int mode)
{
  init(mode, DMXMODEPIN);
}

// (Re)Initialize the specified mode.
// The mode parameter should be a value from enum DMXMode.
void DMXSerialClass::init(int mode, int dmxModePin)
{
  // initialize global variables
  _dmxMode = DMXNone;
  _dmxModePin = dmxModePin;
  _dmxRecvState = STARTUP; // initial state
  _dmxChannel = 0;
  _dmxDataPtr = _dmxData;
  _dmxLastPacket = millis(); // remember current (relative) time in msecs.

  _dmxMaxChannel = DMXSERIAL_MAX; // The default in Receiver mode is reading all possible 512 channels.
  _dmxDataLastPtr = _dmxData + _dmxMaxChannel;

  // initialize the DMX buffer
  //  memset(_dmxData, 0, sizeof(_dmxData));
  for (int n = 0; n < DMXSERIAL_MAX + 1; n++)
    _dmxData[n] = 0;

  // now start
  _dmxMode = (DMXMode)mode;

  if ((_dmxMode == DMXController) || (_dmxMode == DMXReceiver) || (_dmxMode == DMXProbe)) {
    // a valid mode was given
    // Setup external mode signal
    _DMX_init();

    pinMode(_dmxModePin, OUTPUT); // enables the pin for output to control data direction
    digitalWrite(_dmxModePin, DmxModeIn); // data in direction, to avoid problems on the DMX line for now.

    if (_dmxMode == DMXController) {
      digitalWrite(_dmxModePin, DmxModeOut); // data Out direction
      _dmxMaxChannel = 32; // The default in Controller mode is sending 32 channels.
      _DMXStartSending();

    } else if (_dmxMode == DMXReceiver) {
      // Setup Hardware
      _DMXStartReceiving();

      // } else if (_dmxMode == DMXProbe) {
      //   // don't setup the Hardware now

    } // if
  } // if
} // init()


// Set the maximum used channel.
// This method can be called any time before or after the init() method.
void DMXSerialClass::maxChannel(int channel)
{
  if (channel < 1)
    channel = 1;
  if (channel > DMXSERIAL_MAX)
    channel = DMXSERIAL_MAX;
  _dmxMaxChannel = channel;
  _dmxDataLastPtr = _dmxData + channel;
} // maxChannel


// Read the current value of a channel.
uint8_t DMXSerialClass::read(int channel)
{
  // adjust parameter
  if (channel < 1)
    channel = 1;
  if (channel > DMXSERIAL_MAX)
    channel = DMXSERIAL_MAX;
  // read value from buffer
  return (_dmxData[channel]);
} // read()


// Write the value into the channel.
// The value is just stored in the sending buffer and will be picked up
// by the DMX sending interrupt routine.
void DMXSerialClass::write(int channel, uint8_t value)
{
  // adjust parameters
  if (channel < 1)
    channel = 1;
  if (channel > DMXSERIAL_MAX)
    channel = DMXSERIAL_MAX;
  if (value < 0)
    value = 0;
  if (value > 255)
    value = 255;

  // store value for later sending
  _dmxData[channel] = value;

  // Make sure we transmit enough channels for the ones used
  if (channel > _dmxMaxChannel) {
    _dmxMaxChannel = channel;
    _dmxDataLastPtr = _dmxData + _dmxMaxChannel;
  } // if
} // write()


// Return the DMX buffer for un-save direct but faster access
uint8_t *DMXSerialClass::getBuffer()
{
  return (_dmxData);
} // getBuffer()


// Calculate how long no data packet was received
unsigned long DMXSerialClass::noDataSince()
{
  unsigned long now = millis();
  return (now - _dmxLastPacket);
} // noDataSince()


// return true when some DMX data was updated.
bool DMXSerialClass::dataUpdated()
{
  return (_dmxUpdated);
}


// reset DMX data update flag.
void DMXSerialClass::resetUpdated()
{
  _dmxUpdated = false;
}


// When mode is DMXProbe this function reads one DMX buffer and then returns.
// wait a meaningful time on a packet.
// return true when a packet has been received.
bool DMXSerialClass::receive()
{
  return (receive(DMXPROBE_RECV_MAX));
} // receive()


// When mode is DMXProbe this function reads one DMX buffer and then returns.
// wait approximately gives the number of msecs for waiting on a packet.
// return true when a packet has been received.
bool DMXSerialClass::receive(uint8_t wait)
{
  bool ret = false;

  if (_dmxMode == DMXProbe) {
    _DMXStartReceiving();
    // UCSRnA
    while ((wait > 0) && (_dmxRecvState != DONE)) {
      delay(1);
      wait--;
    } // while

    if (_dmxRecvState == DONE) {
      ret = true;
    } else {
      _DMX_setMode(DMXUARTMode::RONLY);
    } // if
  } // if

  return (ret);
} // receive(wait)


// Terminate operation
void DMXSerialClass::term(void)
{
  // Disable all USART Features, including Interrupts
  _DMX_setMode(DMXUARTMode::OFF);
} // term()


// ----- internal functions and interrupt implementations -----


// Setup Hardware for Sending
void _DMXStartSending()
{
  // Start sending a BREAK and send more bytes in UDRE ISR
  // Enable transmitter and interrupt
  _DMX_setMode(DMXUARTMode::TBREAK);
  _DMX_writeByte((uint8_t)0);
} // _DMXStartSending()


// Setup Hardware for Receiving
void _DMXStartReceiving()
{
  uint8_t voiddata;

  // Enable receiver and Receive interrupt
  _dmxDataPtr = _dmxData;
  _dmxRecvState = STARTUP;

  _DMX_setMode(DMXUARTMode::RDATA);
  _DMX_flush();
} // _DMXStartReceiving()


// This function is called by the Interrupt Service Routine when a byte or frame error was received.
// In DMXController mode this interrupt is disabled and will not occur.
// In DMXReceiver mode when a byte was received it is stored to the dmxData buffer.
void _DMXReceived(uint8_t data, uint8_t frameerror)
{
  uint8_t DmxState = _dmxRecvState; //just load once from SRAM to increase speed

  if (DmxState == STARTUP) {
    // just ignore any first frame comming in
    _dmxRecvState = IDLE;
    return;
  }

  if (frameerror) { //check for break
    // break condition detected.
    _dmxRecvState = BREAK;
    _dmxDataPtr = _dmxData;

  } else if (DmxState == BREAK) {
    // first byte after a break was read.
    if (data == 0) {
      // normal DMX start code (0) detected
      _dmxRecvState = DATA;
      _dmxLastPacket = millis(); // remember current (relative) time in msecs.
      _dmxDataPtr++; // start saving data with channel # 1

    } else {
      // This might be a RDM or customer DMX command -> not implemented so wait for next BREAK !
      _dmxRecvState = DONE;
    } // if

  } else if (DmxState == DATA) {
    // check for new data
    if (*_dmxDataPtr != data) {
      _dmxUpdated = true;
      // store received data into dmx data buffer.
      *_dmxDataPtr = data;
    } // if
    _dmxDataPtr++;

    if (_dmxDataPtr > _dmxDataLastPtr) {
      // all channels received.
      _dmxRecvState = DONE;
    } // if
  } // if

  if (_dmxRecvState == DONE) {
    if (_dmxMode == DMXProbe) {
      // stop creating interrupts on the serial port for now.
      _DMX_setMode(DMXUARTMode::RONLY);

    } else {
      // continue on DMXReceiver mode.
      _dmxRecvState = IDLE; // wait for next break
    }
  } // if

} // _DMXReceived()


// This function is called by the Transmission complete or Data Register empty interrupt routine.
// When changing speed (after sending BREAK) we use TX finished interrupt that occurs shortly after the last stop bit is sent
// When staying at the same speed (sending data bytes) we use data register empty interrupt that occurs shortly after the start bit of the *previous* byte
// When sending a DMX sequence it just takes the next channel byte and sends it out.
// In DMXController mode when the buffer was sent completely the DMX sequence will resent, starting with a BREAK pattern.
// In DMXReceiver mode this interrupt is disabled and will not occur.
void _DMXTransmitted()
{
  if ((_dmxMode == DMXController) && (_dmxChannel == -1)) {
    // this occurs after the stop bits of the last data byte
    // start sending a BREAK and loop forever in ISR
    _DMX_setMode(DMXUARTMode::TBREAK);
    _DMX_writeByte((uint8_t)0);
    _dmxChannel = 0; // next time send start byte

  } else if (_dmxChannel == 0) {
    // this occurs after the stop bits of the break byte
    // now back to DMX speed: 250000baud
    // take next interrupt when data register empty (early)
    _DMX_setMode(DMXUARTMode::TDATA);

    // write start code
    _DMX_writeByte((uint8_t)0);
    _dmxChannel = 1;

  } else {
    if (_dmxChannel < _dmxMaxChannel) {
      // just send the next data
      _DMX_writeByte(_dmxData[_dmxChannel++]);
    } else {
      // last data
      _DMX_setMode(DMXUARTMode::TDONE);
      _DMX_writeByte(_dmxData[_dmxChannel]);
      _dmxChannel = -1; // this series is done. Next time: restart with break.
    } // if

  } // if
} // _DMXTransmitted


// The End
