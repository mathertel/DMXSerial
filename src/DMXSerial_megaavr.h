// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial_magaavr.h: Hardware specific functions for MEGAAVR processors like 4809 used in Arduino Every.

// Copyright (c) 2011-2020 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// - - - - -

// global variables and functions are prefixed with "_DMX_"

// ----- MegaAVR specific Hardware abstraction functions -----

#ifndef DMXSERIAL_MEGAAVR_H
#define DMXSERIAL_MEGAAVR_H

#if defined(DMXFORMAT) && defined(ARDUINO_ARCH_MEGAAVR)

#include "Arduino.h"
#include "DMXSerial.h"
#include "avr/io.h"


int32_t _DMX_dmxDivider; // BAUD Devider factor for DMX speed.
int32_t _DMX_breakDivider; // BAUD Devider factor for BREAK speed.


/// Initialize the Hardware MUX and UART serial port.
void _DMX_init()
{
  int32_t baud;

  int8_t oscErr = SIGROW.OSC16ERR5V;

  // calculate DMX speed Divider
  baud = (DMXSPEED * (1024 + oscErr)) / 1024;
  _DMX_dmxDivider = (64 * F_CPU) / (16 * baud);

  // calculate BREAK speed Divider
  baud = (BREAKSPEED * (1024 + oscErr)) / 1024;
  _DMX_breakDivider = (64 * F_CPU) / (16 * baud);

  // disable interrupts during initialization
  uint8_t oldSREG = SREG;
  cli();

  // Setup port mux
  PORTMUX.USARTROUTEA |= PORTMUX_USART1_ALT1_gc;

  // Disable CLK2X, clock normal rate
  (USART1).CTRLB = USART_RXMODE_NORMAL_gc;

  //Set up the rx & tx pins
  pinMode(PIN_WIRE_HWSERIAL1_RX, INPUT_PULLUP);
  pinMode(PIN_WIRE_HWSERIAL1_TX, OUTPUT);

  // enable interrupts again, restore SREG content
  SREG = oldSREG;
} // _DMX_init()


/// Initialize the Hardware UART serial port to the required mode.
void _DMX_setMode(DMXUARTMode mode)
{
  uint16_t baud_setting;
  uint8_t flags;
  uint8_t format;

  // disable interrupts during initialization
  uint8_t oldSREG = SREG;
  cli();

  if (mode == DMXUARTMode::OFF) {
    // Disable transmitter and receiver
    (USART1).CTRLB = USART_RXMODE_NORMAL_gc; // (USART_RXEN_bm | USART_TXEN_bm);
    (USART1).CTRLA = 0; // disable all interrupts

  } else if (mode == DMXUARTMode::RONLY) {
    (USART1).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (USART1).CTRLC = DMXREADFORMAT; // accept data packets after first stop bit
    (USART1).CTRLB = USART_RXEN_bm | USART_RXMODE_NORMAL_gc; // Enable receiver only, normal speed
    (USART1).CTRLA = 0; // disable all interrupts

  } else if (mode == DMXUARTMode::RDATA) {
    (USART1).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (USART1).CTRLC = DMXREADFORMAT; // accept data packets after first stop bit
    (USART1).CTRLB = USART_RXEN_bm | USART_RXMODE_NORMAL_gc; // Enable receiver only, normal speed
    (USART1).CTRLA = USART_RXCIE_bm; // enable receive complete interrupt

  } else if (mode == DMXUARTMode::TBREAK) {
    // start UART with break settings, don't enable interrupts yet
    (USART1).CTRLB = 0; // no operation
    (USART1).CTRLA = 0; // disable all interrupts

    (USART1).BAUD = (int16_t)_DMX_breakDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (USART1).CTRLC = BREAKFORMAT; // Set USART mode of operation
    (USART1).CTRLB = USART_TXEN_bm; // Enable transmitter only, normal speed
    pinMode(PIN_WIRE_HWSERIAL1_TX, OUTPUT); // is required again after disabling UART
    (USART1).STATUS = USART_TXCIF_bm; //  clear transmit complete flag
    (USART1).CTRLA = USART_TXCIE_bm; // enable transmit complete interrupt

  } else if (mode == DMXUARTMode::TDATA) {
    // switch to dmx data mode
    (USART1).CTRLA = 0; // disable all interrupts
    (USART1).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (USART1).CTRLC = DMXFORMAT; // send with 2 stop bits for compatibility
    (USART1).CTRLA = USART_DREIE_bm; // enable data register empty interrupt

  } else if (mode == DMXUARTMode::TDONE) {
    (USART1).BAUD = (int16_t)_DMX_dmxDivider; // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (USART1).CTRLC = DMXFORMAT; // send with 2 stop bits for compatibility
    (USART1).STATUS = USART_TXCIF_bm; //  clear transmit complete flag
    (USART1).CTRLA = USART_TXCIE_bm; // enable transmit complete interrupt
  } // if

  // enable interrupts again, restore SREG content
  SREG = oldSREG;
} // _DMX_setMode()


// flush all incomming data packets in the queue
void _DMX_flush()
{
  uint8_t voiddata = (USART1).RXDATAL;
}


// send the next byte after current byte was sent completely.
inline void _DMX_writeByte(uint8_t data)
{
  // putting data into TXDATAL sends the data
  (USART1).TXDATAL = data;
} // _DMX_writeByte


// This Interrupt Service Routine is called when a byte or frame error was received.
// In DMXController mode this interrupt is disabled and will not occur.
// In DMXReceiver mode when a byte or frame error was received
ISR(USART1_RXC_vect)
{
  register8_t rxferr = (USART1).RXDATAH & USART_FERR_bm;
  register8_t rxdata = (USART1).RXDATAL;
  _DMXReceived(rxdata, rxferr);
} // ISR(USART1_RXC_vect)


// Interrupt service routines that are called when the actual byte was sent.
ISR(USART1_TXC_vect)
{
  _DMXTransmitted();
} // ISR(USART1_TXC_vect)


// this interrupt occurs after data register was emptied by handing it over to the shift register.
ISR(USART1_DRE_vect)
{
  _DMXTransmitted();
} // ISR(USART1_DRE_vect)

#endif

#endif
