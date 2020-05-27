// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial_avr.h: Hardware specific functions for AVR processors like ATmega168 and ATmega328 used in Aurduino UNO.
// Also supported boards are Leonardo and Mega2560.
//
// Copyright (c) 2011-2020 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// - - - - -

// global variables and functions are prefixed with "_DMX_"

// ----- ATMega specific hardware related functions -----

#ifndef DMXSERIAL_AVR_H
#define DMXSERIAL_AVR_H

#if defined(DMXFORMAT) && defined(ARDUINO_ARCH_AVR)

#include "Arduino.h"
#include "DMXSerial.h"
#include "avr/io.h"
#include "avr/interrupt.h"

// ----- Constants -----

// Define port & bit values for Hardware Serial Port.
// The library works unchanged with the Arduino 2009, UNO, MEGA 2560 and Leonardo boards.
// The Arduino MEGA 2560 boards use the serial port 0 on pins 0 an 1.
// The Arduino Leonardo will use serial port 1, also on pins 0 an 1. (on the 32u4 boards the first USART is USART1)
// This is consistent to the Layout of the Arduino DMX Shield http://www.mathertel.de/Arduino/DMXShield.aspx.

// For using the serial port 1 on a Arduino MEGA 2560 board, enable the following DMX_USE_PORT1 definition.
// #define DMX_USE_PORT1

#if !defined(DMX_USE_PORT1) && defined(USART_RXC_vect)
// These definitions are used on ATmega8 boards
#define UCSRnA UCSRA // Control and Status Register A
#define TXCn TXC // Transmit buffer clear

#define UCSRnB UCSRB // USART Control and Status Register B

#define RXCIEn RXCIE // Enable Receive Complete Interrupt
#define TXCIEn TXCIE // Enable Transmission Complete Interrupt
#define UDRIEn UDRIE // Enable Data Register Empty Interrupt
#define RXENn RXEN // Enable Receiving
#define TXENn TXEN // Enable Sending

#define UCSRnC UCSRC // Control and Status Register C
#define USBSn USBS // Stop bit select 0=1bit, 1=2bits
#define UCSZn0 UCSZ0 // Character size 00=5, 01=6, 10=7, 11=8 bits
#define UPMn0 UPM0 // Parity setting 00=N, 10=E, 11=O

#define UBRRnH UBRRH // USART Baud Rate Register High
#define UBRRnL UBRRL // USART Baud Rate Register Low

#define UDRn UDR // USART Data Register
#define UDREn UDRE // USART Data Ready
#define FEn FE // Frame Error

#define USARTn_RX_vect USART_RXC_vect // Interrupt Data received
#define USARTn_TX_vect USART_TXC_vect // Interrupt Data sent
#define USARTn_UDRE_vect USART_UDRE_vect // Interrupt Data Register empty


#elif !defined(DMX_USE_PORT1) && defined(USART_RX_vect)
// These definitions are used on ATmega168p and ATmega328p boards
// like the Arduino Diecimila, Duemilanove, 2009, Uno
#define UCSRnA UCSR0A
#define RXCn RXC0
#define TXCn TXC0
#define UCSRnB UCSR0B
#define RXCIEn RXCIE0
#define TXCIEn TXCIE0
#define UDRIEn UDRIE0
#define RXENn RXEN0
#define TXENn TXEN0
#define UCSRnC UCSR0C
#define USBSn USBS0
#define UCSZn0 UCSZ00
#define UPMn0 UPM00
#define UBRRnH UBRR0H
#define UBRRnL UBRR0L
#define UDRn UDR0
#define UDREn UDRE0
#define FEn FE0
#define USARTn_RX_vect USART_RX_vect
#define USARTn_TX_vect USART_TX_vect
#define USARTn_UDRE_vect USART_UDRE_vect

#elif !defined(DMX_USE_PORT1) && defined(USART0_RX_vect)
// These definitions are used on ATmega1280 and ATmega2560 boards
// like the Arduino MEGA boards
#define UCSRnA UCSR0A
#define RXCn RXC0
#define TXCn TXC0
#define UCSRnB UCSR0B
#define RXCIEn RXCIE0
#define TXCIEn TXCIE0
#define UDRIEn UDRIE0
#define RXENn RXEN0
#define TXENn TXEN0
#define UCSRnC UCSR0C
#define USBSn USBS0
#define UCSZn0 UCSZ00
#define UPMn0 UPM00
#define UBRRnH UBRR0H
#define UBRRnL UBRR0L
#define UDRn UDR0
#define UDREn UDRE0
#define FEn FE0
#define USARTn_RX_vect USART0_RX_vect
#define USARTn_TX_vect USART0_TX_vect
#define USARTn_UDRE_vect USART0_UDRE_vect

#elif defined(DMX_USE_PORT1) || defined(USART1_RX_vect)
// These definitions are used for using serial port 1
// on ATmega32U4 boards like Arduino Leonardo, Esplora
// You can use it on other boards with USART1 by enabling the DMX_USE_PORT1 port definition
#define UCSRnA UCSR1A
#define RXCn RXC1
#define TXCn TXC1
#define UCSRnB UCSR1B
#define RXCIEn RXCIE1
#define TXCIEn TXCIE1
#define UDRIEn UDRIE1
#define RXENn RXEN1
#define TXENn TXEN1
#define UCSRnC UCSR1C
#define USBSn USBS1
#define UCSZn0 UCSZ10
#define UPMn0 UPM10
#define UBRRnH UBRR1H
#define UBRRnL UBRR1L
#define UDRn UDR1
#define UDREn UDRE1
#define FEn FE1
#define USARTn_RX_vect USART1_RX_vect
#define USARTn_TX_vect USART1_TX_vect
#define USARTn_UDRE_vect USART1_UDRE_vect

#endif


// ----- ATMega specific Hardware abstraction functions -----

// calculate prescaler from baud rate and cpu clock rate at compile time.
// This is a processor specific formular from the datasheet.
// It implements rounding of ((clock / 16) / baud) - 1.
#define CalcPreScale(B) (((((F_CPU) / 8) / (B)) - 1) / 2)

const int32_t _DMX_dmxPreScale = CalcPreScale(DMXSPEED); // BAUD prescale factor for DMX speed.
const int32_t _DMX_breakPreScale = CalcPreScale(BREAKSPEED); // BAUD prescale factor for BREAK speed.


// initialize mode independent registers.
void _DMX_init()
{
  // 04.06.2012: use normal speed operation
  UCSRnA = 0;
} // _DMX_init()


/// Initialize the Hardware UART serial port registers to the required mode.
void _DMX_setMode(DMXUARTMode mode)
{
  if (mode == DMXUARTMode::OFF) {
    UCSRnB = 0;

  } else if (mode == DMXUARTMode::RONLY) {
    // assign the baud_setting to the USART Baud Rate Register
    UBRRnH = _DMX_dmxPreScale >> 8;
    UBRRnL = _DMX_dmxPreScale;
    // enable USART functions RX, TX, Interrupts
    UCSRnB = (1 << RXENn);
    // stop bits and character size
    UCSRnC = DMXREADFORMAT; // accept data packets after first stop bit

  } else if (mode == DMXUARTMode::RDATA) {
    UBRRnH = _DMX_dmxPreScale >> 8;
    UBRRnL = _DMX_dmxPreScale;
    UCSRnB = (1 << RXENn) | (1 << RXCIEn);
    UCSRnC = DMXREADFORMAT; // accept data packets after first stop bit

  } else if (mode == DMXUARTMode::TBREAK) {
    UBRRnH = _DMX_breakPreScale >> 8;
    UBRRnL = _DMX_breakPreScale;
    UCSRnB = ((1 << TXENn) | (1 << TXCIEn));
    UCSRnC = BREAKFORMAT;

  } else if (mode == DMXUARTMode::TDATA) {
    UBRRnH = _DMX_dmxPreScale >> 8;
    UBRRnL = _DMX_dmxPreScale;
    UCSRnB = ((1 << TXENn) | (1 << UDRIEn));
    UCSRnC = DMXFORMAT; // send with 2 stop bits for compatibility

  } else if (mode == DMXUARTMode::TDONE) {
    UBRRnH = _DMX_dmxPreScale >> 8;
    UBRRnL = _DMX_dmxPreScale;
    UCSRnB = ((1 << TXENn) | (1 << TXCIEn));
    UCSRnC = DMXFORMAT; // send with 2 stop bits for compatibility
  } // if
} // _DMX_setMode()


// flush all incomming data packets in the queue
void _DMX_flush()
{
  uint8_t voiddata;
  while (UCSRnA & (1 << RXCn)) {
    voiddata = UDRn; // get data
  }
}


// send the next byte after current byte was sent completely.
inline void _DMX_writeByte(uint8_t data)
{
  // putting data into buffer sends the data
  UDRn = data;
} // _DMX_writeByte


// This Interrupt Service Routine is called when a byte or frame error was received.
// In DMXController mode this interrupt is disabled and will not occur.
// In DMXReceiver mode when a byte or frame error was received
ISR(USARTn_RX_vect)
{
  uint8_t rxferr = (UCSRnA & (1 << FEn)); // get state before data!
  uint8_t rxdata = UDRn; // get data
  _DMXReceived(rxdata, rxferr);
} // ISR(USARTn_RX_vect)


// Interrupt service routines that are called when the actual byte was sent.
ISR(USARTn_TX_vect)
{
  _DMXTransmitted();
} // ISR(USARTn_TX_vect)


// this interrupt occurs after data register was emptied by handing it over to the shift register.
ISR(USARTn_UDRE_vect)
{
  _DMXTransmitted();
} // ISR(USARTn_UDRE_vect)


#endif

#endif
