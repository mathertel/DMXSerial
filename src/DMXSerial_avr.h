// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial_avr.h: Hardware specific functions for AVR processors like ATmega168p used in Aurduino UNO.
// 
// Copyright (c) 2011-2020 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// - - - - -


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


// formats for serial transmission

#define SERIAL_8N1 ((0 << USBSn) | (0 << UPMn0) | (3 << UCSZn0))
#define SERIAL_8N2 ((1 << USBSn) | (0 << UPMn0) | (3 << UCSZn0))
#define SERIAL_8E1 ((0 << USBSn) | (2 << UPMn0) | (3 << UCSZn0))
#define SERIAL_8E2 ((1 << USBSn) | (2 << UPMn0) | (3 << UCSZn0))


// ----- ATMega specific Hardware abstraction functions -----

// calculate prescaler from baud rate and cpu clock rate at compile time
// nb implements rounding of ((clock / 16) / baud) - 1 per atmega datasheet
#define CalcPreScale(B) (((((F_CPU) / 8) / (B)) - 1) / 2)

void _DMX_initUART()
{
  // Serial.println("Initialized.");
}

/// Initialize the Hardware UART serial port to the required mode.
void _DMX_setMode(DMXUARTMode mode)
{
  uint16_t baud_setting;
  uint8_t flags;
  uint8_t format;

  if (mode == DMXUARTMode::OFF) {
    UCSRnB = 0;
    return;

  } else if (mode == DMXUARTMode::RONLY) {
    baud_setting = CalcPreScale(DMXSPEED);
    flags = (1 << RXENn);
    format = DMXFORMAT;

  } else if (mode == DMXUARTMode::RDATA) {
    baud_setting = CalcPreScale(DMXSPEED);
    flags = (1 << RXENn) | (1 << RXCIEn);
    format = DMXFORMAT;

  } else if (mode == DMXUARTMode::TBREAK) {
    baud_setting = CalcPreScale(BREAKSPEED);
    flags = ((1 << TXENn) | (1 << TXCIEn));
    format = BREAKFORMAT;

  } else if (mode == DMXUARTMode::TDATA) {
    baud_setting = CalcPreScale(DMXSPEED);
    flags = ((1 << TXENn) | (1 << UDRIEn));
    format = DMXFORMAT;

  } else if (mode == DMXUARTMode::TDONE) {
    baud_setting = CalcPreScale(DMXSPEED);
    flags = ((1 << TXENn) | (1 << TXCIEn));
    format = DMXFORMAT;
  } // if

  // now set the registers:

  // 04.06.2012: use normal speed operation
  UCSRnA = 0;

  // assign the baud_setting to the USART Baud Rate Register
  UBRRnH = baud_setting >> 8;
  UBRRnL = baud_setting;

  // enable USART functions RX, TX, Interrupts
  UCSRnB = flags;

  // stop bits and character size
  UCSRnC = format;
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
  uint8_t USARTstate = UCSRnA; // get state before data!
  uint8_t DmxByte = UDRn; // get data
  _DMXReceived(USARTstate & (1 << FEn), DmxByte);
} // ISR(USARTn_RX_vect)
