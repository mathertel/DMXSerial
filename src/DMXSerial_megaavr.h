// DMX HAL for MEGAAVR Architecture


// ----- MegAVR specific Hardware abstraction functions -----

#include "Arduino.h"
#include "DMXSerial.h"
#include "avr/io.h"


int32_t _dmx_divider;
int32_t _break_divider;

#define DBG(k, v)  \
  Serial.print(k); \
  Serial.println(v);


/// Initialize the Hardware UART serial port.
void _DMX_initUART()
{

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  int32_t baud;

  int8_t oscErr = SIGROW.OSC16ERR5V;
  DBG("SIGROW.OSC16ERR5V:", oscErr);

  // calculate DMX speed Divider
  baud = (DMXSPEED * (1024 + oscErr)) / 1024;
  DBG("DMX Baud Rate adjusted:", baud);

  _dmx_divider = (64 * F_CPU) / (16 * baud);
  DBG("DMX Divider:", _dmx_divider);

  // calculate BREAK speed Divider
  baud = (BREAKSPEED * (1024 + oscErr)) / 1024;
  DBG("BREAK Baud Rate adjusted:", baud);

  _break_divider = (64 * F_CPU) / (16 * baud);
  DBG("BREAK Divider:", _break_divider);

  // disable interrupts during initialization
  uint8_t oldSREG = SREG;
  cli();

  // Setup port mux
  PORTMUX.USARTROUTEA |= PORTMUX_USART1_ALT1_gc;

  // Disable CLK2X
  (USART1).CTRLB &= (~USART_RXMODE_CLK2X_gc);
  (USART1).CTRLB |= USART_RXMODE_NORMAL_gc;

  //Set up the rx & tx pins
  pinMode(PIN_WIRE_HWSERIAL1_RX, INPUT_PULLUP);
  pinMode(PIN_WIRE_HWSERIAL1_TX, OUTPUT);
  // digitalWrite(PIN_WIRE_HWSERIAL1_TX, HIGH);

  // enable interrupts again, restore SREG content
  SREG = oldSREG;

  Serial.println("Initialized.");
} // _DMX_initUART()


/// Initialize the Hardware UART serial port to the required mode.
void _DMX_setMode(DMXUARTMode mode)
{
  uint16_t baud_setting;
  uint8_t flags;
  uint8_t format;

  uint8_t oldSREG = SREG;
  cli();

  if (mode == DMXUARTMode::OFF) {
    // Disable transmitter and receiver
    (USART1).CTRLB |= USART_RXEN_bm; // (USART_RXEN_bm | USART_TXEN_bm);

  } else if (mode == DMXUARTMode::RONLY) {
    // baud_setting = CalcPreScale(DMXSPEED);
    // flags = (1 << RXENn);
    // format = DMXFORMAT;

  } else if (mode == DMXUARTMode::RDATA) {
    // disable interrupts during initialization

    // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    (USART1).BAUD = (int16_t)_dmx_divider;

    // Set USART mode of operation
    (USART1).CTRLC = SERIAL_8N1;

    // Enable transmitter and receiver
    (USART1).CTRLB |= USART_RXEN_bm; // (USART_RXEN_bm | USART_TXEN_bm);

    // enable interrupts
    (USART1).CTRLA |= USART_RXCIE_bm;

  } else if (mode == DMXUARTMode::TBREAK) {
    // baud_setting = CalcPreScale(BREAKSPEED);
    // flags = ((1 << TXENn) | (1 << TXCIEn));
    // format = BREAKFORMAT;

  } else if (mode == DMXUARTMode::TDATA) {
    // baud_setting = CalcPreScale(DMXSPEED);
    // flags = ((1 << TXENn) | (1 << UDRIEn));
    // format = DMXFORMAT;

  } else if (mode == DMXUARTMode::TDONE) {
    // baud_setting = CalcPreScale(DMXSPEED);
    // flags = ((1 << TXENn) | (1 << TXCIEn));
    // format = DMXFORMAT;
  } // if

  // now set the registers:

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
  // putting data into buffer sends the data
  // UDRn = data;
} // _DMX_writeByte


// This Interrupt Service Routine is called when a byte or frame error was received.
// In DMXController mode this interrupt is disabled and will not occur.
// In DMXReceiver mode when a byte or frame error was received
ISR(USART1_RXC_vect)
{
  register8_t rxferr = (USART1).RXDATAH & USART_FERR_bm;
  register8_t rxdata = (USART1).RXDATAL;
  _DMXReceived(rxferr, rxdata);
} // ISR(USART1_RXC_vect)
