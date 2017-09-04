// neopixel.h


/*
  The Neopixel driving routines are taken from the article and sketch from bigjosh
  http://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
  where the interrupt cli() and sei() are included in the sendBit function.
  At the sources from his github this is not the case but it's important for the usage with DMXSerial library.
  (see https://github.com/bigjosh/SimpleNeoPixelDemo )

  These routines fit very good to the DMXSerial implementation because they switch on and off the
  Interrupt

  On DMX usual channels are used in the red then green then blue order.
  Neopixel wants colors in green then red then blue order so the 2 channels are switched.
*/

// ----- global defines from josh: -----

// These values are for the pin that connects to the Data Input pin on the LED strip. They correspond to...

#define PIXEL_PORT  PORTB  // Port of the pin the pixels are connected to
#define PIXEL_DDR   DDRB   // Port of the pin the pixels are connected to
#define PIXEL_BIT   4      // Bit of the pin the pixels are connected to

// This re3sults in the following Arduino Pins:
// Arduino Yun:     Digital Pin 8
// DueMilinove/UNO: Digital Pin 12
// Arduino Mega     PWM Pin 4

// You'll need to look up the port/bit combination for other boards.
// Note that you could also include the DigitalWriteFast header file to not need to to this lookup.

// These are the timing constraints taken mostly from the WS2812 datasheets
// These are chosen to be conservative and avoid problems rather than for maximum throughput

#define T1H  900    // Width of a 1 bit in ns
#define T1L  600    // Width of a 1 bit in ns

#define T0H  400    // Width of a 0 bit in ns
#define T0L  900    // Width of a 0 bit in ns

#define RES 6000    // Width of the low gap between bits to cause a frame to latch

// Here are some convience defines for using nanoseconds specs to generate actual CPU delays

#define NS_PER_SEC (1000000000L)          // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives

#define CYCLES_PER_SEC (F_CPU)

#define NS_PER_CYCLE ( NS_PER_SEC / CYCLES_PER_SEC )

#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )

#define DELAY_CYCLES(n) ( ((n)>0) ? __builtin_avr_delay_cycles( n ) :  __builtin_avr_delay_cycles( 0 ) )  // Make sure we never have a delay less than zero

// Low level function with mixed in assembler code.

// Actually send a bit to the string. We turn off optimizations to make sure the compile does
// not reorder things and make it so the delay happens in the wrong place.
inline void sendBit( bool bitVal )
{
  if (bitVal) {        // 0 bit
    asm volatile (
      "sbi %[port], %[bit] \n\t"        // Set the output bit
      ".rept %[onCycles] \n\t"                                // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      "cbi %[port], %[bit] \n\t"                              // Clear the output bit
      ".rept %[offCycles] \n\t"                               // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      ::
      [port]    "I" (_SFR_IO_ADDR(PIXEL_PORT)),
      [bit]   "I" (PIXEL_BIT),
      [onCycles]  "I" (NS_TO_CYCLES(T1H) - 2),    // 1-bit width less overhead  for the actual bit setting, note that this delay could be longer and everything would still work
      [offCycles]   "I" (NS_TO_CYCLES(T1L) - 2)     // Minimum interbit delay. Note that we probably don't need this at all since the loop overhead will be enough, but here for correctness
    );
  
  } else {          // 1 bit
    // **************************************************************************
    // This line is really the only tight goldilocks timing in the whole program!
    // **************************************************************************
    asm volatile (
      "sbi %[port], %[bit] \n\t"        // Set the output bit
      ".rept %[onCycles] \n\t"        // Now timing actually matters. The 0-bit must be long enough to be detected but not too long or it will be a 1-bit
      "nop \n\t"                                              // Execute NOPs to delay exactly the specified number of cycles
      ".endr \n\t"
      "cbi %[port], %[bit] \n\t"                              // Clear the output bit
      ".rept %[offCycles] \n\t"                               // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      ::
      [port]    "I" (_SFR_IO_ADDR(PIXEL_PORT)),
      [bit]   "I" (PIXEL_BIT),
      [onCycles]  "I" (NS_TO_CYCLES(T0H) - 2),
      [offCycles] "I" (NS_TO_CYCLES(T0L) - 2)
    );
  } // if
  
  // Note that the inter-bit gap can be as long as you want as long as it doesn't exceed the 5us reset timeout (which is A long time)
  // Here I have been generous and not tried to squeeze the gap tight but instead erred on the side of lots of extra time.
  // This has thenice side effect of avoid glitches on very long strings becuase
} // sendBit()

// Neopixel wants bit in highest-to-lowest order
// so send highest bit (bit #7 in an 8-bit byte since they start at 0)
inline void sendByte(uint8_t byte)
{
  for (uint8_t bit = 0; bit < 8; bit++) {
    sendBit(byte & 0x80);
    byte <<=
        1; // and then shift left so bit 6 moves into 7, 5 moves into 6, etc
  } // for
} // sendByte()

/*
  The following three functions are the public API:
  ledSetup() - set up the pin that is connected to the string. Call once at the begining of the program.
  sendPixel( r g , b ) - send a single pixel to the string. Call this once for each pixel in a frame.
  show() - show the recently sent pixel on the LEDs . Call once per frame.
*/

// Set the specified pin up as digital out

void sendPixel(uint8_t r, uint8_t g, uint8_t b)  {
  sendByte(g);          // Neopixel wants colors in green then red then blue order
  sendByte(r);
  sendByte(b);
} // sendPixel


// ----- defines and routines from josh - End -----

void setupNeopixel() {
  bitSet( PIXEL_DDR , PIXEL_BIT );
} // setupNeopixel()


// read data from the DMX buffer (RGB) and send it to the neopixels...
void updateNeopixel(uint8_t *ptr, uint8_t pixels) {
  uint8_t  r, g, b;

  // no interrupt is welcome.
  cli();

  for (int p = 0; p < pixels; p++ ) {
    r = *ptr++;
    g = *ptr++;
    b = *ptr++;
    // send to Neopixels
    // sendPixel(r, g , b);
    sendPixel(r >> 2, g >> 2, b >> 2);
  } // for

  // interrupt may come.
  sei();

  // Just wait long enough without sending any bots to cause the pixels to latch and display the last sent frame
  _delay_us((RES / 1000UL) + 1);
} // updateNeopixel()

// End
