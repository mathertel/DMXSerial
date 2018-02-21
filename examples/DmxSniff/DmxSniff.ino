// DmxSerial - A hardware supported interface to DMX.
// DmxSniff.ino: Sample DMX application for receiving and displaying all DMX channels
//
// Copyright (c) 2017 by Matthijs Kooijman, http://www.stderr.nl
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
//
// This example uses DMXSerial to receive DMX data, and then prints this
// data to a different serial port. This only works on boards with
// multiple UARTs (like the Mega2560), or with a virtual USB serial port
// in addition to a normal UART (like the Leonardo or Micro). Be sure to
// set the "Console" macro correctly below.
//
// Whenever the data changes, all channels are dumped through serial,
// allowing to see changes pretty much in realtime. On a dumb terminal
// (like the Arduino IDE serial console), the values are a bit hard to
// read when they're still changing. When using a proper terminal (one
// that understands ANSI escape codes), the data is nicely overwritten,
// so it should be readable even while changing. On dumb terminals,
// these escape codes can just be ignored.
//
// By default, all channels are captured and dumped, but the number of
// channels and how to display them can be changed by modifiying some
// constants below.
#include <DMXSerial.h>

// The serial port to use to dump data. Should be a different one from
// the UART used by DMXSerial. On the 32u4-based Leonardo and Micro,
// this works as-is, because "Serial" is the virtual USB serial port and
// Serial1 is the UART used by DMXSerial. On the Mega2560, you should
// let DMXSerial use the second UART, by defining DMX_USE_PORT1 in
// DMXSerial.cpp.
#define Console Serial

void setup () {
  Console.begin(0);
  DMXSerial.init(DMXReceiver);
  while(!Console) /* wait for Console to be opened */;
  // Reset terminal
  Console.print(F("\x1b\x63"));
  // Clear screen
  Console.print(F("\x1b[2J"));
}

static constexpr const uint16_t channels = 512;
static constexpr const uint16_t channels_per_line = 32;
static constexpr const uint16_t channels_per_group = 8;

void printByte(uint8_t b) {
}

void loop() {
  if (DMXSerial.dataUpdated()) {
    DMXSerial.resetUpdated();
    // For the dumb consoles (like the Arduino IDE serial console), add
    // some newlines to separate subsequent dumps
    Console.println();
    Console.println();
    // For smarter terminals, reposition the cursor at the top left
    Console.print(F("\x1b[1;1H"));

    for (uint16_t i = 0; i < channels; ++i) {
      // Channels are 1-based
      uint16_t channel = i + 1;

      if (i % channels_per_line == 0) {
        Console.println();
        // Prefix each line with the DMX channel number, adding spaces to align
	if (channel < 100) Console.write(' ');
	if (channel < 10) Console.write(' ');
	Console.print(channel);
	Console.print(F(": "));
      } else {
        // Print one space between channels, and two spaces between each group
        if (i % channels_per_group == 0)
          Console.write(' ');
        Console.write(' ');
      }

      // Print the actual channel value, padding with a zero if needed
      uint8_t value = DMXSerial.read(channel);
      if (value < 0x10) Console.write('0');
      Console.print(value, HEX);
    }
  }
}
