# DMXSerial

[![arduino-library-badge](https://www.ardu-badge.com/badge/DMXSerial.svg?)](https://www.ardu-badge.com/DMXSerial)
[![GitLicense](https://gitlicense.com/badge/mathertel/DMXSerial)](https://gitlicense.com/license/mathertel/DMXSerial)


This is a library for sending and receiving DMX codes using the Arduino platform.
You can implement DMX devices and DMX controllers with this library.

The DMX communication implemented by the DMXSerial library relies completely on the hardware support of a builtin USART / Serial port interface
by using interrupts that handle all I/O transfer in the background.

There is a full 512 byte buffer allocated to support all possible DMX values of a line at the same time. 

Since Version 1.5.0 the original ATmega168 based implementation was refactored and enhanced
to support also other processor architectures like the ATMEGA4809.


## Supported Boards and processors

The supported processors and Arduino Boards are:
* Arduino UNO, Ethernet, Fio, Nano and Arduino Pro, Arduino Pro Mini (ATmega328 or ATmega168)
* Arduino Mega2560 (ATmega2560)
* Arduino Leonardo (Atmega32U4)
* Arduino nano Every (ATMEGA4809)

Other compatible boards my work as well.

You can find more detail on this library at http://www.mathertel.de/Arduino/DMXSerial.aspx.


## Supported Shields

A suitable hardware is the Arduino platform plus a shield for the DMX physical protocol implementation.
You can find such a shield at: http://www.mathertel.de/Arduino/DMXShield.aspx.

Without or some modification this library can also used with other DMX shields
that use the Serial port for connecting to the DMX bus.


## License

Copyright (c) 2005-2020 by Matthias Hertel,  http://www.mathertel.de/

The detailed Software License Agreement can be found at: http://www.mathertel.de/License.aspx

