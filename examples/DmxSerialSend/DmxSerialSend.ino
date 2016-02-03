// - - - - -
// DmxSerial - A hardware supported interface to DMX.
// DmxSerialSend.ino: Sample DMX application for sending 3 DMX values.
// There colors in the 3 lists (RedList, GreenList and BlueList) are placed into the DMX buffer with a slow fade.
// DMXSerial works in the background and constantly sends the actual values over the DMX interface.
// The actual values are also available on the built in PWM ports:
// address 1 (red) -> also available on PWM Port 9
// address 2 (green) -> also available on PWM Port 6
// address 3 (blue) -> also available on PWM Port 5
// 
// Copyright (c) 2011-2015 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// 
// Documentation and samples are available at http://www.mathertel.de/Arduino
// The repository is on github : https://github.com/mathertel/DMXSerial
// The repository on github is made available in the Arduino Library Manager.
//
// 25.07.2011 creation of the DmxSerial library.
// 10.09.2011 fully control the serial hardware register
//            without using the Arduino Serial (HardwareSerial) class to avoid ISR implementation conflicts.
// 01.12.2011 include file and extension changed to work with the Arduino 1.0 environment
// - - - - -

#include <DMXSerial.h>

// Constants for demo program

const int RedPin =    9;  // PWM output pin for Red Light.
const int GreenPin =  6;  // PWM output pin for Green Light.
const int BluePin =   5;  // PWM output pin for Blue Light.

// The color fading pattern

int RedList[]   = {255, 128,   0,   0,   0, 128};
int GreenList[] = {  0, 128, 255, 128,   0,   0};
int BlueList[]  = {  0,   0,   0, 128, 255, 128};

int RedLevel, GreenLevel, BlueLevel;

int RedNow = 0;
int GreenNow = 0;
int BlueNow = 0;

int state = 0;

void setup() {
  DMXSerial.init(DMXController);

  pinMode(RedPin,   OUTPUT); // sets the digital pin as output
  pinMode(GreenPin, OUTPUT);
  pinMode(BluePin,  OUTPUT);
} // setup


// loop through the rainbow colors 
void loop() {
  RedLevel = RedList[state];
  GreenLevel = GreenList[state];
  BlueLevel = BlueList[state];
  
  if ((RedLevel == RedNow) && (GreenLevel == GreenNow) && (BlueLevel == BlueNow)) {
    state += 1;
    if (state == 6)
      state = 0;

  } else {
    if (RedNow < RedLevel)  RedNow++; 
    if (RedNow > RedLevel)  RedNow--; 
    DMXSerial.write(1, RedNow);
    analogWrite(RedPin,   RedNow); 

    if (GreenNow < GreenLevel)  GreenNow++; 
    if (GreenNow > GreenLevel)  GreenNow--; 
    DMXSerial.write(2, GreenNow);
    analogWrite(GreenPin, GreenNow); 

    if (BlueNow < BlueLevel)  BlueNow++; 
    if (BlueNow > BlueLevel)  BlueNow--; 
    DMXSerial.write(3, BlueNow);
    analogWrite(BluePin,  BlueNow); 
  } // if

  delayMicroseconds(2000); // wait a little bit
} // loop
