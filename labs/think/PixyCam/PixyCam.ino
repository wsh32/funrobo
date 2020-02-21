//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP port.  For more information go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
//
// It prints the detected blocks once per second because printing all of the 
// blocks for all 50 frames per second would overwhelm the Arduino's serial port.
//
   
#include <SPI.h>  
#include <Pixy.h>

// This is the main Pixy object 
Pixy pixy;
int whaleData[5];

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{ 
  whaleLock();  
}

void whaleLock()
{
  static int i = 0;
  uint16_t blocks;
  char buf[32]; 
 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++; 
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%25==0){
      whaleData[0] = pixy.blocks[0].x;
      whaleData[1] = pixy.blocks[0].y;
      whaleData[2] = pixy.blocks[0].width;
      whaleData[3] = pixy.blocks[0].height;
      whaleData[4] = pixy.blocks[0].width * pixy.blocks[0].height;
       
      sprintf(buf, "x: %d\ny: %d\nwidth: %d\nheight: %d\narea: %d\n\n", whaleData[0], whaleData[1], whaleData[2], whaleData[3], whaleData[4]);
      Serial.print(buf);
    }
  }
}
