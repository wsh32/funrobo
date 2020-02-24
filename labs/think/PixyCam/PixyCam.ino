#include "thinkpixy.h"

// This is the main Pixy object 
Pixy pixy;
PixyCamData data;

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
  int mid_point = 319/2 + 1;
  
 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++; 
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%25==0){
      data.x = pixy.blocks[0].x;
      data.y = pixy.blocks[0].y;
      data.w = pixy.blocks[0].width;
      data.h = pixy.blocks[0].height;
      data.a = pixy.blocks[0].width * pixy.blocks[0].height;
       
      sprintf(buf, "x: %d\ny: %d\nwidth: %d\nheight: %d\narea: %d\n\n", data.x, data.y, data.w, data.h, data.a);
      Serial.print(buf);
    }
  }
}
