#include "thinkpixy.h"

// This is the main Pixy object 
Pixy pixy;
PixyCamData pixyData;

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{ 
  camFindWhale();
  if (pixyData.isDetected)
  {
    Serial.println("Detected");
  }
  else
  {
    Serial.println("Not detected");
  }
}

void camFindWhale()
{
  uint16_t blocks;
  char buf[32];
  int mid_point = 319/2 + 1;
  
 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    pixyData.isDetected = true;
    pixyData.x = pixy.blocks[0].x;
    pixyData.y = pixy.blocks[0].y;
    pixyData.w = pixy.blocks[0].width;
    pixyData.h = pixy.blocks[0].height;
    pixyData.a = pixy.blocks[0].width * pixy.blocks[0].height;
    pixyData.theta = ((pixy.blocks[0].x * 75)/319) - 37.5;
  }
  else
  {
    pixyData.isDetected = false;
  }
}
