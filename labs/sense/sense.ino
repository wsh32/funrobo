/**
 * FunRobo Think Lab
 * 
 * Team C: 
 * David Tarazi
 * Wesley Soo-Hoo
 * Samuel Cabrera Valencia
 * Forian Michael Schwarzinger
 * Richard Gao
 * Ben Ziemann
 * 
 */

#include "sense.h"
#include <SharpIR.h>

//Setup IR sensors, pins found in sense.h
SharpIR port90IR = SharpIR(PORT_90_IR_PIN, model);
SharpIR port45IR = SharpIR(PORT_45_IR_PIN, model);
SharpIR bowIR = SharpIR(BOW_IR_PIN, model);
SharpIR starboard90IR = SharpIR(STARBOARD_45_IR_PIN, model);
SharpIR starboard45IR = SharpIR(STARBOARD_90_IR_PIN, model);

//Map - in current pass stays fixed relative to world.
//bool world_map[map_height][map_width] = {{0}};

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(500);

  //Sense
  //Get current IR distances
  Serial.println("Getting distances from IR: ");
  float dist_array[5] = {0,0,0,0,0};
  if (getIR(dist_array, 5) == -1) {
    Serial.println("Error getting distances from IR");
  }
  for (int i = 0; i < 5; i++) {
    Serial.println(dist_array[i]);
  }
  
  //Get heading from team
//  int heading = getHeading();
  
  //Remap the points relative to the heaing to be applied to the map
  //Update map

}

//Sense Functions 

int getIR(float *distances, size_t length) {
  /*
   * Get distances from IR sensor suite
   * 
   * Parameters:
   * - distances: float array to hold distances
   * - length: size of the array
   */
  if (length != 5) {
    return -1;
  }
  
  distances[0] = port90IR.distance();
  distances[1] = port45IR.distance();
  distances[2] = port90IR.distance();
  distances[3] = starboard45IR.distance();
  distances[4] = starboard90IR.distance();
  return 0;
}

//Think Functions

//Act Functions

//Controller Functions
