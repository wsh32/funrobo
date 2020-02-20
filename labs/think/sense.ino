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
SharpIR port90IR = SharpIR(PORT_90_IR_PIN, model)
SharpIR port45IR = SharpIR(PORT_45_IR_PIN, model)
SharpIR bowIR = SharpIR(BOW_IR_PIN, model)
SharpIR starboard90IR = SharpIR(STARBOARD_45_IR_PIN, model)
SharpIR starboard45IR = SharpIR(STARBOARD_90_IR_PIN, model)

//Map - in current pass stays fixed relative to world.
bool world_map[map_height][map_width] = {{0}};

void setup(){
    Serial.begin(9600);
}

void loop(){
    delay(500);

    //Sense
    //Get current IR distances
    float dist_array[5] = getIR();
    //Get heading from team
    int heading = getHeading();
    //Remap the points relative to the heaing to be applied to the map
    //Update map

}

//Sense Functions 

//Return distances from each IR
float[] getIR(){
    float distances[5] = {0,0,0,0,0};
    distances[0] = port90IR.distance();
    distances[1] = port45IR.distance();
    distances[2] = port90IR.distance();
    distances[3] = starboard45IR.distance();
    distances[4] = starboard90IR.distance();
    return distances;
}

//Think Functions

//Act Functions

//Controller Functions