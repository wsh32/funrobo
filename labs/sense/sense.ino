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
  delay(2000);

  //Sense
  //Get current IR distances
  Serial.println("*****Getting distances from IR*****");
  float dist_array[5] = {0,0,0,0,0};
  if (getIR(dist_array, 5) == -1) {
    Serial.println("Error getting distances from IR");
  }

  // Testing print out distances
  Serial.print("Port 90: ");
  Serial.println(dist_array[0]);
  Serial.print("Port 45: ");
  Serial.println(dist_array[1]);
  Serial.print("Bow: ");
  Serial.println(dist_array[2]);
  Serial.print("Starboard 45: ");
  Serial.println(dist_array[3]);
  Serial.print("Port 90: ");
  Serial.println(dist_array[4]);
  
  Serial.println("*****Solving Distance and Angle from COR*******");
  float angle_cor;
  float d_cor;
  solve_IR(PORT_45_ANG, dist_array[1], &angle_cor, &d_cor);
  Serial.print("Distance (meters): ");
  Serial.println(d_cor);
  Serial.print("Angle (deg): ");
  Serial.println(angle_cor);
  Serial.println();
  
    
  //Get heading from team
  //int heading = getHeading();
  
  //Remap the points relative to the heaing to be applied to the map
  //Update map

}

//Sense Functions 

int getIR(float *distances, size_t length) {
  /*
   * Get distances from IR sensor suite in meters
   * 
   * Parameters:
   * - distances: float array to hold distances
   * - length: size of the array
   */
  if (length != 5) {
    return -1;
  }
  
  distances[0] = port90IR.distance() / 100.0;
  distances[1] = port45IR.distance() / 100.0;
  distances[2] = bowIR.distance() / 100.0;
  distances[3] = starboard45IR.distance() / 100.0;
  distances[4] = starboard90IR.distance() / 100.0;
  return 0;
}

int solve_IR(float irAngle, float irDistance, float *rotAngle, float *distance) {
  /*
   * Determine angle and distance from center of rotation for a detected object
   * 
   * Coordinates are set such that N: 0deg, W: -90deg, E: 90deg
   * 
   * Parameters:
   * - irAngle: angle of the IR sensor with respect to heading
   * - length: size of the array
   */
  // Deterimine what side based on argument angle
  int side = 0;
  if (irAngle < 0) {
    side = -1; // Port
  }
  else {
    side = 1; // Starboard
  }
  
  float angle = degToRad(90 + (90 - abs(irAngle)));
  *distance = pow(irDistance + D2, 2) + pow(D1, 2) - 2 * D1 * (D2 + irDistance) * cos(angle); // Law of cosines
  // TODO: Debug why getting closer increases the angle instead of decreases
  *rotAngle = asin((sin(angle) * (irDistance + D2)) / (*distance)); // Law of sines
  *rotAngle = radToDeg(*rotAngle * side);
}

float degToRad(float deg) { return deg * M_PI / 180; }
float radToDeg(float rad) { return rad * 180 / M_PI; }

//Think Functions

//Act Functions

//Controller Functions
