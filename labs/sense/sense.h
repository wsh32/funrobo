#include <Arduino.h>
#include <math.h>

//Pins
#define PORT_90_IR_PIN      A1
#define PORT_45_IR_PIN       A2
#define BOW_IR_PIN           A3
#define STARBOARD_45_IR_PIN  A4
#define STARBOARD_90_IR_PIN  A5

//Model of the Sharp IR sensor - 2Y0A21
#define model 1080

//Distance in meters from center of rotation to center of sensor array
#define D1 .24765
//Distance in meters from center of array to sensors
#define D2 .05715

//Define angles of IR sensors 
#define PORT_90_ANG -90
#define PORT_45_ANG -45
#define BOW_ANG 0
#define STARBOARD_45_ANG 45
#define STARBOARD_90_ANG 90

//Map width and height and resolution (cm)
#define map_width = 250
#define map_height = 250
#define map_resolution 1
 
// Sense functions
int solve_IR(int irAngle, float *rotAngle, float *distance);
int getIR(float *distances, size_t length);
float degToRad(int deg);
float radToDeg(float rad);

//Think functions

//Act functions

//Controller functions
