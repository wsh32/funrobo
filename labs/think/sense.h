#include <SharpIR.h>
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
#define dist_cor_coIR .24765
//Distance in meters from center of array to sensors
#define dist_coIR_IR .05715

//Map width and height and resolution (cm)
#define map_width = 250
#define map_height = 250
#define map_resolution 1
 
// Sense functions
float[] solve_IR_dist
float[] update_map(float[] ir_readings;)

//Think functions

//Act functions

//Controller functions


