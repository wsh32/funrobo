#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include <Pixy.h>

//Pins
#define PORT_90_IR_PIN      A1
#define PORT_45_IR_PIN       A2
#define BOW_IR_PIN           A3
#define STARBOARD_45_IR_PIN  A4
#define STARBOARD_90_IR_PIN  A5
#define POT_PIN       A0
#define RUDDER_PIN    11
#define PROPS_PIN     9
#define TURNTABLE_PIN 5
#define LED_PIN       0
#define BUZZER_PIN    0

//Model of the Sharp IR sensor - 2Y0A21
#define MODEL 1080
//Distance in meters from center of rotation to center of sensor array
#define D1 .24765
//Distance in meters from center of array to sensors
#define D2 .05715

//Map width and height and resolution (cm)
#define map_width  250
#define map_height 250
#define map_resolution 1

// Filter gain for heading potentiometer
#define FILTER_GAIN 0.125

// Heading command data
#define MIN_HEADING   -90
#define MAX_HEADING   90
#define STEP_HEADING  5

// Arbiter weights
#define AVOID_WEIGHT 1
#define HUNT_WEIGHT  1

// Sensor data structs
struct RawSharpIRData {
  float port90Dist, port45Dist, bowDist, starboard45Dist, starboard90Dist;
};

struct ProcessedSharpIRData {
  // rotAngle in degrees
  float distance, rotAngle;
};

// PixyCam data structs
struct PixyCamData {
  boolean isDetected;
  float x, y, w, h, a, theta;
};

// Command Structs
struct Command {
  float heading;
  int vel;
};

struct HeadingCommand {
  float turntableCommand;
  int rudderCommand;
};

// Sense functions
ProcessedSharpIRData solveIR(int irAngle, float irDistance);
RawSharpIRData getIR();
float getIRDist(int pin);
float degToRad(int deg);
float radToDeg(float rad);
float getHeading(float lastHeading);
PixyCamData getPixyCam();

// Think functions
// Behaviors
void avoid(RawSharpIRData irRawData, float heading, int headingWeightsAvoid[]);
void hunt(float heading, boolean dir, int headingWeightsHunt[], int velWeightHunt);
void follow(float heading, int headingWeightsHunt[], int velWeightHunt);

// Arbiter
Command arbitrate(int headingAvoid[], int velAvoid, int headingHunt[], int velHunt);

// Controller functions
HeadingCommand setHeading(float headingCommand, float potPosition);
int setVel(int vel);

// Act functions
void setRudder(int rudderPcnt, Servo rudderServo);
void setProps(int propsPcnt, Servo propsServo);
void setTurntable(float turntablePcnt, Servo turntableServo);
