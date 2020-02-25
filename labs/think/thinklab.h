#include <Servo.h>
#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>

#define POT_PIN       A0
#define RUDDER_PIN    11
#define PROPS_PIN     9
#define TURNTABLE_PIN 5
#define LED_PIN       0
#define BUZZER_PIN    0

#define FILTER_GAIN 0.125

// Sensor data structs
struct SharpIRData {
  int dist1, dist2, dist3, dist4, dist5;
  float angle1, angle2, angle3, angle4, angle5;
};

// PixyCam data structs
struct PixyCamData {
  boolean isDetected;
  float x, y, w, h, a, theta;
};

// Command Structs
struct HeadingCommand {
  float turntableCommand;
  int rudderCommand;
};

// Sense functions
float getHeading(float lastHeading);
SharpIRData getSharpIR();
PixyCamData getPixyCam();

// Think functions


// Controller functions
HeadingCommand setHeading(float headingCommand, float potPosition);
int setVel(int vel);

// Act functions
void setRudder(int rudderPcnt, Servo rudderServo);
void setProps(int propsPcnt, Servo propsServo);
void setTurntable(float turntablePcnt, Servo turntableServo);
