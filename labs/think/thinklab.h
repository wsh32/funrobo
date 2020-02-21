#include <Servo.h>
#include <Arduino.h>

#define POT_PIN       A0
#define RUDDER_PIN    11
#define PROPS_PIN     9
#define TURNTABLE_PIN 5
#define LED_PIN       0
#define BUZZER_PIN    0

#define FILTER_GAIN 0.125

struct HeadingCommand {
  float turntableCommand;
  int rudderCommand;
};

// Sense functions
float getHeading(float lastHeading);

// Think functions


// Controller functions
HeadingCommand setHeading(float headingCommand, float potPosition);
int setVel(int vel);

// Act functions
void setRudder(int rudderPcnt, Servo rudderServo);
void setProps(int propsPcnt, Servo propsServo);
void setTurntable(float turntablePcnt, Servo turntableServo);
