#include <Servo.h>
#include <Arduino.h>

#define POT_PIN       A0
#define RUDDER_PIN    0
#define PROPS_PIN     0
#define TURNTABLE_PIN 5
#define LED_PIN       0
#define BUZZER_PIN    0

struct HeadingCommand {
  float turntableCommand;
  float rudderCommand;
};

// Sense functions
float getHeading();

// Think functions


// Controller functions
HeadingCommand setHeading(float headingCommand, float potPosition);
float setVel(float vel);

// Act functions
void setRudder(float rudderPcnt, Servo rudderServo);
void setProps(float propsPcnt, Servo propsServo);
void setTurntable(float turntablePcnt, Servo turntableServo);
