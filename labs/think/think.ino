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

#include "thinklab.h"
#include <Servo.h>

// Pin definitions in thinklab.h file

Servo rudderServo;
Servo propsServo;
Servo turntableServo;

// P control for turntable
int kP_headingControl = 1.1;
float heading = 0;

void setup() {
  Serial.begin(9600);
  // pinMode(POT_PIN, INPUT);
  rudderServo.attach(RUDDER_PIN);
  propsServo.attach(PROPS_PIN);
  turntableServo.attach(TURNTABLE_PIN);
}

void loop() {
  // SENSE
  // Get current heading
  heading = getHeading(heading);

  // THINK
  // TODO get commands from arbiter
  float headingCommandDegs = 0;
  int velCommand = 10;

  // CONTROLLER
  HeadingCommand headingOutput = setHeading(headingCommandDegs, heading);
  float turntableOutput = headingOutput.turntableCommand;
  int rudderOutput = headingOutput.rudderCommand;
  int propsOutput = setVel(velCommand);

  // ACT
  setRudder(rudderOutput, rudderServo);
  setProps(propsOutput, propsServo);
  setTurntable(turntableOutput, turntableServo);
}

// SENSE FUNCTIONS
float getHeading(float lastHeading) {
  int rawPotReading = analogRead(POT_PIN);
  float headingAngle = map(rawPotReading, 200, 420, -90, 90); // Map raw units to degrees
  
  // filter the potentiometer angle so it is smooth
  headingAngle = FILTER_GAIN * headingAngle + (1-FILTER_GAIN)*lastHeading;
  Serial.println(headingAngle);
  return headingAngle;
}

// CONTROLLER FUNCTIONS
HeadingCommand setHeading(float headingCommand, float potPosition) {
  /**
   * Closed loop position control for heading
   * Sets rudder to fixed angle
   * P controller for turntable
   * 
   * Parameters:
   * - headingCommand: Heading command (in degrees)
   * - potPosition: position feedback (in degrees)
   */
  // Set rudder angle
  // Move turntable to position
  // PID loop, kP_headingControl is defined in thinklab.h
  float turntablePower = kP_headingControl * (headingCommand - potPosition);
  turntablePower = max(min(turntablePower, 100), -100);  // Threshold to -100 to 100
  
  HeadingCommand command;
  command.rudderCommand = headingCommand - potPosition;
  command.turntableCommand = turntablePower;

  return command;
}

int setVel(int vel) {
  /**
   * Velocity controller
   * Open loop control of propeller speed and direction
   * Passes through velocity as a int
   * 
   * Parameters:
   * - vel: Velocity command
   */

  return vel;
}

// ACT FUNCTIONS
void setRudder(int rudderPcnt, Servo rudderServo) {
  /**
   * Set rudder servo
   * 
   * Parameters:
   * - rudderPcnt: Value from -100 to 100
   * - rudderServo: Initialized Servo class
   */
  int rudderAng = map(rudderPcnt, -180, 180, 0, 185);
  // threshold so we aren't burning out motors
  rudderAng = max(min(rudderAng, 120), 55);
  rudderServo.write(rudderAng);
}

void setProps(int propsPcnt, Servo propsServo) {
  /**
   * Set propeller servo
   * 
   * Parameters:
   * - propsPcnt: Value from -100 to 100
   * - propsServo: Initialized Servo class
   */
  // map raw velocity command to motor power
  Serial.println(propsPcnt);
  int propPower = map(propsPcnt, 0, 100, 0, 255);
  Serial.println(propPower);
  propsServo.write(propPower);
}

void setTurntable(float turntablePcnt, Servo turntableServo) {
  /**
   * Set turntable motor value
   * 
   * Parameters:
   * - turntablePcnt: Value from -1 to 1
   * - turntableServo: Initialized Servo class
   */
  int turntablePower = map(turntablePcnt, -100, 100, 0, 185);
  turntableServo.write(turntablePower);
}
