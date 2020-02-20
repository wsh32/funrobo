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

// TODO tune
float kP_headingControl = 0.5;

void setup() {
  Serial.begin(9600);
  pinMode(POT_PIN, INPUT);
  rudderServo.attach(RUDDER_PIN);
  propsServo.attach(PROPS_PIN);
  turntableServo.attach(TURNTABLE_PIN);
}

void loop() {
  // SENSE
  // Get current heading
  float heading = getHeading();

  // THINK
  // TODO get commands from arbiter
  float headingCommandDegs = 0;
  float velCommand = 0;

  // CONTROLLER
  HeadingCommand headingOutput = setHeading(headingCommandDegs, heading);
  float turntableOutput = headingOutput.turntableCommand;
  float rudderOutput = headingOutput.rudderCommand;

  float propsOutput = setVel(velCommand);

  // ACT
//  setRudder(rudderOutput, rudderServo);
  setProps(propsOutput, propsServo);
//  setTurntable(turntableOutput, turntableServo);
}

// SENSE FUNCTIONS
float getHeading() {
  int rawPotReading = analogRead(POT_PIN);
  // TODO Calibrate these values
  return map(rawPotReading, 0, 1024, -180, 180); // Map raw units to degrees
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
  // TODO mapping for rudder from heading
  float rudderAngle = map(headingCommand, -180, 180, -1, 1);  // map from degrees to rudder units (-1 to 1)

  // Move turntable to position
  // PID loop, kP_headingControl is defined in thinklab.h
  float turntablePower = kP_headingControl * (headingCommand - potPosition);
  turntablePower = max(min(turntablePower, 1), -1);  // Threshold to -1 to 1

  HeadingCommand command;
  command.rudderCommand = rudderAngle;
  command.turntableCommand = turntablePower;

  return command;
}

float setVel(float vel) {
  /**
   * Velocity controller
   * Open loop control of propeller speed and direction
   * 
   * Parameters:
   * - vel: Velocity command
   */

  float propPower = map(vel, -10, 10, -1, 1);  // map from velocity (TODO get actual values) to prop units (-1 to 1)
  return propPower;
}

// ACT FUNCTIONS
void setRudder(float rudderPcnt, Servo rudderServo) {
  /**
   * Set rudder servo
   * 
   * Parameters:
   * - rudderPcnt: Value from -1 to 1
   * - rudderServo: Initialized Servo class
   */
   rudderServo.write(map(rudderPcnt, -1, 1, 0, 255));
}

void setProps(float propsPcnt, Servo propsServo) {
  /**
   * Set propeller servo
   * 
   * Parameters:
   * - propsPcnt: Value from -1 to 1
   * - propsServo: Initialized Servo class
   */
   int propPower = map(propsPcnt, -1, 1, 0, 255);
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
   turntableServo.write(map(turntablePcnt, -1, 1, 0, 186));
}
