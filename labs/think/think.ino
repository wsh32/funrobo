/**
 * FunRobo Think Lab
 * 
 * Team C: 
 * David Tarazi
 * Wesley Soo-Hoo
 * Samuel Cabrera Valencia
 * Florian Schwarzinger
 * Richard Gao
 * Ben Ziemann
 * 
 */

#include "thinklab.h"
#include <Servo.h>
#include <SharpIR.h>

SharpIR port90IR(PORT_90_IR_PIN, MODEL);
SharpIR port45IR(PORT_45_IR_PIN, MODEL);
SharpIR bowIR(BOW_IR_PIN, MODEL);
SharpIR starboard45IR(STARBOARD_45_IR_PIN, MODEL);
SharpIR starboard90IR(STARBOARD_90_IR_PIN, MODEL);

// Pin definitions in thinklab.h file
Servo rudderServo;
Servo propsServo;
Servo turntableServo;
Pixy pixy;
PixyCamData pixyData;

// P control for turntable
int kP_headingControl = 1.1;
float heading = 0;

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);
  rudderServo.attach(RUDDER_PIN);
  propsServo.attach(PROPS_PIN);
  turntableServo.attach(TURNTABLE_PIN);

  pixy.init();
}

void loop() {
  // SENSE
  // Get current heading
  heading = getHeading(heading);
  RawSharpIRData irData = getIR();

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

RawSharpIRData getIR() {
  /*
   * Get distances from IR sensor suite in meters
   */
  RawSharpIRData rawData;

  //Using custom function
  rawData.port90Dist = getIRDist(PORT_90_IR_PIN) / 100.0;
  rawData.port45Dist = getIRDist(PORT_45_IR_PIN) / 100.0;
  rawData.bowDist = getIRDist(BOW_IR_PIN) / 100.0;
  rawData.starboard45Dist = getIRDist(STARBOARD_45_IR_PIN) / 100.0;
  rawData.starboard90Dist = getIRDist(STARBOARD_90_IR_PIN) / 100.0;

  return rawData;
}

ProcessedSharpIRData solveIR(float irAngle, float irDistance) {
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
  float targetDistance = irDistance + D2;

  ProcessedSharpIRData processedData;
  processedData.distance = sqrt(pow(targetDistance, 2) + pow(D1, 2) - 2 * D1 * (targetDistance) * cos(angle)); // Law of cosines
  processedData.rotAngle = asin((sin(angle) * (targetDistance)) / (processedData.distance)); // Law of sines
  processedData.rotAngle = radToDeg(processedData.rotAngle * side);

  return processedData;
}

float getIRDist(int pin){
  /* 
   * Compute the distance from the IR sensor
   * This uses the same setup as the SharpIR library but can
   * be modified as need (different averaging or tweaking values)
   * 
   * The equation Distance = 29.988 X POW(Volt , -1.173) was derived
   * by guillaume-rico
   * 
   * Parameters:
   * - IR_pin: analog pin number of the IR sensor to be read from
   */
  int count = 25;
  int analog_val[count];
  int dist;

  for (int i=0; i<count; i++){
    analog_val[i] = analogRead(pin);
  }
  //Get median value
  sort(analog_val, count);

  dist = 29.988 * pow(map(analog_val[count / 2], 0, 1023, 0, 5000)/1000.0, -1.173);
  return dist;
}

float degToRad(float deg) { return deg * M_PI / 180; }
float radToDeg(float rad) { return rad * 180 / M_PI; }

void camFindWhale()
{
  uint16_t blocks;
  int mid_point = 319/2 + 1;


  blocks = pixy.getBlocks();
  
  // If blocks are detected, update struct
  if (blocks)
  {
    pixyData.isDetected = true;
    pixyData.x = pixy.blocks[0].x;
    pixyData.y = pixy.blocks[0].y;
    pixyData.w = pixy.blocks[0].width;
    pixyData.h = pixy.blocks[0].height;
    pixyData.a = pixy.blocks[0].width * pixy.blocks[0].height;
    pixyData.theta = ((pixy.blocks[0].x * 75)/319) - 37.5;
  }
  else
  {
    pixyData.isDetected = false;
  }
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

void sort(int a[], int size) {
  /*
   * SharpIR library sort function
   */
  for(int i=0; i<(size-1); i++) {
    bool flag = true;
    for(int d=0; d<(size-(i+1)); d++) {
      if(a[d] > a[d+1]) {
        int t = a[d];
        a[d] = a[d+1];
        a[d+1] = t;
        flag = false;
      }
    }
  if (flag) break;
  }
}
