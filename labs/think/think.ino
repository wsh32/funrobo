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

// P control for turntable
int kP_headingControl = 1.1;
float heading = 0;

// data buffers for commands
const int headingSize = (MAX_HEADING - MIN_HEADING) / STEP_HEADING;

int headingWeightsAvoid[headingSize] = {0};
int headingWeightsHunt[headingSize] = {0};
float desiredAvg = 100;
int velWeightAvoid = 0;
int velWeightHunt = 0;
boolean dir = false;
int avoidLastMainIndex = headingSize / 2;

float pixyDecay = 0;

//Initialize threat array, no threats
int consistent_count[5] = {0,0,0,0,0}; //Consistency counter for nothing in IR
float consistent_dist[5] = {0,0,0,0,0};

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
  PixyCamData pixyData = getPixyCam(pixy);

  // THINK
  // Behaviors
  // always run avoid, but choose between follow and hunt based on whether or not we found the narwal
  avoid(irData, heading);
  if (pixyData.isDetected){
    follow(pixyData, heading);
  }
  else {
    hunt(pixyData, heading);
  }
  
  Command command = arbitrate(headingWeightsAvoid, velWeightAvoid, headingWeightsHunt, velWeightHunt);
  float headingCommandDegs = command.heading;
  int velCommand = command.vel;
//  Serial.println(heading);
//   Serial.println(headingCommandDegs);
//
//  for (int i =0; i < headingSize; i++){
//    Serial.print(headingWeightsAvoid[i]);
//    Serial.print(" ");
//  }
//  Serial.println("");

  // CONTROLLER
  HeadingCommand headingOutput = setHeading(headingCommandDegs, heading);
  float turntableOutput = headingOutput.turntableCommand;
  int rudderOutput = headingOutput.rudderCommand;
//  Serial.println(rudderOutput);
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
  return headingAngle;
}


RawSharpIRData getIR() {
  /*
   * Get distances from IR sensor suite in meters
   */
  RawSharpIRData rawIRData;

  rawIRData.port90Dist = port90IR.distance() / 100.0;
  rawIRData.port45Dist = port45IR.distance() / 100.0;
  rawIRData.bowDist = bowIR.distance() / 100.0;
  rawIRData.starboard45Dist = starboard45IR.distance() / 100.0;
  rawIRData.starboard90Dist = starboard90IR.distance() / 100.0;

  return rawIRData;
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
  processedData.dist = sqrt(pow(targetDistance, 2) + pow(D1, 2) - 2 * D1 * (targetDistance) * cos(angle)); // Law of cosines
  processedData.rotAngle = asin((sin(angle) * (targetDistance)) / (processedData.dist)); // Law of sines
  processedData.rotAngle = radToDeg(processedData.rotAngle * side);

  return processedData;
}


PixyCamData getPixyCam(Pixy cam) {
  PixyCamData pixyData;

  uint16_t blocks;
  int mid_point = 319/2 + 1;

  blocks = cam.getBlocks();
  if (blocks) {
    pixyDecay = 1;
  } else {
    pixyDecay = pixyDecay / 2;
  }
  
  // If blocks are detected, update struct
  if (pixyDecay > 0.03125) {
    pixyData.isDetected = true;
    pixyData.x = pixy.blocks[0].x;
    pixyData.y = pixy.blocks[0].y;
    pixyData.w = pixy.blocks[0].width;
    pixyData.h = pixy.blocks[0].height;
    pixyData.a = pixy.blocks[0].width * pixy.blocks[0].height;
    pixyData.theta = ((pixy.blocks[0].x * 75)/319) - 37.5;
  } else {
    pixyData.isDetected = false;
  }

  return pixyData;
}

// THINK FUNCTIONS
void avoid(RawSharpIRData irRawData, float heading) {
  /**
   * Avoid behavior
   * 
   * Sets values into headingWeightsAvoid and velWeightAvoid global variables
   * 
   * Parameters:
   * - irRawData: Raw Sharp IR data
   */
   int avoid_array[5] = {100, 100, 100 , 100, 100};
   
   ProcessedSharpIRData port_90_data = solveIR(PORT_90_IR_ANGLE, irRawData.port90Dist);
   ProcessedSharpIRData port_45_data =solveIR(PORT_45_IR_ANGLE, irRawData.port45Dist);
   ProcessedSharpIRData bow_data =solveIR(BOW_IR_ANGLE, irRawData.bowDist);
   ProcessedSharpIRData starboard_45_data =solveIR(STARBOARD_45_IR_ANGLE, irRawData.starboard45Dist);
   ProcessedSharpIRData starboard_90_data =solveIR(STARBOARD_90_IR_ANGLE, irRawData.starboard90Dist);
  
   ProcessedSharpIRData sensor_suite[5] = {port_90_data, port_45_data, bow_data, starboard_45_data, starboard_90_data};

   // clear headingWeightsAvoid
   for (int i = 0; i < headingSize; i++) {
    headingWeightsAvoid[i] = 100;
   }

   //Create magnitudes of avoidance 
   for(int i = 0; i < 5; i++){
    ProcessedSharpIRData ir_processed = sensor_suite[i];
//    if (i == 0) {
//      Serial.println();
//      Serial.println(ir_processed.dist); 
//    }
    
    
    //Check for consistency, otherwise value is probably out of range
    if ((ir_processed.dist > consistent_dist[i] + 0.05) || (ir_processed.dist < consistent_dist[i] - 0.05)){
      consistent_dist[i] = ir_processed.dist;
      consistent_count[i] =0;
      avoid_array[i] = 1;
      continue;
    }
    
    consistent_dist[i] = ir_processed.dist;
    consistent_count[i] += 1;

    if(consistent_count[i] > 2){
    
      //Correct for global heading data
      ir_processed.rotAngle += heading;
  
   
      float threat = 1;
      float dist = ir_processed.dist;
      //Outside our range, no threat
      if (dist > D_MAX){
        threat = 1;
      }
      //Within sensing distance, take caution
      else if ((dist > D_MIN) && (dist < D_MAX)){
        float num = (dist-D_MIN);
        float denom = (D_MAX - D_MIN*1.0);
        threat = num / denom;
      }
      //Too close -- avoid immediately
      else{
        threat = 0;
      }
      
      //Add to threat assesment
      avoid_array[i] = (int)(threat*100);
      int index = (ir_processed.rotAngle / STEP_HEADING) + (headingSize / 2);
      index = min(max(index, 0), headingSize-1);
//      Serial.println(index);
      headingWeightsAvoid[index] = (int) (threat * 100);
      
    }
  }
  //Check if there are immediate concerns
  Serial.println(sensor_suite[3].dist);
  if(sensor_suite[3].dist < 0.2) {
    velWeightAvoid = -100;
    Serial.println("AFLIHSDLKFHSDLFHALIS:FJHSD:");
  } else {
    velWeightAvoid = 0;
  }
  
  normalize(headingWeightsAvoid, desiredAvg);
}

void hunt(PixyCamData pixyCamData, float heading) {
  /**
   * Hunt behavior
   * 
   * Sets values into headingWeightsHunt and velWeightHunt global variables
   * 
   * Parameters:
   * - pixyCamData: Biggest blob from the Pixy Cam
   * - dir: false is left, true is right - last moving direction
   */
  // switch direction if reaching end 
  if (heading > 30){
    dir = false;
  }
  else if (heading < -75) {
    dir = true;
  }

  for (int i = 0; i < headingSize; i++) {
    headingWeightsHunt[i] = 0;
  }

  headingWeightsHunt[0] = dir ? 0 : 100;
  headingWeightsHunt[headingSize - 1] = dir ? 100 : 0;
  normalize(headingWeightsHunt, desiredAvg);
  
  velWeightHunt = 0;
}

void follow(PixyCamData pixyCamData, float heading) {
  /**
   * Follow behavior
   * 
   * Sets values into headingWeightsHunt and velWeightHunt global variables
   * 
   * Parameters:
   * - pixyCamData: Biggest blob from the Pixy Cam
   * - heading: Boat heading
   */

  if (pixyCamData.isDetected){
    // Clear weights
    for(int i = 0; i < headingSize; i++) {
      headingWeightsHunt[i] = 0;
    }

//    int mainIndex = mapFloat(pixyCamData.x, -40, 40, 10, 26);
//    mainIndex += mapFloat(heading, -90, 90, 0, headingSize);
    int pixyHeadingCommand = pixyCamData.theta + heading;
    int mainIndex = mapFloat(pixyHeadingCommand, -90, 90, 0, headingSize);
//    Serial.println("--------------");
//    Serial.println(pixyCamData.x);
    mainIndex = min(max(mainIndex, 1), headingSize - 2);
    headingWeightsHunt[mainIndex] = 70;
    headingWeightsHunt[mainIndex+1] = 15;
    headingWeightsHunt[mainIndex-1] = 15;
    normalize(headingWeightsHunt, desiredAvg);
    avoidLastMainIndex = mainIndex;
  }

  velWeightHunt = 100;

}

//FOLLOW SUB-FUNCTIONS
int mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  x = x + 2.5;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Command arbitrate(int headingAvoid[], int velAvoid, int headingHunt[], int velHunt) {
  Command c;
  int headingSum[headingSize];
  for (int i = 0; i < headingSize; i++) {
    headingSum[i] = headingHunt[i] * HUNT_WEIGHT;
  }

  int maxIntensity = 0;
  int maxIndex = 0;
  for (int i = 0; i < headingSize; i++) {
    if (headingSum[i] > maxIntensity) {
      maxIndex = i;
      maxIntensity = headingSum[i];
    }
  }
  
//  for (int i =0; i < headingSize; i++){
//    Serial.print(headingSum[i]);
//    Serial.print(" ");
//  }
//  Serial.println("");

  Serial.println(velAvoid);
  
  c.heading = (maxIndex - headingSize / 2) * STEP_HEADING;
  c.vel = min(max((velAvoid * AVOID_WEIGHT + velHunt * HUNT_WEIGHT), 0), 100);
  return c;
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
  int propPower = map(propsPcnt, 0, 100, 0, 255);
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
   * Low to high, min to max sort
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

void normalize(int arr[], float desiredAvg) {
  /*
   * Normalizes a weight array to have the same average
   * arr[] is the weight array to input
   * desiredAvg is the average that all other weight arrays will be 
  */
  float sum = 0;
  for (int i = 0; i < headingSize; i++) {
    sum += arr[i];
  }
  // the desired average that we divide every element by
  float avg = (sum / headingSize) / desiredAvg;
  for (int i = 0; i < headingSize; i++) {
    arr[i] = arr[i] / avg;
  }
}
