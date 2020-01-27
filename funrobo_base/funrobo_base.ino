/*******************************************************************************************************************************
 * 2020 FunRobo Arduino Controller base 
 * Template for Simple Arduino Sense-Think-Act data flow
 * Baby Boy Bot 
 * Loops and responds to certain text commands 
 * 
 */
// TODO: ???? 
// load Libs
#include<Servo.h>
// Global vars, obj, and constants
const int aliveLED = 13; // name for blinking light while robot functions
const int eStopPin = 12; // pin connected to ESTOP switch
boolean aliveLEDState = true; // Blinking light for timer 
boolean ESTOP = true; // Emergency stop for motors
boolean realTimeRunStop = true; // Real time control loop flag
String command = "stop"; // String object for command string
String loopError = "no error"; // String for error system
unsigned long oldLoopTime = 0; // past loop time in miliseconds
unsigned long newLoopTime = 0; // new loop time in miliseconds
unsigned long cycleTime = 0; // elapsed time loop cycle 
const long controlLoopInterval = 1000; // name for control loop cycle time in miliseconds

// Functions
String getOperatorInput() {
  // This function prints operator command options on the serial console and prompts operator to input desired robot command
  Serial.println("=======================================================================================");
  Serial.println("| Robot Behavior-Commands: move(moves robot), stop(e-stops motors), idle(robot idles) |");
  Serial.println("|                                                                                     |");
  Serial.println("=======================================================================================");
  Serial.println("| Please type desired robot behavior in command line at top of this window            |");
  Serial.println("=======================================================================================");
  while (Serial.available() == 0) {};
  command = Serial.readString();
  Serial.print("| New robot behavior command is:  ");
  Serial.println(command);
  Serial.println("| Type stop to stop control loop and wait for new command                             |");
  Serial.println("=======================================================================================");
  return command;
}

void blinkAliveLED() {
  // this function toggles state of aliveLED blinky light LED
  // if the LED is off turn it on and vice-versa
  if (aliveLEDState == LOW) {
    aliveLEDState = HIGH;
  } else {
    aliveLEDState = LOW;
  }

  // set the LED with the ledState of the variable
  digitalWrite(aliveLED, aliveLEDState);
}

//========================================================================================
//Startup Robot Code and presets
//========================================================================================
void setup() {
  //Step 1) Put robot setup code here, Run once:
  pinMode(aliveLED, OUTPUT); //Initialize AliveLED pin as an output
  pinMode(eStopPin, INPUT_PULLUP); // use Internal Pull-up on ESTOP Switch input pin
  Serial.begin(9600); //Start Serial Communications
  Serial.println("Robot Controller Starting UP! Keeping finger away spinning things");
  //Step 2) Put your Robot Mission Here, run once:
  //Add Mission code

}
//========================================================================================
//Flight code to run continuosly until robot is powered down
//========================================================================================
void loop() {

  //Step 3)Put Operator-input-to-Robot and Robot-Reports-Back-State code in non-real-time "outer" loop:
  //  Put real-time dependant sense-think-act control in the inner loop
  //GET Operator Control Unit (OCU) Input: OCU--OCU--OCU--OCU--OCU--OCU--OCU--OCU--OCU--OCU--OCU--OCU
  command = getOperatorInput();  // get operator input from serial monitor
  if (command == "stop") realTimeRunStop = false; // skip real time inner loop
  else realTimeRunStop = true;  // Set loop flag to run = true

  //4)Put your main flight code into "inner" soft-real-time while loop structure below, to run repeatedly,
  // at a known fixed "real-time" periodic interval. This "soft real-time" loop timing structure, runs
  // fast flight control code once every controlLoopInterval

  // Real Time Loop
  while (realTimeRunStop) {
    // Check if operator inputs a command during real-time loop execution
    if (Serial.available() > 0) {
      realTimeRunStop = false;
      command = Serial.readString();
      break;
    } else {
      realTimeRunStop = true;
    }

    // Real time clock control. Check to see if one clock cycle has elapsed before running this control code
    newLoopTime = millis(); // get current Arduino time (50 days till wrap)
    if (newLoopTime - oldLoopTime >= controlLoopInterval) {  // if true run flight code
      oldLoopTime = newLoopTime;  // reset time stamp
      blinkAliveLED();  // toggle blinky alive light

      // SENSE
      // TODO add sensor code here

      // THINK
      // pick robot behavior based on operator input command typed at console
      if (command == "stop") {
        Serial.println("stop robot");
        realTimeRunStop = false;  // exit real time control loop
        break;
      } else if (command == "move") {  // move robot to operator commanded position
        Serial.println("move robot");
        Serial.println("type stop to stop robot");
        realTimeRunStop = true;  // dont exit loop after running once
      } else if (command == "idle") {  // make robot alive with small motions
        Serial.println("idle robot");
        Serial.println("type stop to stop robot");
        realTimeRunStop = true;  // run loop continually
      } else {
        Serial.println("***** WARNING ***** Invalid Input, Robot stopped, please try again");
        realTimeRunStop = false;
      }

      // ACT
      ESTOP = digitalRead(eStopPin);  // check estop switch

      // check to see if all code ran successfully in one real time increment
      cycleTime = millis() - newLoopTime; // calculate loop execution time
      if (cycleTime > controlLoopInterval) {
        Serial.println("****************************");
        Serial.println("Error - real time has failed, stop robot");  // loop took too long to run
        Serial.print(" 1000 ms real time loop took = ");
        Serial.println(cycleTime);  // print loop time"
        Serial.println("****************************");
        break;  // break
      }
    }
  }


  // SEND robot state to operator control unit (ocu)
  Serial.println("==========================");
  Serial.println("| Robot control loop stopping to wait for new command ");  // send robot status
  if (ESTOP) Serial.println("| Robot motors E-stopped by external switch");  // send estop
}

//=================================================================================================
// END OF FLIGHT CODE
//=================================================================================================
