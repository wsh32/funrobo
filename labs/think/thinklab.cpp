//// SENSE FUNCTIONS
//float getHeading() {
//  int rawPotReading = analogRead(POT_PIN);
//  // TODO Calibrate these values
//  return map(rawPotReading, 0, 1024, -180, 180); // Map raw units to degrees
//}
//
//// CONTROLLER FUNCTIONS
//HeadingCommand setHeading(float headingCommand, float potPosition) {
//  /**
//   * Closed loop position control for heading
//   * Sets rudder to fixed angle
//   * P controller for turntable
//   * 
//   * Parameters:
//   * - headingCommand: Heading command (in degrees)
//   * - potPosition: position feedback (in degrees)
//   */
//  // Set rudder angle
//  // TODO mapping for rudder from heading
//  float rudderAngle = map(headingCommand, -180, 180, -1, 1);  // map from degrees to rudder units (-1 to 1)
//
//  // Move turntable to position
//  // PID loop, kP_headingControl is defined in thinklab.h
//  float turntablePower = kP_headingControl * (headingCommand - potPosition);
//  turntablePower = max(min(turntablePower, 1), -1);  // Threshold to -1 to 1
//
//  HeadingCommand command;
//  command.rudderCommand = rudderAngle;
//  command.turntableCommand = turntablePower;
//
//  return command;
//}
//
//void setVel(float vel) {
//  /**
//   * Velocity controller
//   * Open loop control of propeller speed and direction
//   * 
//   * Parameters:
//   * - vel: Velocity command
//   */
//
//   float propPower = map(vel, -10, 10, -1, 1);  // map from velocity (TODO get actual values) to prop units (-1 to 1)
////   setProps(propPower);
//}
//
//// ACT FUNCTIONS
//void setRudder(float rudderPcnt, Servo rudderServo) {
//  /**
//   * Set rudder servo
//   * 
//   * Parameters:
//   * - rudderPcnt: Value from -1 to 1
//   * - rudderServo: Initialized Servo class
//   */
//   rudderServo.write(map(rudderPcnt, -1, 1, 0, 255));
//}
//
//void setProps(float propsPcnt, Servo propsServo) {
//  /**
//   * Set propeller servo
//   * 
//   * Parameters:
//   * - propsPcnt: Value from -1 to 1
//   * - propsServo: Initialized Servo class
//   */
//   propsServo.write(map(propsPcnt, -1, 1, 0, 255));
//}
//
//void setTurntable(float turntablePcnt, Servo turntableServo) {
//  /**
//   * Set turntable motor value
//   * 
//   * Parameters:
//   * - turntablePcnt: Value from -1 to 1
//   * - turntableServo: Initialized Servo class
//   */
//   turntableServo.write(map(turntablePcnt, -1, 1, 0, 186));
//}
