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

#define POT_PIN       0
#define RUDDER_PIN    0
#define PROPS_PIN     0
#define TURNTABLE_PIN 0
#define LED_PIN       0
#define BUZZER_PIN    0

Servo rudderServo;
Servo propsServo;
Servo turnTableServo;

void setup() {
  pinMode(POT_PIN, INPUT);
  rudderServo.attach(RUDDER_PIN);
  propsServo.attach(PROPS_PIN);
  turnTableServo.attach(TURNTABLE_PIN);
}

void loop() {

}
