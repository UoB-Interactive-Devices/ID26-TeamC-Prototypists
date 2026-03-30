#include <Wire.h>
#include "PCA9685.h"

ServoDriver servo;

// channels
const int SHOULDER_L = 13;
const int SHOULDER_R = 14;
const int ELBOW      = 7;
const int WRIST      = 4;
const int WRIST_ROT  = 10;    // 360 degree
const int GRIPPER    = 16;


const int WRIST_ROT_STOP = 90;


int curShoulderL = 90;
int curShoulderR = 90;
int curElbow     = 90;
int curWrist     = 90;
int curGripper   = 90;


int clampAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return angle;
}

void safeAngle(int ch, int angle) {
  angle = clampAngle(angle);
  servo.setAngle(ch, angle);
}


void stopWristRotate() {
  servo.setAngle(WRIST_ROT, WRIST_ROT_STOP);
}


void moveServoSmooth(int ch, int &currentAngle, int targetAngle, int stepDelay = 20) {
  targetAngle = clampAngle(targetAngle);

  if (currentAngle < targetAngle) {
    for (int a = currentAngle; a <= targetAngle; a++) {
      servo.setAngle(ch, a);
      delay(stepDelay);
    }
  } else {
    for (int a = currentAngle; a >= targetAngle; a--) {
      servo.setAngle(ch, a);
      delay(stepDelay);
    }
  }

  currentAngle = targetAngle;
}

// shoulder pair control
void moveShoulderPairSmooth(int targetL, int targetR, int stepDelay = 20) {
  targetL = clampAngle(targetL);
  targetR = clampAngle(targetR);

  int startL = curShoulderL;
  int startR = curShoulderR;

  int stepsL = abs(targetL - startL);
  int stepsR = abs(targetR - startR);
  int steps = max(stepsL, stepsR);

  if (steps == 0) return;

  for (int i = 1; i <= steps; i++) {
    int nextL = startL + (targetL - startL) * i / steps;
    int nextR = startR + (targetR - startR) * i / steps;

    servo.setAngle(SHOULDER_L, nextL);
    servo.setAngle(SHOULDER_R, nextR);
    delay(stepDelay);
  }

  curShoulderL = targetL;
  curShoulderR = targetR;
}


void moveToPose(int shoulderL, int shoulderR, int elbow, int wrist, int gripper, int stepDelay = 20) {

  moveShoulderPairSmooth(shoulderL, shoulderR, stepDelay);

  moveServoSmooth(ELBOW,   curElbow,   elbow,   stepDelay);
  moveServoSmooth(WRIST,   curWrist,   wrist,   stepDelay);
  moveServoSmooth(GRIPPER, curGripper, gripper, stepDelay);
}



void poseHome() {

  moveToPose(90, 90, 95, 90, 170, 15);   // gripper open
}

void posePickAbove() {
  
  moveToPose(70, 110, 110, 75, 170, 15);
}

void posePickDown() {
  
  moveToPose(70, 110, 150, 60, 10, 15);
}

void poseLift() {
  
  moveToPose(80, 100, 105, 100, 10, 15); // gripper closed
}

void posePlaceAbove() {
  
  moveToPose(130, 50, 75, 70, 10, 15);
}



void openGripper() {
  moveServoSmooth(GRIPPER, curGripper, 170, 15);
}

void closeGripper() {
  moveServoSmooth(GRIPPER, curGripper, 10, 15);
}

// pick and place sequence
void pickAndPlace() {
  Serial.println("Go home");
  poseHome();
  delay(800);

  Serial.println("Move above pick");
  posePickAbove();
  delay(800);

  Serial.println("Move down to pick");
  posePickDown();
  delay(800);

  Serial.println("Close gripper");
  closeGripper();
  delay(800);

  Serial.println("Lift object");
  poseLift();
  delay(800);

  Serial.println("Move above place");
  posePlaceAbove();
  delay(800);

  Serial.println("Open gripper");
  openGripper();
  delay(800);

  Serial.println("Return home");
  poseHome();
  delay(1000);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  servo.init(0x7f);
  delay(1000);

  Serial.println("Start simple pick and place");

  
  safeAngle(SHOULDER_L, curShoulderL);
  safeAngle(SHOULDER_R, curShoulderR);
  safeAngle(ELBOW, curElbow);
  safeAngle(WRIST, curWrist);
  safeAngle(GRIPPER, curGripper);
  stopWristRotate();

  delay(1500);
  poseHome();
  delay(1000);
}

void loop() {
  pickAndPlace();
  delay(3000);
}
