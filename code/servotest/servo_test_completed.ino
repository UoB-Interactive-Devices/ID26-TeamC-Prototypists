#include <Wire.h>
#include "PCA9685.h"

ServoDriver servo;

// channels
const int SHOULDER_L = 11;
const int SHOULDER_R = 13;
const int ELBOW      = 7;
const int WRIST      = 4;
const int WRIST_ROT  = 10;    // 360 degree
const int GRIPPER    = 16;


const int CENTER = 90;
const int DELTA  = 45;

// stop value for 360 degree servo
int WRIST_ROT_STOP = 90;

void safeAngle(int ch, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  servo.setAngle(ch, angle);
}

void stopWristRotate() {
  servo.setAngle(WRIST_ROT, WRIST_ROT_STOP);
}

void wristRotateCW() {
  servo.setAngle(WRIST_ROT, WRIST_ROT_STOP + 20);
}

void wristRotateCCW() {
  servo.setAngle(WRIST_ROT, WRIST_ROT_STOP - 20);
}

void centerAll180Servos() {
  safeAngle(SHOULDER_L, CENTER);
  safeAngle(SHOULDER_R, CENTER);
  safeAngle(ELBOW, CENTER);
  safeAngle(WRIST, CENTER);
  safeAngle(GRIPPER, CENTER);
}

void stopAll() {
  centerAll180Servos();
  stopWristRotate();
}

void testShoulderPair() {
  Serial.println("Testing shoulder pair");

  safeAngle(SHOULDER_L, CENTER - DELTA);
  safeAngle(SHOULDER_R, CENTER + DELTA);
  delay(1200);

  centerAll180Servos();
  delay(800);

  safeAngle(SHOULDER_L, CENTER - DELTA);
  safeAngle(SHOULDER_R, CENTER + DELTA);
  delay(1200);

  centerAll180Servos();
  delay(800);
}

void testElbow() {
  Serial.println("Testing elbow");

  safeAngle(ELBOW, CENTER - DELTA);
  delay(1200);

  safeAngle(ELBOW, CENTER);
  delay(800);

  safeAngle(ELBOW, CENTER + DELTA);
  delay(1200);

  safeAngle(ELBOW, CENTER);
  delay(800);
}

void testWrist() {
  Serial.println("Testing wrist");

  safeAngle(WRIST, CENTER - DELTA);
  delay(1200);

  safeAngle(WRIST, CENTER);
  delay(800);

  safeAngle(WRIST, CENTER + DELTA);
  delay(1200);

  safeAngle(WRIST, CENTER);
  delay(800);
}

void testGripper() {
  Serial.println("Testing gripper");

  safeAngle(GRIPPER, CENTER - DELTA);
  delay(1200);

  safeAngle(GRIPPER, CENTER);
  delay(800);

  safeAngle(GRIPPER, CENTER + DELTA);
  delay(1200);

  safeAngle(GRIPPER, CENTER);
  delay(800);
}

void testWristRotate() {
  Serial.println("Testing wrist rotate");

  stopWristRotate();
  delay(1000);

  wristRotateCW();
  delay(1000);

  stopWristRotate();
  delay(1000);

  wristRotateCCW();
  delay(1000);

  stopWristRotate();
  delay(1000);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  servo.init(0x7f);
  delay(1000);

  Serial.println("Start robot arm full test");
  stopAll();
  delay(2000);
}

void loop() {
  testShoulderPair();
  testElbow();
  testWrist();
  testGripper();
  testWristRotate();

  Serial.println("Test cycle complete");
  delay(3000);
}