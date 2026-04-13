#include <Wire.h>
#include "PCA9685.h"

ServoDriver servo;

const int SHOULDER_L = 11;
const int SHOULDER_R = 9;
const int ELBOW      = 7;
const int WRIST      = 5;
const int WRIST_ROT  = 3;
const int GRIPPER    = 1;


const int CENTER = 90;


const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;


const int STEP_DELAY = 20;


int shoulderLCurrent = CENTER;
int shoulderRCurrent = CENTER;
int elbowCurrent     = CENTER;
int wristCurrent     = CENTER;
int wristRotCurrent  = CENTER;
int gripperCurrent   = CENTER;

void clampAngle(int &angle) {
  if (angle < MIN_ANGLE) angle = MIN_ANGLE;
  if (angle > MAX_ANGLE) angle = MAX_ANGLE;
}

void moveServoSlow(int ch, int &currentAngle, int targetAngle) {
  clampAngle(targetAngle);

  if (currentAngle < targetAngle) {
    for (int a = currentAngle; a <= targetAngle; a++) {
      servo.setAngle(ch, a);
      delay(STEP_DELAY);
    }
  } else if (currentAngle > targetAngle) {
    for (int a = currentAngle; a >= targetAngle; a--) {
      servo.setAngle(ch, a);
      delay(STEP_DELAY);
    }
  }

  currentAngle = targetAngle;
}

void setShoulders(int leftAngle) {
  int rightAngle = 180 - leftAngle;
  clampAngle(leftAngle);
  clampAngle(rightAngle);

  while (shoulderLCurrent != leftAngle || shoulderRCurrent != rightAngle) {
    if (shoulderLCurrent < leftAngle) shoulderLCurrent++;
    else if (shoulderLCurrent > leftAngle) shoulderLCurrent--;

    if (shoulderRCurrent < rightAngle) shoulderRCurrent++;
    else if (shoulderRCurrent > rightAngle) shoulderRCurrent--;

    servo.setAngle(SHOULDER_L, shoulderLCurrent);
    servo.setAngle(SHOULDER_R, shoulderRCurrent);
    delay(STEP_DELAY);
  }

  Serial.print("Shoulder L: ");
  Serial.print(leftAngle);
  Serial.print(" , Shoulder R: ");
  Serial.println(rightAngle);
}

void setElbow(int angle) {
  moveServoSlow(ELBOW, elbowCurrent, angle);
  Serial.print("Elbow: ");
  Serial.println(angle);
}

void setWrist(int angle) {
  moveServoSlow(WRIST, wristCurrent, angle);
  Serial.print("Wrist: ");
  Serial.println(angle);
}

void setWristRot(int angle) {
  moveServoSlow(WRIST_ROT, wristRotCurrent, angle);
  Serial.print("Wrist Rotate: ");
  Serial.println(angle);
}

void setGripper(int angle) {
  moveServoSlow(GRIPPER, gripperCurrent, angle);
  Serial.print("Gripper: ");
  Serial.println(angle);
}

void centerAll() {
  setShoulders(CENTER);
  setElbow(CENTER);
  setWrist(CENTER);
  setWristRot(CENTER);
  setGripper(CENTER);
  Serial.println("All servos centered at 90");
  delay(1500);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  servo.init(0x7f);
  delay(500);

  centerAll();
}

void loop() {
  Serial.println("Testing shoulders");
  setShoulders(70);
  delay(1000);
  setShoulders(110);
  delay(1000);
  setShoulders(90);
  delay(1500);

  Serial.println("Testing elbow");
  setElbow(60);
  delay(1000);
  setElbow(120);
  delay(1000);
  setElbow(90);
  delay(1500);

  Serial.println("Testing wrist");
  setWrist(60);
  delay(1000);
  setWrist(120);
  delay(1000);
  setWrist(90);
  delay(1500);

  Serial.println("Testing wrist rotation");
  setWristRot(60);
  delay(1000);
  setWristRot(120);
  delay(1000);
  setWristRot(90);
  delay(1500);

  Serial.println("Testing gripper");
  setGripper(0);
  delay(1000);
  setGripper(120);
  delay(1000);
  setGripper(90);
  delay(2000);
}
