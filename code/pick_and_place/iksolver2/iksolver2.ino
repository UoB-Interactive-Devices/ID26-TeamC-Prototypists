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
const int STEP_DELAY = 30;
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
  Serial.print("Wrist Rot: ");
  Serial.println(angle);
}
void setGripper(int angle) {
  moveServoSlow(GRIPPER, gripperCurrent, angle);
  Serial.print("Gripper: ");
  Serial.println(angle);
}
void printStatus() {
  Serial.println("------ CURRENT ANGLES ------");
  Serial.print("  Shoulder L: ");
  Serial.print(shoulderLCurrent);
  Serial.print("  R: ");
  Serial.println(shoulderRCurrent);
  Serial.print("  Elbow:      ");
  Serial.println(elbowCurrent);
  Serial.print("  Wrist:      ");
  Serial.println(wristCurrent);
  Serial.print("  Wrist Rot:  ");
  Serial.println(wristRotCurrent);
  Serial.print("  Gripper:    ");
  Serial.println(gripperCurrent);
  Serial.println("----------------------------");
}
void setup() {
  Wire.begin();
  Serial.begin(9600);
  servo.init(0x7f);
  delay(500);
  servo.setAngle(SHOULDER_L, 90);
  servo.setAngle(SHOULDER_R, 90);
  servo.setAngle(ELBOW, 90);
  servo.setAngle(WRIST, 90);
  delay(1000);
  setShoulders(90);
  setElbow(90);
  setWrist(90);
  Serial.println();
  Serial.println("===== JOINT TEST =====");
  Serial.println("Commands:");
  Serial.println("  s <angle>   Shoulder");
  Serial.println("  e <angle>   Elbow");
  Serial.println("  w <angle>   Wrist");
  Serial.println("  r <angle>   Wrist Rotation");
  Serial.println("  g <angle>   Gripper");
  Serial.println("  status      Print all angles");
  Serial.println("======================");
}

void PICKUP() {
  servo.setAngle(GRIPPER, 180);
  servo.setAngle(SHOULDER_L, 50);
  servo.setAngle(SHOULDER_R, 130);
  servo.setAngle(ELBOW, 40);
  servo.setAngle(WRIST, 130);
  Serial.print("picking up pose: ");
  servo.setAngle(GRIPPER, 0);
}

void PLACE() {
  servo.setAngle(SHOULDER_L, 130);
  servo.setAngle(SHOULDER_R, 50);
  servo.setAngle(ELBOW, 140);
  servo.setAngle(WRIST, 50);
  Serial.print("placing  pose: ");
  servo.setAngle(GRIPPER, 180);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    String lower = input;
    lower.toLowerCase();

    if (lower == "status") {
      printStatus();
    }
    else if (lower.startsWith("s ")) {
      int angle = input.substring(2).toInt();
      setShoulders(angle);
    }
    else if (lower.startsWith("e ")) {
      int angle = input.substring(2).toInt();
      setElbow(angle);
    }
    else if (lower.startsWith("w ")) {
      int angle = input.substring(2).toInt();
      setWrist(angle);
    }
    else if (lower.startsWith("r ")) {
      int angle = input.substring(2).toInt();
      setWristRot(angle);
    }
    else if (lower.startsWith("g ")) {
      int angle = input.substring(2).toInt();
      setGripper(angle);
    }

    else if (lower.startsWith("pick up")) {
      PICKUP();
    }

    else if (lower.startsWith("place")) {
      PICKUP();
    }
    
    else {
      Serial.println("Unknown. Use: s/e/w/r/g <angle> or status");
    }
  }
}