#include <Wire.h>
#include "PCA9685.h"
ServoDriver servo;

const int SHOULDER_L = 11;
const int SHOULDER_R = 9;
const int ELBOW      = 7;
const int WRIST      = 5;
const int WRIST_ROT  = 3;
const int GRIPPER    = 1;

// --- Base stepper (A4988 + NEMA 17) ---
const int dirPin  = 2;
const int stepPin = 3;

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

// Run N steps using the exact pattern from the example sketch.
// Positive n = clockwise, negative n = counter-clockwise.
void stepBase(int n) {
  if (n >= 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
    n = -n;
  }

  for (int i = 0; i < n; i++) {
    // take one step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    // pause before taking next step
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }

  Serial.print("Base stepped ");
  Serial.print(n);
  Serial.println(" steps");
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

// Coordinated move: drives all 5 joints toward their targets simultaneously,
// stepping 1 degree at a time. Much smoother than setting each joint in sequence.
void moveAllJoints(int sTarget, int eTarget, int wTarget, int rTarget, int gTarget) {
  clampAngle(sTarget);
  clampAngle(eTarget);
  clampAngle(wTarget);
  clampAngle(rTarget);
  clampAngle(gTarget);

  int sRightTarget = 180 - sTarget;
  clampAngle(sRightTarget);

  while (shoulderLCurrent != sTarget ||
         shoulderRCurrent != sRightTarget ||
         elbowCurrent     != eTarget ||
         wristCurrent     != wTarget ||
         wristRotCurrent  != rTarget ||
         gripperCurrent   != gTarget) {

    if (shoulderLCurrent < sTarget) shoulderLCurrent++;
    else if (shoulderLCurrent > sTarget) shoulderLCurrent--;

    if (shoulderRCurrent < sRightTarget) shoulderRCurrent++;
    else if (shoulderRCurrent > sRightTarget) shoulderRCurrent--;

    if (elbowCurrent < eTarget) elbowCurrent++;
    else if (elbowCurrent > eTarget) elbowCurrent--;

    if (wristCurrent < wTarget) wristCurrent++;
    else if (wristCurrent > wTarget) wristCurrent--;

    if (wristRotCurrent < rTarget) wristRotCurrent++;
    else if (wristRotCurrent > rTarget) wristRotCurrent--;

    if (gripperCurrent < gTarget) gripperCurrent++;
    else if (gripperCurrent > gTarget) gripperCurrent--;

    servo.setAngle(SHOULDER_L, shoulderLCurrent);
    servo.setAngle(SHOULDER_R, shoulderRCurrent);
    servo.setAngle(ELBOW,      elbowCurrent);
    servo.setAngle(WRIST,      wristCurrent);
    servo.setAngle(WRIST_ROT,  wristRotCurrent);
    servo.setAngle(GRIPPER,    gripperCurrent);
    delay(STEP_DELAY);
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  // set direction of rotation to clockwise
  digitalWrite(dirPin, HIGH);

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
  Serial.println("  b <steps>   Base stepper (+CW, -CCW)");
  Serial.println("  s <angle>   Shoulder");
  Serial.println("  e <angle>   Elbow");
  Serial.println("  w <angle>   Wrist");
  Serial.println("  r <angle>   Wrist Rotation");
  Serial.println("  g <angle>   Gripper");
  Serial.println("  ik s e w r g   All joints (degrees, 0-180)");
  Serial.println("  pick up     Preset pick pose");
  Serial.println("  place       Preset place pose");
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
    else if (lower.startsWith("ik ")) {
      // Format: "ik shoulder elbow wrist wristrot gripper"
      // Example: "ik 90 45 120 90 0"
      String data = input.substring(3);
      data.trim();

      int idx1 = data.indexOf(' ');
      int idx2 = data.indexOf(' ', idx1 + 1);
      int idx3 = data.indexOf(' ', idx2 + 1);
      int idx4 = data.indexOf(' ', idx3 + 1);

      if (idx1 == -1 || idx2 == -1 || idx3 == -1 || idx4 == -1) {
        Serial.println("Bad IK format. Need: ik s e w r g");
        return;
      }

      int s = data.substring(0, idx1).toInt();
      int e = data.substring(idx1 + 1, idx2).toInt();
      int w = data.substring(idx2 + 1, idx3).toInt();
      int r = data.substring(idx3 + 1, idx4).toInt();
      int g = data.substring(idx4 + 1).toInt();

      Serial.print("IK received: s=");
      Serial.print(s); Serial.print(" e=");
      Serial.print(e); Serial.print(" w=");
      Serial.print(w); Serial.print(" r=");
      Serial.print(r); Serial.print(" g=");
      Serial.println(g);

      moveAllJoints(s, e, w, r, g);
      Serial.println("IK move done");
    }
    else if (lower.startsWith("b ")) {
      int steps = input.substring(2).toInt();
      stepBase(steps);
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
      PLACE();
    }
    else {
      Serial.println("Unknown. Use: b <steps>, s/e/w/r/g <angle>, ik s e w r g, pick up, place, status");
    }
  }
}