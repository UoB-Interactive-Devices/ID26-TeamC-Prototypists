#include <Wire.h>
#include "PCA9685.h"
ServoDriver servo;

// ========== SERVO CHANNELS ==========
const int SHOULDER_L = 11;
const int SHOULDER_R = 9;
const int ELBOW      = 7;
const int WRIST      = 5;
const int WRIST_ROT  = 3;
const int GRIPPER    = 1;

// ========== STEPPER PINS ==========
const int DIR_PIN  = 2;
const int STEP_PIN = 3;

// ========== CONSTANTS ==========
const int CENTER = 90;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int STEP_DELAY = 30;        // ms between servo degree increments

// Stepper: NEMA 17 standard is 200 full steps per revolution (1.8°/step).
// A4988 in full-step mode (MS pins floating) = 200 steps for 360° of rotation.
const int STEPPER_STEPS_PER_REV = 800;
const int STEPPER_PULSE_US = 2000; // delay between HIGH/LOW transitions

// ========== STATE TRACKING ==========
int shoulderLCurrent = CENTER;
int shoulderRCurrent = CENTER;
int elbowCurrent     = CENTER;
int wristCurrent     = CENTER;
int wristRotCurrent  = CENTER;
int gripperCurrent   = CENTER;

// Stepper tracks its position in "steps from the starting orientation"
// Positive = CW as viewed from above, Negative = CCW
long baseStepsCurrent = 0;

// ========== HELPERS ==========
void clampAngle(int &angle) {
  if (angle < MIN_ANGLE) angle = MIN_ANGLE;
  if (angle > MAX_ANGLE) angle = MAX_ANGLE;
}

// Convert a signed angle (degrees) into a signed step count.
// +angle = CW rotation, -angle = CCW rotation.
long angleToSteps(float angleDegrees) {
  return (long)(angleDegrees * STEPPER_STEPS_PER_REV / 360.0);
}

// Drive the stepper by a signed number of steps.
// Positive = CW (DIR HIGH), Negative = CCW (DIR LOW).
void stepBase(long steps) {
  if (steps == 0) return;

  if (steps > 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
    steps = -steps;
  }

  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEPPER_PULSE_US);
  }
}

// Move the base to a target angle in degrees (relative to startup orientation).
// Computes the shortest step delta from where the base currently is.
void moveBaseToAngle(float targetAngleDegrees) {
  long targetSteps = angleToSteps(targetAngleDegrees);
  long deltaSteps = targetSteps - baseStepsCurrent;
  stepBase(deltaSteps);
  baseStepsCurrent = targetSteps;

  Serial.print("Base: ");
  Serial.print(targetAngleDegrees);
  Serial.print("°  (");
  Serial.print(baseStepsCurrent);
  Serial.println(" steps)");
}

// ========== SERVO MOTION HELPERS ==========
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
  Serial.print("  Base (steps): ");
  Serial.print(baseStepsCurrent);
  Serial.print("  (~");
  Serial.print(baseStepsCurrent * 360.0 / STEPPER_STEPS_PER_REV);
  Serial.println("°)");
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

// Coordinated servo move (base stepper is handled separately before or after)
void moveAllServos(int sTarget, int eTarget, int wTarget, int rTarget, int gTarget) {
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

// ========== SETUP / PRESETS ==========
void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Stepper pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

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
  Serial.println("===== ROBOT ARM =====");
  Serial.println("Commands:");
  Serial.println("  s <angle>   Shoulder");
  Serial.println("  e <angle>   Elbow");
  Serial.println("  w <angle>   Wrist");
  Serial.println("  r <angle>   Wrist Rotation");
  Serial.println("  g <angle>   Gripper");
  Serial.println("  b <angle>   Base rotation (degrees from home)");
  Serial.println("  b_raw <steps> Base rotation (raw step count)");
  Serial.println("  b_home      Reset base step counter to 0 (without moving)");
  Serial.println("  ik b s e w r g   All joints including base");
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

// ========== MAIN LOOP ==========
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    String lower = input;
    lower.toLowerCase();

    if (lower == "status") {
      printStatus();
    }
    else if (lower == "b_home") {
      baseStepsCurrent = 0;
      Serial.println("Base home position reset to 0 (no movement performed).");
    }
    else if (lower.startsWith("b_raw ")) {
      // Direct step count: "b_raw 50" moves 50 steps CW, "b_raw -50" moves CCW
      long steps = input.substring(6).toInt();
      stepBase(steps);
      baseStepsCurrent += steps;
      Serial.print("Base moved by raw steps: ");
      Serial.println(steps);
    }
    else if (lower.startsWith("b ")) {
      // Angle-based: "b 45" moves base to +45° from home
      float angle = input.substring(2).toFloat();
      moveBaseToAngle(angle);
    }
    else if (lower.startsWith("ik ")) {
      // Format: "ik base shoulder elbow wrist wristrot gripper"
      // Example: "ik 0 90 45 120 90 0"
      // Base is in degrees (float, can be signed)
      // Servo joints are 0-180 integers
      String data = input.substring(3);
      data.trim();

      int idx1 = data.indexOf(' ');
      int idx2 = data.indexOf(' ', idx1 + 1);
      int idx3 = data.indexOf(' ', idx2 + 1);
      int idx4 = data.indexOf(' ', idx3 + 1);
      int idx5 = data.indexOf(' ', idx4 + 1);

      if (idx1 == -1 || idx2 == -1 || idx3 == -1 || idx4 == -1 || idx5 == -1) {
        Serial.println("Bad IK format. Need: ik b s e w r g");
        return;
      }

      float b = data.substring(0, idx1).toFloat();
      int   s = data.substring(idx1 + 1, idx2).toInt();
      int   e = data.substring(idx2 + 1, idx3).toInt();
      int   w = data.substring(idx3 + 1, idx4).toInt();
      int   r = data.substring(idx4 + 1, idx5).toInt();
      int   g = data.substring(idx5 + 1).toInt();

      Serial.print("IK received: b=");
      Serial.print(b); Serial.print("° s=");
      Serial.print(s); Serial.print(" e=");
      Serial.print(e); Serial.print(" w=");
      Serial.print(w); Serial.print(" r=");
      Serial.print(r); Serial.print(" g=");
      Serial.println(g);

      // Move the base first (stepper rotation), then the servos.
      // Doing base first means the rest of the arm swings into its new vertical
      // plane before the shoulder/elbow/wrist adjust to the final pose.
      moveBaseToAngle(b);
      moveAllServos(s, e, w, r, g);
      Serial.println("IK move done");
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
      Serial.println("Unknown. Use: s/e/w/r/g/b <angle>, ik b s e w r g, pick up, place, status");
    }
  }
}