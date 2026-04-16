/*
 * ROBOT ARM - INVERSE KINEMATICS SOLVER
 * 
 * Calibrated offsets:
 *   Shoulder: horizontal at SL=180, SR=0. Direction inverted (-1).
 *   Elbow: straight at 90. Direction normal (+1).
 * 
 * Geometry:
 *   Base height = 92mm (turntable center to shoulder axis)
 *   L1 = 185mm (shoulder to elbow)
 *   L2 = 85mm  (elbow to wrist)
 * 
 * Coordinate system:
 *   Origin = center of turntable
 *   X = forward, Y = left/right, Z = up
 *   Shoulder sits at (0, 0, 92)
 * 
 * Usage (Serial, 9600 baud, newline ending):
 *   goto x,z       -> move to position (x=forward mm, z=height mm)
 *   home           -> arm horizontal, elbow straight
 *   angles         -> print current IK and servo angles
 *   grip open      -> open gripper
 *   grip close     -> close gripper
 *   help           -> show commands
 * 
 * NOTE: Base rotation (stepper) is not included yet.
 *       For now we work in 2D: the vertical plane at whatever
 *       base angle the arm is currently pointing.
 *       x = horizontal distance from base center
 *       z = height from ground (not from shoulder)
 */

#include "PCA9685.h"
#include <Wire.h>
#include <math.h>

ServoDriver servo;

// ====== SERVO CHANNELS ======
const int SHOULDER_L = 11;
const int SHOULDER_R = 9;
const int ELBOW      = 7;
const int WRIST      = 5;
const int WRIST_ROT  = 3;
const int GRIPPER    = 1;

// ====== ARM GEOMETRY (mm) ======
const float BASE_HEIGHT = 92.0;
const float L1 = 185.0;  // shoulder to elbow
const float L2 = 85.0;   // elbow to wrist

// ====== CALIBRATION OFFSETS ======
// Shoulder: servo reads 180 (L) / 0 (R) when arm is horizontal (IK angle 0)
// Direction: inverted — decreasing servo = arm goes up = positive IK angle
const float SHOULDER_OFFSET = 180.0;
const float SHOULDER_DIR    = -1.0;

// Elbow: servo reads 90 when straight (IK angle 0)
// Direction: normal — increasing servo = elbow bends up = positive IK angle
const float ELBOW_OFFSET = 90.0;
const float ELBOW_DIR    = 1.0;

// ====== SERVO LIMITS ======
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

// ====== MOVEMENT SPEED ======
const int STEP_DELAY = 20;  // ms per degree of movement

// ====== CURRENT STATE ======
int shoulderLCurrent = 180;
int shoulderRCurrent = 0;
int elbowCurrent     = 90;

float currentIKShoulder = 0.0;  // radians
float currentIKElbow    = 0.0;  // radians

// ====== IK ANGLE TO SERVO ANGLE ======
// Takes IK angle in degrees, returns servo angle

int shoulderIKToServo(float ikAngleDeg) {
  int servoAngle = (int)(SHOULDER_OFFSET + SHOULDER_DIR * ikAngleDeg);
  return constrain(servoAngle, SERVO_MIN, SERVO_MAX);
}

int elbowIKToServo(float ikAngleDeg) {
  int servoAngle = (int)(ELBOW_OFFSET + ELBOW_DIR * ikAngleDeg);
  return constrain(servoAngle, SERVO_MIN, SERVO_MAX);
}

// ====== SMOOTH MOVEMENT ======

void moveShoulderSmooth(int targetL) {
  int targetR = 180 - targetL;
  targetL = constrain(targetL, SERVO_MIN, SERVO_MAX);
  targetR = constrain(targetR, SERVO_MIN, SERVO_MAX);

  while (shoulderLCurrent != targetL || shoulderRCurrent != targetR) {
    if (shoulderLCurrent < targetL) shoulderLCurrent++;
    else if (shoulderLCurrent > targetL) shoulderLCurrent--;

    if (shoulderRCurrent < targetR) shoulderRCurrent++;
    else if (shoulderRCurrent > targetR) shoulderRCurrent--;

    servo.setAngle(SHOULDER_L, shoulderLCurrent);
    servo.setAngle(SHOULDER_R, shoulderRCurrent);
    delay(STEP_DELAY);
  }
}

void moveElbowSmooth(int target) {
  target = constrain(target, SERVO_MIN, SERVO_MAX);

  while (elbowCurrent != target) {
    if (elbowCurrent < target) elbowCurrent++;
    else elbowCurrent--;

    servo.setAngle(ELBOW, elbowCurrent);
    delay(STEP_DELAY);
  }
}

// ====== INVERSE KINEMATICS ======
// Input: x (horizontal distance from base center, mm)
//        z (height from ground, mm)
// Output: true if reachable, false if not
//         Sets currentIKShoulder and currentIKElbow (radians)

bool solveIK(float x, float z) {
  // Convert z to height relative to shoulder
  float z_rel = z - BASE_HEIGHT;

  // Distance from shoulder to target in the vertical plane
  float r = sqrt(x * x + z_rel * z_rel);

  // Check if target is reachable
  float maxReach = L1 + L2;
  float minReach = fabs(L1 - L2);

  if (r > maxReach) {
    Serial.print("Target too far! Distance: ");
    Serial.print(r, 1);
    Serial.print("mm, max reach: ");
    Serial.print(maxReach, 1);
    Serial.println("mm");
    return false;
  }
  if (r < minReach) {
    Serial.print("Target too close! Distance: ");
    Serial.print(r, 1);
    Serial.print("mm, min reach: ");
    Serial.print(minReach, 1);
    Serial.println("mm");
    return false;
  }

  // Elbow angle via law of cosines
  float D = (x * x + z_rel * z_rel - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  D = constrain(D, -1.0, 1.0);
  float elbowAngle = acos(D);  // always positive = elbow up

  // Shoulder angle
  float shoulderAngle = atan2(z_rel, x) - atan2(L2 * sin(elbowAngle), L1 + L2 * cos(elbowAngle));

  currentIKShoulder = shoulderAngle;
  currentIKElbow = elbowAngle;

  return true;
}

// ====== MOVE ARM TO COORDINATES ======

bool moveArmTo(float x, float z) {
  Serial.print("Target: x=");
  Serial.print(x, 1);
  Serial.print("mm, z=");
  Serial.print(z, 1);
  Serial.println("mm");

  if (!solveIK(x, z)) {
    return false;
  }

  // Convert from radians to degrees
  float shoulderDeg = currentIKShoulder * (180.0 / M_PI);
  float elbowDeg = currentIKElbow * (180.0 / M_PI);

  Serial.print("IK angles -> shoulder: ");
  Serial.print(shoulderDeg, 1);
  Serial.print("°, elbow: ");
  Serial.print(elbowDeg, 1);
  Serial.println("°");

  // Convert IK degrees to servo values
  int servoSL = shoulderIKToServo(shoulderDeg);
  int servoE  = elbowIKToServo(elbowDeg);
  int servoSR = 180 - servoSL;

  Serial.print("Servo values -> SL: ");
  Serial.print(servoSL);
  Serial.print(", SR: ");
  Serial.print(servoSR);
  Serial.print(", Elbow: ");
  Serial.println(servoE);

  // Check if any servo is at its limit (clamped)
  if (servoSL == SERVO_MIN || servoSL == SERVO_MAX) {
    Serial.println("WARNING: Shoulder at servo limit!");
  }
  if (servoE == SERVO_MIN || servoE == SERVO_MAX) {
    Serial.println("WARNING: Elbow at servo limit!");
  }

  // Move smoothly
  moveShoulderSmooth(servoSL);
  moveElbowSmooth(servoE);

  return true;
}

// ====== GO HOME (horizontal, straight) ======

void goHome() {
  Serial.println("Going home (horizontal, elbow straight)...");
  moveShoulderSmooth(180);
  moveElbowSmooth(90);
  currentIKShoulder = 0.0;
  currentIKElbow = 0.0;
  Serial.println("Home.");
}

// ====== PRINT CURRENT STATE ======

void printAngles() {
  float sDeg = currentIKShoulder * (180.0 / M_PI);
  float eDeg = currentIKElbow * (180.0 / M_PI);

  Serial.println("------ CURRENT STATE ------");
  Serial.print("  IK shoulder: ");
  Serial.print(sDeg, 1);
  Serial.println("°");
  Serial.print("  IK elbow:    ");
  Serial.print(eDeg, 1);
  Serial.println("°");
  Serial.print("  Servo SL:    ");
  Serial.println(shoulderLCurrent);
  Serial.print("  Servo SR:    ");
  Serial.println(shoulderRCurrent);
  Serial.print("  Servo elbow: ");
  Serial.println(elbowCurrent);
  Serial.println("---------------------------");
}

// ====== GRIPPER ======

void gripOpen() {
  servo.setAngle(GRIPPER, 0);
  Serial.println("Gripper open");
}

void gripClose() {
  servo.setAngle(GRIPPER, 120);
  Serial.println("Gripper closed");
}

// ====== HELP ======

void printHelp() {
  Serial.println();
  Serial.println("===== ROBOT ARM IK COMMANDS =====");
  Serial.println("  goto x,z     Move to position (mm)");
  Serial.println("                 x = horizontal distance from base");
  Serial.println("                 z = height from ground");
  Serial.println("  home         Arm horizontal, elbow straight");
  Serial.println("  angles       Print current angles");
  Serial.println("  grip open    Open gripper");
  Serial.println("  grip close   Close gripper");
  Serial.println("  help         Show this message");
  Serial.println();
  Serial.println("  Examples:");
  Serial.println("    goto 200,92   -> reach forward at shoulder height");
  Serial.println("    goto 150,50   -> reach forward and down");
  Serial.println("    goto 100,150  -> reach forward and up");
  Serial.println("=================================");
}

// ====== SETUP ======

void setup() {
  Wire.begin();
  Serial.begin(9600);
  servo.init(0x7f);

  delay(500);

  Serial.println();
  Serial.println("================================");
  Serial.println("   ROBOT ARM - IK SOLVER");
  Serial.println("================================");

  goHome();
  printHelp();
}

// ====== LOOP ======

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    String lower = input;
    lower.toLowerCase();

    if (lower == "home") {
      goHome();
    }
    else if (lower == "angles") {
      printAngles();
    }
    else if (lower == "grip open") {
      gripOpen();
    }
    else if (lower == "grip close") {
      gripClose();
    }
    else if (lower == "help") {
      printHelp();
    }
    else if (lower.startsWith("goto ")) {
      String coords = input.substring(5);
      coords.trim();
      int commaIdx = coords.indexOf(',');

      if (commaIdx > 0) {
        float x = coords.substring(0, commaIdx).toFloat();
        float z = coords.substring(commaIdx + 1).toFloat();
        moveArmTo(x, z);
      } else {
        Serial.println("Format: goto x,z  (e.g. goto 200,92)");
      }
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(input);
      Serial.println("Type 'help' for commands.");
    }
  }
}