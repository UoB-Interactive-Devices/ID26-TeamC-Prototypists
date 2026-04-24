#include <Wire.h>
#include "PCA9685.h"
#include <Pixy2.h>

ServoDriver servo;
Pixy2 pixy;

const int SHOULDER_L = 11;
const int SHOULDER_R = 9;
const int ELBOW = 7;
const int WRIST = 5;
const int WRIST_ROTATION = 3;
const int GRIP = 1;

const int STEPPER_STEP_PIN = 3;
const int STEPPER_DIR_PIN = 2;
const int STEPPER_ENABLE_PIN = 4;

const unsigned long SERIAL_BAUD = 115200;

const int SHOULDER_MIN = 45;
const int SHOULDER_MAX = 135;
const int ELBOW_MIN = 40;
const int ELBOW_MAX = 150;
const int WRIST_MIN = 30;
const int WRIST_MAX = 150;
const int WRIST_ROTATION_MIN = 0;
const int WRIST_ROTATION_MAX = 180;
const int GRIP_MIN = 20;
const int GRIP_MAX = 120;
const int ROTATE_MIN = 60;
const int ROTATE_MAX = 120;

const int HOME_SHOULDER = 90;
const int HOME_ELBOW = 90;
const int HOME_WRIST = 90;
const int HOME_WRIST_ROTATION = 90;
const int HOME_GRIP = 90;
const int HOME_ROTATE = 90;
const int PICKUP_OPEN_GRIP = 180;
const int PICKUP_CLOSE_GRIP = 5;
const int PICKUP_SHOULDER = 50;
const int PICKUP_ELBOW = 40;
const int PICKUP_WRIST = 130;
const int PLACE_SHOULDER = 50;
const int PLACE_ELBOW = 40;
const int PLACE_WRIST = 130;
const int PLACE_OPEN_GRIP = 180;
const int MACRO_STEP_DELAY_MS = 20;
const uint8_t BLUE_SIGNATURE = 1;
const int PIXY_CENTER_X = 158;
const int PIXY_TOLERANCE_X = 18;
const int PIXY_CLOSE_WIDTH = 55;
const int AUTO_ROTATE_STEP = 2;
const int AUTO_ARM_STEP = 2;
const unsigned long AUTO_LOOP_INTERVAL_MS = 180;
const unsigned long SEARCH_SPIN_INTERVAL_MS = 500;
const int NOD_ELBOW_DELTA = 8;
const int NOD_WRIST_DELTA = 8;
const unsigned long NOD_COOLDOWN_MS = 1800;
const unsigned int STEPPER_PULSE_US = 700;
const int STEPPER_MAX_STEPS_PER_COMMAND = 5000;
const int STEPPER_STEPS_PER_DEGREE = 9;
const bool STEPPER_HOLD_AFTER_MOVE = true;

String inputLine = "";

int shoulderAngle = HOME_SHOULDER;
int elbowAngle = HOME_ELBOW;
int wristAngle = HOME_WRIST;
int wristRotationAngle = HOME_WRIST_ROTATION;
int gripAngle = HOME_GRIP;
int rotateAngle = HOME_ROTATE;
bool autoBlueEnabled = false;
bool blueNodEnabled = false;
bool autoPickupDone = false;
bool pixyInitialized = false;
bool stepperEnabled = false;
unsigned long lastAutoStepMs = 0;
unsigned long lastSearchSpinMs = 0;
unsigned long lastBlueNodMs = 0;

struct BlueTarget {
  bool found;
  int x;
  int y;
  int width;
  int height;
};

int clampInt(int value, int minimumValue, int maximumValue) {
  if (value < minimumValue) return minimumValue;
  if (value > maximumValue) return maximumValue;
  return value;
}

void applyShoulder(int angle) {
  shoulderAngle = clampInt(angle, SHOULDER_MIN, SHOULDER_MAX);
  servo.setAngle(SHOULDER_L, shoulderAngle);
  servo.setAngle(SHOULDER_R, 180 - shoulderAngle);
}

void applyElbow(int angle) {
  elbowAngle = clampInt(angle, ELBOW_MIN, ELBOW_MAX);
  servo.setAngle(ELBOW, elbowAngle);
}

void applyWrist(int angle) {
  wristAngle = clampInt(angle, WRIST_MIN, WRIST_MAX);
  servo.setAngle(WRIST, wristAngle);
}

void applyWristRotation(int angle) {
  wristRotationAngle = clampInt(angle, WRIST_ROTATION_MIN, WRIST_ROTATION_MAX);
  servo.setAngle(WRIST_ROTATION, wristRotationAngle);
}

void applyGrip(int angle) {
  gripAngle = clampInt(angle, GRIP_MIN, GRIP_MAX);
  servo.setAngle(GRIP, gripAngle);
}

void setStepperEnabled(bool enabled) {
  stepperEnabled = enabled;
  digitalWrite(STEPPER_ENABLE_PIN, enabled ? LOW : HIGH);
}

void moveStepper(int steps) {
  int clampedSteps = clampInt(steps, -STEPPER_MAX_STEPS_PER_COMMAND, STEPPER_MAX_STEPS_PER_COMMAND);
  if (clampedSteps == 0) {
    return;
  }

  bool wasEnabled = stepperEnabled;
  if (!wasEnabled) {
    setStepperEnabled(true);
    delay(2);
  }

  digitalWrite(STEPPER_DIR_PIN, clampedSteps >= 0 ? HIGH : LOW);
  int pulseCount = abs(clampedSteps);
  for (int i = 0; i < pulseCount; i++) {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_PULSE_US);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(STEPPER_PULSE_US);
  }

  if (!wasEnabled && !STEPPER_HOLD_AFTER_MOVE) {
    setStepperEnabled(false);
  }
}

void applyRotateRaw(int angle) {
  int targetAngle = clampInt(angle, ROTATE_MIN, ROTATE_MAX);
  int deltaAngle = targetAngle - rotateAngle;
  moveStepper(deltaAngle * STEPPER_STEPS_PER_DEGREE);
  rotateAngle = targetAngle;
}

void stopRotate() {
  digitalWrite(STEPPER_STEP_PIN, LOW);
}

void nudgeRotateLeft() {
  applyRotateRaw(rotateAngle - AUTO_ROTATE_STEP);
}

void nudgeRotateRight() {
  applyRotateRaw(rotateAngle + AUTO_ROTATE_STEP);
}

void nudgeArmDown() {
  applyShoulder(shoulderAngle + AUTO_ARM_STEP);
  applyElbow(elbowAngle + AUTO_ARM_STEP);
  applyWrist(wristAngle - AUTO_ARM_STEP);
}

void homeAll() {
  applyShoulder(HOME_SHOULDER);
  applyElbow(HOME_ELBOW);
  applyWrist(HOME_WRIST);
  applyWristRotation(HOME_WRIST_ROTATION);
  applyGrip(HOME_GRIP);
  applyRotateRaw(HOME_ROTATE);
}

void moveJointSlow(int currentValue, int targetValue, void (*applyFn)(int), int minValue, int maxValue) {
  int clampedTarget = clampInt(targetValue, minValue, maxValue);
  while (currentValue != clampedTarget) {
    if (currentValue < clampedTarget) {
      currentValue++;
    } else {
      currentValue--;
    }
    applyFn(currentValue);
    delay(MACRO_STEP_DELAY_MS);
  }
}

void homeAllSlow() {
  moveJointSlow(gripAngle, HOME_GRIP, applyGrip, GRIP_MIN, GRIP_MAX);
  delay(150);
  moveJointSlow(wristAngle, HOME_WRIST, applyWrist, WRIST_MIN, WRIST_MAX);
  delay(150);
  moveJointSlow(
    wristRotationAngle,
    HOME_WRIST_ROTATION,
    applyWristRotation,
    WRIST_ROTATION_MIN,
    WRIST_ROTATION_MAX
  );
  delay(150);
  moveJointSlow(elbowAngle, HOME_ELBOW, applyElbow, ELBOW_MIN, ELBOW_MAX);
  delay(150);
  moveJointSlow(shoulderAngle, HOME_SHOULDER, applyShoulder, SHOULDER_MIN, SHOULDER_MAX);
  delay(150);
  moveJointSlow(rotateAngle, HOME_ROTATE, applyRotateRaw, ROTATE_MIN, ROTATE_MAX);
}

void doPickup() {
  while (gripAngle != clampInt(PICKUP_OPEN_GRIP, GRIP_MIN, GRIP_MAX)) {
    if (gripAngle < PICKUP_OPEN_GRIP) {
      applyGrip(gripAngle + 1);
    } else {
      applyGrip(gripAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  while (shoulderAngle != clampInt(PICKUP_SHOULDER, SHOULDER_MIN, SHOULDER_MAX)) {
    if (shoulderAngle < PICKUP_SHOULDER) {
      applyShoulder(shoulderAngle + 1);
    } else {
      applyShoulder(shoulderAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  while (elbowAngle != clampInt(PICKUP_ELBOW, ELBOW_MIN, ELBOW_MAX)) {
    if (elbowAngle < PICKUP_ELBOW) {
      applyElbow(elbowAngle + 1);
    } else {
      applyElbow(elbowAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  while (wristAngle != clampInt(PICKUP_WRIST, WRIST_MIN, WRIST_MAX)) {
    if (wristAngle < PICKUP_WRIST) {
      applyWrist(wristAngle + 1);
    } else {
      applyWrist(wristAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  while (gripAngle != clampInt(PICKUP_CLOSE_GRIP, GRIP_MIN, GRIP_MAX)) {
    if (gripAngle < PICKUP_CLOSE_GRIP) {
      applyGrip(gripAngle + 1);
    } else {
      applyGrip(gripAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  moveJointSlow(shoulderAngle, HOME_SHOULDER, applyShoulder, SHOULDER_MIN, SHOULDER_MAX);
}

void doPlace() {
  while (shoulderAngle != clampInt(PLACE_SHOULDER, SHOULDER_MIN, SHOULDER_MAX)) {
    if (shoulderAngle < PLACE_SHOULDER) {
      applyShoulder(shoulderAngle + 1);
    } else {
      applyShoulder(shoulderAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  while (elbowAngle != clampInt(PLACE_ELBOW, ELBOW_MIN, ELBOW_MAX)) {
    if (elbowAngle < PLACE_ELBOW) {
      applyElbow(elbowAngle + 1);
    } else {
      applyElbow(elbowAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  while (wristAngle != clampInt(PLACE_WRIST, WRIST_MIN, WRIST_MAX)) {
    if (wristAngle < PLACE_WRIST) {
      applyWrist(wristAngle + 1);
    } else {
      applyWrist(wristAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
  delay(200);

  while (gripAngle != clampInt(PLACE_OPEN_GRIP, GRIP_MIN, GRIP_MAX)) {
    if (gripAngle < PLACE_OPEN_GRIP) {
      applyGrip(gripAngle + 1);
    } else {
      applyGrip(gripAngle - 1);
    }
    delay(MACRO_STEP_DELAY_MS);
  }
}

void doBlueNod() {
  moveJointSlow(elbowAngle, elbowAngle + NOD_ELBOW_DELTA, applyElbow, ELBOW_MIN, ELBOW_MAX);
  moveJointSlow(wristAngle, wristAngle - NOD_WRIST_DELTA, applyWrist, WRIST_MIN, WRIST_MAX);
  delay(120);
  moveJointSlow(elbowAngle, HOME_ELBOW, applyElbow, ELBOW_MIN, ELBOW_MAX);
  moveJointSlow(wristAngle, HOME_WRIST, applyWrist, WRIST_MIN, WRIST_MAX);
}

BlueTarget getLargestBlueTarget() {
  BlueTarget target = {false, 0, 0, 0, 0};
  if (!pixyInitialized) {
    return target;
  }

  pixy.ccc.getBlocks();

  uint16_t bestArea = 0;
  for (uint16_t i = 0; i < pixy.ccc.numBlocks; i++) {
    Block block = pixy.ccc.blocks[i];
    if (block.m_signature != BLUE_SIGNATURE) {
      continue;
    }

    uint16_t area = block.m_width * block.m_height;
    if (!target.found || area > bestArea) {
      target.found = true;
      target.x = block.m_x;
      target.y = block.m_y;
      target.width = block.m_width;
      target.height = block.m_height;
      bestArea = area;
    }
  }

  return target;
}

bool ensurePixyReady() {
  if (pixyInitialized) {
    return true;
  }

  int result = pixy.init();
  if (result < 0) {
    Serial.println("ERR PIXY no response");
    return false;
  }

  pixyInitialized = true;
  Serial.println("OK PIXY_READY");
  return true;
}

void autoBlueOff(const char* reason) {
  autoBlueEnabled = false;
  stopRotate();
  Serial.print("OK AUTO_BLUE_OFF");
  if (reason && reason[0] != '\0') {
    Serial.print(" ");
    Serial.print(reason);
  }
  Serial.println();
}

void autoBlueOn() {
  if (!ensurePixyReady()) {
    autoBlueEnabled = false;
    return;
  }

  autoBlueEnabled = true;
  blueNodEnabled = false;
  autoPickupDone = false;
  lastAutoStepMs = 0;
  lastSearchSpinMs = 0;
  Serial.println("OK AUTO_BLUE_ON");
}

void blueNodOn() {
  if (!ensurePixyReady()) {
    blueNodEnabled = false;
    return;
  }

  blueNodEnabled = true;
  autoBlueEnabled = false;
  lastBlueNodMs = 0;
  Serial.println("OK BLUE_NOD_ON");
}

void blueNodOff(const char* reason) {
  blueNodEnabled = false;
  Serial.print("OK BLUE_NOD_OFF");
  if (reason && reason[0] != '\0') {
    Serial.print(" ");
    Serial.print(reason);
  }
  Serial.println();
}

void runAutoBlueLoop() {
  if (!autoBlueEnabled) {
    return;
  }

  unsigned long now = millis();
  if (now - lastAutoStepMs < AUTO_LOOP_INTERVAL_MS) {
    return;
  }
  lastAutoStepMs = now;

  if (autoPickupDone) {
    autoBlueOff("DONE");
    return;
  }

  BlueTarget target = getLargestBlueTarget();
  if (!target.found) {
    if (now - lastSearchSpinMs >= SEARCH_SPIN_INTERVAL_MS) {
      nudgeRotateRight();
      delay(80);
      stopRotate();
      lastSearchSpinMs = now;
      Serial.println("AUTO searching");
    }
    return;
  }

  int xError = target.x - PIXY_CENTER_X;
  if (xError < -PIXY_TOLERANCE_X) {
    nudgeRotateLeft();
    Serial.println("AUTO align left");
    return;
  }

  if (xError > PIXY_TOLERANCE_X) {
    nudgeRotateRight();
    Serial.println("AUTO align right");
    return;
  }

  stopRotate();

  if (target.width < PIXY_CLOSE_WIDTH) {
    nudgeArmDown();
    Serial.println("AUTO approach");
    return;
  }

  Serial.println("AUTO pickup");
  doPickup();
  autoPickupDone = true;
}

void runBlueNodLoop() {
  if (!blueNodEnabled) {
    return;
  }

  unsigned long now = millis();
  if (now - lastBlueNodMs < NOD_COOLDOWN_MS) {
    return;
  }

  BlueTarget target = getLargestBlueTarget();
  if (!target.found) {
    return;
  }

  lastBlueNodMs = now;
  Serial.println("BLUE detected");
  doBlueNod();
}

bool parseTwoArgCommand(const String& line, String& keyword, String& arg1, int& arg2) {
  int firstSpace = line.indexOf(' ');
  if (firstSpace < 0) return false;
  int secondSpace = line.indexOf(' ', firstSpace + 1);
  if (secondSpace < 0) return false;

  keyword = line.substring(0, firstSpace);
  keyword.trim();

  arg1 = line.substring(firstSpace + 1, secondSpace);
  arg1.trim();
  arg1.toLowerCase();

  String numberText = line.substring(secondSpace + 1);
  numberText.trim();
  arg2 = numberText.toInt();
  return true;
}

bool moveJointTo(const String& joint, int angle) {
  if (joint == "shoulder") {
    applyShoulder(angle);
    return true;
  }
  if (joint == "elbow") {
    applyElbow(angle);
    return true;
  }
  if (joint == "wrist") {
    applyWrist(angle);
    return true;
  }
  if (joint == "wrist_rotation" || joint == "wrist_rotate") {
    applyWristRotation(angle);
    return true;
  }
  if (joint == "grip") {
    applyGrip(angle);
    return true;
  }
  if (joint == "rotate") {
    applyRotateRaw(angle);
    return true;
  }
  return false;
}

bool stepJointBy(const String& joint, int delta) {
  if (joint == "shoulder") {
    applyShoulder(shoulderAngle + delta);
    return true;
  }
  if (joint == "elbow") {
    applyElbow(elbowAngle + delta);
    return true;
  }
  if (joint == "wrist") {
    applyWrist(wristAngle + delta);
    return true;
  }
  if (joint == "wrist_rotation" || joint == "wrist_rotate") {
    applyWristRotation(wristRotationAngle + delta);
    return true;
  }
  if (joint == "grip") {
    applyGrip(gripAngle + delta);
    return true;
  }
  if (joint == "rotate") {
    applyRotateRaw(rotateAngle + delta);
    return true;
  }
  return false;
}

int channelForJoint(const String& joint) {
  if (joint == "shoulder_l") return SHOULDER_L;
  if (joint == "shoulder_r") return SHOULDER_R;
  if (joint == "elbow") return ELBOW;
  if (joint == "wrist") return WRIST;
  if (joint == "wrist_rotation" || joint == "wrist_rotate") return WRIST_ROTATION;
  if (joint == "grip") return GRIP;
  return -1;
}

bool testJointServo(const String& joint) {
  int channel = channelForJoint(joint);
  if (channel < 0) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    servo.setAngle(channel, 60);
    delay(500);
    servo.setAngle(channel, 120);
    delay(500);
  }

  if (joint == "shoulder_l" || joint == "shoulder_r") {
    applyShoulder(shoulderAngle);
  } else if (joint == "elbow") {
    applyElbow(elbowAngle);
  } else if (joint == "wrist") {
    applyWrist(wristAngle);
  } else if (joint == "wrist_rotation" || joint == "wrist_rotate") {
    applyWristRotation(wristRotationAngle);
  } else if (joint == "grip") {
    applyGrip(gripAngle);
  }
  return true;
}

void printKnownI2CProbe() {
  const byte addresses[] = {0x40, 0x70, 0x7F};
  Serial.print("I2C");
  for (byte i = 0; i < sizeof(addresses); i++) {
    byte address = addresses[i];
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(" 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
    }
  }
  Serial.println();
}

void printState() {
  Serial.print("STATE {\"shoulder\":");
  Serial.print(shoulderAngle);
  Serial.print(",\"elbow\":");
  Serial.print(elbowAngle);
  Serial.print(",\"wrist\":");
  Serial.print(wristAngle);
  Serial.print(",\"wrist_rotation\":");
  Serial.print(wristRotationAngle);
  Serial.print(",\"grip\":");
  Serial.print(gripAngle);
  Serial.print(",\"rotate\":");
  Serial.print(rotateAngle);
  Serial.println("}");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "RESET") {
    inputLine = "";
    Serial.println("OK RESET");
    return;
  }

  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  if (cmd == "HOME") {
    Serial.println("OK HOME");
    homeAllSlow();
    return;
  }

  if (cmd == "STOP") {
    Serial.println("OK STOP");
    homeAllSlow();
    return;
  }

  if (cmd == "GET_STATE") {
    printState();
    return;
  }

  if (cmd == "I2C_SCAN") {
    printKnownI2CProbe();
    return;
  }

  if (cmd == "I2C_PROBE") {
    printKnownI2CProbe();
    return;
  }

  if (cmd == "AUTO_BLUE_ON") {
    autoBlueOn();
    return;
  }

  if (cmd == "AUTO_BLUE_OFF") {
    autoBlueOff("MANUAL");
    return;
  }

  if (cmd == "BLUE_NOD_ON") {
    blueNodOn();
    return;
  }

  if (cmd == "BLUE_NOD_OFF") {
    blueNodOff("MANUAL");
    return;
  }

  if (cmd == "AUTO_BLUE_STATUS") {
    Serial.print("AUTO_BLUE ");
    Serial.println(autoBlueEnabled ? "ON" : "OFF");
    return;
  }

  if (cmd == "PICKUP") {
    autoBlueEnabled = false;
    Serial.println("OK PICKUP");
    doPickup();
    return;
  }

  if (cmd == "PLACE") {
    autoBlueEnabled = false;
    Serial.println("OK PLACE");
    doPlace();
    return;
  }

  String keyword;
  String joint;
  int value = 0;
  if (!parseTwoArgCommand(cmd, keyword, joint, value)) {
    Serial.println("ERR bad command");
    return;
  }

  keyword.toUpperCase();

  if (keyword == "MOVE") {
    if (moveJointTo(joint, value)) {
      Serial.print("OK MOVE ");
      Serial.println(joint);
      return;
    }
    Serial.println("ERR unknown joint");
    return;
  }

  if (keyword == "STEP") {
    if (stepJointBy(joint, value)) {
      Serial.print("OK STEP ");
      Serial.println(joint);
      return;
    }
    Serial.println("ERR unknown joint");
    return;
  }

  if (keyword == "SPIN") {
    if (joint == "rotate") {
      applyRotateRaw(rotateAngle + value);
      Serial.println("OK SPIN rotate");
      return;
    }
    Serial.println("ERR SPIN only supports rotate");
    return;
  }

  if (keyword == "STEPPER") {
    if (joint == "move") {
      moveStepper(value);
      Serial.println("OK STEPPER move");
      return;
    }
    if (joint == "enable") {
      setStepperEnabled(value != 0);
      Serial.println(stepperEnabled ? "OK STEPPER enable" : "OK STEPPER disable");
      return;
    }
    Serial.println("ERR STEPPER supports move or enable");
    return;
  }

  if (keyword == "SERVO_TEST") {
    if (testJointServo(joint)) {
      Serial.print("OK SERVO_TEST ");
      Serial.println(joint);
      return;
    }
    Serial.println("ERR unknown joint");
    return;
  }

  Serial.println("ERR unknown command");
}

void setup() {
  Wire.begin();
  Serial.begin(SERIAL_BAUD);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_STEP_PIN, LOW);
  digitalWrite(STEPPER_DIR_PIN, LOW);
  setStepperEnabled(false);
  delay(300);
  Serial.println("BOOT serial ready");

  Serial.println("BOOT servo init");
  servo.init(0x7f);
  Serial.println("BOOT home");
  delay(500);

  homeAll();
  Serial.println("OpenClaw primitive bridge ready");
}

void loop() {
  runAutoBlueLoop();
  runBlueNodLoop();

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }
}
