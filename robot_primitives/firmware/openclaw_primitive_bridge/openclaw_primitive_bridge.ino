#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

const unsigned long SERIAL_BAUD = 115200;
const byte PCA9685_ADDRESS = 0x40;
const int SERVO_FREQ_HZ = 50;
const int SERVO_MIN_US = 600;
const int SERVO_MAX_US = 2400;
const int SHOULDER_LEFT_CHANNEL = 9;
const int SHOULDER_RIGHT_CHANNEL = 11;
const int SHOULDER_SERVO_MIN_PULSE = 150;
const int SHOULDER_SERVO_MAX_PULSE = 600;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

const byte EEPROM_MAGIC = 0x43;
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_JOINTS_ADDR = 1;

const int STEPPER_STEP_PIN = 3;
const int STEPPER_DIR_PIN = 2;
const int STEPPER_ENABLE_PIN = 4;
const unsigned int STEPPER_PULSE_US = 3000;
const int STEPPER_STEPS_PER_DEGREE = 3.1;
const bool STEPPER_HOLD_AFTER_MOVE = false;

const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SHOULDER_MIN_SAFE_ANGLE = 0;
const int SHOULDER_MAX_SAFE_ANGLE = 180;
const int MACRO_STEP_DELAY_MS = 18;
const unsigned long SERVO_MOTION_INTERVAL_MS = 25;
const unsigned long SHOULDER_MOTION_INTERVAL_MS = 55;
const unsigned long PICKUP_FLOOR_SETTLE_MS = 1200;
const unsigned long CARRY_SETTLE_MS = 500;

struct JointConfig {
  const char* name;
  byte channel;
  int minimum;
  int maximum;
  int home;
  int value;
  bool inverted;
  int offset;
};

enum JointIndex {
  JOINT_SHOULDER_L = 0,
  JOINT_SHOULDER_R,
  JOINT_WRIST,
  JOINT_WRIST_ROTATION,
  JOINT_GRIP,
  JOINT_COUNT
};

JointConfig joints[JOINT_COUNT] = {
  {"shoulder_l", SHOULDER_LEFT_CHANNEL, SHOULDER_MIN_SAFE_ANGLE, SHOULDER_MAX_SAFE_ANGLE, 0, 0, false, 0},
  {"shoulder_r", SHOULDER_RIGHT_CHANNEL, SHOULDER_MIN_SAFE_ANGLE, SHOULDER_MAX_SAFE_ANGLE, 0, 0, false, 0},
  {"wrist", 5, 0, 180, 120, 120, false, 0},
  {"wrist_rotation", 3, 0, 180, 90, 90, false, 0},
  {"grip", 1, 0, 180, 90, 90, false, 0},
};

const int HOME_ROTATE = 90;
const int PICKUP_OPEN_GRIP = 160;
const int PICKUP_CLOSE_GRIP = 3;
const int PICKUP_SHOULDER = 150;
const int PICKUP_WRIST = 140;
const int PICKUP_SHAKE_LEFT = 25;
const int PICKUP_SHAKE_RIGHT = 155;
const int PICKUP_SHAKE_CENTER = 90;
const int PICKUP_SHAKE_REPEAT = 2;
const int CARRY_SHOULDER = 60;
const int CARRY_WRIST = 170;
const int PLACE_SHOULDER = 60;
const int PLACE_WRIST = 130;
const int PLACE_OPEN_GRIP = 180;
const int FEED_BASE_DELTA = 45;

String inputLine = "";
int rotateAngle = HOME_ROTATE;
bool stepperEnabled = false;
int jointTargets[JOINT_COUNT];
bool motionEnabled = true;
unsigned long lastServoMotionMs = 0;
unsigned long lastShoulderMotionMs = 0;

int clampInt(int value, int minimumValue, int maximumValue) {
  if (value < minimumValue) return minimumValue;
  if (value > maximumValue) return maximumValue;
  return value;
}

int physicalAngle(const JointConfig& joint, int logicalAngle) {
  int angle = joint.inverted ? (180 - logicalAngle) : logicalAngle;
  return clampInt(angle + joint.offset, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

int angleToMicroseconds(int angle) {
  int safeAngle = clampInt(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  return map(safeAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_US, SERVO_MAX_US);
}

int shoulderAngleToPulse(int angle) {
  int safeAngle = clampInt(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  return map(safeAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SHOULDER_SERVO_MIN_PULSE, SHOULDER_SERVO_MAX_PULSE);
}

void writeServoAngle(byte channel, int angle) {
  byte safeChannel = (byte)clampInt(channel, 0, 15);
  pwm.writeMicroseconds(safeChannel, angleToMicroseconds(angle));
}

void writeShoulders(int angle) {
  int leftAngle = clampInt(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int rightAngle = clampInt(180 - angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  pwm.setPWM(joints[JOINT_SHOULDER_L].channel, 0, shoulderAngleToPulse(leftAngle));
  pwm.setPWM(joints[JOINT_SHOULDER_R].channel, 0, shoulderAngleToPulse(rightAngle));

  joints[JOINT_SHOULDER_L].value = leftAngle;
  joints[JOINT_SHOULDER_R].value = leftAngle;
}

void writeJointIndex(byte index, int angle) {
  if (index == JOINT_SHOULDER_L || index == JOINT_SHOULDER_R) {
    writeShoulders(angle);
    return;
  }

  JointConfig& joint = joints[index];
  joint.value = clampInt(angle, joint.minimum, joint.maximum);
  writeServoAngle(joint.channel, physicalAngle(joint, joint.value));
}

void syncTargetsToCurrent() {
  for (byte i = 0; i < JOINT_COUNT; i++) {
    jointTargets[i] = joints[i].value;
  }
}

void savePersistentState() {
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  for (byte i = 0; i < JOINT_COUNT; i++) {
    EEPROM.update(EEPROM_JOINTS_ADDR + i, (byte)clampInt(jointTargets[i], 0, 180));
  }
}

void loadPersistentState() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) {
    syncTargetsToCurrent();
    savePersistentState();
    return;
  }

  for (byte i = 0; i < JOINT_COUNT; i++) {
    int saved = EEPROM.read(EEPROM_JOINTS_ADDR + i);
    joints[i].value = clampInt(saved, joints[i].minimum, joints[i].maximum);
  }
  rotateAngle = HOME_ROTATE;
  syncTargetsToCurrent();
}

void setJointTargetIndex(byte index, int angle) {
  jointTargets[index] = clampInt(angle, joints[index].minimum, joints[index].maximum);
  motionEnabled = true;
  savePersistentState();
}

void setShoulderTarget(int angle) {
  int safeAngle = clampInt(angle, joints[JOINT_SHOULDER_L].minimum, joints[JOINT_SHOULDER_L].maximum);
  jointTargets[JOINT_SHOULDER_L] = safeAngle;
  jointTargets[JOINT_SHOULDER_R] = safeAngle;
  motionEnabled = true;
  savePersistentState();
}

bool servoMotionActive() {
  for (byte i = 0; i < JOINT_COUNT; i++) {
    if (joints[i].value != jointTargets[i]) {
      return true;
    }
  }
  return false;
}

bool shoulderMotionActive() {
  return joints[JOINT_SHOULDER_L].value != jointTargets[JOINT_SHOULDER_L];
}

void updateServoMotion() {
  if (!motionEnabled) return;

  unsigned long now = millis();
  bool canMoveServo = now - lastServoMotionMs >= SERVO_MOTION_INTERVAL_MS;
  bool canMoveShoulder = now - lastShoulderMotionMs >= SHOULDER_MOTION_INTERVAL_MS;
  if (!canMoveServo && !canMoveShoulder) {
    return;
  }

  bool movedServo = false;
  bool shoulderActive = shoulderMotionActive();
  for (byte i = 0; i < JOINT_COUNT; i++) {
    if (joints[i].value == jointTargets[i]) {
      continue;
    }

    if (i == JOINT_SHOULDER_R) {
      continue;
    }

    if (i == JOINT_SHOULDER_L) {
      if (!canMoveShoulder) {
        continue;
      }
      int nextValue = joints[i].value + (joints[i].value < jointTargets[i] ? 1 : -1);
      writeJointIndex(i, nextValue);
      lastShoulderMotionMs = now;
      continue;
    }

    if (shoulderActive) {
      continue;
    }

    if (!canMoveServo) {
      continue;
    }
    int nextValue = joints[i].value + (joints[i].value < jointTargets[i] ? 1 : -1);
    writeJointIndex(i, nextValue);
    movedServo = true;
  }

  if (movedServo) {
    lastServoMotionMs = now;
  }
}

void stopServoMotion() {
  syncTargetsToCurrent();
  motionEnabled = false;
  savePersistentState();
}

void waitForServoMotion() {
  while (servoMotionActive()) {
    updateServoMotion();
  }
}

void writeRawChannel(byte channel, int angle) {
  writeServoAngle(channel, angle);
}

int findJointIndex(const String& rawName) {
  String name = rawName;
  name.trim();
  name.toLowerCase();
  if (name == "shoulder") return JOINT_SHOULDER_L;
  if (name == "shoulder_l" || name == "left_shoulder") return JOINT_SHOULDER_L;
  if (name == "shoulder_r" || name == "right_shoulder") return JOINT_SHOULDER_R;
  if (name == "wrist") return JOINT_WRIST;
  if (name == "wrist_rotation" || name == "wrist_rotate") return JOINT_WRIST_ROTATION;
  if (name == "grip" || name == "gripper") return JOINT_GRIP;
  return -1;
}

void applyShoulder(int angle) {
  int safeAngle = clampInt(angle, joints[JOINT_SHOULDER_L].minimum, joints[JOINT_SHOULDER_L].maximum);
  writeJointIndex(JOINT_SHOULDER_L, safeAngle);
  writeJointIndex(JOINT_SHOULDER_R, safeAngle);
  setShoulderTarget(safeAngle);
}

void setStepperEnabled(bool enabled) {
  stepperEnabled = enabled;
  digitalWrite(STEPPER_ENABLE_PIN, enabled ? LOW : HIGH);
}

void moveStepper(long steps) {
  if (steps == 0) return;

  bool wasEnabled = stepperEnabled;
  if (!wasEnabled) {
    setStepperEnabled(true);
    delay(2);
  }

  digitalWrite(STEPPER_DIR_PIN, steps >= 0 ? HIGH : LOW);
  unsigned long pulseCount = labs(steps);
  for (unsigned long i = 0; i < pulseCount; i++) {
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
  int deltaAngle = angle - rotateAngle;
  moveStepper(deltaAngle * STEPPER_STEPS_PER_DEGREE);
  rotateAngle = angle;
  savePersistentState();
}

void moveLogicalJointSlow(byte index, int targetValue) {
  setJointTargetIndex(index, targetValue);
  waitForServoMotion();
}

void moveShoulderSlow(int targetValue) {
  setShoulderTarget(targetValue);
  waitForServoMotion();
}

void moveRotateSlow(int targetValue) {
  while (rotateAngle != targetValue) {
    int nextValue = rotateAngle + (rotateAngle < targetValue ? 1 : -1);
    applyRotateRaw(nextValue);
    delay(MACRO_STEP_DELAY_MS);
  }
}

void homeAll(bool slow) {
  if (slow) {
    setJointTargetIndex(JOINT_GRIP, joints[JOINT_GRIP].home);
    setJointTargetIndex(JOINT_WRIST, joints[JOINT_WRIST].home);
    setJointTargetIndex(JOINT_WRIST_ROTATION, joints[JOINT_WRIST_ROTATION].home);
    setShoulderTarget(joints[JOINT_SHOULDER_L].home);
    waitForServoMotion();
    moveRotateSlow(HOME_ROTATE);
    return;
  }

  writeJointIndex(JOINT_GRIP, joints[JOINT_GRIP].home);
  writeJointIndex(JOINT_WRIST, joints[JOINT_WRIST].home);
  writeJointIndex(JOINT_WRIST_ROTATION, joints[JOINT_WRIST_ROTATION].home);
  applyShoulder(joints[JOINT_SHOULDER_L].home);
  rotateAngle = HOME_ROTATE;
  syncTargetsToCurrent();
}

void moveToCarryPose() {
  setShoulderTarget(CARRY_SHOULDER);
  waitForServoMotion();
  setJointTargetIndex(JOINT_WRIST, CARRY_WRIST);
  waitForServoMotion();
  delay(CARRY_SETTLE_MS);
  moveLogicalJointSlow(JOINT_GRIP, PICKUP_CLOSE_GRIP);
}

void moveToPickupFloorPose() {
  setShoulderTarget(PICKUP_SHOULDER);
  waitForServoMotion();
  setJointTargetIndex(JOINT_WRIST, PICKUP_WRIST);
  waitForServoMotion();
  delay(PICKUP_FLOOR_SETTLE_MS);
}

void shakeWristRotationLarge() {
  for (int i = 0; i < PICKUP_SHAKE_REPEAT; i++) {
    moveLogicalJointSlow(JOINT_WRIST_ROTATION, PICKUP_SHAKE_LEFT);
    delay(120);
    moveLogicalJointSlow(JOINT_WRIST_ROTATION, PICKUP_SHAKE_RIGHT);
    delay(120);
  }
  moveLogicalJointSlow(JOINT_WRIST_ROTATION, PICKUP_SHAKE_CENTER);
}

void doPickup() {
  moveLogicalJointSlow(JOINT_WRIST_ROTATION, PICKUP_SHAKE_CENTER);
  delay(100);

  moveLogicalJointSlow(JOINT_GRIP, PICKUP_OPEN_GRIP);
  delay(150);

  setShoulderTarget(PICKUP_SHOULDER);
  waitForServoMotion();
  delay(150);

  setJointTargetIndex(JOINT_WRIST, PICKUP_WRIST);
  waitForServoMotion();
  delay(PICKUP_FLOOR_SETTLE_MS);

  moveLogicalJointSlow(JOINT_GRIP, PICKUP_CLOSE_GRIP);
  delay(150);

  setShoulderTarget(CARRY_SHOULDER);
  waitForServoMotion();
  delay(150);

  setJointTargetIndex(JOINT_WRIST, CARRY_WRIST);
  waitForServoMotion();
  delay(CARRY_SETTLE_MS);

  delay(150);
  shakeWristRotationLarge();
}

void doPlace() {
  setShoulderTarget(PLACE_SHOULDER);
  waitForServoMotion();
  setJointTargetIndex(JOINT_WRIST, PLACE_WRIST);
  waitForServoMotion();
  setShoulderTarget(CARRY_SHOULDER);
  waitForServoMotion();
  delay(120);
  moveLogicalJointSlow(JOINT_GRIP, PLACE_OPEN_GRIP);
}

void doFeed() {
  int feedStartAngle = rotateAngle;
  moveRotateSlow(feedStartAngle + FEED_BASE_DELTA);
  doPickup();
  moveRotateSlow(feedStartAngle);
  doPlace();
}

bool parseTwoArgCommand(const String& line, String& keyword, String& arg1, long& arg2) {
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

bool moveJointTo(const String& jointName, int angle) {
  String name = jointName;
  name.trim();
  name.toLowerCase();
  if (name == "rotate" || name == "base") {
    applyRotateRaw(angle);
    return true;
  }
  if (name == "shoulder") {
    setShoulderTarget(angle);
    return true;
  }

  int index = findJointIndex(name);
  if (index < 0) return false;
  if (index == JOINT_SHOULDER_L || index == JOINT_SHOULDER_R) {
    setShoulderTarget(angle);
    return true;
  }
  setJointTargetIndex((byte)index, angle);
  return true;
}

bool stepJointBy(const String& jointName, int delta) {
  String name = jointName;
  name.trim();
  name.toLowerCase();
  if (name == "rotate" || name == "base") {
    applyRotateRaw(rotateAngle + delta);
    return true;
  }
  if (name == "shoulder") {
    setShoulderTarget(jointTargets[JOINT_SHOULDER_L] + delta);
    return true;
  }

  int index = findJointIndex(name);
  if (index < 0) return false;
  if (index == JOINT_SHOULDER_L || index == JOINT_SHOULDER_R) {
    setShoulderTarget(jointTargets[JOINT_SHOULDER_L] + delta);
    return true;
  }
  setJointTargetIndex((byte)index, jointTargets[index] + delta);
  return true;
}

bool applyMoveMulti(String args) {
  args.trim();
  bool appliedAny = false;

  while (args.length() > 0) {
    int firstSpace = args.indexOf(' ');
    if (firstSpace < 0) return false;

    String jointName = args.substring(0, firstSpace);
    jointName.trim();
    args = args.substring(firstSpace + 1);
    args.trim();

    int secondSpace = args.indexOf(' ');
    String valueText;
    if (secondSpace < 0) {
      valueText = args;
      args = "";
    } else {
      valueText = args.substring(0, secondSpace);
      args = args.substring(secondSpace + 1);
    }
    valueText.trim();

    if (!moveJointTo(jointName, valueText.toInt())) {
      return false;
    }
    appliedAny = true;
  }

  return appliedAny;
}

void printI2CScan() {
  Serial.print("I2C");
  for (byte address = 1; address < 127; address++) {
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

void printChannels() {
  Serial.print("CHANNELS");
  for (byte i = 0; i < JOINT_COUNT; i++) {
    Serial.print(" ");
    Serial.print(joints[i].name);
    Serial.print("=");
    Serial.print(joints[i].channel);
  }
  Serial.println();
}

void printState() {
  Serial.print("STATE {\"shoulder\":");
  Serial.print(joints[JOINT_SHOULDER_L].value);
  Serial.print(",\"shoulder_l\":");
  Serial.print(joints[JOINT_SHOULDER_L].value);
  Serial.print(",\"shoulder_r\":");
  Serial.print(joints[JOINT_SHOULDER_R].value);
  Serial.print(",\"wrist\":");
  Serial.print(joints[JOINT_WRIST].value);
  Serial.print(",\"wrist_rotation\":");
  Serial.print(joints[JOINT_WRIST_ROTATION].value);
  Serial.print(",\"grip\":");
  Serial.print(joints[JOINT_GRIP].value);
  Serial.print(",\"rotate\":");
  Serial.print(rotateAngle);
  Serial.println("}");
}

bool testJointServo(const String& jointName) {
  int index = findJointIndex(jointName);
  if (index < 0) return false;

  int original = joints[index].value;
  writeJointIndex(index, 75);
  delay(450);
  writeJointIndex(index, 105);
  delay(450);
  writeJointIndex(index, original);
  return true;
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
    homeAll(true);
    return;
  }

  if (cmd == "STOP") {
    stopServoMotion();
    setStepperEnabled(false);
    Serial.println("OK STOP");
    return;
  }

  if (cmd == "GET_STATE") {
    printState();
    return;
  }

  if (cmd == "I2C_SCAN" || cmd == "I2C_PROBE") {
    printI2CScan();
    return;
  }

  if (cmd == "CHANNELS") {
    printChannels();
    return;
  }

  if (cmd == "PICKUP" || cmd == "PICKUP_1") {
    Serial.println("OK PICKUP");
    doPickup();
    return;
  }

  if (cmd == "PICKUP_2") {
    Serial.println("OK PICKUP_2");
    moveRotateSlow(120);
    doPickup();
    return;
  }

  if (cmd == "PLACE" || cmd == "PLACE_1" || cmd == "PLACE_2") {
    Serial.println("OK PLACE");
    doPlace();
    return;
  }

  if (cmd == "FEED") {
    Serial.println("OK FEED");
    doFeed();
    return;
  }

  if (cmd == "AUTO_BLUE_ON" || cmd == "BLUE_NOD_ON") {
    Serial.println("ERR PIXY_DISABLED use PC webcam control");
    return;
  }

  if (cmd == "AUTO_BLUE_OFF") {
    Serial.println("OK AUTO_BLUE_OFF DISABLED");
    return;
  }

  if (cmd == "BLUE_NOD_OFF") {
    Serial.println("OK BLUE_NOD_OFF DISABLED");
    return;
  }

  if (cmd == "AUTO_BLUE_STATUS") {
    Serial.println("AUTO_BLUE OFF");
    return;
  }

  if (cmd.startsWith("MOVE_MULTI ")) {
    if (applyMoveMulti(cmd.substring(11))) {
      Serial.println("OK MOVE_MULTI");
      return;
    }
    Serial.println("ERR MOVE_MULTI expects joint angle pairs");
    return;
  }

  String keyword;
  String arg1;
  long value = 0;
  if (!parseTwoArgCommand(cmd, keyword, arg1, value)) {
    Serial.println("ERR bad command");
    return;
  }

  keyword.toUpperCase();

  if (keyword == "MOVE") {
    if (moveJointTo(arg1, (int)value)) {
      Serial.print("OK MOVE ");
      Serial.println(arg1);
      return;
    }
    Serial.println("ERR unknown joint");
    return;
  }

  if (keyword == "STEP") {
    if (stepJointBy(arg1, (int)value)) {
      Serial.print("OK STEP ");
      Serial.println(arg1);
      return;
    }
    Serial.println("ERR unknown joint");
    return;
  }

  if (keyword == "SPIN") {
    if (arg1 == "rotate") {
      applyRotateRaw(rotateAngle + (int)value);
      Serial.println("OK SPIN rotate");
      return;
    }
    Serial.println("ERR SPIN only supports rotate");
    return;
  }

  if (keyword == "STEPPER") {
    if (arg1 == "move") {
      moveStepper(value);
      Serial.println("OK STEPPER move");
      return;
    }
    if (arg1 == "enable") {
      setStepperEnabled(value != 0);
      Serial.println(stepperEnabled ? "OK STEPPER enable" : "OK STEPPER disable");
      return;
    }
    Serial.println("ERR STEPPER supports move or enable");
    return;
  }

  if (keyword == "SERVO_TEST") {
    if (testJointServo(arg1)) {
      Serial.print("OK SERVO_TEST ");
      Serial.println(arg1);
      return;
    }
    Serial.println("ERR unknown joint");
    return;
  }

  if (keyword == "RAW") {
    int channel = clampInt(arg1.toInt(), 0, 15);
    writeRawChannel((byte)channel, (int)value);
    Serial.println("OK RAW");
    return;
  }

  if (keyword == "CHANNEL") {
    int index = findJointIndex(arg1);
    if (index < 0) {
      Serial.println("ERR unknown joint");
      return;
    }
    joints[index].channel = (byte)clampInt((int)value, 0, 15);
    Serial.print("OK CHANNEL ");
    Serial.println(arg1);
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
  Serial.println("BOOT adafruit pwm init");
  if (!pwm.begin()) {
    Serial.println("ERR PWM no response");
  }
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(300);

  loadPersistentState();
  Serial.println("OpenClaw primitive bridge ready");
}

void loop() {
  updateServoMotion();

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

  updateServoMotion();
}
