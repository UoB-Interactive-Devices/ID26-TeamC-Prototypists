#include <Wire.h>
#include "PCA9685.h"

ServoDriver servo;

// 채널 번호는 네 연결에 맞게 바꿔라
const int SHOULDER_L   = 10;   // 좌측 페어
const int SHOULDER_R   = 12;   // 우측 페어
const int ELBOW_L      = 6;   // 좌측 페어
const int ELBOW_R      = 3;   // 우측 페어
const int WRIST_ROT    = 15;   // 일반 180도 서보
const int WRIST_CR     = 9;   // 360도 continuous rotation 서보

const int CENTER = 90;
const int DELTA  = 15;

void safeAngle(int ch, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  servo.setAngle(ch, angle);
}

void stopContinuousServo(int ch) {
  servo.setAngle(ch, 90);   // 대부분 90 근처가 정지
}

void rotateContinuousServoCW(int ch) {
  servo.setAngle(ch, 120);  // 한 방향 회전
}

void rotateContinuousServoCCW(int ch) {
  servo.setAngle(ch, 60);   // 반대 방향 회전
}

void centerAll180Servos() {
  safeAngle(SHOULDER_L, CENTER);
  safeAngle(SHOULDER_R, CENTER);
  safeAngle(ELBOW_L, CENTER);
  safeAngle(ELBOW_R, CENTER);
  safeAngle(WRIST_ROT, CENTER);
}

void stopAll() {
  centerAll180Servos();
  stopContinuousServo(WRIST_CR);
}

void testShoulderPair() {
  Serial.println("Testing shoulder pair");

  // 반대 방향 페어 테스트
  safeAngle(SHOULDER_L, CENTER - DELTA);
  safeAngle(SHOULDER_R, CENTER + DELTA);
  delay(1200);

  centerAll180Servos();
  delay(800);

  safeAngle(SHOULDER_L, CENTER + DELTA);
  safeAngle(SHOULDER_R, CENTER - DELTA);
  delay(1200);

  centerAll180Servos();
  delay(800);
}

void testElbowPair() {
  Serial.println("Testing elbow pair");

  // 네 요청대로 반대 방향 적용
  safeAngle(ELBOW_L, CENTER - DELTA);
  safeAngle(ELBOW_R, CENTER + DELTA);
  delay(1200);

  centerAll180Servos();
  delay(800);

  safeAngle(ELBOW_L, CENTER + DELTA);
  safeAngle(ELBOW_R, CENTER - DELTA);
  delay(1200);

  centerAll180Servos();
  delay(800);
}

void testWristRotate180() {
  Serial.println("Testing wrist rotate 180 servo");

  safeAngle(WRIST_ROT, CENTER - DELTA);
  delay(1200);

  safeAngle(WRIST_ROT, CENTER);
  delay(800);

  safeAngle(WRIST_ROT, CENTER + DELTA);
  delay(1200);

  safeAngle(WRIST_ROT, CENTER);
  delay(800);
}

void testWristContinuous360() {
  Serial.println("Testing wrist continuous servo");

  stopContinuousServo(WRIST_CR);
  delay(1000);

  rotateContinuousServoCW(WRIST_CR);
  delay(1000);

  stopContinuousServo(WRIST_CR);
  delay(1000);

  rotateContinuousServoCCW(WRIST_CR);
  delay(1000);

  stopContinuousServo(WRIST_CR);
  delay(1000);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  servo.init(0x7f);
  delay(1000);

  Serial.println("Start robot arm test");
  stopAll();
  delay(2000);
}

void loop() {
  testShoulderPair();
  testElbowPair();
  testWristRotate180();
  testWristContinuous360();

  Serial.println("Test cycle complete");
  delay(3000);
}
