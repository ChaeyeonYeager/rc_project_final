#include <Arduino.h>
#include <Servo.h>

// ─────────────────────────────────────────────
// 핀 정의
const int ESC_PIN   = 5;   // ESC 제어용 핀
const int SERVO_PIN = 9;   // 스티어링 서보 제어용 핀

// ─────────────────────────────────────────────
// 전진·후진 제한값 (µs)
const int MAX_REVERSE = 1421;
const int MAX_FORWARD = 1545;

// 기본 정지 PWM값
const int ESC_NEUTRAL = 1500;

// 유효한 제어 각도 범위
const int MIN_ANGLE = 45;
const int MAX_ANGLE = 135;

// ─────────────────────────────────────────────
// Recovery 상태 머신 정의
enum RecoveryState {
  STATE_NORMAL,
  STATE_STOP,
  STATE_REVERSING,
  STATE_FORWARDING
};
RecoveryState state = STATE_NORMAL;

int lastValidAngle = 90;
unsigned long reverseStartTime = 0;

// ─────────────────────────────────────────────
// 모터/서보 객체
Servo esc;
Servo steer;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);  // 시리얼 입력 대기 시간 설정

  esc.attach(ESC_PIN);
  steer.attach(SERVO_PIN);

  esc.writeMicroseconds(ESC_NEUTRAL);
  steer.write(90);

  Serial.println("[DEBUG] 초기화 완료: STATE_NORMAL 대기 중");
  delay(2000);
}

void loop() {
  // Serial로부터 rawAngle 읽기
  int rawAngle = INT32_MIN;
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      rawAngle = input.toInt();
      Serial.print("[DEBUG] 수신된 angle = ");
      Serial.println(rawAngle);
    }
  }

  // ─────────────────────────────────────────────
  switch (state) {
    case STATE_NORMAL:
      if (rawAngle == -1) {
        Serial.println("[DEBUG] STATE_NORMAL → 라인 미검출 → STATE_STOP");
        state = STATE_STOP;
      }
      else if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
        lastValidAngle = 180 - rawAngle;
        Serial.print("[DEBUG] STATE_NORMAL → 라인 감지, 뒤집은 각도 = ");
        Serial.println(lastValidAngle);
        steer.write(lastValidAngle);
        esc.writeMicroseconds(MAX_FORWARD);
      }
      else {
        steer.write(lastValidAngle);
        esc.writeMicroseconds(MAX_FORWARD);
      }
      break;

    case STATE_STOP:
      Serial.println("[DEBUG] STATE_STOP → 즉시 정지");
      steer.write(90);
      esc.writeMicroseconds(ESC_NEUTRAL);
      state = STATE_REVERSING;
      reverseStartTime = millis();
      Serial.println("[DEBUG] STATE_STOP → STATE_REVERSING 전환");
      break;

    case STATE_REVERSING:
      steer.write(90);
      esc.writeMicroseconds(MAX_REVERSE);
      Serial.println("[DEBUG] STATE_REVERSING → 후진 중");

      if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
        lastValidAngle = 180 - rawAngle;
        Serial.print("[DEBUG] STATE_REVERSING → 라인 재감지, 뒤집은 각도 = ");
        Serial.println(lastValidAngle);
        steer.write(lastValidAngle);
        esc.writeMicroseconds(MAX_FORWARD);
        state = STATE_NORMAL;
        Serial.println("[DEBUG] STATE_REVERSING → STATE_NORMAL 전환");
      }
      else if (millis() - reverseStartTime >= 2000) {
        Serial.println("[DEBUG] STATE_REVERSING → 2초 후진 완료 → STATE_FORWARDING");
        state = STATE_FORWARDING;
      }
      break;

    case STATE_FORWARDING:
      steer.write(lastValidAngle);
      esc.writeMicroseconds(MAX_FORWARD);
      Serial.print("[DEBUG] STATE_FORWARDING → 전진 중, 각도 = ");
      Serial.println(lastValidAngle);

      if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
        lastValidAngle = 180 - rawAngle;
        Serial.print("[DEBUG] STATE_FORWARDING → 라인 감지, 각도 업데이트 = ");
        Serial.println(lastValidAngle);
      }
      else if (rawAngle == -1) {
        Serial.println("[DEBUG] STATE_FORWARDING → 라인 미검출 → STATE_STOP");
        state = STATE_STOP;
      }
      break;
  }

  delay(10);
}
