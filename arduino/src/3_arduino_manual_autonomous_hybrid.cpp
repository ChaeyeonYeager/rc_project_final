#include <Arduino.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>

// ─────────────────────────────────────────────
// 핀 정의
const int pinSteering    = A0;  // CH1: 조향(PPM 입력)
const int pinThrottle    = A1;  // CH2: 스로틀(PPM 입력)
const int pinCh5         = A2;  // CH5: A 스위치(PPM 입력)

const int ESC_PIN        = 5;   // ESC 제어 핀
const int SERVO_PIN      = 9;   // 조향 서보 핀

const int LED_LEFT_PIN   = 6;   // 좌측 LED (Active-High: HIGH=켜짐, LOW=꺼짐)
const int LED_RIGHT_PIN  = 7;   // 우측 LED (Active-Low: LOW=켜짐, HIGH=꺼짐)

// ─────────────────────────────────────────────
// RC(PPM) 수신값 변수
volatile int steerPulse     = 1500; // CH1 펄스 폭 (μs)
volatile int throttlePulse  = 1500; // CH2 펄스 폭 (μs)
volatile int ch5Pulse       = 1500; // CH5 펄스 폭 (μs)

volatile unsigned long steerStart     = 0;
volatile unsigned long throttleStart  = 0;
volatile unsigned long ch5Start       = 0;

volatile bool newSteer      = false;   // CH1 폭 측정 플래그
volatile bool newThrottle   = false;   // CH2 폭 측정 플래그
volatile bool newCh5        = false;   // CH5 폭 측정 플래그

// ─────────────────────────────────────────────
// 모터/서보 객체
Servo esc;
Servo steerServo;

// ─────────────────────────────────────────────
// LED 깜빡임 제어용 (수동 모드 전진 때만)
const unsigned long BLINK_INTERVAL = 500; // 500ms 간격
unsigned long lastBlinkToggle = 0;
bool blinkState = false;

// ─────────────────────────────────────────────
// ─── 자동 모드용 Recovery 상태 머신 ────────────
// 유효한 제어 각도 범위
const int MIN_ANGLE = 45;
const int MAX_ANGLE = 135;

// 자동 모드 ESC 속도 상수 (µs)
const int MAX_FORWARD_AUTO = 1545;
const int MAX_REVERSE_AUTO = 1421;
const int ESC_NEUTRAL_US   = 1500;

// 상태 머신 정의
enum RecoveryState {
  STATE_NORMAL,     // 정상: 라인 감지 시마다 lastValidAngle 업데이트 → 전진
  STATE_STOP,       // 라인 미검출 직후: 즉시 정지 → REVERSING으로 전환
  STATE_REVERSING,  // 후진 중 (2초 간) → 복구 시 전진 또는 2초 후 FORWARDING
  STATE_FORWARDING  // 복구 후 전진 상태: 전진 중 다시 장애 발생 시 STATE_STOP
};

RecoveryState state = STATE_NORMAL;

// 마지막 유효(라인 감지) 각도 (뒤집은 각도로 저장)
int lastValidAngle = 90;  // 초깃값: 중앙(90°)

// 후진 시작 시각 (millis)
unsigned long reverseStartTime = 0;

// “시리얼로 한 번도 data를 받은 적이 없는 상태” 플래그
bool firstSerialReceived = false;

// Auto 모드에서 전송된 raw 각도
int rawAngle = -9999;  // 초기값: 유효 범위 밖

// ─────────────────────────────────────────────
// 이전 모드 저장 (모드 전환 감지용)
bool prevManualMode = false;

// ─────────────────────────────────────────────
// CH1 (Steer) 인터럽트 핸들러
void isrSteer() {
  if (digitalRead(pinSteering) == HIGH) {
    steerStart = micros();
  } else {
    steerPulse = micros() - steerStart;
    newSteer = true;
  }
}

// CH2 (Throttle) 인터럽트 핸들러
void isrThrottle() {
  if (digitalRead(pinThrottle) == HIGH) {
    throttleStart = micros();
  } else {
    throttlePulse = micros() - throttleStart;
    newThrottle = true;
  }
}

// CH5 (A 스위치) 인터럽트 핸들러
void isrCh5() {
  if (digitalRead(pinCh5) == HIGH) {
    ch5Start = micros();
  } else {
    ch5Pulse = micros() - ch5Start;
    newCh5 = true;
  }
}

// ─────────────────────────────────────────────
// LED 제어 헬퍼 함수
inline void turnOnLeft()   { digitalWrite(LED_LEFT_PIN, HIGH); }   // Active-High
inline void turnOffLeft()  { digitalWrite(LED_LEFT_PIN, LOW);  }

inline void turnOnRight()  { digitalWrite(LED_RIGHT_PIN, LOW);  }   // Active-Low
inline void turnOffRight() { digitalWrite(LED_RIGHT_PIN, HIGH); }

// ─────────────────────────────────────────────
void setup() {
  // 1) 시리얼 통신 (Raspberry Pi ↔ Arduino)
  Serial.begin(9600);
  Serial.setTimeout(10);  // readStringUntil() 타임아웃을 10ms로 설정

  // 2) RC(PPM) 입력 핀 설정
  pinMode(pinSteering, INPUT_PULLUP);
  pinMode(pinThrottle, INPUT_PULLUP);
  pinMode(pinCh5, INPUT_PULLUP);

  // 3) 인터럽트 연결
  attachPinChangeInterrupt(digitalPinToPCINT(pinSteering), isrSteer, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(pinThrottle), isrThrottle, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(pinCh5), isrCh5, CHANGE);

  // 4) ESC/서보 초기화
  esc.attach(ESC_PIN);
  steerServo.attach(SERVO_PIN);
  esc.writeMicroseconds(ESC_NEUTRAL_US);  // 정지(중립)
  steerServo.write(90);                   // 중립(90°)

  // 5) LED 초기화
  pinMode(LED_LEFT_PIN, OUTPUT);
  pinMode(LED_RIGHT_PIN, OUTPUT);
  turnOffLeft();
  turnOffRight();

  // 6) 인터럽트로 한 번이라도 ch5Pulse가 갱신될 수 있도록 잠시 대기
  delay(200);
  prevManualMode = (ch5Pulse > 1500);
  Serial.print("[DEBUG] 초기 prevManualMode = ");
  Serial.println(prevManualMode ? "MANUAL" : "AUTO");

  // 7) 자동 모드용 변수 초기화
  lastValidAngle = 90;
  firstSerialReceived = false;
  rawAngle = -9999;
  state = STATE_NORMAL;

  delay(3000);  // ESC 초기화 대기
  Serial.println("[DEBUG] 시작 준비 완료");
}

void loop() {
  // ─────────────────────────────────────────────────────────────────
  // (A) CH5(스위치) 값이 바뀌면 디버깅: CH5 펄스 폭과 모드 출력
  if (newCh5) {
    newCh5 = false;
    Serial.print("[DEBUG] CH5 Pulse -> ");
    Serial.print(ch5Pulse);
    Serial.print("   (manualMode? ");
    Serial.print((ch5Pulse > 1500) ? "YES" : "NO");
    Serial.println(" )");
  }

  // (B) 현재 모드 판단
  bool manualMode = (ch5Pulse > 1500);

  // ─────────────────────────────────────────────────────────────────
  // (C) 모드 전환 감지: prevManualMode와 비교해서 달라지면 → 모드 전환 처리
  if (manualMode != prevManualMode) {
    if (manualMode && !prevManualMode) {
      // 자동 → 수동
      Serial.println("[DEBUG] >>> 자동 모드에서 수동 모드로 전환됨");
      newSteer = false;
      newThrottle = false;
    }
    else if (!manualMode && prevManualMode) {
      // 수동 → 자동(Recovery) 모드
      Serial.println("[DEBUG] >>> 수동 모드에서 자동(Recovery) 모드로 전환됨");
      // 자동 모드 변수 초기화
      firstSerialReceived = false;
      rawAngle = -9999;
      lastValidAngle = 90;
      state = STATE_NORMAL;
    }
    prevManualMode = manualMode;
  }

  // ─────────────────────────────────────────────────────────────────
  // [1] 수동 모드 분기 (RC PPM 제어 + LED 깜빡임)
  if (manualMode) {
    Serial.println("[DEBUG] 현재 모드: MANUAL");
    if (newSteer || newThrottle) {
      newSteer = false;
      newThrottle = false;

      // 1) 조향 처리
      int steerAngle = map(steerPulse, 1000, 2000, 30, 150);
      steerServo.write(steerAngle);

      // 2) 속도 제어
      int escPulse;
      bool isReversing = false;
      static unsigned long reverseStartTime_manual = 0;
      static bool reverseModeTriggered_manual = false;

      if (throttlePulse >= 1500) {
        // 전진
        escPulse = map(throttlePulse, 1500, 2000, 1500, 1560);
        isReversing = false;
        reverseModeTriggered_manual = false;
      } else {
        // 후진
        isReversing = true;
        if (!reverseModeTriggered_manual) {
          escPulse = ESC_NEUTRAL_US;
          reverseModeTriggered_manual = true;
          reverseStartTime_manual = millis();
        }
        else if (millis() - reverseStartTime_manual > 10) {
          escPulse = map(throttlePulse, 1000, 1500, 1400, 1500);
        }
        else {
          escPulse = ESC_NEUTRAL_US;
        }
      }
      esc.writeMicroseconds(escPulse);

      // 3) LED 제어 (디버깅 출력 포함)
      if (isReversing) {
        // 후진 중: 양쪽 LED 모두 ON
        turnOnLeft();
        turnOnRight();
        blinkState = false;
        Serial.println("[DEBUG] 수동 후진 중 → LED 양쪽 ON");
      }
      else {
        // 전진/중립 중: steerAngle에 따라 깜빡임
        unsigned long now = millis();
        if (now - lastBlinkToggle >= BLINK_INTERVAL) {
          blinkState = !blinkState;
          lastBlinkToggle = now;
        }

        if (steerAngle < 65) {
          // 좌회전: 좌측 LED만 깜빡임, 우측은 무조건 끔
          turnOffRight();
          if (blinkState)  turnOnLeft();
          else             turnOffLeft();
          Serial.println("[DEBUG] 수동 전진(좌회전) → LEFT LED 깜빡임");
        }
        else if (steerAngle > 115) {
          // 우회전: 우측 LED만 깜빡임, 좌측은 무조건 끔
          turnOffLeft();
          if (blinkState)  turnOnRight();
          else             turnOffRight();
          Serial.println("[DEBUG] 수동 전진(우회전) → RIGHT LED 깜빡임");
        }
        else {
          // 중립: 양쪽 LED 모두 OFF
          blinkState = false;
          turnOffLeft();
          turnOffRight();
          Serial.println("[DEBUG] 수동 전진(중립) → LED OFF");
        }
      }
    }
    return;  // 수동 모드면 여기서 loop 종료
  }

  // ─────────────────────────────────────────────────────────────────
  // [2] 자동 모드 분기 (Recovery 상태 머신)
  Serial.println("[DEBUG] 현재 모드: AUTO (Recovery)");

  // (D) Serial로부터 rawAngle 읽기 (Non-blocking)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      rawAngle = input.toInt();
      firstSerialReceived = true;
      Serial.print("[DEBUG] 수신된 rawAngle = ");
      Serial.println(rawAngle);
    }
  }

  // (E) Recovery 상태 머신
  switch (state) {

    // ─────────────────────────────────────────────────────────────────
    case STATE_NORMAL:
      // 정상 상태: 라인 감지 시 lastValidAngle 업데이트 → 전진
      // 라인 미검출(-1) 발견 시 → STATE_STOP 으로 전환
      if (rawAngle == -1) {
        Serial.println("[DEBUG] STATE_NORMAL → 라인 미검출 → STATE_STOP");
        state = STATE_STOP;
      }
      else if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
        // 유효한 angle 수신: 뒤집어서 저장
        lastValidAngle = 180 - rawAngle;
        Serial.print("[DEBUG] STATE_NORMAL → 유효 angle, 뒤집은 각도 = ");
        Serial.println(lastValidAngle);

        // 전진: steer = lastValidAngle, esc = MAX_FORWARD_AUTO
        steerServo.write(lastValidAngle);
        esc.writeMicroseconds(MAX_FORWARD_AUTO);
      }
      else {
        // 그 외(못 받은 상태)인 경우, 이전 lastValidAngle로 계속 전진
        steerServo.write(lastValidAngle);
        esc.writeMicroseconds(MAX_FORWARD_AUTO);
      }
      break;

    // ─────────────────────────────────────────────────────────────────
    case STATE_STOP:
      // 라인 미검출 직후: 즉시 정지
      Serial.println("[DEBUG] STATE_STOP → 즉시 정지");
      steerServo.write(90);                        // 서보 중앙(정면)
      esc.writeMicroseconds(ESC_NEUTRAL_US);       // ESC 중립(정지)

      // 정지 후 바로 후진 단계로 전환
      state = STATE_REVERSING;
      reverseStartTime = millis();
      Serial.println("[DEBUG] STATE_STOP → STATE_REVERSING 전환");
      break;

    // ─────────────────────────────────────────────────────────────────
    case STATE_REVERSING:
      // 후진 중: steer 중앙(90°), esc = MAX_REVERSE_AUTO
      steerServo.write(90);
      esc.writeMicroseconds(MAX_REVERSE_AUTO);
      Serial.println("[DEBUG] STATE_REVERSING → 후진 중");

      // (가) 후진 중에 유효한 angle 수신 시 즉시 복구
      if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
        lastValidAngle = 180 - rawAngle;
        Serial.print("[DEBUG] STATE_REVERSING → 라인 재감지, 뒤집은 각도 = ");
        Serial.println(lastValidAngle);

        // 전진으로 복귀
        steerServo.write(lastValidAngle);
        esc.writeMicroseconds(MAX_FORWARD_AUTO);
        state = STATE_NORMAL;
        Serial.println("[DEBUG] STATE_REVERSING → STATE_NORMAL 전환");
        break;
      }
      // (나) 아직 2초(2000ms)가 지나지 않은 경우 계속 후진
      if (millis() - reverseStartTime < 2000) {
        // 아무 동작 추가 없이 후진 유지
      }
      else {
        // 2초 후진 완료 → FORWARDING 상태로 전환
        Serial.println("[DEBUG] STATE_REVERSING → 2초 후진 완료 → STATE_FORWARDING");
        state = STATE_FORWARDING;
      }
      break;

    // ─────────────────────────────────────────────────────────────────
    case STATE_FORWARDING:
      // 복구 후 전진 중: steer = lastValidAngle, esc = MAX_FORWARD_AUTO
      steerServo.write(lastValidAngle);
      esc.writeMicroseconds(MAX_FORWARD_AUTO);
      Serial.print("[DEBUG] STATE_FORWARDING → 전진 중 (각도 = ");
      Serial.print(lastValidAngle);
      Serial.println(" )");

      // (가) 전진 중에 유효한 angle 수신 시 각도만 업데이트
      if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
        lastValidAngle = 180 - rawAngle;
        Serial.print("[DEBUG] STATE_FORWARDING → 라인 계속 감지, 뒤집은 각도 업데이트 = ");
        Serial.println(lastValidAngle);
      }
      // (나) 전진 중에 라인 미검출(-1) 발생 시 → 다시 STATE_STOP
      else if (rawAngle == -1) {
        Serial.println("[DEBUG] STATE_FORWARDING → 라인 미검출 → STATE_STOP");
        state = STATE_STOP;
      }
      // 그 외(못 받은 상태)인 경우, 전진 유지
      break;
  }

  // (F) Auto 모드에서는 LED 모두 끔
  turnOffLeft();
  turnOffRight();

  // (G) 짧은 딜레이를 두어 Serial 버퍼 안정화
  delay(10);
}
