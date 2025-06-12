#include <Arduino.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>

// ─────────────────────────────────────────────
// 핀 정의
const int pinSteering = A0;  // 방향 조종 (좌우)
const int pinThrottle = A1;  // 스로틀 (전후진)
const int ESC_PIN = 5;       // ESC 제어 핀
const int SERVO_PIN = 9;     // 서보 제어 핀

// ─────────────────────────────────────────────
// RC 수신값 변수
volatile int steerPulse = 1500;
volatile int throttlePulse = 1500;
volatile unsigned long steerStart = 0;
volatile unsigned long throttleStart = 0;
volatile bool newSteer = false;
volatile bool newThrottle = false;

// ─────────────────────────────────────────────
// 모터 및 서보
Servo esc;
Servo steerServo;

// ─────────────────────────────────────────────
// 인터럽트 핸들러
void isrSteer() {
  if (digitalRead(pinSteering) == HIGH) {
    steerStart = micros();
  } else {
    steerPulse = micros() - steerStart;
    newSteer = true;
  }
}

void isrThrottle() {
  if (digitalRead(pinThrottle) == HIGH) {
    throttleStart = micros();
  } else {
    throttlePulse = micros() - throttleStart;
    newThrottle = true;
  }
}

// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // RC 수신기 핀 설정
  pinMode(pinSteering, INPUT_PULLUP);
  pinMode(pinThrottle, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPCINT(pinSteering), isrSteer, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(pinThrottle), isrThrottle, CHANGE);

  // 모터/서보 초기화
  esc.attach(ESC_PIN);
  steerServo.attach(SERVO_PIN);
  esc.writeMicroseconds(1500);  // 정지
  steerServo.write(90);         // 중립

  delay(3000);  // ESC 초기화 대기
  Serial.println("시작 준비 완료");
}

unsigned long lastThrottleTime = 0;
bool reverseModeTriggered = false;

void loop() {
  if (newSteer || newThrottle) {
    newSteer = false;
    newThrottle = false;

    // 조향
    int steerAngle = map(steerPulse, 1000, 2000, 30, 150);
    steerServo.write(steerAngle);

    // 속도 제어
    int escPulse;

    if (throttlePulse >= 1500) {
      // 전진 구간 → 후진 상태 초기화
      escPulse = map(throttlePulse, 1500, 2000, 1500, 1560);
      reverseModeTriggered = false;

    } else {
      // 후진 구간
      if (!reverseModeTriggered) {
        // 후진 요청 처음 감지 → ESC 정지용 중립값 먼저 보내기
        escPulse = 1500;
        reverseModeTriggered = true;
        lastThrottleTime = millis();
      } else if (millis() - lastThrottleTime > 10) {
        // 후진 유지 시간이 300ms 넘으면 진짜 후진 명령 보냄
        escPulse = map(throttlePulse, 1000, 1500, 1400, 1500);
      } else {
        // 대기 중 → 계속 정지 상태 유지
        escPulse = 1500;
      }
    }

    esc.writeMicroseconds(escPulse);

    // 디버깅 출력
    Serial.print("Throttle PWM: "); Serial.print(throttlePulse);
    Serial.print(" → ESC Output: "); Serial.println(escPulse);
  }
}
