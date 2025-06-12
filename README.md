# **🔥 프로젝트 개요 (Overview)**

**이 프로젝트는 단순한 RC카를 넘어서, ‘영상 기반 라인트레이싱 + 수동 조작’이라는 하이브리드 제어 시스템을 구현하는 데 목적이 있습니다.**

- **목표**: Raspberry Pi와 Arduino를 통합한 임베디드 시스템 구축 + 실시간 비전 알고리즘 적용

---

# **📁프로젝트 구조**

<aside>

```cpp

├── arduino
│   ├── include
│   │   └── README
│   ├── lib
│   │   └── README
│   ├── platformio.ini
│   ├── src
│   │   ├── 1_arduino_manual_pwm_control.cpp
│   │   ├── 2_arduino_autonomous_only.cpp
│   │   └── 3_arduino_manual_autonomous_hybrid.cpp
│   └── test
│       └── README
├── image
└── raspberry_pi
    └── line_tracing.py

```

</aside>

- **1_arduino_manual_pwm_control.cpp**:  수동 조작을 위한 PWM을 디코딩하여 조작하는 아두이노 소스
- **2_arduino_autonomous_only.cpp**: 자율 주행을 위한 아두이노 소스
- 3_**arduino_manual_autonomous_hybrid.cpp**: 수동 조작과 자율주행을 동시에 수행할 수 있는 아두이노 소스
- **line_tracing.py**:자율 주행을 위한 이미지 처리를 포함하는 파이썬 소스

---

# **👥 팀원 소개 (Team)**

| **이름** | **역할** | **기여도** |
| --- | --- | --- |
| 김채연 | 수동 조작 로직 + 자율 주행 로직 | 100% |
| 최지혜 | 수동 조작 로직 + RC Car 회로 구성 | 100% |
| 서종원 | Python 기반 라인트레이싱 | 100% |

---

# **🛠 하드웨어 구성 요약 (Hardware Setup)**

| **부품** | **역할** | **주요 연결** |
| --- | --- | --- |
| Arduino UNO | 하위 제어기 | 모터 드라이버, RD9S |
| Raspberry Pi 5 | 영상처리 + 상위 제어 | PiCamera2, UART |
| AT-9S | RC Car 조종기 | IN1~IN4, ENA/ENB |
| RD9S 수신기 | PWM 신호 입력 | PWM 2채널 |

---

# **⚙️ 시스템 구성 (System Structure)**

## **🎮 수동 제어 모드 (Manual Mode)**

### RC Car 제어

- RD9S 수신기로부터 PWM 신호를 Arduino가 수신
- 모터의 속도 및 방향을 PWM/디지털 핀으로 제어
- 운전자가 직접 조작할 수 있음

### 코드 핵심 기능

- **PWM 신호 디코딩**: RC 조종기에서 입력되는 PWM(1000~2000µs)을 마이크로초 단위로 측정
- **조향 제어**: 측정한 PWM을 각도로 변환하여 서보 모터 제어
- **속도 제어 (ESC)**: 전진/후진 영역을 구분하고, ESC 특성을 고려한 안정적인 후진 로직 구현

### 동작 원리

- HIGH 상태 감지 시 `micros()`로 시작 시간 기록
- LOW 상태 감지 시 현재 시간에서 시작 시간을 빼서 **펄스 폭 계산**

- **✅조향 제어**
    
    ```cpp
    int steerAngle = map(steerPulse, 1000, 2000, 30, 150);
    steerServo.write(steerAngle);
    ```
    
    - PWM을 서보각도로 변환
    - 1500µs는 약 90도 → 중립 상태
    - 우회전/좌회전 범위를 물리적 한계에 맞게 설정

- **✅속도 제어**
    - **전진**
        
        ```cpp
        
        map(throttlePulse, 1500, 2000, 1500, 1560);
        ```
        
    - 1500µs 이상일 경우 전진
    - 최고 속도는 1560µs로 제한 → 과속 방지
    
    - **후진** (ESC 보호 고려)
        
        ```cpp
        
        if (!reverseModeTriggered) {
          escPulse = 1500; // 중립 먼저
          reverseModeTriggered = true;
          lastThrottleTime = millis();
        } else if (millis() - lastThrottleTime > 10) {
          escPulse = map(throttlePulse, 1000, 1500, 1400, 1500);
        }
        ```
        
    - ESC는 갑작스러운 후진을 허용하지 않음 → 중립값을 한번 보내야 함
    - 후진 신호를 처음 감지하면 **중립값(1500µs) 전송 후 대기**
    - 이후 일정 시간 경과하면 진짜 후진 값 전달

---

## **🧠 라인트레이싱 → 자율 주행 모드 (Line Tracing Mode)**

### 📈 Line Tracing 방법

| **단계** | **설명** |
| --- | --- |
| 1. 캡처 | Picamera2로 실시간 영상 입력 |
| 2. 영상 전처리 | CLAHE → HSV 변환 → 마스크 처리 |
| 3. ROI 설정 | 상/하단 관심영역 설정 |
| 4. 각도 계산 | 중심 좌표로부터 예측 및 현재 각도 추출 |
| 5. 시리얼 전송 | 기울기 또는 각도 전송 (5프레임 주기) |
| 6. 시각화 출력 | 결과 영상 및 디버깅 정보 실시간 출력 |

### **🖼 영상 전처리 흐름 (Image Processing)**

- **영상 전처리 요약**
    
    
    | **단계** |
    | --- |
    | 1. BGR → LAB 변환 |
    | 2. LAB에서 L채널 분리 |
    | 3. L채널에 Clache 적용 |
    | 4. LAB 채널 HSV로 변환 |
    | 5. HSV에서 검은색 범위 추출 |
    | 6. Morphology로 노이즈 최소화 |
    | 7. 파라미터 조절로 정확도 올림 |
- **✅ LAB 변환 + CLAHE**
    - RGB → LAB 컬러 공간으로 변환한 뒤, 밝기(L) 채널에 CLAHE 적용
    - **CLAHE (Contrast Limited Adaptive Histogram Equalization)**:
    - 이미지의 **국소적인 명암 대비를 향상**시켜 어두운 라인도 뚜렷하게 보이도록 함.
    - **Code**
        
        ```python
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        enhanced = cv2.cvtColor(cv2.merge((cl, a, b)), cv2.COLOR_LAB2BGR)
        ```
        
- **✅ HSV 변환 + 검정색 마스크 생성**
    - 명암 강화된 이미지 → HSV 변환
    - 검정색 범위를 지정하여 마스크 생성
    - 작은 잡음을 제거하기 위해 모폴로지 열림(Opening) 처리 적용
    - **Code**
        
        ```python
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        ```
        
- **✅Mask 파라미터 조정**
    - Code
        
        ```python
        lower_black = np.array([0, 0, 0]) # 각각 H,S,V값을 의미함
        upper_black = np.array([180, 255, 110]) # 각각 H,S,V값을 의미함
        mask = cv2.inRange(hsv, lower_black, upper_black)
        ```
        
    - lower_black의 영역부터 upper_black의 영역까지만 mask에서 통과시킴
    - **V값의 범위 : 0~70**
        
        ![Screenshot 2025-06-05 at 12.21.23 PM.png](%F0%9F%9A%97%20Line%20Tracing%20RC%20Car%2021052859656b80e39b59d4469f0adc6a/Screenshot_2025-06-05_at_12.21.23_PM.png)
        
    - **V값의 범위 : 0~110**
        
        ![Screenshot 2025-06-05 at 12.33.35 PM.png](%F0%9F%9A%97%20Line%20Tracing%20RC%20Car%2021052859656b80e39b59d4469f0adc6a/Screenshot_2025-06-05_at_12.33.35_PM.png)
        

### **🎯 중심 좌표 계산**

- **✅ROI 설정**
    - 예측 영역 : Top(0.15)~Bottom(0.35) , Left(0.1)~Right(0.9)
    - 현재 영역 : Top(0.65)~Bottom(0.85) , Left(0.1)~Right(0.9)
- **✅좌표 계산 방법**
    - `M = cv2.moments(roi)`  : 흰색 픽셀 영역의 x좌표를 모두 더한 후, 흰색 영역 픽셀 갯수로 나눔
    - `angle = 90 + ((cx - width // 2) / (width // 2)) * 45`  : -1~1 범위의 값으로 정규화 한 후에 45를 곱하고 중심 각도(90)을 기준으로 더함

### **📡UART 통신**

- **✅시리얼 연결**
    - 라즈베리파이의 USB포트와 아두이노 USB 포트를 연결함
    - **Code**
        
        ```python
        # ✅ 시리얼 연결
        try:
            arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)
            print("[INFO] Arduino connected.")
        except Exception as e:
            print(f"[ERROR] Arduino connection failed: {e}")
            arduino = None
        ```
        
- **✅각도 전송**
    - 5프레임 마다 각도를 전송함
    - predicted_angle이 추적될 때는 해당 값을 전송
    - current_angle만 추적이 될 때는 해당 값을 전송
    - 둘 다 추적이 안될 때는 ‘-1’ 전송
    - **Code**
        
        ```python
        # ✅ 5프레임마다 각도 전송
        if frame_count % 5 == 0 and arduino:
            try:
                if predicted_angle_tmp is not None:
                    arduino.write(f'{predicted_angle}\n'.encode())
                    print(f"[SERIAL] Sent Predicted: {predicted_angle}")
                elif current_angle_tmp is not None:
                    arduino.write(f'{current_angle}\n'.encode())
                    print(f"[SERIAL] Sent Current: {current_angle}")
                else:
                    arduino.write(f'-1\n'.encode())
                    print(f"[SERIAL] Sent: -1 (No line detected)")
            except Exception as e:
                print(f"[ERROR] Serial write failed at frame {frame_count}: {e}")
        ```
        

### **🌐 웹 스트리밍 (Web Streaming)**

- **✅웹 스트리밍 단계 요약**
    
    
    | **단계** |
    | --- |
    | 1. [Picamera2 프레임] |
    | 2. [처리 + 인코딩] |
    | 3. generate_frames() |
    | 4. /video 라우트 |
    | 5. <img src="/video"> |
    | 6. 브라우저에 실시간 표시 |
- **✅ @app.route('/video') — MJPEG 스트리밍 라우터**
    - generate_frames() 함수가 이미지 프레임을 **무한 루프**로 지속 전송함.
    - **Code**
        
        ```python
        @app.route('/video')
        def video():
            return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
        ```
        
- **✅ generate_frames() — 프레임 생성 제너레이터**
    - 여기서 **프레임을 MJPEG 형식으로 생성**하고 yield로 계속 반환함.
    - **Code**
        
        ```python
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               buffer.tobytes() + b'\r\n')
        ```
        
- **✅ @app.route('/') — 메인 웹페이지**
    - 이 태그는 /video에서 전송되는 MJPEG을 **브라우저에서 실시간 영상처럼 보여줌**.
    - **Code**
        
        ```python
        @app.route('/')
        def index():
            return '<h1>📷 예측/현재 각도 + 전처리 영상</h1><img src="/video">'
        ```
        
- **✅ 결과물**
    
    ![Screenshot 2025-06-05 at 12.20.37 PM.png](%F0%9F%9A%97%20Line%20Tracing%20RC%20Car%2021052859656b80e39b59d4469f0adc6a/551f0dd6-1e8d-4904-851c-58046c20de56.png)
    

### **🚗**RC Car 제어

**라즈베리파이와 아두이노 간의 통신 프로토콜**

- **프로토콜 종류**: UART (Serial 통신)
- **물리적 연결**: Raspberry Pi의 `TX/RX` ↔ Arduino의 `RX/TX`
- **Baud Rate**: `9600` bps
- **종단 문자**: `\n` (newline 기준으로 한 줄씩 전송)
- **전송 내용**:
    - **라즈베리파이 → 아두이노**
        - **의미**: 카메라 영상 기반 라인트레이싱 결과 전달
        - **형식**: `int`형 각도 (예: `87\n`, `1\n`)
            - `45~135`: 유효한 steering angle
            - `1`: 라인 미검출 신호 (Recovery 트리거)
- **오류 처리:**
    - **flush() 사용**: Python 측에서 `serial.write()` 후 `flush()`로 전송 보장
    - **시리얼 타임아웃 설정**: 아두이노 측에서 `Serial.setTimeout(10);`
    - **이상 값 필터링**: `1` 외 값이 들어와도 유효 범위(45~135)만 처리

| 핀 번호 | 용도 | 연결 대상 |
| --- | --- | --- |
| D5 | ESC PWM 출력 | 브러시리스 모터 ESC |
| D9 | 서보 모터 PWM 출력 | 스티어링 서보 |
| UART TX | 파이썬 → 아두이노 | 각도 데이터 송신 |

아두이노는 총 **4단계 상태(State)** 로 차량을 제어합니다.

### **1. `STATE_NORMAL`: 정상 주행 상태**

- **동작 조건**: 라즈베리파이에서 유효한 각도(45°~135°) 수신 시
- **주요 로직**:
    - 수신 각도가 유효하면 → `180 - rawAngle`로 반전하여 서보 제어
    - ESC는 전진 값을 출력 (`1545µs`)
    - 각도 수신이 없거나 `1`이면 → `STATE_STOP`으로 전환
    
    ```cpp
    
    if (rawAngle == -1) {
      state = STATE_STOP;
    }
    else if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
      lastValidAngle = 180 - rawAngle;
      steer.write(lastValidAngle);
      esc.writeMicroseconds(MAX_FORWARD);
    }
    else {
      steer.write(lastValidAngle);  // 유지
      esc.writeMicroseconds(MAX_FORWARD);
    }
    
    ```
    

### **2. `STATE_STOP`: 라인 손실 시 정지 상태**

- **동작 조건**: `rawAngle == -1`일 때 진입
- **주요 동작**:
    - 차량 즉시 정지 (ESC = 1500, steer = 90°)
    - 이후 즉시 `STATE_REVERSING`으로 전환
    
    ```cpp
    
    steer.write(90); // 조향 정면
    esc.writeMicroseconds(ESC_NEUTRAL); // 모터 정지
    state = STATE_REVERSING;
    reverseStartTime = millis();
    ```
    

### **3. `STATE_REVERSING`: 후진 복구 상태**

- **동작 조건**: `STATE_STOP` 직후 진입
- **후진 시간**: 최대 2초
- **상황별 전이**:
    - 후진 중 유효 각도 수신 시 → 즉시 `STATE_NORMAL`로 복귀
    - 2초 경과 시에도 미수신 → `STATE_FORWARDING`으로 전환
    
    ```cpp
    
    steer.write(90);
    esc.writeMicroseconds(MAX_REVERSE); // 후진 시작
    
    if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
      lastValidAngle = 180 - rawAngle;
      steer.write(lastValidAngle);
      esc.writeMicroseconds(MAX_FORWARD);
      state = STATE_NORMAL;
    }
    else if (millis() - reverseStartTime >= 2000) {
      state = STATE_FORWARDING;
    }
    
    ```
    

### **4. `STATE_FORWARDING`: 복구 전진 상태**

- **동작 조건**: 2초 후진했음에도 각도 감지 실패 시 진입
- **동작**:
    - `lastValidAngle`을 기반으로 전진 시도
    - 전진 도중 유효한 각도 수신 시 → 각도 갱신
    - 라인 미감지 시 → 다시 `STATE_STOP`으로 복귀 (후진 재시도)
    
    ```cpp
    
    steer.write(lastValidAngle);
    esc.writeMicroseconds(MAX_FORWARD);
    
    if (rawAngle >= MIN_ANGLE && rawAngle <= MAX_ANGLE) {
      lastValidAngle = 180 - rawAngle;
    }
    else if (rawAngle == -1) {
      state = STATE_STOP;
    }
    ```
    

![스크린샷 2025-06-12 오후 2.04.32.png](%F0%9F%9A%97%20Line%20Tracing%20RC%20Car%2021052859656b80e39b59d4469f0adc6a/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-06-12_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_2.04.32.png)

| 변수명 | 설명 |
| --- | --- |
| `rawAngle` | 시리얼로 수신한 라즈베리파이 측 각도 |
| `lastValidAngle` | 마지막 유효했던 조향 각도 (복구용) |
| `reverseStartTime` | 후진 시작 시간 기록 |
| `state` | 현재 상태 머신의 상태 |

### 라인 미탐지 대응 로직 비교표

| 시도 번호 | 대응 방식 설명 | 의도 | 문제점 |
| --- | --- | --- | --- |
| 1 | 라인 미탐지 시, **마지막 기억 각도**로 계속 전진 | 트랙에서 빠르게 복구하고자 함 | 좁은 커브에서 그대로 진행되면 탈선 위험 → **트랙을 원활히 돌지 못함** |
| 2 | 라인 미탐지 시, **마지막 각도와 90° 중간값**으로 전진 | 중심에 복귀하도록 유도 | 순간적인 노이즈로 인한 미탐지일 경우 → **비정상 각도로 오히려 다른 곳으로 이탈** |
| 3 | 라인 미탐지 시, **좌우 살짝 회전 → 후진 반복**하며 탐색 | 탐색 범위를 넓혀 복귀 시도 | 노이즈가 지속될 경우 → **무한 반복 루프 발생 가능** |
| 4 | `-1` 수신 시, **기억 각도로 후진 유지** | 후진으로 자연스럽게 복귀 시도 | 라인 탐지 실패가 지속되면 → **계속 후진만 하며 복귀 못함** |

### 결론: 현재 코드에서 채택한 방식

→ `-1` 수신 시 **즉시 정지 → 후진 2초 → 전진 복구 시도 (상태 머신 기반)**

- **장점**:
    - 일시적 노이즈 대응 가능 (후진 중 라인 다시 감지 시 바로 전진 복귀)
    - **무한 루프/무한 후진 방지**
    - 상태 기반이라 **안정성과 예측 가능성**이 높음

---

## **🎮 🧠수동 조작 + 자율주행 동시 수행**

이 소스는 RC 조종기 기반의 수동 조작과 라즈베리파이에서 전달된 각도 정보 기반 자율주행을 모두 지원하는 이중 모드 아두이노 제어 시스템입니다.

`CH5 스위치 (A 스위치)`를 통해 모드를 전환하며, LED 시그널 및 PWM 기반 모터/서보 제어 기능까지 포함되어 있습니다.

![KakaoTalk_20250611_161019433.jpg](%F0%9F%9A%97%20Line%20Tracing%20RC%20Car%2021052859656b80e39b59d4469f0adc6a/KakaoTalk_20250611_161019433.jpg)

| 핀 번호 | 기능 | 설명 |
| --- | --- | --- |
| A0 | 조향 입력 (CH1) | 수동 모드: PWM 입력 (PPM) |
| A1 | 스로틀 입력 (CH2) | 수동 모드: PWM 입력 (PPM) |
| A2 | 모드 전환 (CH5) | HIGH: 수동 / LOW: 자동 |
| D5 | ESC 출력 | 모터 속도 제어 |
| D9 | 서보 출력 | 조향 각도 제어 |
| D6 | 좌측 LED (HIGH=ON) | 방향 표시 |
| D7 | 우측 LED (LOW=ON) | 방향 표시 |

### 수동모드 ← → 자율주행전환 방식

![Screenshot 2025-06-11 at 2.28.01 PM.png](%F0%9F%9A%97%20Line%20Tracing%20RC%20Car%2021052859656b80e39b59d4469f0adc6a/Screenshot_2025-06-11_at_2.28.01_PM.png)

| 조건 | 활성화 모드 |
| --- | --- |
| SWA > 1500µs(up) | **수동 모드** |
| SWA ≤ 1500µs(down) | **자율주행 모드 (Recovery)** |

> 모드 변경 시 내부 상태 자동 초기화
> 

### 수동 모드 동작

- **조향**: `CH1`의 PWM을 `30°~150°`로 매핑하여 서보 제어
- **스로틀**: `CH2`의 PWM으로 전진/후진 속도 결정
    - 후진 시 ESC 보호를 위해 중립 → 후진 딜레이 포함
- **LED 제어**:
    - **좌/우 방향 조작 시 해당 방향 LED 깜빡임**
    - **후진 시 양쪽 LED 점등**
    
    ```cpp
    
    int steerAngle = map(steerPulse, 1000, 2000, 30, 150);
    steerServo.write(steerAngle);
    
    ```
    

### 자율주행 모드 (Recovery 상태 머신)

라즈베리파이에서 시리얼로 각도 전송 시 동작하며, 라인 손실 복구를 위한 상태 기반 제어 시스템이 포함되어 있습니다.

- **✅상태 머신 구성**

| 상태 | 설명 |
| --- | --- |
| `STATE_NORMAL` | 유효 각도 수신 → 전진 |
| `STATE_STOP` | 각도 -1 수신 시 정지 |
| `STATE_REVERSING` | 정지 직후 2초간 후진 |
| `STATE_FORWARDING` | lastValidAngle로 전진 재시도 |

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `ESC_NEUTRAL_US` | 1500 | 정지 |
| `MAX_FORWARD_AUTO` | 1545 | 자동 전진 속도 |
| `MAX_REVERSE_AUTO` | 1421 | 자동 후진 속도 |
| `MIN_ANGLE`~`MAX_ANGLE` | 45~135 | 유효 조향 각도 범위 |
| 후진 시간 | 2초 | Recovery 중 |
- **✅LED**

| 핀 번호 | 방향 | 연결 방식 | 설명 |
| --- | --- | --- | --- |
| `D6` | 좌측 LED | Active-High | `HIGH` → 켜짐, `LOW` → 꺼짐 |
| `D7` | 우측 LED | Active-Low | `LOW` → 켜짐, `HIGH` → 꺼짐 |

| 차량 상태 | 좌측 LED(D6) | 우측 LED(D7) | 설명 |
| --- | --- | --- | --- |
| **후진 중** | 항상 ON | 항상 ON | 경고등 역할 |
| **좌회전 중** | **깜빡임** (500ms) | 꺼짐 | 방향지시등 역할 |
| **우회전 중** | 꺼짐 | **깜빡임** (500ms) | 방향지시등 역할 |
| **직진/중립** | 꺼짐 | 꺼짐 | LED 모두 OFF |

- **✅후진 시 LED 제어**
    
    ```cpp
    
    if (isReversing) {
      turnOnLeft();
      turnOnRight();
      blinkState = false;
      Serial.println("[DEBUG] 수동 후진 중 → LED 양쪽 ON");
    }
    
    ```
    
    - **후진 조건**: 스로틀(PPM) 신호가 1500보다 작음
    - **동작**: 양쪽 LED 모두 켜짐
    - 좌측: `digitalWrite(D6, HIGH)` → ON
    - 우측: `digitalWrite(D7, LOW)` → ON
- **✅좌/우 조향 시 깜빡임 로직**
    
    ```cpp
    
    if (steerAngle < 65) {
      // 좌회전
      turnOffRight();  // 우측 꺼짐 (Active-Low → HIGH)
      if (blinkState) turnOnLeft(); else turnOffLeft();
    }
    else if (steerAngle > 115) {
      // 우회전
      turnOffLeft();   // 좌측 꺼짐
      if (blinkState) turnOnRight(); else turnOffRight();
    }
    
    ```
    
    - **좌회전 조건**: 조향 각도(steerAngle) < 65
    - **우회전 조건**: 조향 각도 > 115
    - **깜빡임 구현**: 500ms 주기로 `blinkState` 플래그 전환
    
    ```cpp
    
    if (now - lastBlinkToggle >= BLINK_INTERVAL) {
      blinkState = !blinkState;
      lastBlinkToggle = now;
    }
    ```
    
    - `blinkState == true` → 해당 방향 LED ON
    - `blinkState == false` → 해당 방향 LED OFF

- **✅직진/중립 시**

```cpp

else {
  // steerAngle 65~115: 중립
  blinkState = false;
  turnOffLeft();
  turnOffRight();
  Serial.println("[DEBUG] 수동 전진(중립) → LED OFF");
}

```

- **✅조향 각도 65~115**인 경우
    - 좌우 모두 OFF

---

## **🧠 배운 점 (What We Learned)**

- Arduino-RPi 간의 시리얼 통신, 센서 인터페이싱, 실시간성 고려
- 조도 변화, 배경 노이즈 등의 영상 처리 현실 문제 대응 경험
- 하드웨어와 소프트웨어의 협업 중요성을 체득
- 인터럽트 기반 수신기 처리, 데이터 포맷 설계의 필요성 인식
- 물리 환경은 이상적 시뮬레이션과 다르므로 반복적인 실험과 튜닝이 필수

---

## **🧩 문제 해결 팁 (Troubleshooting)**

| **문제** | **해결 방법** |
| --- | --- |
| PiCamera 인식 안 됨 | sudo rpicam-setup 실행 후 재부팅 |
| 중심값 튐 현상 | ROI 필터링 범위 조정 및 Morphology 연산 적용 |
| 특정 모터 속도 이하에서 바퀴 안 돎 | 바닥과의 마찰력으로 인해 토크 부족 → 실험 통해 **ESC 전진값 상한 고정** |
| 파이썬과 아두이노 간 통신 안됨 | 양쪽 코드의 **baud rate 불일치** → 모두 `9600`으로 맞춤 |
