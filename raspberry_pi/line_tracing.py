from flask import Flask, Response
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial

app = Flask(__name__)

# ✅ 시리얼 연결
try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    print("[INFO] Arduino connected.")
except Exception as e:
    print(f"[ERROR] Arduino connection failed: {e}")
    arduino = None

# ✅ 카메라 설정
picam2 = Picamera2()
picam2.options["BufferCount"] = 1
picam2.configure(picam2.create_video_configuration(main={"size": (320, 240)}))
picam2.start()
time.sleep(2)

frame_count = 0
predicted_angle = 90
current_angle = 90

def extract_angle_from_roi(roi, top_offset, width):
    M = cv2.moments(roi)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        angle = int(np.clip(90 + ((cx - width // 2) / (width // 2)) * 45, 45, 135))
        return angle, cx
    return None, None

def generate_frames():
    global frame_count, predicted_angle, current_angle

    while True:
        frame = picam2.capture_array()
        height, width = frame.shape[:2]

        # ✅ 전처리: CLAHE → HSV → 검정 마스크
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        lab_clahe = cv2.merge((cl, a, b))
        enhanced = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 110])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # 🔽 ROI 설정
        top1, bottom1 = int(height * 0.15), int(height * 0.35)
        top2, bottom2 = int(height * 0.55), int(height * 0.75)
        x1 = int(width * 0.10)
        x2 = int(width * 0.90)
        roi_width = x2 - x1

        # 예측 각도 (위)
        pred_roi = clean[top1:bottom1, x1:x2]
        predicted_angle_tmp, cx1 = extract_angle_from_roi(pred_roi, top1, roi_width)
        if predicted_angle_tmp is not None:
            predicted_angle = predicted_angle_tmp
            cx1 += x1
            cv2.circle(frame, (cx1, (top1 + bottom1) // 2), 5, (255, 0, 0), -1)
            cv2.putText(frame, f'Pred: {predicted_angle}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 현재 각도 (아래)
        curr_roi = clean[top2:bottom2, x1:x2]
        current_angle_tmp, cx2 = extract_angle_from_roi(curr_roi, top2, roi_width)
        if current_angle_tmp is not None:
            current_angle = current_angle_tmp
            cx2 += x1
            cv2.circle(frame, (cx2, (top2 + bottom2) // 2), 5, (0, 255, 0), -1)
            cv2.putText(frame, f'Curr: {current_angle}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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

        # ROI 시각화
        cv2.rectangle(frame, (x1, top1), (x2, bottom1), (255, 0, 0), 1)
        cv2.rectangle(frame, (x1, top2), (x2, bottom2), (0, 255, 0), 1)

        frame_count += 1

        # 🔧 RGBA → BGR 변환 (예외 방지)
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        # 전처리 이미지 나란히 표시
        clean_bgr = cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR)
        stacked = np.hstack((frame, clean_bgr))

        # MJPEG 인코딩
        ret, buffer = cv2.imencode('.jpg', stacked, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return '<h1>📷 예측/현재 각도 + 전처리 영상</h1><img src="/video">'

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)