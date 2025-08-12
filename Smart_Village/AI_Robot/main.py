import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

# تحميل نموذج YOLO
model = YOLO("model/yolov8s.pt")

# تشغيل الكاميرا
cap = cv2.VideoCapture(0)

# الاتصال بالبلوتوث
bt = serial.Serial('COM7', 9600)
time.sleep(2)

last_cmd = None

def detect_blue_robot(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cnt = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(cnt)
        cx = x + w // 2
        cy = y + h // 2
        return cx, cy
    return None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    robot_pos = detect_blue_robot(frame)
    results = model(frame)
    target_pos = None

    # البحث عن الزجاجة
    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            label = r.names[cls]
            if label.lower() == "bottle":
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                target_pos = (cx, cy)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                break

    # لو الزجاجة والروبوت موجودين
    if target_pos and robot_pos:
        rx, ry = robot_pos
        tx, ty = target_pos

        cv2.circle(frame, (rx, ry), 5, (255, 0, 0), -1)
        cv2.circle(frame, (tx, ty), 5, (0, 255, 0), -1)
        cv2.line(frame, (rx, ry), (tx, ty), (0, 0, 255), 2)

        dx = tx - rx
        dy = ty - ry

        if abs(dy) > 50:
            cmd = 'F'
        elif abs(dx) > 40:
            cmd = 'R' if dx > 0 else 'L'
        else:
            cmd = 'S'

    else:
        # في حالة فقدان الزجاجة أو الروبوت
        cmd = 'S'

    # إرسال الأمر فقط إذا تغيّر
    if cmd != last_cmd:
        bt.write(f"{cmd}\n".encode())
        print("Sent:", cmd)
        last_cmd = cmd

    cv2.imshow("Frame", frame)

    # الخروج عند الضغط على Q أو إغلاق النافذة
    if cv2.getWindowProperty("Frame", cv2.WND_PROP_VISIBLE) < 1 or cv2.waitKey(1) & 0xFF == ord('q'):
        bt.write(b'S\n')  # تأكيد الوقوف
        break

cap.release()
bt.close()
cv2.destroyAllWindows()
