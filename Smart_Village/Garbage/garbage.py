import cv2
from ultralytics import YOLO
import serial
import time

# تحميل النموذج المدرب
model = YOLO("model/yolov8s.pt")  # تأكد من مسار النموذج

# تشغيل الكاميرا
cap = cv2.VideoCapture(0)

# الاتصال بالبلوتوث
# bt = serial.Serial('COM7', 9600)  # غيّر COM7 حسب المنفذ
# time.sleep(2)

last_sent = "NONE"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    detected_class = "NONE"

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            label = r.names[cls].lower()

            if label == "bottle":
                detected_class = "BOTTLE"
                break
            elif label == "book":
                detected_class = "BOOK"
                break

        if detected_class != "NONE":
            break

    # إرسال فقط إذا تغير التصنيف
    if detected_class != last_sent:
        bt.write(f"{detected_class}\n".encode())
        print("Sent:", detected_class)
        last_sent = detected_class

    # عرض الكاميرا (اختياري)
    cv2.imshow("Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        bt.write(b"NONE\n")
        break

cap.release()
bt.close()
cv2.destroyAllWindows()
