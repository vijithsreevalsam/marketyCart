from ultralytics import YOLO
import cv2
import time
import os

print("🥦🍎 YOLOv8 Fruit and Vegetable Recognition Test")
print("==============================================")

# Dynamically resolve the path to detect.pt relative to this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "detect.pt")
if not os.path.exists(model_path):
    print(f"❌ Model not found at: {model_path}")
    exit()

print(f"✅ Loading model from: {model_path}")
model = YOLO(model_path)

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Unable to open webcam.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cv2.namedWindow("YOLOv8 Fruit Detection", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("YOLOv8 Fruit Detection", 1600, 900)
cv2.setWindowProperty("YOLOv8 Fruit Detection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# Price Dictionary
price_dict = {
    "apple": 1.20,
    "banana": 0.50,
    "orange": 0.80,
    "tomato": 0.75,
    "carrot": 0.60,
    "broccoli": 1.50,
    "potato": 0.30,
    "cucumber": 0.90,
    "lemon": 0.70,
    "strawberry": 2.50,
}

conf_threshold = 0.4
os.makedirs("fruit_veg_screenshots", exist_ok=True)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        start_time = time.time()
        results = model.predict(frame, conf=conf_threshold, stream=False, verbose=False)
        inference_time = time.time() - start_time

        result_frame = results[0].plot()
        fps = 1.0 / inference_time
        cv2.putText(result_frame, f"FPS: {fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        boxes = results[0].boxes
        detection_count = len(boxes)
        cv2.putText(result_frame, f"Items: {detection_count}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if detection_count > 0:
            class_counts = {}
            cart_price = 0.0

            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
                if cls_name.lower() in price_dict:
                    cart_price += price_dict[cls_name.lower()]

            y_pos = 120
            for cls_name, count in class_counts.items():
                price = price_dict.get(cls_name.lower(), 0) * count
                label = f"{cls_name}: {count} (${price:.2f})"
                cv2.putText(result_frame, label, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                y_pos += 30

            cv2.putText(result_frame, f"TOTAL: ${cart_price:.2f}", (frame.shape[1] - 300, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.putText(result_frame, f"Conf: {conf_threshold:.2f}", (20, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow("YOLOv8 Fruit Detection", result_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('+'):
            conf_threshold = min(0.95, conf_threshold + 0.05)
        elif key == ord('-'):
            conf_threshold = max(0.05, conf_threshold - 0.05)
        elif key == ord('s'):
            filename = f"fruit_veg_screenshots/cart_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(filename, result_frame)

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    cv2.destroyAllWindows()
