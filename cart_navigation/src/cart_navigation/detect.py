from ultralytics import YOLO
import cv2
import sys
from pathlib import Path

# ------------------ Load YOLOv8 Model ------------------ #
BASE_DIR = Path(__file__).resolve().parent
model_path = BASE_DIR / "yolov8n.pt"

if not model_path.exists():
    print(f"❌ Model not found at: {model_path}")
    sys.exit(1)

model = YOLO(str(model_path))

# ------------------ OpenCV Video Capture ------------------ #
cap = cv2.VideoCapture(2)  # Adjust if necessary

if not cap.isOpened():
    print("❌ Unable to open webcam.")
    sys.exit(1)

cv2.namedWindow("YOLOv8 - Webcam", cv2.WINDOW_NORMAL)
cv2.resizeWindow("YOLOv8 - Webcam", 960, 540)  # Optional: Wider view

# ------------------ Live Detection Loop ------------------ #
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Run YOLO prediction silently
    results = model.predict(source=frame, stream=True, verbose=False)

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls]
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Draw bounding box and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{name} {conf:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show GUI window
    cv2.imshow("YOLOv8 - Webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ------------------ Cleanup ------------------ #
cap.release()
cv2.destroyAllWindows()
