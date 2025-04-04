# yolo_train_detect.py

from ultralytics import YOLO
import os
import zipfile
import yaml
import shutil
import random
from pathlib import Path

# ------------------ Step 1: Setup paths ------------------ #
BASE_DIR = Path(__file__).resolve().parent

dataset_zip = BASE_DIR / "dataset_fruits_and_vegetables.zip"
extract_path = BASE_DIR / "extracted_dataset"
yolo_dataset_path = BASE_DIR / "yolo_dataset"

# ------------------ Step 2: Extract dataset ------------------ #
if not extract_path.exists():
    extract_path.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(dataset_zip, 'r') as zip_ref:
        zip_ref.extractall(extract_path)

# ------------------ Step 3: Prepare YOLO dataset ------------------ #
def prepare_yolo_dataset(extract_path, yolo_dataset_path):
    for split in ['train', 'val', 'test']:
        (yolo_dataset_path / split / 'images').mkdir(parents=True, exist_ok=True)
        (yolo_dataset_path / split / 'labels').mkdir(parents=True, exist_ok=True)

    class_dirs = [d for d in extract_path.iterdir() if d.is_dir()]
    class_names = sorted([d.name for d in class_dirs])
    class_dict = {name: idx for idx, name in enumerate(class_names)}

    # Create data.yaml
    data_yaml = {
        'train': str((yolo_dataset_path / 'train' / 'images').resolve()),
        'val': str((yolo_dataset_path / 'val' / 'images').resolve()),
        'test': str((yolo_dataset_path / 'test' / 'images').resolve()),
        'nc': len(class_names),
        'names': class_names
    }
    with open(yolo_dataset_path / 'data.yaml', 'w') as f:
        yaml.dump(data_yaml, f)

    # Process each class folder
    for class_name in class_names:
        class_path = extract_path / class_name
        class_id = class_dict[class_name]
        image_files = [f for f in class_path.iterdir() if f.suffix.lower() in ['.jpg', '.jpeg', '.png']]
        random.shuffle(image_files)

        train_split = int(0.7 * len(image_files))
        val_split = int(0.9 * len(image_files))

        splits = {
            'train': image_files[:train_split],
            'val': image_files[train_split:val_split],
            'test': image_files[val_split:]
        }

        for split, files in splits.items():
            for img_file in files:
                dst_img = yolo_dataset_path / split / 'images' / f"{class_name}_{img_file.name}"
                shutil.copy2(img_file, dst_img)

                label_path = yolo_dataset_path / split / 'labels' / f"{class_name}_{img_file.stem}.txt"
                with open(label_path, 'w') as f:
                    f.write(f"{class_id} 0.5 0.5 1.0 1.0\n")  # dummy bbox

    return yolo_dataset_path / 'data.yaml'

data_yaml_path = prepare_yolo_dataset(extract_path, yolo_dataset_path)

# ------------------ Step 4: Train the model ------------------ #
model = YOLO('yolov8n.pt')  # Load YOLOv8 nano

results = model.train(
    data=str(data_yaml_path),
    epochs=50,
    imgsz=640,
    batch=16,
    name='fruits_vegetables_yolo'
)

# ------------------ Step 5: Prediction ------------------ #
def predict_fruits_vegetables(model_path, image_path):
    model = YOLO(str(model_path))
    results = model.predict(str(image_path))
    results[0].show()

    detections = []
    for box in results[0].boxes:
        class_id = int(box.cls[0])
        class_name = model.names[class_id]
        confidence = float(box.conf[0])
        detections.append({'class': class_name, 'confidence': confidence})
    return detections

# ------------------ Step 6: Run test ------------------ #
if __name__ == "__main__":
    best_model_path = BASE_DIR / "runs" / "detect" / "fruits_vegetables_yolo" / "weights" / "best.pt"
    test_image_dir = yolo_dataset_path / "test" / "images"

    if not test_image_dir.exists():
        print(f"Test image directory not found: {test_image_dir}")
        exit(1)

    test_images = list(test_image_dir.glob("*"))
    if test_images and best_model_path.exists():
        image_to_predict = test_images[0]
        detections = predict_fruits_vegetables(best_model_path, image_to_predict)

        print("Detected objects:")
        for d in detections:
            print(f"- {d['class']} (confidence: {d['confidence']:.2f})")
    else:
        print("No test images or trained model found.")
