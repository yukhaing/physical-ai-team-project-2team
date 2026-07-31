# ============================================================
# Defective Box Detection Training 
# ============================================================
# Colab notebook: DefectiveBox_Training.ipynb (Google Drive)
# Requires Google Drive mounted with the dataset/ folder under PROJECT_DIR.

# ---------- Install ----------
import sys
import subprocess

subprocess.run(
    [sys.executable, "-m", "pip", "install", "-q", "ultralytics"],
    check=True
)

# ---------- Import ----------
import os
from google.colab import drive
from ultralytics import YOLO

# ---------- Mount Google Drive ----------
drive.mount("/content/drive", force_remount=False)

# ---------- Project Settings ----------
PROJECT_DIR = "/content/drive/MyDrive/Classifying defective boxes"
DATASET_DIR = os.path.join(PROJECT_DIR, "dataset")
RUNS_DIR = os.path.join(PROJECT_DIR, "runs")

# ============================================================
# Model Settings
# ============================================================

MODEL_NAME = "yolov8l"          # yolov8n / yolov8s / yolov8m / yolov8l
RUN_NAME = "yolov8l_100e_v2"

# ============================================================
# Training Settings
# ============================================================

EPOCHS = 100
IMAGE_SIZE = 640
BATCH_SIZE = 16
WORKERS = 2

# ============================================================
# Check Dataset
# ============================================================

print("=" * 60)
print(f"Working Directory : {PROJECT_DIR}")
print("=" * 60)

if not os.path.exists(DATASET_DIR):
    raise FileNotFoundError(
        f"Dataset not found!\nExpected path:\n{DATASET_DIR}"
    )

os.chdir(PROJECT_DIR)

print("Dataset Found.")
print("=" * 60)

print(f"Model      : {MODEL_NAME}")
print(f"Run Name   : {RUN_NAME}")
print(f"Epochs     : {EPOCHS}")
print(f"Image Size : {IMAGE_SIZE}")
print(f"Batch Size : {BATCH_SIZE}")
print("=" * 60)

# ============================================================
# Load Model
# ============================================================

model = YOLO(f"{MODEL_NAME}.pt")

# ============================================================
# Train
# ============================================================

results = model.train(
    data=os.path.join(DATASET_DIR, "data.yaml"),
    epochs=EPOCHS,
    imgsz=IMAGE_SIZE,
    batch=BATCH_SIZE,
    workers=WORKERS,
    cache=True,
    patience=100,
    project=RUNS_DIR,
    name=RUN_NAME,
    exist_ok=False,
    seed=42,
    deterministic=True,
    verbose=True
)

# ============================================================
# Validation
# ============================================================

print("\nRunning validation...\n")

metrics = model.val()

print("=" * 60)
print("Training Completed!")
print("=" * 60)
print(f"Results Folder : {RUNS_DIR}/{RUN_NAME}")
print("=" * 60)
