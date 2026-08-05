# ============================================================
# Defective Box Detection Training
# Auto Resume Support for Google Colab
# ============================================================

# ---------- Install ----------
import sys
import subprocess

subprocess.run(
    [sys.executable, "-m", "pip", "install", "-q", "ultralytics"],
    check=True
)

# ---------- Import ----------
import os
import csv
from google.colab import drive
from ultralytics import YOLO

# ---------- Mount Google Drive ----------
drive.mount("/content/drive", force_remount=False)

# ============================================================
# Project Settings
# ============================================================

PROJECT_DIR = "/content/drive/MyDrive/Two classes New Classifying defective boxes"
DATASET_DIR = os.path.join(PROJECT_DIR, "dataset")
RUNS_DIR = os.path.join(PROJECT_DIR, "runs")

# ============================================================
# Model Settings
# ============================================================

MODEL_NAME = "yolov8l"  # yolov8s / yolov8m / yolov8l
RUN_NAME = "yolov8l_100e_two_classes"

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
print(f"Model      : {MODEL_NAME}")
print(f"Run Name   : {RUN_NAME}")
print(f"Epochs     : {EPOCHS}")
print(f"Image Size : {IMAGE_SIZE}")
print(f"Batch Size : {BATCH_SIZE}")
print("=" * 60)

# ============================================================
# Resume Helpers
# ============================================================

RUN_DIR = os.path.join(RUNS_DIR, RUN_NAME)
LAST_PT = os.path.join(RUN_DIR, "weights", "last.pt")
BEST_PT = os.path.join(RUN_DIR, "weights", "best.pt")
RESULTS_CSV = os.path.join(RUN_DIR, "results.csv")


def isTrainingComplete(resultsCsv, targetEpochs):
    """results.csv에서 목표 epoch까지 끝났는지 확인합니다."""
    if not os.path.exists(resultsCsv):
        return False

    with open(resultsCsv, newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        return False

    lastEpoch = int(float(rows[-1]["epoch"])) + 1
    return lastEpoch >= targetEpochs


# ============================================================
# Train or Resume
# ============================================================

if os.path.exists(LAST_PT) and not isTrainingComplete(RESULTS_CSV, EPOCHS):
    print("=" * 60)
    print("Interrupted run found. Resuming training...")
    print(f"Checkpoint : {LAST_PT}")
    print("=" * 60)

    # last.pt에는 모델, optimizer, scheduler, 완료 epoch 상태가 저장됩니다.
    model = YOLO(LAST_PT)
    results = model.train(resume=True)

elif os.path.exists(LAST_PT):
    print("=" * 60)
    print("To start again, change RUN_NAME.")
    print("=" * 60)

else:
    print("=" * 60)
    print("Starting a new training run...")
    print("=" * 60)

    model = YOLO(f"{MODEL_NAME}.pt")

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
# Validation: best.pt
# ============================================================

if os.path.exists(BEST_PT):
    print("\nRunning validation with best.pt...\n")

    bestModel = YOLO(BEST_PT)

    metrics = bestModel.val(
        data=os.path.join(DATASET_DIR, "data.yaml"),
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        workers=WORKERS
    )

    print("=" * 60)
    print("Training Completed!")
    print(f"Results Folder : {RUN_DIR}")
    print(f"Best Model     : {BEST_PT}")
    print("=" * 60)

else:
    print("best.pt was not created yet. Run the cell again after training resumes.")
