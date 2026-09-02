# notebooks/ — Training & Experimentation

This folder contains training/experimentation code:
- Training the YOLOv8 model (fine-tuning on Roboflow dataset + our own photos)
- Evaluating results (checking mAP50, confusion matrix, etc.)

## Files
- `new_dataset_train_yolo8.py` — current Colab training script for the two-class defective-box dataset. It mounts Google Drive, trains `yolov8s` as `yolov8s_v3` for up to 100 epochs, resumes an incomplete run from `last.pt`, and validates `best.pt` after training.
- `train_yolov8l.py` — YOLOv8l training script (Colab, Drive-mounted dataset)
- `YOLOsize_comparison.ipynb` — Colab notebook comparing YOLOv8 s/m/l runs (mAP50, mAP50-95, precision, recall), with outputs

## Current training run

`new_dataset_train_yolo8.py` expects the following Google Drive layout:

```text
Two classes New Classifying defective boxes/
├── dataset/data.yaml
└── runs/yolov8s_v3/
    └── weights/
        ├── last.pt
        └── best.pt
```

For a new run it uses automatic batch sizing (`batch=0.9`), 640-pixel images,
and four workers. When `runs/yolov8s_v3/weights/last.pt` exists but the
configured 100 epochs are not complete, it resumes that run; when it is
complete, change `RUN_NAME` before starting another run.

## Note
Code here is for experimentation. Once something works reliably, 
move the clean version into src/ as a proper .py script.
