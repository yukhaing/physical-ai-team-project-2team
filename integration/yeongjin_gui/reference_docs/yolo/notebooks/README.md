# notebooks/ — Training & Experimentation

This folder contains training/experimentation code:
- Training the YOLOv8 model (fine-tuning on Roboflow dataset + our own photos)
- Evaluating results (checking mAP50, confusion matrix, etc.)

## Files
- `train_yolov8l.py` — YOLOv8l training script (Colab, Drive-mounted dataset)
- `YOLOsize_comparison.ipynb` — Colab notebook comparing YOLOv8 s/m/l runs (mAP50, mAP50-95, precision, recall), with outputs

## Note
Code here is for experimentation. Once something works reliably, 
move the clean version into src/ as a proper .py script.