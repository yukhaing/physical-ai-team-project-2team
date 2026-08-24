# notebooks/ — Training & Experimentation

This folder contains Jupyter notebooks used for:
- Training the YOLOv8 model (fine-tuning on Roboflow dataset + our own photos)
- Evaluating results (checking mAP50, confusion matrix, etc.)
- Quick testing/experimentation before turning working code into clean scripts in src/

## Files (to be added)
- `train.ipynb` — main training notebook
- `evaluate.ipynb` — model evaluation and metrics review

## Note
Code here is for experimentation. Once something works reliably,
move the clean version into src/ as a proper .py script.