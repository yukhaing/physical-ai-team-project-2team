# yolo/ — YOLO Detection Team

This folder contains everything related to the YOLO-based box detection 
and classification part of the project.

## Responsibility
Detect boxes in the camera image, classify each as "damaged" or "intact", 
and identify the single best box to pick next (based on confidence and 
occlusion level). Output is handed off to the OMX team for actual robot 
arm movement.

## Subfolders
- `data/` — datasets (Roboflow dataset + our own captured photos)
- `src/` — reusable Python code (detection, occlusion scoring, coordinate conversion)
- `models/` — trained model weight files
- `notebooks/` — training and evaluation notebooks

## Team Members
- [유유카인]
- [하영진]

## Setup
See `requirements.txt` for Python dependencies.