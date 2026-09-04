# src/ — YOLO Detection Logic Code

This folder contains the actual Python scripts for the YOLO detection pipeline.

## Files (to be added)
- `detect.py` — main function that runs YOLO detection and returns box location + class
- `occlusion.py` — calculates how much each detected box is covered/overlapped by others
- `coordinate_convert.py` — converts pixel coordinates (from camera) into robot coordinates (for OMX arm)
- `capture.py` — script to capture photos from the C270 webcam

## Purpose
This is where all reusable code lives (not experimental/notebook code). 
Final integration script will import functions from here.