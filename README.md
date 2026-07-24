# physical-ai-team-project-2team
Box detection and sorting

Box detection (damaged/intact classification) and pick-and-place system using OMX robot arm + YOLOv8.

## Team Members
- [하영진]   — YOLO (detection & classification)
- [유유카인] — YOLO (detection & classification)
- [배민지]  — OMX (robot arm control)
- [최재현]  — OMX (robot arm control)

## Project Structure
- `yolo/` — YOLO training, dataset, and detection logic
- `omx/` — OMX robot arm control code
- `integration/` — Combined pipeline connecting YOLO output to OMX control
- `docs/` — Team decisions, data contract, and other shared documentation

## Setup
See `yolo/requirements.txt` for Python dependencies.
