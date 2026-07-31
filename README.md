# physical-ai-team-project-2team
Box detection and sorting

카메라로 촬영한 이미지에서 YOLOv8로 박스를 검출하고 정상(intact)/손상(damaged) 2개 클래스로 분류한 뒤, 검출 결과(클래스, confidence, 로봇 base 기준 좌표)를 OMX 로봇 팔에 전달해 정상 박스와 손상 박스를 각각 다른 위치로 pick-and-place하는 시스템입니다.

- **검출/분류**: YOLOv8, Roboflow 데이터셋(Box/Damaged Box, 총 926장) 기반 학습 
- **로봇 제어**: OMX 로봇 팔로 검출된 박스를 집어 클래스별 목적지에 배치
- **연동**: YOLO의 `detect_and_select()`가 반환한 결과를 OMX의 `pick_and_place()`에 전달하는
  파이프라인으로 두 팀의 코드를 결합 

  
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
- `reports/` — Weekly, individual, and final reports for LMS submission
- `photos/` — Photos/screenshots attached to reports
- `results/` — Evaluation results and demo video links

## Setup
See `yolo/requirements.txt` for Python dependencies.
