# physical-ai-team-project-2team
파손박스 검출 및 분류 시스템 (YOLO + OMX 로봇팔 + Beagle 이동 로봇)

카메라로 촬영한 이미지에서 YOLOv8로 박스를 검출해 정상(intact)/파손(defect) 박스를 분류하고, 좌표 보정을 거쳐 적재 OMX-F가 박스를 집습니다. 파손 박스는 Beagle 이동 로봇이 LiDAR 기반 A*+Pure Pursuit 주행으로 불량 구역까지 운반하고, 하역 OMX-F가 트레이에서 박스를 집어 하역합니다. Beagle은 OMX 로봇팔과 TCP 신호(box_placed / box_picked)로 동기화되어 자동으로 왕복합니다.

- **검출/분류**: YOLOv8 기반 정상/파손 박스 검출
- **좌표 보정**: 7-point 카메라 Calibration으로 영상 좌표를 로봇 좌표로 변환
- **로봇팔 제어**: OMX-F 2대 (적재용/하역용), Analytic IK + Cyclo MoveJ로 Pick & Place
- **이동 로봇**: Beagle — LiDAR + A* + Pure Pursuit 주행, point-cloud map 기반 정밀 정렬(find_pose_via_map)로 zone 도착 정확도 확보
- **연동**: OMX ↔ Beagle TCP 신호(box_placed/box_picked)로 동기화, 통합 관제 GUI로 전체 상태 모니터링

## Final Deliverables
- 최종 보고서: [`reports/final/`](<reports/final/[피지컬AI-2팀] 파손박스 검출 및 분류 시스템 최종보고서.pdf>)
- 시연 영상: https://youtu.be/ZgOZATNfR4I?si=Yb7ebuxFRUER3OmE

## Team Members
- 배민서 — YOLO 검출 환경 구축, 7-point Calibration, Docker 실행 환경
- 유유카인 — Beagle LiDAR+A* 주행·정렬 개발, OMX-Beagle 신호연동,YOLOv8 s/m/l 모델 학습·성능 비교 지원
- 최재현 — OMX Pick Coordinator 연동, Calibration 검증, ROS2/Docker 환경
- 하영진 — YOLO 모델 구축·비교, 통합 관제 GUI, OMX-Beagle 상태 연동

## Project Structure
- `yolo/` — YOLO 학습, 데이터셋, 검출 로직
- `omx/` — OMX-F 로봇팔 제어, 좌표 보정, Pick & Place
- `Beagle_Lidar_and_AStar/` — **Beagle 이동 로봇 최종 버전** (LiDAR 기반 A*+Pure Pursuit 주행, zone 정렬, OMX 신호 연동) — 실제 제출/시연에 사용
- `Beagle/`, `Beagle_mobile_robot/` — Beagle 이전 버전 구현, `Beagle_Lidar_and_AStar/`로 대체됨 (개발 과정 기록용으로 보존)
- `integration/` — YOLO-OMX-Beagle 통합 파이프라인, 상태 연동, GUI
- `docs/` — 팀 의사결정, 데이터 계약 등 공유 문서
- `reports/` — LMS 제출용 보고서 (`weekly/`, `individual/`, `final/`)
- `photos/` — 보고서 첨부 사진/스크린샷
- `results/` — 평가 결과, 시연 영상 링크 (위 Final Deliverables 참고)

## Setup
- YOLO: `yolo/requirements.txt`
- Beagle: `Beagle_Lidar_and_AStar/requirements.txt`, 실행법은 `Beagle_Lidar_and_AStar/scripts/10_shuttle_mission.py` 참고
