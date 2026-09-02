# physical-ai-team-project-2team

## 불량 박스 검출·이송 통합 시스템

YOLOv8로 불량(`defect`) 박스를 검출하고 OMX-F 로봇팔로 Beagle 이동 로봇에
적재한 뒤 불량 구역으로 이송하는 통합 관제 프로토타입입니다.

## 구현 기능

- YOLO `normal`/`defect` 검출과 불량 박스 선택
- Qt 기반 통합 관제 GUI와 SQLite 작업 로그
- OMX 집기·적재, 카메라 좌표 보정, TCP 기반 Beagle 상태·트리거 연동
- 하역 뒤 Beagle이 수령 위치로 복귀하는 동안 다음 불량 박스 집기
- Beagle 수령 위치 도착 확인 뒤 다음 박스 최종 배치
- 집기 실패 복구와 비상정지 뒤 HOME 복귀·그리퍼 해제 절차

## 구조

- `yolo/` — 모델, 데이터셋, 학습·검출 코드
- `omx/` — ROS 2 Jazzy 기반 OMX-F 제어와 Docker 환경
- `integration/` — GUI·Beagle 통합 관제 운영 문서
- `Beagle_mobile_robot/`, `Beagle_Lidar_and_AStar/` — Beagle 셔틀·경로 계획 구현
- `docs/` — 팀 의사결정과 인터페이스 문서

## 실행

초기 Docker/ROS 설정과 GUI·Beagle 실행 절차는
[`integration/DEFECT_TRANSFER_CONSOLE.md`](integration/DEFECT_TRANSFER_CONSOLE.md)를
참고합니다. 빌드 후 GUI 스택은 다음과 같이 실행합니다.

```bash
cd omx/docker
BEAGLE_MODE=auto ./container.sh gui-up
```

실장비 운용 전에는 카메라 보정값과 `config/console.yaml`의 Beagle 우회·자동
진행 설정을 확인해야 합니다. 위험 상황에서는 GUI 소프트웨어 정지와 별도로
OMX 및 Beagle의 물리 E-stop을 사용해야 합니다.

## 팀 구성

- 하영진 — YOLO 검출·분류, 통합 관제 GUI
- 유유카인 — YOLO 검출·분류, Beagle
- 배민서 — OMX 로봇팔 제어
- 최재현 — OMX 로봇팔 제어
