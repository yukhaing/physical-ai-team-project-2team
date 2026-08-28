# physical-ai-team-project-2team
Box defect detection and transfer console

카메라 영상에서 YOLOv8로 박스를 검출하고, 그중 불량 박스를 관제 GUI에서 선택해
OMX 로봇팔 적재 흐름으로 넘기고, Beagle 이송 사이클까지 연결한 통합 관제
프로토타입입니다. 현재 저장소의 중심은 불량 박스 이송 시나리오 검증과 실제
장비 연동입니다.

- **검출/분류**: YOLOv8, Roboflow 데이터셋(Box/Damaged Box, 총 926장) 기반 학습
- **관제/로봇 제어**: OMX 로봇팔의 기존 pick-and-place 흐름을 GUI와 연결
- **통합 방식**: 별도 `main_pipeline.py`가 아니라 OMX launch/노드 내부에서
  YOLO 표시, 선택, 상태 전이, 로봇 흐름을 결합

  
## Team Members
- [하영진]   — YOLO (detection & classification)
- [유유카인] — YOLO (detection & classification)
- [배민서]  — OMX (robot arm control)
- [최재현]  — OMX (robot arm control)

## Project Structure
- `yolo/` — YOLO training, dataset, and detection logic
- `omx/` — OMX robot arm control code
- `integration/` — 통합 관제 문서와 YOLO-OMX 연계 설명
- `docs/` — Team decisions, data contract, and other shared documentation
- `reports/` — Weekly, individual, and final reports for LMS submission
- `photos/` — Photos/screenshots attached to reports
- `results/` — Evaluation results and demo video links

## Setup
See `yolo/requirements.txt` for Python dependencies.

OMX 통합 관제 GUI의 초기 세팅과 실행 방법은 `omx/README.md` 와
`integration/DEFECT_TRANSFER_CONSOLE.md` 를 우선 참고한다. 현재 OMX 쪽은
`./container.sh gui-up` 으로 실행에 필요한 프로세스를 한 번에 올릴 수 있다.

## Current Status

### Implemented
- Qt 기반 통합 관제 GUI
- YOLO 검출 결과 표시와 불량 박스 선택
- OMX pick coordinator와 상태 연동
- Beagle TCP 기반 트리거/상태 연동
- OMX 적재 후 Beagle 이동, 하역 완료, 복귀 사이클
- 작업 로그 SQLite 기록
- Beagle 분리 실행 또는 2-PC 배치 지원

### Using Dummy/Temporary Values
- 현재 캘리브레이션 값은 실제 환경 기준 최종값이 아니며 임시값 기준으로 테스트
- 최종 좌표 보정값은 추후 팀원에게 받아 반영 예정

### To Be Updated Next
- Beagle 정지/예외 상황 프로토콜 보강
- 팀원에게 받은 최종 캘리브레이션 값으로 좌표 변환 재검증
- 테스트용 자동 진행/우회 설정과 실운영 설정 분리
- 통합 관제 문서와 실운영 문서 경계 재정리
