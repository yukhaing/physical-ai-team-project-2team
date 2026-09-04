# Beagle LiDAR/A* 셔틀 미션

통합 GUI가 로컬 Beagle을 실행할 때 사용하는 현재 미션은
`scripts/10_shuttle_mission.py`다. `Beagle_mobile_robot/.venv`의 Robomation 실행
환경을 재사용하며, 이 디렉터리의 지도·경로 계획·도킹 로직으로 운행한다.

## 통합 실행

저장소 루트에서 실행한다.

```bash
BEAGLE_MODE=local ./docker/container.sh gui-up
```

Robomation EXPRESS RECEIVER가 하나 감지되면 미션이 자동 시작된다. 자동 실행에는
다음 파일이 필요하다.

```text
integration/yeongjin_gui/Beagle_mobile_robot/.venv/bin/python
```

가상환경은 Git에 포함되지 않는다. 새 PC에서는 해당 경로에 환경을 만들고
`Beagle_mobile_robot/requirements.txt`와 Robomation이 제공하는 `roboid`를 설치한다.

## 통신 상태와 명령

미션은 TCP 8765에서 GUI 명령을 받고 TCP 9000으로 상태와 heartbeat를 보낸다.

- `box_placed`: 적재 완료, 하역 구역으로 출발
- `box_picked`: 하역 OMX가 박스를 들어 올림, 수령 위치로 복귀
- `operator_unloaded`: `box_picked`의 호환용 별칭
- `WAIT_PICKED`: 하역 구역 도착 후 박스 수거를 기다리는 상태

모든 사이클 명령은 현재 `job_id`를 포함한다. 같은 작업의 중복 신호는 다음 운행을
중복 시작시키지 않아야 한다.

## 운행 데이터

- `config/course_config.json`: 속도, LiDAR, 도킹, 정렬 및 구역 설정
- `data/map_points.json`: 운행 지도 점군
- `data/obstacle_map.json`: 장애물 지도
- `data/receiving_reference_scan.json`: 수령 위치 정렬 기준
- `data/defect_reference_scan.json`: 하역 위치 정렬 기준

지도나 트레이 위치를 바꿨다면 실제 주행 전에 바퀴를 띄운 저속 시험과 1회 왕복 시험을
먼저 수행한다. 주행 중에는 GUI 소프트웨어 정지만 믿지 말고 물리적인 비상 정지 수단을
준비한다.
