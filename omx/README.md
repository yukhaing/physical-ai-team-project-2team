# OMX 통합 관제 및 불량 이송 프로토타입

이 디렉터리는 OMX-F 로봇팔의 카메라 기반 박스 집기 흐름, YOLO 검출 표시,
불량 박스 선택용 관제 GUI, 작업 SQLite 로그를 포함합니다. 현재 구현의 목적은
통합 관제 시나리오와 OMX-Beagle 연계 동작을 실제 실행 흐름 기준으로 검증하는 것입니다.

통합 관제 절차 문서는 저장소 루트의 `integration/` 폴더에서 함께 관리합니다.

## 구성

- `src/omx_box_control/`: ROS 2 Jazzy 패키지와 OMX 집기/적재 노드
- `models/best.pt`: normal/defect YOLO 모델
- `docker/`: OMX-F와 Cyclo MoveJ controller용 독립 Docker 환경

관제 GUI는 카메라 영상을 normal/defect 박스와 함께 표시합니다. 현재는
불량 박스만 선택 대상으로 사용하며, 선택 후 OMX 집기 흐름을 시작하는
관제 절차를 검증합니다. `beagle_adapter_node.py`는 TCP 기반으로 Beagle 미션과
연결되어 적재 완료 트리거와 상태 수신을 담당하며, 같은 PC 또는 별도 제어 PC
배치를 모두 지원합니다.

## 현재 구현 상태

### 구현 완료
- Qt 기반 통합 관제 GUI
- YOLO 검출 결과 표시와 불량 박스 선택
- OMX pick coordinator 시작/계속/정지 흐름 연결
- Beagle 트리거/상태 TCP 연동
- OMX 적재 후 Beagle 이동, 하역 완료, 복귀 사이클
- 작업 이력 SQLite 기록
- 2-PC 배치 기준 통합 관제 시나리오 테스트

### 더미 또는 임시 처리
- 실환경 기준 최종 캘리브레이션 값 반영

### 외부 입력 대기
- 실제 환경에서 측정한 최종 캘리브레이션 값
- 팀원에게 받은 보정값 반영 후 좌표 정합 재검증

## 초기 설정

Linux 또는 USB 장치가 전달된 WSL 환경에서 실행합니다. OMX-F와 카메라는 각각
`/dev/ttyACM0`, `/dev/video0`으로 확인되어야 합니다.

처음 실행하는 PC이거나 코드를 새로 pull한 뒤 첫 실행이라면 아래 순서로
컨테이너와 워크스페이스를 먼저 준비합니다.

```bash
cd /path/to/physical-ai-team-project-2team/omx/docker
chmod +x container.sh
./container.sh start
./container.sh enter
```

컨테이너 안에서 ROS 패키지를 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/omx_box_project_ws
colcon build --symlink-install
source install/setup.bash
```

`docker-compose.yml`의 워크스페이스 마운트는 현재 `omx/` 폴더 구조에 맞게
`/root/omx_box_project_ws`로 연결되어야 합니다.

## GUI 빠른 실행

한 번 빌드가 끝난 뒤에는 아래 한 줄로 GUI 실행에 필요한 항목을 모두 시작할 수
있습니다.

```bash
cd /path/to/physical-ai-team-project-2team/omx/docker
./container.sh gui-up
```

이 명령은 `zenohd`, OMX bringup, MoveJ controller, 카메라, 통합 GUI를 순서대로
실행합니다.

상태 확인:

```bash
./container.sh gui-status
```

종료:

```bash
./container.sh gui-down
```

## 수동 실행 순서

각 ROS 명령은 컨테이너의 별도 터미널에서 실행하고, 매번 아래 환경을 먼저
불러옵니다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/omx_box_project_ws/install/setup.bash
```

1. OMX-F bringup

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py \
  start_rviz:=false port_name:=/dev/ttyACM0
```

2. Cyclo MoveJ controller

```bash
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py \
  controller_type:=movej start_interactive_marker:=false \
  config_file:=/root/omx_box_project_ws/docker/config/omx_config_physical.yaml
```

3. USB 카메라

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
  name:=camera1 video_device:=/dev/video0
```

4. 통합 관제 프로그램

```bash
export PATH=/opt/ultralytics-venv/bin:$PATH
ros2 launch omx_box_control integrated_console.launch.py
```

통합 launch는 YOLO, homography 좌표 변환, pick coordinator, Beagle 시뮬레이터,
SQLite 로거와 Qt 관제 GUI를 함께 시작합니다.

현재 설정은 테스트 편의를 위해 일부 자동 진행 또는 우회 옵션이 켜져 있을 수
있습니다. 실장비 검증 전에는 `config/console.yaml`의 Beagle 우회, 상태 대기,
자동 진행 설정을 반드시 확인해야 합니다.

## GUI 작업 순서

1. `가동`을 눌러 시스템을 활성화합니다.
2. 카메라의 감지 박스 근처를 클릭합니다. 현재는 불량 박스만 선택 대상입니다.
3. Beagle이 `READY` 상태인지 확인한 뒤 불량 박스 선택과 적재 사이클을 시작합니다.
4. OMX가 수령 위치 Beagle 위로 집기·배치를 완료하면 `box_placed`가 자동 전송됩니다.
5. 하역 완료 후 Beagle 복귀까지 끝나면 다음 사이클을 진행합니다.
6. 성공 완료 시 SQLite에는 전체 작업 정보가 남고, GUI 로그에는 완료 시간과
   분류가 표시됩니다.

SQLite 로그 위치는 `/root/omx_box_project_ws/logs/operations.sqlite3`입니다.

## 캘리브레이션 주의

현재 좌표 변환과 관련된 보정값은 실환경 최종값이 아닐 수 있습니다. 실제 작업
위치 정확도는 팀원에게 전달받을 최종 캘리브레이션 값 반영 후 다시 확인해야
합니다. 최종 보정값을 받으면 관련 YAML 또는 runtime calibration 파일을 교체하고
집기 좌표 오차를 재검증해야 합니다.

## 안전 주의

MoveJ와 MoveL controller를 동시에 실행하지 마십시오. GUI의 `비상정지`는
현재 명령 취소·자세 유지·Beagle 정지 요청을 보내는 소프트웨어 정지입니다.
즉시 물리 정지를 보장하지 않으므로 위험 상황에서는 반드시 OMX의 물리 E-stop을
사용해야 합니다.

## 다음 변경 예정

- Beagle 정지/예외 처리 프로토콜 보강
- 팀원에게 받은 최종 캘리브레이션 값 반영
- 테스트용 자동 진행/우회 설정과 실운영 설정 분리
- GUI/시나리오 테스트 기준 문서와 실운영 문서 분리
