# OMX 통합 관제 및 분류 적재

이 디렉터리는 OMX-F 로봇팔의 카메라 기반 박스 집기, YOLO 분류,
Beagle 적재 위치 대기, 관제 GUI, 작업 SQLite 로그를 포함합니다.

## 구성

- `src/omx_box_control/`: ROS 2 Jazzy 패키지와 OMX 집기/적재 노드
- `models/best.pt`: normal/defect YOLO 모델
- `docker/`: OMX-F와 Cyclo MoveJ controller용 독립 Docker 환경

관제 GUI는 카메라 영상을 normal/defect 박스와 함께 표시합니다. 박스를
클릭하면 분류에 맞는 Beagle 적재 위치 요청을 보내고, Beagle 도착 뒤에만
기존 OMX 집기 흐름을 시작할 수 있습니다. Beagle은 현재 시뮬레이션
어댑터이며, 실제 통신 방식이 결정되면 `beagle_adapter_node.py`만 교체합니다.

## 초기 설정

Linux 또는 USB 장치가 전달된 WSL 환경에서 실행합니다. OMX-F와 카메라는 각각
`/dev/ttyACM0`, `/dev/video0`으로 확인되어야 합니다.

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

## 실행 순서

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
ros2 launch omx_box_control integrated_console.launch.py
```

통합 launch는 YOLO, homography 좌표 변환, pick coordinator, Beagle 시뮬레이터,
SQLite 로거와 Qt 관제 GUI를 함께 시작합니다.

## GUI 작업 순서

1. `가동`을 눌러 시스템을 활성화합니다.
2. 카메라의 감지 박스 근처를 클릭합니다.
3. Beagle 도착 상태 후 `OMX 집기 시작`을 누릅니다.
4. `집기/배치 계속`으로 기존 집기 단계와 grasp 확인 뒤의 배치 단계를 진행합니다.
5. 성공 완료 시 SQLite에는 전체 작업 정보가 남고, GUI 로그에는 완료 시간과
   normal/defect 분류만 표시됩니다.

SQLite 로그 위치는 `/root/omx_box_project_ws/logs/operations.sqlite3`입니다.

## 안전 주의

MoveJ와 MoveL controller를 동시에 실행하지 마십시오. GUI의 `비상정지`는
현재 명령 취소·자세 유지·Beagle stop 요청을 보내는 소프트웨어 정지입니다.
즉시 물리 정지를 보장하지 않으므로 위험 상황에서는 반드시 OMX의 물리 E-stop을
사용해야 합니다.
