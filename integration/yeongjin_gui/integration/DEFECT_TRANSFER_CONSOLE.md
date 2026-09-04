# OMX 불량 박스 이송 통합 관제

이 문서는 YOLO가 검출한 `defect` 박스만 OMX와 Beagle 시나리오로 이송하는
통합 관제의 현재 구현 상태와 GUI 동작 테스트 절차를 설명한다. Beagle은 TCP
어댑터를 통해 같은 PC 또는 별도 제어 PC에서 실행할 수 있다.

## 동작 흐름

1. YOLO가 `defect` 박스를 검출한다. `normal` 박스는 선택과 이송 대상에서 제외된다.
2. Beagle이 수령 위치에서 `WAIT_SIGNAL` 상태를 보내면 관제가 준비 상태가 된다.
3. 불량 박스를 선택하면 OMX가 집어서 수령 위치의 Beagle 위에 놓는다.
4. 적재 완료 후 관제가 Beagle에 `box_placed` 신호를 보낸다.
5. Beagle은 불량 구역으로 이동해 대기한다.
6. 하역 OMX가 Beagle 위의 박스를 집고, 180도 뒤쪽에 내려놓은 다음 대기 자세로 복귀한다.
7. 하역 완료를 확인한 관제가 Beagle에 복귀 신호를 보내고 다음 사이클을 허용한다.

## 하역 OMX 구성

하역 동작은 기존 적재 동작을 반대로 구성했다.

1. 하역 대기 자세로 이동하고 그리퍼를 연다.
2. 기존 적재 OMX의 검증된 place 상단 Z로 접근한다.
3. 기존 place Z까지 수직 하강해 Beagle 위 박스를 집는다.
4. 같은 상단 Z로 상승한다.
5. `joint1`을 180도 회전해 반대편 배출 위치로 이동한다.
6. 수직 하강해 박스를 놓고 다시 상승한다.
7. 빈 그리퍼를 원위치 방향으로 회전한 뒤 대기 자세로 복귀한다.

각 단계는 관절 피드백이 목표 오차 안에 들어온 경우에만 다음 단계로 진행한다. 하역
OMX는 `/unload_omx` 네임스페이스를 사용하므로 기존 적재 OMX 토픽과 섞이지 않는다.

처음에는 반드시 두 컨트롤러의 영구 장치 경로를 확인한다.

```bash
ls -l /dev/serial/by-id/
```

두 OMX가 연결된 상태에서는 `/dev/ttyACM0`처럼 번호가 바뀔 수 있는 이름을 사용하지
않는다. 적재 OMX와 하역 OMX에 서로 다른 `usb-ROBOTIS_OpenRB-150_...` 경로를 지정한다.

하역 설정 파일 `config/unload_coordinator.yaml`의 `dry_run` 값으로 실제 명령 전송 여부를
제어한다.
별도 OMX의 베이스 설치 위치가 다르면 같은 XY 수치라도 실제 위치가 달라지므로,
`source_xy`가 Beagle 위 박스 중심과 맞는지 빈 그리퍼로 먼저 확인해야 한다. 현재 초기값은
기존 적재 OMX의 place 좌표 `[-0.02621, -0.17335]`를 재사용한다.

하역 OMX만 기동해 계획을 확인하는 명령은 다음과 같다.

```bash
ros2 launch omx_box_control unload_omx_system.launch.py \
  port_name:=/dev/serial/by-id/<하역_OMX_ID>

ros2 service call /unload_omx/unload_coordinator/start \
  std_srvs/srv/Trigger "{}"
```

`dry_run=true`에서는 관절 목표만 계산해 응답하고 로봇 명령은 보내지 않는다. 빈 공간,
Beagle 정차 위치, 그리퍼 중심을 확인한 뒤에만 `dry_run: false`로 바꾼다.

최종적으로는 아래 한 명령으로 적재 OMX, 하역 OMX, Beagle, 카메라와 GUI를 함께
기동한다.

```bash
OMX_PORT_NAME=/dev/serial/by-id/<적재_OMX_ID> \
UNLOAD_OMX_PORT_NAME=/dev/serial/by-id/<하역_OMX_ID> \
AUTOMATIC_UNLOAD_OMX=true \
BEAGLE_MODE=local \
./docker/container.sh gui-up
```

`AUTOMATIC_UNLOAD_OMX=false`이면 하역 OMX가 연결되어 있어도 Beagle 도착 후 기존
`하역 완료` 버튼을 사용하는 수동 방식이 유지된다. 자동 하역 중 오류가 발생해도 Beagle은
정지 상태를 유지하고 GUI 버튼이 다시 활성화되어 수동 하역 후 복귀시킬 수 있다.

## Ubuntu 사전 준비

호스트에서 Docker와 X11 실행 권한이 필요하다.

```bash
sudo apt-get update
sudo apt-get install -y docker.io xauth x11-xserver-utils
```

Docker 이미지에는 `fonts-noto-cjk`, `ko_KR.UTF-8` 로케일이 포함되어 있어 Qt 화면의 한글이 깨지지 않도록 구성되어 있다. Dockerfile을 변경했거나 처음 실행한다면 이미지를 다시 빌드해야 한다.

## 초기 1회 세팅

처음 실행하는 PC이거나 소스 코드를 새로 pull한 뒤 처음 실행하는 경우에는 먼저
컨테이너를 빌드하고 ROS 워크스페이스를 한 번 빌드한다.

Ubuntu 호스트 터미널에서 실행한다.

```bash
cd physical-ai-team-project-2team/omx/docker
chmod +x container.sh
./container.sh build
./container.sh start
./container.sh enter
```

컨테이너 안에서 OMX 워크스페이스를 빌드한다.

```bash
cd /root/omx_box_project_ws
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`install` Docker 볼륨은 재시작 뒤에도 유지된다. Python 또는 launch 파일을 수정한 뒤에는 위 `colcon build --symlink-install`을 다시 실행한다.

## 빠른 실행

한 번 빌드가 끝난 뒤에는 호스트 터미널에서 아래 한 줄로 GUI 실행에 필요한 프로세스를
순서대로 모두 시작할 수 있다.

```bash
cd physical-ai-team-project-2team/omx/docker
./container.sh gui-up
```

Beagle 제어 프로그램을 같은 PC에서 실행할 때:

```bash
BEAGLE_MODE=local ./container.sh gui-up
cd ../integration/yeongjin_gui/Beagle_mobile_robot
python3 missions/receiving_defect_shuttle.py \
  --trigger-port 8765 --status-host 127.0.0.1 --status-port 9000
```

Beagle을 별도 PC에서 실행할 때는 GUI를 다음처럼 시작한다. `auto`는 상태 연결의
상대 IP를 자동으로 사용하고, 고정 IP가 필요하면 `remote`를 사용한다.

```bash
BEAGLE_MODE=auto ./container.sh gui-up
# 또는
BEAGLE_MODE=remote BEAGLE_TRIGGER_HOST=<BEAGLE_PC_IP> ./container.sh gui-up
```

배포 방식은 `BEAGLE_MODE`만 바꾸며, 나중에 한 방식을 제거해도 OMX 및 GUI의
사이클 로직은 수정할 필요가 없다. 통신 구현은 `beagle_adapter_node.py`에만 있다.

이 명령은 다음 항목을 자동으로 실행한다.

1. `zenohd`
2. OMX-F bringup
3. Cyclo MoveJ controller
4. USB 카메라
5. 통합 관제 GUI

실행 상태 확인:

```bash
./container.sh gui-status
```

전체 종료:

```bash
./container.sh gui-down
```

## 수동 실행

`gui-up` 대신 각 프로세스를 수동으로 띄우고 싶다면 아래 순서를 따른다.

각 명령은 컨테이너의 별도 터미널에서 실행한다. 실행 전에는 모두 다음 환경을 불러온다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/omx_box_project_ws/install/setup.bash
```

1. Zenoh daemon

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

2. OMX-F bringup

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py start_rviz:=false port_name:=/dev/ttyACM0
```

3. Cyclo MoveJ controller

```bash
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py \
  controller_type:=movej \
  start_interactive_marker:=false \
  config_file:=/root/omx_box_project_ws/docker/config/omx_config_physical.yaml
```

4. USB camera

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=camera1 video_device:=/dev/video0
```

5. 통합 관제를 시작한다.

```bash
export PATH=/opt/ultralytics-venv/bin:$PATH
ros2 launch omx_box_control integrated_console.launch.py
```

현재 launch는 GUI/상태 전이 테스트 목적의 설정을 포함할 수 있으므로, 실장비
검증 전에는 `config/console.yaml`의 자동 진행 및 Beagle 우회 옵션을 확인한다.

## 관제 화면 조작

1. Beagle 미션을 실행하고 GUI에 `READY`가 표시되는지 확인한다.
2. `가동`을 눌러 시스템을 활성화한다.
3. YOLO가 불량 박스를 검출하면 OMX 집기·배치가 자동 진행된다.
4. OMX가 Beagle 위에 놓기를 완료하면 `box_placed`가 자동 전송된다.
5. Beagle이 불량 구역을 왕복하는 동안 다음 사이클은 잠긴다.
6. Beagle이 수령 위치로 돌아와 `READY`가 표시되면 다음 박스를 처리한다.

`정지`는 현재 소프트웨어 명령을 취소하고 정지 요청을 보낸다. `비상정지`도 소프트웨어 수준의 요청이므로, 위험 상황에서는 반드시 OMX 장비의 물리 E-stop을 함께 사용한다. 비상정지 뒤에는 하드웨어를 점검하고 `리셋` 후 다시 `가동`한다.

## 현재 한계와 후속 작업

- Beagle 원격 정지는 팀원 미션 프로토콜에 아직 정의되지 않았다. 긴급 상황에서는
  각 장비의 물리 정지를 사용해야 한다.
- 현재 캘리브레이션 값은 실제 환경 기준 최종 보정값이 아닐 수 있다.
- 최종 보정값은 추후 팀원에게 받아 반영하고, 좌표 변환 및 집기 정확도를 다시 확인해야 한다.
- Beagle 없이 OMX만 운용할 때는 `bypass_beagle: true`로 전환할 수 있다.

## 문제 해결

- Qt 창이 열리지 않으면 Ubuntu 호스트에서 `echo $DISPLAY`를 확인하고, 컨테이너는 `./container.sh start`로 시작한다. 이 스크립트가 root 사용자에 대한 X11 접근을 설정한다.
- 한글이 네모로 표시되면 이미지를 다시 빌드한다. 컨테이너에서 `fc-match "Noto Sans CJK KR"`로 폰트를 확인할 수 있다.
- YOLO가 시작되지 않으면 컨테이너에서 `/root/omx_box_project_ws/models/best.pt` 존재 여부와 카메라 토픽 `/camera1/image_raw`를 확인한다.
- `gui-up` 뒤 카메라가 보이지 않으면 `./container.sh gui-status`와 `/root/omx_box_project_ws/logs/gui_stack/integrated_console.log`를 먼저 확인한다.
- `WAIT_BEAGLE`이 계속되면 Beagle 미션의 `--status-host`, TCP 9000 방화벽,
  GUI의 `BEAGLE_MODE`를 확인한다.
- 상태가 예상보다 빨리 넘어가면 `config/console.yaml`의 `bypass_beagle`, `auto_start_omx`, `auto_continue_pick`, `auto_complete_unload` 값을 먼저 확인한다.
- 좌표가 실제 작업 위치와 다르면 최신 캘리브레이션 값이 반영되었는지 확인한다. 현재 값은 임시값일 수 있으며, 팀원에게 받은 최종 보정값으로 교체해야 한다.
