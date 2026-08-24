# OMX 불량 박스 이송 통합 관제

이 문서는 YOLO가 검출한 `defect` 박스만 OMX와 Beagle로 이송하는 통합 관제의 초기 설정과 조작 순서를 설명한다.

## 동작 흐름

1. YOLO가 `defect` 박스를 검출한다. `normal` 박스는 선택과 이송 대상에서 제외된다.
2. 관제 화면에서 불량 박스를 선택하면 Beagle이 불량 적재 위치로 이동한다.
3. Beagle 도착 상태를 확인한 뒤 OMX 집기/적재를 진행한다.
4. OMX 적재가 끝나면 시스템은 작업자 하역 완료 신호를 기다린다.
5. 작업자가 박스를 내린 뒤 `작업자 하역 완료: 원위치 복귀`를 누르면 Beagle이 원위치로 복귀한다.
6. Beagle 복귀 완료 후 다음 불량 박스를 처리할 수 있다.

## Ubuntu 사전 준비

- Docker Engine과 Docker Compose plugin을 설치한다.
- OMX-F는 `/dev/ttyACM0`, USB 카메라는 `/dev/video0`인지 확인한다.
- Ubuntu 데스크톱 세션에서 X11 또는 XWayland가 동작하고 `DISPLAY` 환경 변수가 설정되어 있어야 한다.
- YOLO 가중치 파일은 `omx/models/best.pt`에 둔다.

Docker 이미지에는 `fonts-noto-cjk`, `ko_KR.UTF-8` 로케일이 포함되어 있어 Qt 화면의 한글이 깨지지 않도록 구성되어 있다. Dockerfile을 변경했거나 처음 실행한다면 이미지를 다시 빌드해야 한다.

## 컨테이너 시작과 빌드

Ubuntu 호스트 터미널에서 실행한다.

```bash
cd physical-ai-team-project-2team/omx/docker
./container.sh start
./container.sh enter
```

컨테이너 안에서 ROS 패키지를 빌드하고 환경을 불러온다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/omx_box_project_ws
colcon build --symlink-install
source install/setup.bash
```

`install` Docker 볼륨은 재시작 뒤에도 유지된다. Python 또는 launch 파일을 수정한 뒤에는 위 `colcon build --symlink-install`을 다시 실행한다.

## 실행

각 명령은 컨테이너의 별도 터미널에서 실행한다. 실행 전에는 모두 다음 환경을 불러온다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/omx_box_project_ws/install/setup.bash
```

1. OMX-F bringup을 시작한다.

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py start_rviz:=false port_name:=/dev/ttyACM0
```

2. Cyclo MoveJ controller를 시작한다.

```bash
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py \
  controller_type:=movej start_interactive_marker:=false \
  config_file:=/root/omx_box_project_ws/docker/config/omx_config_physical.yaml
```

3. 카메라를 시작한다.

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=camera1 video_device:=/dev/video0
```

4. 통합 관제를 시작한다.

```bash
ros2 launch omx_box_control integrated_console.launch.py
```

## 관제 화면 조작

1. `가동`을 눌러 시스템을 활성화한다.
2. 카메라 화면에서 검출된 불량 박스 가까이를 클릭한다. 정상 박스만 있으면 선택할 수 없다.
3. 상태가 `BEAGLE_ARRIVED`가 된 뒤 `OMX 집기 시작`을 누른다.
4. 상태가 `TARGET_READY`가 되면 `집기/배치 계속`으로 OMX의 검증된 집기·적재 절차를 진행한다.
5. `OMX_COMPLETE` 상태에서는 작업자가 불량 박스를 하역한다.
6. 하역이 끝난 뒤 `작업자 하역 완료: 원위치 복귀`를 한 번 누른다.
7. `BEAGLE_HOME` 상태가 표시되면 다음 불량 박스를 선택한다.

`정지`는 현재 소프트웨어 명령을 취소하고 정지 요청을 보낸다. `비상정지`도 소프트웨어 수준의 요청이므로, 위험 상황에서는 반드시 OMX 장비의 물리 E-stop을 함께 사용한다. 비상정지 뒤에는 하드웨어를 점검하고 `리셋` 후 다시 `가동`한다.

## 문제 해결

- Qt 창이 열리지 않으면 Ubuntu 호스트에서 `echo $DISPLAY`를 확인하고, 컨테이너는 `./container.sh start`로 시작한다. 이 스크립트가 root 사용자에 대한 X11 접근을 설정한다.
- 한글이 네모로 표시되면 이미지를 다시 빌드한다. 컨테이너에서 `fc-match "Noto Sans CJK KR"`로 폰트를 확인할 수 있다.
- YOLO가 시작되지 않으면 컨테이너에서 `/root/omx_box_project_ws/models/best.pt` 존재 여부와 카메라 토픽 `/camera1/image_raw`를 확인한다.
- Beagle 실제 통신 방식이 확정되기 전에는 `beagle_adapter_node.py`가 도착/복귀를 시간 기반으로 시뮬레이션한다. 실제 Beagle API 연동 시 이 어댑터만 교체한다.
