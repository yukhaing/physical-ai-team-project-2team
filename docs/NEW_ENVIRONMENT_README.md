# OMX Box Project: 새 작업 환경 실행 안내

> 이 문서는 단일 적재 OMX 개발 환경의 과거 재현 절차다. 현재 듀얼 OMX·Beagle·GUI
> 통합 환경은 [프로젝트 루트 README](../README.md)와
> [통합 관제 문서](../integration/yeongjin_gui/integration/DEFECT_TRANSFER_CONSOLE.md)를
> 따른다.

이 문서는 `cjh-dev` 브랜치를 새로운 PC 또는 새로운 작업 공간에서 받아
7점 camera calibration부터 YOLO 기반 pick-and-place 검증까지 진행하는
순서를 정리한 실행 안내서다.

## 1. 안전 원칙

- 카메라·로봇·작업대를 최종 위치에 고정한 뒤 calibration한다.
- calibration 노드는 로봇을 움직이지 않지만 coordinator의 `start`와
  `continue` 서비스는 실제 동작을 시작한다.
- 최초 검증은 항상 비상 정지가 가능한 상태에서 한 단계씩 진행한다.
- 과거 환경의 calibration은 사용하지 않는다. 현재 환경의 7점 결과 파일이
  없으면 YOLO 시작 스크립트가 의도적으로 실행을 거부한다.
- bringup, MoveJ 또는 coordinator를 중복 실행하지 않는다.

## 2. 저장소 받기

새 PC에서 다음 명령을 실행한다.

```bash
git clone --branch cjh-dev --single-branch \
  https://github.com/yukhaing/physical-ai-team-project-2team.git \
  ~/omx_box_project_ws
cd ~/omx_box_project_ws
git status
git log -1 --oneline
```

`git status`가 깨끗한지 확인한다. YOLO 모델도 받아졌는지 확인한다.

```bash
ls -lh integration/omx_box_system/models/box_defect_best.pt
sha256sum integration/omx_box_system/models/box_defect_best.pt
```

현재 검증한 모델의 SHA-256은 다음과 같다.

```text
253f075b370ff3e9284a052fcd457c6728180e6159de80e13e0cf55bc69f298b
```

## 3. 호스트와 장치 확인

필요한 항목:

- Docker와 Docker Compose
- X11 GUI 환경(RViz, camera calibration 창)
- OMX-F 시리얼 장치
- USB 카메라 장치
- 팀의 `physical_ai_server` 컨테이너

장치를 확인한다.

```bash
ls -l /dev/ttyACM*
ls -l /dev/video*
docker ps --format 'table {{.Names}}\t{{.Status}}'
```

장치 번호는 연결 순서에 따라 바뀔 수 있다. 이 문서에서는 예시로
`/dev/ttyACM0`, `/dev/video0`을 사용한다.

`physical_ai_server`는 YOLO 실행 환경으로 사용되므로 먼저 실행 중이어야 한다.
이 컨테이너를 만드는 팀 공통 스택은 현재 저장소 밖의 선행 조건이다.

## 4. OMX 프로젝트 컨테이너 준비

호스트에서 실행한다.

```bash
cd ~/omx_box_project_ws
./docker/container.sh build
./docker/container.sh start
./docker/container.sh status
```

컨테이너에 들어가 패키지를 빌드한다.

```bash
./docker/container.sh enter
```

컨테이너 내부:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/omx_box_project_ws
colcon build --packages-select omx_box_control --symlink-install
source install/setup.bash
```

## 5. 실제 기준점 7개 입력

다음 설정 파일을 연다.

```text
src/omx_box_control/config/homography_7point_calibration.yaml
```

실제 작업면에서 측정한 `link0` 기준 X/Y를 m 단위로 입력한다.

```yaml
reference_points_link0:
  [x1, y1,
   x2, y2,
   x3, y3,
   x4, y4,
   x5, y5,
   x6, y6,
   x7, y7]
```

기준점은 영상과 작업영역 전체에 넓게 분산시키고, 일직선에 몰리지 않게 한다.
설정 파일의 순서와 카메라 화면 클릭 순서는 반드시 같아야 한다.

## 6. 7점 calibration 실행

Calibration 동안에는 로봇 bringup이나 coordinator가 필요 없다. 세 개의
컨테이너 터미널을 사용한다.

터미널 A — Zenoh:

```bash
cd ~/omx_box_project_ws
./docker/container.sh enter
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

터미널 B — 카메라:

```bash
cd ~/omx_box_project_ws
./docker/container.sh enter
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
  name:=camera1 video_device:=/dev/video0
```

터미널 C — 7점 calibration:

```bash
cd ~/omx_box_project_ws
./docker/container.sh enter
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/omx_box_project_ws/install/setup.bash
ros2 launch omx_box_control \
  camera_homography_7point_calibration.launch.py
```

카메라 창 조작:

```text
c : calibration 시작 또는 처음부터 다시 측정
u : 마지막 기준점 클릭 취소
r : 전체 초기화
v : validation 표시 제거
q : 종료
```

`c`를 누른 뒤 설정 파일에 적은 순서대로 1번부터 7번까지 클릭한다. 결과는
자동으로 다음 경로에 저장된다.

```text
integration/omx_box_system/calibration/omx_camera_homography_7point.yaml
```

저장 후 calibration에 사용하지 않은 실제 지점을 클릭한다. 화면과 로그에
표시되는 `link0 X/Y`를 실측값과 비교한다. 이 확인 클릭은 로봇 target을
발행하지 않는다.

권장 초기 판정 기준:

- 평균 실측 XY 오차: 5 mm 이하
- 최대 실측 XY 오차: 10 mm 이하
- 작업영역 가장자리에서도 축 방향과 부호가 일치

검증이 끝나면 세 터미널을 `Ctrl+C`로 종료한다.

## 7. 통합 시스템 실행

현재 장치 번호를 사용하여 호스트에서 실행한다.

```bash
cd ~/omx_box_project_ws
OMX_PORT_NAME=/dev/ttyACM0 OMX_VIDEO_DEVICE=/dev/video0 \
  ./scripts/start_omx_system.sh
```

이 스크립트는 컨테이너 내부 tmux에 다음을 실행한다.

```text
zenoh
bringup
MoveJ controller
USB camera
pick coordinator와 stage nodes
YOLO target bridge
RViz
coordinator status monitor
```

시스템 시작만으로 로봇 동작은 요청되지 않는다.

다른 호스트 터미널에서 YOLO를 실행한다.

```bash
cd ~/omx_box_project_ws
./scripts/start_yolo_detector.sh
```

이 명령은 다음 작업을 자동 수행한다.

- 최신 모델과 현재 calibration을 `physical_ai_server`로 복사
- `/opt/omx_yolo/venv` 전용 환경 준비
- ROS와 호환되는 NumPy 버전 사용
- `/yolo/selected_box` 발행

YOLO 터미널은 실행 중 계속 열어둔다. 중복 실행은 자동으로 거부된다.

## 8. 실행 상태 확인

```bash
cd ~/omx_box_project_ws
./scripts/status_omx_system.sh
./scripts/status_yolo_detector.sh
```

최소한 다음 항목이 `OK`여야 한다.

```text
/omx_movej_controller
/pick_coordinator
target source (/yolo_target_bridge)
/joint_states
/camera1/image_raw
/yolo/selected_box publisher
```

tmux 화면 연결:

```bash
./scripts/attach_omx_system.sh
```

tmux 조작:

```text
Ctrl-b, n : 다음 창
Ctrl-b, p : 이전 창
Ctrl-b, 숫자 : 해당 창
Ctrl-b, d : tmux에서 빠져나오기(프로세스는 계속 실행)
```

## 9. 로봇을 움직이지 않고 데이터 확인

컨테이너 터미널에서 실행한다.

```bash
./docker/container.sh enter
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/omx_box_project_ws/install/setup.bash
```

터미널 1:

```bash
ros2 topic echo /yolo/selected_box
```

터미널 2:

```bash
ros2 topic echo /camera_box_target
```

YOLO 형식:

```text
[is_defect, confidence, x_link0_m, y_link0_m, joint5_rad]
```

Bridge 출력에서 다음이 일치해야 한다.

```text
position.x = YOLO X
position.y = YOLO Y
orientation roll = YOLO joint5
frame_id = link0
```

실제 박스 중심의 X/Y와도 비교한다. 좌표 부호나 축 방향이 다르면 로봇을
움직이지 말고 calibration부터 다시 확인한다.

## 10. 실제 동작 시작

먼저 로봇 주변, 그리퍼, 케이블, 카메라 구조물과 비상 정지를 확인한다.
컨테이너 내부에서 coordinator 상태를 본다.

```bash
ros2 topic echo /pick_coordinator/status
```

Staging 시작:

```bash
ros2 service call /pick_coordinator/start std_srvs/srv/Trigger "{}"
```

정상적으로 staging이 끝나면 coordinator가 `WAIT_PICK_TARGET`에서 안정된
YOLO target을 기다린다. 다음 상태를 확인한다.

```text
pick target received x=..., y=..., joint5=...deg
```

X/Y와 joint5가 안전한지 확인한 다음 pick 접근을 시작한다.

```bash
ros2 service call /pick_coordinator/continue std_srvs/srv/Trigger "{}"
```

이후 다음 과정은 자동으로 이어진다.

```text
X/Y + joint5 approach
→ pitch pregrasp
→ gripper open
→ PTP pick descent
→ gripper close
```

`WAIT_GRASP_CONFIRM`에서는 물체가 안정적으로 잡혔는지 사람이 확인한다.
정상 파지일 때만 다시 실행한다.

```bash
ros2 service call /pick_coordinator/continue std_srvs/srv/Trigger "{}"
```

그다음 loaded lift부터 place, release, staging 복귀까지 자동으로 진행된다.

## 11. 최초 실제 환경 검증 순서

전체 cycle을 바로 반복하지 말고 다음 순서로 검증한다.

1. Calibration 독립 지점 오차 확인
2. YOLO X/Y와 bridge X/Y 비교
3. 박스 angle 부호와 joint5 회전 방향 확인
4. 작업영역 중앙에서 X/Y+joint5 approach 확인
5. pitch pregrasp의 XY/Z/pitch 확인
6. pick PTP 하강 높이 확인
7. gripper 파지와 watchdog 확인
8. loaded lift 충돌 여유 확인
9. place 상부 위치와 place 하강 높이 확인
10. 전체 cycle 5~10회 반복
11. 작업영역 가장자리에서도 dry-run과 저속 검증

박스 높이가 바뀌면 최소한 다음 설정을 다시 검증한다.

```text
src/omx_box_control/config/movej_single_ptp_descent.yaml
src/omx_box_control/config/movej_single_ptp_place_descent.yaml
```

## 12. 중단과 종료

Coordinator가 명령 사이에서 대기 중일 때 취소:

```bash
ros2 service call /pick_coordinator/cancel std_srvs/srv/Trigger "{}"
```

움직이는 중에는 먼저 비상 정지 또는 해당 controller 상태를 확인한다.
무조건 coordinator 서비스를 반복 호출하지 않는다.

통합 tmux 종료:

```bash
cd ~/omx_box_project_ws
./scripts/stop_omx_system.sh
```

YOLO는 실행 터미널에서 `Ctrl+C`로 종료한다.

프로젝트 Docker 종료:

```bash
./docker/container.sh stop
```

## 13. 대표 오류 확인

### Calibration 파일이 없다는 오류

```text
ERROR: current seven-point calibration does not exist
```

7점 calibration을 먼저 완료해 현재 환경 YAML을 생성한다.

### YOLO executable 또는 모델 오류

```bash
./scripts/status_yolo_detector.sh
docker ps --format 'table {{.Names}}\t{{.Status}}'
```

`physical_ai_server`가 실행 중인지 확인하고 YOLO를 하나만 실행한다.

### Target source가 MISSING

```bash
./scripts/status_omx_system.sh
```

`/yolo_target_bridge`와 `/yolo/selected_box` publisher가 모두 필요하다.

### staging 또는 IK 실패

```bash
ros2 topic echo /pick_coordinator/status
ros2 topic echo /joint_states
```

실제 관절 위치, X/Y, joint5, Z와 오류 메시지를 기록한다. 같은 서비스를
반복 호출하기 전에 로봇 자세와 장애물을 확인한다.

## 14. Git에 저장할 것과 저장하지 않을 것

코드와 검증된 공통 설정은 `cjh-dev`에 저장한다. 다음은 실제 환경마다 달라질
수 있으므로 검토 없이 공통값으로 덮어쓰지 않는다.

```text
integration/omx_box_system/calibration/omx_camera_homography_7point.yaml
/dev/ttyACM* 번호
/dev/video* 번호
박스 높이에 따른 pick/place Z
실측 place teaching 위치
```

변경 전후에는 항상 확인한다.

```bash
git status
git diff --check
git log -3 --oneline
```
