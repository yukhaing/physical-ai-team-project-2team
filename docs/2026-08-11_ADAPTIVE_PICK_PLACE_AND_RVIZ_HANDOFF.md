# 2026-08-11 OMX-F adaptive pick/place 작업 기록 및 인수인계

## 1. 오늘의 최종 결과

- Homography 기준 pick 작업영역을 10mm 셀로 나눠 RViz에 안전/주의/위험 색상으로 표시했다.
- 프로젝트 RViz 설정을 기본 설정으로 연결해 컨테이너에서 `rviz2`만 실행해도 불러오도록 했다.
- RViz가 `open_manipulator_description` 메시를 찾도록 `/root/ros2_ws/install/setup.bash` 환경을 기본 셸에 포함했다.
- Pick 하강 XY 허용오차를 실측에 따라 5mm에서 7mm로 조정했다.
- 낮아진 시험 박스에 맞춰 pick/place 목표 높이와 복구 범위를 조정했다.
- 그리퍼 close뿐 아니라 pick/place open에도 비종료 action watchdog을 추가했다.
- 전체 MoveJ 프로파일을 보수적인 고속 설정으로 조정했다.
- 하강은 높은 구간에서 빠르고 바닥 근처에서 느린 Z 기반 2단 프로파일로 변경했다.
- Place 하강이 46.8mm에서 불필요한 추가 step을 실행해 34.8mm까지 내려간 원인을 확인하고, 안전한 근접 위치를 완료로 인정하도록 수정했다.

## 2. 전체 실행 순서

각 명령은 source가 완료된 컨테이너의 별도 터미널에서 실행한다.

### 2.1 OMX-F bringup

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py \
  start_rviz:=false \
  port_name:=/dev/ttyACM0
```

### 2.2 Cyclo MoveJ controller

MoveL controller와 동시에 실행하면 안 된다. MoveJ controller는 정확히 하나만 실행한다.

```bash
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py \
  controller_type:=movej \
  start_interactive_marker:=false \
  config_file:=/root/omx_box_project_ws/docker/config/omx_config_physical.yaml
```

### 2.3 USB 카메라

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
  name:=camera1 \
  video_device:=/dev/video0
```

영상 토픽은 `/camera1/image_raw`이다.

### 2.4 RViz

```bash
rviz2
```

프로젝트 설정은 `/root/.rviz2/default.rviz`에 연결돼 있다.

### 2.5 Homography

```bash
ros2 launch omx_box_control camera_homography_7point_calibration.launch.py
```

### 2.6 Pick coordinator

```bash
ros2 launch omx_box_control pick_coordinator.launch.py
```

### 2.7 Staging 시작

```bash
ros2 service call /pick_coordinator/start std_srvs/srv/Trigger "{}"
```

### 2.8 Continue

`WAIT_PICK_TARGET`에서 선택한 박스로 pick을 시작하거나, `WAIT_GRASP_CONFIRM`에서 실물 grasp 확인 후 lift를 시작할 때 사용한다.

```bash
ros2 service call /pick_coordinator/continue std_srvs/srv/Trigger "{}"
```

상태 확인:

```bash
ros2 topic echo /pick_coordinator/status
```

## 3. RViz 위험영역

Homography 노드는 `/camera_workspace_markers`에 `visualization_msgs/msg/MarkerArray`를 발행한다.

- 기준 셀 크기: 10mm
- 안전: 유효 반경 280mm 이하
- 주의: 280~285mm
- 위험: 285mm 초과
- RViz display 이름: `Pick Workspace Risk Grid`
- QoS: Reliable + Transient Local

실패했던 `(x=0.2724, y=0.0518)`은 유효 반경 약 287mm로 위험영역에 해당했다.

RViz 기본 화면은 5cm grid, 가까운 orbit view로 저장했다. RobotModel STL을 표시하려면 `/root/ros2_ws/install/setup.bash`가 source돼 있어야 한다.

관련 파일:

```text
src/omx_box_control/scripts/camera_homography_7point_calibration_node.py
src/omx_box_control/config/homography_7point_calibration.yaml
src/omx_box_control/rviz/omx_box_project.rviz
docker/Dockerfile
```

## 4. 최종 높이 설정

### 4.1 Pick

```text
target_z:       32mm
target_z_min:   25mm
target_z_max:   38mm
min_path_z:     25mm
min_final_z:    25mm
max_xy_error:    7mm
```

Coordinator grasp 복구 범위도 25~38mm로 동기화했다.

### 4.2 Place

```text
target_z:       42mm
target_z_min:   40mm
target_z_max:   50mm
min_path_z:     35mm
min_final_z:    35mm
```

Place는 pick 목표보다 10mm 높다. 물리 step이 12~17mm씩 발생하므로 46~50mm의 안전한 근접 자세를 완료로 인정해 추가 overshoot를 방지한다.

## 5. 고속 MoveJ 설정

Controller 설정:

```text
smooth_max_velocity:     0.25 rad/s
smooth_max_acceleration: 0.40 rad/s^2
```

일반 단계 설정:

```text
staging:               move 5s / validation 8s
XY approach:           move 7s / validation 11s
pitch pregrasp:        move 5s / validation 8s
loaded lift:           move 8s / validation 12s
place high XY:         move 6s / validation 10s
place Z/pitch align:   move 5s / validation 8s
```

Timeout, settle gate, 실제 TCP/관절 안전검사는 유지했다.

## 6. Z 기반 2단 하강 프로파일

Pick pitch pregrasp 목표를 130mm에서 100mm로 낮췄다. 실제 추종 오차를 고려해 pitch 및 gripper-open 최소 Z gate는 80mm/75mm로 설정했다.

하강 프로파일은 현재 Z와 완료 band 상단의 차이로 자동 선택한다.

```text
fast 조건: current Z > target_z_max + 25mm
fast:      move 3.5s / validation 4.5s
slow:      move 5.0s / validation 6.0s
settle:    0.30s
```

- Pick fast 경계: 약 63mm
- Place fast 경계: 약 75mm
- 바닥 근처에서는 자동으로 slow profile을 사용한다.
- 로그에 `profile=fast|slow`, `duration`, `validation`을 기록한다.
- `planned_step`은 0.05mm를 유지했다. 작은 계획 step에도 물리 하강량이 크게 나타나므로 키우지 않는다.
- 예상 pick 하강시간은 약 25~35초다. 실물 검증 전 더 공격적으로 줄이지 않는다.

관련 파일:

```text
src/omx_box_control/scripts/movej_closed_loop_descent_node.py
src/omx_box_control/config/movej_closed_loop_descent.yaml
src/omx_box_control/config/movej_place_descent.yaml
src/omx_box_control/config/movej_pitch_pregrasp.yaml
src/omx_box_control/config/gripper_open.yaml
```

## 7. Gripper open/close watchdog

ID16은 실제 물리 위치에 도달한 뒤에도 작은 nonzero velocity를 계속 보고해 action이 종료되지 않을 수 있다.

실측 사례:

```text
position: 0.9894rad
minimum open: 0.8rad
reported velocity: 약 0.024rad/s
state: WAIT_PICK_GRIPPER_OPEN에서 action 결과 대기
```

기존 close watchdog은 안정적인 grasp contact를 감지했다. 오늘 pick open 노드와 coordinator의 place open에도 watchdog을 추가했다.

```text
minimum_open_position:             0.80rad
open target/tolerance:             1.0rad ± 0.05rad
open_watchdog_timeout:             2.0s
open_watchdog_stable_time:         0.50s
open_watchdog_position_epsilon:    0.003rad
```

조건이 만족되면 non-terminating goal을 취소하고 정상 open으로 인정한다.

예상 상태 로그:

```text
open watchdog accepted stable opening
accepted watchdog open
COMPLETED: gripper fully open
```

관련 파일:

```text
src/omx_box_control/scripts/gripper_open_node.py
src/omx_box_control/scripts/pick_coordinator_node.py
src/omx_box_control/config/gripper_open.yaml
src/omx_box_control/config/pick_coordinator.yaml
```

## 8. 실측 결과와 문제 원인

### 8.1 Pick XY safe stop

목표 `(245.5mm, 69.1mm)`에 대해 낮은 Z와 약 90도 pitch에서 실제 XY 오차가 약 6~7mm까지 증가했다. 기존 5mm gate가 정지시켰으며, 실물 위치 확인 후 7mm로 제한을 완화했다. 이후 동일 흐름의 전체 pick/place 사이클이 완료됐다.

### 8.2 Place overshoot

```text
start:       78.8mm
step 1:      63.7mm  (drop 15.1mm)
step 2:      46.8mm  (drop 16.8mm)
old band:    40~45mm
step 3:      34.8mm  (safe stop below 35mm)
```

46.8mm가 기존 상한보다 1.8mm 높다는 이유로 추가 step을 실행한 것이 원인이었다. Place 상한을 50mm로 넓혀 46.8mm에서 완료하도록 수정했다.

## 9. 비정상 정지 후 staging 복귀

Pending gripper action이 있으면 `/pick_coordinator/start`를 바로 호출하지 않는다.

1. Coordinator launch 터미널에서 `Ctrl+C`.
2. 남은 gripper action을 취소한다.

```bash
ros2 service call \
  /gripper_controller/gripper_cmd/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
```

3. Coordinator를 다시 실행한다.
4. 물체를 잡고 있지 않고 경로가 비어 있음을 확인한다.
5. `/pick_coordinator/start`를 호출한다.

물체를 잡고 낮은 Z에서 실패했다면 staging으로 바로 이동하지 않는다. Grasp 복구 후 loaded lift를 먼저 실행해야 한다.

```bash
ros2 service call /pick_coordinator/recover_grasp std_srvs/srv/Trigger "{}"
ros2 service call /pick_coordinator/continue std_srvs/srv/Trigger "{}"
```

## 10. 다음 실물 시험 체크리스트

1. MoveJ controller가 정확히 하나인지 확인한다.
2. MoveL controller가 없는지 확인한다.
3. 그리퍼 LED와 open/close feedback을 확인한다.
4. Coordinator를 재시작해 최신 watchdog과 adaptive descent 코드를 로드한다.
5. Staging 후 안전영역 안의 가까운 target을 선택한다.
6. Pitch pregrasp 실제 Z가 80mm 이상인지 확인한다.
7. Pick descent 로그의 fast/slow 전환과 실제 step drop을 기록한다.
8. Pick 최종 Z가 25~38mm인지 확인한다.
9. Place가 40~50mm에서 완료되고 추가 step을 내보내지 않는지 확인한다.
10. Open watchdog이 작동할 경우 실제 위치가 0.95~1.05rad 안인지 확인한다.

## 11. 저장 및 검증 상태

- `omx_box_control` 패키지 `colcon build --symlink-install` 성공
- 변경된 Python 파일 `py_compile` 성공
- 변경된 YAML 파일 파싱 성공
- `git diff --check` 성공
- 실물 adaptive fast/slow 하강은 다음 재시작 후 검증 필요

현재 실행 프로세스에는 이전 Python 코드가 남아 있을 수 있다. 최신 코드를 적용하려면 coordinator launch를 반드시 재시작한다.
