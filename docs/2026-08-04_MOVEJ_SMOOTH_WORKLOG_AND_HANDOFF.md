# OMX-F MoveJ S-curve 실물 검증 작업 기록 및 인수인계

- 작업일: 2026-08-04
- 작업 경로: `/home/itec/omx_box_project_ws`
- 목적: 외부에서 작은 MoveJ 목표를 반복 발행하지 않고, MoveJ controller 내부에서 속도·가속도·저크가 연속적인 단일 S-curve 궤적을 생성한다.
- 최종 결과: staging은 안정적으로 동작했지만 target XY approach는 joint3 추종 오차 때문에 아직 완료되지 않았다.

## 1. 오늘의 결론

기존 smooth relay처럼 작은 목표를 연속 발행하는 방식은 사용하지 않기로 했다. relay 정지 후에도 MoveJ controller 내부의 마지막 목표가 남아 지연 이동할 수 있기 때문이다.

대신 기존 MoveJ 입력 토픽에 최종 목표를 한 번만 보내고, MoveJ controller 내부에서 quintic S-curve를 생성하도록 구현했다.

```text
blend(s) = 10s^3 - 15s^4 + 6s^5
q_ref = q_start + blend(s) * (q_goal - q_start)
```

S-curve 시간은 각 관절 이동량에 대해 속도, 가속도, 저크 제한을 모두 계산하고 가장 긴 시간을 사용한다. 모든 관절은 동일한 시간축으로 움직인다.

구현과 빌드는 성공했다. 실물 staging 복귀도 반복 성공했다. 그러나 target approach에서는 하중을 받는 joint3가 목표를 충분히 따라가지 못해 최대 관절 오차와 TCP XY 오차가 허용값을 초과했다.

## 2. 구현한 내용

### 선택형 S-curve

`smooth_profile_enabled`가 `true`이면 S-curve를 사용한다. `false`이면 기존 직접 MoveJ 경로를 그대로 사용할 수 있다.

현재 물리 설정은 다음과 같다.

```yaml
smooth_profile_enabled: true
smooth_max_velocity: 0.20
smooth_max_acceleration: 0.30
smooth_max_jerk: 1.0
smooth_min_duration: 1.0
smooth_settle_tolerance: 0.04
smooth_settle_timeout: 3.0
```

### 실제 피드백 시작점

새 S-curve 명령을 받을 때 이전 목표가 아니라 실제 `/joint_states`를 시작점으로 사용한다.

### 종료 시 feedback hold

S-curve가 끝나면 하위 controller에 마지막 목표를 계속 남기지 않고 실제 관절 피드백을 hold 목표로 보낸다. 이는 relay 시험에서 확인된 지연 이동을 방지하기 위한 안전 조치다.

### bounded settle

S-curve 종료 즉시 hold하면 joint3가 따라잡기 전에 보정이 끊기는 문제가 있어 최대 3초의 settle 구간을 추가했다.

- 실제 최대 관절 오차가 `0.04 rad` 이하가 되면 즉시 feedback hold
- 3초 동안 허용 오차에 들어오지 않으면 강제로 feedback hold하고 경고 출력
- 무제한으로 목표를 계속 추종하지 않음

### 애플리케이션 허용값 정렬

`movej_xy_approach.yaml`의 `joint_tolerance`를 `0.03 rad`에서 `0.04 rad`로 변경했다. staging에서 사용 중인 값 및 측정된 joint3 deadband와 맞춘 것이다.

## 3. 추가·수정 파일

오늘 직접 추가하거나 수정한 핵심 파일은 다음과 같다.

```text
docker/Dockerfile
docker/config/omx_config_physical.yaml
docker/patches/cyclo_movej_s_curve.patch
docker/patches/cyclo_movej_s_curve_feedback_hold.patch
docker/patches/cyclo_movej_s_curve_bounded_settle.patch
src/omx_box_control/config/movej_xy_approach.yaml
```

Docker 이미지를 다시 만들 때 세 controller 패치가 순서대로 자동 적용되도록 `docker/Dockerfile`을 수정했다.

컨테이너의 `/root/ros2_ws`에서도 패치를 적용하고 다음 빌드를 완료했다.

```bash
colcon build --symlink-install --packages-select cyclo_motion_controller_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

colcon build --symlink-install --packages-select omx_box_control
```

두 빌드 모두 성공했다. cyclo 빌드에는 기존 upstream의 unused-function 경고만 있었다.

## 4. 실물 검증 결과

### 작은 joint1 시험

- 요청 이동량: `+0.020 rad`
- 자동 계산된 S-curve 시간: `1.063 s`
- 실제 이동량: `0.012272 rad`
- 요청보다 작게 움직인 결과는 이전에 측정한 약 `0.014 rad` 기계적 deadband와 일치한다.

### staging 복귀

여러 번 실물 복귀에 성공했다.

대표 결과:

- S-curve 시간: `7.023 s`, 최대 완료 오차 `1.76 deg`
- S-curve 시간: `6.000 s`, 최대 완료 오차 `1.59 deg`
- bounded settle 적용 후 프로파일 종료 실제 최대 오차: `0.0262 rad`
- staging 애플리케이션 완료 판정 성공

### target XY approach 첫 시험

클릭 target:

```text
X=0.2096 m
Y=0.0979 m
```

dry-run 계획:

- 계획 XY 오차: `0.00 mm`
- 계획 최종 Z: 약 `168.2~168.9 mm`
- 계획 경로 최저 Z: 약 `134.0~135.3 mm`
- `min_path_z=120 mm` 통과
- 계획 pitch: 약 `45.6~46.2 deg`

pitch가 70~80도가 아닌 것은 이 노드가 XY approach 단계이기 때문이다. 이후 pitch-pregrasp 단계에서 아래 방향으로 조정하도록 구성돼 있다.

첫 실제 결과:

- S-curve 시간: `6 s`
- 최대 관절 오차: `2.10 deg`
- joint3 오차: `0.03672 rad`
- 실제 XY 오차: 약 `4.77 mm`
- 애플리케이션의 당시 `0.03 rad` 관절 허용값 때문에 timeout

### `joint_tolerance=0.04 rad` 재시험

허용값을 `0.04 rad`로 바꿔 다시 시험했지만 timeout이 재현됐다.

- 최대 관절 오차: `3.09 deg`
- joint3 오차: 약 `0.0539 rad`
- 실제 XY 오차: 약 `6.19 mm`
- XY 허용값 `5 mm`도 초과

단순히 관절 허용값을 더 넓히면 Cartesian 정확도까지 나빠지므로 적절한 해결책이 아니다.

### bounded settle 3초 재시험

S-curve 종료 후 최대 3초 동안 실제 관절이 따라오도록 수정하고 다시 시험했다.

- S-curve 종료 시 settle 시작
- 3초 후 실제 최대 관절 오차: `0.0447 rad`
- `0.04 rad`에 들어오지 못해 settle timeout
- feedback hold 후 하중으로 오차가 다시 증가
- 애플리케이션 최종 최대 관절 오차: `3.09 deg`
- 최종 실제 TCP: `X=0.20464, Y=0.09434, Z=0.15419 m`
- target 대비 실제 XY 오차: 약 `6.11 mm`

따라서 bounded settle만으로도 완전히 해결되지 않았다.

## 5. 현재 판단한 원인

핵심 원인은 완료 허용값이 아니라 이동 중 joint3 추종 지연이다.

- joint3에는 하중 방향 deadband와 hysteresis가 있다.
- 이전 직접 시험에서 최소 deadband는 약 `0.014 rad (0.79 deg)`였다.
- 현재 approach는 애플리케이션에서 `move_duration=6.0 s`를 요청한다.
- controller의 속도·가속도·저크 제한으로 계산한 시간보다 요청 시간 6초가 길면 6초를 사용한다.
- 하지만 6초 동안에도 실제 joint3가 S-curve 참조를 충분히 따라가지 못한다.
- 프로파일 종료 후 3초를 더 기다려도 목표 오차 안으로 안정적으로 들어오지 않는다.

다음 시험은 허용 오차 확대가 아니라 approach 이동시간 자체를 늘려 이동 중 추종 오차를 줄여야 한다.

## 6. 다음 구현 및 시험 우선순위

1. `movej_xy_approach.yaml`의 `move_duration`과 `minimum_completion_time`을 `6.0 s`에서 우선 `10.0 s`로 변경한다.
2. timeout은 S-curve 10초와 bounded settle 3초를 포함하도록 최소 `18~20 s`로 변경한다.
3. 현재 자세에서 바로 재명령하지 말고 staging으로 안전 복귀한다.
4. staging 노드를 완전히 종료해 MoveJ publisher가 0개인지 확인한다.
5. approach 노드를 dry-run으로 실행하고 같은 target 또는 새 클릭 target의 경로 최저 Z와 XY 계획 오차를 확인한다.
6. 실제 10초 S-curve approach를 한 번 실행한다.
7. 다음 수치를 기록한다.
   - 프로파일 종료 시 실제 관절별 오차
   - settle 진입 시점과 완료/timeout 시점
   - 최종 joint3 오차
   - 최종 TCP XY 오차
   - 최종 Z
8. 10초에서도 joint3 오차가 크면 `12~14 s`를 시험하기 전에, 하위 Dynamixel profile velocity/acceleration 및 MoveJ QP 출력과 실제 피드백의 시계열을 기록한다.
9. 관절 허용값을 `0.04 rad`보다 더 넓히는 방식은 사용하지 않는다. Cartesian XY 5 mm 제한을 우선한다.
10. XY approach가 안정적으로 완료된 후에만 pitch-pregrasp 70~80도 단계로 진행한다.

## 7. 현재 로봇과 ROS 상태

2026-08-04 작업 종료 직전 확인값이다.

### 실행 노드

- `/omx_movej_controller` 실행 중
- MoveL controller 없음
- MoveJ 애플리케이션 명령 노드 없음
- `/omx_movej_controller/movej` publisher 수: `0`

### 그리퍼

- `gripper_joint_1 = -0.029145634970034084 rad`
- 열린 상태

### 실제 관절값

```text
joint1 =  0.41877675509257317
joint2 = -0.36201946594121814
joint3 =  0.19328157927338374
joint4 =  1.0538448012772283
joint5 = -0.001533980788092748
```

관절 속도는 모두 0으로 확인했다. `/joint_states`의 joint4 velocity 필드에 간헐적으로 `0.0239691227`이 표시되지만 위치는 정지 상태였다.

### TCP 자세

```text
X = 0.20467753381716514 m
Y = 0.09435933777542949 m
Z = 0.15423905855514783 m
```

로봇은 약 `Z=154.24 mm`의 안전 높이에서 정지해 있다.

### 종료 시 주의

현재 그리퍼는 열려 있고 박스를 잡고 있지 않은 상태로 판단된다. 그래도 실제 로봇 주변을 육안 확인한 후 controller, 컨테이너 또는 하드웨어 전원을 종료해야 한다.

## 8. 재개 시 안전 확인 명령

호스트에서 컨테이너 상태 확인:

```bash
cd /home/itec/omx_box_project_ws
docker ps
git status --short
```

ROS 상태 확인:

```bash
docker exec -it omx_box_project bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 node list | sort
ros2 topic info /omx_movej_controller/movej --verbose
ros2 topic echo /joint_states --once
ros2 topic echo /omx_movej_controller/current_pose --once
ros2 param get /omx_movej_controller smooth_profile_enabled
ros2 param get /omx_movej_controller smooth_settle_tolerance
ros2 param get /omx_movej_controller smooth_settle_timeout
```

확인 조건:

- MoveJ와 MoveL controller를 동시에 실행하지 않는다.
- MoveJ controller 프로세스는 정확히 1개여야 한다.
- 이동 전 MoveJ 명령 publisher 종류와 개수를 확인한다.
- staging, approach 등 명령 노드는 한 번에 하나만 실행한다.
- 실제 로봇 자세, 그리퍼 상태, 주변 장애물을 육안 확인한다.
- target 클릭 토픽은 일회성이므로 approach 노드를 먼저 실행한 후 클릭한다.
- 로봇 명령 전 반드시 dry-run을 수행한다.

## 9. Git 상태

오늘 변경을 포함해 작업트리는 아직 commit되지 않았다. 기존 미커밋 파일도 다수 있으므로 임의로 reset하거나 삭제하면 안 된다.

핵심 신규 변경은 다음과 같다.

```text
M  docker/Dockerfile
M  docker/config/omx_config_physical.yaml
M  src/omx_box_control/config/movej_xy_approach.yaml
?? docker/patches/
```

그 밖의 rule-based pick, coordinator, staging, lift, descent 관련 기존 미커밋 파일도 그대로 남아 있다. 전체 목록은 `git status --short`로 다시 확인한다.

## 10. 다음 GPT 세션에 넣을 프롬프트

아래 내용을 새 GPT 세션의 첫 메시지로 그대로 전달한다.

```text
/home/itec/omx_box_project_ws에서 OMX-F 박스 픽 프로젝트를 이어서 작업해줘.

먼저 다음 문서를 전부 읽어줘.

- docs/CODEX_HANDOFF.md
- docs/2026-08-03_RULE_BASED_PICK_WORKLOG.md
- docs/OMX_BOX_PROJECT_PROGRESS.md
- docs/2026-08-04_MOVEJ_SMOOTH_WORKLOG_AND_HANDOFF.md

그 다음 파일을 변경하거나 로봇을 움직이기 전에 반드시 다음을 확인해.

- git status --short
- 실행 중인 ROS 노드
- MoveJ/MoveL controller 종류와 프로세스 개수
- /omx_movej_controller/movej publisher 종류와 개수
- 실제 /joint_states
- 실제 TCP pose
- 그리퍼 상태

실제 로봇에서는 MoveJ와 MoveL controller를 동시에 실행하면 안 되고, MoveJ 명령 publisher도 중복 실행하면 안 된다.

2026-08-04에 기존 외부 waypoint relay 대신 MoveJ controller 내부 quintic S-curve를 구현했다. 기존 MoveJ 방식은 smooth_profile_enabled=false로 보존되어 있다. 현재 물리 설정은 velocity 0.20 rad/s, acceleration 0.30 rad/s^2, jerk 1.0 rad/s^3, settle tolerance 0.04 rad, settle timeout 3초다. controller 패치는 docker/patches에 있고 Dockerfile에서 자동 적용된다.

staging은 6초 S-curve로 성공했고 최대 실제 오차는 약 1.59도였다. 하지만 target X=0.2096, Y=0.0979의 XY approach는 6초 이동과 3초 bounded settle 후에도 joint3 오차가 남아 timeout됐다. 최종 최대 관절 오차는 3.09도, 실제 XY 오차는 약 6.11mm로 5mm 제한을 초과했다. 단순 tolerance 확대는 하지 말아야 한다.

다음 목표는 movej_xy_approach의 move_duration과 minimum_completion_time을 6초에서 10초로 늘리고 timeout을 18~20초로 조정한 다음 실물 검증하는 것이다. 먼저 staging으로 복귀하고 staging publisher를 완전히 종료한 뒤 approach dry-run을 수행해라. 그 다음 실제 10초 S-curve 접근을 한 번 실행하면서 프로파일 종료 시 관절별 오차, settle 결과, 최종 joint3 오차, TCP XY 오차와 Z를 기록해라. 단계별 문제가 생기면 즉시 추가 이동을 중단하고 원인을 알려줘.

현재 작업 종료 시 로봇은 약 X=0.20468, Y=0.09436, Z=0.15424m에 정지해 있고 그리퍼는 열려 있다. /omx_movej_controller만 실행 중이며 MoveJ 입력 publisher는 0개였다. 이 값은 재개 시 반드시 다시 측정해서 확인해.

기존 미커밋 변경이 많으므로 git reset, checkout, clean 또는 파일 삭제를 하지 말고 사용자 변경을 보존해.
```
