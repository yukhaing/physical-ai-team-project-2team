# 2026-08-05 OMX-F MoveJ Pick 검증 및 인수인계

## 1. 오늘의 결론

MoveJ controller 내부 quintic S-curve와 joint3 terminal trim을 사용해 다음 물리 동작을 실증했다.

1. staging 복귀
2. 카메라 목표 XY 접근
3. pitch pregrasp
4. 고정 높이 범위를 목표로 하는 단일 coarse 하강

최종 하강은 실제 TCP Z `91.48 mm`, 목표 XY 오차 `2.48 mm`, pitch `81.83 deg`로 완료됐다. 설정한 grasp-ready Z 범위 `80–95 mm`, XY 제한 `5 mm`, pitch 제한 `90 deg`를 모두 만족했다.

정확한 mm 단위 하강을 반복 보정하는 방식은 사용하지 않는다. joint3의 하중 방향 히스테리시스를 고려해 안전한 Z 범위 안에 한 번 진입하면 완료하는 방식으로 정리했다.

## 2. 절대 안전 규칙

- 실제 로봇에서 MoveJ controller와 MoveL controller를 동시에 실행하지 않는다.
- `/omx_movej_controller/movej` publisher는 항상 정확히 하나만 허용한다.
- 각 파일 변경 및 실제 이동 전 다음을 확인한다.
  - `git status --short`
  - 실행 중인 ROS 노드
  - MoveJ/MoveL controller 종류와 프로세스 개수
  - `/omx_movej_controller/movej` publisher 종류와 개수
  - 실제 `/joint_states`
  - 실제 joint-state FK TCP pose
  - 그리퍼 상태
- `/omx_movej_controller/current_pose`는 controller 명령 상태 기반일 수 있으므로 정밀 완료 판정에는 실제 `/joint_states` FK를 사용한다.
- 단계별 실패 시 자동 재시도나 추가 이동을 하지 않는다.
- 기존 미커밋 변경을 보존한다. `git reset`, `git checkout`, `git clean`, 파일 삭제를 하지 않는다.

## 3. MoveJ controller 개선

### Quintic S-curve

기존 외부 waypoint relay 대신 MoveJ controller 내부에서 quintic S-curve를 생성한다. 기존 방식은 `smooth_profile_enabled=false`로 보존되어 있다.

물리 설정:

- velocity: `0.20 rad/s`
- acceleration: `0.30 rad/s^2`
- jerk: `1.0 rad/s^3`
- settle tolerance: `0.04 rad`
- settle timeout: `3.0 s`

### Joint3 terminal trim

하중 방향 히스테리시스로 남는 joint3 오차를 제한된 속도와 offset 안에서 보정하는 terminal trim을 추가했다.

관련 파일:

- `docker/patches/cyclo_movej_s_curve_terminal_trim.patch`
- `docker/config/omx_config_physical.yaml`
- `docker/Dockerfile`

현재 주요 값:

```yaml
smooth_terminal_trim_enabled: true
smooth_terminal_trim_stall_velocity: 0.002
smooth_terminal_trim_stall_time: 0.5
smooth_terminal_trim_rate: 0.02
smooth_terminal_trim_max_offset: 0.06
smooth_terminal_trim_timeout: 5.0
smooth_terminal_trim_capture_tolerance: 0.01
```

## 4. 단계별 실증 결과

### 4.1 Staging

- 6초 S-curve 및 terminal trim 성공
- 최종 joint3 오차: 약 `0.02092 rad` (`1.20 deg`)

### 4.2 XY approach

목표:

- X: `0.2096 m`
- Y: `0.0979 m`

설정:

- move duration: `10 s`
- minimum completion time: `16 s`
- timeout: `20 s`

결과:

- 실제 joint-state FK XY 오차: `4.17 mm`
- 실제 FK Z: `0.16004 m`
- 최종 joint3 오차: 약 `0.01936 rad` (`1.11 deg`)
- 최대 관절 오차: joint4 약 `1.90 deg`
- 판정: 성공

### 4.3 Pitch pregrasp

계획 Z는 `150 mm`로 유지하고 실제 완료 허용 최저 Z를 `130 mm`로 분리했다.

결과:

- 실제 pitch: `75.94 deg`
- 실제 FK Z: `134.3 mm`
- 실제 XY 오차: `2.64 mm`
- 판정: 성공

### 4.4 첫 coarse 하강 진단

계획상 9 mm 하강을 단 한 번 실행했다.

결과:

- 시작 실제 Z: `113.57 mm`
- 최종 실제 Z: `100.43 mm`
- 실제 하강량: `13.13 mm`
- 실제 XY 오차: `2.49 mm`
- 실제 pitch: `82.00 deg`
- 당시 pitch 제한: `81 deg`

단일 이동 자체는 끝났지만 실제 pitch가 제한을 초과해 후속 이동을 안전 차단했다. 자동 추가 명령은 없었다.

bag:

```text
/tmp/omx_descent_coarse_single_20260805_0815
```

### 4.5 Pitch 제한 90도 적용 후 최종 하강

설정 파일의 시작/최종 최대 pitch를 모두 90도로 변경했다.

```yaml
start_max_pitch: 1.570796327
max_pitch: 1.570796327
```

관련 파일:

- `src/omx_box_control/config/movej_closed_loop_descent.yaml`

dry-run 결과:

- 시작 Z: `100.4 mm`
- 계획 종료 Z: `95.0 mm`
- 경로 최저 Z: `95.0 mm`
- 계획 XY 오차: `1.01 mm`
- 계획 pitch: `80.68 deg`
- 계획 관절 변화: `[0.504, -0.563, 2.839, -3.595, 0.0] deg`

실물 결과:

- 시작 실제 Z: `100.43 mm`
- 최종 실제 Z: `91.48 mm`
- 실제 하강량: `8.96 mm`
- 목표 Z 범위: `80–95 mm`
- 실제 XY 오차: `2.48 mm`
- 실제 pitch: `81.83 deg`
- pitch 허용 최대치: `90 deg`
- 자동 후속 명령: 없음
- 판정: 성공

bag:

```text
/tmp/omx_descent_pitch90_single_20260805_0828
```

## 5. Closed-loop descent 구현 요약

관련 파일:

- `src/omx_box_control/scripts/movej_closed_loop_descent_node.py`
- `src/omx_box_control/config/movej_closed_loop_descent.yaml`
- `src/omx_box_control/launch/movej_closed_loop_descent.launch.py`

핵심 동작:

- 실제 `/joint_states` FK로 Z, XY, pitch를 판정한다.
- `single_step_mode=true`에서는 명령을 정확히 한 번만 보낸다.
- joint 목표 오차가 남더라도 최소 완료 시간 이후 실제 Cartesian 안전 결과를 확인한다.
- 목표 Z는 정밀 점이 아니라 `80–95 mm` 범위다.
- 실제 Z가 `80 mm` 아래로 내려가거나 XY/pitch/step 제한을 위반하면 후속 명령을 차단한다.
- 히스테리시스 때문에 exact-Z 반복 보정은 하지 않는다.

최종 실증 시 실행 override:

```text
dry_run=false
single_step_mode=true
planned_step=0.009
max_actual_step=0.04
max_xy_error=0.005
xy_weight=1000.0
joint_delta_weight=1.0
```

설정 파일 기본값은 안전을 위해 `dry_run: true`로 남아 있다. 실물 실행 시 반드시 dry-run 후 명시적으로 `dry_run:=false`를 사용해야 한다.

## 6. 기타 변경

컨테이너 시작 시 GUI 권한을 자동 설정하도록 `docker/container.sh`를 변경했다.

- start: `xhost +si:localuser:root`
- stop: `xhost -si:localuser:root`

따라서 정상적인 `./docker/container.sh start` 사용 시 매 세션마다 xhost 명령을 직접 입력할 필요가 없다.

## 7. 작업 종료 상태

마지막 정상 수신 관절값:

```text
gripper_joint_1 = -0.0383495197  # open
joint1 =  0.4157087935
joint2 =  0.1840776945
joint3 = -0.2408349837
joint4 =  1.4848934027
joint5 =  0.0015339808
```

이 관절값으로 계산한 실제 joint-state FK:

```text
X = 0.21057891 m
Y = 0.09561796 m
Z = 0.09147707 m
pitch = 81.8262 deg
```

마지막 확인 당시:

- 그리퍼: 열림
- MoveJ command publisher: `0`
- MoveJ controller 프로세스: `1`
- MoveL controller 프로세스: `0`

작업 종료 직전에는 Zenoh router가 내려가 `/joint_states`와 TCP를 새로 수신하지 못했다. 위 값은 router가 정상일 때 마지막으로 측정한 값이다. 다음 재개 시 현재 상태라고 가정하지 말고 반드시 다시 측정한다.

## 8. 다음 작업 재개 절차

1. `git status --short`로 기존 변경을 확인한다.
2. 컨테이너와 Zenoh router 상태를 확인한다.
3. ROS bringup이 실제로 살아 있는지 확인한다.
4. 남아 있는 MoveJ controller 프로세스를 확인해 controller를 중복 실행하지 않는다.
5. MoveL controller가 0개인지 확인한다.
6. `/omx_movej_controller/movej` publisher가 0개인지 확인한다.
7. 실제 `/joint_states`, joint-state FK TCP, 그리퍼 상태를 다시 측정한다.
8. 현재 로봇이 낮은 자세에 있으므로 staging 또는 lift 전에 주변 공간을 육안 확인한다.
9. 다음 단계는 검증된 개별 노드들을 하나의 전체 pick sequence로 연결하는 것이다.
10. 자동 시퀀스에서도 각 단계 종료 후 실제 FK 기반 gate를 통과해야만 다음 명령을 허용한다.

## 9. 남은 작업

- staging → XY approach → pitch pregrasp → coarse descent → gripper close → lift를 하나의 coordinator에 통합
- 각 단계의 단일 publisher 수명 관리 및 중복 방지
- gripper close 실물 검증
- grasp 이후 lift 실물 검증
- 실패 시 즉시 정지하고 재시도를 사용자 승인 뒤에만 수행하는 상태 머신 구현
- bag 분석 결과와 최종 파라미터를 장기 보관할 위치 결정 (`/tmp` bag은 컨테이너 재생성 시 사라질 수 있음)

## 10. 관련 문서

- `docs/CODEX_HANDOFF.md`
- `docs/2026-08-03_RULE_BASED_PICK_WORKLOG.md`
- `docs/2026-08-04_MOVEJ_SMOOTH_WORKLOG_AND_HANDOFF.md`
- `docs/2026-08-05_MOVEJ_TERMINAL_TRIM_WORKLOG.md`
- `docs/OMX_BOX_PROJECT_PROGRESS.md`
