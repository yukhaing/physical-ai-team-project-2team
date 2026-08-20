# Codex 작업 인수인계

## 다음 세션 시작 프롬프트

아래 내용을 새 Codex 세션에 그대로 입력한다.

```text
/home/itec/omx_box_project_ws에서 OMX-F 박스 픽 프로젝트를 이어서 작업해줘.
먼저 docs/CODEX_HANDOFF.md와 docs/2026-08-03_RULE_BASED_PICK_WORKLOG.md,
docs/OMX_BOX_PROJECT_PROGRESS.md를 읽고 현재 git status와 실행 중인 ROS 노드를 확인해.
실제 로봇은 MoveJ/MoveL controller를 동시에 실행하면 안 된다.
오늘 검증한 흐름은 MoveJ staging → 수동 MoveL 전환 → 집기 → 수동 MoveJ 전환
→ MoveJ lift이며, 다음 목표는 이 controller handoff를 안전하게 coordinator로 만드는 것이다.
파일 변경 전에 현재 로봇 자세, 그리퍼 상태, controller 종류와 중복 publisher를 먼저 확인해.
```

## 저장소 위치

```text
호스트: /home/itec/omx_box_project_ws
컨테이너: /root/omx_box_project_ws
컨테이너 이름: omx_box_project
ROS 2: Jazzy
RMW: rmw_zenoh_cpp
ROS_DOMAIN_ID: 30
```

## 반드시 먼저 읽을 파일

1. `docs/2026-08-03_RULE_BASED_PICK_WORKLOG.md`
2. `docs/OMX_BOX_PROJECT_PROGRESS.md`
3. `src/omx_box_control/scripts/rule_based_pick_node.py`
4. `src/omx_box_control/scripts/movej_staging_node.py`
5. `src/omx_box_control/config/rule_based_pick.yaml`
6. `src/omx_box_control/config/movej_staging.yaml`
7. `src/omx_box_control/config/movej_lift.yaml`

## 세션 시작 확인 명령

```bash
cd /home/itec/omx_box_project_ws
git status --short

docker exec omx_box_project bash -lc '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/omx_box_project_ws/install/setup.bash 2>/dev/null || true
ros2 node list | sort
ros2 topic info /omx_movel_controller/movel --verbose 2>/dev/null || true
ros2 topic info /omx_movej_controller/movej --verbose 2>/dev/null || true
timeout 5 ros2 topic echo /joint_states --once
'
```

## 안전 규칙

- MoveJ와 MoveL controller를 동시에 실행하지 않는다.
- `/rule_based_pick` 중복 노드를 허용하지 않는다.
- 실제 명령 전 `/arm_controller`, `/joint_states`, 현재 controller와 그리퍼 상태를 확인한다.
- MoveJ/MoveL 전환 전에 기존 controller launch와 자식 노드가 모두 종료됐는지 확인한다.
- 그리퍼가 박스를 잡은 상태에서 하드웨어 전원이나 컨테이너를 갑자기 종료하지 않는다.
- Cartesian workspace 검사는 충돌 안전을 보장하지 않는다.

## 다음 구현 우선순위

첫 번째 목표는 안전한 상위 coordinator다.

```text
READY_MOVEJ_STAGING
→ WAIT_STAGING_COMPLETE
→ REQUEST_SWITCH_TO_MOVEL
→ WAIT_MOVEL_READY
→ PICK_APPROACH_AND_GRASP
→ REQUEST_SWITCH_TO_MOVEJ
→ WAIT_MOVEJ_READY
→ LIFT
→ COMPLETE
```

Cyclo controller는 lifecycle node가 아니므로 controller 전환 방식을 먼저 설계해야 한다. 두 controller 동시 실행은 금지한다. 초기 구현은 명시적인 사용자 확인을 받는 수동 handoff coordinator가 가장 안전하다.

## Git 상태 주의

오늘 추가한 rule-based pick, MoveJ staging, MoveJ lift 파일은 아직 untracked이며 CMake/package 변경도 commit되지 않았다. 사용자의 기존 변경이므로 삭제하거나 reset하지 않는다.


## 2026-08-10 통합 Pick/Place 실물 검증

- MoveJ coordinator 단일 발행 구조로 staging, XY 접근, pitch, pick 하강, grasp, lift, place 회전, place 하강까지 실물 동작했다.
- `joint5`는 위치 명령을 추종하지 않아 staging에서 현재값을 보존한다.
- pitch pregrasp의 실측 잔차를 반영해 `joint_tolerance`를 `0.045 rad`로 설정했다. 별도의 XY, Z, pitch 검사는 유지된다.
- 사용자가 승인한 pick 실측 자세는 TCP `Z=27.23 mm`, pitch `90.44 deg`다. pick 완료 범위는 `25-38 mm`, pitch 상한은 `92 deg`다.
- lift는 `Z=27.2 -> 127.5 mm`, XY shift `9.02 mm`로 검증되어 시작 하한 `25 mm`, XY shift 한계 `10 mm`를 사용한다.
- place 회전은 고정 endpoint 방향까지 5도 이하 단계로 완료됐다.
- 사용자가 승인한 place 실측 자세는 TCP `Z=35.30 mm`, pitch `82.97 deg`, 목표 XY 오차 `5.75 mm`다. place 완료 범위는 `33-38 mm`, 최저 경로 `30 mm`, 최종 XY 오차 한계 `6 mm`다.
- 낮은 place 목표에서는 불필요한 lift correction을 피하기 위해 `place_correction_trigger_z`를 `100 mm`로 설정했다.

### 그리퍼 과부하 주의

- 박스 접촉 close 결과는 `stalled=true`, 실제 위치 약 `0.302 rad`였다.
- close 목표 `0.0`과 `max_effort=10`을 유지한 채 place에서 열지 않으면 ID16이 과부하 shutdown되어 빨간 LED가 점멸하고 명령을 추종하지 않는다.
- 다음 실행 전에 사용자가 ID16을 초기화하고 LED가 정상인지 확인한다.
- 재발 방지를 위해 향후 coordinator에 grasp stall 감지 후 저부하 hold 전환, place 완료 즉시 gripper open 로직을 보강해야 한다.
- 빨간 LED가 점멸하면 반복 명령을 보내지 말고 박스와 팔을 받친 뒤 원인을 제거하고 ID16을 재부팅한다.

### 현재 실행 상태

- 마지막 place descent 노드는 이미 idle이었고 추가 하강 명령은 없다.
- 마지막 측정 당시 TCP는 `Z=35.30 mm`, gripper는 약 `0.356 rad`였다.
- 사용자가 현재 그리퍼 ID16을 초기화 중이다. 초기화 후 gripper 단독 open/close 확인을 먼저 하고 coordinator를 단일 인스턴스로 시작한다.
- launch 부모만 종료하면 자식 노드가 orphan으로 남을 수 있으므로 재시작 전 같은 이름의 coordinator/stage 프로세스와 MoveJ publisher 수를 반드시 확인한다.


## 2026-08-10 place flow integration

The active coordinator launch no longer starts or requests movej_place_rotate or movej_place_lift_correction. Loaded placement now uses: loaded lift -> place XY/pitch approach -> place descent -> gripper open -> staging.

The new movej_place_recovery.yaml uses remembered place XY, target Z 60 mm, pitch 78--82 deg (preferred 80 deg), and a 10 mm final XY gate. These preserve the physical trial where planned Z 59.1 mm settled at 38.9 mm, pitch 81.65 deg, and XY error 9.22 mm. Place descent retains the approved 33--38 mm final Z band. Legacy rotation and lift-correction files remain diagnostic-only and are not launched.
