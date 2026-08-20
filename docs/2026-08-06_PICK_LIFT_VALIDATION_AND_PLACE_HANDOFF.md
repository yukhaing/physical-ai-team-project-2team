# 2026-08-06 OMX-F Pick/Lift 실증 및 Place 작업 인계

## 1. 오늘의 목표와 결론

오늘은 기존 MoveJ 기반 박스 픽 흐름을 실제 로봇에서 다시 검증하고, 하강 히스테리시스와 적재 상승 오차를 소프트웨어적으로 보정했다.

개별 단계 기준으로 다음 순서까지 구현 및 실증했다.

1. staging 복귀
2. 카메라 목표 선택
3. XY approach
4. pitch pregrasp
5. 그리퍼 완전 열기
6. closed-loop Z 하강
7. 박스 파지
8. joint2/joint3 보정 loaded lift
9. 상승 위치 유지

최종 보정 상승 실증은 실제 Z `135.01 mm`, 목표 Z 오차 `0.60 mm`, 목표 XY 오차 `4.15 mm`로 성공했다.

단, 그리퍼가 파지 직후에는 박스를 유지하다가 시간이 지나면서 0 rad까지 닫혀 박스를 놓치는 현상이 반복되었다. 따라서 장시간 파지 유지 문제는 아직 남아 있다.

## 2. 작업 중 준수한 안전 조건

- 실제 MoveJ와 MoveL controller를 동시에 실행하지 않았다.
- `/omx_movej_controller/movej` publisher는 실제 명령 전 항상 0개, 명령 node 실행 중 정확히 1개인지 확인했다.
- 각 node는 완료 또는 실패 후 종료하여 publisher를 0개로 되돌렸다.
- timeout 또는 안전 gate 실패 후 추가 이동을 중단했다.
- 실제 `/joint_states` 기반 FK를 Cartesian 완료 판정에 사용했다.
- `/omx_movej_controller/current_pose`는 command-state 기반이므로 참고값으로만 사용했다.
- 기존 dirty worktree를 보존했고 reset, checkout, clean, 파일 삭제를 하지 않았다.

## 3. 단계별 실증 결과

### 3.1 Staging

초기화 후 팔이 낮고 접힌 자세에 있었지만, 관절 보간 경로를 샘플링한 결과 시작점보다 더 내려가지 않는 것을 확인한 뒤 실제 staging을 실행했다.

- 경로 예상 최저 Z: 약 `46.9 mm` (시작점)
- staging 실제 완료 최대 관절 오차: `1.02 deg`
- staging 종료 후 MoveJ publisher: 0개

### 3.2 XY Approach

최종 선택 목표:

- X: 약 `204.7 mm`
- Y: 약 `106.7 mm`

실제 결과:

- 실제 XY 오차: `3.51 mm`
- 실제 Z: `160.8 mm`
- 최대 관절 오차: `1.60 deg`
- 기존 6 mm approach 완료 제한 안에서 성공

### 3.3 Pitch Pregrasp

첫 시도는 Cartesian 결과가 안전 범위였지만 joint3 오차가 기존 0.04 rad 제한을 아주 조금 초과했다.

- 실제 XY 오차: `2.02 mm`
- 실제 Z: `134.7 mm`
- 계획 pitch: `73.15 deg`
- joint3 오차: 약 `2.35 deg` (`0.04109 rad`)
- 기존 제한: `0.04 rad` (`2.29 deg`)

실제 Cartesian 결과를 포함하도록 다음 값만 최소 확대했다.

```yaml
joint_tolerance: 0.042
```

유지한 독립 안전 조건:

- XY 제한: `5 mm`
- 최종 Z 하한: `130 mm`

수정 파일:

- `src/omx_box_control/config/movej_pitch_pregrasp.yaml`

### 3.4 Gripper Open

- 시작 실제 FK Z: `134.7 mm`
- 시작 XY 오차: `2.02 mm`
- 시작 pitch: `76.03 deg`
- 최종 그리퍼 실측: 약 `0.9986 rad`
- 양쪽 핑거 대칭 열림은 사용자 육안 확인

### 3.5 Closed-loop Descent 문제와 수정

초기 하강에서는 계획 1 mm에 실제 TCP가 17.8~20.3 mm 내려가는 문제가 발생했다.

관찰값:

- 첫 실패: 실제 하강 약 `17.8 mm`, 최대 관절 오차 `3.43 deg`
- 두 번째 실패: 실제 하강 약 `20.3 mm`, 최대 관절 오차 `4.48 deg`
- `max_actual_step`은 `15 mm`에서 `18 mm`로 변경했지만 이것만으로는 해결되지 않았다.

원인은 IK 후보 선택에서 미세한 XY 오차 차이를 우선하면서 joint3/joint4가 크게 반대 방향으로 회전하는 해를 고른 것이었다. 관절 추종 오차로 두 관절의 상쇄가 깨지면서 Z 변화가 증폭되었다.

적용한 수정:

- 하강 step마다 현재 실제 pitch를 preferred pitch로 사용
- 최소 XY 오차에서 0.5 mm 이내인 후보 중 목적함수가 가장 작은 해 선택
- 하강 `joint_delta_weight: 100.0`
- `candidate_xy_slack: 0.0005`
- `max_actual_step: 0.018`

수정 파일:

- `src/omx_box_control/scripts/movej_xy_approach_node.py`
- `src/omx_box_control/scripts/movej_closed_loop_descent_node.py`
- `src/omx_box_control/config/movej_closed_loop_descent.yaml`

수정 후 단일-step 계획:

- joint2 변화: `+0.136 deg`
- joint3 변화: `+0.113 deg`
- joint4 변화: `-0.045 deg`
- 계획 XY 오차: `0.03 mm`

수정 후 실제 단일-step 결과:

- 시작 Z: 약 `96.6 mm`
- 최종 실제 Z: `87.39 mm`
- 실제 하강: 약 `9.2~9.5 mm`
- 실제 XY 오차: `2.04 mm`
- pitch: `82.44 deg`
- 목표 Z 구간 `80~95 mm` 진입 성공
- 자동 추가 step 없음

### 3.6 Grasp

정상적으로 박스가 중앙에 배치된 실증에서:

- 접촉 stall 위치: 약 `0.2516 rad`
- 3초 후 실측: 약 `0.2500 rad`
- controller 결과: `stalled=true`, `reached_goal=false`, action status `ABORTED`

박스 접촉 때문에 0 rad에 도달하지 못한 ABORTED는 예상 결과이다. 이때는 사용자 육안 파지 확인을 함께 사용했다.

문제점:

- 일부 파지에서는 stall 직후 0.15~0.30 rad였지만 시간이 지나면서 0 rad까지 계속 닫혔다.
- 박스가 결국 빠지는 현상이 반복되었다.
- 장시간 운반 전에 별도 grasp-hold 전략 또는 지속 감시가 필요하다.

### 3.7 Loaded Lift 실패 분석

일반 position-priority lift를 pitch 유지 또는 Z 우선으로 실행했지만 상승 목표를 따라가지 못했다.

두 상승의 반복 잔류 오차:

| 관절 | pitch 우선 상승 | Z 우선 상승 |
|---|---:|---:|
| joint2 | `+4.78 deg` | `+4.54 deg` |
| joint3 | `+2.44 deg` | `+2.82 deg` |
| joint4 | `-1.27 deg` | `-0.61 deg` |

현재 자세의 Z 민감도:

- joint2 오차 1 deg당 Z 약 `-4.17 mm`
- joint3 오차 1 deg당 Z 약 `-3.20 mm`
- joint4 오차 1 deg당 Z 약 `-0.40 mm`

joint2와 joint3 잔류 오차로 예상되는 Z 부족분은 약 27.9 mm였고, 실제 부족분 약 28 mm와 일치했다.

Controller 로그:

- 10초 S-curve 종료
- 3초 bounded settle 수행
- 최대 오차 `0.0603 rad`, `0.0562 rad`
- 기존 terminal trim은 joint3만 대상으로 하므로 joint2 최대 오차를 보정하지 못함
- controller가 feedback hold로 전환한 뒤 application timeout을 늘려도 추가 추종은 발생하지 않음

### 3.8 Joint2/Joint3 보정 Loaded Lift 성공

최근 반복 잔류 오차를 전량 적용하지 않고 기존 terminal trim 범위에 맞춰 제한적으로 보정했다.

```yaml
joint2 compensation: -0.06 rad
joint3 compensation: -0.04 rad
```

명령 전 관절 경로 검사:

- 시작 실제 Z: 약 `107.92 mm`
- 명령 관절 경로는 시작점부터 계속 상승
- 명령 경로 최저 Z: 시작점
- 보정 명령 FK 종점 Z: 약 `157.85 mm`
- 실제 히스테리시스 잔류를 고려한 예상 Z: 약 `135.6 mm`

실물 결과:

- 시작 Z: `107.92 mm`
- 최종 실제 Z: `135.01 mm`
- 실제 상승량: `27.09 mm`
- 목표 Z: `135.61 mm`
- 목표 Z 오차: `-0.60 mm`
- XY 이동량: `3.99 mm`
- 목표 XY 오차: `4.15 mm`
- 최종 pitch: `71.54 deg`
- 당시 그리퍼: 약 `0.2470 rad`
- MoveJ publisher 종료 후 0개

## 4. 정식 Loaded Lift 구현

실증된 보정을 고정 staging 명령으로 덮어쓰지 않고 전용 feedback-gated node로 구현했다.

추가/변경 파일:

- `src/omx_box_control/scripts/movej_loaded_lift_node.py`
- `src/omx_box_control/config/movej_lift.yaml`
- `src/omx_box_control/launch/movej_lift.launch.py`
- `src/omx_box_control/CMakeLists.txt`

정식 node의 안전 gate:

- 시작 Z: `75~120 mm`
- 파지 그리퍼: `0.05~0.60 rad`
- 이동 중 허용 경로 하강: 최대 `1 mm`
- 보정 명령 FK Z 상한: `170 mm`
- 실제 완료 Z: `130~145 mm`
- 실제 상승량: `15~60 mm`
- 실제 XY 이동: 최대 `5 mm`
- 그리퍼 변화: 최대 `0.05 rad`
- 실패 시 후속 명령 없음
- 최종 완료는 관절 오차가 아니라 실제 Z, XY, 상승량, 그리퍼 유지로 판정

기본 설정은 안전을 위해 `dry_run: true`이다.

실제 실행:

```bash
ros2 launch omx_box_control movej_lift.launch.py dry_run:=false
```

다른 터미널에서:

```bash
ros2 service call /movej_lift/confirm std_srvs/srv/Trigger "{}"
```

검증:

- Python 구문 검사 통과
- `colcon build --packages-select omx_box_control --symlink-install` 통과
- `dry_run:=false` launch argument가 실제 Boolean false로 로드됨을 확인
- 그리퍼가 약 0 rad일 때 `gripper is not holding a box`로 confirm 거부 확인
- node 종료 후 MoveJ publisher 0개 확인

## 5. 현재 종료 상태

문서 작성 직전 ROS graph 전체가 내려가 있었다.

- `/omx_movej_controller/movej`: 존재하지 않음
- `/joint_states`: 발행 없음
- gripper action server: 0개
- MoveJ controller 프로세스: 없음
- MoveL controller 프로세스: 없음

따라서 현재 물리 자세는 새로 측정할 수 없었다.

ROS 종료 전 마지막 신뢰 측정:

- 팔은 보정 상승 후 높은 자세에 있었음
- `/omx_movej_controller/current_pose` 참고값:
  - X 약 `208.51 mm`
  - Y 약 `107.57 mm`
  - Z 약 `141.36 mm`
- 보정 상승 직후 실제 joint-state FK:
  - X 약 `209.06 mm`
  - Y 약 `107.21 mm`
  - Z 약 `135.01 mm`
- 이후 그리퍼는 약 `0.003 rad`까지 닫혔고 박스는 더 이상 파지되지 않았음

내일 재개 시 위 값들을 현재 상태로 가정하지 말고 반드시 다시 측정해야 한다.

## 6. 내일 반드시 수행할 재개 전 점검

파일 변경 또는 실제 이동 전에 다음을 모두 확인한다.

```bash
git status --short
```

- 실행 중인 ROS node
- MoveJ controller 종류와 프로세스 수
- MoveL controller 종류와 프로세스 수
- `/omx_movej_controller/movej` publisher 종류와 개수
- 실제 `/joint_states`
- 실제 joint-state FK TCP pose
- 그리퍼 실제 위치와 action client/server

제약:

- MoveJ와 MoveL controller 동시 실행 금지
- MoveJ publisher 중복 실행 금지
- 기존 dirty worktree 보존
- `git reset`, `checkout`, `clean`, 파일 삭제 금지

## 7. 다음 작업: 단순 Place 흐름

사용자가 원하는 place 방식은 복잡한 Cartesian 운반 대신 상승 자세에서 joint1만 회전하여 OMX 우측 고정 섹션으로 옮기는 방식이다.

권장 흐름:

1. loaded lift 완료
2. joint1-only 우측 회전
3. 기존 MoveJ closed-loop descent 로직을 place 설정으로 재사용
4. 지정된 place Z 구간에서 그리퍼 완전 열기
5. 동일 경로 또는 저장한 하강 전 관절 자세로 상승
6. 필요 시 joint1을 원래 값으로 복귀

### 7.1 `movej_place_rotate_node` 계획

- joint2~joint5는 회전 시작 시 실제값으로 고정
- joint1만 고정 target 또는 상대 delta로 이동
- 10초 S-curve
- joint1 물리 한계와 최대 회전량 검사
- FK로 전체 원호 경로의 최저 Z 검사
- 회전 중 그리퍼 파지 유지 확인
- 실제 최종 TCP X/Y 기록
- 박스 없이 작은 각도로 먼저 방향 검증

아직 결정되지 않은 값:

- OMX 우측 방향의 joint1 부호
- 최종 joint1 각도
- 고정 배치면의 허용 Z 구간

### 7.2 Place Descent 계획

기존 `movej_closed_loop_descent_node.py` 알고리즘을 재사용하되 설정은 분리한다.

```text
movej_closed_loop_descent.yaml   # pick용
movej_place_descent.yaml         # place용
```

Place용 차이:

- 카메라 목표 대신 joint1 회전 직후의 실제 X/Y 자동 고정
- 배치면용 Z 구간 사용
- 시작 시 그리퍼 파지 확인
- 하강 중 XY 이동 제한
- 실제 Z가 place 구간에 진입하기 전 gripper release 금지

### 7.3 남은 핵심 문제

Place 운반 전에 그리퍼 장시간 유지 문제를 해결하거나 최소한 감시해야 한다.

권장:

- grasp 위치를 계속 유지하는 action/hold 전략 검토
- 이동 중 그리퍼 값이 시작 파지값에서 일정 이상 변하면 즉시 팔 이동 중단
- 파지 직후 3초뿐 아니라 10~20초 유지 검증

## 8. 현재 구현 수준 요약

완료:

- staging부터 보정 loaded lift까지 개별 단계 구현
- 실제 pick 위치 접근, 하강, 단기 파지, 보정 상승 실증
- loaded lift 전용 safety-gated node 정식 반영

미완료:

- 최신 로직을 하나의 coordinator로 연결한 완전 자동 실행
- 장시간 안정적인 gripper grasp hold
- joint1-only place rotate
- place descent/release/retract

현재 시스템은 목표 선택, 박스 배치, 손 이탈, 육안 파지 확인이 포함된 반자동 단계 실행 방식이다.
