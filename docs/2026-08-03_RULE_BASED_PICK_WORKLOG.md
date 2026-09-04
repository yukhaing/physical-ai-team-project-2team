# OMX-F Rule-based Pick 개발 기록 — 2026-08-03

## 오늘의 목표

단안 카메라 Homography로 선택한 박스 좌표를 이용해 OMX-F가 다음 동작을 수행하도록 연결했다.

```text
카메라 목표 선택
→ 집기 자세 접근
→ 그리퍼 닫기
→ 박스를 위로 들어 올리기
```

초기 목표는 그리퍼를 정확히 수직 90도로 유지하는 것이었지만, 실제 OMX-F/Cyclo 제약을 시험한 결과 약 60~72도 정도의 기울기로 집고 MoveJ로 후퇴하는 방식이 더 안정적이었다.

## 구현한 파일

### Rule-based pick

- `src/omx_box_control/scripts/rule_based_pick_node.py`
- `src/omx_box_control/config/rule_based_pick.yaml`
- `src/omx_box_control/launch/rule_based_pick.launch.py`

주요 인터페이스:

```text
입력 목표: /camera_box_target (geometry_msgs/PoseStamped)
MoveL 출력: /omx_movel_controller/movel
현재 자세: /omx_movel_controller/current_pose
그리퍼: /gripper_controller/gripper_cmd
확인 서비스: /rule_based_pick/confirm
취소 서비스: /rule_based_pick/cancel
상태: /rule_based_pick/status
```

상태 머신에 목표 유효시간, 작업범위 검사, 위치·방향 완료 검사, timeout, 재시도, controller error 처리를 추가했다.

현재 주요 설정:

```yaml
movej_staging_done: true
tool_pitch: 1.25
approach_offset: 0.05
grasp_offset: 0.0
lift_offset: 0.05
```

### MoveJ staging

- `src/omx_box_control/scripts/movej_staging_node.py`
- `src/omx_box_control/config/movej_staging.yaml`
- `src/omx_box_control/launch/movej_staging.launch.py`

MoveL이 수렴하지 못한 staging pose를 관절 공간에서 이동하도록 구현했다.

```yaml
staging_positions: [0.0, -0.467, 0.376, 1.291, 0.0]
```

실제 결과:

```text
말단 X = 0.1812m
말단 Y = -0.0016m
말단 Z = 0.1201m
최대 관절 오차 ≈ 1.13도
```

### MoveJ lift

- `src/omx_box_control/config/movej_lift.yaml`
- `src/omx_box_control/launch/movej_lift.launch.py`

정확한 수직 상승 대신 X/Y 변화를 허용하고 Z 상승을 우선하는 후퇴 자세를 구현했다.

```yaml
staging_positions: [0.0, -0.113, 0.098, 0.820, 0.0]
```

실제 결과:

```text
상승 전: X=0.2799m, Z=0.0627m
상승 후: X=0.2629m, Z=0.1302m
실제 Z 상승량: 약 6.75cm
최대 관절 오차: 0.89도
그리퍼: 닫힌 상태 유지
```

## 실제 시험에서 확인한 내용

### MoveL staging 실패

MoveL로 위치와 자세를 동시에 고정하면 목표에 수렴하지 않았다.

```text
목표: (0.180, 0.000, 0.120), pitch=1.20rad
실제: (0.227, 0.000, 0.089)
위치 오차: 약 56.9mm
```

OMX-F는 5자유도이므로 임의의 3차원 위치와 3차원 자세를 모두 독립적으로 만족시킬 수 없다.

### 완전 수직 자세의 한계

URDF 기반 계산에서 완전 수직 staging 자세는 `joint4` 상한에 매우 가까웠다.

```text
필요 joint4 ≈ 1.735rad
joint4 상한 = 1.745rad
```

완전 수직 상태에서 높은 접근점도 도달 불가능했다.

```text
X=0.205, Z=0.150, pitch=90도
이론적 최소 위치 오차 ≈ 35.6mm
```

따라서 `tool_pitch=1.25rad`로 낮췄다. 약 72도 명령에서도 MoveL 제자리 회전은 timeout이 발생했지만, 실물에서는 당시 자세로 박스를 집을 수 있음을 확인했다.

### 그리퍼 닫기

```text
Goal status: SUCCEEDED
reached_goal: true
position: -0.0061
effort: 10.0
```

### MoveL lift와 MoveJ lift 비교

MoveL로 현 X/Y와 자세를 고정한 5cm 상승은 약 2cm만 상승하고 X가 약 2cm 변했다. MoveJ lift는 X 변화를 허용해 약 6.75cm 상승에 성공했다.

## Cyclo controller 제약

공식 `omx_controller.launch.py`는 다음 중 하나만 실행한다.

```text
controller_type:=movej
controller_type:=movel
```

두 controller를 동시에 실행하면 둘 다 `/arm_controller/joint_trajectory`를 발행하므로 사용하면 안 된다. 현재 검증된 방식은 수동 전환이다.

```text
MoveJ staging
→ MoveJ 종료
→ MoveL 실행
→ 카메라 목표 및 집기
→ MoveL 종료
→ MoveJ 실행
→ MoveJ lift
```

## 오늘 종료 시 실제 런타임 상태

실행 중인 주요 노드:

```text
/omx_movej_controller
/movej_lift
```

실행되지 않은 노드:

```text
/omx_movel_controller
/rule_based_pick
```

현재 말단 자세:

```text
X=0.262911m
Y=-0.001600m
Z=0.130202m
```

현재 관절:

```text
joint1 =  0.000000
joint2 = -0.095107
joint3 =  0.116583
joint4 =  0.825282
joint5 =  0.001534
```

그리퍼는 닫혀 있고 MoveJ controller가 lift 자세를 유지 중이다. 실제 박스가 물려 있다면 전원이나 컨테이너를 끄기 전에 박스를 안전하게 내려놓고 그리퍼를 열어야 한다.

## 남은 문제와 다음 작업

1. 한 launch에서 전체 사이클을 실행할 수 있도록 MoveJ/MoveL 전환 coordinator 설계
2. 집기 성공 후 MoveL 픽 노드를 종료하고 MoveJ lift로 넘어가는 명확한 handoff 추가
3. `ROTATING_DOWN` 실패를 오류가 아닌 허용 가능한 실제 집기 자세로 처리하는 조건 설계
4. 그리퍼 action 결과의 `reached_goal`, abort, timeout 처리 강화
5. controller error 한 번으로 즉시 실패하지 않고 지속 여부를 판단하도록 정책 통일
6. staging/lift 관절 목표에 대한 충돌 및 관절 margin 검사 추가
7. 전체 실행 중 중복 노드와 중복 publisher 방지
8. 실제 박스 높이에 맞춰 Homography `target_z` 및 grasp 높이 보정

## 빌드 검증

```text
Python syntax: 통과
git diff --check: 통과
colcon build --packages-select omx_box_control: 통과
```

현재 신규 기능 파일은 아직 Git에 commit되지 않았다.
