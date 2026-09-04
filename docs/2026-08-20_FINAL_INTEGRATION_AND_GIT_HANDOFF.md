# 2026-08-20 OMX 박스 제어 최종 통합 및 Git 인수인계

## 오늘 작업 결과

- 카메라로 선택한 pick 목표에서 시작해 staging, XY 접근, pitch 정렬, pick, loaded lift, place 접근, place 하강, gripper open, staging 복귀까지 이어지는 통합 coordinator 구성을 최종본으로 정리했다.
- pick/place 하강은 기존 반복 하강 방식 대신 별도로 구현한 단일 PTP 하강 노드를 통합 흐름에 연결했다.
- place 하강 승인과 gripper open 전 continue 입력을 제거해, 검증된 이전 단계가 완료되면 자동으로 다음 단계가 진행되도록 구성했다.
- staging 단계의 빈 그리퍼 close, pick gripper open/close watchdog, 안전 실패 시 다음 명령을 발행하지 않는 상태 처리를 반영했다.
- loaded lift에서는 충돌 회피에 중요한 Z 상승을 기준으로 완료를 판단하고, 상승 중 XY 이동량 때문에 불필요하게 중단되던 조건을 완화했다.
- pick 후 이동 경로의 높이와 저장한 place 접근 자세를 반영하고, place 최종 자세에 맞춘 전용 단일 PTP 하강을 추가했다.
- 각 단계의 trajectory duration, 최소 완료 시간, settle 시간을 조정해 전체 cycle의 불필요한 대기를 줄였다.
- RViz 설정, homography 설정, Docker controller 패치, 실행 launch/config/script와 작업 기록을 재현 가능한 형태로 포함했다.

## 최종 주요 동작 흐름

```text
STAGING + EMPTY GRIPPER CLOSE
→ WAIT_PICK_TARGET
→ 사용자 /pick_coordinator/continue 1회
→ XY APPROACH
→ PITCH PREGRASP
→ PICK GRIPPER OPEN
→ SINGLE PTP PICK DESCENT
→ PICK GRIPPER CLOSE
→ LOADED LIFT
→ PLACE XY/PITCH APPROACH
→ SINGLE PTP PLACE DESCENT
→ GRIPPER OPEN
→ STAGING RETURN
→ EMPTY GRIPPER CLOSE
→ COMPLETE
```

정상 cycle에서는 pick 목표를 받은 뒤 접근 시작을 위해 `/pick_coordinator/continue`를 호출하고,
`require_grasp_confirmation: true` 설정에서는 실제 grasp 확인 후 loaded lift 시작을 위해 한 번 더 호출한다.
Place 하강과 gripper open에는 추가 승인이 필요 없다.

## 최종 주요 설정값

- pick 단일 PTP 하강 duration: `1.0 s`
- loaded lift 목표 Z: `0.17561 m`
- loaded lift 완료 Z 범위: `0.165–0.190 m`
- place 상부 접근 자세: `[0.02347, -0.15494, 0.19253] m`
- place 최종 자세: `[0.02379, -0.15335, 0.14399] m`
- place 최종 pitch: `82.27 deg`
- place 단일 PTP 하강 duration: `1.0 s`

주요 timing 설정:

- staging: trajectory `4.0 s`, minimum completion `5.0 s`, settle `0.15 s`
- XY approach: trajectory `5.5 s`, minimum completion `6.5 s`, settle `0.15 s`
- pitch: trajectory `4.0 s`, minimum completion `5.0 s`, settle `0.15 s`
- loaded lift: trajectory `6.0 s`, minimum completion `7.0 s`, settle `0.15 s`
- place transfer: trajectory `4.5 s`, minimum completion `5.5 s`, settle `0.15 s`

실물 환경과 박스 크기가 바뀌면 Z 목표와 안전 범위를 다시 측정해야 한다. Cartesian 범위 검사는 실제 충돌 안전을 보장하지 않는다.

## 주요 파일

- 통합 coordinator: `src/omx_box_control/scripts/pick_coordinator_node.py`
- 통합 launch: `src/omx_box_control/launch/pick_coordinator.launch.py`
- coordinator 설정: `src/omx_box_control/config/pick_coordinator.yaml`
- pick 단일 PTP: `movej_single_ptp_descent_node.py`, `movej_single_ptp_descent.yaml`
- place 단일 PTP: 같은 노드를 별도 이름과 `movej_single_ptp_place_descent.yaml` 설정으로 실행
- XY/pitch/place 접근: `movej_xy_approach_node.py`와 관련 YAML
- staging: `movej_staging_node.py`, `movej_staging.yaml`
- loaded lift: `movej_loaded_lift_node.py`, `movej_lift.yaml`
- RViz: `src/omx_box_control/rviz/omx_box_project.rviz`
- controller 변경: `docker/patches/`

## 검증 결과

- 모든 Python script와 launch 파일의 문법 검사를 통과했다.
- 모든 package YAML 파일의 파싱 검사를 통과했다.
- 실행 중인 `omx_box_project` ROS 2 Jazzy 컨테이너에서 다음 빌드를 통과했다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/omx_box_project_ws
colcon build --packages-select omx_box_control --symlink-install
```

## Git 저장 상태

최종 코드 커밋:

```text
38c0fcd Finalize coordinated OMX box pick and place flow
```

동일한 커밋이 다음 두 위치에 업로드되었다.

- 개인 저장소: `origin/main`
  - `https://github.com/cjh-123468/omx_box_project/tree/main`
- 팀 저장소: `team/cjh-dev`
  - `https://github.com/yukhaing/physical-ai-team-project-2team/tree/cjh-dev`

팀 저장소의 `cjh-dev`는 `git push --force-with-lease team main:cjh-dev`로 현재 로컬 `main` 상태에 맞췄다.

`docker/config/omx_config_position_priority.yaml`은 실험용 미추적 파일로 판단해 최종 코드 커밋과 GitHub 업로드에서 제외했다.

## 다음 작업 시작 절차

```bash
cd /home/itec/omx_box_project_ws
git status --short
git log -1 --oneline --decorate
git fetch team
git status
```

실행 전에는 반드시 다음을 확인한다.

1. 로봇 주변과 이동 경로에 충돌물이 없는지 확인한다.
2. gripper ID16의 빨간 LED 점멸이나 torque shutdown이 없는지 확인한다.
3. 동일한 coordinator/controller 노드가 중복 실행 중이지 않은지 확인한다.
4. 실제 `/joint_states`와 현재 staging 자세를 확인한다.
5. 카메라, homography, MoveJ controller, coordinator 순서로 필요한 노드를 실행한다.

작업을 이어갈 때는 이 문서와 `docs/CODEX_HANDOFF.md`를 먼저 읽고, 현재 코드와 실제 로봇 상태가 기록 당시 조건과 같은지 확인한다.
