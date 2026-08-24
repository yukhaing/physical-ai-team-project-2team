# OMX Box System 인수인계 — 2026-08-11

이 문서는 다음 세션에서 **이 파일 하나만 첨부해도 현재 상태에서 바로 이어서 작업**할 수 있도록 작성한 인수인계 문서다.

## 1. 현재 목표와 전체 파이프라인

최종 목표:

```text
고정 카메라 YOLO defect 검출
→ bbox 중심을 link0 X/Y로 변환 및 보정
→ OMX-F를 상자 위 시작 자세로 이동
→ ACT 모방학습 정책 실행
→ 파손 상자 집기 및 분류
```

현재까지 YOLO 좌표 이동과 ACT 실제 입력 dry-run, 제한된 ACT arm 10-step 시험까지 성공했다. ACT 전체 동작과 gripper 제어를 연결하는 작업이 남아 있다.

## 2. 하드웨어와 실행 환경

- 작업공간: `/home/itec/omx_box_system`
- Docker 컨테이너:
  - `omx_box_system`: ROS 2 Jazzy, OMX-F, 카메라, YOLO
  - `physical_ai_server`: LeRobot/ACT 추론 환경
- ROS Domain ID: `30`
- RMW: `rmw_zenoh_cpp`
- 로봇: OMX-F
- OpenRB serial ID: `CAD761565157375037202020FF0D022B`
- 안정적인 포트 경로:

```text
/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_CAD761565157375037202020FF0D022B-if00
```

- DYNAMIXEL ID 11~16 확인 완료
- 카메라 두 대 모두 Logitech C270:
  - `/dev/video0`: 고정 상단 카메라
  - `/dev/video2`: wrist camera
  - 각 카메라의 보조 video node는 `/dev/video1`, `/dev/video3`
- GPU: 사용 불가. `torch.cuda.is_available() == False`
- ACT는 CPU에서 정상 작동

## 3. 현재 실행 중인 주요 노드와 프로세스

문서 작성 시점에 다음이 실행 중이다.

- Zenoh router
- 고정 카메라 `/camera1` (`/dev/video0`)
- wrist camera `/wrist_camera` (`/dev/video2`)
- 고정 카메라 Homography 창
- wrist camera 미리보기 창
- OMX-F ros2_control bringup, torque ON
- YOLO defect 우선 GUI 임시 프로세스
- `/move_to_selected_box` 임시 ROS Trigger 서비스

주요 ROS 노드:

```text
/arm_controller
/gripper_controller
/joint_state_broadcaster
/controller_manager
/robot_state_publisher
/usb_cam                  # 이름이 같은 노드 2개 존재
/camera_homography_target # 이름이 같은 노드 2개 존재
/yolo_priority_go
/yolo_defect_move_service
```

중요: YOLO와 이동 서비스는 `python3 -`로 실행한 임시 프로세스다. 컨테이너 재시작 시 사라진다.

## 4. 현재 실제 로봇 상태

문서 작성 직전 `/joint_states`:

```text
관절 순서:
[joint1, joint2, joint3, joint4, joint5, gripper_joint_1]

현재값:
[-0.40803889,
  0.53842726,
 -0.58751464,
  1.75487402,
  0.07209710,
  0.69949524]
```

- torque ON
- 로봇은 마지막 ACT 10-step 제한 시험 종료 위치에서 정지
- gripper는 학습 시작 분포에 맞춰 약 `0.70 rad`
- 연속 ACT 정책은 실행 중이 아님

## 5. Homography 캘리브레이션

7점 Homography 캘리브레이션 완료.

파일:

```text
/tmp/omx_camera_homography_7point.yaml
```

주의: 컨테이너 내부 `/tmp` 파일이므로 컨테이너 삭제/재생성 시 사라진다. 단순 stop/start에는 유지된다.

오늘 캘리브레이션 점:

| 점 | link0 X,Y (m) | pixel |
|---|---|---|
| 1 | `(0.00, -0.33)` | `(28, 43)` |
| 2 | `(0.00, 0.12)` | `(593, 37)` |
| 3 | `(0.30, 0.07)` | `(592, 408)` |
| 4 | `(0.29, -0.28)` | `(37, 417)` |
| 5 | `(0.11, -0.17)` | `(219, 177)` |
| 6 | `(0.20, -0.025)` | `(420, 272)` |
| 7 | `(0.10, 0.075)` | `(551, 147)` |

- 평균 reference reprojection error: `4.3 mm`
- 독립 검증 `(0.035, -0.120)` 클릭 결과 `(0.036, -0.119)`
- 독립 검증 합성 오차 약 `1.4 mm`

## 6. YOLO 모델과 선택 규칙

현재 시험 모델은 팀 GitHub의 기존 YOLOv8s 모델이다.

```text
GitHub: https://github.com/yukhaing/physical-ai-team-project-2team
branch: yolo-dev
model: yolo/models/best.pt
컨테이너 경로: /tmp/yolo/models/best.pt
```

- 클래스: `normal`, `defect`
- `normal` 표시 기준: `confidence >= 0.75`
- `defect` 표시 기준: `confidence >= 0.35`
- 선택 우선순위:
  1. defect가 있으면 defect만 후보
  2. defect가 여러 개면 confidence 최고값
  3. defect가 없을 때만 normal 최고값
- 5프레임 좌표 변화가 반경 `5 mm` 이내일 때 stable

기존 모델 평가 참고:

- overall Precision 87.4%, Recall 88.6%, mAP50 88.3%
- defect Precision 83.9%, Recall 88.7%, mAP50 86.5%

## 7. YOLO bbox 중심 추가 보정

상자 전체 bbox 중심과 실제 상자 윗면 중심의 편향을 4점으로 보정했다.

Homography raw 좌표를 `(raw_X, raw_Y)`라고 할 때 화면과 이동에 사용한 보정식:

```text
X = 0.93976184 * raw_X + 0.07624367 * raw_Y + 0.01421060
Y = 0.02776865 * raw_X + 0.99214406 * raw_Y - 0.00519881
```

보정 데이터 기준:

- 평균 오차: `7.3 mm → 2.8 mm`
- 최대 오차: `15.5 mm → 5.5 mm`
- 6번 점 재검증 목표 `(0.200, -0.025)`, 표시 `(0.200, -0.028)`

## 8. OMX analytic IK

Cyclo MoveL은 이 작업에서 사용하지 않는다. 고정 orientation/Z 유지가 불안정했고 QP 실패가 반복됐다.

확정 파이프라인:

```text
보정된 link0 X/Y
→ analytic IK
→ Z 중력 처짐 보정
→ /arm_controller/joint_trajectory
```

기구 파라미터:

```text
joint1 origin X = -0.01125 m
shoulder Z = 0.0975 m
first offset = (0.0415, 0.11315) m
second link = 0.162 m
tool length = 0.0287 + 0.09193 = 0.12063 m
tool lateral offset = -0.0016 m
fixed downward pitch = 1.612214 rad
fixed joint5 = -0.00614 rad
gravity Z compensation = +0.0092 m
nominal target Z = 0.1148 m
IK calculation Z = 0.1240 m
```

선택 방식:

1. pitch `1.612214 rad` 고정 아래보기 IK 시도
2. 도달 불가능하면 pitch `0.60~1.612214 rad` 탐색
3. 가능한 해 중 가장 아래보기에 가까운 pitch 선택
4. cosine `<= 0.95`로 완전 펴진 특이자세 회피

검증 결과:

- 목표 `(0.2000, -0.0280, 0.1148)`
- 실제 `(0.2010, -0.0268, 0.1165)`
- 오차 X/Y/Z = `1.0/1.2/1.7 mm`

먼 위치 adaptive IK 검증:

- 목표 `(0.1828, -0.1906, 0.1148)`
- 실제 `(0.1811, -0.1889, 0.1108)`
- 오차 X/Y/Z = `-1.7/+1.7/-4.0 mm`
- 실제 pitch 약 `78.3°`

고정 아래보기 경계 검증:

- 목표 `(0.0656, -0.2081, 0.1148)`
- 실제 `(0.0678, -0.2126, 0.1126)`
- 실제 pitch `92.7°`

## 9. `/move_to_selected_box` 임시 서비스

서비스:

```text
/move_to_selected_box
std_srvs/srv/Trigger
```

호출:

```bash
ros2 service call /move_to_selected_box std_srvs/srv/Trigger '{}'
```

현재 임시 서비스의 안전 XY:

```text
0.03 <= X <= 0.30
-0.28 <= Y <= 0.10
```

`X min`은 처음 `0.08`이었으나 캘리브레이션 검증 범위에 맞춰 `0.03`으로 낮췄다.

마지막 서비스 성공 응답:

```text
defect X=0.1872 Y=-0.1313 pitch=89.6 spread=0.3mm
```

주의: 이 서비스는 아직 소스 파일로 영구 저장되지 않았다. 파일 편집 도구가 다음 오류로 실패했다.

```text
bwrap: loopback: Failed RTM_NEWADDR: Operation not permitted
```

다음 세션에서 우선 정식 ROS 노드로 저장해야 한다.

## 10. ACT 모방학습 모델

Hugging Face 모델:

```text
https://huggingface.co/baemseo/omx_box_v1
```

학습 데이터셋:

```text
https://huggingface.co/datasets/baemseo/omx_f_pick_and_place_damaged_box2
```

모델 정보:

- 정책: ACT
- 파라미터: 51,668,662 (F32)
- 모델 파일 약 207 MB
- 데이터셋: 60 episodes, 18,023 frames, 30 FPS
- task: `Pick up the damaged box and place it in the target area`
- action chunk size: 100
- n_action_steps: 100

필수 입력:

```text
observation.images.wrist_camera: [3, 480, 640]
observation.images.fixed_camera: [3, 480, 640]
observation.state: [6]
```

state/action 관절 순서:

```text
[joint1, joint2, joint3, joint4, joint5, gripper_joint_1]
```

출력:

```text
action: [6]
```

카메라 토픽 대응:

```text
fixed_camera  <- /camera1/image_raw 또는 /camera1/image_raw/compressed
wrist_camera  <- /wrist_camera/image_raw 또는 /wrist_camera/image_raw/compressed
state         <- /joint_states
```

## 11. LeRobot 버전 호환 문제와 해결법

현장 `physical_ai_server`의 LeRobot:

```text
version 0.2.0
```

모델 설정은 `device: cuda`지만 현장 CUDA는 사용 불가다.

다음 방식은 실패한다.

```python
ACTConfig.from_pretrained('baemseo/omx_box_v1')
```

오류:

```text
The fields `type` are not valid for ACTConfig
```

정상 로딩 방식:

```python
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy

cfg = PreTrainedConfig.from_pretrained('baemseo/omx_box_v1')
# cuda가 없으므로 자동으로 cpu 전환
policy = ACTPolicy.from_pretrained(
    'baemseo/omx_box_v1',
    config=cfg,
    cache_dir='/tmp/hf_models',
)
policy.eval()
```

CPU dry-run 결과:

- 모델 로드 약 `13.85초`
- 더미 입력 첫 추론 약 `0.619초`
- 실제 입력 추론 약 `0.263초`
- 출력 shape `(1, 6)`

실제 입력 dry-run 성공. 로봇에는 발행하지 않았다.

## 12. ACT 학습 시작 분포

60개 에피소드 첫 프레임 state 중앙값:

```text
[ 0.042951,
  0.013039,
 -0.114282,
  1.638291,
  0.093573,
  0.703330]
```

첫 프레임 state 범위:

```text
min [-0.524621, -0.556835, -1.012427, 1.360641, -0.346680, 0.636602]
max [ 0.612058,  0.730175,  0.754719, 1.788622,  0.401903, 0.756253]
```

첫 action 중앙값:

```text
[ 0.052155, -0.031447, -0.145728, 1.642893, 0.091272, 0.705631]
```

기존 파일 `src/omx_box_control/config/imitation_start.yaml`은 이번 데이터셋의 실제 시작 분포와 일치하지 않는다. 그대로 사용하지 말 것.

정책 시험 전에 현재 자세에서 다음을 조정했다.

- joint4를 `1.78 rad`로 조정
- gripper를 `0.70 rad`로 조정

## 13. ACT 실제 시험 결과

### 실제 입력 dry-run

정렬 전 실제 입력 예측은 로봇에 발행하지 않았다.

정렬 후 state:

```text
[-0.587515, 0.538427, -0.875903, 1.782486, -0.006136, 0.699495]
```

첫 ACT action:

```text
[-0.283395, 0.396264, -0.714608, 1.822114, -0.028844, 0.697673]
```

첫 action 최대 arm delta는 joint1의 약 `0.304 rad`이므로 raw action 직접 발행은 금지한다.

### 실제 arm 10-step 제한 시험

성공 조건:

- arm 5관절만 실행
- gripper는 `0.70`으로 유지
- 실제 state 대비 관절당 최대 `0.02 rad/step` clamp
- 각 command `time_from_start=0.15s`
- 총 10 step 후 자동 종료

결과:

- 10 step 정상 완료
- NaN 없음
- stale camera/state 없음
- 관절 범위 오류 없음
- 정책 출력이 일관된 방향으로 이어짐
- 연속 정책은 현재 실행 중이 아님

## 14. 절대 금지/주의사항

1. ACT raw action을 `/arm_controller/joint_trajectory`에 바로 연결하지 말 것.
2. Physical AI Server 기본 converter는 `JointTrajectoryPoint.time_from_start`를 지정하지 않는다. 안전 브리지가 필요하다.
3. 다음 안전 브리지를 반드시 둘 것.

```text
ACT action
→ finite 검사
→ 관절 순서 검사
→ 실제 state 대비 delta clamp
→ 절대 관절 범위 검사
→ time_from_start 추가
→ arm_controller 및 gripper_controller로 분리 발행
```

4. ACT 모델은 wrist/fixed 카메라 두 개가 모두 필요하다.
5. 카메라 위치/각도가 학습 당시와 달라지면 정책 성공률이 크게 떨어진다.
6. YOLO adaptive IK 시작 자세가 ACT 첫 프레임 분포와 너무 다르면 raw 정책이 크게 움직일 수 있다.
7. gripper는 arm controller가 아니라 `/gripper_controller/gripper_cmd` action으로 제어해야 한다.
8. Cyclo MoveL은 이 자세에서 사용하지 않는다.

## 15. 내일 바로 시작할 순서

1. 이 문서를 읽는다.
2. 컨테이너와 현재 실행 프로세스가 살아 있는지 확인한다.
3. `/joint_states`, 두 카메라 토픽, `/move_to_selected_box` 서비스 확인.
4. wrist/fixed 영상이 학습 시점과 일치하는지 재확인.
5. ACT를 `PreTrainedConfig.from_pretrained()` 경로로 CPU 로드.
6. 현재 실제 입력으로 **비발행 dry-run** 1회.
7. ACT 안전 브리지를 정식 ROS 노드로 구현:
   - arm delta 초기 `0.02 rad/step`
   - gripper delta 초기 `0.01~0.02 rad/step`
   - command duration `0.15s`
   - 최대 실행 step/timeout
   - stop service 또는 즉시 중단 경로
8. gripper 포함 30-step 제한 시험.
9. 정상일 때만 한 episode 전체 실행.
10. 완료 후 home 복귀.

## 16. 재시작용 핵심 명령

환경:

```bash
source /opt/ros/jazzy/setup.bash
source /root/omx_box_system/install/setup.bash
```

OMX bringup(자동 초기자세 이동 금지):

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py \
  start_rviz:=false \
  init_position:=false \
  port_name:=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_CAD761565157375037202020FF0D022B-if00
```

고정 카메라:

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
  name:=camera1 video_device:=/dev/video0
```

wrist camera:

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
  name:=wrist_camera video_device:=/dev/video2
```

서비스 호출:

```bash
ros2 service call /move_to_selected_box std_srvs/srv/Trigger '{}'
```

## 17. 다음 세션의 첫 요청 문구

다음 세션에서 이 파일을 첨부하고 아래처럼 요청하면 된다.

```text
이 인수인계 문서를 읽고 현재 실행 상태를 확인한 다음,
ACT 안전 브리지를 구현해서 gripper 포함 30-step 제한 시험부터 이어서 진행해줘.
raw ACT action은 절대 직접 발행하지 말고 현재 joint state 기준 delta clamp를 유지해줘.
```
