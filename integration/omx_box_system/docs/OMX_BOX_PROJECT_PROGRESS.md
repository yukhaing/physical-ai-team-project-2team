# OMX-F 파손 상자 검출·분류 프로젝트 개발 기록

> 최종 수정: 2026-08-03
> 담당 영역: 카메라/YOLO 좌표와 OMX-F Cyclo 제어 연결

## 1. 프로젝트 목표

전체 프로젝트의 최종 목표는 다음과 같다.

```text
상자더미 촬영
→ YOLO로 파손 상자 검출
→ 검출된 상자의 위치와 방향 계산
→ OMX-F가 해당 상자로 이동
→ 상자를 집어 분류
```

카메라는 완전한 탑뷰가 아니라 상자더미를 높은 측면 또는 사선 방향에서 촬영한다.

## 2. 담당 역할

담당 범위는 YOLO 모델 학습보다, 검출 또는 카메라에서 얻은 목표를 실제 로봇 명령으로 연결하는 제어 인터페이스다.

```text
상자 위치 입력
→ 카메라 좌표를 link0 로봇 좌표로 변환
→ 안전 범위와 도달 가능성 검사
→ RViz에서 목표 시각화
→ Cyclo MoveL 명령 생성
→ OMX-F 접근 위치 이동
→ 향후 집기·상승·분류 동작으로 확장
```

핵심 메시지 흐름은 다음과 같다.

```text
카메라 또는 YOLO
→ geometry_msgs/msg/PoseStamped
→ robotis_interfaces/msg/MoveL
→ Cyclo Motion Controller
→ /arm_controller/joint_trajectory
→ OMX-F
```

## 3. 개발 환경

- Ubuntu
- ROS 2 Jazzy
- Docker
- OMX-F
- `ros2_control`
- `rmw_zenoh_cpp`
- Cyclo Motion Controller
- RViz2
- HD USB 웹캠
- 프로젝트 작업공간: `/home/itec/omx_box_system`
- 컨테이너 작업공간: `/root/omx_box_system`
- 사용자 패키지: `omx_box_control`

## 4. 초기 환경 문제와 해결 내용

### Docker 및 장치

- Docker socket 권한 오류 해결
- 사용자를 `docker` 그룹에 추가
- `/dev/ttyACM0` 접근 권한과 점유 프로세스 확인
- `joint_state_broadcaster` 실행 상태 확인
- `robot_description` 대기 문제 확인
- Zenoh router 실행 필요성 확인

### Cyclo trajectory 토픽

Cyclo 기본 출력은 다음 토픽이었다.

```text
/leader/joint_trajectory
```

실제 OMX-F 컨트롤러 입력은 다음 토픽이다.

```text
/arm_controller/joint_trajectory
```

독립 프로젝트에는 물리 로봇용 설정을 다음 파일로 관리한다.

```text
/root/omx_box_system/docker/config/omx_config_physical.yaml
```

이 설정에서는 `joint_command_topic`이 `/arm_controller/joint_trajectory`로 지정되어 있다.

### QP controller

초기 이동 중 `QP solver failed` 오류를 확인했으며, 목표 자세·좌표·관절 한계와 Cyclo 설정을 점검했다. Cyclo의 충돌 회피는 로봇 자체 충돌과 제약을 보조하지만 모든 상황의 안전을 보장하지 않으므로 실제 이동 전 별도 검증이 필요하다.

## 5. Cyclo OMX 인터페이스 분석

`omx_goal`은 TF 프레임이 아니라 RViz Interactive Marker 이름이다.

| 항목 | 인터페이스 |
|---|---|
| 기준 좌표계 | `link0` |
| Interactive Marker | `omx_goal_marker` |
| Marker feedback | `/omx_goal_marker/feedback` |
| Marker update | `/omx_goal_marker/update` |
| MoveL 목표 | `/omx_movel_controller/movel` |
| 현재 말단 자세 | `/omx_movel_controller/current_pose` |
| 실제 궤적 출력 | `/arm_controller/joint_trajectory` |

제어 흐름은 다음과 같다.

```text
RViz Interactive Marker
→ /omx_goal_marker/feedback
→ interactive marker node
→ /omx_movel_controller/movel
→ Cyclo OMX MoveL controller
→ /arm_controller/joint_trajectory
→ OMX-F
```

카메라 연결 코드에서는 Interactive Marker 메시지를 모방하지 않고 실제 제어 입력인 `/omx_movel_controller/movel`에 명령을 전달한다.

## 6. PoseStamped → MoveL 브리지

구현 파일:

```text
src/omx_box_control/scripts/box_target_pose_bridge_node.py
```

입력:

```text
/box_target_pose
geometry_msgs/msg/PoseStamped
```

출력:

```text
/omx_movel_controller/movel
robotis_interfaces/msg/MoveL
```

주요 기능:

- 입력 frame이 `link0`인지 확인
- NaN/무한대 좌표 거부
- 직육면체 안전 범위 검사
- 이동 시간 설정
- 집게 수평 자세 적용
- X/Y 위치에 따라 radial yaw 계산
- `MoveL` 메시지 변환 및 발행

현재 기본 안전 범위:

```text
0.08 ≤ X ≤ 0.32 m
|Y| ≤ 0.25 m
0.01 ≤ Z ≤ 0.32 m
```

이 범위는 1차 안전 경계이며 실제 충돌 안전성과 정밀 도달 가능성을 완전히 보장하지 않는다.

## 7. RViz OMX Target 패널

구현 파일:

```text
src/omx_box_control/src/omx_target_panel.cpp
src/omx_box_control/include/omx_box_control/omx_target_panel.hpp
src/omx_box_control/plugin_description.xml
```

패널 기능:

```text
OMX Target
X (m)
Y (m)
Z (m)
[Move]
[Show/Hide Reachable Grid]
```

`Move` 버튼은 `/box_target_pose`에 `PoseStamped`를 발행한다.

### 목표 시각화

`/box_target_marker`에 `visualization_msgs/msg/MarkerArray`를 발행한다.

- 빨간색 화살표: X축
- 초록색 화살표: Y축
- 파란색 화살표: Z축
- 노란색 구: 목표 위치
- 흰색 텍스트: X/Y/Z 좌표

### 근사 도달 가능 영역

OMX-F URDF 링크 길이와 관절 제한, 수평 집게 조건을 이용한 근사 grid를 표시한다.

```text
joint2 + joint3 + joint4 ≈ 0
```

- 초록색: 관절 한계에서 비교적 여유가 있는 영역
- 노란색: 관절 한계에 가까운 영역
- 표시 없음: 근사 계산에서 도달 불가능 또는 안전 범위 밖

정밀 충돌 검사 결과가 아니라 기구학 기반 근사치다.

## 8. HD 웹캠과 평면 Homography

사용 카메라는 Depth가 없는 단안 HD 웹캠이다. 따라서 RGB-D 방식의 픽셀 depth 조회는 사용할 수 없고, 하나의 평면에 대해 Homography로 픽셀 좌표를 `link0` X/Y로 변환한다.

```text
웹캠 픽셀 (u, v)
→ 3×3 Homography
→ link0 평면 좌표 (x, y)
→ 설정된 target_z 추가
```

구현 파일:

```text
src/omx_box_control/scripts/camera_homography_7point_calibration_node.py
src/omx_box_control/config/homography_7point_calibration.yaml
src/omx_box_control/launch/camera_homography_7point_calibration.launch.py
```

### 현재 기본 기준점

| 순서 | X | Y |
|---:|---:|---:|
| 1 | 0.12 m | -0.10 m |
| 2 | 0.28 m | -0.10 m |
| 3 | 0.28 m | 0.10 m |
| 4 | 0.12 m | 0.10 m |

기본 사각형 크기:

```text
X 방향: 16 cm
Y 방향: 20 cm
```

실제 환경에서는 작업 영역을 확정한 뒤 실측한 `link0` 좌표로 반드시 변경해야 한다.

### 조작 방법

```text
c: 캘리브레이션 시작
r: 캘리브레이션 초기화
q: 영상 창 닫기
```

`c`를 누른 뒤 실제 기준점을 YAML 순서 1→2→3→4→5→6→7로 클릭한다. 계산된 행렬은 현재 환경의 `integration/omx_box_system/calibration/omx_camera_homography_7point.yaml`에 저장된다.

### 출력

계산된 미리보기 Pose:

```text
/camera_box_target
geometry_msgs/msg/PoseStamped
```

RViz Marker:

```text
/camera_box_marker
visualization_msgs/msg/Marker
```

Homography Marker publisher는 Reliable + Transient Local QoS를 사용한다. RViz에서는 `Marker` Display를 사용해야 하며 `MarkerArray`가 아니다.

### 현재 안전 상태

```text
/camera_box_target ≠ /box_target_pose
```

카메라 클릭 결과는 실제 로봇 명령과 의도적으로 분리되어 있다. 현재는 미리보기와 좌표 검증만 수행한다.

### Homography의 한계

- 하나의 평면에서만 정확함
- 카메라 위치·각도·해상도·줌이 바뀌면 재캘리브레이션 필요
- 기준 평면과 상자 윗면 높이가 다르면 사선 시차로 X/Y 오차 발생
- 네 기준점의 재투영 오차만으로 실제 정확도를 판단할 수 없음
- 캘리브레이션에 사용하지 않은 검증점으로 실측 오차를 확인해야 함

## 9. 독립 프로젝트 분리

ROBOTIS 원본 저장소에서 사용자 코드를 분리해 다음 독립 작업공간을 생성했다.

```text
/home/itec/omx_box_system
```

구조:

```text
omx_box_system/
├── docker/
├── docs/
├── calibration/
├── src/
│   └── omx_box_control/
├── README.md
└── .gitignore
```

Git 저장소는 `main` 브랜치로 초기화되어 있으며 `build/`, `install/`, `log/`와 임시 calibration 결과는 추적하지 않는다.

## 10. 독립 Docker 환경

Docker 구성:

```text
docker/Dockerfile
docker/docker-compose.yml
docker/container.sh
docker/config/omx_config_physical.yaml
```

특징:

- `robotis/open-manipulator:5.0.0` 기반 이미지
- 공식 `ROBOTIS-GIT/cyclo_control` 소스를 고정 커밋으로 이미지에 빌드
- 호스트의 기존 `~/open_manipulator` 저장소를 마운트하지 않음
- 프로젝트만 `/root/omx_box_system`으로 마운트
- `build/install/log`는 Docker named volume 사용
- 컨테이너 이름: `omx_box_system`
- ROS Domain ID: `30`
- RMW: `rmw_zenoh_cpp`

기존 `open_manipulator` 컨테이너와 새 컨테이너를 동시에 실행하지 않는다. 두 컨테이너가 동일한 물리 장치에 접근하면 시리얼 포트 점유 충돌이 발생할 수 있다.

## 11. 실행 명령

### 컨테이너 시작

호스트에서:

```bash
cd ~/omx_box_system/docker
./container.sh start
./container.sh enter
```

### 프로젝트 빌드

컨테이너 안에서:

```bash
cd /root/omx_box_system
colcon build --symlink-install
source install/setup.bash
```

편의 alias:

```bash
omx_box_build
omx_box_source
```

### Zenoh router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### OMX-F bringup

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py start_rviz:=false
```

### Cyclo MoveL

```bash
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py \
  start_interactive_marker:=true \
  config_file:=/root/omx_box_system/docker/config/omx_config_physical.yaml
```

### Pose bridge

```bash
ros2 run omx_box_control box_target_pose_bridge_node.py
```

### USB 웹캠

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
  name:=camera1 \
  video_device:=/dev/video0
```

### Homography target node

```bash
ros2 launch omx_box_control camera_homography_7point_calibration.launch.py
```

### 프로젝트 RViz

```bash
rviz2 -d \
  /root/omx_box_system/install/omx_box_control/share/omx_box_control/rviz/omx_box_system.rviz
```

RViz 확인 사항:

- Fixed Frame: `link0`
- `InteractiveMarkers`: `/omx_goal_marker/update`
- `MarkerArray`: `/box_target_marker`
- `Marker`: `/camera_box_marker`
- Panel: `omx_box_control/OmxTargetPanel`

## 12. 현재 완료 상태

- [x] OMX-F 실제 bringup
- [x] Cyclo MoveL을 실제 arm controller 토픽과 연결
- [x] `PoseStamped` → `MoveL` 브리지
- [x] 직육면체 안전 범위 검사
- [x] 수평 radial orientation 계산
- [x] RViz 숫자 입력 패널
- [x] 목표 XYZ 축·점·텍스트 시각화
- [x] 근사 도달 가능 영역 grid
- [x] HD 웹캠 영상 연동
- [x] 7점 Homography 계산
- [x] 카메라 클릭 목표 Pose 및 Marker 발행
- [x] 카메라 목표와 로봇 명령 분리
- [x] 사용자 코드 독립 ROS 패키지 분리
- [x] 독립 Docker 이미지와 Compose 환경 구축

## 13. 미완료 작업

- [ ] 실제 작업대와 카메라 고정 구조 구축
- [ ] 웹캠 내부 파라미터 캘리브레이션 및 렌즈 왜곡 보정
- [ ] 실제 `link0` 기준 Homography 기준점 측정
- [ ] 별도 검증점에서 위치 오차 측정
- [ ] Homography 노드의 안전 범위 검사
- [ ] 근사 기구학 도달 가능 여부를 노드 판정과 연결
- [ ] 상자 Pose와 Approach Pose 분리
- [ ] Confirm 기능
- [ ] 이동 완료 판정
- [ ] 그리퍼 열기/닫기
- [ ] 집기 위치로 하강
- [ ] 상자 상승 및 분류함 이동
- [ ] YOLO 자동 검출 연결
- [ ] 상자 yaw 계산
- [ ] 높이가 다른 상자에 대한 3D 위치 보완
- [ ] 모방학습 적용

## 14. 다음 개발 순서

실험 환경 구축 전:

1. Homography 노드에 안전 범위 검사 추가
2. 상자 Marker와 Approach Marker 분리
3. Confirm 전에는 `/box_target_pose` 발행 금지
4. 카메라 내부 캘리브레이션과 왜곡 보정 기능 준비
5. 실행 launch와 상태 점검 절차 통합

실험 환경 구축 후:

1. 로봇·작업대·카메라를 단단히 고정
2. 작업 평면 Z와 네 기준점의 실제 `link0` 좌표 측정
3. Homography 재캘리브레이션
4. 캘리브레이션에 사용하지 않은 여러 점에서 오차 측정
5. 안전 범위와 도달 가능성 검사
6. `box_z + 0.08~0.10 m` Approach 생성
7. RViz에서 카메라 목표와 이동 목표 비교
8. Confirm 후 저속 접근 시험
9. 위치 정확도 확인 후 집기 시퀀스 구현
10. 마지막 단계에서 YOLO 자동 검출 연결

## 15. 한 문장 요약

카메라 또는 YOLO에서 얻은 파손 상자 위치를 `link0` 기준 로봇 좌표로 변환하고, 안전 검증과 사용자 확인을 거쳐 Cyclo MoveL로 OMX-F를 상자 위 접근 위치까지 이동시키는 시스템을 개발하고 있다.
