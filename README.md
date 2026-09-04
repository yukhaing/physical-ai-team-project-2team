# OMX–Beagle 통합 박스 이송 시스템

ROS 2 Jazzy 기반의 적재·하역 OMX-F, Beagle 이동 로봇, 듀얼 카메라와 관제
GUI를 하나의 작업 흐름으로 실행하는 프로젝트다. 현재 운용 소스는
`integration/yeongjin_gui/omx`에 있으며, 루트의 `src/omx_box_control`은 기존
단독 OMX 워크플로를 보존한다.

상세 운용 문서는 다음을 기준으로 한다.

- [통합 관제 및 전체 사이클](integration/yeongjin_gui/integration/DEFECT_TRANSFER_CONSOLE.md)
- [하역 OMX 보정·티칭·단독 실행](integration/yeongjin_gui/UNLOAD_PROCESS.md)
- [Beagle 최신 셔틀 미션](integration/yeongjin_gui/Beagle_Lidar_and_AStar/README.md)
- [Beagle SLAM 지도 작성](integration/yeongjin_gui/Beagle_mobile_robot/ros2/beagle_slam/README.md)

`docs/`의 날짜가 붙은 파일은 당시 시험 기록이므로 현재 실행 명령보다 위 문서를
우선한다.

## 빠른 실행

현재 장비에서는 호스트의 다음 작업공간을 사용한다.

```bash
cd /home/itec/omx_box_project_ws
BEAGLE_MODE=local ./docker/container.sh gui-up
```

`gui-up`은 필요한 ROS 패키지를 빌드하고 다음 프로세스를 하나의 tmux 세션에서
실행한다.

1. 공용 Zenoh daemon
2. 적재 OMX bringup 및 MoveJ controller
3. 하역 OMX bringup, controller 및 초기화
4. 적재·하역 USB 카메라
5. 카메라 역할 라우터와 적재 YOLO/하역 랜드마크 검출
6. Beagle adapter와 로컬 셔틀 미션
7. 관제 GUI와 상태 monitor

실행 상태와 로그는 다음 명령으로 확인한다.

```bash
./docker/container.sh gui-status
./docker/container.sh gui-attach
```

전체 종료:

```bash
./docker/container.sh gui-down
```

`gui-up`은 적재·하역 OMX의 초기 자세를 준비하지만 박스 사이클을 시작하지 않는다.
작업 공간과 Beagle 위치를 확인한 뒤 GUI에서 `가동`을 누른다.

## 기본 장치와 환경변수

기본 카메라는 적재 `/dev/video0`, 하역 `/dev/video2`다. OpenRB-150 두 대는 알려진
영구 장치 경로를 우선 사용하며 자동 판별할 수 없으면 실행을 거부한다.

```bash
OMX_PORT_NAME=/dev/serial/by-id/<적재_OMX_ID> \
UNLOAD_OMX_PORT_NAME=/dev/serial/by-id/<하역_OMX_ID> \
OMX_VIDEO_DEVICE=/dev/video0 \
UNLOAD_VIDEO_DEVICE=/dev/video2 \
BEAGLE_MODE=local \
./docker/container.sh gui-up
```

- `ENABLE_UNLOAD_OMX=false`: 하역 OMX 자체를 실행하지 않는다.
- `AUTOMATIC_UNLOAD_OMX=false`: 하역 OMX 자동 시작을 끄고 GUI의 `하역 완료`를
  Beagle 복귀용 예비 경로로 사용한다.
- `BEAGLE_MODE=local|auto|remote`: 같은 PC 또는 별도 PC의 Beagle adapter 연결 방식을
  선택한다.
- `BEAGLE_LOCAL_MISSION_LAUNCH=auto|true|false`: 로컬 Robomation receiver가 있을 때
  셔틀 미션 자동 실행 여부를 정한다.

시작 전 스크립트는 두 OMX 시리얼 장치와 Beagle receiver의 중복, 두 카메라의 존재와
장치 중복을 검사한다.

## 현재 자동 사이클

1. Beagle이 수령 위치 복귀 완료 상태를 보낸다.
2. 적재 카메라에서 선택한 박스를 적재 OMX가 집어 Beagle에 놓는다.
3. `box_placed`를 받은 Beagle이 하역 구역으로 이동한다.
4. `defect_arrived`를 받으면 현재 작업 ID로 하역 OMX를 한 번 시작한다.
5. 하역 OMX가 박스를 집어 안전 높이로 올리면 `box_picked`를 즉시 전송한다.
6. Beagle은 수령 위치로 복귀하고, 하역 OMX는 동시에 180도 회전·배출·복귀를
   계속한 뒤 그리퍼를 닫는다.
7. 다음 적재는 Beagle의 수령 위치 복귀 완료 신호가 들어온 뒤에만 시작한다.

하역 파지는 최초 시도 후 최대 2회 재시도한다. 복귀 신호 전 실패하면 Beagle을
정지 상태로 유지한다. 박스를 올린 뒤 실패하면 Beagle 복귀는 계속하며 동일 작업의
`box_picked`를 재전송하지 않는다.

## 카메라 역할

장치 입력과 작업 역할 토픽은 분리되어 있다.

- 장치 입력: `/camera_devices/loading/image_raw`,
  `/camera_devices/unloading/image_raw`
- 작업 역할: `/camera/image_raw`, `/unload_camera/image_raw`
- GUI 영상: `/console/annotated_image`, `/unload_vision/annotated_image`

두 카메라가 반대로 연결됐으면 GUI의 `카메라 맞바꾸기`를 사용한다. 영상 표시뿐 아니라
적재 검출과 하역 랜드마크 입력이 함께 교환되고 상태는
`integration/yeongjin_gui/runtime/camera_roles.yaml`에 저장된다. 카메라 하나가 끊긴
상태에서는 역할 교환이 거부된다.

## 현재 적재 설정

- 적재 그리퍼 완전 열림: `0.98 rad`
- Beagle place 접근 Z: `0.19253 m`
- Beagle place release Z: `0.13899 m`
- place 하강 안전 하한: `0.133 m`

이 값은 현재 설치 상태의 실측 설정이다. 로봇 베이스, 트레이 또는 카메라 위치를
바꿨다면 기존 수치를 그대로 사용하지 말고 캘리브레이션과 빈 그리퍼 검증을 다시 한다.

## 안전

GUI의 정지·비상정지는 소프트웨어 명령 취소와 hold 요청이다. 사람이나 장비가 위험한
상황에서는 반드시 물리 E-stop 또는 전원 차단을 사용한다. 실패로 멈춘 사이클은 원인을
확인한 뒤 GUI의 `재설정`으로 두 OMX를 초기화하고 다시 `가동`한다.
