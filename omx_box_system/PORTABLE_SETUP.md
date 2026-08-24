# 새 PC에서 OMX Box System 실행

이 디렉터리는 OMX 제어 Docker 이미지, ROBOTIS/Physical AI ROS 소스,
YOLO 가중치, 카메라 캘리브레이션과 통합 분류 코드를 포함한다.

## 1. 준비사항

- Ubuntu Linux
- Docker Engine 및 Docker Compose plugin
- X11 데스크톱 환경(GUI를 사용할 경우)
- USB로 연결된 ROBOTIS OpenRB와 카메라

저장소를 받은 뒤 프로젝트 디렉터리로 이동한다.

```bash
git clone https://github.com/yukhaing/physical-ai-team-project-2team.git
cd physical-ai-team-project-2team
git switch minseo-dev
cd omx_box_system
```

## 2. Docker 컨테이너 준비

OMX 작업 컨테이너를 빌드하고 실행한다.

```bash
./docker/container.sh start
```

Physical AI 컨테이너를 실행한다.

```bash
./docker/physical_ai/container.sh start
```

첫 빌드와 이미지 다운로드에는 시간이 걸릴 수 있다. 다음 명령으로 상태를 확인한다.

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}'
```

## 3. ROS 작업공간 빌드

```bash
docker exec -it omx_box_system bash -lc \
  "source /opt/ros/jazzy/setup.bash && cd /root/omx_box_system && colcon build --symlink-install"
```

## 4. 하드웨어 경로 확인

OpenRB 경로는 PC마다 달라질 수 있다.

```bash
ls -l /dev/serial/by-id
```

현재 launch 기본값과 다르면 bringup 실행 시 `port_name:=...`을 지정한다.

카메라 번호도 PC마다 달라질 수 있다.

```bash
v4l2-ctl --list-devices
```

검증 당시 고정 카메라는 `/dev/video2`를 `/camera1/image_raw`로 발행했다.

## 5. 실행 순서

각 명령은 별도 터미널에서 실행한다.

Zenoh router:

```bash
docker exec -it omx_box_system bash -lc \
  "source /opt/ros/jazzy/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"
```

OMX bringup:

```bash
docker exec -it omx_box_system bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /root/omx_box_system/install/setup.bash && \
  ros2 launch open_manipulator_bringup omx_f.launch.py"
```

고정 카메라:

```bash
docker exec -it omx_box_system bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /root/omx_box_system/install/setup.bash && \
  ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=camera1 video_device:=/dev/video2"
```

YOLO 파일을 AI 컨테이너로 복사:

```bash
docker cp experiments/yolo_calibrated_preview.py physical_ai_server:/tmp/yolo_calibrated_preview.py
docker cp calibration/omx_camera_homography_7point_20260820.yaml \
  physical_ai_server:/tmp/omx_camera_homography_7point.yaml
docker cp models/box_defect_best.pt physical_ai_server:/tmp/box_defect_best.pt
```

YOLO 노드:

```bash
docker exec -it physical_ai_server bash -lc \
  "source /opt/ros/jazzy/setup.bash && python3 /tmp/yolo_calibrated_preview.py"
```

선택사항인 YOLO GUI:

```bash
docker exec -it omx_box_system bash -lc \
  "source /opt/ros/jazzy/setup.bash && ros2 run rqt_image_view rqt_image_view /yolo/angle_annotated"
```

통합 자동 분류:

```bash
docker exec -it omx_box_system bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
  python3 /root/omx_box_system/experiments/yolo_pick_sort_once.py"
```

## 6. PC별로 반드시 확인할 값

- `/dev/serial/by-id/...`: OpenRB 시리얼 ID
- `/dev/video*`: 고정 카메라 번호
- 카메라 위치가 바뀐 경우 7점 Homography 재캘리브레이션
- `ROS_DOMAIN_ID=30`, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
- 로봇 주변 충돌 여부와 HOME 자세

`build/`, `install/`, `log/`, 크래시 덤프와 Hugging Face 캐시는 Git에 포함하지 않는다.
이들은 새 PC에서 빌드 또는 다운로드로 다시 생성된다.
