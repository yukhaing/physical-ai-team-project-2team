# OMX YOLO Box Sorting Integration

고정 카메라의 YOLO 검출 좌표를 7점 Homography로 `link0` 좌표로 변환하고,
OMX-F가 파손 상자를 집어 분류 위치로 옮기는 계산 기반 통합 시스템이다.

## 핵심 파일

- `experiments/yolo_calibrated_preview.py`: YOLO 검출, 좌표·상자 각도 계산 및 `/yolo/selected_box` 발행
- `experiments/yolo_pick_sort_once.py`: HOME부터 검출·파지·분류·HOME 복귀까지 한 번에 실행
- `calibration/omx_camera_homography_7point_20260820.yaml`: 고정 카메라 7점 캘리브레이션
- `models/box_defect_best.pt`: YOLO 파손 상자 검출 가중치
- `launch/omx_f.launch.py`: 현재 OpenRB 기본 시리얼 ID가 반영된 bringup launch 파일
- `experiments/act_*.py`: 모방학습 ACT 연결 실험 및 안전 브리지

## 실행 순서

## 새 작업 환경에서 7점 calibration

카메라를 최종 위치에 단단히 고정한 다음 진행한다. 로봇을 움직이는 노드가
아니며 카메라 영상에서 기준점을 측정하고 YOLO 호환 YAML을 저장하기만 한다.

먼저 실제 작업면에서 서로 충분히 떨어진 일곱 점의 `link0` 기준 X/Y(m)를
측정하고 다음 파일의 `reference_points_link0`를 수정한다.

```text
src/omx_box_control/config/homography_7point_calibration.yaml
```

빌드 후 카메라와 calibration 창을 실행한다.

```bash
cd /root/omx_box_project_ws
colcon build --packages-select omx_box_control --symlink-install
source install/setup.bash
ros2 launch omx_box_control camera_homography_7point_calibration.launch.py
```

창에서 `c`를 누르고 설정 파일에 적은 순서대로 1번부터 7번까지 클릭한다.
잘못 클릭했으면 `u`로 마지막 점을 취소하고, 전부 다시 하려면 `c`, 종료는
`q`를 사용한다. 일곱 번째 점을 클릭하면 다음 파일로 자동 저장된다.

```text
integration/omx_box_system/calibration/omx_camera_homography_7point.yaml
```

화면과 로그의 평균·최대 재투영 오차를 확인한다. 이 오차는 사용한 기준점에
대한 오차이므로, 저장 후 calibration에 쓰지 않은 별도 위치들을 실측해 최종
XY 오차를 검증해야 한다. 카메라 위치나 각도, 작업면 높이가 바뀌면 다시
calibration한다. 새 결과는 다음 YOLO 시작 시 자동으로 우선 적용된다.

현재 프로젝트에서는 모델, calibration, 별도 Python 가상환경을 준비하고
검출기를 실행하는 다음 호스트 명령을 권장한다.

```bash
cd ~/omx_box_project_ws
./scripts/start_yolo_detector.sh
```

상태 확인:

```bash
./scripts/status_yolo_detector.sh
```

기본 실행 장치는 CPU다. CUDA 실행 환경이 별도로 검증된 경우에만 다음처럼
변경한다.

```bash
OMX_YOLO_DEVICE=0 ./scripts/start_yolo_detector.sh
```

아래는 구성 요소를 수동으로 실행할 때의 상세 순서다.

각 명령은 별도 터미널에서 실행한다.

1. Zenoh router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

2. OMX bringup

```bash
ros2 launch open_manipulator_bringup omx_f.launch.py
```

3. 고정 카메라

```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py \
  name:=camera1 video_device:=/dev/video2
```

4. YOLO 코드, 캘리브레이션, 가중치를 `physical_ai_server`에 복사

```bash
docker cp experiments/yolo_calibrated_preview.py \
  physical_ai_server:/tmp/yolo_calibrated_preview.py
docker cp calibration/omx_camera_homography_7point_20260820.yaml \
  physical_ai_server:/tmp/omx_camera_homography_7point.yaml
docker cp models/box_defect_best.pt \
  physical_ai_server:/tmp/box_defect_best.pt
```

5. `physical_ai_server` 컨테이너에서 YOLO 노드

```bash
python3 /tmp/yolo_calibrated_preview.py
```

6. `omx_box_system` 컨테이너에서 통합 분류

```bash
python3 /root/omx_box_system/experiments/yolo_pick_sort_once.py
```

YOLO GUI는 `/yolo/angle_annotated`, 자동 분류 입력은 `/yolo/selected_box`를 사용한다.
정상 상자 임계값은 `0.75`, 파손 상자 임계값은 `0.35`다.

## 검증된 동작

```text
bringup 초기 HOME
-> defect 안정 검출
-> adaptive pitch analytic IK
-> 상자 방향에 맞춘 joint5 정렬
-> 연속 하강 및 파지
-> 연속 상승
-> 분류 위치 이동 및 배출
-> bringup 초기 HOME 복귀
```

파지 중 액션 응답이 지연되는 경우 실제 gripper 관절 위치로 파지 성공을 판정한다.
중간 종료 후 상자를 잡고 있는 상태에서만 `--resume-grasped`를 사용할 수 있다.

YOLO 가중치 `models/box_defect_best.pt`는 이 저장소에 포함한다. 모방학습 모델은
용량 때문에 Git에 포함하지 않고 <https://huggingface.co/baemseo/omx_box_v1>에 보관한다.
