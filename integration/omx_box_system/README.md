# OMX YOLO Box Sorting Integration

고정 카메라의 YOLO 검출 좌표를 7점 Homography로 `link0` 좌표로 변환하고,
OMX-F가 파손 상자를 집어 분류 위치로 옮기는 계산 기반 통합 시스템이다.

## 핵심 파일

- `experiments/yolo_calibrated_preview.py`: YOLO 검출, 좌표·상자 각도 계산 및 `/yolo/selected_box` 발행
- `experiments/yolo_pick_sort_once.py`: HOME부터 검출·파지·분류·HOME 복귀까지 한 번에 실행
- `calibration/omx_camera_homography_7point_20260820.yaml`: 고정 카메라 7점 캘리브레이션
- `calibration/tools/camera_homography_target_node.py`: 화면에서 기준점을 클릭해 Homography를 생성하는 노드
- `calibration/launch/camera_homography_target.launch.py`: 캘리브레이션 launch 파일
- `models/box_defect_best.pt`: YOLO 파손 상자 검출 가중치
- `launch/omx_f.launch.py`: 현재 OpenRB 기본 시리얼 ID가 반영된 bringup launch 파일
- `experiments/act_*.py`: 모방학습 ACT 연결 실험 및 안전 브리지

## 실행 순서

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

## 7점 캘리브레이션 다시 하기

카메라를 옮겼거나 화면 구도가 바뀌었을 때만 다시 진행한다. 고정 카메라
`/camera1/image_raw`를 실행한 상태에서 다음 명령을 사용한다.

```bash
python3 calibration/tools/camera_homography_target_node.py --ros-args \
  -p image_topic:=/camera1/image_raw \
  -p reference_points_link0:='[0.0,-0.33,0.0,0.12,0.30,0.07,0.29,-0.28,0.11,-0.17,0.20,-0.025,0.10,0.075]' \
  -p calibration_file:=/tmp/omx_camera_homography_7point_new.yaml \
  -p show_window:=true
```

창에서 `c`를 누른 후 아래 순서대로 기준점 7개를 클릭한다.

| 순서 | link0 X (m) | link0 Y (m) |
|---:|---:|---:|
| 1 | 0.00 | -0.33 |
| 2 | 0.00 | 0.12 |
| 3 | 0.30 | 0.07 |
| 4 | 0.29 | -0.28 |
| 5 | 0.11 | -0.17 |
| 6 | 0.20 | -0.025 |
| 7 | 0.10 | 0.075 |

일곱 번째 점을 클릭하면 YAML이 자동 저장된다. `r`은 초기화, `q`는 창 닫기다.
이 노드는 좌표 미리보기만 발행하며 로봇을 움직이지 않는다. 새 결과를 실제 YOLO에
사용하기 전에는 기준점을 다시 클릭해 표시 좌표 오차를 확인한다.

YOLO 가중치 `models/box_defect_best.pt`는 이 저장소에 포함한다. 모방학습 모델은
용량 때문에 Git에 포함하지 않고 <https://huggingface.co/baemseo/omx_box_v1>에 보관한다.
