# OMX YOLO Box Sorting Integration

고정 카메라의 YOLO 검출 좌표를 7점 Homography로 `link0` 좌표로 변환하고,
OMX-F가 파손 상자를 집어 분류 위치로 옮기는 계산 기반 통합 시스템이다.

## 핵심 파일

- `experiments/yolo_calibrated_preview.py`: YOLO 검출, 좌표·상자 각도 계산 및 `/yolo/selected_box` 발행
- `experiments/yolo_pick_sort_once.py`: HOME부터 검출·파지·분류·HOME 복귀까지 한 번에 실행
- `calibration/omx_camera_homography_7point_20260820.yaml`: 고정 카메라 7점 캘리브레이션
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

4. `physical_ai_server` 컨테이너에서 YOLO 노드

```bash
python3 /tmp/yolo_calibrated_preview.py
```

5. `omx_box_system` 컨테이너에서 통합 분류

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

모델 가중치는 Git에 포함하지 않는다. 모방학습 모델은
<https://huggingface.co/baemseo/omx_box_v1>에 보관되어 있다.
