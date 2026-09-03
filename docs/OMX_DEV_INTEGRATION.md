# OMX 통합본 기반 관제 연결

이 관제 패키지는 PyQt 운영 화면을 유지하면서 `omx-dev` 통합본의 ROS 인터페이스를 별도 모듈로 사용한다.

## 분리된 구성

- `scripts/camera_homography_7point_calibration_node.py`: OMX 통합본 방식의 측정 전용 7점 카메라 보정 도구다. 로봇 명령을 발행하지 않는다.
- `config/homography_7point_calibration.yaml`: 장비별 7개 `link0` 기준점을 관리한다.
- `scripts/yolo_target_bridge_node.py`: `/yolo/selected_box`의 불량 판정·신뢰도·작업영역·좌표 안정성을 확인한 뒤 `/camera_box_target`을 발행한다.
- `sorting_orchestrator_node.py`: 같은 `/yolo/selected_box`에서 불량을 확인하면 먼저 Beagle 이송을 시작한다. Beagle 도착 후 OMX 흐름을 시작하면 YOLO bridge가 안정 타깃을 전달한다.

캘리브레이션 결과는 현재 `runtime/calibration/active.yaml`에 저장된다. 재보정은 이 파일만 바꾸므로 GUI와 Beagle 제어 코드에 영향을 주지 않는다.

## 보정

카메라를 먼저 실행한 뒤 다음 명령을 실행한다.

```bash
ros2 launch omx_box_control camera_homography_7point_calibration.launch.py
```

`c`를 누르고 YAML에 정의된 순서대로 7개 기준점을 클릭한다. 저장된 결과를 관제에 적용하려면 기존 통합 관제 노드를 재시작한다.

## OMX YOLO 인터페이스

외부 YOLO 런타임은 `std_msgs/msg/Float64MultiArray`를 `/yolo/selected_box`으로 발행한다.

```text
[is_defect, confidence, x_link0_m, y_link0_m, joint5_rad(optional)]
```

bridge는 최소 4개 안정 샘플을 요구하며, 불량이 아니거나 작업영역 밖인 타깃은 OMX로 전달하지 않는다. 실행은 다음 launch 파일을 사용한다.

```bash
ros2 launch omx_box_control yolo_target_bridge.launch.py
```

GUI 카메라 선택 경로와 OMX 외부 YOLO 경로는 둘 다 지원한다. 실제 장비에서는 하나의 타깃 공급자만 활성화해 `/camera_box_target` 중복 발행을 피한다.
