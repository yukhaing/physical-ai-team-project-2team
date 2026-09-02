# OMX 외부 YOLO 연동

통합 관제는 PyQt 운영 화면과 OMX ROS 인터페이스를 분리해 사용합니다.

- `camera_homography_7point_calibration_node.py` — 로봇 명령을 내리지 않는
  7점 카메라 보정 도구
- `yolo_target_bridge_node.py` — 불량 판정, 신뢰도, 작업 영역, 좌표 안정성을
  검증해 `/camera_box_target`을 발행하는 bridge
- `sorting_orchestrator_node.py` — YOLO·OMX·Beagle 상태 전이와 작업 흐름 제어

보정 결과는 `runtime/calibration/active.yaml`에 저장합니다. 외부 YOLO는
`/yolo/selected_box`에 아래 형식으로 발행합니다.

```text
[is_defect, confidence, x_link0_m, y_link0_m, joint5_rad(optional)]
```

bridge는 최소 4개 안정 샘플을 요구하며, 불량이 아니거나 작업영역 밖인 대상은
OMX로 전달하지 않습니다.
