# OMX 통합 관제 및 불량 이송

이 디렉터리는 OMX-F 로봇팔의 카메라 기반 집기, YOLO 검출 표시, Qt 관제 GUI,
Beagle 연동과 SQLite 작업 로그를 포함합니다.

## 구현 상태

- 불량 박스 선택과 OMX pick-and-place 흐름
- `beagle_adapter_node.py`의 TCP 상태·트리거·정지 요청 연동
- 같은 PC 및 2-PC Beagle 배치 지원
- Beagle 복귀 중 다음 박스 집기, 수령 위치 도착 뒤 최종 배치
- 집기 실패 복구와 비상정지 뒤 안전 복귀

## 빠른 실행

```bash
cd omx/docker
BEAGLE_MODE=auto ./container.sh gui-up
```

`gui-status`로 상태를 확인하고 `gui-down`으로 종료합니다. 초기 Docker/ROS 빌드,
수동 실행, GUI 조작 절차는
[`../integration/DEFECT_TRANSFER_CONSOLE.md`](../integration/DEFECT_TRANSFER_CONSOLE.md)를
참고합니다.

## 안전

MoveJ와 MoveL controller를 동시에 실행하지 마십시오. 실장비 운용 전에는
`config/console.yaml` 설정과 최종 카메라 보정값을 검증하고, 위험 상황에서는
물리 E-stop을 사용해야 합니다.
