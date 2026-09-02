# 통합 관제

이 폴더는 YOLO 검출, OMX-F 적재, Beagle 이송이 만나는 통합 관제 문서를
관리합니다.

- `DEFECT_TRANSFER_CONSOLE.md` — 초기 설정, GUI 실행, Beagle 연동과 실장비
  확인 절차

별도 `main_pipeline.py` 대신 `omx/src/omx_box_control`의 ROS 2 launch와 노드가
YOLO 표시·선택, 좌표 변환, OMX pick coordinator, Beagle TCP 통신, SQLite 로그를
통합합니다. Beagle이 복귀를 시작하면 다음 박스를 집을 수 있고, 최종 배치는
수령 위치 도착이 확인된 뒤에만 진행합니다.
