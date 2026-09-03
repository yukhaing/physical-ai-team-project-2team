# integration/ - Defect Transfer Integration

이 폴더는 YOLO 검출, 불량 박스 선택 GUI, OMX 적재 흐름이 실제로 만나는
통합 관제 관련 문서와 진입점을 모아 둡니다.

## Current Structure

- `README.md`: 통합 관제 개요와 현재 구조 설명
- `DEFECT_TRANSFER_CONSOLE.md`: 통합 관제 GUI 초기 설정, 실행, 조작 절차

## Current Integration Shape

기존 계획과 달리 별도 `main_pipeline.py` 하나로 연결하는 구조가 아니라,
`omx/src/omx_box_control` 내부 launch와 노드에서 아래 기능이 함께 동작합니다.

- YOLO 검출 결과 표시
- 불량 박스 선택
- Beagle TCP 상태 수신과 `box_placed` 트리거 전송
- OMX pick coordinator 연동
- Beagle 불량 구역 이동, 하역 완료, 복귀 사이클
- 작업 로그 기록

## Notes

- 현재 목적은 통합 관제 시나리오와 실장비 연동 흐름을 함께 검증하는 것입니다.
- Beagle은 같은 PC 또는 별도 제어 PC에서 실행할 수 있습니다.
- 실환경 최종 캘리브레이션 값은 추후 반영 예정입니다.
