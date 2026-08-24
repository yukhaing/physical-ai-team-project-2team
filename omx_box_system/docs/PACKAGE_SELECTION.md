# 팀 결정 사항 기록

## 손상 유형 (Damage Type)
찌그러짐, 찢어짐 등 여러 손상 유형을 세부적으로 나누지 않고,
"Damaged Box" 클래스 하나로 통합.
클래스는 총 2개: Box(정상), Damaged Box(손상)

## Dataset 소스
기존 Roboflow 데이터셋을 base로 사용:
https://universe.roboflow.com/inft2060-ffjce/cardboard-boxes-4h3rr/dataset/2
- 클래스: Box, Damaged Box
- 총 926장
- 계획: 이 데이터셋으로 1차 fine-tuning 후, 필요하면
   실제 환경에서 촬영한 사진으로 2차 fine-tuning 진행

## Data Contract (YOLO → OMX)
[OMX 팀과 협의 후 작성 예정]

sample one
------------

YOLO팀의 detect_and_select() 함수가 반환할 것으로 예상되는 형식:

{
    "class": 문자열, "damaged" 또는 "intact" 둘 중 하나,
    "confidence": 실수, 0.0 ~ 1.0 사이 값,
    "robot_x": 실수, 단위는 meter, 로봇 base 기준 좌표,
    "robot_y": 실수, 단위는 meter, 로봇 base 기준 좌표
}

박스가 하나도 감지되지 않거나, 모든 감지된 박스의 confidence가
threshold 미만이면 함수는 None을 반환함

OMX팀과 확인해야 할 사항:
- 좌표 기준점(원점, (0,0))이 어디인지 확인 필요
- 단위 확인 필요 (meter인지 millimeter인지)
- None이 반환됐을 때 OMX쪽에서 어떻게 처리할지 확인 필요 (재시도? 건너뛰기? 알림?)
- 필드 이름(robot_x, robot_y)이 괜찮은지, OMX쪽에서 다른 이름 규칙을 쓰는지 확인 필요