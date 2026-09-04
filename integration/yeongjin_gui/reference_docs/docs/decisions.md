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
