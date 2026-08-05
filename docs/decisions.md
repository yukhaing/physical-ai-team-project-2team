# 팀 결정 사항 기록

## 손상 유형 (Damage Type)
찌그러짐, 찢어짐 등 여러 손상 유형을 세부적으로 나누지 않고, 
"Damaged Box" 클래스 하나로 통합. 
클래스는 총 2개: normal(정상), defect(손상)

## Dataset 소스 (업데이트됨)
최초 Roboflow 데이터셋(926장)에서 → carton-can-detection 데이터셋(4-class, 3439장)으로 전환 
→ 2-class로 merge하여 최종 사용
- 최종 클래스: normal, defect
- Train 3009 / Valid 288 / Test 142

## Model 비교 및 최종 선정
YOLOv8s, m, l을 동일 데이터셋으로 학습 후 model.val() 직접 출력으로 비교:

| Model   | mAP50  | mAP50-95 | Precision | Recall |
| YOLOv8s | 0.9210 | 0.7750   | 0.9231    | 0.8509 |
| YOLOv8m | 0.9159 | 0.7679   | 0.8571    | 0.9226 |
| YOLOv8l | 0.9180 | 0.7707   | 0.9097    | 0.8517 |

최종 선정 모델: [YOLOv8s]

best.pt는 로컬 yolo/models/best.pt에 저장 (GitHub 미포함, 용량 문제)

