# YOLO 모델 크기별 성능 비교

Drive의 `runs/yolov8s_100e_two_classes`, `yolov8m_100e_two_classes`,
`yolov8l_100e_two_classes` 실행 결과를 비교한 표입니다. 각 수치는
`metrics/mAP50-95(B)`가 가장 높았던 epoch의 검증 지표입니다.

| 모델 | 최고 epoch | Precision | Recall | mAP@50 | mAP@50-95 | 100 epoch 기록 시간 |
|---|---:|---:|---:|---:|---:|---:|
| YOLOv8s | 81 | 0.88149 | 0.90858 | 0.93704 | **0.78847** | 5,775.47초 |
| YOLOv8m | 79 | 0.86727 | 0.88621 | 0.93060 | 0.78047 | 2,507.95초 |
| YOLOv8l | 69 | 0.86436 | 0.90346 | 0.93297 | 0.77657 | 15,495.90초 |

## 결론

이 세 실행 결과에서는 YOLOv8s가 Precision, Recall, mAP@50, mAP@50-95 모두
가장 높았습니다. 특히 YOLOv8l은 가장 긴 학습 시간이 기록됐지만, 이 비교에서
성능 우위는 확인되지 않았습니다.

학습 시간은 각 실행의 `results.csv` 마지막 행에 기록된 값이므로, GPU·배치 크기
등 실행 환경이 다르면 절대적인 속도 비교 지표로 사용하면 안 됩니다.

## 포함 파일

각 모델 폴더에는 다음 Drive 산출물을 저장했습니다.

- `results.csv`, `results.png`
- `BoxF1_curve.png`, `BoxPR_curve.png`
- `confusion_matrix.png`, `confusion_matrix_normalized.png`
