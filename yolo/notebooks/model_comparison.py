"""YOLOv8 s/m/l 모델 크기별 학습 결과 비교.

Google Drive의 각 모델 학습 run 폴더(results.csv)를 읽어
mAP50 / mAP50-95 / Precision / Recall을 비교하고,
model_size_comparison.csv, model_size_comparison.png로 저장한다.

Colab에서 실행: Drive를 마운트한 뒤 아래 runs 딕셔너리 경로를
본인의 학습 run 폴더 경로로 맞추면 된다.
"""

import pandas as pd
import matplotlib.pyplot as plt

runs = {
    "YOLOv8s": "/content/drive/.shortcut-targets-by-id/1dJubw57Wyx5h83vyYX7wvnbYiO7bou-Y/Classifying defective boxes/runs/yolov8s_100e/results.csv",
    "YOLOv8m": "/content/drive/.shortcut-targets-by-id/1dJubw57Wyx5h83vyYX7wvnbYiO7bou-Y/Classifying defective boxes/runs/yolov8m_100e/results.csv",
    "YOLOv8l": "/content/drive/.shortcut-targets-by-id/1dJubw57Wyx5h83vyYX7wvnbYiO7bou-Y/Classifying defective boxes/runs/yolov8l_100e_v2/results.csv",
}

dfs = {}
for name, path in runs.items():
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()
    dfs[name] = df
    print(f"{name}: {len(df)} epochs loaded")

summary = []
for name, df in dfs.items():
    last_row = df.iloc[-1]
    summary.append({
        "Model": name,
        "mAP50": round(last_row["metrics/mAP50(B)"], 4),
        "mAP50-95": round(last_row["metrics/mAP50-95(B)"], 4),
        "Precision": round(last_row["metrics/precision(B)"], 4),
        "Recall": round(last_row["metrics/recall(B)"], 4),
    })

summary_df = pd.DataFrame(summary)
print(summary_df)

fig, axes = plt.subplots(1, 2, figsize=(14, 5))

metric1 = "metrics/mAP50(B)"
metric2 = "metrics/mAP50-95(B)"

for name, df in dfs.items():
    axes[0].plot(df["epoch"], df[metric1], label=name)
    axes[1].plot(df["epoch"], df[metric2], label=name)

axes[0].set_title("mAP50 Comparison")
axes[0].set_xlabel("Epoch")
axes[0].set_ylabel("mAP50")
axes[0].legend()
axes[0].grid(True)

axes[1].set_title("mAP50-95 Comparison")
axes[1].set_xlabel("Epoch")
axes[1].set_ylabel("mAP50-95")
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.savefig("model_size_comparison.png")
plt.show()

summary_df.to_csv("model_size_comparison.csv", index=False)
