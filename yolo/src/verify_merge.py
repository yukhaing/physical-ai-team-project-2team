import os
from collections import Counter

def count_classes_in_split(labels_dir):
    class_counts = Counter()
    total_files = 0
    
    for filename in os.listdir(labels_dir):
        if not filename.endswith('.txt'):
            continue
        total_files += 1
        filepath = os.path.join(labels_dir, filename)
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if parts:
                    class_idx = int(parts[0])
                    class_counts[class_idx] += 1
    
    return class_counts, total_files


if __name__ == "__main__":
    class_names = ['Box', 'Damaged Box']
    dataset_dir = "yolo/data/processed/carton_2class"
    
    for split in ['train', 'valid', 'test']:
        labels_path = os.path.join(dataset_dir, split, 'labels')
        counts, num_files = count_classes_in_split(labels_path)
        
        print(f"\n{'='*50}")
        print(f"{split.upper()} — {num_files} label files")
        print(f"{'='*50}")
        for idx, name in enumerate(class_names):
            print(f"  [{idx}] {name}: {counts.get(idx, 0)} instances")