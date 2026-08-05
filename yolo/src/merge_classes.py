import os

def remap_classes(labels_dir, class_mapping):
    """
    labels_dir 안의 모든 label 파일을 열어서, 
    class_mapping에 따라 class index를 바꿔서 다시 저장함
    """
    for filename in os.listdir(labels_dir):
        if not filename.endswith('.txt'):
            continue
        filepath = os.path.join(labels_dir, filename)
        
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        new_lines = []
        for line in lines:
            parts = line.strip().split()
            if not parts:
                continue
            old_class = int(parts[0])
            new_class = class_mapping[old_class]
            new_line = f"{new_class} {' '.join(parts[1:])}\n"
            new_lines.append(new_line)
        
        with open(filepath, 'w') as f:
            f.writelines(new_lines)


if __name__ == "__main__":
    # 0=carton box → 0 (Box)
    # 1=cracked, 2=opened, 3=wet → 1 (Damaged Box)
    class_mapping = {0: 0, 1: 1, 2: 1, 3: 1}
    
    dataset_dir = "yolo/data/processed/carton_2class"
    
    for split in ['train', 'valid', 'test']:
        labels_path = os.path.join(dataset_dir, split, 'labels')
        remap_classes(labels_path, class_mapping)
        print(f"{split}: class remapping complete")