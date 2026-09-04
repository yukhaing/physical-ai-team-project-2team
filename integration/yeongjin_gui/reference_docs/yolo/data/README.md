# data/ — All Dataset-Related Files

This folder contains two types of data:

- `roboflow_dataset/` — existing public dataset used as our Stage 1 training base 
- `raw_photos/` — our own photos captured with the C270 camera, used as 
  Stage 2 fine-tuning data to adapt the model to our specific setup 

## Important — Do Not Push Large Files
Actual image files, labels, and datasets should NOT be committed to GitHub 
(too large, will bloat the repo). 

- Keep large files ignored via `.gitignore`
- Store actual data in the shared Google Drive folder instead: [link to be added]
- Only small files (README, data.yaml config) belong in this repo