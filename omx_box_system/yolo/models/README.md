# models/ — Trained Model Weights

This folder is where trained YOLOv8 model weight files (.pt files) would go locally.

## Files (to be added)
- `best.pt` — our best-performing trained model, used for actual detection

## Important — Do Not Push Model Files to GitHub
Model weight files are large (often 20-200+ MB) and should NOT be committed to GitHub.

- These files are already excluded via `.gitignore`
- Actual trained model weights are stored here instead: [Google Drive link — add once created]
- After each training run, upload the resulting `best.pt` to that Drive folder
  and update the link/date here so teammates know which is the latest version