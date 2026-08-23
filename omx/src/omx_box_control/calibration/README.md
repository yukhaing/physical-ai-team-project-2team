# Camera calibration profiles

This directory is the integration boundary for calibration work maintained on
`minseo-dev`. The control nodes do not contain camera-specific measurements.

- `profiles/` contains versioned, read-only seed profiles committed to Git.
- The active calibration is copied to `runtime/calibration/active.yaml` at first
  launch and is the only file changed by interactive recalibration.

To adopt a newer profile, add it under `profiles/`, update
`config/homography_target.yaml`, and remove the runtime `active.yaml` before the
next launch. This keeps calibration updates independent from the operator GUI,
YOLO, and Beagle workflow.
