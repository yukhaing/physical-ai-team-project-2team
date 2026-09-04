# Beagle LiDAR + A* — Receiving/Defect Shuttle

**This is the final version of the Beagle shuttle mission** — the one used
for the actual submission/demo. `Beagle/` and `Beagle_mobile_robot/` are
earlier implementations kept only for development history; they are
superseded by this folder and should not be used.

Beagle robot mission: idle at a **receiving zone**, wait for a `box_placed`
signal from the OMX arm, drive to a **defect zone** using A* + Pure Pursuit,
align precisely against a LiDAR map, wait for a `box_picked` signal once the
OMX arm has picked the box, then drive back and wait for the next signal.

Unlike `Beagle_mobile_robot` (dead-reckoning only), this version continuously
corrects its position against a real LiDAR point-cloud map while driving, and
does a final precision alignment pass (position + heading) at each zone
before considering the leg done. Real-hardware-only, no `--dry-run` mode.

## Layout

Work area: 0.86m x 0.70m.

- Receiving zone: (0.35m, 0.345m), heading 0°
- Defect zone: (0.72m, 0.135m), heading 180°

All of this lives in
[`config/course_config.json`](config/course_config.json), not hardcoded in
the mission script.

## Folder layout

```text
common/      Hardware wrapper, navigation, and precision-alignment logic
             (hw.py, navigate.py, dock.py, mapping.py, scan_align.py, comm.py)
config/      course_config.json — zone positions/headings, boundary size
data/        map_points.json (LiDAR point-cloud map), obstacle_map.json,
             per-zone reference scans -- built once via the setup scripts below
scripts/     Numbered setup/calibration scripts + 10_shuttle_mission.py (the mission itself)
simulator/   Simulated scene, used by early sim-only scripts (untested/unused
             in the current real-hardware workflow)
```

## Install

```powershell
python -m venv venv
.\venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`roboid` (the real-hardware driver) may need to come from Robomation's
installer rather than PyPI.

The main mission script (`10_shuttle_mission.py`) is real-hardware only --
there is no `--dry-run` mode.

## Run on the real robot

One-time setup (only needed for a new robot/room):

```powershell
python scripts\00_connection_check.py
python scripts\00b_check_drive_direction.py
python scripts\03_calibrate_and_realign.py --zone receiving --calibrate
python scripts\03_calibrate_and_realign.py --zone defect --calibrate
python scripts\05_map_obstacles.py
python scripts\08_build_map.py
```

Then, with the robot physically placed at the receiving zone (position +
heading matching `course_config.json`):

```powershell
python scripts\10_shuttle_mission.py
```

Trigger signals from a second terminal (or from the OMX side) via TCP:

```powershell
python scripts\11_send_trigger.py --event box_placed
python scripts\11_send_trigger.py --event box_picked
```

Useful flags on `10_shuttle_mission.py`: `--cycles N` to stop after N round
trips, `--dynamic-obstacles` to react to obstacles that appear mid-drive
(off by default, re-verify before relying on it), `--align`/`--align-heading`
to fall back to older alignment methods for comparison.

## Results

Physical setup (receiving zone top, defect zone bottom, OMX arm and boxes visible):

![physical setup](../photos/beagle_shuttle_결과/Beagle_실제환경사진1.jpg)

10 consecutive round-trip cycles completed without stopping:

![10 cycle success](../photos/beagle_shuttle_결과/Beagle_실행결과_10cycle.png)

OMX-Beagle TCP signal exchange (`box_placed` / `box_picked`):

![omx to beagle signal](../photos/beagle_shuttle_결과/Beagle_omx_to_beagle_신호.png)

A known-limitation case: heading search failed to converge within `max_iters`
at the defect zone (see Notes below):

![alignment failure case](../photos/beagle_shuttle_결과/Beagle_alignment실패결과.png)

More run logs and setup photos: [`../photos/beagle_shuttle_결과/`](../photos/beagle_shuttle_결과/)

## Notes

- Pose while driving is continuous odometry (encoder + gyro dead-reckoning),
  corrected periodically against the LiDAR map (`drive_with_localization()`
  in `common/navigate.py`) -- this gets the robot close, not exact.
- On arrival, `find_pose_via_map()` (`common/dock.py`) does a separate,
  tighter position+heading alignment pass against the same map before the
  leg is considered done (position error < 2cm, heading error < 6.5° by
  default). If it can't converge within `max_iters` (12), the mission stops
  rather than continuing from an unverified pose.
- Alignment iteration count varies run to run (commonly 3-9) and can
  occasionally take longer near ambiguous headings (e.g. near the defect
  zone, where the OMX arm affects the scan match) -- still open for further
  tuning.
- Confirmed on real hardware: 10 consecutive round-trip cycles completed
  without stopping.
