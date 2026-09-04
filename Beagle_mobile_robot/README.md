# Beagle Mobile Robot — Receiving/Defect Shuttle

> **Superseded**: this is an earlier implementation, kept for development
> history. The final version used for submission/demo is
> [`Beagle_Lidar_and_AStar/`](../Beagle_Lidar_and_AStar/README.md).

Beagle robot mission: idle at a **receiving zone**, wait for a "box placed"
signal from the OMX arm, drive to a **defect zone**, wait 5 seconds, drive
back, and wait for the next signal. While driving, a reactive layer routes
around obstacles instead of stopping the mission.

## Layout

Work area: 90cm x 70cm.

- Receiving zone: (36cm, 37cm)
- Defect zone: (75cm, 12cm)

All of this — plus lidar thresholds, speeds, and avoidance tuning — lives in
[`config/course_config.json`](config/course_config.json), not hardcoded in
the mission script.

## Folder layout

```text
common/      Shared robot/geometry/lidar/motion helpers (SafeBeagle wrapper, dry-run mock)
config/      course_config.json — all tunable mission parameters
missions/    receiving_defect_shuttle.py — the mission itself
```

## Install

```powershell
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`roboid` (the real-hardware driver) may need to come from Robomation's
installer rather than PyPI — `--dry-run` works without it either way.

## Try it without hardware

```powershell
python missions\receiving_defect_shuttle.py --dry-run --cycles 2
```

This uses the built-in `shuttle` mock scene (90x70cm room with an obstacle
placed on the direct line between the two zones, to exercise avoidance) and
auto-triggers the box-placed signal a few seconds into each wait so the
mission runs unattended.

## Run on the real robot

1. Confirm connectivity first, wheels not moving:
   ```powershell
   python -c "from common.robot import SafeBeagle; b = SafeBeagle(); print(b.battery_state()); b.stop()"
   ```
2. Physically mark the 90x70cm work area and place the robot at (36, 37)cm
   facing the direction you want to count as heading 0.
3. Test with wheels off the ground first, then at low speed:
   ```powershell
   python missions\receiving_defect_shuttle.py --cycles 1
   ```
4. Trigger the box-placed signal from wherever the OMX side runs -- a TCP
   client sending newline-delimited JSON to `--trigger-port` (default 8765):
   ```powershell
   python -c "import socket; s=socket.create_connection(('localhost', 8765)); s.sendall(b'{\"event\": \"box_placed\"}\n')"
   ```
5. Once a single cycle looks right, drop `--cycles` to run continuously
   until Ctrl+C.

Other useful flags: `--status-host`/`--status-port` to stream status to a
dashboard, `--visualize` for a live top-down plot, `--output` for the CSV
mission log path (see `missions/receiving_defect_shuttle.py --help` for the
full list).

## Notes

- Odometry is dead-reckoned from commanded wheel speed + gyro (no encoders,
  no SLAM correction) — expect some drift over many cycles.
- The gyro bias is calibrated once at mission start; the robot must be
  completely still for that moment.
- The `lidar.*`, `avoidance.*`, and `navigation.*` values in
  `config/course_config.json` were tuned against the mock scene, not real
  hardware — expect to retune them after the first real run.
