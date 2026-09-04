# Beagle Mobile Robot — legacy runtime and support code

> 현재 통합 GUI가 실행하는 운행 로직은
> [`../Beagle_Lidar_and_AStar/scripts/10_shuttle_mission.py`](../Beagle_Lidar_and_AStar/scripts/10_shuttle_mission.py)다.
> 이 디렉터리는 해당 미션이 사용하는 `.venv`와 기존 지원 코드를 제공한다. 아래 내용은
> 과거 `missions/receiving_defect_shuttle.py` 단독 실행 설명이다.

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
4. For a standalone test, trigger the same TCP signal used by the OMX adapter:
   ```powershell
   $client = [Net.Sockets.TcpClient]::new("127.0.0.1", 8765)
   $writer = [IO.StreamWriter]::new($client.GetStream())
   $writer.WriteLine('{"event":"box_placed"}'); $writer.Flush(); $client.Close()
   ```
5. Once a single cycle looks right, drop `--cycles` to run continuously
   until Ctrl+C.

## OMX GUI connection

The mission and GUI communicate with newline-delimited JSON over two TCP
ports. The protocol is identical for local and two-PC operation, so changing
deployment does not change the mission or OMX state machine.

Run Beagle control on the same Ubuntu PC:

```bash
python3 missions/receiving_defect_shuttle.py \
  --trigger-port 8765 --status-host 127.0.0.1 --status-port 9000
```

Run Beagle control on another PC (replace the address with the Ubuntu GUI PC):

```powershell
python missions\receiving_defect_shuttle.py `
  --trigger-port 8765 --status-host <GUI_PC_IP> --status-port 9000
```

The GUI adapter listens on TCP 9000 and sends `box_placed` to the mission's
TCP 8765. Allow inbound TCP 8765 on the Beagle-control PC and inbound TCP 9000
on the GUI PC. Start this mission before pressing `가동`; its one-second
heartbeat tells the GUI that Beagle is waiting at the receiving zone.

To temporarily operate OMX without Beagle, set `bypass_beagle: true` in
`omx/src/omx_box_control/config/console.yaml`. This keeps the Beagle transport
optional and removable without changing the OMX pick/place implementation.

## Notes

- Odometry is dead-reckoned from commanded wheel speed + gyro (no encoders,
  no SLAM correction) — expect some drift over many cycles.
- The gyro bias is calibrated once at mission start; the robot must be
  completely still for that moment.
- The `lidar.*`, `avoidance.*`, and `navigation.*` values in
  `config/course_config.json` were tuned against the mock scene, not real
  hardware — expect to retune them after the first real run.
