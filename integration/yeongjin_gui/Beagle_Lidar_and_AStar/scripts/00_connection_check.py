from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Connection check only -- never calls hw.wheels(), so the robot cannot move.
Run this first, before any script that turns the wheels, to confirm USB/LiDAR
are actually working."""

import argparse
import time

from common.hw import Hardware

DURATION_S = 5.0
NUM_RAYS = 12  # coarse (30deg steps) -- just enough to sanity-check the sensor is alive


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=DURATION_S)
    args = parser.parse_args()

    hw = Hardware()
    try:
        print(f"battery={hw.battery_state()} signal={hw.signal_strength()}")

        hw.start_lidar()
        print("LiDAR ready. Streaming scan for", args.duration, "s (wheels never move)...")

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            scan_m = hw.scan(num_rays=NUM_RAYS)
            rounded_cm = [round(v * 100.0) for v in scan_m]
            valid = sum(1 for v in scan_m if v < 4.9)
            print(f"scan(cm) front->ccw = {rounded_cm}  valid={valid}/{NUM_RAYS}")
            time.sleep(0.3)
    finally:
        hw.stop()

    print("Connection check done (wheels never moved).")


if __name__ == "__main__":
    main()

