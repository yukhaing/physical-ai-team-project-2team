from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. Safety diagnostic for common/hw.py's drive_forward() --
run this BEFORE trusting any script that calls it (scripts/04_find_pose.py).

Commands a short, low-speed forward pulse (1s at 10%) and prints the raw
encoder delta for each wheel. Watch the robot with your own eyes at the same
time: if it visibly moves FORWARD but the printed delta is negative (or it
visibly moves BACKWARD but the delta is positive), drive_forward()'s
`forward_percent` sign is backward for this robot -- pass a negative
forward_percent to drive_forward() calls (see common/dock.py's find_pose(),
POSITION_DRIVE_PERCENT) until this checks out positive-forward.

Give the robot ~15-20cm of clear space ahead before running this.
"""

import time

from common.hw import Hardware

TEST_PERCENT = 10.0
TEST_DURATION_S = 1.0


def main() -> None:
    hw = Hardware()
    try:
        hw.encoder_delta_m()  # reset baseline
        print(f"Driving both wheels at {TEST_PERCENT:.0f}% for {TEST_DURATION_S:.1f}s -- WATCH THE ROBOT NOW.")
        hw.wheels(TEST_PERCENT, TEST_PERCENT)
        time.sleep(TEST_DURATION_S)
    finally:
        hw.stop()

    d_left, d_right = hw.encoder_delta_m()
    print(f"encoder delta: left={d_left * 100:+.2f}cm right={d_right * 100:+.2f}cm")
    if d_left > 0 and d_right > 0:
        print("Encoders say FORWARD. Did the robot actually move forward? "
              "If yes, forward_percent=+10 is correct as-is. If it actually moved "
              "backward, the encoder sign itself is flipped (different issue -- report this).")
    elif d_left < 0 and d_right < 0:
        print("Encoders say BACKWARD even though forward_percent was positive. "
              "If the robot actually moved forward, pass a NEGATIVE forward_percent "
              "to drive_forward() calls. If it actually moved backward too, everything "
              "is consistent (this robot's positive percent = backward).")
    else:
        print("Left/right encoder signs disagree -- the robot probably didn't drive "
              "straight (wheels imbalanced or one encoder misbehaving). Investigate "
              "before trusting drive_forward().")


if __name__ == "__main__":
    main()

