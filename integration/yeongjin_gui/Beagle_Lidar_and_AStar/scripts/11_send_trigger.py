from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Send a test {"event": <event>} message to scripts/10_shuttle_mission.py's
TriggerServer -- for testing the mission's signal handling before the real
OMX arm is sending it. Run this from a second terminal while
10_shuttle_mission.py is running (--host 127.0.0.1 if on the same machine).

scripts/10_shuttle_mission.py listens for two events on the same port:
  box_placed  -- at receiving, starts GOTO_DEFECT (WAIT_SIGNAL)
  box_picked  -- at defect, after DWELL_DEFECT, starts GOTO_RECEIVING (WAIT_PICKED)

Usage: python scripts\\11_send_trigger.py [--event box_placed|box_picked]
"""

import argparse
import json
import socket


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--event", default="box_placed",
                         help="Event name to send (default: box_placed). Use box_picked to "
                              "release the mission from its WAIT_PICKED stage at defect.")
    args = parser.parse_args()

    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        sock.sendall((json.dumps({"event": args.event}) + "\n").encode("utf-8"))
    print(f"sent {args.event}")


if __name__ == "__main__":
    main()

