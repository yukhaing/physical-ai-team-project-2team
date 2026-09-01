from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Send a test {"event": "box_placed"} message to scripts/10_shuttle_mission.py's
TriggerServer -- for testing the mission's signal handling before the real
OMX arm is sending it. Run this from a second terminal while
10_shuttle_mission.py is running (--host 127.0.0.1 if on the same machine).

Usage: python scripts\\11_send_trigger.py
"""

import argparse
import json
import socket


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        sock.sendall((json.dumps({"event": "box_placed"}) + "\n").encode("utf-8"))
    print("sent box_placed")


if __name__ == "__main__":
    main()

