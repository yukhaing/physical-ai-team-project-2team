from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""TriggerServer로 테스트 메시지를 보냅니다. 실제 YOLO/OMX 코드가 준비되기 전,
Mission이 신호를 올바르게 받는지 확인하는 용도입니다.

사용 예:
  python scripts\\05_send_trigger.py --class normal
  python scripts\\05_send_trigger.py --class defect
  python scripts\\05_send_trigger.py --box-placed
"""

import argparse
import json
import socket


def send(host: str, port: int, message: dict) -> None:
    with socket.create_connection((host, port), timeout=5.0) as sock:
        sock.sendall((json.dumps(message) + "\n").encode("utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--class", dest="box_class", choices=["normal", "defect"])
    group.add_argument("--box-placed", action="store_true")
    args = parser.parse_args()

    if args.box_class:
        send(args.host, args.port, {"class": args.box_class})
    else:
        send(args.host, args.port, {"event": "box_placed"})
    print("sent")


if __name__ == "__main__":
    main()
