from __future__ import annotations

"""Windows에서 보내는 로봇 상태(출발/도착/대기 중)를 실시간으로 보여주는 Ubuntu용 GUI.

common/ 모듈에 의존하지 않는 독립 스크립트입니다 -- Python 표준 라이브러리(socket, json,
threading, tkinter)만 있으면 실행됩니다. 즉 이 저장소를 git clone만 하고, roboid/matplotlib/
numpy 등 나머지 requirements를 설치하지 않아도 이 파일 하나는 그대로 실행됩니다.
Ubuntu에 tkinter가 없다면: sudo apt install python3-tk

동작:
  1) --port(기본 8770)로 TCP 서버를 띄우고, Windows 쪽 15_mission_defect_delivery.py의
     StatusClient(common/comm.py) 연결을 기다립니다.
  2) 줄바꿈으로 구분된 JSON 메시지({"status": "출발", ...})를 받을 때마다 화면을 갱신합니다.

Windows와 Ubuntu가 서로 다른 네트워크에 있으면 (같은 WiFi/공유기가 아니면) 포트포워딩
없이는 서로 접속할 수 없습니다 -- 둘 다 Tailscale 같은 VPN을 설치해서 서로의 tailscale
IP로 접속해야 합니다.

사용법:
  python3 scripts/16_status_dashboard.py                 # 0.0.0.0:8770에서 대기
  python3 scripts/16_status_dashboard.py --port 9000
"""

import argparse
import json
import queue
import socket
import threading
import time
import tkinter as tk

STATUS_COLORS = {
    "출발": "#2C74F5",
    "이동중": "#F5A623",
    "도착": "#3B9C4C",
    "대기 중": "#888888",
}
DEFAULT_COLOR = "#20242C"


def serve(host: str, port: int, message_queue: "queue.Queue[dict]", running: threading.Event) -> None:
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(5)
    server_socket.settimeout(0.5)
    print(f"상태 대시보드 서버 시작: {host}:{port}")

    def handle_client(client: socket.socket, addr) -> None:
        print(f"연결됨: {addr}")
        buffer = ""
        try:
            while running.is_set():
                chunk = client.recv(4096)
                if not chunk:
                    break
                buffer += chunk.decode("utf-8")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        message_queue.put(json.loads(line))
                    except json.JSONDecodeError:
                        print(f"잘못된 JSON 무시: {line!r}")
        except OSError:
            pass
        finally:
            client.close()
            print(f"연결 종료: {addr}")

    try:
        while running.is_set():
            try:
                client, addr = server_socket.accept()
            except OSError:
                continue
            threading.Thread(target=handle_client, args=(client, addr), daemon=True).start()
    finally:
        server_socket.close()


class Dashboard:
    def __init__(self, root: tk.Tk, message_queue: "queue.Queue[dict]") -> None:
        self.queue = message_queue
        self.root = root
        root.title("Beagle Robot Status")
        root.geometry("480x360")
        root.configure(bg="#F4F5F7")

        self.status_label = tk.Label(
            root, text="연결 대기 중...", font=("TkDefaultFont", 30, "bold"),
            fg="white", bg=DEFAULT_COLOR, pady=30,
        )
        self.status_label.pack(fill="x", padx=16, pady=(16, 8))

        self.detail_label = tk.Label(
            root, text="", font=("TkDefaultFont", 11), bg="#F4F5F7", justify="left", anchor="w"
        )
        self.detail_label.pack(fill="x", padx=16)

        tk.Label(root, text="기록", font=("TkDefaultFont", 10, "bold"), bg="#F4F5F7", anchor="w").pack(
            fill="x", padx=16, pady=(12, 0)
        )
        self.log = tk.Listbox(root, font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=16, pady=(4, 16))

        self.poll()

    def poll(self) -> None:
        while True:
            try:
                message = self.queue.get_nowait()
            except queue.Empty:
                break
            self._apply(message)
        self.root.after(100, self.poll)

    def _apply(self, message: dict) -> None:
        status = str(message.get("status", "?"))
        color = STATUS_COLORS.get(status, DEFAULT_COLOR)
        self.status_label.configure(text=status, bg=color)

        ts = message.get("ts")
        time_str = time.strftime("%H:%M:%S", time.localtime(ts)) if ts else time.strftime("%H:%M:%S")
        extra = {k: v for k, v in message.items() if k not in {"status", "ts"}}
        detail = "  ".join(f"{k}={v}" for k, v in extra.items())
        self.detail_label.configure(text=f"마지막 업데이트: {time_str}\n{detail}")

        self.log.insert(0, f"[{time_str}] {status}" + (f"  ({detail})" if detail else ""))
        if self.log.size() > 200:
            self.log.delete(200, tk.END)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8770)
    args = parser.parse_args()

    message_queue: "queue.Queue[dict]" = queue.Queue()
    running = threading.Event()
    running.set()
    server_thread = threading.Thread(
        target=serve, args=(args.host, args.port, message_queue, running), daemon=True
    )
    server_thread.start()

    root = tk.Tk()
    Dashboard(root, message_queue)
    try:
        root.mainloop()
    finally:
        running.clear()


if __name__ == "__main__":
    main()
