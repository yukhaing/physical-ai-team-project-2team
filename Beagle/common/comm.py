from __future__ import annotations

import json
import queue
import socket
import threading
import time
import time


class TriggerServer:
    """YOLO/OMX가 보내는 JSON 메시지를 받아 큐에 쌓는 간단한 TCP 서버.

    줄바꿈으로 구분된 JSON 메시지(newline-delimited JSON)를 사용합니다.
    예: {"class": "normal"}\\n  또는  {"event": "box_placed"}\\n

    accept/recv는 블로킹 호출이라 별도 스레드에서 돌리고, 메인 루프(시뮬레이터/로봇
    제어 루프)는 poll()로 큐를 비블로킹으로 확인합니다.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8765) -> None:
        self.host = host
        self.port = port
        self._queue: "queue.Queue[dict]" = queue.Queue()
        self._server_socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False

    def start(self) -> None:
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.host, self.port))
        self._server_socket.listen(5)
        self._server_socket.settimeout(0.5)
        self._running = True
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        print(f"TriggerServer listening on {self.host}:{self.port}")

    def stop(self) -> None:
        self._running = False
        if self._server_socket is not None:
            self._server_socket.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def poll(self) -> list[dict]:
        """큐에 쌓인 메시지를 모두 꺼내 반환합니다 (없으면 빈 리스트)."""
        messages = []
        while True:
            try:
                messages.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return messages

    def _accept_loop(self) -> None:
        while self._running:
            try:
                client, addr = self._server_socket.accept()
            except OSError:
                continue  # timeout(0.5s) 또는 stop()으로 소켓이 닫힘
            threading.Thread(target=self._client_loop, args=(client, addr), daemon=True).start()

    def _client_loop(self, client: socket.socket, addr) -> None:
        print(f"TriggerServer: connected from {addr}")
        buffer = ""
        try:
            while self._running:
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
                        message = json.loads(line)
                    except json.JSONDecodeError:
                        print(f"TriggerServer: bad JSON: {line!r}")
                        continue
                    self._queue.put(message)
        except OSError:
            pass
        finally:
            client.close()
            print(f"TriggerServer: disconnected {addr}")


class StatusClient:
    """로봇 상태(출발/도착/대기 중 등)를 원격 대시보드(Ubuntu GUI)로 보내는 TCP 클라이언트.

    TriggerServer와 반대 방향: 여기서는 로봇 제어 루프가 클라이언트가 됩니다. 대시보드가
    꺼져 있거나 네트워크가 끊겨도 로봇 제어 루프가 멈추면 안 되므로, 실제 연결/전송은
    별도 스레드에서 처리하고 send_status()는 큐에 넣기만 하는 논블로킹 호출입니다.
    연결이 끊기면 백그라운드에서 계속 재연결을 시도합니다.
    """

    def __init__(self, host: str, port: int, *, reconnect_interval: float = 3.0) -> None:
        self.host = host
        self.port = port
        self.reconnect_interval = reconnect_interval
        self._queue: "queue.Queue[dict]" = queue.Queue(maxsize=50)
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def send_status(self, status: str, **extra: object) -> None:
        """상태를 큐에 넣습니다 (논블로킹). 큐가 가득 차면 가장 오래된 것을 버리고
        최신 상태를 우선합니다 -- 대시보드에는 과거 기록보다 지금 상태가 더 중요합니다."""
        message = {"status": status, "ts": time.time(), **extra}
        try:
            self._queue.put_nowait(message)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(message)
            except queue.Full:
                pass

    def _run(self) -> None:
        sock: socket.socket | None = None
        while self._running:
            if sock is None:
                try:
                    sock = socket.create_connection((self.host, self.port), timeout=3.0)
                except OSError:
                    sock = None
                    time.sleep(self.reconnect_interval)
                    continue
            try:
                message = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                sock.sendall((json.dumps(message) + "\n").encode("utf-8"))
            except OSError:
                sock.close()
                sock = None
        if sock is not None:
            sock.close()
