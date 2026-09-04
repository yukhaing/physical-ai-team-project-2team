from __future__ import annotations

import json
import queue
import socket
import threading
import time


class TriggerServer:
    """Minimal TCP server for the OMX arm's newline-delimited JSON signal,
    e.g. {"event": "box_placed"}\\n.

    Two background threads do the waiting so the mission loop never blocks:
    one thread loops on accept(), checking every 0.5s for a new connection
    (so it can stop cleanly); a second thread per connected client waits on
    recv() for that client's data. The mission loop just calls poll() --
    returns immediately -- to grab whatever messages have arrived so far.
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
                continue  # 0.5s accept timeout, or stop() closed the socket
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
    """Queue status without ever blocking navigation on a GUI connection."""

    def __init__(self, host: str, port: int, reconnect_interval: float = 2.0) -> None:
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
                # Requeue the current state; send_status retains newest-first
                # behavior if the bounded queue is already full.
                status = str(message.pop("status", "status"))
                message.pop("ts", None)
                self.send_status(status, **message)
        if sock is not None:
            sock.close()

