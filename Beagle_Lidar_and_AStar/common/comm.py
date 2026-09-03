from __future__ import annotations

import json
import queue
import socket
import threading


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
