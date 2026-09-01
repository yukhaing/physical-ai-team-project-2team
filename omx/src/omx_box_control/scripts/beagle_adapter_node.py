#!/usr/bin/env python3
"""Bridge ROS orchestration to the Beagle TCP TriggerServer/StatusClient."""

import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BeagleAdapter(Node):
    """Send box handoff triggers and publish Beagle mission status on ROS."""

    def __init__(self):
        super().__init__('beagle_adapter')
        self.declare_parameter('command_topic', '/beagle/command')
        self.declare_parameter('status_topic', '/beagle/status')
        self.declare_parameter('connection_mode', 'auto')
        self.declare_parameter('trigger_host', '')
        self.declare_parameter('trigger_port', 8765)
        self.declare_parameter('status_bind_host', '0.0.0.0')
        self.declare_parameter('status_port', 9000)
        self.declare_parameter('connect_timeout', 2.0)
        self.declare_parameter('retry_interval', 2.0)
        self.declare_parameter('status_heartbeat_timeout', 3.0)
        # Starting the local mission makes roboid auto-probe serial devices.
        # Keep this opt-in because it can otherwise open OMX's /dev/ttyACM0.
        self.declare_parameter('launch_local_mission', False)
        self.declare_parameter('local_port_name', '')
        self.declare_parameter(
            'local_python',
            '/root/omx_box_project_ws/integration/yeongjin_gui/Beagle_mobile_robot/.venv/bin/python')
        self.declare_parameter(
            'local_mission',
            '/root/omx_box_project_ws/integration/yeongjin_gui/Beagle_mobile_robot/missions/receiving_defect_shuttle copy.py')
        self.declare_parameter(
            'local_output',
            '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/logs/beagle_shuttle.csv')
        self.declare_parameter(
            'local_process_log',
            '/root/omx_box_project_ws/integration/yeongjin_gui/runtime/logs/beagle_reconnect.log')

        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('command_topic').value),
            self.on_command, 10)

        self._stop = threading.Event()
        self._outbound = queue.Queue(maxsize=20)
        self._inbound = queue.Queue(maxsize=100)
        self._queued_jobs = set()
        self._active_job_id = None
        self._mission_in_motion = False
        self._discovered_trigger_host = None
        self._status_server = None
        self._threads = []
        self._last_retry_report = 0.0
        self._status_client_count = 0
        self._status_client_lock = threading.Lock()
        self._mission_status_seen = False
        self._mission_connection_live = False
        self._mission_hardware_connected = False
        self._reconnect_requested = False
        self._last_mission_status_time = None
        self._reconnect_lock = threading.Lock()
        self._local_process = None
        self._local_process_log = None

        mode = self._connection_mode()
        self.get_logger().info(
            f'Beagle connection mode={mode}, trigger host='
            f'{self._trigger_host() or "auto-discovery pending"}')

        self._start_status_server()
        sender = threading.Thread(
            target=self._sender_loop, name='beagle-trigger-sender', daemon=True)
        sender.start()
        self._threads.append(sender)
        self.create_timer(0.05, self._drain_events)
        self.create_timer(0.5, self._check_status_heartbeat)
        self.publish_status(
            'disconnected', None,
            f'waiting for Beagle mission status on '
            f'{self.get_parameter("status_bind_host").value}:'
            f'{self.get_parameter("status_port").value}')

    def publish_status(self, state, job_id=None, detail='', raw=None):
        payload = {'state': state, 'job_id': job_id, 'detail': detail}
        if raw is not None:
            payload['raw'] = raw
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))))

    def _queue_event(self, kind, payload):
        try:
            self._inbound.put_nowait((kind, payload))
        except queue.Full:
            self.get_logger().warning('Dropped Beagle event because the queue is full')

    def _start_status_server(self):
        host = str(self.get_parameter('status_bind_host').value)
        port = int(self.get_parameter('status_port').value)
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen(4)
            server.settimeout(0.5)
        except OSError as error:
            self.get_logger().error(
                f'Unable to listen for Beagle status on {host}:{port}: {error}')
            if 'server' in locals():
                server.close()
            return
        self._status_server = server
        thread = threading.Thread(
            target=self._accept_loop, name='beagle-status-server', daemon=True)
        thread.start()
        self._threads.append(thread)

    def _accept_loop(self):
        while not self._stop.is_set() and self._status_server is not None:
            try:
                client, address = self._status_server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            client.settimeout(0.5)
            with self._status_client_lock:
                self._status_client_count += 1
            self._discovered_trigger_host = address[0]
            # A TCP connection is only a status-channel handshake.  The
            # mission opens this socket before it initializes the physical
            # Beagle, so it must not be presented as a robot connection.
            self._queue_event('transport', {
                'state': 'status_link_connected',
                'detail': f'waiting for Beagle mission heartbeat from {address[0]}:{address[1]}',
            })
            thread = threading.Thread(
                target=self._status_client_loop,
                args=(client, address), daemon=True)
            thread.start()
            self._threads.append(thread)

    def _status_client_loop(self, client, address):
        buffer = ''
        try:
            while not self._stop.is_set():
                try:
                    chunk = client.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk.decode('utf-8', errors='replace')
                if len(buffer) > 1048576:
                    raise ValueError('status input exceeded 1 MiB without a newline')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line)
                    except json.JSONDecodeError as error:
                        self._queue_event('protocol_error', str(error))
                        continue
                    if isinstance(payload, dict):
                        self._queue_event('status', payload)
        except (OSError, ValueError) as error:
            self._queue_event('protocol_error', str(error))
        finally:
            client.close()
            with self._status_client_lock:
                self._status_client_count = max(0, self._status_client_count - 1)
                disconnected = self._status_client_count == 0
            if disconnected:
                self._mission_connection_live = False
                self._mission_hardware_connected = False
                self._mission_status_seen = False
                self._last_mission_status_time = None
                self._queue_event('transport', {
                    'state': 'disconnected',
                    'detail': f'Beagle status disconnected from {address[0]}',
                })

    def _trigger_host(self):
        mode = self._connection_mode()
        if mode == 'local':
            return '127.0.0.1'
        configured = str(self.get_parameter('trigger_host').value).strip()
        return configured or self._discovered_trigger_host

    def _connection_mode(self):
        mode = str(self.get_parameter('connection_mode').value).strip().lower()
        if mode not in ('auto', 'local', 'remote'):
            self.get_logger().warning(
                f'Unknown Beagle connection_mode={mode!r}; using auto')
            return 'auto'
        return mode

    def _sender_loop(self):
        retry = max(0.1, float(self.get_parameter('retry_interval').value))
        timeout = max(0.1, float(self.get_parameter('connect_timeout').value))
        port = int(self.get_parameter('trigger_port').value)
        while not self._stop.is_set():
            try:
                command = self._outbound.get(timeout=0.2)
            except queue.Empty:
                continue
            job_id = command.get('job_id')
            while not self._stop.is_set():
                host = self._trigger_host()
                if not host:
                    mode = self._connection_mode()
                    detail = (
                        'remote mode requires trigger_host or an inbound Beagle '
                        'status connection' if mode == 'remote' else
                        'waiting for Beagle status connection to discover its IP')
                    self._report_retry(
                        'waiting_for_beagle',
                        detail,
                        job_id)
                    self._stop.wait(retry)
                    continue
                try:
                    with socket.create_connection((host, port), timeout=timeout) as sock:
                        sock.sendall((json.dumps(
                            command, ensure_ascii=False,
                            separators=(',', ':')) + '\n').encode('utf-8'))
                    self._queue_event('command_sent', {
                        'job_id': job_id,
                        'host': host,
                        'port': port,
                        'command': command,
                    })
                    break
                except OSError as error:
                    self._report_retry(
                        'connecting',
                        f'Beagle trigger {host}:{port} unavailable: {error}',
                        job_id)
                    self._stop.wait(retry)
            if job_id is not None:
                self._queued_jobs.discard(job_id)

    def _report_retry(self, state, detail, job_id):
        now = time.monotonic()
        if now - self._last_retry_report >= 2.0:
            self._last_retry_report = now
            self._queue_event('transport', {
                'state': state, 'detail': detail, 'job_id': job_id})

    def on_command(self, message):
        try:
            command = json.loads(message.data)
            name = str(command['command'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'Ignored Beagle command: {error}')
            return
        job_id = command.get('job_id')
        if name == 'reconnect':
            self._reconnect_requested = True
            if self._mission_connection_live:
                try:
                    self._outbound.put_nowait({'event': 'reset', 'job_id': job_id})
                except queue.Full:
                    self.publish_status(
                        'failed', job_id, 'Beagle reset queue is full')
                    return
                self.publish_status(
                    'connecting', job_id,
                    'reset queued for Beagle TriggerServer')
                return
            self._request_reconnect(job_id)
            return
        if name in ('box_placed', 'home'):
            if (not self._mission_connection_live or
                    not self._mission_hardware_connected):
                self.publish_status(
                    'disconnected', job_id,
                    'box_placed blocked: physical Beagle is not connected')
                return
            if job_id in self._queued_jobs:
                return
            self._active_job_id = job_id
            self._mission_in_motion = False
            self._queued_jobs.add(job_id)
            payload = {'event': 'box_placed'}
            if job_id is not None:
                payload['job_id'] = job_id
            try:
                self._outbound.put_nowait(payload)
            except queue.Full:
                self._queued_jobs.discard(job_id)
                self.publish_status(
                    'failed', job_id, 'Beagle trigger queue is full')
                return
            self.publish_status(
                'waiting_for_beagle', job_id,
                'box_placed queued for Beagle TriggerServer')
            return
        if name == 'operator_unloaded':
            if (not self._mission_connection_live or
                    not self._mission_hardware_connected):
                self.publish_status(
                    'disconnected', job_id,
                    'operator_unloaded blocked: physical Beagle is not connected')
                return
            if job_id in self._queued_jobs:
                return
            self._queued_jobs.add(job_id)
            payload = {'event': 'operator_unloaded'}
            if job_id is not None:
                payload['job_id'] = job_id
            try:
                self._outbound.put_nowait(payload)
            except queue.Full:
                self._queued_jobs.discard(job_id)
                self.publish_status(
                    'failed', job_id, 'Beagle trigger queue is full')
                return
            self.publish_status(
                'waiting_for_beagle', job_id,
                'operator_unloaded queued for Beagle TriggerServer')
            return
        if name == 'defect_loading':
            self.publish_status(
                'ready', job_id,
                'post-place shuttle mode: Beagle remains at receiving zone')
            return
        if name == "stop":
            payload = {"event": "emergency_stop"}
            if job_id is not None:
                payload["job_id"] = job_id
            try:
                self._outbound.put_nowait(payload)
            except queue.Full:
                self.publish_status(
                    "failed", job_id, "Beagle emergency-stop queue is full")
                return
            self.publish_status(
                "stopping", job_id,
                "emergency_stop queued for Beagle TriggerServer")
            return
        self.publish_status('failed', job_id, f'unsupported command: {name}')

    def _request_reconnect(self, job_id):
        if self._connection_mode() != 'local':
            self.publish_status(
                'connecting', job_id,
                'Waiting for the remote Beagle mission to reconnect')
            return
        if not bool(self.get_parameter('launch_local_mission').value):
            self.publish_status(
                'disconnected', job_id,
                'local Beagle mission launch is disabled; connect Beagle first')
            return
        thread = threading.Thread(
            target=self._restart_local_mission,
            args=(job_id,), name='beagle-local-reconnect', daemon=True)
        thread.start()
        self._threads.append(thread)

    def _restart_local_mission(self, job_id):
        if not self._reconnect_lock.acquire(blocking=False):
            self.publish_status(
                'connecting', job_id,
                'Local Beagle reconnect is already in progress')
            return
        try:
            if self._local_process is not None and self._local_process.poll() is None:
                self.publish_status(
                    'connecting', job_id,
                    'Local Beagle mission is already running; waiting for status')
                return
            if self._trigger_server_available():
                self.publish_status(
                    'connecting', job_id,
                    'Beagle TriggerServer is already running; waiting for status')
                return

            python = Path(str(self.get_parameter('local_python').value))
            mission = Path(str(self.get_parameter('local_mission').value))
            port_name = Path(str(self.get_parameter('local_port_name').value))
            output = Path(str(self.get_parameter('local_output').value))
            process_log = Path(str(self.get_parameter('local_process_log').value))
            if not python.is_file() or not os.access(python, os.X_OK):
                self.publish_status(
                    'failed', job_id, f'Local Beagle Python is unavailable: {python}')
                return
            if not mission.is_file():
                self.publish_status(
                    'failed', job_id, f'Local Beagle mission is unavailable: {mission}')
                return
            if not port_name.exists() or 'Robomation_EXPRESS_RECEIVER' not in port_name.name:
                self.publish_status(
                    'failed', job_id, f'Dedicated Beagle receiver is unavailable: {port_name}')
                return

            output.parent.mkdir(parents=True, exist_ok=True)
            process_log.parent.mkdir(parents=True, exist_ok=True)
            if self._local_process_log is not None:
                self._local_process_log.close()
            self._local_process_log = process_log.open('a', encoding='utf-8')
            command = [
                str(python), str(mission),
                '--port-name', str(port_name),
                '--trigger-port', str(int(self.get_parameter('trigger_port').value)),
                '--status-host', '127.0.0.1',
                '--status-port', str(int(self.get_parameter('status_port').value)),
                '--output', str(output),
            ]
            self._local_process = subprocess.Popen(
                command,
                cwd=str(mission.parent.parent),
                stdout=self._local_process_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.publish_status(
                'connecting', job_id,
                f'Local Beagle mission restarted (pid={self._local_process.pid})')
        except (OSError, ValueError) as error:
            self.publish_status(
                'failed', job_id, f'Local Beagle reconnect failed: {error}')
        finally:
            self._reconnect_lock.release()

    def _trigger_server_available(self):
        timeout = min(0.5, max(0.1, float(self.get_parameter('connect_timeout').value)))
        try:
            with socket.create_connection(
                    ('127.0.0.1', int(self.get_parameter('trigger_port').value)),
                    timeout=timeout):
                return True
        except OSError:
            return False

    def _drain_events(self):
        while True:
            try:
                kind, payload = self._inbound.get_nowait()
            except queue.Empty:
                return
            if kind == 'status':
                self._publish_mission_status(payload)
            elif kind == 'command_sent':
                event = payload.get('command', {}).get('event', 'command')
                if event == 'reset':
                    self.publish_status(
                        'connecting', payload.get('job_id'),
                        f'reset sent to {payload["host"]}:{payload["port"]}')
                else:
                    self.publish_status(
                        'signal_sent', payload.get('job_id'),
                        f'{event} sent to {payload["host"]}:{payload["port"]}')
            elif kind == 'transport':
                self.publish_status(
                    payload['state'], payload.get('job_id', self._active_job_id),
                    payload.get('detail', ''))
            elif kind == 'protocol_error':
                self.get_logger().warning(f'Beagle status protocol error: {payload}')

    def _publish_mission_status(self, payload):
        self._mission_status_seen = True
        self._mission_connection_live = True
        self._last_mission_status_time = time.monotonic()
        hardware = payload.get('hardware_connected')
        # New missions report the physical Roboid connector explicitly.  Keep
        # remote compatibility with older missions that do not include it.
        self._mission_hardware_connected = (
            bool(hardware) if hardware is not None else True)
        text = str(payload.get('status', '')).strip()
        destination = str(payload.get('to', '')).strip()
        location = str(payload.get('at', '')).strip()
        mission_state = str(payload.get('mission_state', '')).strip()
        if not self._mission_hardware_connected:
            state = 'connecting' if self._reconnect_requested else 'disconnected'
        elif self._reconnect_requested and mission_state == 'EMERGENCY_STOP':
            # A stale stopped heartbeat can arrive after reset was queued.
            # Keep showing the requested transition until WAIT_SIGNAL confirms
            # that the Beagle emergency latch has actually cleared.
            state = 'connecting'
        elif text == '대기' or mission_state == 'WAIT_SIGNAL':
            self._reconnect_requested = False
            state = 'idle'
        elif (text == 'defect 존으로 이동중' or
              (text == '출발' and destination == 'defect_zone') or
              mission_state == 'GOTO_DEFECT'):
            state = 'moving_to_defect'
        elif ((text == '도착' and location == 'defect_zone') or
              mission_state == 'DWELL_DEFECT'):
            state = 'defect_arrived'
        elif (text == '대기 존으로 이동중' or
              (text == '출발' and destination == 'receiving_zone') or
              mission_state in ('GOTO_RECEIVING', 'ALIGN_RECEIVING')):
            state = 'returning'
        elif text == '비상정지' or mission_state == 'EMERGENCY_STOP':
            state = 'stopped'
        elif text == 'SENSOR_FAIL' or mission_state == 'SENSOR_FAIL':
            state = 'failed'
        elif text == '도착' and location == 'receiving_zone':
            state = 'idle'
        else:
            state = 'status'
        job_id = payload.get('job_id', self._active_job_id)
        self.publish_status(state, job_id, text or mission_state, payload)
        if state in ('moving_to_defect', 'defect_arrived', 'returning'):
            self._mission_in_motion = True
        elif state == 'idle' and self._mission_in_motion:
            self._mission_in_motion = False
            self._active_job_id = None

    def _check_status_heartbeat(self):
        """Do not retain a stale connection after the Beagle mission stops."""
        if not self._mission_connection_live or self._last_mission_status_time is None:
            return
        elapsed = time.monotonic() - self._last_mission_status_time
        if elapsed <= float(self.get_parameter('status_heartbeat_timeout').value):
            return
        self._mission_connection_live = False
        self._mission_hardware_connected = False
        self.publish_status(
            'disconnected', self._active_job_id,
            f'Beagle mission heartbeat timed out after {elapsed:.1f}s')

    def close(self):
        self._stop.set()
        process = self._local_process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        if self._local_process_log is not None:
            self._local_process_log.close()
            self._local_process_log = None
        if self._status_server is not None:
            self._status_server.close()
            self._status_server = None
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = BeagleAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
