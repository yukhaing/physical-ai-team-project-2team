#!/usr/bin/env python3
"""Persist completed sorting jobs while exposing a deliberately minimal GUI log."""

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FAILURE_LABELS = {
    'box_pick_failed': '박스 집기 실패',
    'box_place_failed': '박스 배치 실패',
    'beagle_disconnected': 'Beagle 연결 끊김',
    'beagle_operation_failed': 'Beagle 동작 실패',
    'omx_operation_failed': 'OMX 동작 실패',
    'unload_omx_failed': '하역 OMX 동작 실패',
}


def failure_label(event):
    failure_type = str(event.get('failure_type') or '').strip()
    if failure_type in FAILURE_LABELS:
        return FAILURE_LABELS[failure_type]
    reason = str(event.get('reason') or '').lower()
    if 'beagle' in reason and any(token in reason for token in ('disconnect', 'connection lost')):
        return FAILURE_LABELS['beagle_disconnected']
    if any(token in reason for token in ('pick', 'grasp', 'gripper')):
        return FAILURE_LABELS['box_pick_failed']
    return '작업 실패'


class OperationsLog(Node):
    def __init__(self):
        super().__init__('operations_log')
        self.declare_parameter('event_topic', '/console/job_event')
        self.declare_parameter('recent_log_topic', '/console/recent_log')
        self.declare_parameter('database_path', '/root/omx_box_project_ws/logs/operations.sqlite3')
        path = Path(str(self.get_parameter('database_path').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.execute('''CREATE TABLE IF NOT EXISTS operations (
            job_id TEXT PRIMARY KEY, completed_at TEXT NOT NULL, classification TEXT NOT NULL,
            confidence REAL, pixel_x REAL, pixel_y REAL, robot_x REAL, robot_y REAL,
            beagle_destination TEXT, omx_result TEXT NOT NULL, beagle_return_result TEXT
        )''')
        columns = {
            row[1] for row in self.db.execute('PRAGMA table_info(operations)')}
        if 'failure_type' not in columns:
            self.db.execute('ALTER TABLE operations ADD COLUMN failure_type TEXT')
        if 'failure_reason' not in columns:
            self.db.execute('ALTER TABLE operations ADD COLUMN failure_reason TEXT')
        self.db.commit()
        self.recent_pub = self.create_publisher(
            String, str(self.get_parameter('recent_log_topic').value), 10)
        self.create_subscription(String, str(self.get_parameter('event_topic').value), self.on_event, 10)

    def on_event(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        event_name = event.get('event')
        if (event_name in ('awaiting_operator_unload', 'return_completed') and
                not event.get('job_id')):
            self.get_logger().warning(
                f'Ignoring {event_name} event without a job_id')
            return
        if event_name == 'awaiting_operator_unload':
            timestamp = datetime.now(timezone.utc).astimezone().strftime(
                '%Y-%m-%d %H:%M:%S')
            label = 'defect'
            self.db.execute('''INSERT OR REPLACE INTO operations
                (job_id, completed_at, classification, confidence, pixel_x, pixel_y,
                 robot_x, robot_y, beagle_destination, omx_result, beagle_return_result)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''', (
                    event['job_id'], timestamp, label, event.get('confidence'), event.get('x'),
                    event.get('y'), event.get('robot_x'), event.get('robot_y'),
                    'defect_loading', 'placed_waiting_operator', 'pending'))
            self.db.commit()
        elif event_name == 'cycle_failed':
            timestamp = datetime.now(timezone.utc).astimezone().strftime(
                '%Y-%m-%d %H:%M:%S')
            self.db.execute('''INSERT OR REPLACE INTO operations
                (job_id, completed_at, classification, confidence, pixel_x, pixel_y,
                 robot_x, robot_y, beagle_destination, omx_result, beagle_return_result,
                 failure_type, failure_reason)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''', (
                    event.get('job_id'), timestamp, 'defect', event.get('confidence'),
                    event.get('x'), event.get('y'), event.get('robot_x'),
                    event.get('robot_y'), 'defect_loading',
                    event.get('failure_type') or 'failed', 'not_started',
                    event.get('failure_type'), event.get('reason')))
            self.db.commit()
            self.recent_pub.publish(String(data=json.dumps({
                'time': timestamp[-8:], 'status': failure_label(event)})))
        elif event_name == 'return_completed':
            row = self.db.execute(
                'SELECT beagle_return_result FROM operations WHERE job_id = ?',
                (event['job_id'],)).fetchone()
            # Ignore a stale/duplicate status.  This also guarantees that one
            # physical return produces exactly one GUI success entry.
            if row is None or row[0] == 'completed':
                return
            timestamp = datetime.now(timezone.utc).astimezone().strftime(
                '%Y-%m-%d %H:%M:%S')
            self.db.execute(
                '''UPDATE operations
                   SET completed_at = ?, beagle_return_result = ?
                   WHERE job_id = ?''',
                (timestamp, 'completed', event['job_id']))
            self.db.commit()
            # Keep the operator view deliberately compact and Korean-only.
            self.recent_pub.publish(String(data=json.dumps({
                'time': timestamp[-8:], 'status': '성공'})))


def main(args=None):
    rclpy.init(args=args)
    node = OperationsLog()
    try:
        rclpy.spin(node)
    finally:
        node.db.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
