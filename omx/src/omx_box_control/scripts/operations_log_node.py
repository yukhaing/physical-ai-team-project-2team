#!/usr/bin/env python3
"""Persist completed sorting jobs while exposing a deliberately minimal GUI log."""

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        self.db.commit()
        self.recent_pub = self.create_publisher(
            String, str(self.get_parameter('recent_log_topic').value), 10)
        self.create_subscription(String, str(self.get_parameter('event_topic').value), self.on_event, 10)

    def on_event(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if event.get('event') == 'awaiting_operator_unload':
            timestamp = datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')
            label = 'defect'
            self.db.execute('''INSERT OR REPLACE INTO operations
                (job_id, completed_at, classification, confidence, pixel_x, pixel_y,
                 robot_x, robot_y, beagle_destination, omx_result, beagle_return_result)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''', (
                    event['job_id'], timestamp, label, event.get('confidence'), event.get('x'),
                    event.get('y'), event.get('robot_x'), event.get('robot_y'),
                    'defect_loading', 'placed_waiting_operator', 'pending'))
            self.db.commit()
            # GUI intentionally receives only these two fields.
            self.recent_pub.publish(String(data=json.dumps({
                'completed_at': timestamp, 'classification': 'defect awaiting operator unload'})))
        elif event.get('event') == 'return_completed':
            self.db.execute('UPDATE operations SET beagle_return_result = ? WHERE job_id = ?',
                            ('completed', event['job_id']))
            self.db.commit()


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
