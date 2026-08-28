#!/usr/bin/env python3
"""Qt operator console for the OMX/Beagle sorting workflow."""

import json
import sys
import threading
import time

import numpy as np
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QListWidget,
                             QPushButton, QTableWidget, QTableWidgetItem,
                             QVBoxLayout, QWidget)
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VideoLabel(QLabel):
    clicked = pyqtSignal(int, int)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.pixmap() and self.width() and self.height():
            self.clicked.emit(event.x(), event.y())


class ConsoleSignals(QObject):
    frame_ready = pyqtSignal(QImage)
    status_ready = pyqtSignal(str)
    beagle_status_ready = pyqtSignal(str, str, str, str)
    log_ready = pyqtSignal(str, str)


class ConsoleNode(Node):

    def __init__(self):
        super().__init__('omx_console')
        self.signals = ConsoleSignals()
        self.detections = []
        self.frame_size = (1, 1)
        self.command_pub = self.create_publisher(String, '/console/command', 10)
        self.pixel_pub = self.create_publisher(String, '/console/select_pixel', 10)
        self.selection_pub = self.create_publisher(String, '/console/selected_box', 10)
        self.create_subscription(Image, '/console/annotated_image', self.on_image, 2)
        self.create_subscription(String, '/console/detections', self.on_detections, 10)
        self.create_subscription(String, '/console/status', lambda msg: self.signals.status_ready.emit(msg.data), 10)
        self.create_subscription(String, '/beagle/status', self.on_beagle_status, 10)
        self.create_subscription(String, '/console/recent_log', self.on_log, 10)

    def command(self, name):
        self.command_pub.publish(String(data=name))

    def on_image(self, message):
        if message.encoding not in ('bgr8', 'rgb8'):
            return
        frame = np.frombuffer(message.data, dtype=np.uint8).reshape((message.height, message.width, 3))
        if message.encoding == 'bgr8':
            frame = frame[:, :, ::-1]
        self.frame_size = (message.width, message.height)
        self.signals.frame_ready.emit(QImage(frame.copy().data, message.width, message.height,
                                             message.width * 3, QImage.Format_RGB888).copy())

    def on_detections(self, message):
        try:
            self.detections = json.loads(message.data).get('detections', [])
        except json.JSONDecodeError:
            self.detections = []

    def on_log(self, message):
        try:
            row = json.loads(message.data)
            timestamp = row.get('time')
            status = row.get('status')
            # Backward compatibility for a log publisher from an older build.
            if timestamp is None and row.get('completed_at'):
                value = str(row['completed_at']).replace('T', ' ')
                timestamp = value[11:19] if len(value) >= 19 else value
            if status is None and row.get('classification'):
                status = ('실패' if 'fail' in str(row['classification']).lower()
                          else '성공')
            if timestamp and status:
                self.signals.log_ready.emit(str(timestamp), status)
        except (TypeError, json.JSONDecodeError):
            pass

    def on_beagle_status(self, message):
        try:
            payload = json.loads(message.data)
            state = str(payload.get('state', 'status'))
            raw = payload.get('raw') if isinstance(payload.get('raw'), dict) else {}
            labels = {
                'adapter_ready': '상태 수신 대기',
                'connected': '연결됨',
                'disconnected': '연결 끊김',
                'idle': '대기 중',
                'moving_to_defect': '이동중 · 불량 구역',
                'defect_arrived': '도착 · 불량 구역',
                'returning': '이동/정렬중 · 수령 구역',
                'signal_sent': '적재 신호 전송',
                'waiting_for_beagle': '연결 대기 중',
                'connecting': '연결 재시도',
                'reconnecting': '재연결 중',
                'failed': '오류',
                'stop_unsupported': '원격 정지 미지원',
            }
            label = labels.get(state, str(raw.get('status') or payload.get('detail') or state))
            extras = []
            if raw.get('to'):
                extras.append(f"to={raw['to']}")
            if raw.get('at'):
                extras.append(f"at={raw['at']}")
            detail = '  '.join(extras) or str(payload.get('detail', ''))
            timestamp = raw.get('ts')
            clock = (time.strftime('%H:%M:%S', time.localtime(float(timestamp)))
                     if timestamp else time.strftime('%H:%M:%S'))
            self.signals.beagle_status_ready.emit(state, label, detail, clock)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def select_pixel(self, x, y):
        candidates = []
        for detection in self.detections:
            if (detection.get('class') != 'defect' or
                    not detection.get('joint5_stable')):
                continue
            dx, dy = detection['center_x'] - x, detection['center_y'] - y
            candidates.append((dx * dx + dy * dy, detection))
        if not candidates:
            self.signals.status_ready.emit(
                '선택 거부: 안정화된 defect 각도가 아직 없습니다.')
            return
        _distance, detection = min(candidates, key=lambda item: item[0])
        selection = dict(detection, x=x, y=y)
        self.pixel_pub.publish(String(data=json.dumps(selection)))
        self.selection_pub.publish(String(data=json.dumps(selection)))
        self.signals.status_ready.emit(f"선택됨: {detection['class']} ({detection['confidence']:.2f})")


class ConsoleWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('OMX 통합 관제')
        self.resize(1180, 720)
        self.setWindowTitle('OMX 불량 박스 이송 통합 관제')
        self.video = VideoLabel('카메라 대기 중')
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(760, 560)
        self.video.setStyleSheet('background:#111;color:#ddd;')
        self.video.clicked.connect(self.on_click)
        self.status = QLabel('DISABLED')
        self.status.setWordWrap(True)
        self.beagle_status = QLabel('상태 수신 대기')
        self.beagle_status.setAlignment(Qt.AlignCenter)
        self.beagle_status.setWordWrap(True)
        self.beagle_status.setStyleSheet(
            'background:#20242C;color:white;font-size:18px;font-weight:bold;'
            'padding:14px;border-radius:4px;')
        self.beagle_detail = QLabel('')
        self.beagle_detail.setWordWrap(True)
        self.beagle_log = QListWidget()
        self.beagle_log.setMaximumHeight(120)
        self._last_beagle_log = None
        self.operator_unloaded_button = QPushButton('하역 완료')
        self.operator_unloaded_button.setEnabled(False)
        self.run_button = QPushButton('가동')
        self.stop_button = QPushButton('정지')
        self.estop_button = QPushButton('비상정지 (소프트웨어)')
        self.reset_button = QPushButton('재설정')
        self.estop_button.setStyleSheet('background:#c62828;color:white;font-weight:bold;min-height:46px;')
        self.log = QTableWidget(0, 2)
        self.log.setHorizontalHeaderLabels(['시간', '상태'])
        self.log.horizontalHeader().setStretchLastSection(True)
        side = QVBoxLayout()
        side.addWidget(QLabel('운전 상태'))
        side.addWidget(self.status)
        for button in (self.run_button, self.stop_button, self.estop_button,
                       self.reset_button):
            side.addWidget(button)
        side.addWidget(self.operator_unloaded_button)
        side.addWidget(QLabel('Beagle 상태'))
        side.addWidget(self.beagle_status)
        side.addWidget(self.beagle_detail)
        side.addWidget(self.beagle_log)
        side.addWidget(QLabel('작업 로그'))
        side.addWidget(self.log, 1)
        layout = QHBoxLayout(self)
        layout.addWidget(self.video, 3)
        layout.addLayout(side, 1)
        self.run_button.clicked.connect(lambda: self.node.command('enable'))
        self.stop_button.clicked.connect(lambda: self.node.command('stop'))
        self.estop_button.clicked.connect(lambda: self.node.command('estop'))
        self.operator_unloaded_button.clicked.connect(self.complete_unload)
        self.reset_button.clicked.connect(lambda: self.node.command('reset'))
        self.node.signals.frame_ready.connect(self.show_frame)
        self.node.signals.status_ready.connect(self.update_operation_status)
        self.node.signals.beagle_status_ready.connect(self.update_beagle_status)
        self.node.signals.log_ready.connect(self.add_log)

    def update_operation_status(self, text):
        self.status.setText(text)
        if text.startswith('BEAGLE_DEFECT_ARRIVED:'):
            self.operator_unloaded_button.setEnabled(True)
        elif text.startswith(('UNLOAD_COMPLETE:', 'UNLOAD_SIGNAL_SENT:',
                              'READY:', 'DISABLED:',
                              'RESET', 'RECOVERING:', 'BEAGLE_HOME:')):
            self.operator_unloaded_button.setEnabled(False)

    def complete_unload(self):
        self.operator_unloaded_button.setEnabled(False)
        self.node.command('operator_unloaded')

    def update_beagle_status(self, state, label, detail, timestamp):
        colors = {
            'connected': '#2C74F5',
            'signal_sent': '#2C74F5',
            'moving_to_defect': '#F5A623',
            'returning': '#F5A623',
            'defect_arrived': '#3B9C4C',
            'idle': '#888888',
            'disconnected': '#B00020',
            'reconnecting': '#2C74F5',
            'failed': '#B00020',
            'stop_unsupported': '#B00020',
        }
        color = colors.get(state, '#20242C')
        self.beagle_status.setText(label)
        self.beagle_status.setStyleSheet(
            f'background:{color};color:white;font-size:18px;font-weight:bold;'
            'padding:14px;border-radius:4px;')
        self.beagle_detail.setText(
            f'마지막 업데이트: {timestamp}' + (f'\n{detail}' if detail else ''))
        log_key = (state, label, detail)
        if log_key == self._last_beagle_log:
            return
        self._last_beagle_log = log_key
        self.beagle_log.insertItem(
            0, f'[{timestamp}] {label}' + (f'  ({detail})' if detail else ''))
        while self.beagle_log.count() > 200:
            self.beagle_log.takeItem(self.beagle_log.count() - 1)

    def show_frame(self, image):
        self.video.setPixmap(QPixmap.fromImage(image).scaled(
            self.video.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation))

    def on_click(self, x, y):
        width, height = self.node.frame_size
        self.node.select_pixel(int(x * width / self.video.width()), int(y * height / self.video.height()))

    def add_log(self, timestamp, label):
        row = self.log.rowCount()
        self.log.insertRow(row)
        self.log.setItem(row, 0, QTableWidgetItem(timestamp))
        self.log.setItem(row, 1, QTableWidgetItem(label))


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = ConsoleNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    window = ConsoleWindow(node)
    window.show()
    code = app.exec_()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
