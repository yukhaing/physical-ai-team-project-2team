#!/usr/bin/env python3
"""Qt operator console for the OMX/Beagle sorting workflow."""

import json
import sys
import threading

import numpy as np
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QPushButton,
                             QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)
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
            self.signals.log_ready.emit(row['completed_at'], row['classification'])
        except (KeyError, json.JSONDecodeError):
            pass

    def select_pixel(self, x, y):
        candidates = []
        for detection in self.detections:
            if detection.get('class') != 'defect':
                continue
            dx, dy = detection['center_x'] - x, detection['center_y'] - y
            candidates.append((dx * dx + dy * dy, detection))
        if not candidates:
            self.signals.status_ready.emit('선택 거부: YOLO 감지 박스가 없습니다.')
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
        self.operator_unloaded_button = QPushButton('작업자 하역 완료: 원위치 복귀')
        self.run_button = QPushButton('가동')
        self.stop_button = QPushButton('정지')
        self.estop_button = QPushButton('비상정지 (소프트웨어)')
        self.start_omx_button = QPushButton('OMX 집기 시작')
        self.continue_button = QPushButton('집기/배치 계속')
        self.reset_button = QPushButton('재설정')
        self.estop_button.setStyleSheet('background:#c62828;color:white;font-weight:bold;min-height:46px;')
        self.log = QTableWidget(0, 2)
        self.log.setHorizontalHeaderLabels(['완료 시간', '분류'])
        self.log.horizontalHeader().setStretchLastSection(True)
        side = QVBoxLayout()
        side.addWidget(QLabel('운전 상태'))
        side.addWidget(self.status)
        for button in (self.run_button, self.stop_button, self.estop_button,
                       self.start_omx_button, self.continue_button, self.reset_button):
            side.addWidget(button)
        side.insertWidget(6, self.operator_unloaded_button)
        side.addWidget(QLabel('완료 작업 로그'))
        side.addWidget(self.log, 1)
        layout = QHBoxLayout(self)
        layout.addWidget(self.video, 3)
        layout.addLayout(side, 1)
        self.run_button.clicked.connect(lambda: self.node.command('enable'))
        self.stop_button.clicked.connect(lambda: self.node.command('stop'))
        self.estop_button.clicked.connect(lambda: self.node.command('estop'))
        self.start_omx_button.clicked.connect(lambda: self.node.command('start_omx'))
        self.continue_button.clicked.connect(lambda: self.node.command('continue'))
        self.operator_unloaded_button.clicked.connect(lambda: self.node.command('operator_unloaded'))
        self.reset_button.clicked.connect(lambda: self.node.command('reset'))
        self.node.signals.frame_ready.connect(self.show_frame)
        self.node.signals.status_ready.connect(self.status.setText)
        self.node.signals.log_ready.connect(self.add_log)

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
