import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, Int32
from cv_bridge import CvBridge
import cv2

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *


MISSION_STATES = [
    "Searching",
    "Fire Detected",
    "Aligning",
    "Deploying Cover",
    "Retreating"
]


class ROS2Signals(QObject):
    # 가벼운 데이터만 Qt 시그널로 전달
    temp_signal = pyqtSignal(float)
    fire_signal = pyqtSignal(bool)
    state_signal = pyqtSignal(int)
    log_signal = pyqtSignal(str)


class ThermalUINode(Node):
    def __init__(self, signals):
        super().__init__('thermal_ui_node')
        self.signals = signals
        self.bridge = CvBridge()

        # 최신 이미지만 보관
        self.latest_image = None
        self.image_lock = threading.Lock()

        # 상태 변화 감지용
        self.last_state = -1

        self.img_sub = self.create_subscription(
            Image,
            '/thermal/image',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.temp_sub = self.create_subscription(
            Float32,
            '/thermal/max_temp',
            self.temp_callback,
            qos_profile_sensor_data
        )
        self.fire_sub = self.create_subscription(
            Bool,
            '/thermal/fire_detected',
            self.fire_callback,
            qos_profile_sensor_data
        )
        self.state_sub = self.create_subscription(
            Int32,
            '/mission/state',
            self.state_callback,
            qos_profile_sensor_data
        )

    def push_log(self, text):
        self.signals.log_signal.emit(text)

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.image_lock:
            self.latest_image = cv_img

    def temp_callback(self, msg):
        self.signals.temp_signal.emit(msg.data)

    def fire_callback(self, msg):
        self.signals.fire_signal.emit(msg.data)

    def state_callback(self, msg):
        state = msg.data

        # 상태 변화가 있을 때만 로그 기록
        if state != self.last_state:
            now = datetime.now().strftime("%H:%M:%S")

            if 0 <= state < len(MISSION_STATES):
                state_name = MISSION_STATES[state]
                self.push_log(f"[{now}] [MISSION] {state_name}")
            else:
                self.push_log(f"[{now}] [MISSION] Unknown state: {state}")

            self.last_state = state

        self.signals.state_signal.emit(state)


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()

        # 약 30 FPS 렌더링
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.render_latest_image)
        self.render_timer.start(1)

    def initUI(self):
        self.setWindowTitle('EV Fire-Fighting Robot Control Center')
        self.setGeometry(100, 100, 1200, 700)
        self.setStyleSheet("background-color: #2c3e50; color: white;")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.status_label = QLabel("SYSTEM READY")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "font-size: 20pt; font-weight: bold; background-color: #34495e; padding: 10px;"
        )
        layout.addWidget(self.status_label)

        content_layout = QHBoxLayout()

        # 왼쪽: 영상
        self.video_label = QLabel()
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet(
            "border: 2px solid #7f8c8d; background-color: black;"
        )
        self.video_label.setScaledContents(True)
        content_layout.addWidget(self.video_label)

        # 오른쪽 전체 레이아웃
        right_layout = QVBoxLayout()

        # 오른쪽 상단: 좌(thermal+mission), 우(log)
        top_right_layout = QHBoxLayout()

        # 오른쪽 상단 왼쪽 열
        info_layout = QVBoxLayout()

        temp_group = QGroupBox("Thermal Info")
        temp_vbox = QVBoxLayout()
        self.temp_display = QLabel("0.0 °C")
        self.temp_display.setAlignment(Qt.AlignCenter)
        self.temp_display.setStyleSheet(
            "font-size: 40pt; color: #e74c3c; font-family: 'Consolas';"
        )
        temp_vbox.addWidget(self.temp_display)
        temp_group.setLayout(temp_vbox)
        info_layout.addWidget(temp_group)

        state_group = QGroupBox("Mission Status")
        state_vbox = QVBoxLayout()

        self.state_list = [
            "1. Searching",
            "2. Fire Detected",
            "3. Aligning",
            "4. Deploying Cover",
            "5. Retreating"
        ]

        self.state_labels = []
        for s in self.state_list:
            lbl = QLabel(s)
            lbl.setStyleSheet("font-size: 12pt; color: #95a5a6;")
            state_vbox.addWidget(lbl)
            self.state_labels.append(lbl)

        state_group.setLayout(state_vbox)
        info_layout.addWidget(state_group)

        # 오른쪽 상단 오른쪽 열: ROS 로그
        log_group = QGroupBox("ROS Log")
        log_vbox = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #00ff99;
                font-family: Consolas, monospace;
                font-size: 11pt;
                border: 1px solid #7f8c8d;
            }
        """)
        self.log_text.setPlaceholderText("Mission event logs will appear here...")
        log_vbox.addWidget(self.log_text)
        log_group.setLayout(log_vbox)

        # 가로 비율은 창 크기에 따라 자연스럽게 증가
        top_right_layout.addLayout(info_layout, 1)
        top_right_layout.addWidget(log_group, 1)

        # 하단 emergency stop
        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet(
            "background-color: #c0392b; font-weight: bold; height: 50px;"
        )
        self.stop_btn.setMinimumHeight(50)

        right_layout.addLayout(top_right_layout, 1)
        right_layout.addWidget(self.stop_btn)

        content_layout.addLayout(right_layout, 1)
        layout.addLayout(content_layout)

        # 시작 상태 강조
        self.update_state(0)

    def render_latest_image(self):
        with self.ros_node.image_lock:
            if self.ros_node.latest_image is None:
                return
            img_to_show = self.ros_node.latest_image.copy()

        qt_img = self.convert_cv_to_qt(img_to_show)
        self.video_label.setPixmap(qt_img)

    def convert_cv_to_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_format = QImage(
            rgb_image.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888
        )
        return QPixmap.fromImage(qt_format)

    @pyqtSlot(float)
    def update_temp(self, temp):
        self.temp_display.setText(f"{temp:.1f} °C")
        if temp > 60.0:
            self.temp_display.setStyleSheet(
                "font-size: 40pt; color: #ff0000; font-weight: bold;"
            )
        else:
            self.temp_display.setStyleSheet(
                "font-size: 40pt; color: #e74c3c;"
            )

    @pyqtSlot(bool)
    def update_fire_status(self, is_fire):
        if is_fire:
            self.status_label.setText("🔥 FIRE DETECTED! 🔥")
            self.status_label.setStyleSheet(
                "font-size: 20pt; font-weight: bold; background-color: #c0392b; color: yellow;"
            )
        else:
            self.status_label.setText("MONITORING...")
            self.status_label.setStyleSheet(
                "font-size: 20pt; font-weight: bold; background-color: #34495e; color: white;"
            )

    @pyqtSlot(int)
    def update_state(self, state_index):
        if state_index < 0 or state_index >= len(self.state_labels):
            return

        for i, lbl in enumerate(self.state_labels):
            if i == state_index:
                lbl.setStyleSheet(
                    "font-size: 14pt; color: yellow; font-weight: bold;"
                )
            else:
                lbl.setStyleSheet(
                    "font-size: 12pt; color: #95a5a6;"
                )

    @pyqtSlot(str)
    def append_log(self, text):
        self.log_text.append(text)
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    ros_signals = ROS2Signals()
    node = ThermalUINode(ros_signals)

    main_win = MainWindow(node)

    ros_signals.temp_signal.connect(main_win.update_temp)
    ros_signals.fire_signal.connect(main_win.update_fire_status)
    ros_signals.state_signal.connect(main_win.update_state)
    ros_signals.log_signal.connect(main_win.append_log)

    spin_thread = threading.Thread(
        target=ros_spin_thread,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    main_win.show()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()