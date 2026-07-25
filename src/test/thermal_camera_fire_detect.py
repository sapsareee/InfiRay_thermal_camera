import sys
import threading

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
 
#

MISSION_STATES = [
    "Searching",
    "Fire Detected",
    "Aligning",
    "Deploying Cover",
    "Retreating"
]

VIDEO_SIZE = (500, 500)


class ROS2Signals(QObject):
    # 가벼운 데이터만 Qt 시그널로 전달
    temp_signal = pyqtSignal(float)
    trend_signal = pyqtSignal(float)
    fire_signal = pyqtSignal(bool)
    state_signal = pyqtSignal(int)


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
            '/thermal/max_temperature',
            self.temp_callback,
            qos_profile_sensor_data
        )
        self.trend_sub = self.create_subscription(
            Float32,
            '/thermal/temperature_trend',
            self.trend_callback,
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

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.image_lock:
            self.latest_image = cv_img

    def temp_callback(self, msg):
        self.signals.temp_signal.emit(msg.data)

    def trend_callback(self, msg):
        self.signals.trend_signal.emit(msg.data)

    def fire_callback(self, msg):
        self.signals.fire_signal.emit(msg.data)

    def state_callback(self, msg):
        state = msg.data

        # 상태 변화가 있을 때만 업데이트
        if state != self.last_state:
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
            "font-size: 20pt; font-weight: bold; background-color: #34495e; padding: 10px; height: 150px;"
        )
        self.status_label.setMinimumHeight(150)
        layout.addWidget(self.status_label)

        content_layout = QHBoxLayout()

        # 왼쪽: 영상
        self.video_label = QLabel()
        self.video_label.setFixedSize(*VIDEO_SIZE)
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
        info_layout.addStretch()

        temp_group = QGroupBox("Thermal Info Quick test")
        temp_vbox = QVBoxLayout()
        temp_vbox.addStretch()
        self.temp_display = QLabel("0.0 °C")
        self.temp_display.setAlignment(Qt.AlignCenter)
        self.temp_display.setStyleSheet(
            "font-size: 40pt; color: #e74c3c; font-family: 'Consolas';"
        )
        temp_vbox.addWidget(self.temp_display)
        # 추가: 최고온도(명확 표시) 및 추세
        self.max_label = QLabel("Max: 0.0 °C")
        self.max_label.setAlignment(Qt.AlignCenter)
        self.max_label.setStyleSheet("font-size: 20pt; color: #ecf0f1;")
        temp_vbox.addWidget(self.max_label)

        self.trend_label = QLabel("Trend: 0.00 C/s")
        self.trend_label.setAlignment(Qt.AlignCenter)
        self.trend_label.setStyleSheet("font-size: 20pt; color: #bdc3c7;")
        temp_vbox.addWidget(self.trend_label)
        temp_vbox.addStretch()
        temp_group.setLayout(temp_vbox)
        info_layout.addWidget(temp_group)
        info_layout.addStretch()

        # 가로 비율은 창 크기에 따라 자연스럽게 증가
        top_right_layout.addLayout(info_layout, 1)

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

    def render_latest_image(self):
        with self.ros_node.image_lock:
            if self.ros_node.latest_image is None:
                return
            img_to_show = self.ros_node.latest_image.copy()

        if (img_to_show.shape[1], img_to_show.shape[0]) != VIDEO_SIZE:
            img_to_show = cv2.resize(
                img_to_show,
                VIDEO_SIZE,
                interpolation=cv2.INTER_AREA if img_to_show.shape[1] > VIDEO_SIZE[0] else cv2.INTER_LINEAR,
            )

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
        # 최고온도도 별도 레이블에 표시
        self.max_label.setText(f"Max: {temp:.1f} °C")
        if temp > 60.0:
            self.temp_display.setStyleSheet(
                "font-size: 40pt; color: #ff0000; font-weight: bold;"
            )
        else:
            self.temp_display.setStyleSheet(
                "font-size: 40pt; color: #e74c3c;"
            )

    @pyqtSlot(float)
    def update_trend(self, trend):
        self.trend_label.setText(f"Trend: {trend:.2f} C/s")

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
    ros_signals.trend_signal.connect(main_win.update_trend)

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