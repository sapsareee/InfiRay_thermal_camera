import sys
import math
import time
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32


MISSION_STATES = [
    "Searching",
    "Fire Detected",
    "Aligning",
    "Deploying Cover",
    "Retreating",
]

VIDEO_SIZE = (620, 520)
FIRE_TEMP_WARNING_C = 60.0
FIRE_TEMP_CRITICAL_C = 80.0


class ROS2Signals(QObject):
    image_signal = pyqtSignal()
    max_temp_signal = pyqtSignal(float)
    trend_signal = pyqtSignal(float)
    fire_signal = pyqtSignal(bool)
    state_signal = pyqtSignal(int)


class ThermalUINode(Node):
    def __init__(self, signals):
        super().__init__("thermal_camera_sdk_ui_node")
        self.signals = signals
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.last_state = -1

        self.create_subscription(
            Image,
            "/thermal/image",
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            "/thermal/max_temperature",
            self.max_temp_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            "/thermal/temperature_trend",
            self.trend_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/thermal/fire_detected",
            self.fire_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Int32,
            "/mission/state",
            self.state_callback,
            qos_profile_sensor_data,
        )

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert thermal image: {exc}")
            return

        with self.image_lock:
            self.latest_image = cv_img
        self.signals.image_signal.emit()

    def max_temp_callback(self, msg):
        self.signals.max_temp_signal.emit(float(msg.data))

    def trend_callback(self, msg):
        self.signals.trend_signal.emit(float(msg.data))

    def fire_callback(self, msg):
        self.signals.fire_signal.emit(bool(msg.data))

    def state_callback(self, msg):
        state = int(msg.data)
        if state != self.last_state:
            self.last_state = state
        self.signals.state_signal.emit(state)


class MetricCard(QFrame):
    def __init__(self, title, value="--", unit="", large=False, accent="normal"):
        super().__init__()
        self.title = QLabel(title)
        self.value = QLabel(value)
        self.unit = QLabel(unit)
        self.accent = accent
        self.large = large
        self.setup_ui()

    def setup_ui(self):
        self.setObjectName("metricCard")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        self.title.setAlignment(Qt.AlignLeft)
        self.title.setStyleSheet("color: #9fb3c8; font-size: 11pt; font-weight: 600;")
        layout.addWidget(self.title)

        value_row = QHBoxLayout()
        value_row.setSpacing(6)
        self.value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        size = "38pt" if self.large else "22pt"
        self.value.setStyleSheet(f"color: #eef6ff; font-size: {size}; font-weight: 800; font-family: Consolas;")
        self.unit.setAlignment(Qt.AlignBottom)
        self.unit.setStyleSheet("color: #b6c5d6; font-size: 13pt; font-weight: 600;")
        value_row.addWidget(self.value, 1)
        value_row.addWidget(self.unit)
        layout.addLayout(value_row)

    def set_value(self, value, unit=None):
        self.value.setText(value)
        if unit is not None:
            self.unit.setText(unit)

    def set_alert_level(self, level):
        if level == "critical":
            self.value.setStyleSheet(
                f"color: #ff3b30; font-size: {'38pt' if self.large else '22pt'}; "
                "font-weight: 900; font-family: Consolas;"
            )
        elif level == "warning":
            self.value.setStyleSheet(
                f"color: #ffcc00; font-size: {'38pt' if self.large else '22pt'}; "
                "font-weight: 900; font-family: Consolas;"
            )
        elif level == "safe":
            self.value.setStyleSheet(
                f"color: #30d158; font-size: {'38pt' if self.large else '22pt'}; "
                "font-weight: 900; font-family: Consolas;"
            )
        else:
            self.value.setStyleSheet(
                f"color: #eef6ff; font-size: {'38pt' if self.large else '22pt'}; "
                "font-weight: 800; font-family: Consolas;"
            )


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.current_max_temp = 0.0
        self.current_trend = 0.0
        self.current_fire = False
        self.current_state = 0
        self.start_time = time.time()
        self.fake_frame_count = 0
        self.blink_on = False
        self.init_ui()

        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.render_latest_image)
        self.render_timer.start(33)

        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.update_sdk_like_metrics)
        self.sim_timer.start(1000)

        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.update_fire_banner)
        self.blink_timer.start(450)

    def init_ui(self):
        self.setWindowTitle("Thermal Camera Fire Detection Control Center")
        self.setGeometry(80, 60, 1380, 820)
        self.setStyleSheet(
            """
            QMainWindow { background-color: #0f172a; color: #e5edf7; }
            QWidget { background-color: #0f172a; color: #e5edf7; }
            QGroupBox {
                border: 1px solid #334155;
                border-radius: 10px;
                margin-top: 14px;
                padding: 14px 10px 10px 10px;
                font-size: 12pt;
                font-weight: 700;
                color: #dbeafe;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }
            QFrame#metricCard {
                background-color: #162033;
                border: 1px solid #26364f;
                border-radius: 14px;
            }
            QPushButton {
                border-radius: 12px;
                padding: 14px;
                font-size: 15pt;
                font-weight: 900;
            }
            """
        )

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        root = QVBoxLayout(central_widget)
        root.setContentsMargins(18, 16, 18, 18)
        root.setSpacing(14)

        self.status_label = QLabel("SYSTEM READY  |  FIRE OFF")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMinimumHeight(86)
        self.status_label.setStyleSheet(
            "background-color: #1e293b; color: #e5edf7; border-radius: 18px; "
            "font-size: 28pt; font-weight: 900; padding: 12px;"
        )
        root.addWidget(self.status_label)

        content = QHBoxLayout()
        content.setSpacing(16)
        root.addLayout(content, 1)

        left = QVBoxLayout()
        left.setSpacing(12)
        content.addLayout(left, 3)

        video_group = QGroupBox("Thermal Image Stream")
        video_layout = QVBoxLayout(video_group)
        self.video_label = QLabel()
        self.video_label.setFixedSize(*VIDEO_SIZE)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setText("Waiting for /thermal/image")
        self.video_label.setStyleSheet(
            "background-color: #020617; border: 2px solid #475569; border-radius: 12px; "
            "color: #64748b; font-size: 16pt;"
        )
        self.video_label.setScaledContents(True)
        video_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)
        left.addWidget(video_group, 1)

        bottom_group = QGroupBox("SDK-Style Camera Parameters")
        bottom_grid = QGridLayout(bottom_group)
        self.palette_card = MetricCard("Palette", "WhiteHot", "")
        self.gain_card = MetricCard("Gain Mode", "High", "")
        self.frame_rate_card = MetricCard("Frame Rate", "15.0", "fps")
        self.fpa_temp_card = MetricCard("FPA Temp", "36.8", "°C")
        self.camera_temp_card = MetricCard("Camera Temp", "38.2", "°C")
        self.emissivity_card = MetricCard("Emissivity", "0.95", "")
        for index, card in enumerate([
            self.palette_card,
            self.gain_card,
            self.frame_rate_card,
            self.fpa_temp_card,
            self.camera_temp_card,
            self.emissivity_card,
        ]):
            bottom_grid.addWidget(card, index // 3, index % 3)
        left.addWidget(bottom_group)

        right = QVBoxLayout()
        right.setSpacing(12)
        content.addLayout(right, 2)

        primary_group = QGroupBox("Real-Time Fire Detection")
        primary_grid = QGridLayout(primary_group)
        primary_grid.setSpacing(12)
        self.highest_temp_card = MetricCard("Highest Temperature", "0.0", "°C", large=True)
        self.max_temp_card = MetricCard("Max Temperature", "0.0", "°C", large=True)
        self.trend_card = MetricCard("Temperature Trend", "0.00", "°C/s", large=True)
        self.fire_card = MetricCard("Fire Detection", "OFF", "", large=True)
        primary_grid.addWidget(self.highest_temp_card, 0, 0)
        primary_grid.addWidget(self.max_temp_card, 0, 1)
        primary_grid.addWidget(self.trend_card, 1, 0)
        primary_grid.addWidget(self.fire_card, 1, 1)
        right.addWidget(primary_group)

        area_group = QGroupBox("Area Temperature Analysis")
        area_grid = QGridLayout(area_group)
        self.area_max_card = MetricCard("ROI Max", "0.0", "°C")
        self.area_min_card = MetricCard("ROI Min", "24.0", "°C")
        self.area_avg_card = MetricCard("ROI Avg", "28.0", "°C")
        self.area_center_card = MetricCard("Center Temp", "27.5", "°C")
        self.alarm_threshold_card = MetricCard("Alarm Threshold", "60.0", "°C")
        self.hold_time_card = MetricCard("Fire Hold Time", "1.0", "s")
        for index, card in enumerate([
            self.area_max_card,
            self.area_min_card,
            self.area_avg_card,
            self.area_center_card,
            self.alarm_threshold_card,
            self.hold_time_card,
        ]):
            area_grid.addWidget(card, index // 2, index % 2)
        right.addWidget(area_group)

        env_group = QGroupBox("Environment Compensation")
        env_grid = QGridLayout(env_group)
        self.ambient_card = MetricCard("Ambient Temp", "24.5", "°C")
        self.reflected_card = MetricCard("Reflected Temp", "24.0", "°C")
        self.humidity_card = MetricCard("Humidity", "45", "%")
        self.distance_card = MetricCard("Target Distance", "3.0", "m")
        for index, card in enumerate([
            self.ambient_card,
            self.reflected_card,
            self.humidity_card,
            self.distance_card,
        ]):
            env_grid.addWidget(card, index // 2, index % 2)
        right.addWidget(env_group)

        mission_group = QGroupBox("Robot Mission State")
        mission_layout = QVBoxLayout(mission_group)
        self.mission_label = QLabel("Searching")
        self.mission_label.setAlignment(Qt.AlignCenter)
        self.mission_label.setStyleSheet(
            "background-color: #1e293b; border-radius: 12px; padding: 18px; "
            "font-size: 24pt; font-weight: 900; color: #dbeafe;"
        )
        mission_layout.addWidget(self.mission_label)
        right.addWidget(mission_group)

        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet("background-color: #b91c1c; color: white;")
        self.stop_btn.setMinimumHeight(64)
        right.addWidget(self.stop_btn)

    def render_latest_image(self):
        with self.ros_node.image_lock:
            if self.ros_node.latest_image is None:
                return
            img_to_show = self.ros_node.latest_image.copy()

        if self.current_fire:
            cv2.rectangle(img_to_show, (8, 8), (img_to_show.shape[1] - 8, img_to_show.shape[0] - 8), (0, 0, 255), 6)
            cv2.putText(
                img_to_show,
                "FIRE DETECTED",
                (24, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.4,
                (0, 255, 255),
                4,
                cv2.LINE_AA,
            )

        if (img_to_show.shape[1], img_to_show.shape[0]) != VIDEO_SIZE:
            interpolation = cv2.INTER_AREA if img_to_show.shape[1] > VIDEO_SIZE[0] else cv2.INTER_LINEAR
            img_to_show = cv2.resize(img_to_show, VIDEO_SIZE, interpolation=interpolation)

        self.video_label.setPixmap(self.convert_cv_to_qt(img_to_show))

    def convert_cv_to_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        qt_img = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return QPixmap.fromImage(qt_img)

    def update_sdk_like_metrics(self):
        elapsed = time.time() - self.start_time
        self.fake_frame_count += 1

        frame_rate = 15.0 + 0.6 * math.sin(elapsed / 2.5)
        fpa_temp = 36.8 + 0.35 * math.sin(elapsed / 5.0)
        camera_temp = 38.2 + 0.45 * math.sin(elapsed / 6.0)
        ambient = 24.5 + 0.2 * math.sin(elapsed / 8.0)
        reflected = 24.0 + 0.15 * math.cos(elapsed / 9.0)
        humidity = 45 + 2 * math.sin(elapsed / 11.0)
        distance = 3.0 + 0.05 * math.sin(elapsed / 7.0)

        synthetic_avg = max(ambient + 3.2, self.current_max_temp - 9.5 - abs(self.current_trend) * 1.2)
        synthetic_min = max(ambient - 1.2, synthetic_avg - 5.0)
        synthetic_center = synthetic_avg + 0.8 * math.sin(elapsed / 3.0)

        self.frame_rate_card.set_value(f"{frame_rate:.1f}")
        self.fpa_temp_card.set_value(f"{fpa_temp:.1f}")
        self.camera_temp_card.set_value(f"{camera_temp:.1f}")
        self.ambient_card.set_value(f"{ambient:.1f}")
        self.reflected_card.set_value(f"{reflected:.1f}")
        self.humidity_card.set_value(f"{humidity:.0f}")
        self.distance_card.set_value(f"{distance:.1f}")

        self.area_max_card.set_value(f"{self.current_max_temp:.1f}")
        self.area_min_card.set_value(f"{synthetic_min:.1f}")
        self.area_avg_card.set_value(f"{synthetic_avg:.1f}")
        self.area_center_card.set_value(f"{synthetic_center:.1f}")

    @pyqtSlot(float)
    def update_max_temperature(self, temp):
        self.current_max_temp = temp
        self.highest_temp_card.set_value(f"{temp:.1f}")
        self.max_temp_card.set_value(f"{temp:.1f}")
        self.area_max_card.set_value(f"{temp:.1f}")

        if temp >= FIRE_TEMP_CRITICAL_C:
            level = "critical"
        elif temp >= FIRE_TEMP_WARNING_C:
            level = "warning"
        else:
            level = "normal"

        self.highest_temp_card.set_alert_level(level)
        self.max_temp_card.set_alert_level(level)

    @pyqtSlot(float)
    def update_trend(self, trend):
        self.current_trend = trend
        self.trend_card.set_value(f"{trend:.2f}")
        if trend >= 1.0:
            self.trend_card.set_alert_level("critical")
        elif trend >= 0.3:
            self.trend_card.set_alert_level("warning")
        else:
            self.trend_card.set_alert_level("normal")

    @pyqtSlot(bool)
    def update_fire_status(self, is_fire):
        self.current_fire = is_fire
        if is_fire:
            self.fire_card.set_value("ON")
            self.fire_card.set_alert_level("critical")
            self.mission_label.setText("Fire Detected")
            self.mission_label.setStyleSheet(
                "background-color: #7f1d1d; border-radius: 12px; padding: 18px; "
                "font-size: 24pt; font-weight: 900; color: #fde68a;"
            )
        else:
            self.fire_card.set_value("OFF")
            self.fire_card.set_alert_level("safe")
            self.update_mission_state(self.current_state)
        self.update_fire_banner()

    @pyqtSlot(int)
    def update_mission_state(self, state):
        self.current_state = state
        if self.current_fire:
            return
        if 0 <= state < len(MISSION_STATES):
            text = MISSION_STATES[state]
        else:
            text = f"Unknown State ({state})"
        self.mission_label.setText(text)
        self.mission_label.setStyleSheet(
            "background-color: #1e293b; border-radius: 12px; padding: 18px; "
            "font-size: 24pt; font-weight: 900; color: #dbeafe;"
        )

    def update_fire_banner(self):
        if self.current_fire:
            self.blink_on = not self.blink_on
            background = "#dc2626" if self.blink_on else "#7f1d1d"
            self.status_label.setText("FIRE DETECTED  |  DEPLOYMENT REQUIRED")
            self.status_label.setStyleSheet(
                f"background-color: {background}; color: #fff7ed; border-radius: 18px; "
                "font-size: 30pt; font-weight: 950; padding: 12px;"
            )
        else:
            self.status_label.setText("MONITORING  |  FIRE OFF")
            self.status_label.setStyleSheet(
                "background-color: #1e293b; color: #e5edf7; border-radius: 18px; "
                "font-size: 28pt; font-weight: 900; padding: 12px;"
            )


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    app.setFont(QFont("Arial", 10))

    ros_signals = ROS2Signals()
    node = ThermalUINode(ros_signals)
    main_win = MainWindow(node)

    ros_signals.max_temp_signal.connect(main_win.update_max_temperature)
    ros_signals.trend_signal.connect(main_win.update_trend)
    ros_signals.fire_signal.connect(main_win.update_fire_status)
    ros_signals.state_signal.connect(main_win.update_mission_state)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    main_win.show()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
