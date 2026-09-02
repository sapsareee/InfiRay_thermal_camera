import math
import os
import sys
import threading
import time

import cv2
import rclpy
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
from std_msgs.msg import Bool, Float32


THERMAL_VIDEO_PATH = os.path.expanduser("~/Desktop/thermal_obj_test_cut5_1.3x.mp4")
RGB_VIDEO_PATH = os.path.expanduser("~/Desktop/rgb_obj_test_cut2_1.3x.mp4")

MIN_VIDEO_SIZE = (360, 270)
FIRE_TEMP_WARNING_C = 60.0
FIRE_TEMP_CRITICAL_C = 80.0


class ROS2Signals(QObject):
    max_temp_signal = pyqtSignal(float)
    trend_signal = pyqtSignal(float)
    fire_signal = pyqtSignal(bool)


class FireDetectionNode(Node):
    """Receives the existing fire-detection results used by the UI cards."""

    def __init__(self, signals):
        super().__init__("thermal_and_rgb_fire_detection_ui_node")
        self.signals = signals

        self.create_subscription(
            Float32,
            "/thermal/max_temperature",
            lambda msg: self.signals.max_temp_signal.emit(float(msg.data)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            "/thermal/temperature_trend",
            lambda msg: self.signals.trend_signal.emit(float(msg.data)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/thermal/fire_detected",
            lambda msg: self.signals.fire_signal.emit(bool(msg.data)),
            qos_profile_sensor_data,
        )


class LoopingVideo:
    """Small OpenCV video reader that restarts automatically at end of file."""

    def __init__(self, path):
        self.path = path
        self.capture = cv2.VideoCapture(path)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open video: {path}")

        fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.fps = fps if fps and fps > 0 else 30.0
        self.timer_interval_ms = max(1, round(1000.0 / self.fps))

    def read(self):
        ok, frame = self.capture.read()
        if ok:
            return frame

        self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ok, frame = self.capture.read()
        return frame if ok else None

    def release(self):
        self.capture.release()


class MetricCard(QFrame):
    def __init__(self, title, value="--", unit="", large=False):
        super().__init__()
        self.title = QLabel(title)
        self.value = QLabel(value)
        self.unit = QLabel(unit)
        self.large = large
        self.value_font_size = 26 if large else 17
        self.setup_ui()

    def setup_ui(self):
        self.setObjectName("metricCard")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.setMinimumHeight(68 if self.large else 58)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 7, 10, 7)
        layout.setSpacing(2)

        self.title.setAlignment(Qt.AlignLeft)
        self.title.setStyleSheet(
            "color: #9fb3c8; font-size: 9pt; font-weight: 600;"
        )
        layout.addWidget(self.title)

        value_row = QHBoxLayout()
        value_row.setSpacing(4)
        self.value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.unit.setAlignment(Qt.AlignBottom)
        self.unit.setStyleSheet(
            "color: #b6c5d6; font-size: 10pt; font-weight: 600;"
        )
        value_row.addWidget(self.value, 1)
        value_row.addWidget(self.unit)
        layout.addLayout(value_row)
        self.set_alert_level("normal")

    def set_value(self, value, unit=None):
        self.value.setText(value)
        if unit is not None:
            self.unit.setText(unit)

    def set_alert_level(self, level):
        colors = {
            "critical": "#ff3b30",
            "warning": "#ffcc00",
            "safe": "#30d158",
            "normal": "#eef6ff",
        }
        color = colors.get(level, colors["normal"])
        weight = 900 if level in ("critical", "warning", "safe") else 800
        self.value.setStyleSheet(
            f"color: {color}; font-size: {self.value_font_size}pt; "
            f"font-weight: {weight}; font-family: Consolas;"
        )


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.current_max_temp = 0.0
        self.current_trend = 0.0
        self.current_fire = False
        self.start_time = time.time()
        self.blink_on = False

        self.thermal_video = None
        self.rgb_video = None

        self.init_ui()
        self.open_videos()

        self.metrics_timer = QTimer(self)
        self.metrics_timer.timeout.connect(self.update_sdk_like_metrics)
        self.metrics_timer.start(1000)

        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.update_fire_banner)
        self.blink_timer.start(450)

    def init_ui(self):
        self.setWindowTitle("Thermal + RGB Fire Detection Control Center")
        self.resize_to_screen()
        self.setStyleSheet(
            """
            QMainWindow { background-color: #0f172a; color: #e5edf7; }
            QWidget { background-color: #0f172a; color: #e5edf7; }
            QGroupBox {
                border: 1px solid #334155;
                border-radius: 8px;
                margin-top: 10px;
                padding: 9px 7px 7px 7px;
                font-size: 10pt;
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
                border-radius: 10px;
            }
            QPushButton {
                border-radius: 10px;
                padding: 8px;
                font-size: 12pt;
                font-weight: 900;
            }
            """
        )

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        root = QVBoxLayout(central_widget)
        root.setContentsMargins(10, 8, 10, 10)
        root.setSpacing(8)

        self.status_label = QLabel("MONITORING  |  FIRE OFF")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMinimumHeight(54)
        self.status_label.setMaximumHeight(72)
        root.addWidget(self.status_label)
        self.update_fire_banner()

        content = QGridLayout()
        content.setHorizontalSpacing(10)
        content.setVerticalSpacing(6)
        content.setColumnStretch(0, 1)
        content.setColumnStretch(1, 1)
        content.setRowStretch(0, 1)
        content.setRowStretch(1, 0)
        root.addLayout(content, 1)

        # Putting both streams in the same grid row keeps their heights equal.
        thermal_group, self.thermal_video_label = self.create_video_group(
            "Thermal Image Stream", "Loading thermal video..."
        )
        content.addWidget(thermal_group, 0, 0)

        camera_group = QGroupBox("SDK-Style Thermal Camera Parameters")
        camera_grid = QGridLayout(camera_group)
        camera_grid.setSpacing(6)
        self.palette_card = MetricCard("Palette", "WhiteHot")
        self.gain_card = MetricCard("Gain Mode", "High")
        self.frame_rate_card = MetricCard("Frame Rate", "--", "fps")
        self.fpa_temp_card = MetricCard("FPA Temp", "36.8", "°C")
        self.camera_temp_card = MetricCard("Camera Temp", "38.2", "°C")
        self.emissivity_card = MetricCard("Emissivity", "0.95")
        camera_cards = [
            self.palette_card,
            self.gain_card,
            self.frame_rate_card,
            self.fpa_temp_card,
            self.camera_temp_card,
            self.emissivity_card,
        ]
        for index, card in enumerate(camera_cards):
            camera_grid.addWidget(card, index // 3, index % 3)
        content.addWidget(camera_group, 1, 0)

        # Right: RGB object-detection video and fire result below it.
        rgb_group, self.rgb_video_label = self.create_video_group(
            "RGB Object Detection Stream", "Loading RGB video..."
        )
        content.addWidget(rgb_group, 0, 1)

        right_controls = QWidget()
        right_controls_layout = QVBoxLayout(right_controls)
        right_controls_layout.setContentsMargins(0, 0, 0, 0)
        right_controls_layout.setSpacing(6)

        detection_group = QGroupBox("Real-Time Fire Detection")
        detection_grid = QGridLayout(detection_group)
        detection_grid.setSpacing(6)
        self.highest_temp_card = MetricCard(
            "Highest Temperature", "0.0", "°C", large=True
        )
        self.max_temp_card = MetricCard(
            "Max Temperature", "0.0", "°C", large=True
        )
        self.trend_card = MetricCard(
            "Temperature Trend", "0.00", "°C/s", large=True
        )
        self.fire_card = MetricCard("Fire Detection", "OFF", large=True)
        detection_grid.addWidget(self.highest_temp_card, 0, 0)
        detection_grid.addWidget(self.max_temp_card, 0, 1)
        detection_grid.addWidget(self.trend_card, 1, 0)
        detection_grid.addWidget(self.fire_card, 1, 1)
        right_controls_layout.addWidget(detection_group)

        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet("background-color: #b91c1c; color: white;")
        self.stop_btn.setMinimumHeight(46)
        self.stop_btn.setMaximumHeight(56)
        right_controls_layout.addWidget(self.stop_btn)
        content.addWidget(right_controls, 1, 1)

    def create_video_group(self, title, placeholder):
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)

        label = QLabel(placeholder)
        label.setMinimumSize(*MIN_VIDEO_SIZE)
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(
            "background-color: #020617; border: 2px solid #475569; "
            "border-radius: 12px; color: #64748b; font-size: 14pt;"
        )
        layout.addWidget(label)
        return group, label

    def open_videos(self):
        try:
            self.thermal_video = LoopingVideo(THERMAL_VIDEO_PATH)
            self.thermal_timer = QTimer(self)
            self.thermal_timer.timeout.connect(self.render_thermal_frame)
            self.thermal_timer.start(self.thermal_video.timer_interval_ms)
            self.frame_rate_card.set_value(f"{self.thermal_video.fps:.1f}")
        except RuntimeError as exc:
            self.show_video_error(self.thermal_video_label, str(exc))

        try:
            self.rgb_video = LoopingVideo(RGB_VIDEO_PATH)
            self.rgb_timer = QTimer(self)
            self.rgb_timer.timeout.connect(self.render_rgb_frame)
            self.rgb_timer.start(self.rgb_video.timer_interval_ms)
        except RuntimeError as exc:
            self.show_video_error(self.rgb_video_label, str(exc))

    def show_video_error(self, label, message):
        label.setText(message)
        label.setStyleSheet(
            "background-color: #020617; border: 2px solid #b91c1c; "
            "border-radius: 12px; color: #fca5a5; font-size: 12pt;"
        )

    def resize_to_screen(self):
        screen = QApplication.primaryScreen()
        if screen is None:
            self.setGeometry(60, 40, 1400, 820)
            return

        available = screen.availableGeometry()
        width = min(1600, int(available.width() * 0.96))
        height = min(900, int(available.height() * 0.94))
        left = available.x() + max(0, (available.width() - width) // 2)
        top = available.y() + max(0, (available.height() - height) // 2)
        self.setGeometry(left, top, width, height)

    def render_thermal_frame(self):
        if self.thermal_video is None:
            return
        frame = self.thermal_video.read()
        if frame is not None:
            self.render_video_frame(frame, self.thermal_video_label, "THERMAL")

    def render_rgb_frame(self):
        if self.rgb_video is None:
            return
        frame = self.rgb_video.read()
        if frame is not None:
            self.render_video_frame(frame, self.rgb_video_label, "RGB")

    def render_video_frame(self, frame, label, source_name):
        frame_to_show = frame.copy()
        if self.current_fire:
            height, width = frame_to_show.shape[:2]
            thickness = max(3, min(width, height) // 100)
            cv2.rectangle(
                frame_to_show,
                (8, 8),
                (width - 8, height - 8),
                (0, 0, 255),
                thickness,
            )
            cv2.putText(
                frame_to_show,
                f"FIRE DETECTED - {source_name}",
                (24, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.1,
                (0, 255, 255),
                3,
                cv2.LINE_AA,
            )

        rgb_image = cv2.cvtColor(frame_to_show, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        qt_image = QImage(
            rgb_image.data,
            width,
            height,
            channels * width,
            QImage.Format_RGB888,
        ).copy()
        pixmap = QPixmap.fromImage(qt_image)
        pixmap = pixmap.scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        label.setPixmap(pixmap)

    def update_sdk_like_metrics(self):
        elapsed = time.time() - self.start_time
        fpa_temp = 36.8 + 0.35 * math.sin(elapsed / 5.0)
        camera_temp = 38.2 + 0.45 * math.sin(elapsed / 6.0)
        self.fpa_temp_card.set_value(f"{fpa_temp:.1f}")
        self.camera_temp_card.set_value(f"{camera_temp:.1f}")

    @pyqtSlot(float)
    def update_max_temperature(self, temp):
        self.current_max_temp = temp
        self.highest_temp_card.set_value(f"{temp:.1f}")
        self.max_temp_card.set_value(f"{temp:.1f}")

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
            level = "critical"
        elif trend >= 0.3:
            level = "warning"
        else:
            level = "normal"
        self.trend_card.set_alert_level(level)

    @pyqtSlot(bool)
    def update_fire_status(self, is_fire):
        self.current_fire = is_fire
        self.fire_card.set_value("ON" if is_fire else "OFF")
        self.fire_card.set_alert_level("critical" if is_fire else "safe")
        self.update_fire_banner()

    def update_fire_banner(self):
        if self.current_fire:
            self.blink_on = not self.blink_on
            background = "#dc2626" if self.blink_on else "#7f1d1d"
            self.status_label.setText("FIRE DETECTED  |  DEPLOYMENT REQUIRED")
            self.status_label.setStyleSheet(
                f"background-color: {background}; color: #fff7ed; "
                "border-radius: 12px; font-size: 23pt; font-weight: 950; "
                "padding: 8px;"
            )
        else:
            self.status_label.setText("MONITORING  |  FIRE OFF")
            self.status_label.setStyleSheet(
                "background-color: #1e293b; color: #e5edf7; "
                "border-radius: 12px; font-size: 22pt; font-weight: 900; "
                "padding: 8px;"
            )

    def closeEvent(self, event):
        if self.thermal_video is not None:
            self.thermal_video.release()
        if self.rgb_video is not None:
            self.rgb_video.release()
        event.accept()


def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    app.setFont(QFont("Arial", 10))

    ros_signals = ROS2Signals()
    node = FireDetectionNode(ros_signals)
    main_window = MainWindow(node)

    ros_signals.max_temp_signal.connect(main_window.update_max_temperature)
    ros_signals.trend_signal.connect(main_window.update_trend)
    ros_signals.fire_signal.connect(main_window.update_fire_status)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    main_window.show()

    exit_code = app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
