"""Video-derived thermal and RGB fire monitoring dashboard.

This UI is intended for recorded videos that do not have matching sensor logs.
It calculates repeatable visual metrics directly from every video frame:

* Thermal: normalized peak intensity, hot-area ratio, contrast, frame change,
  and the colour-coded Fire ON/OFF cue already burned into the supplied video.
* RGB: cyan detector-overlay ratio, large detection regions, frame motion, and
  a visual evidence score that is active only while a bounding box is visible.

The displayed values are image-derived estimates, not radiometric temperature
measurements or raw neural-network confidence values.  The final FIRE ON state
is asserted only while both the thermal and RGB decisions are ON.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
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


THERMAL_VIDEO_PATH = os.path.expanduser(
    "~/Desktop/thermal_obj_test_cut5_1.3x.mp4"
)
RGB_VIDEO_PATH = os.path.expanduser(
    "~/Desktop/rgb_obj_test_cut2_1.3x_conf_0.15.avi"
)

MIN_VIDEO_SIZE = (360, 270)
THERMAL_ON_THRESHOLD = 60.0
THERMAL_OFF_THRESHOLD = 40.0


def clamp(value: float, low: float = 0.0, high: float = 100.0) -> float:
    return max(low, min(high, value))


def exponential_step(
    current: float, target: float, dt: float, rise_tau: float, fall_tau: float
) -> float:
    """Move a value toward a target without depending on the video's FPS."""

    tau = rise_tau if target > current else fall_tau
    alpha = 1.0 - math.exp(-max(dt, 0.001) / max(tau, 0.001))
    return current + alpha * (target - current)


class LoopingVideo:
    """OpenCV video reader that restarts at the first frame at EOF."""

    def __init__(self, path: str):
        self.path = path
        self.capture = cv2.VideoCapture(path)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open video: {path}")

        fps = float(self.capture.get(cv2.CAP_PROP_FPS))
        self.fps = fps if fps > 0.0 else 30.0
        self.timer_interval_ms = max(1, round(1000.0 / self.fps))

    def read(self) -> tuple[Optional[np.ndarray], bool]:
        ok, frame = self.capture.read()
        if ok:
            return frame, False

        self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ok, frame = self.capture.read()
        return (frame if ok else None), True

    def release(self) -> None:
        self.capture.release()


@dataclass
class ThermalMetrics:
    apparent_peak: float = 0.0
    hot_area: float = 0.0
    contrast: float = 0.0
    frame_change: float = 0.0
    confidence: float = 0.0
    detected: bool = False


@dataclass
class RGBMetrics:
    overlay_ratio: float = 0.0
    alert_regions: int = 0
    scene_motion: float = 0.0
    visual_evidence: float = 0.0
    confidence: float = 0.0
    detected: bool = False


class ThermalFrameAnalyzer:
    """Calculate thermal-looking metrics from rendered video pixels.

    Absolute temperature cannot be recovered from an 8-bit rendered video.
    The supplied recording does, however, contain a colour-coded Fire ON/OFF
    HUD.  Its green/cyan cue is combined with generic intensity features so the
    dashboard follows the contents of the video rather than a timer script.
    """

    def __init__(self, fps: float):
        self.dt = 1.0 / max(fps, 1.0)
        self.previous_gray: Optional[np.ndarray] = None
        self.frame_change = 0.0
        self.confidence = 0.0
        self.detected = False

    def reset(self) -> None:
        self.previous_gray = None
        self.frame_change = 0.0
        self.confidence = 0.0
        self.detected = False

    def analyze(self, frame: np.ndarray) -> ThermalMetrics:
        height, width = frame.shape[:2]

        # The supplied 600x600 recording has the useful image in its upper 76%.
        # Proportional cropping also works when an equivalent video is resized.
        image_bottom = max(1, int(height * 0.76))
        content = frame[:image_bottom]
        gray = cv2.cvtColor(content, cv2.COLOR_BGR2GRAY)

        y0, y1 = int(image_bottom * 0.23), int(image_bottom * 0.94)
        x0, x1 = int(width * 0.03), int(width * 0.97)
        analysis_roi = gray[y0:y1, x0:x1]
        if analysis_roi.size == 0:
            analysis_roi = gray

        median, p90, p995 = np.percentile(analysis_roi, [50.0, 90.0, 99.5])
        apparent_peak = float(p995 / 255.0 * 100.0)
        contrast = float((p995 - median) / 255.0 * 100.0)
        hot_threshold = min(245.0, max(float(median + 55.0), float(p90 + 14.0)))
        hot_area = float(np.mean(analysis_roi >= hot_threshold) * 100.0)

        raw_frame_change = 0.0
        if self.previous_gray is not None and self.previous_gray.shape == gray.shape:
            delta = cv2.absdiff(gray, self.previous_gray)
            raw_frame_change = float(np.mean(delta) / 255.0 * 100.0)
        self.previous_gray = gray.copy()
        self.frame_change = exponential_step(
            self.frame_change,
            raw_frame_change,
            self.dt,
            rise_tau=0.08,
            fall_tau=0.25,
        )

        # In the supplied thermal video, x=180..350/y=75..110 contains the
        # colour-coded "Fire: ON/OFF" recording overlay.  Ratios make the test
        # resolution-independent, while feature-only fallback handles videos
        # that do not contain this overlay.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hud = hsv[
            int(height * 0.125) : max(int(height * 0.185), 1),
            int(width * 0.30) : max(int(width * 0.59), 1),
        ]
        hud_on: Optional[bool] = None
        if hud.size:
            green = cv2.inRange(hud, (35, 80, 70), (90, 255, 255))
            cyan = cv2.inRange(hud, (80, 80, 70), (110, 255, 255))
            green_ratio = float(np.mean(green > 0))
            cyan_ratio = float(np.mean(cyan > 0))
            if cyan_ratio >= 0.01:
                hud_on = False
            elif green_ratio >= 0.015:
                hud_on = True

        feature_score = clamp(
            contrast * 1.15 + hot_area * 1.8 + self.frame_change * 0.25
        )
        if hud_on is True:
            target = clamp(78.0 + feature_score * 0.20)
        elif hud_on is False:
            target = clamp(10.0 + feature_score * 0.25)
        else:
            target = feature_score

        self.confidence = exponential_step(
            self.confidence, target, self.dt, rise_tau=0.20, fall_tau=0.18
        )
        if not self.detected and self.confidence >= THERMAL_ON_THRESHOLD:
            self.detected = True
        elif self.detected and self.confidence <= THERMAL_OFF_THRESHOLD:
            self.detected = False

        return ThermalMetrics(
            apparent_peak=apparent_peak,
            hot_area=hot_area,
            contrast=contrast,
            frame_change=self.frame_change,
            confidence=self.confidence,
            detected=self.detected,
        )


class RGBFrameAnalyzer:
    """Extract evidence from the detector annotations in an RGB recording.

    RGB detection deliberately has no hold timer or confidence hysteresis.  It
    is ON only on frames that contain a large cyan detector bounding box.
    """

    def __init__(self, fps: float):
        _ = fps  # Keep the same analyzer constructor interface as thermal.
        self.previous_gray: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.previous_gray = None

    def analyze(self, frame: np.ndarray) -> RGBMetrics:
        height, width = frame.shape[:2]
        image_bottom = max(1, int(height * 0.78))
        content = frame[:image_bottom]
        hsv = cv2.cvtColor(content, cv2.COLOR_BGR2HSV)

        # The recorded object detector uses cyan boxes and labels.  Restricting
        # hue and saturation prevents neutral smoke/background pixels from being
        # mistaken for annotations.
        cyan_mask = cv2.inRange(hsv, (78, 90, 80), (105, 255, 255))
        cyan_mask = cv2.morphologyEx(
            cyan_mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8)
        )
        overlay_ratio = float(np.mean(cyan_mask > 0) * 100.0)

        contours, _ = cv2.findContours(
            cyan_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        alert_regions = 0
        minimum_area = width * image_bottom * 0.002
        for contour in contours:
            _, _, box_width, box_height = cv2.boundingRect(contour)
            if (
                box_width >= width * 0.05
                and box_height >= image_bottom * 0.05
                and cv2.contourArea(contour) >= minimum_area
            ):
                alert_regions += 1

        gray = cv2.cvtColor(content, cv2.COLOR_BGR2GRAY)
        scene_motion = 0.0
        if self.previous_gray is not None and self.previous_gray.shape == gray.shape:
            delta = cv2.absdiff(gray, self.previous_gray)
            scene_motion = float(np.mean(delta >= 18) * 100.0)
        self.previous_gray = gray.copy()

        if alert_regions:
            visual_evidence = clamp(
                68.0 + overlay_ratio * 20.0 + min(alert_regions - 1, 2) * 5.0
            )
        else:
            visual_evidence = 0.0

        # Exact frame-level linkage: no bounding box means RGB OFF immediately.
        detected = alert_regions > 0
        confidence = visual_evidence if detected else 0.0

        return RGBMetrics(
            overlay_ratio=overlay_ratio,
            alert_regions=alert_regions,
            scene_motion=scene_motion,
            visual_evidence=visual_evidence,
            confidence=confidence,
            detected=detected,
        )


class MetricCard(QFrame):
    def __init__(self, title: str, value: str = "--", unit: str = ""):
        super().__init__()
        self.title = QLabel(title)
        self.value = QLabel(value)
        self.unit = QLabel(unit)
        self.setup_ui()

    def setup_ui(self) -> None:
        self.setObjectName("metricCard")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.setMinimumHeight(58)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(1)

        self.title.setStyleSheet(
            "color: #9fb3c8; font-size: 9pt; font-weight: 600;"
        )
        layout.addWidget(self.title)

        row = QHBoxLayout()
        row.setSpacing(4)
        self.value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.unit.setAlignment(Qt.AlignBottom)
        self.unit.setStyleSheet(
            "color: #b6c5d6; font-size: 9pt; font-weight: 600;"
        )
        row.addWidget(self.value, 1)
        row.addWidget(self.unit)
        layout.addLayout(row)
        self.set_alert_level("normal")

    def set_value(self, value: str, unit: Optional[str] = None) -> None:
        self.value.setText(value)
        if unit is not None:
            self.unit.setText(unit)

    def set_alert_level(self, level: str) -> None:
        colors = {
            "critical": "#ff453a",
            "warning": "#ffd60a",
            "safe": "#30d158",
            "normal": "#eef6ff",
        }
        color = colors.get(level, colors["normal"])
        weight = 900 if level in ("critical", "warning", "safe") else 800
        self.value.setStyleSheet(
            f"color: {color}; font-size: 17pt; font-weight: {weight}; "
            "font-family: Consolas;"
        )


class MainWindow(QMainWindow):
    def __init__(self, thermal_path: str, rgb_path: str):
        super().__init__()
        self.thermal_path = thermal_path
        self.rgb_path = rgb_path

        self.thermal_video: Optional[LoopingVideo] = None
        self.rgb_video: Optional[LoopingVideo] = None
        self.thermal_analyzer: Optional[ThermalFrameAnalyzer] = None
        self.rgb_analyzer: Optional[RGBFrameAnalyzer] = None
        self.video_timers: list[QTimer] = []

        self.thermal_detected = False
        self.rgb_detected = False
        self.current_fire = False
        self.paused = False
        self.blink_on = False
        self.last_banner_state: Optional[tuple[bool, bool, bool]] = None

        self.init_ui()
        self.open_videos()

        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.animate_fire_banner)
        self.blink_timer.start(450)
        self.update_final_decision(force=True)

    def init_ui(self) -> None:
        self.setWindowTitle("Video-Derived Thermal + RGB Fire Monitoring")
        self.resize_to_screen()
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background-color: #0f172a; color: #e5edf7; }
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

        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMinimumHeight(54)
        self.status_label.setMaximumHeight(72)
        root.addWidget(self.status_label)

        content = QGridLayout()
        content.setHorizontalSpacing(10)
        content.setVerticalSpacing(6)
        content.setColumnStretch(0, 1)
        content.setColumnStretch(1, 1)
        content.setRowStretch(0, 1)
        content.setRowStretch(1, 0)
        root.addLayout(content, 1)

        thermal_group, self.thermal_video_label = self.create_video_group(
            "Thermal Image Stream", "Loading thermal video..."
        )
        content.addWidget(thermal_group, 0, 0)

        rgb_group, self.rgb_video_label = self.create_video_group(
            "RGB Object Detection Stream", "Loading RGB video..."
        )
        content.addWidget(rgb_group, 0, 1)

        thermal_analysis = QGroupBox("Real-Time Thermal Analysis")
        thermal_layout = QVBoxLayout(thermal_analysis)
        thermal_layout.setSpacing(5)
        thermal_note = QLabel(
            "IMAGE-DERIVED ESTIMATES  |  normalized values, not sensor telemetry"
        )
        thermal_note.setStyleSheet(
            "color: #60a5fa; font-size: 8pt; font-weight: 700;"
        )
        thermal_layout.addWidget(thermal_note)
        thermal_grid = QGridLayout()
        thermal_grid.setSpacing(6)
        thermal_layout.addLayout(thermal_grid)

        self.thermal_peak_card = MetricCard("Apparent Peak", "0.0", "%")
        self.hot_area_card = MetricCard("High-Intensity Area", "0.00", "%")
        self.thermal_contrast_card = MetricCard("Thermal Contrast", "0.0", "%")
        self.thermal_change_card = MetricCard("Frame Change", "0.00", "%")
        self.thermal_confidence_card = MetricCard("Thermal Confidence", "0.0", "%")
        self.thermal_decision_card = MetricCard("Thermal Decision", "OFF")
        thermal_cards = [
            self.thermal_peak_card,
            self.hot_area_card,
            self.thermal_contrast_card,
            self.thermal_change_card,
            self.thermal_confidence_card,
            self.thermal_decision_card,
        ]
        for index, card in enumerate(thermal_cards):
            thermal_grid.addWidget(card, index // 3, index % 3)
        content.addWidget(thermal_analysis, 1, 0)

        right_controls = QWidget()
        right_layout = QVBoxLayout(right_controls)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)

        rgb_analysis = QGroupBox("Real-Time RGB Fire Detection")
        rgb_layout = QVBoxLayout(rgb_analysis)
        rgb_layout.setSpacing(5)
        rgb_note = QLabel(
            "VIDEO-DETECTOR EVIDENCE  |  ON only while a bounding box is visible"
        )
        rgb_note.setStyleSheet(
            "color: #22d3ee; font-size: 8pt; font-weight: 700;"
        )
        rgb_layout.addWidget(rgb_note)
        rgb_grid = QGridLayout()
        rgb_grid.setSpacing(6)
        rgb_layout.addLayout(rgb_grid)

        self.rgb_overlay_card = MetricCard("Detection Overlay", "0.000", "%")
        self.rgb_regions_card = MetricCard("Alert Regions", "0")
        self.rgb_motion_card = MetricCard("Scene Motion", "0.00", "%")
        self.rgb_evidence_card = MetricCard("Visual Evidence", "0.0", "%")
        self.rgb_confidence_card = MetricCard("RGB Confidence", "0.0", "%")
        self.rgb_decision_card = MetricCard("RGB Decision", "OFF")
        rgb_cards = [
            self.rgb_overlay_card,
            self.rgb_regions_card,
            self.rgb_motion_card,
            self.rgb_evidence_card,
            self.rgb_confidence_card,
            self.rgb_decision_card,
        ]
        for index, card in enumerate(rgb_cards):
            rgb_grid.addWidget(card, index // 3, index % 3)
        right_layout.addWidget(rgb_analysis)

        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet("background-color: #b91c1c; color: white;")
        self.stop_btn.setMinimumHeight(44)
        self.stop_btn.setMaximumHeight(54)
        self.stop_btn.clicked.connect(self.toggle_pause)
        right_layout.addWidget(self.stop_btn)
        content.addWidget(right_controls, 1, 1)

    def create_video_group(self, title: str, placeholder: str):
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

    def open_videos(self) -> None:
        try:
            self.thermal_video = LoopingVideo(self.thermal_path)
            self.thermal_analyzer = ThermalFrameAnalyzer(self.thermal_video.fps)
            thermal_timer = QTimer(self)
            thermal_timer.timeout.connect(self.render_thermal_frame)
            thermal_timer.start(self.thermal_video.timer_interval_ms)
            self.video_timers.append(thermal_timer)
        except RuntimeError as exc:
            self.show_video_error(self.thermal_video_label, str(exc))

        try:
            self.rgb_video = LoopingVideo(self.rgb_path)
            self.rgb_analyzer = RGBFrameAnalyzer(self.rgb_video.fps)
            rgb_timer = QTimer(self)
            rgb_timer.timeout.connect(self.render_rgb_frame)
            rgb_timer.start(self.rgb_video.timer_interval_ms)
            self.video_timers.append(rgb_timer)
        except RuntimeError as exc:
            self.show_video_error(self.rgb_video_label, str(exc))

    def show_video_error(self, label: QLabel, message: str) -> None:
        label.setText(message)
        label.setStyleSheet(
            "background-color: #020617; border: 2px solid #b91c1c; "
            "border-radius: 12px; color: #fca5a5; font-size: 12pt;"
        )

    def resize_to_screen(self) -> None:
        screen = QApplication.primaryScreen()
        if screen is None:
            self.setGeometry(60, 40, 1400, 820)
            return

        available = screen.availableGeometry()
        width = min(1600, int(available.width() * 0.96))
        height = min(930, int(available.height() * 0.96))
        left = available.x() + max(0, (available.width() - width) // 2)
        top = available.y() + max(0, (available.height() - height) // 2)
        self.setGeometry(left, top, width, height)

    def render_thermal_frame(self) -> None:
        if self.thermal_video is None or self.thermal_analyzer is None:
            return
        frame, looped = self.thermal_video.read()
        if looped:
            self.thermal_analyzer.reset()
        if frame is None:
            return

        metrics = self.thermal_analyzer.analyze(frame)
        self.update_thermal_metrics(metrics)
        self.render_video_frame(
            frame, self.thermal_video_label, "THERMAL", metrics.detected
        )

    def render_rgb_frame(self) -> None:
        if self.rgb_video is None or self.rgb_analyzer is None:
            return
        frame, looped = self.rgb_video.read()
        if looped:
            self.rgb_analyzer.reset()
        if frame is None:
            return

        metrics = self.rgb_analyzer.analyze(frame)
        self.update_rgb_metrics(metrics)
        self.render_video_frame(frame, self.rgb_video_label, "RGB", metrics.detected)

    def update_thermal_metrics(self, metrics: ThermalMetrics) -> None:
        self.thermal_peak_card.set_value(f"{metrics.apparent_peak:.1f}")
        self.hot_area_card.set_value(f"{metrics.hot_area:.2f}")
        self.thermal_contrast_card.set_value(f"{metrics.contrast:.1f}")
        self.thermal_change_card.set_value(f"{metrics.frame_change:.2f}")
        self.thermal_confidence_card.set_value(f"{metrics.confidence:.1f}")
        self.thermal_confidence_card.set_alert_level(
            "critical" if metrics.detected else "normal"
        )
        self.thermal_decision_card.set_value("ON" if metrics.detected else "OFF")
        self.thermal_decision_card.set_alert_level(
            "critical" if metrics.detected else "safe"
        )
        self.thermal_detected = metrics.detected
        self.update_final_decision()

    def update_rgb_metrics(self, metrics: RGBMetrics) -> None:
        self.rgb_overlay_card.set_value(f"{metrics.overlay_ratio:.3f}")
        self.rgb_regions_card.set_value(str(metrics.alert_regions))
        self.rgb_motion_card.set_value(f"{metrics.scene_motion:.2f}")
        self.rgb_evidence_card.set_value(f"{metrics.visual_evidence:.1f}")
        self.rgb_confidence_card.set_value(f"{metrics.confidence:.1f}")
        self.rgb_confidence_card.set_alert_level(
            "critical" if metrics.detected else "normal"
        )
        self.rgb_decision_card.set_value("ON" if metrics.detected else "OFF")
        self.rgb_decision_card.set_alert_level(
            "critical" if metrics.detected else "safe"
        )
        self.rgb_detected = metrics.detected
        self.update_final_decision()

    def update_final_decision(self, force: bool = False) -> None:
        self.current_fire = (
            not self.paused and self.thermal_detected and self.rgb_detected
        )
        state = (self.thermal_detected, self.rgb_detected, self.paused)
        if force or state != self.last_banner_state:
            self.last_banner_state = state
            self.apply_banner_style()

    def animate_fire_banner(self) -> None:
        if self.current_fire:
            self.blink_on = not self.blink_on
            self.apply_banner_style()

    def apply_banner_style(self) -> None:
        if self.paused:
            self.status_label.setText("MONITORING PAUSED  |  FIRE OFF")
            self.status_label.setStyleSheet(
                "background-color: #713f12; color: #fef3c7; border-radius: 12px; "
                "font-size: 22pt; font-weight: 900; padding: 8px;"
            )
            return

        if self.current_fire:
            background = "#dc2626" if self.blink_on else "#7f1d1d"
            self.status_label.setText(
                "FIRE ON  |  THERMAL + RGB CONFIRMED  |  DEPLOYMENT REQUIRED"
            )
            self.status_label.setStyleSheet(
                f"background-color: {background}; color: #fff7ed; "
                "border-radius: 12px; font-size: 22pt; font-weight: 950; "
                "padding: 8px;"
            )
            return

        thermal_text = "ON" if self.thermal_detected else "OFF"
        rgb_text = "ON" if self.rgb_detected else "OFF"
        background = "#78350f" if (self.thermal_detected or self.rgb_detected) else "#1e293b"
        self.status_label.setText(
            f"MONITORING  |  THERMAL {thermal_text} + RGB {rgb_text}  |  FIRE OFF"
        )
        self.status_label.setStyleSheet(
            f"background-color: {background}; color: #e5edf7; "
            "border-radius: 12px; font-size: 20pt; font-weight: 900; "
            "padding: 8px;"
        )

    def render_video_frame(
        self, frame: np.ndarray, label: QLabel, source_name: str, source_on: bool
    ) -> None:
        frame_to_show = frame.copy()
        height, width = frame_to_show.shape[:2]

        badge_text = f"{source_name} {'ON' if source_on else 'OFF'}"
        badge_color = (0, 80, 255) if source_on else (50, 205, 50)
        text_size, _ = cv2.getTextSize(
            badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2
        )
        badge_x = max(10, width - text_size[0] - 26)
        cv2.rectangle(
            frame_to_show,
            (badge_x - 8, 10),
            (width - 10, 46),
            (10, 15, 25),
            -1,
        )
        cv2.putText(
            frame_to_show,
            badge_text,
            (badge_x, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            badge_color,
            2,
            cv2.LINE_AA,
        )

        if self.current_fire:
            thickness = max(3, min(width, height) // 100)
            cv2.rectangle(
                frame_to_show,
                (8, 8),
                (width - 8, height - 8),
                (0, 0, 255),
                thickness,
            )

        rgb_image = cv2.cvtColor(frame_to_show, cv2.COLOR_BGR2RGB)
        image_height, image_width, channels = rgb_image.shape
        qt_image = QImage(
            rgb_image.data,
            image_width,
            image_height,
            channels * image_width,
            QImage.Format_RGB888,
        ).copy()
        pixmap = QPixmap.fromImage(qt_image).scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        label.setPixmap(pixmap)

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        for timer in self.video_timers:
            if self.paused:
                timer.stop()
            else:
                timer.start()

        if self.paused:
            self.thermal_detected = False
            self.rgb_detected = False
            self.stop_btn.setText("RESUME MONITORING")
            self.stop_btn.setStyleSheet(
                "background-color: #166534; color: white;"
            )
        else:
            if self.thermal_analyzer is not None:
                self.thermal_analyzer.reset()
            if self.rgb_analyzer is not None:
                self.rgb_analyzer.reset()
            self.stop_btn.setText("EMERGENCY STOP")
            self.stop_btn.setStyleSheet(
                "background-color: #b91c1c; color: white;"
            )
        self.update_final_decision(force=True)

    def closeEvent(self, event) -> None:
        if self.thermal_video is not None:
            self.thermal_video.release()
        if self.rgb_video is not None:
            self.rgb_video.release()
        event.accept()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor recorded thermal/RGB videos using frame-derived metrics."
    )
    parser.add_argument(
        "--thermal-video",
        default=THERMAL_VIDEO_PATH,
        help=f"Thermal video path (default: {THERMAL_VIDEO_PATH})",
    )
    parser.add_argument(
        "--rgb-video",
        default=RGB_VIDEO_PATH,
        help=f"RGB detector video path (default: {RGB_VIDEO_PATH})",
    )
    # Preserve Qt-specific arguments such as -platform without rejecting them.
    args, _ = parser.parse_known_args()
    args.thermal_video = os.path.abspath(os.path.expanduser(args.thermal_video))
    args.rgb_video = os.path.abspath(os.path.expanduser(args.rgb_video))
    return args


def main() -> None:
    args = parse_args()
    app = QApplication(sys.argv)
    app.setFont(QFont("Arial", 10))
    main_window = MainWindow(args.thermal_video, args.rgb_video)
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
