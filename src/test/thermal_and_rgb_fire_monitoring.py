"""Video-derived thermal and RGB fire monitoring dashboard.

This UI is intended for recorded videos that do not have matching sensor logs.
It calculates repeatable visual metrics directly from every video frame:

* Thermal: Avg, Max, Trend, Hold, and Fire ON/OFF values read from the HUD
  already burned into the supplied video.
* RGB: cyan detector-overlay ratio, large detection regions, frame motion, and
  a visual evidence score that is active only while a bounding box is visible.

The displayed values are image-derived estimates, not radiometric temperature
measurements or raw neural-network confidence values.  The final FIRE ON state
is controlled by the RGB decision after it remains ON for 0.2 seconds, and an
activated alert stays ON for at least 0.6 seconds.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys
import time
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
FIRE_CONFIRM_SECONDS = 0.2
FIRE_MIN_ON_SECONDS = 0.6


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
    average_temperature: Optional[float] = None
    maximum_temperature: Optional[float] = None
    temperature_trend: Optional[float] = None
    hold_seconds: float = 0.0
    detected: bool = False


@dataclass
class RGBMetrics:
    overlay_ratio: float = 0.0
    alert_regions: int = 0
    scene_motion: float = 0.0
    visual_evidence: float = 0.0
    confidence: float = 0.0
    detected: bool = False


class ThermalHudReader:
    """Read the fixed OpenCV HUD burned into the recorded thermal video.

    The source node writes the values with FONT_HERSHEY_SIMPLEX.  A small
    synthetic glyph set is therefore enough to recognize the digits without
    adding Tesseract or another OCR dependency.
    """

    NORMALIZED_SIZE = (600, 600)
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.735

    def __init__(self, fps: float):
        self.dt = 1.0 / max(fps, 1.0)
        self.white_templates = self._make_digit_templates(thickness=1)
        self.color_templates = self._make_digit_templates(thickness=3)
        self.reset()

    def reset(self) -> None:
        self.average_temperature: Optional[float] = None
        self.maximum_temperature: Optional[float] = None
        self.temperature_trend: Optional[float] = None
        self.hold_seconds = 0.0
        self.hold_initialized = False
        self.last_fire: Optional[bool] = None
        self.transition_grace = 0

    @classmethod
    def _normalize_glyph(cls, glyph: np.ndarray) -> np.ndarray:
        ys, xs = np.where(glyph > 0)
        output = np.zeros((28, 20), dtype=np.uint8)
        if len(xs) == 0:
            return output

        cropped = glyph[ys.min() : ys.max() + 1, xs.min() : xs.max() + 1]
        scale = min(18.0 / cropped.shape[1], 24.0 / cropped.shape[0])
        resized = cv2.resize(
            cropped,
            (
                max(1, round(cropped.shape[1] * scale)),
                max(1, round(cropped.shape[0] * scale)),
            ),
            interpolation=cv2.INTER_NEAREST,
        )
        y = (output.shape[0] - resized.shape[0]) // 2
        x = (output.shape[1] - resized.shape[1]) // 2
        output[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
        return output

    @classmethod
    def _make_digit_templates(cls, thickness: int) -> dict[str, np.ndarray]:
        templates = {}
        for digit in "0123456789":
            canvas = np.zeros((40, 40), dtype=np.uint8)
            cv2.putText(
                canvas,
                digit,
                (5, 28),
                cls.FONT,
                cls.FONT_SCALE,
                255,
                thickness,
            )
            templates[digit] = cls._normalize_glyph(canvas)
        return templates

    @staticmethod
    def _glyph_distance(first: np.ndarray, second: np.ndarray) -> float:
        first_mask = first > 0
        second_mask = second > 0
        if not first_mask.any() or not second_mask.any():
            return float("inf")
        first_distance = cv2.distanceTransform(
            (~first_mask).astype(np.uint8), cv2.DIST_L2, 3
        )
        second_distance = cv2.distanceTransform(
            (~second_mask).astype(np.uint8), cv2.DIST_L2, 3
        )
        return float(
            first_distance[second_mask].mean()
            + second_distance[first_mask].mean()
        )

    def _classify_digit(
        self,
        binary: np.ndarray,
        component: tuple[int, int, int, int],
        colored: bool = False,
    ) -> Optional[str]:
        x, y, width, height = component
        if height < 10 or not 5 <= width <= 18:
            return None
        glyph = self._normalize_glyph(binary[y : y + height, x : x + width])
        templates = self.color_templates if colored else self.white_templates
        ranked = sorted(
            (self._glyph_distance(glyph, template), digit)
            for digit, template in templates.items()
        )
        if not ranked or ranked[0][0] > 3.2:
            return None
        return ranked[0][1]

    @staticmethod
    def _white_components(
        frame: np.ndarray, crop: tuple[int, int, int, int]
    ) -> tuple[np.ndarray, list[tuple[int, int, int, int]]]:
        y0, y1, x0, x1 = crop
        gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        top_hat = cv2.morphologyEx(
            gray, cv2.MORPH_TOPHAT, np.ones((17, 17), dtype=np.uint8)
        )
        _, binary = cv2.threshold(
            top_hat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        components = [
            cv2.boundingRect(contour)
            for contour in contours
            if cv2.contourArea(contour) >= 1.0
        ]
        return binary, sorted(components)

    def _nearest_digit(
        self,
        binary: np.ndarray,
        components: list[tuple[int, int, int, int]],
        target_x: int,
        colored: bool = False,
    ) -> Optional[str]:
        candidates = []
        for component in components:
            digit = self._classify_digit(binary, component, colored=colored)
            if digit is not None and abs(component[0] - target_x) <= 4:
                candidates.append((abs(component[0] - target_x), digit))
        return min(candidates)[1] if candidates else None

    def _fixed_patch_digit(
        self, binary: np.ndarray, target_x: int
    ) -> Optional[str]:
        patch = binary[3:27, max(0, target_x - 1) : target_x + 14]
        glyph = self._normalize_glyph(patch)
        ranked = sorted(
            (self._glyph_distance(glyph, template), digit)
            for digit, template in self.white_templates.items()
        )
        if not ranked or ranked[0][0] > 3.2:
            return None
        return ranked[0][1]

    def _read_digit_at(
        self,
        binary: np.ndarray,
        components: list[tuple[int, int, int, int]],
        target_x: int,
    ) -> Optional[str]:
        digit = self._nearest_digit(binary, components, target_x)
        if digit is not None:
            return digit
        return self._fixed_patch_digit(binary, target_x)

    def _read_one_decimal(
        self, frame: np.ndarray, crop: tuple[int, int, int, int]
    ) -> Optional[float]:
        binary, components = self._white_components(frame, crop)
        digits = [
            self._read_digit_at(binary, components, x) for x in (4, 19, 40)
        ]
        if any(digit is None for digit in digits):
            return None
        return float(f"{digits[0]}{digits[1]}.{digits[2]}")

    def _read_trend(self, frame: np.ndarray) -> Optional[float]:
        binary, components = self._white_components(frame, (55, 82, 85, 170))
        is_negative = any(
            4 <= x <= 12 and height <= 6 and width >= 8
            for x, _, width, height in components
        )
        slots = (25, 45, 60) if is_negative else (8, 28, 43)
        digits = [self._read_digit_at(binary, components, x) for x in slots]
        if any(digit is None for digit in digits):
            return None
        sign = "-" if is_negative else ""
        return float(f"{sign}{digits[0]}.{digits[1]}{digits[2]}")

    def _read_hold(self, frame: np.ndarray) -> Optional[float]:
        roi = frame[80:108, 72:200]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        binary = cv2.inRange(hsv, (35, 60, 80), (115, 255, 255))
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        components = [
            cv2.boundingRect(contour)
            for contour in contours
            if cv2.contourArea(contour) >= 1.0
        ]

        # The trailing "s" moves with the value width, so it tells us whether
        # Hold currently contains one, two, or three integer digits.
        suffix_positions = [
            x
            for x, _, width, height in components
            if x >= 64 and width >= 8 and height >= 10
        ]
        if not suffix_positions:
            return None
        suffix_match = min(
            (abs(x - expected), expected)
            for x in suffix_positions
            for expected in (68, 82, 96)
        )
        if suffix_match[0] > 5:
            return None
        suffix_x = suffix_match[1]
        if suffix_x == 96:
            slots = (6, 22, 36, 56, 71)
            decimal_index = 3
        elif suffix_x == 82:
            slots = (7, 21, 42, 56)
            decimal_index = 2
        else:
            slots = (8, 28, 42)
            decimal_index = 1

        ranked_slots = []
        for target_x in slots:
            patch = binary[3:27, max(0, target_x - 1) : target_x + 14]
            glyph = self._normalize_glyph(patch)
            ranked = sorted(
                (self._glyph_distance(glyph, template), digit)
                for digit, template in self.color_templates.items()
            )[:2]
            if not ranked or ranked[0][0] > 3.2:
                return None
            ranked_slots.append(ranked)

        candidates = []
        for selection in itertools.product(*ranked_slots):
            digits = [item[1] for item in selection]
            number_text = (
                "".join(digits[:decimal_index])
                + "."
                + "".join(digits[decimal_index:])
            )
            value = float(number_text)
            shape_score = sum(item[0] for item in selection)
            if self.hold_initialized:
                score = shape_score + abs(value - self.hold_seconds) * 0.10
            else:
                score = shape_score
            candidates.append((score, value))

        if not candidates:
            return None
        return min(candidates)[1]

    def read(
        self, frame: np.ndarray, fire_on: bool, run_ocr: bool
    ) -> tuple[Optional[float], Optional[float], Optional[float], float]:
        if frame.shape[1] != 600 or frame.shape[0] != 600:
            frame = cv2.resize(frame, self.NORMALIZED_SIZE, interpolation=cv2.INTER_AREA)

        state_changed = self.last_fire is not None and fire_on != self.last_fire
        if state_changed:
            self.transition_grace = 5
        self.last_fire = fire_on

        if self.maximum_temperature is not None:
            if self.maximum_temperature >= 40.0:
                self.hold_seconds += self.dt
            else:
                self.hold_seconds = 0.0

        if not run_ocr:
            return (
                self.average_temperature,
                self.maximum_temperature,
                self.temperature_trend,
                self.hold_seconds,
            )

        average = self._read_one_decimal(frame, (5, 34, 64, 130))
        maximum = self._read_one_decimal(frame, (30, 57, 69, 138))
        trend = self._read_trend(frame)

        allow_jump = self.transition_grace > 0
        if maximum is not None and 0.0 <= maximum <= 99.9:
            average_floor = (
                average
                if average is not None and 0.0 <= average <= 99.9
                else (self.average_temperature or 0.0)
            )
            crossed_hot_threshold = (
                fire_on
                and self.maximum_temperature is not None
                and self.maximum_temperature < 40.0 <= maximum
            )
            if maximum + 0.5 >= average_floor and (
                self.maximum_temperature is None
                or allow_jump
                or crossed_hot_threshold
                or abs(maximum - self.maximum_temperature) <= 8.0
            ):
                self.maximum_temperature = maximum

        if average is not None and 0.0 <= average <= 99.9:
            below_maximum = (
                self.maximum_temperature is None
                or average <= self.maximum_temperature + 0.5
            )
            if below_maximum and (
                self.average_temperature is None
                or allow_jump
                or abs(average - self.average_temperature) <= 1.0
            ):
                self.average_temperature = average

        if trend is not None and -30.0 <= trend <= 30.0:
            self.temperature_trend = trend

        if self.maximum_temperature is not None and self.maximum_temperature < 40.0:
            self.hold_seconds = 0.0
        hold = self._read_hold(frame)
        if hold is not None and 0.0 <= hold <= 999.99:
            predicted_close = abs(hold - self.hold_seconds) <= 0.75
            reset_correction = (
                self.maximum_temperature is not None
                and self.maximum_temperature < 40.0
                and hold <= 0.1
            )
            if (
                not self.hold_initialized
                or allow_jump
                or predicted_close
                or reset_correction
            ):
                self.hold_seconds = hold
                self.hold_initialized = True

        if self.transition_grace > 0:
            self.transition_grace -= 1

        return (
            self.average_temperature,
            self.maximum_temperature,
            self.temperature_trend,
            self.hold_seconds,
        )


class ThermalFrameAnalyzer:
    """Calculate thermal-looking metrics from rendered video pixels.

    Absolute temperature cannot be recovered from an 8-bit rendered video.
    The supplied recording does, however, contain a colour-coded Fire ON/OFF
    HUD.  Its green/cyan cue is combined with generic intensity features so the
    dashboard follows the contents of the video rather than a timer script.
    """

    def __init__(self, fps: float):
        self.dt = 1.0 / max(fps, 1.0)
        self.ocr_interval = max(1, round(fps / 10.0))
        self.frame_number = 0
        self.hud_reader = ThermalHudReader(fps)
        self.previous_gray: Optional[np.ndarray] = None
        self.frame_change = 0.0
        self.confidence = 0.0
        self.detected = False

    def reset(self) -> None:
        self.frame_number = 0
        self.hud_reader.reset()
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

        if hud_on is not None:
            # Follow the recorded thermal Fire indicator on the current frame.
            self.detected = hud_on
            self.confidence = 100.0 if hud_on else 0.0
        else:
            self.confidence = exponential_step(
                self.confidence, target, self.dt, rise_tau=0.20, fall_tau=0.18
            )
            if not self.detected and self.confidence >= THERMAL_ON_THRESHOLD:
                self.detected = True
            elif self.detected and self.confidence <= THERMAL_OFF_THRESHOLD:
                self.detected = False

        run_ocr = self.frame_number % self.ocr_interval == 0
        average, maximum, trend, hold = self.hud_reader.read(
            frame, self.detected, run_ocr
        )
        self.frame_number += 1

        return ThermalMetrics(
            average_temperature=average,
            maximum_temperature=maximum,
            temperature_trend=trend,
            hold_seconds=hold,
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
        self.fire_candidate_started_at: Optional[float] = None
        self.fire_on_started_at: Optional[float] = None
        self.paused = False
        self.last_banner_state: Optional[tuple[bool, bool, bool, bool]] = None

        self.init_ui()
        self.open_videos()

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
        content.setRowStretch(2, 0)
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
        thermal_grid = QGridLayout(thermal_analysis)
        thermal_grid.setSpacing(6)

        self.thermal_avg_card = MetricCard("AVG", "--", "°C")
        self.thermal_max_card = MetricCard("MAX", "--", "°C")
        self.thermal_trend_card = MetricCard("TREND", "--", "°C/s")
        self.thermal_hold_card = MetricCard("HOLD", "0.00", "s")
        self.thermal_fire_card = MetricCard("FIRE", "OFF")
        thermal_cards = [
            self.thermal_avg_card,
            self.thermal_max_card,
            self.thermal_trend_card,
        ]
        for index, card in enumerate(thermal_cards):
            thermal_grid.addWidget(card, 0, index)
        thermal_grid.addWidget(self.thermal_hold_card, 1, 0)
        thermal_grid.addWidget(self.thermal_fire_card, 1, 1, 1, 2)
        content.addWidget(thermal_analysis, 1, 0)

        rgb_analysis = QGroupBox("Real-Time RGB Fire Detection")
        rgb_grid = QGridLayout(rgb_analysis)
        rgb_grid.setSpacing(6)

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
        content.addWidget(rgb_analysis, 1, 1)

        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet("background-color: #b91c1c; color: white;")
        self.stop_btn.setMinimumHeight(44)
        self.stop_btn.setMaximumHeight(54)
        self.stop_btn.clicked.connect(self.toggle_pause)
        content.addWidget(self.stop_btn, 2, 1)

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
        average_text = (
            f"{metrics.average_temperature:.1f}"
            if metrics.average_temperature is not None
            else "--"
        )
        maximum_text = (
            f"{metrics.maximum_temperature:.1f}"
            if metrics.maximum_temperature is not None
            else "--"
        )
        trend_text = (
            f"{metrics.temperature_trend:.2f}"
            if metrics.temperature_trend is not None
            else "--"
        )
        self.thermal_avg_card.set_value(average_text)
        self.thermal_max_card.set_value(maximum_text)
        self.thermal_trend_card.set_value(trend_text)
        self.thermal_hold_card.set_value(f"{metrics.hold_seconds:.2f}")

        if metrics.maximum_temperature is not None:
            if metrics.maximum_temperature >= 60.0:
                max_level = "critical"
            elif metrics.maximum_temperature >= 40.0:
                max_level = "warning"
            else:
                max_level = "normal"
            self.thermal_max_card.set_alert_level(max_level)
        trend_level = (
            "warning"
            if metrics.temperature_trend is not None
            and metrics.temperature_trend >= 0.3
            else "normal"
        )
        self.thermal_trend_card.set_alert_level(trend_level)
        self.thermal_fire_card.set_value("ON" if metrics.detected else "OFF")
        self.thermal_fire_card.set_alert_level(
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

    def update_final_decision(
        self, force: bool = False, now: Optional[float] = None
    ) -> None:
        timestamp = time.monotonic() if now is None else now
        rgb_source_on = not self.paused and self.rgb_detected

        if self.paused:
            self.fire_candidate_started_at = None
            self.fire_on_started_at = None
            self.current_fire = False
        elif self.current_fire:
            # Once activated, keep FIRE ON for at least 0.6 seconds even if the
            # RGB bounding box briefly disappears.
            on_elapsed = (
                timestamp - self.fire_on_started_at
                if self.fire_on_started_at is not None
                else FIRE_MIN_ON_SECONDS
            )
            if not rgb_source_on and on_elapsed + 1e-9 >= FIRE_MIN_ON_SECONDS:
                self.current_fire = False
                self.fire_on_started_at = None
                self.fire_candidate_started_at = None
        elif rgb_source_on:
            if self.fire_candidate_started_at is None:
                self.fire_candidate_started_at = timestamp
            confirmed = (
                timestamp - self.fire_candidate_started_at + 1e-9
                >= FIRE_CONFIRM_SECONDS
            )
            if confirmed:
                self.current_fire = True
                self.fire_on_started_at = timestamp
                self.fire_candidate_started_at = None
        else:
            self.fire_candidate_started_at = None
            self.current_fire = False

        state = (
            self.thermal_detected,
            self.rgb_detected,
            self.paused,
            self.current_fire,
        )
        if force or state != self.last_banner_state:
            self.last_banner_state = state
            self.apply_banner_style()

    def apply_banner_style(self) -> None:
        if self.current_fire:
            self.status_label.setText("FIRE ON")
            self.status_label.setStyleSheet(
                "background-color: #dc2626; color: #fff7ed; "
                "border-radius: 12px; font-size: 27pt; font-weight: 950; "
                "padding: 8px;"
            )
            return

        self.status_label.setText("FIRE OFF")
        self.status_label.setStyleSheet(
            "background-color: #166534; color: #ecfdf5; "
            "border-radius: 12px; font-size: 27pt; font-weight: 950; "
            "padding: 8px;"
        )

    def render_video_frame(
        self, frame: np.ndarray, label: QLabel, source_name: str, source_on: bool
    ) -> None:
        frame_to_show = frame.copy()
        width = frame_to_show.shape[1]

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
