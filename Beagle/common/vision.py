from __future__ import annotations

from dataclasses import dataclass
import os
import time
from typing import Iterable

import numpy as np


def require_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise RuntimeError("OpenCV가 필요합니다: python -m pip install opencv-contrib-python") from exc
    return cv2


def open_camera(camera_id: int = 0, width: int | None = None, height: int | None = None):
    cv2 = require_cv2()
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    cap = cv2.VideoCapture(camera_id, backend)
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    return cap


def parse_video_source(source: str | int) -> int | str:
    """문자열 카메라 번호는 int로, 그 외는 파일/HTTP/RTSP 소스로 반환합니다."""

    if isinstance(source, int):
        return source
    value = str(source).strip()
    return int(value) if value.lstrip("+-").isdigit() else value


def open_video_source(source: str | int = 0, width: int | None = None, height: int | None = None):
    """Windows 카메라 번호, 동영상 파일, HTTP/RTSP 스트림을 엽니다."""

    cv2 = require_cv2()
    parsed = parse_video_source(source)
    if isinstance(parsed, int):
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        cap = cv2.VideoCapture(parsed, backend)
    else:
        cap = cv2.VideoCapture(parsed)
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    return cap


def scan_camera_ids(max_id: int = 8) -> list[dict[str, object]]:
    cv2 = require_cv2()
    found: list[dict[str, object]] = []
    for camera_id in range(max_id + 1):
        cap = open_camera(camera_id)
        opened = bool(cap.isOpened())
        ok, frame = cap.read() if opened else (False, None)
        backend = ""
        if opened:
            try:
                backend = cap.getBackendName()
            except Exception:
                backend = "unknown"
        found.append(
            {
                "id": camera_id,
                "opened": opened,
                "frame_ok": bool(ok),
                "shape": tuple(frame.shape) if ok and frame is not None else None,
                "backend": backend,
            }
        )
        cap.release()
    return found


def normalized_roi(frame: np.ndarray, roi: tuple[float, float, float, float]) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = roi
    px1 = max(0, min(w - 1, int(round(x1 * w))))
    py1 = max(0, min(h - 1, int(round(y1 * h))))
    px2 = max(px1 + 1, min(w, int(round(x2 * w))))
    py2 = max(py1 + 1, min(h, int(round(y2 * h))))
    return frame[py1:py2, px1:px2], (px1, py1, px2, py2)


def hsv_mask(
    frame_bgr: np.ndarray,
    ranges: Iterable[tuple[tuple[int, int, int], tuple[int, int, int]]],
    *,
    kernel_size: int = 5,
) -> np.ndarray:
    cv2 = require_cv2()
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    masks = [
        cv2.inRange(hsv, np.asarray(lower, dtype=np.uint8), np.asarray(upper, dtype=np.uint8))
        for lower, upper in ranges
    ]
    if not masks:
        return np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
    mask = masks[0]
    for other in masks[1:]:
        mask = cv2.bitwise_or(mask, other)
    if kernel_size > 1:
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def red_mask(frame_bgr: np.ndarray, kernel_size: int = 5) -> np.ndarray:
    return hsv_mask(
        frame_bgr,
        [((0, 100, 80), (10, 255, 255)), ((170, 100, 80), (179, 255, 255))],
        kernel_size=kernel_size,
    )


def green_mask(frame_bgr: np.ndarray, kernel_size: int = 5) -> np.ndarray:
    return hsv_mask(frame_bgr, [((35, 70, 60), (90, 255, 255))], kernel_size=kernel_size)


def mask_ratio(mask: np.ndarray) -> float:
    cv2 = require_cv2()
    return float(cv2.countNonZero(mask)) / max(1, int(mask.size))


def red_ratio(
    frame_bgr: np.ndarray,
    roi: tuple[float, float, float, float] = (0.15, 0.25, 0.85, 0.92),
) -> tuple[float, np.ndarray, tuple[int, int, int, int]]:
    cropped, box = normalized_roi(frame_bgr, roi)
    mask = red_mask(cropped)
    return mask_ratio(mask), mask, box


def largest_contour_center(mask: np.ndarray, min_area: float = 200.0) -> tuple[float, float, float] | None:
    cv2 = require_cv2()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return None
    moment = cv2.moments(contour)
    if abs(moment["m00"]) < 1e-9:
        return None
    return float(moment["m10"] / moment["m00"]), float(moment["m01"] / moment["m00"]), area


def line_center(
    frame_bgr: np.ndarray,
    *,
    roi: tuple[float, float, float, float] = (0.0, 0.55, 1.0, 1.0),
    mode: str = "black",
) -> tuple[float | None, np.ndarray, tuple[int, int, int, int]]:
    cv2 = require_cv2()
    cropped, box = normalized_roi(frame_bgr, roi)
    if mode == "black":
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, 80)
    elif mode == "red":
        mask = red_mask(cropped)
    else:
        mask = green_mask(cropped)
    center = largest_contour_center(mask, min_area=120.0)
    return (center[0] if center else None), mask, box


def synthetic_signal_frame(width: int = 640, height: int = 480, t: float | None = None) -> np.ndarray:
    cv2 = require_cv2()
    t = time.monotonic() if t is None else t
    frame = np.full((height, width, 3), 235, dtype=np.uint8)
    phase = t % 9.0
    if phase < 3.0:
        color, label = (0, 200, 0), "GREEN"
    elif phase < 6.0:
        color, label = (0, 0, 255), "RED"
    else:
        color, label = (160, 160, 160), "NONE"
    cv2.rectangle(frame, (220, 130), (420, 390), color, -1)
    cv2.putText(frame, label, (255, 275), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return frame


def synthetic_target_frame(width: int = 640, height: int = 480, t: float | None = None) -> np.ndarray:
    cv2 = require_cv2()
    t = time.monotonic() if t is None else t
    frame = np.full((height, width, 3), 245, dtype=np.uint8)
    center_x = int(width / 2 + (width * 0.32) * np.sin(t * 0.8))
    cv2.circle(frame, (center_x, int(height * 0.58)), 55, (0, 0, 255), -1)
    return frame


def synthetic_line_frame(width: int = 640, height: int = 480, t: float | None = None) -> np.ndarray:
    cv2 = require_cv2()
    t = time.monotonic() if t is None else t
    frame = np.full((height, width, 3), 245, dtype=np.uint8)
    offset = int(width * 0.18 * np.sin(t * 0.7))
    cv2.line(frame, (width // 2 + offset - 40, height), (width // 2 + offset + 20, int(height * 0.50)), (20, 20, 20), 28)
    return frame


@dataclass(slots=True)
class Detection:
    label: str
    confidence: float
    xyxy: tuple[int, int, int, int]
