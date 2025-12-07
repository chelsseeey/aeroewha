"""
Object Tracker Module
Handles object tracking and smoothing using KCF/CSRT/Kalman filters
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)


@dataclass
class TrackingResult:
    """단일 프레임에서의 추적 결과"""
    success: bool
    bbox: Optional[Tuple[int, int, int, int]] = None  # (x, y, w, h)
    center: Optional[Tuple[int, int]] = None          # (cx, cy)
    confidence: float = 0.0


class ObjectTracker:
    """OpenCV Tracker + Kalman smoothing 래퍼."""

    def __init__(self, tracker_type: str = "KCF") -> None:
        """
        Args:
            tracker_type: 'KCF', 'CSRT', 'MOSSE'
        """
        self.tracker_type = tracker_type.upper()
        self.tracker: Optional[object] = None
        self.is_initialized: bool = False

        # Kalman filter for smoothing (state: [x, y, vx, vy])
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], np.float32
        )
        self.kalman.transitionMatrix = np.array(
            [
                [1, 0, 1, 0],
                [0, 1, 0, 1],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            np.float32,
        )
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

    # ------------------------------------------------------------------
    # 내부: tracker 생성
    # ------------------------------------------------------------------
    def _create_tracker(self) -> object:
        """OpenCV tracker 객체 생성 (버전 차이 고려)."""
        t = self.tracker_type
        tracker = None

        if t == "KCF":
            tracker = getattr(cv2, "TrackerKCF_create", None)
            if tracker is None and hasattr(cv2, "legacy"):
                tracker = getattr(cv2.legacy, "TrackerKCF_create", None)
        elif t == "CSRT":
            tracker = getattr(cv2, "TrackerCSRT_create", None)
            if tracker is None and hasattr(cv2, "legacy"):
                tracker = getattr(cv2.legacy, "TrackerCSRT_create", None)
        elif t == "MOSSE":
            # MOSSE 는 보통 legacy 네임스페이스에 존재
            tracker = getattr(cv2, "TrackerMOSSE_create", None)
            if tracker is None and hasattr(cv2, "legacy"):
                tracker = getattr(cv2.legacy, "TrackerMOSSE_create", None)

        if tracker is None:
            raise RuntimeError(f"Unsupported tracker type or OpenCV build: {t}")

        return tracker()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def init(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> bool:
        """
        Initialize tracker with first frame and bbox.

        Args:
            frame: 첫 프레임 (BGR)
            bbox:  초기 바운딩 박스 (x, y, w, h)
        """
        if frame is None:
            logger.error("Cannot initialize tracker: frame is None")
            return False

        try:
            tracker = self._create_tracker()
            ok = tracker.init(frame, bbox)
            if not ok:
                logger.error("OpenCV tracker.init() failed")
                return False

            self.tracker = tracker

            x, y, w, h = bbox
            cx = x + w / 2.0
            cy = y + h / 2.0
            state = np.array([[cx], [cy], [0.0], [0.0]], np.float32)
            self.kalman.statePre = state.copy()
            self.kalman.statePost = state.copy()

            self.is_initialized = True
            logger.info("Tracker initialized: %s", self.tracker_type)
            return True

        except Exception:
            logger.exception("Error initializing tracker")
            self.tracker = None
            self.is_initialized = False
            return False

    def update(self, frame: np.ndarray) -> TrackingResult:
        """
        Update tracker with new frame.

        Args:
            frame: 현재 프레임 (BGR)

        Returns:
            TrackingResult
        """
        if not self.is_initialized or self.tracker is None:
            return TrackingResult(False, None, None, 0.0)

        try:
            success, bbox = self.tracker.update(frame)
            if not success or bbox is None:
                return TrackingResult(False, None, None, 0.0)

            x, y, w, h = [int(v) for v in bbox]
            cx = x + w // 2
            cy = y + h // 2

            # Kalman smoothing
            _ = self.kalman.predict()
            measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
            corrected = self.kalman.correct(measurement)

            smooth_cx = int(corrected[0, 0])
            smooth_cy = int(corrected[1, 0])

            return TrackingResult(
                success=True,
                bbox=(x, y, w, h),
                center=(smooth_cx, smooth_cy),
                confidence=1.0,  # 필요시 later: quality score로 변경 가능
            )

        except Exception:
            logger.exception("Error updating tracker")
            return TrackingResult(False, None, None, 0.0)

    def reset(self) -> None:
        """Reset tracker."""
        self.tracker = None
        self.is_initialized = False
        self.kalman.statePre = np.zeros((4, 1), dtype=np.float32)
        self.kalman.statePost = np.zeros((4, 1), dtype=np.float32)
        logger.info("Tracker reset")
