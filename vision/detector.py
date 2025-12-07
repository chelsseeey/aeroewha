"""
Object Detector Module
Handles object detection using YOLO, color-based methods, etc.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Optional, Iterable

import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """단일 객체 검출 결과"""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]   # (x, y, w, h)
    center: Tuple[int, int]           # (cx, cy)


class ObjectDetector:
    """
    객체 검출기 래퍼.

    method:
        - "yolo"  : ultralytics YOLO 사용
        - "color" : HSV 컬러 threshold 기반 검출
    """

    def __init__(self, method: str = "yolo", **kwargs) -> None:
        self.method = method.lower()
        self.kwargs = kwargs

        # YOLO 관련
        self.yolo_model = None
        self.conf_threshold: float = float(self.kwargs.get("conf_threshold", 0.25))
        self.max_detections: int = int(self.kwargs.get("max_detections", 50))

        # Color detector 관련
        # 기본은 빨간색 영역 예시 (두 구간: 0~10, 170~180)
        hsv_ranges = self.kwargs.get(
            "hsv_ranges",
            [
                ((0, 100, 100), (10, 255, 255)),
                ((170, 100, 100), (180, 255, 255)),
            ],
        )
        # 단일 lower/upper_hsv 만 들어왔을 경우 처리
        if "lower_hsv" in self.kwargs and "upper_hsv" in self.kwargs:
            hsv_ranges = [
                (
                    self.kwargs["lower_hsv"],
                    self.kwargs["upper_hsv"],
                )
            ]
        self.hsv_ranges = hsv_ranges
        self.min_area: int = int(self.kwargs.get("min_area", 500))

        if self.method == "yolo":
            self._init_yolo()
        elif self.method == "color":
            self._init_color_detector()
        else:
            logger.warning("Unknown detection method '%s'", method)

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    def _init_yolo(self) -> None:
        """Initialize YOLO detector."""
        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError:
            logger.error(
                "ultralytics가 설치되어 있지 않습니다. "
                "YOLO를 사용하려면 'pip install ultralytics'를 실행하세요."
            )
            raise

        model_path = self.kwargs.get("model_path", "yolov8n.pt")
        try:
            self.yolo_model = YOLO(model_path)
            logger.info("YOLO detector initialized: %s", model_path)
        except Exception:
            logger.exception("Failed to load YOLO model from %s", model_path)
            raise

    def _init_color_detector(self) -> None:
        """Initialize color-based detector."""
        logger.info("Color-based detector initialized with %d HSV range(s)", len(self.hsv_ranges))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Detect objects in a frame.

        Args:
            frame: BGR 이미지 (OpenCV)

        Returns:
            Detection 리스트
        """
        if frame is None:
            return []

        if self.method == "yolo" and self.yolo_model is not None:
            return self._detect_yolo(frame)
        elif self.method == "color":
            return self._detect_color(frame)
        else:
            return []

    # ------------------------------------------------------------------
    # YOLO detection
    # ------------------------------------------------------------------
    def _detect_yolo(self, frame: np.ndarray) -> List[Detection]:
        """Detect objects using YOLO."""
        detections: List[Detection] = []

        if self.yolo_model is None:
            logger.error("YOLO model is not initialized.")
            return detections

        try:
            results = self.yolo_model(frame, verbose=False)
            for result in results:
                boxes = getattr(result, "boxes", None)
                names = getattr(result, "names", {}) or {}

                if boxes is None:
                    continue

                for box in boxes:
                    # box.xyxy, box.conf, box.cls는 tensor 형태
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])

                    if conf < self.conf_threshold:
                        continue

                    cls_name = names.get(cls_id, str(cls_id))

                    x, y = x1, y1
                    w = max(0, x2 - x1)
                    h = max(0, y2 - y1)
                    cx = x + w // 2
                    cy = y + h // 2

                    detections.append(
                        Detection(
                            class_id=cls_id,
                            class_name=cls_name,
                            confidence=conf,
                            bbox=(x, y, w, h),
                            center=(cx, cy),
                        )
                    )

                    if len(detections) >= self.max_detections:
                        break
        except Exception:
            logger.exception("YOLO detection failed")

        return detections

    # ------------------------------------------------------------------
    # Color-based detection
    # ------------------------------------------------------------------
    def _detect_color(self, frame: np.ndarray) -> List[Detection]:
        """Detect objects using HSV color thresholding."""
        detections: List[Detection] = []

        # BGR → HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 여러 HSV 구간을 OR 해서 하나의 최종 mask 생성
        mask_total = None
        for (lower, upper) in self.hsv_ranges:
            lower_arr = np.array(lower, dtype=np.uint8)
            upper_arr = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_arr, upper_arr)
            if mask_total is None:
                mask_total = mask
            else:
                mask_total = cv2.bitwise_or(mask_total, mask)

        if mask_total is None:
            return detections

        # Morphological operations (노이즈 제거)
        kernel = np.ones((5, 5), np.uint8)
        mask_processed = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)
        mask_processed = cv2.morphologyEx(mask_processed, cv2.MORPH_CLOSE, kernel)

        # Contour 탐색
        contours, _ = cv2.findContours(
            mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2

            detections.append(
                Detection(
                    class_id=0,
                    class_name="target",
                    confidence=1.0,  # 색 기반이므로 일단 1.0 부여 (필요 시 면적 비율 등으로 조정 가능)
                    bbox=(x, y, w, h),
                    center=(cx, cy),
                )
            )

        return detections

    # ------------------------------------------------------------------
    # Helper 
    # ------------------------------------------------------------------
    @staticmethod
    def select_best(
        detections: Iterable[Detection],
        strategy: str = "highest_conf",
    ) -> Optional[Detection]:
        """
        여러 Detection 중에서 하나를 고르는 helper.

        strategy:
            - "highest_conf": confidence 최대
            - "largest":      bbox 면적 최대
        """
        detections = list(detections)
        if not detections:
            return None

        if strategy == "largest":
            return max(detections, key=lambda d: d.bbox[2] * d.bbox[3])
        else:
            return max(detections, key=lambda d: d.confidence)
