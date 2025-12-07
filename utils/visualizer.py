"""
Visualizer

디버깅용 OpenCV 기반 시각화 유틸.
- 카메라 프레임 / 디텍션 결과 등을 띄우는 용도
"""

from __future__ import annotations

import logging
from typing import Tuple, Optional, Iterable

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class Visualizer:

    def __init__(
        self,
        window_name: str = "Drone Debug",
        enabled: bool = True,
        window_size: Optional[Tuple[int, int]] = (960, 540),
    ) -> None:
        self.window_name = window_name
        self.enabled = enabled
        self.window_size = window_size

        if self.enabled:
            self._create_window()

    # ------------------------------------------------------------------
    # 창 관리
    # ------------------------------------------------------------------
    def _create_window(self) -> None:
        """OpenCV 윈도우를 한 번만 생성."""
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            if self.window_size is not None:
                w, h = self.window_size
                cv2.resizeWindow(self.window_name, w, h)
        except cv2.error as e:
            logger.exception("Visualizer: failed to create window: %s", e)

            self.enabled = False

    def close(self) -> None:
        if not self.enabled:
            return

        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass
        finally:
            self.enabled = False
            logger.info("Visualizer closed")

    # ------------------------------------------------------------------
    # 그리기 + 표시
    # ------------------------------------------------------------------
    def draw_text(
        self,
        frame: np.ndarray,
        text: str,
        org: Tuple[int, int] = (10, 30),
        scale: float = 0.7,
        thickness: int = 2,
    ) -> np.ndarray:
        if frame is None:
            return frame

        cv2.putText(
            frame,
            text,
            org,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 255, 0),  
            thickness,
            cv2.LINE_AA,
        )
        return frame

    def draw_bboxes(
        self,
        frame: np.ndarray,
        boxes: Iterable[Tuple[int, int, int, int]],
        color: Tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
    ) -> np.ndarray:

        if frame is None:
            return frame

        for (x, y, w, h) in boxes:
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
        return frame

    def show(
        self,
        frame: Optional[np.ndarray],
        rgb: bool = False,
        wait_ms: int = 1,
    ) -> int:

        if not self.enabled:
            return -1

        if frame is None:
            logger.warning("Visualizer.show: received None frame, skip")
            return -1

        # RGB 입력을 받는 경우 변환
        if rgb:
            try:
                frame_to_show = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            except cv2.error as e:
                logger.exception("Visualizer: COLOR_RGB2BGR failed: %s", e)
                frame_to_show = frame
        else:
            frame_to_show = frame

        try:
            cv2.imshow(self.window_name, frame_to_show)
        except cv2.error as e:
            logger.exception("Visualizer: imshow failed: %s", e)
            return -1

        key = cv2.waitKey(wait_ms) & 0xFF
        return key

    def should_close(self, key: int, exit_keys: Optional[Iterable[int]] = None) -> bool:

        if not self.enabled:
            return False

        if exit_keys is None:
            exit_keys = (ord("q"), 27) 

        return key in exit_keys
