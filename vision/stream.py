from __future__ import annotations

from typing import Optional, Dict, Any

import cv2


class VideoStream:
    """
    camera.yaml 구조를 그대로 받아서 카메라/스트림을 여는 래퍼.

    source_type: "USB" / "CSI" / "FILE" / "RTSP"
      - USB / CSI : device_id 사용 (0, 1, ...)
      - FILE      : video_path 사용
      - RTSP      : rtsp_url 사용

    processing:
      - flip_horizontal: 좌우 반전
      - flip_vertical:   상하 반전
      - rotation:        0, 90, 180, 270
    """

    def __init__(
        self,
        source_type: str,
        device_id: int,
        video_path: str,
        rtsp_url: str,
        width: int,
        height: int,
        fps: int,
        processing_cfg: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.source_type = source_type.upper()
        self.device_id = device_id
        self.video_path = video_path
        self.rtsp_url = rtsp_url

        self.width = int(width) if width else 0
        self.height = int(height) if height else 0
        self.fps = int(fps) if fps else 0

        self.processing_cfg = processing_cfg or {}
        self.cap: Optional[cv2.VideoCapture] = None

        if self.source_type in ("USB", "CSI") and self.device_id is None:
            raise ValueError("USB/CSI source_type requires device_id")
        if self.source_type == "FILE" and not self.video_path:
            raise ValueError("FILE source_type requires video_path")
        if self.source_type == "RTSP" and not self.rtsp_url:
            raise ValueError("RTSP source_type requires rtsp_url")

    # -----------------------------
    # 내부: OpenCV source 선택
    # -----------------------------
    def _get_source_for_opencv(self):
        if self.source_type in ("USB", "CSI"):
            return self.device_id
        elif self.source_type == "FILE":
            return self.video_path
        elif self.source_type == "RTSP":
            return self.rtsp_url
        else:
            raise ValueError(f"Unknown camera source_type: {self.source_type}")

    # -----------------------------
    # life-cycle
    # -----------------------------
    def open(self) -> None:
        """카메라/스트림을 연다."""
        src = self._get_source_for_opencv()
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            raise RuntimeError(f"Failed to open camera source: {src}")

        if self.width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps:
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def read(self):
        if self.cap is None:
            raise RuntimeError("VideoStream not opened. Call open() first.")

        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to read frame from camera.")

        frame = self._apply_processing(frame)
        return frame

    # -----------------------------
    # 후처리
    # -----------------------------
    def _apply_processing(self, frame):
        flip_h = self.processing_cfg.get("flip_horizontal", False)
        flip_v = self.processing_cfg.get("flip_vertical", False)
        rotation = self.processing_cfg.get("rotation", 0)

        if flip_h:
            frame = cv2.flip(frame, 1)
        if flip_v:
            frame = cv2.flip(frame, 0)

        if rotation == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotation == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotation == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        return frame

    # -----------------------------
    # 종료 / context manager
    # -----------------------------
    def release(self) -> None:
        """스트림 리소스를 해제한다."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
