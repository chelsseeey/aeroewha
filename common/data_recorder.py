from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional

import cv2

from .types import ControlCommand, DroneState, TargetObservation, now


class DataRecorder:
    """
    실행 로그(텔레메트리 + 영상)를 저장하는 기록기.

    - CSV : 타임스텝별 상태/제어/타겟 정보를 행 단위로 기록
    - Video : 디버깅용 오버레이 영상 저장 
    """

    def __init__(self, log_dir: str, run_name: str = "run") -> None:
        self.base_dir = Path(log_dir) / run_name
        self.base_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.base_dir / "log.csv"
        self.video_path = self.base_dir / "video.mp4"

        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv_writer = csv.DictWriter(
            self._csv_file,
            fieldnames=[
                "t",
                "mode",
                "target_detected",
                "target_cx",
                "target_cy",
                "target_confidence",
                "pos_x",
                "pos_y",
                "pos_z",
                "vel_n",
                "vel_e",
                "vel_d",
                "yaw_deg",
                "cmd_vn",
                "cmd_ve",
                "cmd_vd",
                "cmd_yaw_rate",
            ],
        )
        self._csv_writer.writeheader()

        self._video_writer: Optional[cv2.VideoWriter] = None
        self.stopped: bool = False  # stop() 중복 호출 방지

    # -----------------------------
    # 비디오 기록
    # -----------------------------
    def init_video(self, width: int, height: int, fps: int = 30) -> None:
        """디버깅용 비디오 기록을 시작한다."""
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._video_writer = cv2.VideoWriter(
            str(self.video_path), fourcc, fps, (width, height)
        )

    def write_frame(self, frame) -> None:
        """비디오 프레임을 기록 (init_video 호출 이후에만 유효)."""
        if self._video_writer is not None:
            self._video_writer.write(frame)

    # -----------------------------
    # 텔레메트리 기록
    # -----------------------------
    def log_step(
        self,
        mode: str,
        target: TargetObservation,
        state: DroneState,
        cmd: ControlCommand,
    ) -> None:
        """
        한 타임스텝의 상태/명령/타겟 관측을 CSV에 기록.

        CSV는 line-buffered로 쓰지 않기 때문에,
        매번 flush() 해서 크래시 상황에서도 데이터 손실을 최소화한다.
        """
        # image_center가 None일 수 있으므로 안전하게 처리
        if target.image_center is not None:
            cx, cy = target.image_center
        else:
            cx, cy = -1, -1

        vn, ve, vd = state.velocity_ned
        cvn, cve, cvd = cmd.velocity_ned

        row = {
            "t": now(),
            "mode": mode,
            "target_detected": int(target.detected),
            "target_cx": cx,
            "target_cy": cy,
            "target_confidence": target.confidence,
            "pos_x": state.position_x,
            "pos_y": state.position_y,
            "pos_z": state.position_z,
            "vel_n": vn,
            "vel_e": ve,
            "vel_d": vd,
            "yaw_deg": state.yaw_deg,
            "cmd_vn": cvn,
            "cmd_ve": cve,
            "cmd_vd": cvd,
            "cmd_yaw_rate": cmd.yaw_rate,
        }
        self._csv_writer.writerow(row)
        # 비정상 종료 대비
        self._csv_file.flush()

    # -----------------------------
    # 정리
    # -----------------------------
    def stop(self) -> None:
        """
        리소스를 정리하고 파일을 닫는다.

        여러 번 호출되어도 문제 없도록 idempotent 하게 구현.
        """
        if self.stopped:
            return
        self.stopped = True

        try:
            self._csv_file.close()
        except Exception:
            pass

        if self._video_writer is not None:
            try:
                self._video_writer.release()
            except Exception:
                pass
            self._video_writer = None

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> "DataRecorder":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
