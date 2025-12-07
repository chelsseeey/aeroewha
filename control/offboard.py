"""
Offboard control module

- Maintains latest velocity command
- Periodically sends to PX4 via MAVSDK in Offboard mode
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Optional, Union

from common.types import ControlCommand  # 프로젝트 공통 제어 명령 타입 사용

logger = logging.getLogger(__name__)


@dataclass
class OffboardCommand:
    """
    Internal velocity command representation for Offboard loop.
    
    NED frame:
      vx: north  (m/s)
      vy: east   (m/s)
      vz: down   (m/s, positive down)
    yaw_rate: deg/s
    """
    vx: float
    vy: float
    vz: float
    yaw_rate: float


class OffboardManager:
    """
    Manages Offboard mode and periodic sending of velocity commands.

    Assumes mavsdk_client exposes:
      - start_offboard()
      - stop_offboard()
      - send_velocity_ned(vx, vy, vz, yaw_rate_deg)
    """

    def __init__(self, mavsdk_client, update_rate_hz: float = 20.0) -> None:
        self.mavsdk_client = mavsdk_client
        self.update_rate_hz = update_rate_hz

        self.current_command: Optional[OffboardCommand] = None
        self.is_active: bool = False
        self._send_task: Optional[asyncio.Task] = None

    # -----------------------------
    # Public API
    # -----------------------------
    def set_command(self, command: Union[OffboardCommand, ControlCommand]) -> None:
        """
        Update current Offboard command.

        Args:
            command: OffboardCommand or common.types.ControlCommand
        """
        if isinstance(command, ControlCommand):
            # ControlCommand → OffboardCommand 변환
            vn, ve, vd = command.velocity_ned
            self.current_command = OffboardCommand(
                vx=vn,
                vy=ve,
                vz=vd,
                yaw_rate=command.yaw_rate,
            )
        else:
            self.current_command = command

    async def start(self) -> bool:
        """
        Start Offboard mode and periodic sending loop.

        Returns:
            True if successfully started or already active, False otherwise.
        """
        if self.is_active:
            logger.warning("Offboard already active; start() ignored.")
            return True

        # 기본 명령이 없다면 정지 명령으로 초기화
        if self.current_command is None:
            self.current_command = OffboardCommand(0.0, 0.0, 0.0, 0.0)

        # PX4가 Offboard를 받아들이도록 pre-stream
        try:
            for _ in range(10):
                await self._send_once(self.current_command)
                await asyncio.sleep(0.05)

            await self.mavsdk_client.start_offboard()
            self.is_active = True

            # 주기적인 전송 loop 시작
            self._send_task = asyncio.create_task(self._send_loop())
            logger.info("Offboard started.")
            return True
        except Exception as e:
            logger.exception("Failed to start offboard: %s", e)
            self.is_active = False
            self._send_task = None
            return False

    async def stop(self) -> None:
        """Stop Offboard mode and sending loop."""
        if not self.is_active:
            logger.info("Offboard already stopped; stop() ignored.")
            return

        self.is_active = False

        if self._send_task is not None:
            self._send_task.cancel()
            try:
                await self._send_task
            except asyncio.CancelledError:
                pass
            self._send_task = None

        try:
            await self.mavsdk_client.stop_offboard()
        except Exception as e:
            logger.exception("Error while stopping offboard: %s", e)

        logger.info("Offboard stopped.")

    # -----------------------------
    # Internal helpers
    # -----------------------------
    async def _send_loop(self) -> None:
        """Periodic sending loop; runs in background task."""
        period = 1.0 / self.update_rate_hz
        try:
            while self.is_active:
                if self.current_command is None:
                    cmd = OffboardCommand(0.0, 0.0, 0.0, 0.0)
                else:
                    cmd = self.current_command

                await self._send_once(cmd)
                await asyncio.sleep(period)
        except asyncio.CancelledError:
            # 정상적인 종료
            pass
        except Exception as e:
            logger.exception("Exception in offboard send loop: %s", e)
            self.is_active = False

    async def _send_once(self, command: OffboardCommand) -> None:
        """Send a single velocity command via mavsdk_client."""
        try:
            await self.mavsdk_client.send_velocity_ned(
                command.vx,
                command.vy,
                command.vz,
                command.yaw_rate,
            )
        except Exception as e:
            logger.exception("Failed to send offboard command: %s", e)
