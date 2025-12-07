"""
mavsdk_client.py

PX4 + MAVSDK를 감싸는 얇은 래퍼.

- system.yaml 의 system 섹션을 받아서 연결 URL/타임아웃 설정
- arm / disarm / takeoff / land 같은 기본 동작 제공
- Offboard 모드용 API:
    - start_offboard()
    - stop_offboard()
    - send_velocity_ned(vx, vy, vz, yaw_rate_deg)

모든 속도 명령은 NED 기준:
    vx: North  (m/s)
    vy: East   (m/s)
    vz: Down   (m/s, +는 아래)
yaw_rate는 deg/s
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

logger = logging.getLogger(__name__)


class MavsdkClient:
    """MAVSDK System 래퍼."""

    def __init__(self, system_config: dict) -> None:
        """
        Args:
            system_config: system.yaml 의 "system" 섹션 딕셔너리.
                예:
                system:
                  mode: "SITL"  # 또는 "REAL"
                  connection:
                    sitl_address: "udp://:14540"
                    real_address: "serial:///dev/ttyACM0:57600"
                    timeout: 30
        """
        mode = system_config.get("mode", "SITL").upper()
        conn = system_config.get("connection", {})

        if mode == "SITL":
            self._connection_url = conn.get("sitl_address", "udp://:14540")
        else:
            self._connection_url = conn.get("real_address", "serial:///dev/ttyACM0:57600")

        self._timeout: float = float(conn.get("timeout", 30.0))

        self._system: Optional[System] = None
        self._connected: bool = False

        self._telemetry_task: Optional[asyncio.Task] = None

        logger.info(
            "MavsdkClient created: mode=%s, connection_url=%s, timeout=%.1f",
            mode,
            self._connection_url,
            self._timeout,
        )

    # ------------------------------------------------------------------
    # 연결/종료
    # ------------------------------------------------------------------
    async def connect(self) -> None:
        """PX4/MAVSDK에 연결한다."""
        if self._connected:
            logger.info("MavsdkClient already connected.")
            return

        self._system = System()
        logger.info("Connecting to vehicle at %s ...", self._connection_url)
        await self._system.connect(system_address=self._connection_url)

        # 연결 대기 (timeout 고려)
        try:
            await asyncio.wait_for(self._wait_until_connected(), timeout=self._timeout)
        except asyncio.TimeoutError:
            logger.error("Connection timeout after %.1f seconds.", self._timeout)
            raise

        self._connected = True
        logger.info("MavsdkClient connected.")

        # self._telemetry_task = asyncio.create_task(self._telemetry_loop())

    async def _wait_until_connected(self) -> None:
        """core.connection_state 스트림을 통해 연결 완료까지 대기."""
        assert self._system is not None

        async for state in self._system.core.connection_state():
            if state.is_connected:
                logger.info("Vehicle discovered.")
                break

    async def close(self) -> None:
        """종료 시 사용할 정리 함수 (필요 시 호출)."""
        if self._telemetry_task is not None:
            self._telemetry_task.cancel()
            try:
                await self._telemetry_task
            except asyncio.CancelledError:
                pass
            self._telemetry_task = None

        self._connected = False
        logger.info("MavsdkClient closed.")

    # ------------------------------------------------------------------
    # 기본 액션 (arm, disarm, takeoff, land 등)
    # ------------------------------------------------------------------
    async def arm(self) -> None:
        assert self._system is not None
        logger.info("Arming...")
        await self._system.action.arm()
        logger.info("Armed.")

    async def disarm(self) -> None:
        assert self._system is not None
        logger.info("Disarming...")
        await self._system.action.disarm()
        logger.info("Disarmed.")

    async def takeoff(self, altitude_m: float = 5.0) -> None:
        """기본 auto-takeoff 사용 (PX4 모드에 따라 동작)."""
        assert self._system is not None
        logger.info("Takeoff to ~%.1fm (PX4 기본 takeoff 사용)", altitude_m)
        await self._system.action.set_takeoff_altitude(altitude_m)
        await self._system.action.takeoff()

    async def land(self) -> None:
        assert self._system is not None
        logger.info("Landing...")
        await self._system.action.land()

    # ------------------------------------------------------------------
    # Offboard 관련 API (OffboardManager 에서 사용)
    # ------------------------------------------------------------------
    async def start_offboard(self) -> None:
        """
        Offboard 모드 시작.

        PX4는 Offboard start 전에 최소 하나의 setpoint를 먼저 받아야 하므로
        0 velocity setpoint를 먼저 전송한다.
        """
        assert self._system is not None

        logger.info("Setting initial offboard setpoint (0 velocity)...")
        try:
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
        except OffboardError as e:
            logger.exception("Failed to set initial offboard setpoint: %s", e)
            raise

        try:
            logger.info("Starting offboard mode...")
            await self._system.offboard.start()
            logger.info("Offboard mode started.")
        except OffboardError as e:
            logger.exception("Failed to start offboard: %s", e)
            raise

    async def stop_offboard(self) -> None:
        """Offboard 모드 종료."""
        assert self._system is not None

        try:
            logger.info("Stopping offboard mode...")
            await self._system.offboard.stop()
            logger.info("Offboard mode stopped.")
        except OffboardError as e:
            logger.warning("Error while stopping offboard: %s", e)

    async def send_velocity_ned(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate_deg: float,
    ) -> None:
        """
        Offboard용 속도 명령 전송 (NED 기준).

        Args:
            vx: North  velocity [m/s]
            vy: East   velocity [m/s]
            vz: Down   velocity [m/s] (양수면 아래로)
            yaw_rate_deg: Yaw rate [deg/s]
        """
        assert self._system is not None

        try:
            cmd = VelocityNedYaw(vx, vy, vz, yaw_rate_deg)
            await self._system.offboard.set_velocity_ned(cmd)
        except OffboardError as e:
            logger.exception("Failed to send velocity_ned: %s", e)
            raise
