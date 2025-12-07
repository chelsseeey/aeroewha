"""
Mock MAVSDK Client
Simulates drone connection for testing
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class MockPosition:
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    absolute_altitude_m: float = 0.0


@dataclass
class MockBattery:
    remaining_percent: float = 1.0  # 1.0 = 100%


class MockMAVSDKClient:

    def __init__(self, connection_string: str = "mock://") -> None:
        self.connection_string: str = connection_string

        # 상태 플래그들
        self.is_connected: bool = False
        self.is_armed: bool = False
        self.offboard_started: bool = False

        # 마지막으로 보낸 속도 명령 (vx, vy, vz, yaw_deg)
        self.last_velocity_cmd: Optional[Tuple[float, float, float, float]] = None

        # 모의 배터리/위치/비행모드
        self.battery_percent: float = 1.0  # 1.0 == 100%
        self.position: MockPosition = MockPosition()
        self.flight_mode: str = "HOLD"

    # ------------------------------------------------------------------
    # 연결 
    # ------------------------------------------------------------------
    async def connect(self) -> None:
        self.is_connected = True

    async def close(self) -> None:
        """모의 종료."""
        self.is_connected = False
        self.is_armed = False
        self.offboard_started = False

    # ------------------------------------------------------------------
    # 기본 액션 (arm, disarm, takeoff, land, rtl)
    # ------------------------------------------------------------------
    async def arm(self) -> None:
        """연결되어 있으면 is_armed = True."""
        if not self.is_connected:
            raise RuntimeError("Cannot arm: not connected")
        self.is_armed = True
        self.flight_mode = "ARMED"

    async def disarm(self) -> None:
        """is_armed = False."""
        if not self.is_connected:
            raise RuntimeError("Cannot disarm: not connected")
        self.is_armed = False
        self.flight_mode = "DISARMED"

    async def takeoff(self, altitude_m: float = 5.0) -> None:
        if not self.is_connected or not self.is_armed:
            raise RuntimeError("Cannot takeoff: not connected or not armed")
        self.flight_mode = "TAKEOFF"
        self.position.absolute_altitude_m = altitude_m

    async def land(self) -> None:
        if not self.is_connected:
            raise RuntimeError("Cannot land: not connected")
        self.flight_mode = "LAND"
        self.position.absolute_altitude_m = 0.0
        self.is_armed = False

    async def rtl(self) -> None:
        if not self.is_connected:
            raise RuntimeError("Cannot RTL: not connected")
        self.flight_mode = "RTL"

    # ------------------------------------------------------------------
    # Offboard 관련 인터페이스
    # ------------------------------------------------------------------
    async def start_offboard(self) -> None:
        if not self.is_connected or not self.is_armed:
            raise RuntimeError("Cannot start offboard: not connected or not armed")
        self.offboard_started = True
        self.flight_mode = "OFFBOARD"

    async def stop_offboard(self) -> None:
        """모의 offboard 종료."""
        if not self.is_connected:
            raise RuntimeError("Cannot stop offboard: not connected")
        self.offboard_started = False
        self.flight_mode = "HOLD"

    async def send_velocity_ned(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_deg: float,
    ) -> None:
        if not self.is_connected:
            raise RuntimeError("Cannot send velocity: not connected")


        self.last_velocity_cmd = (vx, vy, vz, yaw_deg)

    # ------------------------------------------------------------------
    # Telemetry-ish helpers
    # ------------------------------------------------------------------
    async def get_position(self) -> MockPosition:

        return self.position

    async def get_battery(self) -> MockBattery:
        return MockBattery(remaining_percent=self.battery_percent)

    async def get_flight_mode(self) -> str:
        return self.flight_mode
