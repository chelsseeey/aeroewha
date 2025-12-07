"""
Platform package

하드웨어/플랫폼 계층 래퍼.

- MavsdkClient : MAVSDK를 이용한 PX4 연결, arm/takeoff/offboard 제어
- SafetyMonitor: 드론 상태(DroneState)에 기반한 안전 규칙 검사
"""

from .mavsdk_client import MavsdkClient
from .safety import SafetyMonitor, SafetyStatus

__all__ = [
    "MavsdkClient",
    "SafetyMonitor",
    "SafetyStatus",
]
