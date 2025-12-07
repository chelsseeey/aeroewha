"""
safety.py

드론 상태(DroneState)를 받아서 안전 규칙을 검사하는 모듈.

- SafetyStatus : 한 번의 검사 결과
- SafetyMonitor: geofence, 고도 제한 등 검사 수행

현재는 필수적인 것들만 구현:
  - geofence(수평 거리 제한)
  - 고도 제한 (min_altitude, max_altitude)

"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Optional
import logging
import math

from common.types import DroneState
from common.geometry import compute_distance_2d  # (x1,y1),(x2,y2) → m

logger = logging.getLogger(__name__)


@dataclass
class SafetyStatus:
    """안전 검사 결과."""
    ok: bool
    violations: List[str]
    require_rtl: bool = False
    require_land: bool = False
    require_disarm: bool = False


class SafetyMonitor:
    """
    SafetyMonitor

    예시 config 구조:

    safety:
      enable_geofence: true
      geofence_radius: 100.0    # m (수평 거리)
      enable_altitude_check: true
      min_altitude: 1.0         # m (해당 값 이상)
      max_altitude: 50.0        # m (해당 값 이하)

    altitude는 NED 기준 position_z 를 사용:
      altitude_m = -position_z   (Down 이 +이므로 부호 반전)
    """

    def __init__(self, safety_config: dict, home_position_ned: Tuple[float, float, float]) -> None:
        """
        Args:
            safety_config: 위와 같은 safety 섹션 딕셔너리
            home_position_ned: (x, y, z) in NED frame (보통 origin 또는 이륙 위치)
        """
        self.enable_geofence: bool = safety_config.get("enable_geofence", True)
        self.geofence_radius: float = float(safety_config.get("geofence_radius", 100.0))

        self.enable_altitude_check: bool = safety_config.get("enable_altitude_check", True)
        self.min_altitude: float = float(safety_config.get("min_altitude", 1.0))
        self.max_altitude: float = float(safety_config.get("max_altitude", 50.0))

        self.home_x, self.home_y, self.home_z = home_position_ned

        # 같은 위반에 대해 매 프레임마다 로그 찍지 않기 위해 last_violations 기억
        self._last_violations: List[str] = []

        logger.info(
            "SafetyMonitor initialized (geofence=%s, radius=%.1fm, alt_check=%s, alt=[%.1f, %.1f])",
            self.enable_geofence,
            self.geofence_radius,
            self.enable_altitude_check,
            self.min_altitude,
            self.max_altitude,
        )

    # ------------------------------------------------------------------
    # 메인 검사 함수
    # ------------------------------------------------------------------
    def check(self, state: DroneState) -> SafetyStatus:
        """
        현재 DroneState에 대해 안전 규칙을 검사한다.

        Args:
            state: common.types.DroneState 인스턴스

        Returns:
            SafetyStatus: ok 여부 + 위반 목록 + RTL/LAND/DISARM 플래그.
        """
        violations: List[str] = []
        require_rtl = False
        require_land = False
        require_disarm = False

        # -----------------------
        # 1) 지오펜스 검사
        # -----------------------
        if self.enable_geofence:
            dist = compute_distance_2d(
                (state.position_x, state.position_y),
                (self.home_x, self.home_y),
            )
            if dist > self.geofence_radius:
                violations.append("geofence_radius_exceeded")
                # 기본 정책: 지오펜스 이탈 시 RTL 시도
                require_rtl = True

        # -----------------------
        # 2) 고도 제한 검사
        # -----------------------
        if self.enable_altitude_check:
            # NED: z가 양수면 아래로 내려간 것
            altitude_m = -state.position_z  # 지표 기준 up (+)
            if altitude_m < self.min_altitude:
                violations.append("altitude_below_min")
                # 너무 낮으면 Landing 또는 RTL을 고려
                require_land = True
            elif altitude_m > self.max_altitude:
                violations.append("altitude_above_max")
                # 너무 높으면 RTL을 고려 (혹은 강제 하강)
                require_rtl = True

        ok = len(violations) == 0

        # 로그 스팸 방지: 이전 위반과 달라졌을 때만 로그
        if violations != self._last_violations:
            if violations:
                logger.warning("Safety violations: %s", ", ".join(violations))
            else:
                logger.info("Safety OK (no violations).")
            self._last_violations = violations.copy()

        return SafetyStatus(
            ok=ok,
            violations=violations,
            require_rtl=require_rtl,
            require_land=require_land,
            require_disarm=require_disarm,
        )

    # ------------------------------------------------------------------
    # 유틸 함수 
    # ------------------------------------------------------------------
    def distance_from_home(self, state: DroneState) -> float:
        """홈에서의 2D 수평 거리 [m]."""
        return compute_distance_2d(
            (state.position_x, state.position_y),
            (self.home_x, self.home_y),
        )

    def altitude_m(self, state: DroneState) -> float:
        """지표 기준 고도 [m] (NED z -> up +)."""
        return -state.position_z
