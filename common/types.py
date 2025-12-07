from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Tuple


def now() -> float:
    return time.time()


# -------------------------
# 미션 상태 Enum
# -------------------------

class MissionState(Enum):
    """
    드론 미션 상태 정의.

    - IDLE        : 아직 아무 것도 안 하는 초기 상태
    - TAKEOFF     : 이륙 중
    - SEARCH      : 타겟 탐색 중
    - TRACK       : 타겟을 추적 중
    - LOST        : 타겟을 놓침, 재탐색/리커버리 구간
    - RETURN_HOME : 복귀 중 (필요 시)
    - LAND        : 착륙 중
    """
    IDLE = auto()
    TAKEOFF = auto()
    SEARCH = auto()
    TRACK = auto()
    LOST = auto()
    RETURN_HOME = auto()
    LAND = auto()


# -------------------------
# 비전 관측 정보
# -------------------------

@dataclass
class TargetObservation:
    """
    비전 모듈이 한 프레임에서 만든 타겟 관측 결과.

    Attributes
    ----------
    detected:
        이번 프레임에서 타겟을 찾았는지 여부.
    image_center:
        (cx, cy) in pixel. 이미지 좌표계에서 타겟 중심.
        - (0, 0)은 좌상단
        - (width-1, height-1)은 우하단
    bbox:
        (x, y, w, h) in pixel. 타겟의 바운딩 박스.
    confidence:
        0.0 ~ 1.0 사이 신뢰도 점수.
    timestamp:
        관측이 만들어진 시각 (epoch seconds).
    """
    detected: bool
    image_center: Optional[Tuple[int, int]] = None
    bbox: Optional[Tuple[int, int, int, int]] = None
    confidence: float = 0.0
    timestamp: float = field(default_factory=now)


# -------------------------
# 드론 상태
# -------------------------

@dataclass
class DroneState:
    """
    드론의 현재 상태 (NED 좌표계 기준).

    좌표계 가정
    ----------
    - NED(Local) 기준:
        position_x : North  (m)
        position_y : East   (m)
        position_z : Down   (m, 양수면 아래로 내려가는 방향)
    - velocity_ned :
        (north_m_s, east_m_s, down_m_s) in m/s

    yaw_deg:
        지자기 기준 yaw 각도(deg). 0도 = North, +CW(시계방향)로 증가.
    """
    position_x: float = 0.0  # north (m)
    position_y: float = 0.0  # east (m)
    position_z: float = 0.0  # down (m, positive down)

    velocity_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0

    armed: bool = False
    in_offboard: bool = False

    timestamp: float = field(default_factory=now)


# -------------------------
# 제어 명령
# -------------------------

@dataclass
class ControlCommand:
    """
    Offboard 제어 명령.

    Attributes
    ----------
    velocity_ned:
        (north_m_s, east_m_s, down_m_s) in m/s.
        PX4/MAVSDK의 NED 속도 명령과 동일한 좌표계/단위.
    yaw_rate:
        yaw 각속도 (deg/s).
    """
    velocity_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (north, east, down)
    yaw_rate: float = 0.0  # deg/s

    timestamp: float = field(default_factory=now)


# -------------------------
# 미션 상태 요약
# -------------------------

@dataclass
class MissionStatus:
    """
    미션 진행 상황 요약용 구조체.

    state:
        현재 미션 상태 (MissionState).
    last_target_seen_ts:
        마지막으로 타겟을 봤던 시각 (epoch seconds).
    flight_time:
        총 비행 시간 (초 단위). 필요 시 상위에서 갱신.
    """
    state: MissionState = MissionState.IDLE
    last_target_seen_ts: Optional[float] = None
    flight_time: float = 0.0
