from __future__ import annotations

import math
from typing import Tuple


def compute_distance(p1: Tuple[float, float, float],
                     p2: Tuple[float, float, float]) -> float:
    """
    3D 유클리드 거리 계산.

    Parameters
    ----------
    p1, p2 : (x, y, z)

    Returns
    -------
    float
        두 점 사이 거리 (m).
    """
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    dz = p1[2] - p2[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def compute_distance_2d(p1: Tuple[float, float],
                        p2: Tuple[float, float]) -> float:
    """
    2D 평면 상 거리.

    Parameters
    ----------
    p1, p2 : (x, y)

    Returns
    -------
    float
        거리 (m).
    """
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)


def compute_bearing(north: float, east: float) -> float:

    angle_rad = math.atan2(east, north)  # atan2(y, x)가 아니라 (east, north) 순서 주의
    angle_deg = math.degrees(angle_rad)
    return normalize_angle(angle_deg)


def ned_to_gps(origin_lat: float,
               origin_lon: float,
               north: float,
               east: float) -> Tuple[float, float]:
    # 지구 반지름 근사
    R_earth = 6378137.0  # m

    d_lat = north / R_earth
    d_lon = east / (R_earth * math.cos(math.radians(origin_lat)))

    lat = origin_lat + math.degrees(d_lat)
    lon = origin_lon + math.degrees(d_lon)
    return lat, lon


def gps_to_ned(origin_lat: float,
               origin_lon: float,
               lat: float,
               lon: float) -> Tuple[float, float]:
    R_earth = 6378137.0

    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)

    north = d_lat * R_earth
    east = d_lon * R_earth * math.cos(math.radians(origin_lat))
    return north, east


def normalize_angle(angle_deg: float) -> float:
    """
    각도를 [-180, 180] 범위로 정규화.
    """
    a = (angle_deg + 180.0) % 360.0 - 180.0
    return a


def pixel_to_angle(cx: float,
                   cy: float,
                   width: int,
                   height: int,
                   hfov_deg: float,
                   vfov_deg: float) -> Tuple[float, float]:
    cx0 = width / 2.0
    cy0 = height / 2.0

    nx = (cx - cx0) / cx0  # [-1, 1]
    ny = (cy - cy0) / cy0  # [-1, 1]

    yaw = nx * (hfov_deg / 2.0)
    pitch = ny * (vfov_deg / 2.0)
    return yaw, pitch
