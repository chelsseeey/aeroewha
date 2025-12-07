"""
common package

Shared types, geometry helpers, and logging utilities.
"""

from __future__ import annotations

from .types import (
    now,
    MissionState,
    MissionStatus,
    TargetObservation,
    DroneState,
    ControlCommand,
)
from .geometry import (
    compute_distance,
    compute_distance_2d,
    compute_bearing,
    ned_to_gps,
    gps_to_ned,
    normalize_angle,
    pixel_to_angle,
)
from .data_recorder import DataRecorder

__all__ = [
    # types
    "now",
    "MissionState",
    "MissionStatus",
    "TargetObservation",
    "DroneState",
    "ControlCommand",
    # geometry
    "compute_distance",
    "compute_distance_2d",
    "compute_bearing",
    "ned_to_gps",
    "gps_to_ned",
    "normalize_angle",
    "pixel_to_angle",
    # recorder
    "DataRecorder",
]
