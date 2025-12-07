"""
Behaviors Module
Defines state-specific behaviors (search patterns, tracking actions, etc.)
"""

import numpy as np
from typing import Tuple, Optional
import logging
import time

logger = logging.getLogger(__name__)


class SearchBehavior:
    """Implements various search patterns for target search."""

    def __init__(self, config: dict):
        """
        Initialize search behavior.

        Args:
            config: Search configuration dictionary.
        """
        self.pattern_type = config.get("pattern_type", "spiral")  # "spiral" or "grid"
        self.search_radius = config.get("search_radius", 20.0)
        self.search_speed = config.get("search_speed", 2.0)

        self.start_position: Optional[Tuple[float, float, float]] = None
        self.current_angle: float = 0.0
        self.current_radius: float = 0.0

        logger.info("Search behavior initialized: %s", self.pattern_type)

    def start_search(self, start_position: Tuple[float, float, float]) -> None:
        """
        Start search from a given position.

        Args:
            start_position: Starting position (x, y, z) in NED frame.
        """
        self.start_position = start_position
        self.current_angle = 0.0
        self.current_radius = 0.0
        logger.info("Search started from %s", start_position)

    def get_next_waypoint(self, dt: float = 0.1) -> Optional[Tuple[float, float, float]]:
        """
        Get next search waypoint.

        Args:
            dt: Time step [s] since last call.

        Returns:
            Next waypoint (x, y, z) or None if search has not started.
        """
        if self.start_position is None:
            return None

        if self.pattern_type == "spiral":
            return self._spiral_pattern(dt)
        elif self.pattern_type == "grid":
            return self._grid_pattern(dt)
        else:
            logger.warning("Unknown search pattern '%s', falling back to spiral", self.pattern_type)
            return self._spiral_pattern(dt)

    def _spiral_pattern(self, dt: float) -> Tuple[float, float, float]:
        """
        Generate expanding spiral search pattern around the start position.

        Args:
            dt: Time step [s].

        Returns:
            Next waypoint (x, y, z).
        """
        if self.start_position is None:
            raise ValueError("Search not started. Call start_search() first.")

        # Increase radius over time
        radius_increment = self.search_speed * dt
        self.current_radius = min(self.current_radius + radius_increment, self.search_radius)

        # Rotate around center
        angular_speed_deg = 30.0  # deg/s
        self.current_angle += angular_speed_deg * dt

        # Calculate position
        x0, y0, z0 = self.start_position
        x = x0 + self.current_radius * np.cos(np.radians(self.current_angle))
        y = y0 + self.current_radius * np.sin(np.radians(self.current_angle))

        return (x, y, z0)

    def _grid_pattern(self, dt: float) -> Tuple[float, float, float]:
        """
        Generate grid search pattern.

        현재는 간단히 spiral 패턴으로 fallback 한다.
        필요 시 grid 패턴을 구현해도 된다.
        """
        if self.start_position is None:
            raise ValueError("Search not started. Call start_search() first.")

        logger.warning("Grid pattern not implemented, falling back to spiral pattern.")
        return self._spiral_pattern(dt)


class TrackBehavior:
    """Implements target tracking behavior."""

    def __init__(self, config: dict):
        """
        Initialize tracking behavior.

        Args:
            config: Tracking configuration dictionary.
        """
        self.follow_distance = config.get("follow_distance", 5.0)  # m
        self.follow_height = config.get("follow_height", 3.0)      # m (positive up)
        self.max_track_time = config.get("max_track_time", 300.0)  # s
        self.lost_timeout = config.get("lost_timeout", 5.0)        # s

        self.track_start_time: Optional[float] = None
        self.last_detection_time: Optional[float] = None

        logger.info("Track behavior initialized")

    def start_tracking(self) -> None:
        """Mark the start of tracking."""
        now = time.time()
        self.track_start_time = now
        self.last_detection_time = now
        logger.info("Tracking started")

    def update_detection(self) -> None:
        """Update last detection time (call whenever target is detected)."""
        self.last_detection_time = time.time()

    def is_lost(self) -> bool:
        """Check if target is considered 'lost' (no detection for lost_timeout)."""
        if self.last_detection_time is None:
            return True
        return (time.time() - self.last_detection_time) > self.lost_timeout

    def is_timeout(self) -> bool:
        """Check if total tracking time exceeded max_track_time."""
        if self.track_start_time is None:
            return False
        return (time.time() - self.track_start_time) > self.max_track_time

    def compute_follow_position(
        self,
        target_position: Tuple[float, float, float],
        drone_position: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """
        Compute desired follow position.

        Args:
            target_position: Target position (x, y, z) in NED.
            drone_position: Current drone position (x, y, z) in NED.

        Returns:
            Desired position (x, y, z) in NED.
        """
        tx, ty, tz = target_position
        dx, dy, dz = drone_position

        # Vector from drone to target in horizontal plane
        vx = tx - dx
        vy = ty - dy
        dist = float(np.hypot(vx, vy))

        if dist < 1e-3:
            # If almost at the same point, place drone behind target along x-axis
            desired_x = tx - self.follow_distance
            desired_y = ty
        else:
            # Maintain follow_distance behind the target
            desired_dist = max(dist - self.follow_distance, 0.0)
            scale = desired_dist / dist
            desired_x = dx + vx * scale
            desired_y = dy + vy * scale

        # Maintain height offset relative to target (NED: positive down)
        desired_z = tz - self.follow_height  # follow_height above target

        return (desired_x, desired_y, desired_z)


class LostBehavior:
    """Implements behavior when target is lost."""

    def __init__(self, config: dict):
        """
        Initialize lost behavior.

        Args:
            config: Lost-state configuration dictionary.
        """
        self.lost_timeout = config.get("lost_timeout", 10.0)
        self.lost_start_time: Optional[float] = None

        logger.info("Lost behavior initialized")

    def start_lost(self) -> None:
        """Start 'lost' state timer."""
        self.lost_start_time = time.time()
        logger.info("Target lost, starting recovery")

    def is_timeout(self) -> bool:
        """Check if lost timeout reached."""
        if self.lost_start_time is None:
            return False
        return (time.time() - self.lost_start_time) > self.lost_timeout

    def get_search_action(self) -> str:
        """Get recommended search action after lost timeout.
        """
        return "hover_and_scan"
