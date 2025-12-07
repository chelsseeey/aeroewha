"""
State Machine Module
Defines mission states and state transition logic.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Dict, Any, List
import logging
import time

logger = logging.getLogger(__name__)


class MissionState(Enum):
    """High-level mission states for the drone."""
    IDLE = "idle"
    TAKEOFF = "takeoff"
    SEARCH = "search"
    TRACK = "track"
    LOST = "lost"
    RTL = "rtl"          # Return to launch
    LANDING = "landing"
    EMERGENCY = "emergency"


@dataclass
class StateTransition:
    """Record of a single state transition."""
    from_state: MissionState
    to_state: MissionState
    reason: str
    timestamp: float


class StateMachine:
    """Finite State Machine for drone mission control."""

    def __init__(self) -> None:
        """Initialize state machine."""
        self.current_state: MissionState = MissionState.IDLE
        self.previous_state: Optional[MissionState] = None
        self.state_entry_time: float = time.time()

        self.history: List[StateTransition] = []
        self.state_data: Dict[str, Any] = {}

        logger.info("StateMachine initialized in state: %s", self.current_state.value)

    # ------------------------------------------------------------------
    # Core state transition logic
    # ------------------------------------------------------------------
    def transition_to(self, new_state: MissionState, reason: str = "") -> bool:
        """
        Attempt to transition to a new state.

        Args:
            new_state: Target MissionState.
            reason: Optional human-readable transition reason.

        Returns:
            True if transition performed, False if rejected.
        """
        if new_state == self.current_state:
            logger.debug("Already in state %s, transition ignored.", new_state.value)
            return False

        if not self.can_transition(new_state):
            logger.warning(
                "Invalid state transition %s -> %s (reason=%s)",
                self.current_state.value,
                new_state.value,
                reason,
            )
            return False

        now = time.time()
        logger.info(
            "State transition: %s -> %s (reason=%s)",
            self.current_state.value,
            new_state.value,
            reason,
        )

        # Record transition
        transition = StateTransition(
            from_state=self.current_state,
            to_state=new_state,
            reason=reason,
            timestamp=now,
        )
        self.history.append(transition)

        # Update state
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = now

        # Optionally clear per-state data
        self.state_data.clear()

        return True

    def can_transition(self, target_state: MissionState) -> bool:
        """
        Check if current_state can transition to target_state.

        Transition rules are intentionally conservative and can be tuned later.
        """
        allowed_transitions: Dict[MissionState, List[MissionState]] = {
            MissionState.IDLE: [MissionState.TAKEOFF, MissionState.EMERGENCY],
            MissionState.TAKEOFF: [MissionState.SEARCH, MissionState.RTL, MissionState.EMERGENCY],
            MissionState.SEARCH: [MissionState.TRACK, MissionState.LOST, MissionState.RTL, MissionState.EMERGENCY],
            MissionState.TRACK: [MissionState.SEARCH, MissionState.LOST, MissionState.RTL, MissionState.EMERGENCY],
            MissionState.LOST: [MissionState.TRACK, MissionState.SEARCH, MissionState.RTL, MissionState.EMERGENCY],
            MissionState.RTL: [MissionState.LANDING, MissionState.EMERGENCY],
            MissionState.LANDING: [MissionState.IDLE, MissionState.EMERGENCY],
            MissionState.EMERGENCY: [MissionState.IDLE],
        }

        return target_state in allowed_transitions.get(self.current_state, [])

    # ------------------------------------------------------------------
    # State info helpers
    # ------------------------------------------------------------------
    def get_state_duration(self) -> float:
        """Get time spent in current state [seconds]."""
        return time.time() - self.state_entry_time

    def set_state_data(self, key: str, value: Any) -> None:
        """Set state-specific data (e.g., last waypoint, target info, etc.)."""
        self.state_data[key] = value

    def get_state_data(self, key: str, default: Any = None) -> Any:
        """Get state-specific data."""
        return self.state_data.get(key, default)

    def get_status(self) -> dict:
        """Get current status information as a serializable dict."""
        return {
            "current_state": self.current_state.value,
            "previous_state": self.previous_state.value if self.previous_state else None,
            "state_duration": self.get_state_duration(),
            "state_data": self.state_data.copy(),
        }

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Reset FSM to IDLE, clearing history and per-state data."""
        logger.info("StateMachine reset to IDLE.")
        self.current_state = MissionState.IDLE
        self.previous_state = None
        self.state_entry_time = time.time()
        self.history.clear()
        self.state_data.clear()
