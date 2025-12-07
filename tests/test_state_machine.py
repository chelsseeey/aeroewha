"""
StateMachine Unit Tests
Tests for mission state machine logic.
"""

import unittest
import sys
from pathlib import Path

# 프로젝트 루트 경로를 sys.path 에 추가
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mission import StateMachine, MissionState  # type: ignore


class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.sm = StateMachine()

    def test_initial_state_idle(self):
        """초기 상태가 IDLE 인지 확인."""
        self.assertEqual(self.sm.current_state, MissionState.IDLE)
        self.assertIsNone(self.sm.previous_state)
        self.assertGreaterEqual(self.sm.get_state_duration(), 0.0)

    def test_valid_transition_idle_to_takeoff(self):
        """IDLE → TAKEOFF 전이는 허용되는지 확인."""
        ok = self.sm.transition_to(MissionState.TAKEOFF, reason="test_takeoff")
        self.assertTrue(ok)
        self.assertEqual(self.sm.previous_state, MissionState.IDLE)
        self.assertEqual(self.sm.current_state, MissionState.TAKEOFF)
        self.assertEqual(len(self.sm.history), 1)
        self.assertEqual(self.sm.history[0].from_state, MissionState.IDLE)
        self.assertEqual(self.sm.history[0].to_state, MissionState.TAKEOFF)

    def test_invalid_transition_idle_to_landing(self):
        """IDLE → LANDING 전이는 허용되지 않아야 함."""
        ok = self.sm.transition_to(MissionState.LANDING, reason="invalid")
        self.assertFalse(ok)
        self.assertEqual(self.sm.current_state, MissionState.IDLE)
        self.assertEqual(len(self.sm.history), 0)

    def test_state_data_cleared_on_transition(self):
        """state_data 가 상태 전이 시 초기화되는지 확인."""
        self.sm.set_state_data("test_key", "test_value")
        self.assertEqual(self.sm.get_state_data("test_key"), "test_value")

        self.sm.transition_to(MissionState.TAKEOFF, reason="move")
        # 전이 후에는 state_data 가 비워져야 함
        self.assertIsNone(self.sm.get_state_data("test_key"))

    def test_get_status_structure(self):
        """get_status() 반환 구조가 기대한 필드를 포함하는지 확인."""
        self.sm.transition_to(MissionState.TAKEOFF, reason="test")
        status = self.sm.get_status()

        self.assertIn("current_state", status)
        self.assertIn("previous_state", status)
        self.assertIn("state_duration", status)
        self.assertIn("state_data", status)

        self.assertEqual(status["current_state"], MissionState.TAKEOFF.value)
        self.assertEqual(status["previous_state"], MissionState.IDLE.value)

    def test_reset(self):
        """reset() 호출 시 상태/히스토리가 초기화되는지 확인."""
        self.sm.transition_to(MissionState.TAKEOFF, reason="test")
        self.sm.set_state_data("foo", 123)

        self.sm.reset()

        self.assertEqual(self.sm.current_state, MissionState.IDLE)
        self.assertIsNone(self.sm.previous_state)
        self.assertEqual(len(self.sm.history), 0)
        self.assertEqual(self.sm.state_data, {})


if __name__ == "__main__":
    unittest.main()
