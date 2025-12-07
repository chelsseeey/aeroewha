"""
Controller Unit Tests
Tests for PID controller and high-level control logic.
"""

import unittest
import sys
from pathlib import Path

# 프로젝트 루트 경로를 sys.path 에 추가
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control.controller import Controller, PIDController, Target, ControlCommand  # type: ignore


class TestPIDController(unittest.TestCase):
    def test_proportional_only(self):
        """P 게인만 있을 때 출력이 오차에 비례하는지 확인."""
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, output_limits=(-10.0, 10.0))
        out = pid.update(setpoint=1.0, measurement=0.0, current_time=0.0)
        self.assertAlmostEqual(out, 2.0, places=6)

    def test_integral_accumulates(self):
        """I 게인이 시간이 지날수록 적분되는지 확인."""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limits=(-10.0, 10.0))

        out1 = pid.update(setpoint=1.0, measurement=0.0, current_time=0.0)  # dt=0
        out2 = pid.update(setpoint=1.0, measurement=0.0, current_time=1.0)  # dt=1
        out3 = pid.update(setpoint=1.0, measurement=0.0, current_time=2.0)  # dt=1

        # 첫 호출은 dt=0 이라 integral 반영 X
        self.assertAlmostEqual(out1, 0.0, places=6)
        # 이후에는 1씩 누적되어 1, 2
        self.assertAlmostEqual(out2, 1.0, places=6)
        self.assertAlmostEqual(out3, 2.0, places=6)

    def test_derivative_term(self):
        """D 게인이 오차 변화율에 비례하는지 확인."""
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, output_limits=(-10.0, 10.0))

        # 첫 호출: prev_error 없음 → dt=0 → d_term=0
        out1 = pid.update(setpoint=0.0, measurement=0.0, current_time=0.0)
        # 두 번째 호출: error = 1.0, dt = 1.0 → d_term = 1.0
        out2 = pid.update(setpoint=1.0, measurement=0.0, current_time=1.0)

        self.assertAlmostEqual(out1, 0.0, places=6)
        self.assertAlmostEqual(out2, 1.0, places=6)


class TestController(unittest.TestCase):
    def setUp(self):
        # control.yaml 과 비슷한 구조의 최소 config
        self.config = {
            "controller": {
                "position": {"p_gain": 1.0, "i_gain": 0.0, "d_gain": 0.0},
                "velocity": {"p_gain": 0.0, "i_gain": 0.0, "d_gain": 0.0},  # 현재는 사용 안 함
                "yaw": {"p_gain": 1.0, "i_gain": 0.0, "d_gain": 0.0},
            },
            "limits": {
                "max_horizontal_speed": 5.0,
                "max_vertical_speed": 2.0,
                "max_yaw_rate": 45.0,
            },
        }
        self.controller = Controller(self.config)

    def test_zero_error_outputs_zero(self):
        """타겟과 현재 위치/자세가 같으면 출력 속도가 0 근처인지 확인."""
        target = Target(position_x=0.0, position_y=0.0, position_z=0.0, yaw=0.0)
        current_position = (0.0, 0.0, 0.0)
        current_yaw = 0.0

        cmd: ControlCommand = self.controller.compute_control(
            target, current_position, current_yaw
        )

        self.assertAlmostEqual(cmd.velocity_x, 0.0, places=6)
        self.assertAlmostEqual(cmd.velocity_y, 0.0, places=6)
        self.assertAlmostEqual(cmd.velocity_z, 0.0, places=6)
        self.assertAlmostEqual(cmd.yaw_rate, 0.0, places=6)

    def test_moves_towards_target(self):
        """타겟이 앞/아래에 있을 때 속도 방향이 타겟 쪽을 향하는지 확인."""
        # NED 기준: x=North, y=East, z=Down(+)
        target = Target(position_x=5.0, position_y=0.0, position_z=3.0, yaw=0.0)
        current_position = (0.0, 0.0, 0.0)
        current_yaw = 0.0

        cmd: ControlCommand = self.controller.compute_control(
            target, current_position, current_yaw
        )

        # x 방향은 + (앞으로 가야 함), z 방향도 + (아래로 내려가야 함)
        self.assertGreater(cmd.velocity_x, 0.0)
        self.assertGreater(cmd.velocity_z, 0.0)

    def test_yaw_control_wraparound(self):
        """yaw 오차가 -180~180 으로 정규화되어 wrap-around 가 잘 되는지 확인."""
        # 예: 현재 170도, 타겟 -170도 → 오차는 +20도 여야 함
        target = Target(position_x=0.0, position_y=0.0, position_z=0.0, yaw=-170.0)
        current_position = (0.0, 0.0, 0.0)
        current_yaw = 170.0

        cmd: ControlCommand = self.controller.compute_control(
            target, current_position, current_yaw
        )

        # yaw_error = +20 → yaw_rate 도 양수여야 함
        self.assertGreater(cmd.yaw_rate, 0.0)


if __name__ == "__main__":
    unittest.main()
