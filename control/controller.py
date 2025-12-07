"""
Controller Module
P/PID control logic to convert target positions to control commands
"""

from __future__ import annotations

import time
import logging
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class ControlCommand:
    """Control command output."""
    velocity_x: float  # m/s (NED frame, north)
    velocity_y: float  # m/s (NED frame, east)
    velocity_z: float  # m/s (NED frame, down)
    yaw_rate: float    # deg/s
    timestamp: float


@dataclass
class Target:
    """Target position/state (NED frame)."""
    position_x: float  # m (north)
    position_y: float  # m (east)
    position_z: float  # m (down, positive down)
    yaw: float         # degrees (0 = North, +CW)


class PIDController:
    """PID controller for single axis."""
    
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limits: Tuple[float, float] = (-float('inf'), float('inf')),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time: Optional[float] = None
        
    def update(
        self,
        setpoint: float,
        measurement: float,
        current_time: Optional[float] = None,
    ) -> float:
        """
        Calculate PID output.
        
        Args:
            setpoint:   Desired value.
            measurement: Current value.
            current_time: Current timestamp (seconds).
            
        Returns:
            Control output (clamped to output_limits).
        """
        if current_time is None:
            current_time = time.time()
            
        # Calculate time delta
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time
            
        self.prev_time = current_time
        
        # Calculate error
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        if dt > 0.0:
            self.integral += error * dt

            # --- 간단한 anti-windup: integral 자체를 제한 ---
            if self.ki != 0.0 and not np.isinf(self.output_limits[0]) and not np.isinf(self.output_limits[1]):
                # ki > 0 가정 (일반적인 PID)
                int_min = self.output_limits[0] / self.ki
                int_max = self.output_limits[1] / self.ki
                if int_min > int_max:  # 혹시라도 부호 꼬였을 때 swap
                    int_min, int_max = int_max, int_min
                self.integral = np.clip(self.integral, int_min, int_max)

            i_term = self.ki * self.integral
        else:
            i_term = 0.0
        
        # Derivative term
        if dt > 0.0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
            
        self.prev_error = error
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply limits
        output = float(np.clip(output, self.output_limits[0], self.output_limits[1]))
        
        return output
    
    def reset(self) -> None:
        """Reset controller internal state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


class Controller:
    """Main controller for drone position/velocity control."""
    
    def __init__(self, config: dict):
        """
        Initialize controller.
        
        Args:
            config: Control configuration dictionary (control.yaml 전체).
        """
        self.config = config
        
        # Get controller gains
        pos_gains = config['controller']['position']
        # vel_gains = config['controller']['velocity']  # 현재는 사용하지 않음 (추후 2단계 제어에 활용 가능)
        yaw_gains = config['controller']['yaw']
        
        # Get limits
        limits = config['limits']
        self.max_horizontal_speed = float(limits['max_horizontal_speed'])
        self.max_vertical_speed = float(limits['max_vertical_speed'])
        self.max_yaw_rate = float(limits['max_yaw_rate'])
        
        # Create PID controllers (position-based)
        self.pid_x = PIDController(
            pos_gains['p_gain'], pos_gains['i_gain'], pos_gains['d_gain'],
            output_limits=(-self.max_horizontal_speed, self.max_horizontal_speed),
        )
        self.pid_y = PIDController(
            pos_gains['p_gain'], pos_gains['i_gain'], pos_gains['d_gain'],
            output_limits=(-self.max_horizontal_speed, self.max_horizontal_speed),
        )
        self.pid_z = PIDController(
            pos_gains['p_gain'], pos_gains['i_gain'], pos_gains['d_gain'],
            output_limits=(-self.max_vertical_speed, self.max_vertical_speed),
        )
        self.pid_yaw = PIDController(
            yaw_gains['p_gain'], yaw_gains['i_gain'], yaw_gains['d_gain'],
            output_limits=(-self.max_yaw_rate, self.max_yaw_rate),
        )
        
        logger.info("Controller initialized (position + yaw PID)")
    
    def compute_control(
        self,
        target: Target,
        current_position: Tuple[float, float, float],
        current_yaw: float,
    ) -> ControlCommand:
        """
        Compute control command to reach target.
        
        Args:
            target: Target position and yaw (NED frame).
            current_position: Current position (x, y, z) in NED frame.
            current_yaw: Current yaw in degrees.
            
        Returns:
            Control command (velocity + yaw_rate).
        """
        current_time = time.time()
        
        # Position PID -> velocity commands for each axis (NED)
        vx = self.pid_x.update(target.position_x, current_position[0], current_time)
        vy = self.pid_y.update(target.position_y, current_position[1], current_time)
        vz = self.pid_z.update(target.position_z, current_position[2], current_time)
        
        # Yaw control
        yaw_error = self._normalize_angle(target.yaw - current_yaw)
        # setpoint = yaw_error, measurement = 0 → error = yaw_error
        yaw_rate = self.pid_yaw.update(yaw_error, 0.0, current_time)
        
        return ControlCommand(
            velocity_x=vx,
            velocity_y=vy,
            velocity_z=vz,
            yaw_rate=yaw_rate,
            timestamp=current_time,
        )
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-180, 180] range."""
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle
    
    def reset(self) -> None:
        """Reset all controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        logger.info("Controller reset")
