import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class HoverControlNode(Node):
    def __init__(self):
        super().__init__('hover_control_node')
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Hover Control Node Initialized")
        self.armed = False
        self.offboard_mode = False

    def timer_callback(self):
        if not self.offboard_mode:
            self.set_offboard_mode()
        elif not self.armed:
            self.arm_vehicle()
        else:
            self.publish_hover_setpoint()

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # Custom mode
        msg.param2 = 6.0  # Offboard mode
        msg.target_system = 1
        msg.target_component = 1
        self.vehicle_command_publisher.publish(msg)
        self.offboard_mode = True
        self.get_logger().info("Set to Offboard Mode")

    def arm_vehicle(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # Arm
        msg.target_system = 1
        msg.target_component = 1
        self.vehicle_command_publisher.publish(msg)
        self.armed = True
        self.get_logger().info("Vehicle Armed")

    def publish_hover_setpoint(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        offboard_msg.position = True
        self.offboard_publisher.publish(offboard_msg)

        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        traj_msg.position = [0.0, 0.0, -2.0]  # 2m 높이에서 호버링
        traj_msg.yaw = math.radians(0.0)  # 0도 방향
        self.trajectory_publisher.publish(traj_msg)
        self.get_logger().info("Publishing Hover Setpoint")

def main(args=None):
    rclpy.init(args=args)
    node = HoverControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()