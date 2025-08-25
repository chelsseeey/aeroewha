import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class HoverMavrosNode(Node):
    def __init__(self):
        super().__init__('hover_mavros_node')

        # MAVROS 상태 구독
        self.state_sub = self.create_subscription(State, '/mavros2/state', self.state_callback, 10)
        self.current_state = State()

        # 호버링 위치 퍼블리시
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros2/setpoint_position/local', 10)

        # 서비스 클라이언트
        self.arm_client = self.create_client(CommandBool, '/mavros2/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros2/set_mode')

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.armed = False
        self.offboard_mode = False
        self.pose_setpoint = PoseStamped()
        self.pose_setpoint.pose.position.x = 0.0
        self.pose_setpoint.pose.position.y = 0.0
        self.pose_setpoint.pose.position.z = 2.0  # 2m 호버링
        self.get_logger().info("Hover MAVROS Node Initialized")

    def state_callback(self, msg):
        self.current_state = msg

    def timer_callback(self):
        # Offboard 모드 설정
        if not self.offboard_mode:
            self.set_offboard_mode()
        # Arm
        elif not self.armed:
            self.arm_vehicle()
        else:
            # 호버링 setpoint 전송
            self.pose_pub.publish(self.pose_setpoint)
            self.get_logger().info("Publishing Hover Setpoint")

    def set_offboard_mode(self):
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().mode_sent:
                self.offboard_mode = True
                self.get_logger().info("Set to OFFBOARD mode")

    def arm_vehicle(self):
        if self.arm_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            future = self.arm_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.armed = True
                self.get_logger().info("Vehicle Armed")

def main(args=None):
    rclpy.init(args=args)
    node = HoverMavrosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()