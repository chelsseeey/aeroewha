import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')
        
        # /mavros/setpoint_raw/attitude 토픽을 발행하는 퍼블리셔 생성
        self.publisher_ = self.create_publisher(TwistStamped, '/mavros/setpoint_raw/attitude', 10)
        
        # 0.02초(50Hz)마다 publish_attitude_thrust 함수 호출
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.publish_attitude_thrust)
        
        # 목표 추력 (0.0 ~ 1.0)
        self.thrust = 0.6  # 이륙에 필요한 기본 추력

        self.get_logger().info('드론 제어 노드가 시작되었습니다.')

    def publish_attitude_thrust(self):
        """MAVROS에 자세 및 추력 메시지를 발행합니다."""
        
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # ---------- 자세(Attitude) 및 추력(Thrust) 설정 ----------
        
        # 드론의 추력 제어 (위로 올라가는 힘)
        # linear.z 값을 0.0~1.0 사이로 설정하여 드론의 고도를 조절
        msg.twist.linear.z = self.thrust
        
        # 드론의 회전(Roll, Pitch, Yaw) 제어
        # 0.0으로 설정하면 안정적인 자세를 유지
        msg.twist.angular.x = 0.0  # Roll (좌우 기울기)
        msg.twist.angular.y = 0.0  # Pitch (앞뒤 기울기)
        msg.twist.angular.z = 0.0  # Yaw (수평 회전)

        # 메시지 발행
        self.publisher_.publish(msg)

        # 로그 출력
        self.get_logger().info('자세/추력 메시지를 발행했습니다.')


def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()