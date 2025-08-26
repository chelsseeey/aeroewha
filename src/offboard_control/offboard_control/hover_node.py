#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

class HoverNodeMavros(Node):
    def __init__(self):
        super().__init__('hover_node_mavros')

        # 퍼블리셔: ENU 로컬 좌표 setpoint (위 +Z)
        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # 서비스 클라이언트
        self.arm_cli  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode,     '/mavros/set_mode')

        # 서비스 준비 대기
        self.get_logger().info('Waiting for arming / set_mode services...')
        self.arm_cli.wait_for_service(timeout_sec=10.0)
        self.mode_cli.wait_for_service(timeout_sec=10.0)

        # 목표 자세: (0,0,2m)
        self.sp = PoseStamped()
        self.sp.header.frame_id = 'map'
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.sp.pose.position.z = 2.0

        # 상태 플래그
        self.warmup_count = 0
        self.offboard_set = False
        self.armed        = False

        # 20Hz 타이머
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info('HoverNodeMavros started (publishing 20Hz position setpoints).')

    def _tick(self):
        # setpoint 퍼블리시 (타임스탬프 갱신)
        self.sp.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.sp)

        # OFFBOARD 조건: >2Hz로 1~2초 이상 setpoint
        if self.warmup_count < 40:   # ≈ 2초
            self.warmup_count += 1
            return

        # 권장 순서: OFFBOARD → Arming
        if not self.offboard_set:
            self._set_offboard()
            return

        if not self.armed:
            self._arm()
            return

    def _set_offboard(self):
        if not self.mode_cli.service_is_ready():
            self.get_logger().warn('set_mode service not ready yet.')
            return
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        fut = self.mode_cli.call_async(req)
        fut.add_done_callback(self._on_mode)

    def _on_mode(self, fut):
        try:
            resp = fut.result()
            # PX4는 success 필드가 없을 수 있어도, 상태 토픽으로 확인 가능
            self.offboard_set = True
            self.get_logger().info('Requested OFFBOARD mode ✅')
        except Exception as e:
            self.get_logger().error(f'OFFBOARD request failed: {e}')

    def _arm(self):
        if not self.arm_cli.service_is_ready():
            self.get_logger().warn('arming service not ready yet.')
            return
        req = CommandBool.Request()
        req.value = True
        fut = self.arm_cli.call_async(req)
        fut.add_done_callback(self._on_arm)

    def _on_arm(self, fut):
        try:
            resp = fut.result()
            if getattr(resp, 'success', False):
                self.armed = True
                self.get_logger().info('Armed ✅')
            else:
                self.get_logger().warn(f'Arming rejected: {resp}')
        except Exception as e:
            self.get_logger().error(f'Arming call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HoverNodeMavros()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
