import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import time
import math

class QuadController:
    def __init__(self):
        rospy.init_node('quad_controller', anonymous=True)

        # 퍼블리셔: 쿼드콥터 비행 제어 명령을 쿼드콥터에 전달
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 서브스크라이버: IMU 데이터, GPS 데이터
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)

        self.cmd_vel = Twist()

        # 초기 설정 (예: 비행 고도, 무게중심 오프셋 등)
        self.target_altitude = 5.0  # 목표 고도 (5미터)
        self.current_altitude = 0.0
        self.roll_offset = 0.0  # 무게중심 오프셋: 롤 각도
        self.pitch_offset = 0.0  # 피치 각도
        self.thrust_offset = 1.0  # 기본 추력 배수

    def imu_callback(self, msg):
        # IMU 데이터를 받아서 처리
        self.roll = msg.orientation.x
        self.pitch = msg.orientation.y
        self.yaw = msg.orientation.z
        rospy.loginfo(f"IMU Data - Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}")

        # 무게중심 변화에 따른 롤 및 피치 오프셋 조정
        self.roll_offset = self.roll * 0.2  # 예시: 롤 변화에 따른 오프셋 조정
        self.pitch_offset = self.pitch * 0.2  # 예시: 피치 변화에 따른 오프셋 조정

    def gps_callback(self, msg):
        # GPS 데이터를 받아서 고도 정보 처리
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.current_altitude = msg.altitude
        rospy.loginfo(f"GPS Data - Latitude: {self.latitude}, Longitude: {self.longitude}, Altitude: {self.current_altitude}")

    def calculate_thrust(self):
        """
        모터에 필요한 추력을 계산합니다.
        - 목표 고도에 맞춰서 상승하도록 조정합니다.
        - 무게중심 변화에 따른 롤 및 피치 오프셋을 보정합니다.
        """
        # 목표 고도로 상승을 위한 기본 추력
        thrust = self.thrust_offset

        # 목표 고도에 따른 상승 비율을 계산
        altitude_error = self.target_altitude - self.current_altitude
        thrust += altitude_error * 0.5  # 목표 고도까지 비례하여 상승

        # 무게중심 보정 (롤 및 피치 오프셋)
        if abs(self.roll_offset) > 0.05:  # 일정 기준 이상의 롤 오프셋이 있을 때
            thrust += self.roll_offset * 0.5  # 롤 오프셋에 따른 추력 조정

        if abs(self.pitch_offset) > 0.05:  # 일정 기준 이상의 피치 오프셋이 있을 때
            thrust += self.pitch_offset * 0.5  # 피치 오프셋에 따른 추력 조정

        # 상승/하강 상태에 맞춰 Z 방향 속도 수정
        self.cmd_vel.linear.z = max(min(thrust, 1.0), 0.0)  # 추력 값을 0에서 1로 제한

    def control_loop(self):
        """
        비행 제어 루프: 이륙 시 추력 및 자세를 제어합니다.
        """
        self.calculate_thrust()
        rospy.loginfo(f"Sending thrust command: {self.cmd_vel.linear.z}")
        
        # 이륙 명령 전송
        self.pub_cmd_vel.publish(self.cmd_vel)

    def land(self):
        """
        착륙 명령
        """
        self.cmd_vel.linear.z = 0.0  # 착륙을 위한 추력 0 설정
        rospy.loginfo("Sending landing command")
        self.pub_cmd_vel.publish(self.cmd_vel)

if __name__ == '__main__':
    controller = QuadController()
    try:
        while not rospy.is_shutdown():
            controller.control_loop()
            rospy.sleep(1)  # 루프 속도 제어 (1초마다 제어)

            # 목표 고도에 도달하면 착륙
            if controller.current_altitude >= controller.target_altitude:
                controller.land()
                break

    except rospy.ROSInterruptException:
        pass
