#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        
        self.declare_parameter('wheelbase', 0.4)
        self.wheelbase = self.get_parameter('wheelbase').value
        
        # Differential Drive 구독
        self.sub_twist = self.create_subscription(
            Twist, '/cmd_vel', self.on_twist, 10)
        
        # Ackermann Drive 구독
        self.sub_ackermann = self.create_subscription(
            AckermannDriveStamped, '/ackermann_cmd', self.on_ackermann, 10)
        
        # Odometry 발행
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        
        # 타이머 (100Hz)
        self.timer = self.create_timer(0.01, self.update)
        
        self.get_logger().info('🤖 Fake Robot (Differential + Ackermann) started')
        self.get_logger().info(f'   Wheelbase: {self.wheelbase}m')
        self.get_logger().info(f'   Subscribed to: /cmd_vel, /ackermann_cmd')
    
    def on_twist(self, msg: Twist):
        """Differential Drive 명령 처리"""
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z
        self.get_logger().debug(f'Twist: v={self.current_v:.2f}, w={self.current_w:.2f}')
    
    def on_ackermann(self, msg: AckermannDriveStamped):
        """Ackermann Drive 명령 처리"""
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        
        # Ackermann → Differential 변환
        # w = v * tan(δ) / L
        self.current_v = speed
        
        if abs(steering_angle) > 0.001:
            self.current_w = speed * math.tan(steering_angle) / self.wheelbase
        else:
            self.current_w = 0.0
        
        # 로그 출력 (INFO 레벨로 변경)
        self.get_logger().info(
            f'✅ Ackermann: v={speed:.3f} m/s, δ={math.degrees(steering_angle):.1f}°, w={self.current_w:.3f} rad/s'
        )
    
    def update(self):
        """로봇 위치 업데이트 (100Hz)"""
        dt = 0.01
        
        # Kinematic model (Differential Drive)
        self.x += self.current_v * math.cos(self.theta) * dt
        self.y += self.current_v * math.sin(self.theta) * dt
        self.theta += self.current_w * dt
        
        # theta를 [-pi, pi] 범위로 정규화
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Odometry 발행
        now = self.get_clock().now().to_msg()
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.current_v
        odom.twist.twist.angular.z = self.current_w
        
        self.pub_odom.publish(odom)
        
        # TF 발행
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
