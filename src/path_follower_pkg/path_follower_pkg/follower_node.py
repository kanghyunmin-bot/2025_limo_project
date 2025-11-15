#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Empty, Bool, String
import math

from .path_manager import PathManager
from .path_controller import PathController
from .stanley_controller import StanleyController
from .math_utils import quaternion_to_yaw

class PathFollower(Node):
    
    def __init__(self):
        super().__init__('path_follower')
        
        self.declare_parameter('control_mode', 'pure_pursuit')
        self.declare_parameter('drive_mode', 'differential')
        self.declare_parameter('wheelbase', 0.4)
        self.declare_parameter('arrival_threshold', 0.15)
        
        self.control_mode = self.get_parameter('control_mode').value
        self.drive_mode = self.get_parameter('drive_mode').value
        wheelbase = self.get_parameter('wheelbase').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        
        self.path_manager = PathManager(self)
        self.pure_pursuit_controller = PathController(k_ld=0.5, wheelbase=wheelbase)
        self.stanley_controller = StanleyController(k_e=1.0, k_h=0.5, wheelbase=wheelbase)
        
        self.robot_pose = [0.0, 0.0, 0.0]
        self.is_running = False
        self.path_source = 'clicked_point'
        self.interpolation_method = 'spline'
        
        # Publishers
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ackermann = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.pub_path_odom = self.create_publisher(Path, '/path_odom', 10)
        self.pub_global_path = self.create_publisher(Path, '/global_path', 10)  # ✅ 전역 경로 시각화
        
        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.sub_clicked_point = self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked_point, 10)
        self.sub_planner_path = self.create_subscription(
            Path, '/planner/path', self.on_planner_path, 10)
        self.sub_init_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_init_pose, 10)
        
        self.sub_start = self.create_subscription(Empty, '/path_follower/start', self.on_start, 10)
        self.sub_stop = self.create_subscription(Empty, '/path_follower/stop', self.on_stop, 10)
        self.sub_reset = self.create_subscription(Empty, '/path_follower/reset', self.on_reset, 10)
        self.sub_path_source = self.create_subscription(
            String, '/path_follower/path_source', self.on_path_source, 10)
        self.sub_control_mode = self.create_subscription(
            String, '/path_follower/control_mode', self.on_control_mode, 10)
        self.sub_drive_mode = self.create_subscription(
            String, '/path_follower/drive_mode', self.on_drive_mode, 10)
        self.sub_interpolation_method = self.create_subscription(
            String, '/path_follower/interpolation_method', self.on_interpolation_method, 10)
        
        # ✅ 60Hz 타이머 (16.7ms 주기)
        self.timer = self.create_timer(0.0167, self.control_loop)
        
        self.get_logger().info(f"🚗 Path Follower: {self.control_mode} | {self.drive_mode} @ 60Hz")
    
    def on_odom(self, msg: Odometry):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.robot_pose[2] = quaternion_to_yaw(msg.pose.pose.orientation)
    
    def on_clicked_point(self, msg: PointStamped):
        if self.path_source == 'clicked_point':
            self.path_manager.add_waypoint(msg)
    
    def on_planner_path(self, msg: Path):
        if self.path_source == 'planner_path':
            self.path_manager.set_path_from_external(msg)
    
    def on_path_source(self, msg: String):
        if msg.data in ['clicked_point', 'planner_path']:
            self.path_source = msg.data
            self.get_logger().info(f"🔄 Path Source: {self.path_source}")
    
    def on_control_mode(self, msg: String):
        if msg.data in ['pure_pursuit', 'stanley']:
            self.control_mode = msg.data
            self.get_logger().info(f"🔄 Control Mode: {self.control_mode}")
    
    def on_drive_mode(self, msg: String):
        if msg.data in ['differential', 'ackermann']:
            self.drive_mode = msg.data
            self.get_logger().info(f"🔄 Drive Mode: {self.drive_mode}")
    
    def on_interpolation_method(self, msg: String):
        self.interpolation_method = msg.data
        self.get_logger().info(f"🛣️ Interpolation: {self.interpolation_method}")
    
    def on_init_pose(self, msg: PoseWithCovarianceStamped):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.robot_pose[2] = quaternion_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info(f"📍 Robot pose: ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f})")
    
    def on_start(self, msg: Empty):
        if self.path_manager.get_local_path() is not None:
            self.is_running = True
            self.get_logger().info(f"▶️ START ({self.control_mode} + {self.drive_mode})")
        else:
            self.get_logger().warn("⚠️ No path available!")
    
    def on_stop(self, msg: Empty):
        self.is_running = False
        self.publish_stop_command()
        self.get_logger().info("⏸ STOP")
    
    def on_reset(self, msg: Empty):
        self.is_running = False
        self.path_manager.reset()
        self.publish_stop_command()
        self.get_logger().info("🔄 RESET")
    
    def publish_stop_command(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_twist.publish(twist)
        
        ackermann = AckermannDriveStamped()
        ackermann.drive.speed = 0.0
        ackermann.drive.steering_angle = 0.0
        self.pub_ackermann.publish(ackermann)
    
    def control_loop(self):
        """✅ 제어 루프 (60Hz = 16.7ms 주기)"""
        # Local Bézier: 실시간 경로 업데이트 (60Hz)
        if self.interpolation_method == 'local_bezier' and self.is_running:
            self.path_manager.update_local_bezier_path(self.robot_pose, self.robot_pose[2])
        
        # 경로 시각화
        local_path = self.path_manager.get_local_path()
        if local_path is not None:
            self.pub_path_odom.publish(local_path)
        
        # ✅ 전역 경로 시각화 (Local Bézier 모드일 때만)
        if self.interpolation_method == 'local_bezier':
            global_path = self.path_manager.get_global_path()
            if global_path is not None:
                self.pub_global_path.publish(global_path)
        
        # 실행 중이 아니거나 경로가 없으면 종료
        if not self.is_running or local_path is None or len(local_path.poses) < 2:
            return
        
        # 목표 지점까지 거리 확인
        goal = local_path.poses[-1]
        dx = goal.pose.position.x - self.robot_pose[0]
        dy = goal.pose.position.y - self.robot_pose[1]
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # 도착 판정
        if distance_to_goal < self.arrival_threshold:
            self.publish_stop_command()
            self.is_running = False
            self.get_logger().info("🎉 ARRIVED!")
            return
        
        # 경로를 튜플 리스트로 변환
        path_points = [(p.pose.position.x, p.pose.position.y) for p in local_path.poses]
        
        # 컨트롤러 선택
        if self.control_mode == 'pure_pursuit':
            controller = self.pure_pursuit_controller
        else:
            controller = self.stanley_controller
        
        # 가장 가까운 경로점 찾기
        min_dist = float('inf')
        nearest_idx = 0
        for i, (px, py) in enumerate(path_points):
            dist = math.hypot(px - self.robot_pose[0], py - self.robot_pose[1])
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        velocity_ref = self.path_manager.get_velocity_at_index(nearest_idx)
        
        # 제어 계산
        try:
            linear_v, angular_z, ackermann_msg = controller.compute_control(
                self.robot_pose,
                path_points,
                velocity_ref,
                self
            )
            
            # 제어 명령 발행
            if self.drive_mode == 'ackermann':
                if ackermann_msg is not None:
                    self.pub_ackermann.publish(ackermann_msg)
            else:
                twist = Twist()
                twist.linear.x = linear_v
                twist.angular.z = angular_z
                self.pub_twist.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f"❌ {self.control_mode.title()} Control Error: {e}")
            self.publish_stop_command()

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
