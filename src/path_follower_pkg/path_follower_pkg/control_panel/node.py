#!/usr/bin/env python3
from rclpy.node import Node
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class ControlPanelNode(Node):
    def __init__(self):
        super().__init__('control_panel')
        
        # Publishers
        self.pub_start = self.create_publisher(Empty, '/path_follower/start', 10)
        self.pub_stop = self.create_publisher(Empty, '/path_follower/stop', 10)
        self.pub_reset = self.create_publisher(Empty, '/path_follower/reset', 10)
        self.pub_path_source = self.create_publisher(String, '/path_follower/path_source', 10)
        self.pub_control_mode = self.create_publisher(String, '/path_follower/control_mode', 10)
        self.pub_drive_mode = self.create_publisher(String, '/path_follower/drive_mode', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_velocity_params = self.create_publisher(Twist, '/path_follower/velocity_params', 10)
        self.pub_use_ackermann_path = self.create_publisher(Bool, '/path_follower/use_ackermann_path', 10)
        
        # ✅ 보간 방법 토픽 추가
        self.pub_interpolation_method = self.create_publisher(String, '/path_follower/interpolation_method', 10)
        
        # Subscribers
        self.sub_path = self.create_subscription(Path, '/path_odom', self.on_path, 10)
        
        self.current_path = None
        self.get_logger().info("✅ ControlPanelNode initialized with interpolation method support")
    
    def on_path(self, msg: Path):
        self.current_path = msg
