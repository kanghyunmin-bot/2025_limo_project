#!/usr/bin/env python3
from rclpy.node import Node
from std_msgs.msg import Empty, String, Bool, Float32
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
        self.pub_interpolation_method = self.create_publisher(String, '/path_follower/interpolation_method', 10)
        
        # Subscribers
        self.sub_path = self.create_subscription(Path, '/path_odom', self.on_path, 10)
        self.sub_accuracy = self.create_subscription(Float32, '/path_accuracy', self.on_accuracy, 10)
        
        self.current_path = None
        self.current_accuracy = 0.0
        self.accuracy_callback = None
        
        self.get_logger().info("✅ ControlPanelNode initialized")
    
    def on_path(self, msg: Path):
        self.current_path = msg
    
    def on_accuracy(self, msg: Float32):
        self.current_accuracy = msg.data
        if self.accuracy_callback:
            self.accuracy_callback(msg.data)
    
    def set_accuracy_callback(self, callback):
        self.accuracy_callback = callback
