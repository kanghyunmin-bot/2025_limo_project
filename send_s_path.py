#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

class SPathPublisher(Node):
    def __init__(self):
        super().__init__('s_path_publisher')
        self.publisher = self.create_publisher(Path, '/planner/path', 10)
        self.get_logger().info('📍 S-Path Publisher started')
    
    def generate_s_path(self, total_length=5.0, spacing=0.3, amplitude=1.0):
        """S자 경로 생성
        
        Args:
            total_length: 전체 경로 길이 (m)
            spacing: 점 간격 (m)
            amplitude: S자 진폭 (m)
        """
        num_points = int(total_length / spacing)
        
        path_points = []
        for i in range(num_points + 1):
            x = i * spacing
            # S자 곡선: sin(x * frequency) 형태
            y = amplitude * np.sin(x * 2 * np.pi / total_length)
            path_points.append((x, y))
        
        return path_points
    
    def publish_path(self, path_points):
        """Path 메시지 발행"""
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.publisher.publish(path_msg)
        self.get_logger().info(f'✅ Published S-path: {len(path_points)} points, {path_points[-1][0]:.2f}m')

def main():
    rclpy.init()
    node = SPathPublisher()
    
    # S자 경로 생성 (5m, 0.3m 간격, 진폭 1m)
    path_points = node.generate_s_path(total_length=5.0, spacing=0.3, amplitude=1.0)
    
    print(f"\n📍 Generated S-path:")
    print(f"  - Total length: {path_points[-1][0]:.2f}m")
    print(f"  - Number of points: {len(path_points)}")
    print(f"  - Spacing: 0.3m")
    print(f"  - Amplitude: 1.0m\n")
    
    # 1초 대기 후 발행
    time.sleep(1.0)
    node.publish_path(path_points)
    
    print("✅ Path published to /planner/path")
    print("   → GUI에서 'Planner Path' 선택 후 START 누르세요!\n")
    
    # 5초 유지
    rclpy.spin_once(node, timeout_sec=5.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
