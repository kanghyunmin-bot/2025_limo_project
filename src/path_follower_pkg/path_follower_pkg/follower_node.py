#!/usr/bin/env python3
"""
메인 경로 추종 노드
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, Bool

from path_follower_pkg.math_utils import wrap_to_pi, yaw_to_quat
from path_follower_pkg.path_manager import PathManager
from path_follower_pkg.path_controller import PathController


class InteractiveFollower(Node):
    """RViz 기반 대화형 경로 추종 노드"""
    def __init__(self):
        super().__init__('interactive_path_follower')
        self._load_parameters()

        self.path_manager = PathManager(
            logger=self.get_logger(),
            ds=self.ds,
            v_max=self.v_max,
            a_long=self.a_long,
            a_lat_max=self.a_lat_max
        )
        
        self.controller = PathController(
            k_y=self.k_y,
            k_psi=self.k_psi,
            k_ld=self.k_ld,
            L_min=self.L_min,
            L_max=self.L_max,
            v_max=self.v_max,
            a_long=self.a_long,
            omega_max=3.5
        )

        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.s_done = 0.0

        # 제어 플래그
        self.is_running = False
        self.is_edit_mode = False

        # Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_path = self.create_publisher(Path, '/path', 10)

        # Subscriber - RViz
        self.sub_click = self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked, 10)
        self.sub_init = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.sub_clear = self.create_subscription(
            Empty, '/clear_path', self.on_clear, 10)

        # Subscriber - Control Panel
        self.sub_start = self.create_subscription(
            Empty, '/path_follower/start', self.on_start, 10)
        self.sub_stop = self.create_subscription(
            Empty, '/path_follower/stop', self.on_stop, 10)
        self.sub_reset = self.create_subscription(
            Empty, '/path_follower/reset', self.on_reset, 10)
        self.sub_edit = self.create_subscription(
            Bool, '/path_follower/edit_mode', self.on_edit_mode, 10)

        # 타이머
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            "Interactive follower ready.\n"
            "  - RViz 'Publish Point'로 경로 생성\n"
            "  - '2D Pose Estimate'로 시작자세 설정\n"
            "  - GUI로 Start/Stop/Reset/Edit 제어"
        )

    def _load_parameters(self):
        self.declare_parameter('ds', 0.03)
        self.declare_parameter('v_max', 0.8)
        self.declare_parameter('a_long', 0.8)
        self.declare_parameter('a_lat_max', 1.5)
        self.declare_parameter('k_y', 2.0)
        self.declare_parameter('k_psi', 1.8)
        self.declare_parameter('k_ld', 1.0)
        self.declare_parameter('L_min', 0.25)
        self.declare_parameter('L_max', 0.9)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('goal_tol_xy', 0.05)
        self.declare_parameter('goal_tol_yaw_deg', 5.0)

        self.ds = float(self.get_parameter('ds').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.a_long = float(self.get_parameter('a_long').value)
        self.a_lat_max = float(self.get_parameter('a_lat_max').value)
        self.k_y = float(self.get_parameter('k_y').value)
        self.k_psi = float(self.get_parameter('k_psi').value)
        self.k_ld = float(self.get_parameter('k_ld').value)
        self.L_min = float(self.get_parameter('L_min').value)
        self.L_max = float(self.get_parameter('L_max').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)
        self.goal_tol_xy = float(self.get_parameter('goal_tol_xy').value)
        self.goal_tol_yaw = float(self.get_parameter('goal_tol_yaw_deg').value) * math.pi / 180.0

    # ========== RViz 콜백 ==========

    def on_clicked(self, msg: PointStamped):
        if self.is_edit_mode or not self.is_running:
            if self.path_manager.add_point(msg.point.x, msg.point.y):
                if self.path_manager.rebuild_path():
                    self.s_done = 0.0
                    self.publish_path()

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(
            f"Initial pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad"
        )

    def on_clear(self, _msg: Empty):
        self.path_manager.clear()
        self.s_done = 0.0
        self.is_running = False
        self.publish_path()

    # ========== 제어 패널 콜백 ==========

    def on_start(self, _msg: Empty):
        if self.path_manager.have_path:
            self.is_running = True
            self.is_edit_mode = False
            self.get_logger().info("▶ Started")
        else:
            self.get_logger().warn("No path to follow. Please create a path first.")

    def on_stop(self, _msg: Empty):
        self.is_running = False
        self.publish_cmd(0.0, 0.0)
        self.get_logger().info("⏸ Stopped")

    def on_reset(self, _msg: Empty):
        self.path_manager.clear()
        self.s_done = 0.0
        self.is_running = False
        self.publish_path()
        self.publish_cmd(0.0, 0.0)
        self.get_logger().info("🔄 Reset")

    def on_edit_mode(self, msg: Bool):
        self.is_edit_mode = msg.data
        if self.is_edit_mode:
            self.is_running = False
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("✏️  Edit mode enabled")
        else:
            self.get_logger().info("✏️  Edit mode disabled")

    # ========== 제어 루프 ==========

    def on_timer(self):
        self.publish_odom()

        if not self.is_running or not self.path_manager.have_path:
            self.publish_cmd(0.0, 0.0)
            return

        idx = self.path_manager.get_closest_index(self.x, self.y)
        if idx is None:
            self.publish_cmd(0.0, 0.0)
            return

        self.s_done = max(self.s_done, idx * self.ds)

        v_cmd, omega_cmd = self.controller.compute_control(
            self.x, self.y, self.yaw,
            self.path_manager.path_xy,
            self.path_manager.path_v,
            self.path_manager.path_kappa,
            idx, self.ds, self.s_done, self.path_manager.path_len
        )

        self.publish_cmd(v_cmd, omega_cmd)

        self.x += v_cmd * self.dt * math.cos(self.yaw)
        self.y += v_cmd * self.dt * math.sin(self.yaw)
        self.yaw = wrap_to_pi(self.yaw + omega_cmd * self.dt)

    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def publish_odom(self):
        od = Odometry()
        od.header.frame_id = 'odom'
        od.header.stamp = self.get_clock().now().to_msg()
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = float(self.x)
        od.pose.pose.position.y = float(self.y)
        od.pose.pose.orientation = yaw_to_quat(float(self.yaw))
        self.pub_odom.publish(od)

    def publish_path(self):
        path_msg = self.path_manager.get_path_msg(
            frame_id='odom',
            stamp=self.get_clock().now().to_msg()
        )
        self.pub_path.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
