#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Empty, String, Float32, Float32MultiArray
import math
import numpy as np
import time

from .path_manager import PathManager
from .path_controller import PathController
from .stanley_controller import StanleyController, StanleyFeedforwardController
from .math_utils import quaternion_to_yaw
from .path_recorder import PathRecorder
from .accuracy_utils import AccuracyCalculator
from .lidar_constraint_filter import LidarConstraintFilter
from .costmap_constraint_filter import CostmapConstraintFilter


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        # Parameters
        self.declare_parameter('control_mode', 'stanley')
        self.declare_parameter('drive_mode', 'differential')
        self.declare_parameter('wheelbase', 0.4)
        self.declare_parameter('arrival_threshold', 0.15)

        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('lidar_message_type', 'scan')  # 'scan' or 'pointcloud'
        self.declare_parameter('enable_dynamic_avoidance', True)
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('global_cost_threshold', 50)
        self.declare_parameter('global_costmap_search_radius', 3.0)
        self.declare_parameter('global_costmap_path_window', 0.8)
        self.declare_parameter('global_costmap_inflate_margin', 0.0)
        self.declare_parameter('global_costmap_robot_radius', 0.20)
        self.declare_parameter('global_costmap_safety_margin', 0.05)
        self.declare_parameter('global_costmap_stride', 2)
        self.declare_parameter('global_costmap_max_constraints', 60)
        self.declare_parameter('global_costmap_path_stride', 3)
        self.declare_parameter('global_costmap_replan_interval', 0.8)
        self.declare_parameter('local_constraint_inflate_clearance', 0.33)
        self.declare_parameter('global_costmap_avoid_clearance', 0.3)
        
        # ✅ 주기 설정
        self.declare_parameter('controller_frequency', 60.0)  # 제어 주기
        self.declare_parameter('local_planner_frequency', 15.0)  # Local path 업데이트
        self.declare_parameter('global_publish_frequency', 1.0)  # Global path publish
        
        self.control_mode = self.get_parameter('control_mode').value
        self.drive_mode = self.get_parameter('drive_mode').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        
        self.controller_freq = self.get_parameter('controller_frequency').value
        self.local_planner_freq = self.get_parameter('local_planner_frequency').value
        self.global_publish_freq = self.get_parameter('global_publish_frequency').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.lidar_message_type = str(self.get_parameter('lidar_message_type').value).lower()
        self.enable_dynamic_avoidance = self.get_parameter('enable_dynamic_avoidance').value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.costmap_replan_interval = float(self.get_parameter('global_costmap_replan_interval').value)
        self.local_constraint_radius = float(self.get_parameter('local_constraint_inflate_clearance').value)
        self.global_constraint_radius = float(self.get_parameter('global_costmap_inflate_margin').value)
        self.global_constraint_clearance = float(self.get_parameter('global_costmap_avoid_clearance').value)

        # Components
        self.path_manager = PathManager(self)
        self.controllers = {
            'pure_pursuit': PathController(k_ld=0.6, k_theta=2.0, wheelbase=self.wheelbase),
            'stanley': StanleyController(k_e=3.5, wheelbase=self.wheelbase),
            'stanley_ff': StanleyFeedforwardController(k_e=3.5, wheelbase=self.wheelbase),
        }
        
        self.path_recorder = PathRecorder(record_interval=0.1)
        self.accuracy_calculator = AccuracyCalculator()
        self.constraint_filter = LidarConstraintFilter(inflate_clearance=self.local_constraint_radius)
        self.costmap_filter = CostmapConstraintFilter(
            cost_threshold=int(self.get_parameter('global_cost_threshold').value),
            search_radius=float(self.get_parameter('global_costmap_search_radius').value),
            path_window=float(self.get_parameter('global_costmap_path_window').value),
            inflate_margin=self.global_constraint_radius,
            avoid_clearance=self.global_constraint_clearance,
            robot_radius=float(self.get_parameter('global_costmap_robot_radius').value),
            safety_margin=float(self.get_parameter('global_costmap_safety_margin').value),
            stride=int(self.get_parameter('global_costmap_stride').value),
            max_constraints=int(self.get_parameter('global_costmap_max_constraints').value),
            path_stride=int(self.get_parameter('global_costmap_path_stride').value),
        )
        self._sync_costmap_windows()
        
        # State
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.is_running = False
        self.path_source = 'clicked_point'
        self.interpolation_method = 'spline'
        self.lidar_constraints = []
        self.costmap_constraints = []
        self.costmap_constraints_global = []
        self.costmap_obstacles = []
        
        # ✅ 주기 관리용 타이머
        self.last_local_update_time = time.time()
        self.last_global_publish_time = time.time()
        self.last_costmap_replan_time = 0.0
        self.pending_costmap_update = False
        
        self._setup_publishers()
        self._setup_subscribers()
        
        # ✅ Controller 주기 (60Hz)
        control_period = 1.0 / self.controller_freq
        self.timer = self.create_timer(control_period, self.control_loop)
        
        self.get_logger().info(
            f"🚗 Path Follower Ready\n"
            f"   Controller: {self.controller_freq}Hz\n"
            f"   Local Planner: {self.local_planner_freq}Hz\n"
            f"   Global Publish: {self.global_publish_freq}Hz"
        )
    
    def _setup_publishers(self):
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path_odom = self.create_publisher(Path, '/path_odom', 10)
        self.pub_global_path = self.create_publisher(Path, '/global_path', 10)
        self.pub_actual_path = self.create_publisher(Path, '/local_path', 10)
        self.pub_accuracy = self.create_publisher(Float32, '/path_accuracy', 10)
    
    def _setup_subscribers(self):
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.sub_clicked_point = self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10)
        self.sub_planner_path = self.create_subscription(Path, '/planner/path', self.on_planner_path, 10)
        self.sub_init_pose = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_init_pose, 10)

        if self.enable_dynamic_avoidance:
            if self.lidar_message_type == 'pointcloud':
                self.sub_lidar = self.create_subscription(PointCloud2, self.lidar_topic, self.on_pointcloud, 10)
            elif self.lidar_message_type == 'scan':
                self.sub_lidar = self.create_subscription(LaserScan, self.lidar_topic, self.on_scan, 10)
            else:
                self.get_logger().warn(
                    f"Unknown lidar_message_type '{self.lidar_message_type}', defaulting to LaserScan"
                )
                self.sub_lidar = self.create_subscription(LaserScan, self.lidar_topic, self.on_scan, 10)
            self.sub_costmap = self.create_subscription(
                OccupancyGrid, self.global_costmap_topic, self.on_costmap, 2
            )
        
        self.sub_start = self.create_subscription(Empty, '/path_follower/start', self.on_start, 10)
        self.sub_stop = self.create_subscription(Empty, '/path_follower/stop', self.on_stop, 10)
        self.sub_reset = self.create_subscription(Empty, '/path_follower/reset', self.on_reset, 10)
        
        self.sub_path_source = self.create_subscription(String, '/path_follower/path_source', self.on_path_source, 10)
        self.sub_control_mode = self.create_subscription(String, '/path_follower/control_mode', self.on_control_mode, 10)
        self.sub_drive_mode = self.create_subscription(String, '/path_follower/drive_mode', self.on_drive_mode, 10)
        self.sub_interpolation_method = self.create_subscription(String, '/path_follower/interpolation_method', self.on_interpolation_method, 10)
        self.sub_velocity_params = self.create_subscription(Twist, '/path_follower/velocity_params', self.on_velocity_params, 10)
        self.sub_local_radius = self.create_subscription(Float32, '/path_follower/local_constraint_radius', self.on_local_radius, 10)
        self.sub_global_radius = self.create_subscription(Float32, '/path_follower/global_constraint_radius', self.on_global_radius, 10)
        self.sub_global_clearance = self.create_subscription(Float32, '/path_follower/global_constraint_clearance', self.on_global_clearance, 10)
        self.sub_planner_mode = self.create_subscription(String, '/path_follower/planner_mode', self.on_planner_mode, 10)
        self.sub_apf_params = self.create_subscription(Float32MultiArray, '/path_follower/apf_params', self.on_apf_params, 10)
    
    def on_odom(self, msg: Odometry):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.robot_pose[2] = quaternion_to_yaw(msg.pose.pose.orientation)
        
        if self.is_running:
            actual_path = self.path_recorder.record(msg)
            self.pub_actual_path.publish(actual_path)
    
    def on_clicked_point(self, msg: PointStamped):
        if self.path_source == 'clicked_point':
            if len(self.path_manager.waypoints) == 0:
                self.path_manager.set_robot_start(self.robot_pose)
            self.path_manager.add_waypoint(msg)
    
    def on_planner_path(self, msg: Path):
        if self.path_source == 'planner_path':
            self.path_manager.set_path_from_external(msg)
            self.get_logger().info(f"✅ Planner path: {len(msg.poses)} points")
    
    def on_init_pose(self, msg: PoseWithCovarianceStamped):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.robot_pose[2] = quaternion_to_yaw(msg.pose.pose.orientation)

    def on_pointcloud(self, msg: PointCloud2):
        if not self.enable_dynamic_avoidance:
            return

        constraint_points_base = self.constraint_filter.build_constraints(msg)
        if not constraint_points_base:
            self.lidar_constraints = []
            self._update_combined_constraints()
            return

        constraint_points_odom = self._transform_constraints_to_odom(constraint_points_base)
        self.lidar_constraints = constraint_points_odom
        self._update_combined_constraints()

    def on_scan(self, msg: LaserScan):
        if not self.enable_dynamic_avoidance:
            return

        constraint_points_base = self.constraint_filter.build_constraints(msg)
        if not constraint_points_base:
            self.lidar_constraints = []
            self._update_combined_constraints()
            return

        constraint_points_odom = self._transform_constraints_to_odom(constraint_points_base)
        self.lidar_constraints = constraint_points_odom
        self._update_combined_constraints()

    def on_costmap(self, msg: OccupancyGrid):
        if not self.enable_dynamic_avoidance:
            return

        self.costmap_filter.update_costmap(msg)
        self.pending_costmap_update = True
        self._maybe_refresh_costmap_constraints()
    
    def on_path_source(self, msg: String):
        if msg.data in ['clicked_point', 'planner_path']:
            self.path_source = msg.data
            self.get_logger().info(f"🔄 Path Source: {self.path_source}")
    
    def on_control_mode(self, msg: String):
        if msg.data in ['pure_pursuit', 'stanley', 'stanley_ff']:
            self.control_mode = msg.data
            self.get_logger().info(f"🔄 Control: {self.control_mode}")
    
    def on_drive_mode(self, msg: String):
        if msg.data in ['differential', 'ackermann']:
            self.drive_mode = msg.data
            self.get_logger().info(f"🔄 Drive: {self.drive_mode}")
    
    def on_interpolation_method(self, msg: String):
        self.interpolation_method = msg.data
        self.path_manager.interpolation_method = msg.data
        if self.interpolation_method in ['local_bezier', 'only_global_bezier']:
            # Bézier 계열에서 코스트맵 기반 제약을 활용할 수 있도록 갱신 플래그 설정
            self.pending_costmap_update = True
    
    def on_velocity_params(self, msg: Twist):
        self.path_manager.v_max = msg.linear.x
        self.path_manager.v_min = msg.linear.y

    def _transform_constraints_to_odom(self, constraint_points_base):
        yaw = self.robot_pose[2]
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s], [s, c]])
        translation = self.robot_pose[:2]

        return [translation + rot.dot(pt) for pt in constraint_points_base]

    def _update_combined_constraints(self):
        combined = list(self.costmap_constraints)
        combined.extend(self.lidar_constraints)
        self.path_manager.update_constraint_points(combined)

    def on_local_radius(self, msg: Float32):
        val = max(0.0, float(msg.data))
        self.local_constraint_radius = val
        self.constraint_filter.inflate_clearance = val
        self.path_manager.local_constraint_window = val
        self.get_logger().info(f"📏 Local constraint radius set to {val:.2f} m")

    def on_global_radius(self, msg: Float32):
        val = max(0.0, float(msg.data))
        self.global_constraint_radius = val
        self.costmap_filter.inflate_margin = val
        self._sync_costmap_windows()
        self.pending_costmap_update = True
        self.get_logger().info(f"🌐 Global constraint radius set to {val:.2f} m (costmap)")

    def on_global_clearance(self, msg: Float32):
        val = max(0.0, float(msg.data))
        self.global_constraint_clearance = val
        self.costmap_filter.avoid_clearance = val
        # clearance를 창 폭에도 바로 반영해 GUI 입력값이 경로-코스트맵 간 거리 판단에 즉시 적용되도록 한다.
        self.costmap_filter.path_window = val
        self._sync_costmap_windows()
        self.pending_costmap_update = True
        self.get_logger().info(f"🌐 Global clearance set to {val:.2f} m")

    def on_apf_params(self, msg: Float32MultiArray):
        data = list(msg.data)
        params = self.path_manager.apf_planner.params

        fields = [
            'step', 'attract_gain', 'repel_gain',
            'influence_dist', 'goal_tolerance', 'stall_tolerance',
        ]

        for name, val in zip(fields, data):
            try:
                setattr(params, name, float(val))
            except Exception:
                continue

        if len(data) >= 7:
            try:
                params.max_iter = int(max(1, data[6]))
            except Exception:
                pass

        self.path_manager._path_dirty = True
        self.get_logger().info(
            "🧭 APF params updated: step={:.3f}, attract={:.2f}, repel={:.2f}, infl={:.2f}, goal_tol={:.2f}, stall_tol={:.2f}, max_iter={}".format(
                params.step,
                params.attract_gain,
                params.repel_gain,
                params.influence_dist,
                params.goal_tolerance,
                params.stall_tolerance,
                params.max_iter,
            )
        )
        if self.path_manager.waypoints:
            self.path_manager._update_path()

    def on_planner_mode(self, msg: String):
        self.path_manager.planner_mode = msg.data
        self.path_manager._path_dirty = True
        self.get_logger().info(f"🧭 Planner mode → {msg.data.upper()}")
        if self.path_manager.waypoints:
            self.path_manager._update_path()

    def _sync_costmap_windows(self):
        margin = max(self.global_constraint_clearance, self.global_constraint_radius, 0.0)
        window = self.costmap_filter.path_window
        self.path_manager.global_constraint_window = window + margin
        self.path_manager.global_obstacle_window = window + margin

    def _maybe_refresh_costmap_constraints(self):
        if not self.pending_costmap_update:
            return

        if self.path_manager.interpolation_method not in ['local_bezier', 'only_global_bezier']:
            self.pending_costmap_update = False
            return

        now = time.time()
        if now - self.last_costmap_replan_time < self.costmap_replan_interval:
            return

        global_path = self.path_manager.get_global_path()
        if global_path is None or len(global_path.poses) == 0:
            return

        constraints = self.costmap_filter.build_constraints(
            global_path, self.robot_pose[:2], logger=self.get_logger()
        )
        global_constraints = self.costmap_filter.build_constraints_for_path(
            global_path, logger=self.get_logger()
        )
        if not constraints and not global_constraints:
            # 코스트맵이 경로와 겹치지 않으면 기존 글로벌 경로/제약을 그대로 유지한다.
            # 실시간 코스트맵 갱신으로 곡선이 불필요하게 흔들리는 것을 방지.
            self.pending_costmap_update = False
            return

        self.costmap_constraints = constraints
        self.costmap_constraints_global = global_constraints
        self.costmap_obstacles = self.costmap_filter.build_obstacle_circles()
        # GUI에서 조절한 path_window를 그대로 글로벌 장애물 거리창에도 사용해 불필요한 재계산을 막는다
        self._sync_costmap_windows()
        self.path_manager.global_obstacle_cap = int(
            self.get_parameter('global_costmap_max_constraints').value
        )
        self._update_combined_constraints()

        # 이미 확정된 글로벌 경로가 있으면 곡선을 다시 짜지 않고 제약만 갱신한다.
        # (click → 한 번 확정 → 이후에는 costmap에 겹친 구간만 제어점 밀어내기)
        has_global = self.path_manager.get_global_path() is not None
        self.path_manager.update_global_constraints(
            self.costmap_constraints_global,
            window=float(self.costmap_filter.path_window),
            replan=not has_global,
        )
        self.path_manager.update_global_obstacles(
            self.costmap_obstacles,
            replan=not has_global,
        )

        self.last_costmap_replan_time = now
        self.pending_costmap_update = False
    
    def on_start(self, msg: Empty):
        if self.path_manager.get_local_path() is None:
            if not self.path_manager.create_initial_path(self.robot_pose):
                return
        
        self.is_running = True
        self.path_recorder.reset()
        
        for controller in self.controllers.values():
            if hasattr(controller, 'reset'):
                controller.reset()
        
        self.get_logger().info(f"▶️ START")
    
    def on_stop(self, msg: Empty):
        self.is_running = False
        self.pub_cmd_vel.publish(Twist())
        self._publish_accuracy()
    
    def on_reset(self, msg: Empty):
        self.is_running = False
        self.path_manager.reset()
        self.path_recorder.reset()
        for controller in self.controllers.values():
            if hasattr(controller, 'reset'):
                controller.reset()
        self.pub_cmd_vel.publish(Twist())
    
    def control_loop(self):
        """✅ 주기별 작업 분리"""
        current_time = time.time()
        
        # ✅ 1. Local Path 업데이트 (15Hz)
        local_period = 1.0 / self.local_planner_freq
        if current_time - self.last_local_update_time >= local_period:
            self.path_manager.update_local_path_with_cp(self.robot_pose[:2])
            self.last_local_update_time = current_time

        # ✅ 1-1. Costmap 기반 제약 재계산(스로틀)
        self._maybe_refresh_costmap_constraints()
        
        # ✅ 2. Global Path Publish (1Hz)
        global_period = 1.0 / self.global_publish_freq
        if current_time - self.last_global_publish_time >= global_period:
            global_path = self.path_manager.get_global_path()
            if global_path is not None:
                self.pub_global_path.publish(global_path)
            self.last_global_publish_time = current_time
        
        # ✅ 3. Local Path Publish (매 루프)
        local_path = self.path_manager.get_local_path()
        if local_path is not None:
            self.pub_path_odom.publish(local_path)
        
        # ✅ 4. 제어 실행 (60Hz)
        if not self.is_running or local_path is None or len(local_path.poses) < 2:
            return
        
        if self._check_arrival(local_path):
            return
        
        self._execute_control(local_path)
    
    def _check_arrival(self, local_path):
        goal = local_path.poses[-1]
        dx = goal.pose.position.x - self.robot_pose[0]
        dy = goal.pose.position.y - self.robot_pose[1]
        
        if math.sqrt(dx*dx + dy*dy) < self.arrival_threshold:
            self.is_running = False
            self.pub_cmd_vel.publish(Twist())
            self._publish_accuracy()
            self.get_logger().info(f"🎉 ARRIVED!")
            return True
        return False
    
    def _execute_control(self, local_path):
        path_points = local_path.poses
        controller = self.controllers[self.control_mode]

        velocity_ref = self.path_manager.v_max

        # ✅ 장애물 근접 시 속도 완화 (경로 틀기로도 못피할 때 급정지 방지)
        nearest_cp = self.path_manager.nearest_constraint_distance(self.robot_pose[:2])
        if nearest_cp is not None:
            if nearest_cp < 0.25:
                velocity_ref = 0.0
            elif nearest_cp < 0.5:
                velocity_ref = max(self.path_manager.v_min, velocity_ref * 0.35)
        
        try:
            linear_v, angular_z, steering_angle = controller.compute_control(
                self.robot_pose.tolist(), 
                path_points, 
                velocity_ref, 
                self
            )
            
            twist = Twist()
            twist.linear.x = linear_v
            
            if self.drive_mode == 'ackermann':
                twist.angular.z = linear_v * math.tan(steering_angle) / self.wheelbase
            else:
                twist.angular.z = angular_z
            
            twist.linear.x = np.clip(twist.linear.x, 0.0, self.path_manager.v_max)
            twist.angular.z = np.clip(twist.angular.z, -2.0, 2.0)
            
            self.pub_cmd_vel.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f"❌ {e}")
            self.pub_cmd_vel.publish(Twist())
    
    def _publish_accuracy(self):
        actual_path = self.path_recorder.get_path()
        accuracy = self.accuracy_calculator.calculate_accuracy(
            actual_path, self.path_manager.get_local_path()
        )
        self.pub_accuracy.publish(Float32(data=accuracy))
        self.get_logger().info(f"📊 {accuracy:.2f}%")


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
