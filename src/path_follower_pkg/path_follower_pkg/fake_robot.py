#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan

import tkinter as tk


# ----------------- 시뮬레이션 / GUI 파라미터 -----------------
WORLD_SCALE = 50.0      # 1m = 50px
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 600

LIDAR_FOV = 2.0 * math.pi   # 360도
LIDAR_NUM_BEAMS = 360
LIDAR_MAX_RANGE = 10.0       # 5m
LIDAR_UPDATE_HZ = 10.0      # 10Hz

ROBOT_RADIUS = 0.25         # 로봇 반경 (충돌 체크용)


class FakeRobot(Node):
    def __init__(self, gui_ref_callback=None):
        super().__init__('fake_robot')

        self.declare_parameter('wheelbase', 0.4)
        self.wheelbase = self.get_parameter('wheelbase').value

        # GUI에서 벽/화면 정보를 받을 수 있도록 콜백 저장
        self.gui_ref_callback = gui_ref_callback

        # Differential Drive 구독
        self.sub_twist = self.create_subscription(
            Twist, '/cmd_vel', self.on_twist, 10)

        # Ackermann Drive 구독
        self.sub_ackermann = self.create_subscription(
            AckermannDriveStamped, '/ackermann_cmd', self.on_ackermann, 10)

        # Odometry 발행
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # LIDAR 발행
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 로봇 상태 (월드 좌표, 단위 m, rad)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        # 마지막 LaserScan 결과 저장 (GUI에서 그리기 위해)
        self.last_scan_ranges = []
        self.last_scan_angles = []

        # 타이머
        self.timer = self.create_timer(0.01, self.update)  # 100Hz
        self.lidar_timer = self.create_timer(
            1.0 / LIDAR_UPDATE_HZ, self.update_lidar)

        self.get_logger().info('🤖 Fake Robot (Diff + Ackermann + GUI + LiDAR) started')
        self.get_logger().info(f'   Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'   Subscribed to: /cmd_vel, /ackermann_cmd')
        self.get_logger().info(f'   Publishing: /odom, /scan')

    # -------------- 콜백들 -----------------
    def on_twist(self, msg: Twist):
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z
        self.get_logger().debug(
            f'Twist: v={self.current_v:.2f}, w={self.current_w:.2f}')

    def on_ackermann(self, msg: AckermannDriveStamped):
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle

        self.current_v = speed
        if abs(steering_angle) > 0.001:
            self.current_w = speed * math.tan(steering_angle) / self.wheelbase
        else:
            self.current_w = 0.0

        self.get_logger().info(
            f'✅ Ackermann: v={speed:.3f} m/s, δ={math.degrees(steering_angle):.1f}°, '
            f'w={self.current_w:.3f} rad/s'
        )

    # -------------- 충돌 감지 -----------------
    def check_collision(self, new_x, new_y):
        """
        로봇이 new_x, new_y로 이동했을 때 벽과 충돌하는지 체크.
        충돌하면 True, 안하면 False 반환.
        """
        global GUI_INSTANCE
        if GUI_INSTANCE is None:
            return False

        walls = GUI_INSTANCE.get_walls_world()

        # 로봇을 원으로 근사하여 충돌 체크
        for (x1, y1, x2, y2) in walls:
            # 직사각형과 원의 충돌 감지
            # 가장 가까운 점 찾기
            closest_x = max(x1, min(new_x, x2))
            closest_y = max(y1, min(new_y, y2))

            # 거리 계산
            dist = math.hypot(new_x - closest_x, new_y - closest_y)

            if dist < ROBOT_RADIUS:
                return True  # 충돌!

        return False

    # -------------- 상태 업데이트 / Odom / TF -----------------
    def update(self):
        dt = 0.01

        # 새로운 위치 계산
        new_x = self.x + self.current_v * math.cos(self.theta) * dt
        new_y = self.y + self.current_v * math.sin(self.theta) * dt
        new_theta = self.theta + self.current_w * dt

        # 충돌 체크
        if not self.check_collision(new_x, new_y):
            # 충돌 안함 -> 위치 업데이트
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
        else:
            # 충돌! -> 움직임 정지
            self.current_v = 0.0
            self.current_w = 0.0
            self.get_logger().warn('⚠️ Wall collision detected! Robot stopped.')

        # theta 정규화
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        now = self.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.current_v
        odom.twist.twist.angular.z = self.current_w

        self.pub_odom.publish(odom)

        # TF
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

        # GUI에 로봇 포즈 업데이트 (GUI 콜백이 등록되어 있으면 호출)
        if self.gui_ref_callback is not None:
            self.gui_ref_callback(self.x, self.y, self.theta)

    # -------------- LIDAR 시뮬레이션 -----------------
    def update_lidar(self):
        """
        현재 로봇 위치/각도와 GUI에 정의된 직사각형 벽들에 대해
        간단한 레이 캐스팅을 수행하여 LaserScan을 발행.
        """
        global GUI_INSTANCE
        if GUI_INSTANCE is None:
            return

        walls = GUI_INSTANCE.get_walls_world()

        # LaserScan 메시지 구성
        now = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'base_link'
        scan.angle_min = -LIDAR_FOV / 2.0
        scan.angle_max = LIDAR_FOV / 2.0
        scan.angle_increment = LIDAR_FOV / LIDAR_NUM_BEAMS
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / LIDAR_UPDATE_HZ
        scan.range_min = 0.05
        scan.range_max = LIDAR_MAX_RANGE

        ranges = []
        angles = []

        # 로봇 위치/각도 로컬 변수
        rx = self.x
        ry = self.y
        rt = self.theta

        for i in range(LIDAR_NUM_BEAMS):
            angle = scan.angle_min + i * scan.angle_increment + rt
            r = self.cast_ray(rx, ry, angle, walls, LIDAR_MAX_RANGE)
            ranges.append(r)
            angles.append(angle)

        scan.ranges = ranges

        # GUI에서 그리기 위해 저장
        self.last_scan_ranges = ranges
        self.last_scan_angles = angles

        self.pub_scan.publish(scan)

    @staticmethod
    def cast_ray(rx, ry, angle, walls, max_range):
        """
        간단한 레이-사각형 충돌 체크.
        walls: 각 원소가 (x1, y1, x2, y2) [m] 인 직사각형 영역.
        레이와 벽(4개 변)의 교차 중 가장 가까운 거리 리턴.
        충돌 없으면 max_range 리턴.
        """
        # 레이 끝점(최대 거리)
        end_x = rx + max_range * math.cos(angle)
        end_y = ry + max_range * math.sin(angle)

        min_dist = max_range

        # 선분 교차 계산용 헬퍼 함수
        def intersect(ax, ay, bx, by, cx, cy, dx, dy):
            """
            선분 AB와 CD의 교차점 (있으면 (x,y), 없으면 None)
            """
            denom = (ax - bx) * (cy - dy) - (ay - by) * (cx - dx)
            if abs(denom) < 1e-9:
                return None

            t = ((ax - cx) * (cy - dy) - (ay - cy) * (cx - dx)) / denom
            u = -((ax - bx) * (ay - cy) - (ay - by) * (ax - cx)) / denom

            if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                # 교차점
                ix = ax + t * (bx - ax)
                iy = ay + t * (by - ay)
                return ix, iy
            return None

        # 레이 선분
        ax, ay = rx, ry
        bx, by = end_x, end_y

        for (x1, y1, x2, y2) in walls:
            # 직사각형 네 변
            rect_segments = [
                (x1, y1, x2, y1),
                (x2, y1, x2, y2),
                (x2, y2, x1, y2),
                (x1, y2, x1, y1),
            ]
            for (cx, cy, dx, dy) in rect_segments:
                p = intersect(ax, ay, bx, by, cx, cy, dx, dy)
                if p is not None:
                    ix, iy = p
                    d = math.hypot(ix - rx, iy - ry)
                    if d < min_dist:
                        min_dist = d

        return min_dist


# ----------------- Tkinter GUI -----------------
class RobotGUI:
    def __init__(self, root, robot_node):
        self.root = root
        self.robot_node = robot_node
        self.root.title("FakeRobot 2D GUI with LiDAR Visualization")

        self.canvas = tk.Canvas(
            root,
            width=CANVAS_WIDTH,
            height=CANVAS_HEIGHT,
            bg="white"
        )
        self.canvas.pack()

        # 로봇 그래픽 핸들
        self.robot_item = None

        # 라이다 빔 그래픽 핸들들
        self.lidar_lines = []

        # 벽(픽셀 좌표)
        self.walls_canvas = []  # [(x1,y1,x2,y2), ...]
        self.current_rect = None
        self.start_x = None
        self.start_y = None

        # 월드 <-> 픽셀 변환 (월드 원점이 캔버스 중앙이라고 가정)
        self.cx = CANVAS_WIDTH / 2.0
        self.cy = CANVAS_HEIGHT / 2.0

        # 마우스 이벤트 바인딩
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)

        # 라이다 시각화 타이머
        self.update_visualization()

    # -------------- 좌표 변환 --------------
    def world_to_canvas(self, x, y):
        """
        월드 좌표(m)를 Tkinter canvas 좌표(px)로 변환.
        x: 오른쪽 양수, y: 위쪽 양수(월드 기준)라고 가정하고
        캔버스에서는 y축이 아래로 증가하므로 부호 반전.
        """
        u = self.cx + x * WORLD_SCALE
        v = self.cy - y * WORLD_SCALE
        return u, v

    def canvas_to_world(self, u, v):
        x = (u - self.cx) / WORLD_SCALE
        y = (self.cy - v) / WORLD_SCALE
        return x, y

    # -------------- 로봇 그리기 --------------
    def update_robot_pose(self, x, y, theta):
        """
        ROS 노드에서 주기적으로 호출.
        로봇을 빨간 화살표(삼각형)로 표시.
        """
        # 화살표 길이, 폭 (m)
        length = 0.3
        width = 0.2

        # 월드 좌표에서 삼각형 세 꼭짓점 계산
        # 앞쪽 꼭짓점
        x1 = x + math.cos(theta) * length
        y1 = y + math.sin(theta) * length
        # 뒤쪽 좌우
        x2 = x + math.cos(theta + math.pi * 0.75) * width
        y2 = y + math.sin(theta + math.pi * 0.75) * width

        x3 = x + math.cos(theta - math.pi * 0.75) * width
        y3 = y + math.sin(theta - math.pi * 0.75) * width

        u1, v1 = self.world_to_canvas(x1, y1)
        u2, v2 = self.world_to_canvas(x2, y2)
        u3, v3 = self.world_to_canvas(x3, y3)

        coords = [u1, v1, u2, v2, u3, v3]

        if self.robot_item is None:
            self.robot_item = self.canvas.create_polygon(
                coords, fill="red", outline="black", width=1
            )
        else:
            self.canvas.coords(self.robot_item, *coords)

    # -------------- 라이다 빔 시각화 --------------
    def update_visualization(self):
        """
        주기적으로 라이다 빔을 그림.
        """
        # 기존 라이다 라인 삭제
        for line in self.lidar_lines:
            self.canvas.delete(line)
        self.lidar_lines.clear()

        # 로봇 노드에서 마지막 스캔 데이터 가져오기
        if self.robot_node and len(self.robot_node.last_scan_ranges) > 0:
            rx = self.robot_node.x
            ry = self.robot_node.y

            # 몇 개씩 건너뛰며 그리기 (360개 전부 그리면 너무 많음)
            step = max(1, LIDAR_NUM_BEAMS // 72)  # 약 72개만 그리기

            for i in range(0, len(self.robot_node.last_scan_ranges), step):
                r = self.robot_node.last_scan_ranges[i]
                angle = self.robot_node.last_scan_angles[i]

                if r < LIDAR_MAX_RANGE:
                    # 충돌점 계산
                    end_x = rx + r * math.cos(angle)
                    end_y = ry + r * math.sin(angle)

                    # 캔버스 좌표로 변환
                    u1, v1 = self.world_to_canvas(rx, ry)
                    u2, v2 = self.world_to_canvas(end_x, end_y)

                    # 라인 그리기 (초록색, 얇게)
                    line = self.canvas.create_line(
                        u1, v1, u2, v2,
                        fill="lime", width=1
                    )
                    self.lidar_lines.append(line)

        # 100ms마다 갱신
        self.root.after(100, self.update_visualization)

    # -------------- 벽 그리기 (마우스 드래그) --------------
    def on_mouse_down(self, event):
        self.start_x = event.x
        self.start_y = event.y
        self.current_rect = self.canvas.create_rectangle(
            self.start_x, self.start_y, event.x, event.y,
            outline="black", fill="gray20"
        )

    def on_mouse_drag(self, event):
        if self.current_rect is not None:
            self.canvas.coords(
                self.current_rect,
                self.start_x, self.start_y,
                event.x, event.y
            )

    def on_mouse_up(self, event):
        if self.current_rect is not None:
            x1, y1, x2, y2 = self.canvas.coords(self.current_rect)
            # 정규화 (좌상단/우하단)
            x_min = min(x1, x2)
            y_min = min(y1, y2)
            x_max = max(x1, x2)
            y_max = max(y1, y2)

            self.canvas.coords(self.current_rect, x_min, y_min, x_max, y_max)
            self.walls_canvas.append((x_min, y_min, x_max, y_max))
            self.current_rect = None

    # -------------- 월드 좌표계의 벽 목록 --------------
    def get_walls_world(self):
        """
        Laser용으로 월드 좌표계[m]에서의 벽 직사각형 리스트를 반환.
        각 원소는 (x1, y1, x2, y2).
        """
        walls_world = []
        for (x1, y1, x2, y2) in self.walls_canvas:
            wx1, wy1 = self.canvas_to_world(x1, y1)
            wx2, wy2 = self.canvas_to_world(x2, y2)
            # 정규화
            xmin = min(wx1, wx2)
            ymin = min(wy1, wy2)
            xmax = max(wx1, wx2)
            ymax = max(wy1, wy2)
            walls_world.append((xmin, ymin, xmax, ymax))
        return walls_world


# 전역 GUI 인스턴스 (LiDAR 계산용으로 Node에서 참조)
GUI_INSTANCE = None


def main():
    global GUI_INSTANCE

    # Tkinter는 메인 스레드에서 실행
    root = tk.Tk()

    # rclpy 초기화 및 노드 생성
    rclpy.init()
    node = FakeRobot()

    # GUI 생성 (노드 참조 전달)
    gui = RobotGUI(root, node)
    GUI_INSTANCE = gui

    # 노드에 GUI 콜백 등록
    node.gui_ref_callback = gui.update_robot_pose

    # rclpy.spin을 별도 스레드에서 실행
    def ros_spin():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Tkinter 메인루프 (메인 스레드)
    try:
        root.mainloop()
    finally:
        # GUI 종료 시 ROS도 정리
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
