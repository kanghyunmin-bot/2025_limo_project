#!/usr/bin/env python3
import math
import threading
import time
import os
import json
import tkinter as tk
import tkinter.filedialog

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan

WORLD_SCALE = 50.0
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 600

LIDAR_FOV = 2.0 * math.pi
LIDAR_NUM_BEAMS = 360
LIDAR_MAX_RANGE = 10.0
LIDAR_UPDATE_HZ = 10.0
ROBOT_RADIUS = 0.25

class FakeRobot(Node):
    def __init__(self, gui_ref_callback=None):
        super().__init__('fake_robot')
        self.declare_parameter('wheelbase', 0.4)
        self.wheelbase = self.get_parameter('wheelbase').value
        self.gui_ref_callback = gui_ref_callback

        self.sub_twist = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.sub_ackermann = self.create_subscription(AckermannDriveStamped, '/ackermann_cmd', self.on_ackermann, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        self.last_scan_ranges = []
        self.last_scan_angles = []
        self.timer = self.create_timer(0.01, self.update)
        self.lidar_timer = self.create_timer(1.0 / LIDAR_UPDATE_HZ, self.update_lidar)

    def on_twist(self, msg: Twist):
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z

    def on_ackermann(self, msg: AckermannDriveStamped):
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        self.current_v = speed
        if abs(steering_angle) > 0.001:
            self.current_w = speed * math.tan(steering_angle) / self.wheelbase
        else:
            self.current_w = 0.0

    def check_collision(self, new_x, new_y):
        global GUI_INSTANCE
        if GUI_INSTANCE is None:
            return False

        walls = GUI_INSTANCE.get_walls_world()
        for (x1, y1, x2, y2) in walls:
            closest_x = max(x1, min(new_x, x2))
            closest_y = max(y1, min(new_y, y2))
            dist = math.hypot(new_x - closest_x, new_y - closest_y)
            if dist < ROBOT_RADIUS:
                return True
        for (ox, oy, r) in GUI_INSTANCE.get_dynamic_obstacle_world():
            dist = math.hypot(new_x - ox, new_y - oy)
            if dist < (ROBOT_RADIUS + r):
                return True
        return False

    def update(self):
        dt = 0.01
        new_x = self.x + self.current_v * math.cos(self.theta) * dt
        new_y = self.y + self.current_v * math.sin(self.theta) * dt
        new_theta = self.theta + self.current_w * dt

        if not self.check_collision(new_x, new_y):
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
        else:
            self.current_v = 0.0
            self.current_w = 0.0

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.current_v
        odom.twist.twist.angular.z = self.current_w

        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        if self.gui_ref_callback:
            self.gui_ref_callback(self.x, self.y, self.theta)

    def update_lidar(self):
        global GUI_INSTANCE
        if GUI_INSTANCE is None:
            return
        walls = GUI_INSTANCE.get_walls_world()
        obstacles = GUI_INSTANCE.get_dynamic_obstacle_world()
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

        rx = self.x
        ry = self.y
        rt = self.theta
        ranges, angles = [], []
        for i in range(LIDAR_NUM_BEAMS):
            angle = scan.angle_min + i * scan.angle_increment + rt
            r = self.cast_ray(rx, ry, angle, walls, obstacles, LIDAR_MAX_RANGE)
            ranges.append(r)
            angles.append(angle)
        scan.ranges = ranges
        self.last_scan_ranges = ranges
        self.last_scan_angles = angles
        self.pub_scan.publish(scan)

    @staticmethod
    def cast_ray(rx, ry, angle, walls, obstacles, max_range):
        end_x = rx + max_range * math.cos(angle)
        end_y = ry + max_range * math.sin(angle)
        min_dist = max_range

        def intersect(ax, ay, bx, by, cx, cy, dx, dy):
            denom = (ax - bx) * (cy - dy) - (ay - by) * (cx - dx)
            if abs(denom) < 1e-9:
                return None
            t = ((ax - cx) * (cy - dy) - (ay - cy) * (cx - dx)) / denom
            u = -((ax - bx) * (ay - cy) - (ay - by) * (ax - cx)) / denom
            if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                ix = ax + t * (bx - ax)
                iy = ay + t * (by - ay)
                return ix, iy
            return None

        ax, ay = rx, ry
        bx, by = end_x, end_y
        for (x1, y1, x2, y2) in walls:
            rect_segments = [(x1, y1, x2, y1),(x2, y1, x2, y2),(x2, y2, x1, y2),(x1, y2, x1, y1)]
            for (cx, cy, dx, dy) in rect_segments:
                p = intersect(ax, ay, bx, by, cx, cy, dx, dy)
                if p is not None:
                    ix, iy = p
                    d = math.hypot(ix - rx, iy - ry)
                    if d < min_dist:
                        min_dist = d
        for (ox, oy, r) in obstacles:
            # 동적 장애물(원) ray-circle 교차
            dx = math.cos(angle)
            dy = math.sin(angle)
            a = dx*dx + dy*dy
            b = 2 * (dx * (rx - ox) + dy * (ry - oy))
            c = (rx - ox)**2 + (ry - oy)**2 - r**2
            disc = b**2 - 4 * a * c
            if disc >= 0:
                sqrt_disc = math.sqrt(disc)
                s1 = (-b - sqrt_disc) / (2*a)
                s2 = (-b + sqrt_disc) / (2*a)
                for s in [s1, s2]:
                    if 0.05 < s < min_dist:
                        min_dist = s
        return min_dist

class RobotGUI:
    def __init__(self, root, robot_node):
        self.root = root
        self.robot_node = robot_node
        self.root.title("FakeRobot 2D GUI with LiDAR Visualization")
        self.canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
        self.canvas.pack()
        self.load_btn = tk.Button(root, text="불러오기", command=self.on_load_map)
        self.load_btn.pack(side='top')
        self.robot_item = None
        self.lidar_lines = []
        self.walls_canvas = []
        self.current_rect = None
        self.start_x = None
        self.start_y = None
        self.cx = CANVAS_WIDTH / 2.0
        self.cy = CANVAS_HEIGHT / 2.0
        self.dynamic_obstacle_active = False
        self.dynamic_obstacle_pos = None
        self.dynamic_obstacle_item = None
        
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)
        self.canvas.bind('<ButtonPress-2>', self.on_wheel_down)
        self.canvas.bind('<B2-Motion>', self.on_wheel_motion)
        self.canvas.bind('<ButtonRelease-2>', self.on_wheel_up)
        self.root.bind('<Key-s>', self.on_save_map)
        self.update_visualization()

    def on_wheel_down(self, event):
        self.dynamic_obstacle_active = True
        self.update_dynamic_obstacle(event)

    def on_wheel_motion(self, event):
        if self.dynamic_obstacle_active:
            self.update_dynamic_obstacle(event)

    def on_wheel_up(self, event):
        self.dynamic_obstacle_active = False
        self.remove_dynamic_obstacle()

    def update_dynamic_obstacle(self, event):
        wx, wy = self.canvas_to_world(event.x, event.y)
        self.dynamic_obstacle_pos = (wx, wy)
        self.redraw_dynamic_obstacle()

    def redraw_dynamic_obstacle(self):
        self.remove_dynamic_obstacle()
        if self.dynamic_obstacle_active and self.dynamic_obstacle_pos:
            u, v = self.world_to_canvas(*self.dynamic_obstacle_pos)
            dr = ROBOT_RADIUS * WORLD_SCALE
            self.dynamic_obstacle_item = self.canvas.create_oval(
                u-dr, v-dr, u+dr, v+dr,
                fill='deepskyblue', outline='blue', width=2
            )

    def remove_dynamic_obstacle(self):
        if self.dynamic_obstacle_item:
            self.canvas.delete(self.dynamic_obstacle_item)
        self.dynamic_obstacle_item = None

    def get_dynamic_obstacle_world(self):
        if self.dynamic_obstacle_active and self.dynamic_obstacle_pos:
            return [(*self.dynamic_obstacle_pos, ROBOT_RADIUS)]
        else:
            return []

    def on_save_map(self, event=None):
        save_path = os.path.expanduser('~/path_follower/fakemapconfig/maps/user_map.json')
        walls_data = []
        for wall in self.walls_canvas:
            x1, y1, x2, y2 = wall
            wx1, wy1 = self.canvas_to_world(x1, y1)
            wx2, wy2 = self.canvas_to_world(x2, y2)
            walls_data.append([wx1, wy1, wx2, wy2])
        state = {
            "walls": walls_data,
            "dynamic_obstacle": self.dynamic_obstacle_pos if self.dynamic_obstacle_active else None
        }
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "w") as f:
            json.dump(state, f, indent=2)
        print(f"🗂️ 맵 상태가 저장되었습니다 → {save_path}")

    def on_load_map(self):
        filepath = tk.filedialog.askopenfilename(
            initialdir=os.path.expanduser('~/path_follower/fakemapconfig/maps'),
            title="맵 불러오기",
            filetypes=(("JSON files", "*.json"), ("all files", "*.*"))
        )
        if not filepath:
            return
        try:
            with open(filepath) as f:
                state = json.load(f)
            # 벽 복원
            self.clear_walls()
            for wall in state.get("walls", []):
                wx1, wy1, wx2, wy2 = wall
                u1, v1 = self.world_to_canvas(wx1, wy1)
                u2, v2 = self.world_to_canvas(wx2, wy2)
                rect = self.canvas.create_rectangle(u1, v1, u2, v2, outline="black", fill="gray20")
                self.walls_canvas.append((u1, v1, u2, v2))
            # 동적 장애물 복원
            self.dynamic_obstacle_active = False
            self.dynamic_obstacle_pos = None
            if "dynamic_obstacle" in state and state["dynamic_obstacle"]:
                self.dynamic_obstacle_active = True
                self.dynamic_obstacle_pos = tuple(state["dynamic_obstacle"])
                self.redraw_dynamic_obstacle()
        except Exception as e:
            print("맵 불러오기 실패:", e)

    def clear_walls(self):
        # 현재 모든 벽 지우기 (캔버스 & 리스트)
        for wall in self.walls_canvas:
            x1, y1, x2, y2 = wall
            items = self.canvas.find_enclosed(x1, y1, x2, y2)
            for item in items:
                self.canvas.delete(item)
        self.walls_canvas.clear()

    def world_to_canvas(self, x, y):
        u = self.cx + x * WORLD_SCALE
        v = self.cy - y * WORLD_SCALE
        return u, v

    def canvas_to_world(self, u, v):
        x = (u - self.cx) / WORLD_SCALE
        y = (self.cy - v) / WORLD_SCALE
        return x, y

    def update_robot_pose(self, x, y, theta):
        length = 0.3
        width = 0.2
        x1 = x + math.cos(theta) * length
        y1 = y + math.sin(theta) * length
        x2 = x + math.cos(theta + math.pi * 0.75) * width
        y2 = y + math.sin(theta + math.pi * 0.75) * width
        x3 = x + math.cos(theta - math.pi * 0.75) * width
        y3 = y + math.sin(theta - math.pi * 0.75) * width
        u1, v1 = self.world_to_canvas(x1, y1)
        u2, v2 = self.world_to_canvas(x2, y2)
        u3, v3 = self.world_to_canvas(x3, y3)
        coords = [u1, v1, u2, v2, u3, v3]
        if self.robot_item is None:
            self.robot_item = self.canvas.create_polygon(coords, fill="red", outline="black", width=1)
        else:
            self.canvas.coords(self.robot_item, *coords)

    def update_visualization(self):
        for line in self.lidar_lines:
            self.canvas.delete(line)
        self.lidar_lines.clear()
        if self.robot_node and len(self.robot_node.last_scan_ranges) > 0:
            rx = self.robot_node.x
            ry = self.robot_node.y
            step = max(1, LIDAR_NUM_BEAMS // 72)
            for i in range(0, len(self.robot_node.last_scan_ranges), step):
                r = self.robot_node.last_scan_ranges[i]
                angle = self.robot_node.last_scan_angles[i]
                if r < LIDAR_MAX_RANGE:
                    end_x = rx + r * math.cos(angle)
                    end_y = ry + r * math.sin(angle)
                    u1, v1 = self.world_to_canvas(rx, ry)
                    u2, v2 = self.world_to_canvas(end_x, end_y)
                    line = self.canvas.create_line(u1, v1, u2, v2, fill="lime", width=1)
                    self.lidar_lines.append(line)
        self.redraw_dynamic_obstacle()
        self.root.after(100, self.update_visualization)

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
            x_min = min(x1, x2)
            y_min = min(y1, y2)
            x_max = max(x1, x2)
            y_max = max(y1, y2)
            self.canvas.coords(self.current_rect, x_min, y_min, x_max, y_max)
            self.walls_canvas.append((x_min, y_min, x_max, y_max))
            self.current_rect = None

    def get_walls_world(self):
        walls_world = []
        for (x1, y1, x2, y2) in self.walls_canvas:
            wx1, wy1 = self.canvas_to_world(x1, y1)
            wx2, wy2 = self.canvas_to_world(x2, y2)
            xmin = min(wx1, wx2)
            ymin = min(wy1, wy2)
            xmax = max(wx1, wx2)
            ymax = max(wy1, wy2)
            walls_world.append((xmin, ymin, xmax, ymax))
        return walls_world

GUI_INSTANCE = None

def main():
    global GUI_INSTANCE
    root = tk.Tk()
    rclpy.init()
    node = FakeRobot()
    gui = RobotGUI(root, node)
    GUI_INSTANCE = gui
    node.gui_ref_callback = gui.update_robot_pose

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
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.1)

if __name__ == '__main__':
    main()

