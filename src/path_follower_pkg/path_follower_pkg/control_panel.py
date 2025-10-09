#!/usr/bin/env python3
import os
import socket
import time
from collections import deque
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Empty, Bool

try:
    from rclpy.utilities import get_rmw_implementation_identifier
except Exception:
    get_rmw_implementation_identifier = None


class ControlPanel(Node):
    def __init__(self):
        super().__init__("path_follower_gui")

        # Publishers
        self.pub_start   = self.create_publisher(Empty, "/path_follower/start", 10)
        self.pub_stop    = self.create_publisher(Empty, "/path_follower/stop", 10)
        self.pub_reset   = self.create_publisher(Empty, "/path_follower/reset", 10)
        self.pub_edit    = self.create_publisher(Bool,  "/path_follower/edit_mode", 10)
        self.pub_scale   = self.create_publisher(Twist, "/path_follower/cmd_vel_scale", 10)
        self.pub_ovr     = self.create_publisher(Twist, "/path_follower/cmd_vel_override", 10)
        self.pub_ovr_en  = self.create_publisher(Bool,  "/path_follower/override_enable", 10)

        # Subscribers
        self.sub_cmd     = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.sub_clicked = self.create_subscription(PointStamped, "/clicked_point", self.on_clicked_point, 10)

        # Telemetry buffers
        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.points = deque(maxlen=50)
        self.point_counter = 0

        # Control states
        self.scale_lin = 1.0
        self.scale_ang = 1.0
        self.override_enable = False
        self.override_twist = Twist()
        self.sub_scale      = self.create_subscription(Twist, '/path_follower/cmd_vel_scale', self.on_scale, 10)
        self.sub_override   = self.create_subscription(Twist, '/path_follower/cmd_vel_override', self.on_override, 10)
        self.sub_override_en= self.create_subscription(Bool,  '/path_follower/override_enable', self.on_override_enable, 10)

        # ROS/Network status
        self.status = {
            "domain_id": os.getenv("ROS_DOMAIN_ID", "0"),
            "localhost_only": os.getenv("ROS_LOCALHOST_ONLY", "0"),
            "rmw": self._detect_rmw(),
            "host": socket.gethostname(),
            "ip": self._detect_ip(),
            "node_count": 0,
            "other_nodes": [],
            "topic_count": 0,
            "has_cmd_vel": False,
        }
        self.create_timer(1.0, self._refresh_ros_status)

        # Start Tk GUI
        self.gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self.gui_thread.start()

        self.get_logger().info("Enhanced GUI started with monitoring & control features")

    def _detect_rmw(self):
        try:
            if get_rmw_implementation_identifier:
                return get_rmw_implementation_identifier()
        except Exception:
            pass
        return os.getenv("RMW_IMPLEMENTATION", "unknown")

    def _detect_ip(self):
        ip = "unknown"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            try:
                ip = socket.gethostbyname(socket.gethostname())
            except Exception:
                pass
        return ip

    def _refresh_ros_status(self):
        try:
            names = self.get_node_names_and_namespaces()
            topics = self.get_topic_names_and_types()
        except Exception:
            return

        self.status["node_count"] = len(names)
        self_name = self.get_name()
        others = [(n, ns) for (n, ns) in names if n != self_name]
        self.status["other_nodes"] = others[:6]
        self.status["topic_count"] = len(topics)
        self.status["has_cmd_vel"] = any(name == "/cmd_vel" for (name, _typ) in topics)

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def on_clicked_point(self, msg: PointStamped):
        self.point_counter += 1
        self.points.append((self.point_counter, f"{msg.point.x:.3f}", f"{msg.point.y:.3f}", time.strftime("%H:%M:%S")))

    def on_scale(self, msg: Twist):
        self.scale_lin = max(0.0, float(msg.linear.x))
        self.scale_ang = max(0.0, float(msg.angular.z))

    def on_override(self, msg: Twist):
        self.override_twist = msg

    def on_override_enable(self, msg: Bool):
        self.override_enable = bool(msg.data)

    def _run_gui(self):
        root = tk.Tk()
        root.title("Path Follower Control")
        
        # Top bar
        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")
        ttk.Button(top, text="Start", command=self.on_click_start).pack(side="left", padx=4)
        ttk.Button(top, text="Stop",  command=self.on_click_stop ).pack(side="left", padx=4)
        ttk.Button(top, text="Reset Path", command=self.on_click_reset).pack(side="left", padx=4)
        self.var_edit = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Edit Mode", variable=self.var_edit,
                        command=self.on_toggle_edit).pack(side="left", padx=12)

        # Status panel
        status = ttk.LabelFrame(root, text="ROS / Network Status", padding=10)
        status.pack(fill="x", padx=10, pady=(0,10))
        self.lbl_dom  = ttk.Label(status, text="DOMAIN: -")
        self.lbl_rmw  = ttk.Label(status, text="RMW: -")
        self.lbl_loc  = ttk.Label(status, text="LOCALHOST_ONLY: -")
        self.lbl_host = ttk.Label(status, text="Host/IP: -")
        self.lbl_nodes= ttk.Label(status, text="Nodes: -")
        self.lbl_topics=ttk.Label(status, text="Topics: -")
        for i,w in enumerate([self.lbl_dom, self.lbl_rmw, self.lbl_loc, self.lbl_host, self.lbl_nodes, self.lbl_topics]):
            w.grid(row=i//3, column=i%3, sticky="w", padx=6, pady=2)

        # Middle: points + cmd_vel
        mid = ttk.Frame(root, padding=(10, 0, 10, 10))
        mid.pack(fill="both", expand=True)
        
        left = ttk.Frame(mid)
        left.pack(side="left", fill="both", expand=True, padx=(0,10))
        ttk.Label(left, text="Clicked Points (latest 50)").pack(anchor="w")
        self.tree = ttk.Treeview(left, columns=("#","x","y","time"), show="headings", height=10)
        for c in ("#","x","y","time"):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=80, anchor="center")
        self.tree.pack(fill="both", expand=True)

        right = ttk.Frame(mid)
        right.pack(side="left", fill="y")
        ttk.Label(right, text="Current /cmd_vel").pack(anchor="w")
        self.lbl_v = ttk.Label(right, text="linear.x: 0.000 m/s")
        self.lbl_w = ttk.Label(right, text="angular.z: 0.000 rad/s")
        self.lbl_age = ttk.Label(right, text="age: - s")
        self.lbl_v.pack(anchor="w", pady=(6,0))
        self.lbl_w.pack(anchor="w")
        self.lbl_age.pack(anchor="w")
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=10)

        # Bottom: scale & override
        bottom = ttk.Frame(root, padding=(10, 0, 10, 10))
        bottom.pack(fill="x")
        ttk.Label(bottom, text="Scale (multiplier)").grid(row=0, column=0, sticky="w", columnspan=3)
        self.s_lin = tk.DoubleVar(value=1.0)
        self.s_ang = tk.DoubleVar(value=1.0)
        ttk.Label(bottom, text="linear.x ×").grid(row=1, column=0, sticky="e")
        ttk.Scale(bottom, from_=0.0, to=1.5, orient="horizontal", variable=self.s_lin).grid(row=1, column=1, sticky="ew", padx=6)
        ttk.Label(bottom, text="angular.z ×").grid(row=2, column=0, sticky="e")
        ttk.Scale(bottom, from_=0.0, to=1.5, orient="horizontal", variable=self.s_ang).grid(row=2, column=1, sticky="ew", padx=6)
        ttk.Button(bottom, text="Apply Scale", command=self.on_apply_scale).grid(row=1, column=2, rowspan=2, padx=6)

        ttk.Separator(bottom, orient="horizontal").grid(row=3, column=0, columnspan=3, sticky="ew", pady=8)
        self.var_ovren = tk.BooleanVar(value=False)
        ttk.Checkbutton(bottom, text="Manual Override /cmd_vel", variable=self.var_ovren,
                        command=self.on_toggle_override).grid(row=4, column=0, columnspan=3, sticky="w")
        self.ovr_lin = tk.DoubleVar(value=0.0)
        self.ovr_ang = tk.DoubleVar(value=0.0)
        ttk.Label(bottom, text="linear.x").grid(row=5, column=0, sticky="e")
        ttk.Scale(bottom, from_=-1.0, to=1.0, orient="horizontal", variable=self.ovr_lin,
                  command=lambda _: self.publish_override_if_enabled()).grid(row=5, column=1, sticky="ew", padx=6)
        ttk.Label(bottom, text="angular.z").grid(row=6, column=0, sticky="e")
        ttk.Scale(bottom, from_=-3.5, to=3.5, orient="horizontal", variable=self.ovr_ang,
                  command=lambda _: self.publish_override_if_enabled()).grid(row=6, column=1, sticky="ew", padx=6)
        bottom.grid_columnconfigure(1, weight=1)

        def refresh():
            st = self.status
            self.lbl_dom.config(text=f"DOMAIN: {st['domain_id']}")
            self.lbl_rmw.config(text=f"RMW: {st['rmw']}")
            self.lbl_loc.config(text=f"LOCALHOST_ONLY: {st['localhost_only']}")
            self.lbl_host.config(text=f"Host/IP: {st['host']} / {st['ip']}")
            self.lbl_nodes.config(text=f"Nodes: {st['node_count']} (others: {len(st['other_nodes'])})")
            self.lbl_topics.config(text=f"Topics: {st['topic_count']}   /cmd_vel: {'YES' if st['has_cmd_vel'] else 'NO'}")

            self.tree.delete(*self.tree.get_children())
            for row in list(self.points)[-12:][::-1]:
                self.tree.insert("", "end", values=row)

            v = self.last_cmd.linear.x
            w = self.last_cmd.angular.z
            age = time.time() - self.last_cmd_time if self.last_cmd_time else float("inf")
            self.lbl_v.config(text=f"linear.x: {v:+.3f} m/s")
            self.lbl_w.config(text=f"angular.z: {w:+.3f} rad/s")
            self.lbl_age.config(text=f"age: {age:.2f} s")

            root.after(200, refresh)

        refresh()
        root.mainloop()

    def on_click_start(self):
        self.pub_start.publish(Empty())
        self.get_logger().info("▶ Start")
    
    def on_click_stop(self):
        self.pub_stop.publish(Empty())
        self.get_logger().info("⏸ Stop")
    
    def on_click_reset(self):
        self.pub_reset.publish(Empty())
        self.get_logger().info("🔄 Reset")
    
    def on_toggle_edit(self):
        self.pub_edit.publish(Bool(data=self.var_edit.get()))
    
    def on_apply_scale(self):
        msg = Twist()
        msg.linear.x = float(self.s_lin.get())
        msg.angular.z = float(self.s_ang.get())
        self.pub_scale.publish(msg)
        self.get_logger().info(f"Scale: lin={self.s_lin.get():.2f}, ang={self.s_ang.get():.2f}")
    
    def on_toggle_override(self):
        self.pub_ovr_en.publish(Bool(data=self.var_ovren.get()))
        if self.var_ovren.get():
            self.publish_override_if_enabled()
    
    def publish_override_if_enabled(self):
        if not self.var_ovren.get():
            return
        msg = Twist()
        msg.linear.x = float(self.ovr_lin.get())
        msg.angular.z = float(self.ovr_ang.get())
        self.pub_ovr.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
