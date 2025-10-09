#!/usr/bin/env python3
import os
import socket
import time
import subprocess
import signal
from collections import deque
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Empty, Bool


class ControlPanel(Node):
    def __init__(self):
        super().__init__("path_follower_gui")

        # Publishers
        self.pub_start = self.create_publisher(Empty, "/path_follower/start", 10)
        self.pub_stop = self.create_publisher(Empty, "/path_follower/stop", 10)
        self.pub_reset = self.create_publisher(Empty, "/path_follower/reset", 10)
        self.pub_edit = self.create_publisher(Bool, "/path_follower/edit_mode", 10)
        self.pub_scale = self.create_publisher(Twist, "/path_follower/cmd_vel_scale", 10)
        self.pub_ovr = self.create_publisher(Twist, "/path_follower/cmd_vel_override", 10)
        self.pub_ovr_en = self.create_publisher(Bool, "/path_follower/override_enable", 10)

        # Subscribers
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.sub_clicked = self.create_subscription(PointStamped, "/clicked_point", self.on_clicked_point, 10)

        # State
        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.points = deque(maxlen=50)
        self.point_counter = 0
        self.override_enable = False
        self.wasd_keys = set()
        self.wasd_linear = 0.5
        self.wasd_angular = 1.0

        # LIMO
        self.limo_ip = "192.168.1.100"
        self.limo_user = "limo"
        self.ssh_ok = False

        # Network
        self.domain = os.getenv("ROS_DOMAIN_ID", "0")
        self.rmw = os.getenv("RMW_IMPLEMENTATION", "unknown")
        self.localhost = os.getenv("ROS_LOCALHOST_ONLY", "0")
        self.host = socket.gethostname()
        self.ip = self._get_ip()

        # Timers
        self.create_timer(0.05, self._wasd_update)

        # GUI
        self.gui_thread = threading.Thread(target=self._gui_main, daemon=True)
        self.gui_thread.start()

        self.get_logger().info("GUI started")

    def _get_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "unknown"

    def _wasd_update(self):
        if not self.override_enable:
            return
        
        v = 0.0
        w = 0.0
        if 'w' in self.wasd_keys: v += self.wasd_linear
        if 's' in self.wasd_keys: v -= self.wasd_linear
        if 'a' in self.wasd_keys: w += self.wasd_angular
        if 'd' in self.wasd_keys: w -= self.wasd_angular
        
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_ovr.publish(msg)

    def on_cmd_vel(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def on_clicked_point(self, msg):
        self.point_counter += 1
        self.points.append((
            self.point_counter,
            f"{msg.point.x:.3f}",
            f"{msg.point.y:.3f}",
            time.strftime("%H:%M:%S")
        ))

    def _gui_main(self):
        root = tk.Tk()
        root.title("Path Follower Control")
        root.geometry("750x650")

        # === TOP BUTTONS ===
        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")
        ttk.Button(top, text="Start", command=lambda: self.pub_start.publish(Empty())).pack(side="left", padx=4)
        ttk.Button(top, text="Stop", command=lambda: self.pub_stop.publish(Empty())).pack(side="left", padx=4)
        ttk.Button(top, text="Reset", command=lambda: self.pub_reset.publish(Empty())).pack(side="left", padx=4)
        
        edit_var = tk.BooleanVar()
        ttk.Checkbutton(top, text="Edit", variable=edit_var,
                        command=lambda: self.pub_edit.publish(Bool(data=edit_var.get()))).pack(side="left", padx=10)

        # === LIMO ===
        limo_frm = ttk.LabelFrame(root, text="🤖 LIMO Remote", padding=10)
        limo_frm.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(limo_frm, text="IP:").grid(row=0, column=0, sticky="e", padx=5)
        ip_var = tk.StringVar(value=self.limo_ip)
        ttk.Entry(limo_frm, textvariable=ip_var, width=15).grid(row=0, column=1, sticky="w")
        ttk.Button(limo_frm, text="Test", command=lambda: self._test_ssh(ip_var.get(), lbl_ssh)).grid(row=0, column=2, padx=5)
        
        lbl_ssh = ttk.Label(limo_frm, text="⚪ 미확인", foreground="gray")
        lbl_ssh.grid(row=1, column=0, columnspan=3, sticky="w", pady=5)

        # === NETWORK ===
        net_frm = ttk.LabelFrame(root, text="Network Config", padding=10)
        net_frm.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(net_frm, text="DOMAIN:").grid(row=0, column=0, sticky="e")
        domain_var = tk.StringVar(value=self.domain)
        ttk.Entry(net_frm, textvariable=domain_var, width=5).grid(row=0, column=1, sticky="w", padx=5)
        
        ttk.Label(net_frm, text="RMW:").grid(row=1, column=0, sticky="e")
        rmw_var = tk.StringVar(value=self.rmw)
        ttk.Combobox(net_frm, textvariable=rmw_var, width=18,
                     values=["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"]).grid(row=1, column=1, sticky="w", padx=5)
        
        ttk.Label(net_frm, text="LOCALHOST:").grid(row=2, column=0, sticky="e")
        local_var = tk.StringVar(value=self.localhost)
        ttk.Combobox(net_frm, textvariable=local_var, width=5, values=["0", "1"]).grid(row=2, column=1, sticky="w", padx=5)
        
        ttk.Button(net_frm, text="Apply & Restart",
                   command=lambda: self._apply_restart(domain_var.get(), rmw_var.get(), local_var.get(), ip_var.get())).grid(row=0, column=2, rowspan=3, padx=10)
        
        ttk.Label(net_frm, text=f"Host: {self.host} / {self.ip}").grid(row=3, column=0, columnspan=3, sticky="w", pady=5)

        # === POINTS ===
        mid = ttk.Frame(root, padding=10)
        mid.pack(fill="both", expand=True)
        
        ttk.Label(mid, text="Clicked Points").pack(anchor="w")
        tree = ttk.Treeview(mid, columns=("#", "x", "y", "time"), show="headings", height=5)
        for c in ("#", "x", "y", "time"):
            tree.heading(c, text=c)
            tree.column(c, width=60, anchor="center")
        tree.pack(fill="both", expand=True)
        
        lbl_v = ttk.Label(mid, text="linear.x: 0.000")
        lbl_w = ttk.Label(mid, text="angular.z: 0.000")
        lbl_v.pack(anchor="w", pady=(10, 0))
        lbl_w.pack(anchor="w")

        # === CONTROL ===
        ctrl = ttk.Frame(root, padding=10)
        ctrl.pack(fill="x")
        
        ttk.Label(ctrl, text="Scale").grid(row=0, column=0, sticky="w", columnspan=2)
        
        s_lin = tk.DoubleVar(value=1.0)
        s_ang = tk.DoubleVar(value=1.0)
        
        ttk.Label(ctrl, text="linear ×").grid(row=1, column=0, sticky="e")
        ttk.Scale(ctrl, from_=0, to=1.5, variable=s_lin).grid(row=1, column=1, sticky="ew", padx=5)
        
        ttk.Label(ctrl, text="angular ×").grid(row=2, column=0, sticky="e")
        ttk.Scale(ctrl, from_=0, to=1.5, variable=s_ang).grid(row=2, column=1, sticky="ew", padx=5)
        
        ttk.Button(ctrl, text="Apply", command=lambda: self._apply_scale(s_lin.get(), s_ang.get())).grid(row=1, column=2, rowspan=2)
        
        ttk.Separator(ctrl, orient="horizontal").grid(row=3, column=0, columnspan=3, sticky="ew", pady=10)
        
        ovr_var = tk.BooleanVar()
        ttk.Checkbutton(ctrl, text="Override (WASD)", variable=ovr_var,
                        command=lambda: self._toggle_ovr(ovr_var.get())).grid(row=4, column=0, columnspan=3, sticky="w")
        
        lbl_wasd = ttk.Label(ctrl, text="W/A/S/D")
        lbl_wasd.grid(row=5, column=0, columnspan=3, sticky="w", pady=5)
        
        lin_scale = tk.DoubleVar(value=0.5)
        ang_scale = tk.DoubleVar(value=1.0)
        
        ttk.Label(ctrl, text="Speed:").grid(row=6, column=0, sticky="e")
        ttk.Scale(ctrl, from_=0.1, to=1.0, variable=lin_scale,
                  command=lambda _: setattr(self, 'wasd_linear', lin_scale.get())).grid(row=6, column=1, sticky="ew", padx=5)
        
        ttk.Label(ctrl, text="Turn:").grid(row=7, column=0, sticky="e")
        ttk.Scale(ctrl, from_=0.1, to=2.0, variable=ang_scale,
                  command=lambda _: setattr(self, 'wasd_angular', ang_scale.get())).grid(row=7, column=1, sticky="ew", padx=5)
        
        ctrl.grid_columnconfigure(1, weight=1)

        # === KEYS ===
        root.bind('<KeyPress>', lambda e: self.wasd_keys.add(e.keysym.lower()) if e.keysym.lower() in 'wasd' else None)
        root.bind('<KeyRelease>', lambda e: self.wasd_keys.discard(e.keysym.lower()) if e.keysym.lower() in 'wasd' else None)

        # === UPDATE ===
        def update():
            tree.delete(*tree.get_children())
            for row in list(self.points)[-6:][::-1]:
                tree.insert("", "end", values=row)
            
            v = self.last_cmd.linear.x
            w = self.last_cmd.angular.z
            lbl_v.config(text=f"linear.x: {v:+.3f} m/s")
            lbl_w.config(text=f"angular.z: {w:+.3f} rad/s")
            
            if self.override_enable:
                keys = ','.join(sorted(self.wasd_keys)) if self.wasd_keys else "-"
                lbl_wasd.config(text=f"Active | Keys: {keys}")
            else:
                lbl_wasd.config(text="W/A/S/D (disabled)")
            
            root.after(200, update)
        
        update()
        root.mainloop()

    def _test_ssh(self, ip, label):
        self.limo_ip = ip
        try:
            res = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=2',
                                  f'{self.limo_user}@{ip}', 'echo OK'],
                                 capture_output=True, timeout=3)
            if res.returncode == 0:
                self.ssh_ok = True
                label.config(text=f"✅ Connected: {ip}", foreground="green")
            else:
                raise Exception()
        except:
            self.ssh_ok = False
            label.config(text="❌ Failed", foreground="red")

    def _apply_scale(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub_scale.publish(msg)
        self.get_logger().info(f"Scale: {lin:.2f}, {ang:.2f}")

    def _toggle_ovr(self, enabled):
        self.override_enable = enabled
        self.pub_ovr_en.publish(Bool(data=enabled))
        if not enabled:
            self.wasd_keys.clear()

    def _apply_restart(self, domain, rmw, localhost, limo_ip):
        msg = f"Restart with:\nDOMAIN: {domain}\nRMW: {rmw}\nLOCALHOST: {localhost}\n\n"
        msg += "노트북: 자동 재시작\n"
        if self.ssh_ok:
            msg += f"LIMO ({limo_ip}): 자동 재시작"
        else:
            msg += "LIMO: 수동 재시작 필요"
        
        if not messagebox.askyesno("Restart", msg):
            return
        
        # Update bashrc
        bashrc = os.path.expanduser("~/.bashrc")
        with open(bashrc, 'r') as f:
            lines = f.readlines()
        
        lines = [l for l in lines if not any(x in l for x in ['ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY'])]
        lines.append(f"\nexport ROS_DOMAIN_ID={domain}\n")
        lines.append(f"export RMW_IMPLEMENTATION={rmw}\n")
        lines.append(f"export ROS_LOCALHOST_ONLY={localhost}\n")
        
        with open(bashrc, 'w') as f:
            f.writelines(lines)
        
        # LIMO restart
        if self.ssh_ok:
            cmd = f"""
sed -i '/ROS_DOMAIN_ID/d' ~/.bashrc; sed -i '/RMW_IMPLEMENTATION/d' ~/.bashrc; sed -i '/ROS_LOCALHOST_ONLY/d' ~/.bashrc
echo 'export ROS_DOMAIN_ID={domain}' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION={rmw}' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY={localhost}' >> ~/.bashrc
pkill -f path_follower; sleep 1
source ~/.bashrc; cd /home/limo/path_follower; source install/setup.bash
nohup ros2 launch path_follower_pkg path_follower.launch.py > /tmp/pf.log 2>&1 &
"""
            subprocess.Popen(['ssh', f'{self.limo_user}@{limo_ip}', cmd])
        
        # Local restart
        subprocess.run(['pkill', '-f', 'path_follower'])
        subprocess.Popen(['bash', '-c',
                          'sleep 2 && source ~/.bashrc && '
                          'source ~/path_follower/install/setup.bash && '
                          'ros2 launch path_follower_pkg path_follower.launch.py &'])
        
        os.kill(os.getpid(), signal.SIGTERM)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
