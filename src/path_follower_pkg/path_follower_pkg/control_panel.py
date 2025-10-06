#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
import tkinter as tk


class ControlPanel(Node):
    def __init__(self):
        super().__init__("path_follower_gui")
        
        # Publishers
        self.pub_start = self.create_publisher(Empty, "/path_follower/start", 10)
        self.pub_stop = self.create_publisher(Empty, "/path_follower/stop", 10)
        self.pub_reset = self.create_publisher(Empty, "/path_follower/reset", 10)
        self.pub_edit = self.create_publisher(Bool, "/path_follower/edit_mode", 10)
        
        # GUI 초기화
        self.root = tk.Tk()
        self.root.title("Path Follower Control")
        self._setup_gui()
        
        # 타이머로 GUI 업데이트 (100Hz)
        self.create_timer(0.01, self.update_gui)
        
        self.get_logger().info("Control Panel GUI started")
    
    def _setup_gui(self):
        frm = tk.Frame(self.root, padx=12, pady=12)
        frm.pack()
        
        tk.Label(frm, text="Path Follower Control", 
                 font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(0,10))
        
        tk.Button(frm, text="Start", width=14, bg='green', fg='white',
                  command=self.on_start).grid(row=1, column=0, padx=4, pady=4)
        tk.Button(frm, text="Stop", width=14, bg='red', fg='white',
                  command=self.on_stop).grid(row=1, column=1, padx=4, pady=4)
        tk.Button(frm, text="Reset Path", width=14, bg='orange', fg='white',
                  command=self.on_reset).grid(row=2, column=0, padx=4, pady=4)
        
        self.edit_var = tk.BooleanVar(value=False)
        tk.Checkbutton(frm, text="Edit Mode", 
                       variable=self.edit_var,
                       command=self.on_toggle_edit).grid(row=2, column=1, padx=4, pady=4)
    
    def update_gui(self):
        """ROS 타이머에서 호출되어 GUI 업데이트"""
        try:
            self.root.update()
        except:
            rclpy.shutdown()
    
    def on_start(self):
        self.pub_start.publish(Empty())
        self.get_logger().info("▶ Start")
    
    def on_stop(self):
        self.pub_stop.publish(Empty())
        self.get_logger().info("⏸ Stop")
    
    def on_reset(self):
        self.pub_reset.publish(Empty())
        self.get_logger().info("🔄 Reset")
    
    def on_toggle_edit(self):
        v = self.edit_var.get()
        self.pub_edit.publish(Bool(data=v))
        self.get_logger().info(f"✏️  Edit mode: {v}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanel()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'root'):
            node.root.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
