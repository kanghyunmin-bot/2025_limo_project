#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import rclpy

from .node import ControlPanelNode
from .handlers import EventHandlers
from .widgets import (
    ControlModeFrame, DriveModeFrame, PathInterpolationFrame,
    PathSourceFrame, DrivingModeFrame, ControlButtonsFrame,
    VelocityFrame, ManualControlFrame, PathInfoFrame,
    WaypointsListFrame, StatusFrame, AccuracyFrame, ConstraintRadiusFrame,
    PlannerModeFrame,
)

class ControlPanelGUI:
    def __init__(self, node, root):
        self.node = node
        self.root = root
        self.root.title("🚗 Path Follower Control Panel v2.9")
        self.root.geometry("1000x1050")
        
        main_frame = ttk.Frame(root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        self.widgets = {}
        self.handlers = EventHandlers(node, self.widgets)
        
        self.widgets['control_mode'] = ControlModeFrame(main_frame, self.handlers)
        self.widgets['control_mode'].pack(fill=tk.X, pady=5)
        
        self.widgets['drive_mode'] = DriveModeFrame(main_frame, self.handlers)
        self.widgets['drive_mode'].pack(fill=tk.X, pady=5)
        
        self.widgets['interpolation'] = PathInterpolationFrame(main_frame, self.handlers)
        self.widgets['interpolation'].pack(fill=tk.X, pady=5)

        self.widgets['path_source'] = PathSourceFrame(main_frame, self.handlers)
        self.widgets['path_source'].pack(fill=tk.X, pady=5)

        self.widgets['planner_mode'] = PlannerModeFrame(main_frame, self.handlers)
        self.widgets['planner_mode'].pack(fill=tk.X, pady=5)

        self.widgets['driving_mode'] = DrivingModeFrame(main_frame, self.handlers)
        self.widgets['driving_mode'].pack(fill=tk.X, pady=5)

        self.widgets['control_buttons'] = ControlButtonsFrame(main_frame, self.handlers)
        self.widgets['control_buttons'].pack(fill=tk.X, pady=5)

        self.widgets['constraint_radius'] = ConstraintRadiusFrame(main_frame, self.handlers)
        self.widgets['constraint_radius'].pack(fill=tk.X, pady=5)

        self.widgets['velocity'] = VelocityFrame(main_frame, self.handlers)
        self.widgets['velocity'].pack(fill=tk.X, pady=5)
        
        self.widgets['manual_control'] = ManualControlFrame(main_frame, self.handlers)
        self.widgets['manual_control'].pack(fill=tk.X, pady=5)
        
        self.widgets['path_info'] = PathInfoFrame(main_frame)
        self.widgets['path_info'].pack(fill=tk.X, pady=5)
        
        # ✅ 정확도 프레임 추가
        self.widgets['accuracy'] = AccuracyFrame(main_frame)
        self.widgets['accuracy'].pack(fill=tk.X, pady=5)
        
        self.widgets['waypoints_list'] = WaypointsListFrame(main_frame)
        self.widgets['waypoints_list'].pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.widgets['status'] = StatusFrame(main_frame)
        self.widgets['status'].pack(fill=tk.X, pady=5)
        
        self.setup_key_bindings()
        self.root.after(500, self.update_path_info)
        
        self.node.get_logger().info("✅ Control Panel GUI initialized")
    
    def setup_key_bindings(self):
        self.root.bind('<KeyPress-w>', lambda e: self.handlers.handle_key_press('w'))
        self.root.bind('<KeyPress-a>', lambda e: self.handlers.handle_key_press('a'))
        self.root.bind('<KeyPress-s>', lambda e: self.handlers.handle_key_press('s'))
        self.root.bind('<KeyPress-d>', lambda e: self.handlers.handle_key_press('d'))
        
        self.root.bind('<KeyRelease-w>', lambda e: self.handlers.handle_key_release('w'))
        self.root.bind('<KeyRelease-a>', lambda e: self.handlers.handle_key_release('a'))
        self.root.bind('<KeyRelease-s>', lambda e: self.handlers.handle_key_release('s'))
        self.root.bind('<KeyRelease-d>', lambda e: self.handlers.handle_key_release('d'))
    
    def update_path_info(self):
        if self.node.current_path is not None and len(self.node.current_path.poses) > 0:
            num_points = len(self.node.current_path.poses)
            
            total_dist = 0.0
            for i in range(len(self.node.current_path.poses) - 1):
                p1 = self.node.current_path.poses[i].pose.position
                p2 = self.node.current_path.poses[i+1].pose.position
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                total_dist += (dx*dx + dy*dy) ** 0.5
            
            avg_speed = 0.5
            time_est = total_dist / avg_speed if avg_speed > 0 else 0
            
            self.widgets['path_info'].label_waypoints.config(text=str(num_points))
            self.widgets['path_info'].label_distance.config(text=f"{total_dist:.2f} m")
            self.widgets['path_info'].label_time.config(text=f"{time_est:.1f}s")
            
            listbox = self.widgets['waypoints_list'].listbox
            listbox.delete(0, tk.END)
            for i, pose in enumerate(self.node.current_path.poses[:10]):
                x = pose.pose.position.x
                y = pose.pose.position.y
                listbox.insert(tk.END, f"WP{i+1}: x={x:.2f}m, y={y:.2f}m")
            
            if num_points > 10:
                listbox.insert(tk.END, f"... (+{num_points - 10} more)")
        
        self.root.after(500, self.update_path_info)

def main():
    rclpy.init()
    node = ControlPanelNode()
    
    root = tk.Tk()
    gui = ControlPanelGUI(node, root)
    
    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.01)
        root.after(10, spin_once)
    
    root.after(10, spin_once)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
