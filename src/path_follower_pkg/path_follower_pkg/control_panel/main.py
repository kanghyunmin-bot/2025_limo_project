#!/usr/bin/env python3
import rclpy
import tkinter as tk
from .node import ControlPanelNode
from .gui import ControlPanelGUI


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    node = ControlPanelNode()
    gui = ControlPanelGUI(node, root)
    
    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0.01)
        root.after(10, spin_ros)
    
    spin_ros()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
