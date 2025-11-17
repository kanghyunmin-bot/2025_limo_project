#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk

class ControlModeFrame:
    """제어 알고리즘 선택"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="Control Algorithm", padding=10)
        self.handler = handler
        
        self.btn_pure_pursuit = ttk.Button(
            self.frame, text="Pure Pursuit",
            command=lambda: handler.set_control_mode('pure_pursuit'), width=25
        )
        self.btn_pure_pursuit.pack(side=tk.LEFT, padx=5)
        
        self.btn_stanley = ttk.Button(
            self.frame, text="Stanley Method",
            command=lambda: handler.set_control_mode('stanley'), width=25
        )
        self.btn_stanley.pack(side=tk.LEFT, padx=5)
        
        self.label = ttk.Label(
            self.frame, text="현재: Stanley Method",
            foreground="blue", font=("Arial", 10, "bold")
        )
        self.label.pack(side=tk.LEFT, padx=10)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class DriveModeFrame:
    """주행 모드 선택"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="Drive Type", padding=10)
        self.handler = handler
        
        self.btn_differential = ttk.Button(
            self.frame, text="Differential Drive",
            command=lambda: handler.set_drive_mode('differential'), width=25
        )
        self.btn_differential.pack(side=tk.LEFT, padx=5)
        
        self.btn_ackermann = ttk.Button(
            self.frame, text="Ackermann Drive",
            command=lambda: handler.set_drive_mode('ackermann'), width=25
        )
        self.btn_ackermann.pack(side=tk.LEFT, padx=5)
        
        self.label = ttk.Label(
            self.frame, text="현재: Ackermann",
            foreground="orange", font=("Arial", 10, "bold")
        )
        self.label.pack(side=tk.LEFT, padx=10)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class PathInterpolationFrame:
    """경로 보간 방법 선택"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="Path Interpolation Method", padding=10)
        self.handler = handler
        
        self.interp_var = tk.StringVar(value='spline')
        
        methods = [
            ('none', '⚡ No Interpolation (최고 효율)'),
            ('linear', '📏 Linear (직선 연결)'),
            ('subsample', '📍 Subsampling (균등 간격)'),
            ('spline', '🌀 Cubic Spline (부드러움)'),
            ('bezier', '🎨 Bézier (Ackermann 제약)'),
            ('local_bezier', '🚀 Local Bézier (실시간 최적화)')
        ]
        
        for method, label in methods:
            rb = ttk.Radiobutton(
                self.frame,
                text=label,
                variable=self.interp_var,
                value=method,
                command=lambda m=method: handler.set_interpolation_method(m)
            )
            rb.pack(anchor='w', pady=2)
        
        ttk.Label(
            self.frame,
            text="  💡 Local Bézier: 전역 경로 기반 앞 1m만 실시간 최적화",
            foreground='gray',
            font=('TkDefaultFont', 9)
        ).pack(anchor='w', padx=20, pady=5)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class PathSourceFrame:
    """경로 소스 선택"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="Path Source", padding=10)
        self.handler = handler
        
        self.btn_clicked_point = ttk.Button(
            self.frame, text="RViz Clicked Point",
            command=lambda: handler.set_path_source('clicked_point'), width=25
        )
        self.btn_clicked_point.pack(side=tk.LEFT, padx=5)
        
        self.btn_planner_path = ttk.Button(
            self.frame, text="Planner Path",
            command=lambda: handler.set_path_source('planner_path'), width=25
        )
        self.btn_planner_path.pack(side=tk.LEFT, padx=5)
        
        self.label = ttk.Label(
            self.frame, text="현재: RViz Clicked Point",
            foreground="blue", font=("Arial", 10, "bold")
        )
        self.label.pack(side=tk.LEFT, padx=10)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class DrivingModeFrame:
    """AUTO/MANUAL 모드"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="Driving Mode", padding=10)
        self.handler = handler
        
        ttk.Button(self.frame, text="AUTO", command=handler.set_auto_mode, width=20).pack(side=tk.LEFT, padx=5)
        ttk.Button(self.frame, text="MANUAL (W/A/S/D)", command=handler.set_manual_mode, width=20).pack(side=tk.LEFT, padx=5)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class ControlButtonsFrame:
    """제어 버튼"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="AUTO Control", padding=10)
        self.handler = handler
        
        ttk.Button(self.frame, text="START", command=handler.on_start, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(self.frame, text="STOP", command=handler.on_stop, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(self.frame, text="RESET", command=handler.on_reset, width=15).pack(side=tk.LEFT, padx=5)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class VelocityFrame:
    """속도 설정"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="Velocity Control", padding=10)
        self.handler = handler
        
        ttk.Label(self.frame, text="Max Speed (m/s):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.slider_v_max = ttk.Scale(self.frame, from_=10, to=500, orient=tk.HORIZONTAL, command=handler.update_velocity_params)
        self.slider_v_max.set(288)
        self.slider_v_max.grid(row=0, column=1, sticky=tk.EW, padx=5)
        self.label_v_max = ttk.Label(self.frame, text="2.88 m/s", width=10)
        self.label_v_max.grid(row=0, column=2, padx=5)
        
        ttk.Label(self.frame, text="Min Speed (m/s):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.slider_v_min = ttk.Scale(self.frame, from_=5, to=200, orient=tk.HORIZONTAL, command=handler.update_velocity_params)
        self.slider_v_min.set(104)
        self.slider_v_min.grid(row=1, column=1, sticky=tk.EW, padx=5)
        self.label_v_min = ttk.Label(self.frame, text="1.04 m/s", width=10)
        self.label_v_min.grid(row=1, column=2, padx=5)
        
        self.frame.columnconfigure(1, weight=1)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class ManualControlFrame:
    """수동 제어"""
    def __init__(self, parent, handler):
        self.frame = ttk.LabelFrame(parent, text="Manual Control (WASD)", padding=10)
        self.handler = handler
        
        ttk.Label(self.frame, text="Speed (m/s):").pack(side=tk.LEFT, padx=5)
        self.slider_manual_speed = ttk.Scale(self.frame, from_=0, to=200, orient=tk.HORIZONTAL, command=handler.update_manual_display)
        self.slider_manual_speed.set(30)
        self.slider_manual_speed.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.label_manual_speed = ttk.Label(self.frame, text="0.30 m/s", width=10)
        self.label_manual_speed.pack(side=tk.LEFT, padx=5)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class PathInfoFrame:
    """경로 정보"""
    def __init__(self, parent):
        self.frame = ttk.LabelFrame(parent, text="Path Information", padding=10)
        
        info_subframe = ttk.Frame(self.frame)
        info_subframe.pack(fill=tk.X)
        
        ttk.Label(info_subframe, text="Waypoints: ").pack(side=tk.LEFT, padx=5)
        self.label_waypoints = ttk.Label(info_subframe, text="0", foreground="green", font=("Arial", 12, "bold"))
        self.label_waypoints.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(info_subframe, text="Distance: ").pack(side=tk.LEFT, padx=5)
        self.label_distance = ttk.Label(info_subframe, text="0.00 m", foreground="orange", font=("Arial", 12, "bold"))
        self.label_distance.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(info_subframe, text="Time Est: ").pack(side=tk.LEFT, padx=5)
        self.label_time = ttk.Label(info_subframe, text="0s", foreground="purple", font=("Arial", 12, "bold"))
        self.label_time.pack(side=tk.LEFT, padx=5)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class AccuracyFrame:
    """정확도 표시"""
    def __init__(self, parent):
        self.frame = ttk.LabelFrame(parent, text="📊 Path Tracking Accuracy", padding=10)
        
        self.label_accuracy = ttk.Label(
            self.frame, text="Accuracy: -- %",
            font=("Arial", 14, "bold"), foreground="blue"
        )
        self.label_accuracy.pack()
        
        self.progress = ttk.Progressbar(self.frame, length=400, mode='determinate')
        self.progress.pack(pady=5)
        
        self.label_error = ttk.Label(self.frame, text="Avg Error: -- m", font=("Arial", 10))
        self.label_error.pack()
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)
    
    def update_accuracy(self, accuracy):
        self.label_accuracy.config(text=f"Accuracy: {accuracy:.2f} %")
        self.progress['value'] = accuracy
        
        avg_error = (100.0 - accuracy) / 100.0
        self.label_error.config(text=f"Avg Error: {avg_error:.3f} m")
        
        if accuracy >= 95:
            self.label_accuracy.config(foreground="green")
        elif accuracy >= 85:
            self.label_accuracy.config(foreground="orange")
        else:
            self.label_accuracy.config(foreground="red")

class WaypointsListFrame:
    """Waypoints 리스트"""
    def __init__(self, parent):
        self.frame = ttk.LabelFrame(parent, text="Waypoints", padding=10)
        
        scrollbar = ttk.Scrollbar(self.frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.listbox = tk.Listbox(self.frame, height=8, yscrollcommand=scrollbar.set)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.listbox.yview)
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)

class StatusFrame:
    """상태 표시"""
    def __init__(self, parent):
        self.frame = ttk.LabelFrame(parent, text="Status", padding=10)
        
        self.label = ttk.Label(
            self.frame,
            text="Ready | AUTO | Stanley | Ackermann",
            font=("Arial", 11, "bold"),
            foreground="red"
        )
        self.label.pack()
    
    def pack(self, **kwargs):
        self.frame.pack(**kwargs)
