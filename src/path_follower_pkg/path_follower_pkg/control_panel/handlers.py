#!/usr/bin/env python3
from std_msgs.msg import Empty, String, Bool, Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class EventHandlers:
    """GUI 이벤트 핸들러"""
    
    def __init__(self, node, widgets):
        self.node = node
        self.widgets = widgets
        
        self.control_mode = 'stanley'
        self.drive_mode = 'ackermann'
        self.path_source = 'clicked_point'
        self.use_ackermann_path = False
        self.interpolation_method = 'spline'
        self.planner_mode = 'astar'
        self.manual_mode = False
        self.keys_pressed = {}
        
        # ✅ 정확도 콜백 등록
        try:
            self.node.set_accuracy_callback(self.on_accuracy_update)
        except:
            pass

    def set_constraint_radius(self, scope: str):
        """로컬/글로벌 베지어 제약 반경 입력 처리"""
        try:
            if 'constraint_radius' not in self.widgets:
                return

            if scope == 'local':
                raw = self.widgets['constraint_radius'].entry_local.get()
                val = float(raw)
                msg = Float32()
                msg.data = val
                self.node.pub_local_constraint_radius.publish(msg)
                self.widgets['constraint_radius'].status.config(text=f"Local radius → {val:.2f} m", foreground="green")
            elif scope == 'global':
                raw = self.widgets['constraint_radius'].entry_global.get()
                val = float(raw)
                msg = Float32()
                msg.data = val
                self.node.pub_global_constraint_radius.publish(msg)
                self.widgets['constraint_radius'].status.config(text=f"Global radius → {val:.2f} m", foreground="blue")
            elif scope == 'global_clearance':
                raw = self.widgets['constraint_radius'].entry_global_clearance.get()
                val = float(raw)
                msg = Float32()
                msg.data = val
                self.node.pub_global_constraint_clearance.publish(msg)
                self.widgets['constraint_radius'].status.config(
                    text=f"Global clearance → {val:.2f} m", foreground="purple"
                )
        except Exception as e:
            try:
                self.widgets['constraint_radius'].status.config(text=f"입력 오류: {e}", foreground="red")
            except:
                pass

    def set_planner_mode(self, mode: str):
        """글로벌 플래너 선택 (RRT / A* / Dijkstra / APF)"""
        self.planner_mode = mode
        msg = String()
        msg.data = mode
        self.node.pub_planner_mode.publish(msg)

        names = {
            'rrt': 'RRT',
            'astar': 'A*',
            'dijkstra': 'Dijkstra',
            'apf': 'APF',
        }
        try:
            if 'planner_mode' in self.widgets:
                widget = self.widgets['planner_mode']
                # 라디오 버튼과 상태 라벨을 동시에 업데이트
                widget.set_mode(mode)
                widget.label.config(text=f"현재: {names.get(mode, mode).upper()}")
            self.update_status()
        except:
            pass

    def apply_apf_params(self):
        """APF 파라미터를 GUI에서 입력한 값으로 적용"""
        try:
            widget = self.widgets.get('apf_params')
            if widget is None:
                return

            values = []
            for entry in widget.entries:
                values.append(float(entry.get()))

            msg = Float32MultiArray()
            msg.data = values
            self.node.pub_apf_params.publish(msg)

            widget.status.config(text="APF 파라미터 적용 완료", foreground="green")
            self.node.get_logger().info(
                f"🧭 APF params updated: step={values[0]:.3f}, attract={values[1]:.2f}, "
                f"repel={values[2]:.2f}, infl={values[3]:.2f}, goal_tol={values[4]:.2f}, stall_tol={values[5]:.2f}"
            )
        except Exception as e:
            try:
                self.widgets['apf_params'].status.config(text=f"입력 오류: {e}", foreground="red")
            except Exception:
                pass
    
    def on_accuracy_update(self, accuracy):
        """✅ 정확도 업데이트"""
        try:
            if 'accuracy' in self.widgets:
                self.widgets['accuracy'].update_accuracy(accuracy)
        except:
            pass
    
    def set_control_mode(self, mode):
        """제어 알고리즘 변경"""
        self.control_mode = mode
        msg = String()
        msg.data = mode
        self.node.pub_control_mode.publish(msg)

        try:
            if mode == 'pure_pursuit':
                self.widgets['control_mode'].label.config(text="현재: Pure Pursuit", foreground="blue")
            elif mode == 'stanley_ff':
                self.widgets['control_mode'].label.config(text="현재: Stanley + Feedforward", foreground="dark green")
            else:
                self.widgets['control_mode'].label.config(text="현재: Stanley Method", foreground="purple")
            self.update_status()
        except:
            pass
    
    def set_drive_mode(self, mode):
        """주행 모드 변경"""
        self.drive_mode = mode
        msg = String()
        msg.data = mode
        self.node.pub_drive_mode.publish(msg)
        
        try:
            if mode == 'differential':
                self.widgets['drive_mode'].label.config(text="현재: Differential", foreground="green")
            else:
                self.widgets['drive_mode'].label.config(text="현재: Ackermann", foreground="orange")
            self.update_status()
        except:
            pass
    
    def set_interpolation_method(self, method):
        """보간 방법 변경"""
        self.interpolation_method = method
        msg = String()
        msg.data = method
        self.node.pub_interpolation_method.publish(msg)
        
        method_names = {
            'none': 'No Interpolation',
            'linear': 'Linear',
            'subsample': 'Subsampling',
            'spline': 'Cubic Spline',
            'only_global_bezier': 'Global Bézier Only',
            'bezier': 'Bézier',
            'local_bezier': 'Local Bézier'
        }
        
        self.node.get_logger().info(f"🛣️ Interpolation: {method_names.get(method, method)}")
        try:
            self.update_status()
        except:
            pass
    
    def toggle_ackermann_path(self):
        """Ackermann Path 토글"""
        try:
            self.use_ackermann_path = self.widgets['ackermann_path'].use_ackermann_var.get()
            msg = Bool()
            msg.data = self.use_ackermann_path
            self.node.pub_use_ackermann_path.publish(msg)
            
            self.node.get_logger().info(
                f"🛣️ Ackermann Path: {'ON' if self.use_ackermann_path else 'OFF'}"
            )
        except:
            pass
    
    def set_path_source(self, source):
        """경로 소스 변경"""
        self.path_source = source
        msg = String()
        msg.data = source
        self.node.pub_path_source.publish(msg)
        
        try:
            if source == 'clicked_point':
                self.widgets['path_source'].label.config(text="현재: RViz Clicked Point", foreground="blue")
            else:
                self.widgets['path_source'].label.config(text="현재: Planner Path", foreground="green")
        except:
            pass
    
    def set_auto_mode(self):
        """AUTO 모드"""
        self.manual_mode = False
        try:
            self.update_status()
        except:
            pass
    
    def set_manual_mode(self):
        """MANUAL 모드"""
        self.manual_mode = True
        self.keys_pressed = {'w': False, 'a': False, 's': False, 'd': False}
        try:
            self.update_status()
        except:
            pass
    
    def on_start(self):
        """시작"""
        self.node.pub_start.publish(Empty())
        try:
            self.update_status()
        except:
            pass
    
    def on_stop(self):
        """정지"""
        self.node.pub_stop.publish(Empty())
        try:
            self.update_status()
        except:
            pass
    
    def on_reset(self):
        """리셋"""
        self.node.pub_reset.publish(Empty())
        try:
            self.widgets['path_info'].label_waypoints.config(text="0")
            self.widgets['path_info'].label_distance.config(text="0.00 m")
            self.widgets['path_info'].label_time.config(text="0s")
            self.widgets['waypoints_list'].listbox.delete(0, 'end')
            
            if 'accuracy' in self.widgets:
                self.widgets['accuracy'].update_accuracy(0.0)
        except:
            pass
    
    def update_velocity_params(self, val=None):
        """속도 파라미터 업데이트"""
        try:
            if 'velocity' not in self.widgets:
                return
            
            v_max = self.widgets['velocity'].slider_v_max.get() / 100.0
            v_min = self.widgets['velocity'].slider_v_min.get() / 100.0
            
            self.widgets['velocity'].label_v_max.config(text=f"{v_max:.2f} m/s")
            self.widgets['velocity'].label_v_min.config(text=f"{v_min:.2f} m/s")
            
            msg = Twist()
            msg.linear.x = v_max
            msg.linear.y = v_min
            self.node.pub_velocity_params.publish(msg)
        except:
            pass
    
    def update_manual_display(self, val=None):
        """수동 속도 표시 업데이트"""
        try:
            if 'manual_control' not in self.widgets:
                return
            
            manual_v = self.widgets['manual_control'].slider_manual_speed.get() / 100.0
            self.widgets['manual_control'].label_manual_speed.config(text=f"{manual_v:.2f} m/s")
        except:
            pass
    
    def handle_key_press(self, key):
        """키 입력 처리"""
        if not self.manual_mode:
            return
        self.keys_pressed[key] = True
        self.send_manual_command()
    
    def handle_key_release(self, key):
        """키 해제 처리"""
        if key in self.keys_pressed:
            self.keys_pressed[key] = False
        
        if not any(self.keys_pressed.values()):
            self.node.pub_cmd_vel.publish(Twist())
        else:
            self.send_manual_command()
    
    def send_manual_command(self):
        """수동 명령 발행"""
        if not self.manual_mode:
            return
        
        try:
            if 'manual_control' not in self.widgets:
                return
            
            manual_v = self.widgets['manual_control'].slider_manual_speed.get() / 100.0
            msg = Twist()
            
            if self.keys_pressed.get('w', False):
                msg.linear.x = manual_v
            elif self.keys_pressed.get('s', False):
                msg.linear.x = -manual_v
            
            if self.keys_pressed.get('a', False):
                msg.angular.z = manual_v
            elif self.keys_pressed.get('d', False):
                msg.angular.z = -manual_v
            
            self.node.pub_cmd_vel.publish(msg)
        except:
            pass
    
    def update_status(self):
        """상태 라벨 업데이트"""
        try:
            if 'status' not in self.widgets:
                return

            mode_str = "MANUAL" if self.manual_mode else "AUTO"
            if self.control_mode == 'pure_pursuit':
                control_str = "Pure Pursuit"
            elif self.control_mode == 'stanley_ff':
                control_str = "Stanley+FF"
            else:
                control_str = "Stanley"

            drive_type_str = "Differential" if self.drive_mode == 'differential' else "Ackermann"
            
            interp_short = {
                'none': 'NoInterp',
                'linear': 'Linear',
                'subsample': 'Subsamp',
                'spline': 'Spline',
                'only_global_bezier': 'GlobBez',
                'bezier': 'Bézier',
                'local_bezier': 'LocalBez'
            }
            interp_str = interp_short.get(self.interpolation_method, self.interpolation_method)

            planner_short = {
                'rrt': 'RRT',
                'astar': 'A*',
                'dijkstra': 'Dijk',
                'apf': 'APF',
            }
            planner_str = planner_short.get(self.planner_mode, self.planner_mode)

            self.widgets['status'].label.config(
                text=f"Ready | {mode_str} | {control_str} | {drive_type_str} | {interp_str} | {planner_str}"
            )
        except:
            pass
