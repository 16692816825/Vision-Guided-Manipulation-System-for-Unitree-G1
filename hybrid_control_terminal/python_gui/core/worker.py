# core/worker.py
import multiprocessing
import time
from PyQt5.QtCore import QThread, pyqtSignal
from core.robot_process import RobotProcess
import math
from config import ARM_JOINT_IDS, PRESET_POSES, WAIST_JOINTS 

class ControllerWorker(QThread):
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(dict)
    popup_signal = pyqtSignal(str, str)
    finished_signal = pyqtSignal()
    frame_signal = pyqtSignal(object) 

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.running = True
        
        self.cmd_queue = multiprocessing.Queue()
        self.status_queue = multiprocessing.Queue(maxsize=10) 
        
        self.core_process = RobotProcess(self.cmd_queue, self.status_queue, args)
        
        self.last_status = {
            'app_state': 'IDLE', 'mode': 'WALKING', 'weight': 0.0,
            'p_goal': [0,0,0], 'p_hand': [0,0,0], 'vitals': {}, 'joints': {}
        }

    def start(self):
        super().start()
        self.core_process.start()
        self.log_signal.emit("主控线程已启动，等待核心连接...")

    def stop(self):
        self.running = False
        try: self.cmd_queue.put_nowait(('CMD_QUIT', None))
        except: pass
        
        start_wait = time.time()
        while time.time() - start_wait < 1.0:
            try:
                if not self.status_queue.empty(): self.status_queue.get_nowait()
                else: 
                    if not self.core_process.is_alive(): break
                    time.sleep(0.05)
            except: break

        if self.core_process.is_alive(): self.core_process.join(timeout=1.0)
        if self.core_process.is_alive(): self.core_process.terminate()
        
        super().quit()
        super().wait()

    def run(self):
        self.log_signal.emit("主控线程已启动...")
        while self.running:
            try:
                latest_status = None
                while not self.status_queue.empty():
                    try:
                        data = self.status_queue.get_nowait()
                        if data['type'] == 'STATUS': latest_status = data
                        elif data['type'] == 'LOG': self.log_signal.emit(data['msg'])
                        elif data['type'] == 'ERROR': self.log_signal.emit(f"Error: {data['msg']}")
                    except: break
                
                if latest_status:
                    img = latest_status.pop('image', None)
                    self.status_signal.emit(latest_status)
                    if img is not None:
                        latest_status['image'] = img 
                    self.last_status = latest_status 
                    if img is not None: self.frame_signal.emit(img)
                time.sleep(0.03 if latest_status is None else 0.005)     
            except Exception: time.sleep(0.1)
        self.finished_signal.emit()

    # --- 基础控制指令 ---
    def handle_key_event(self, key, pressed): self.cmd_queue.put(('KEY_UPDATE', {'key': key, 'pressed': pressed}))
    
    def trigger_start_squat(self): self.cmd_queue.put(('CMD_START_SQUAT', None))
    def trigger_start_hanger(self): self.cmd_queue.put(('CMD_START_HANGER', None))
    def trigger_stop_squat(self): self.cmd_queue.put(('CMD_STOP_SQUAT', None))
    def trigger_stop_hanger(self): self.cmd_queue.put(('CMD_STOP_HANGER', None))
    
    def trigger_start(self): self.cmd_queue.put(('CMD_START', None))
    def trigger_stop(self): self.cmd_queue.put(('CMD_STOP', None))
    
    def set_arm_speed_level(self, l): self.cmd_queue.put(('SET_ARM_SPD', l))
    def set_walk_speed_level(self, l): self.cmd_queue.put(('SET_WALK_SPD', l))
    def move_camera(self, action, dx, dy): self.cmd_queue.put(('CAM_CTRL', {'action': action, 'dx': dx, 'dy': dy}))
    
    # --- 示教/IK指令 ---
    def start_teaching(self): self.cmd_queue.put(('CMD_TEACH_START', None))
    def stop_teaching(self, filename): self.cmd_queue.put(('CMD_TEACH_STOP', {'filename': filename}))
    def start_replay(self, filename): self.cmd_queue.put(('CMD_REPLAY_START', {'filename': filename}))
    def exit_teaching_mode(self): self.cmd_queue.put(('CMD_EXIT_TEACH', None))
    
    def send_ik_command(self, x, y, z, r, p, roll, duration=2.0):
        self.cmd_queue.put(('CMD_IK_MOVE', {
            'x': x, 'y': y, 'z': z, 'roll': roll, 'duration': duration
        }))
    def send_ik_preview(self, x, y, z, roll):
        self.cmd_queue.put(('CMD_IK_PREVIEW', {'x': x, 'y': y, 'z': z, 'roll': roll}))

    def send_ik_tracking(self, x, y, z, roll=0.0):
        self.cmd_queue.put(('CMD_IK_TRACK', {'x': x, 'y': y, 'z': z, 'roll': roll}))

    def send_traj_command(self, x, y, z, roll, duration=2.0):
        self.cmd_queue.put(('CMD_TRAJ_MOVE', {
            'x': x, 'y': y, 'z': z, 'roll': roll, 'duration': duration
        }))

    # --- 导航 ---
    def start_mapping_mode(self): self.cmd_queue.put(('CMD_LAUNCH_MAPPING', None))
    def start_nav_mode(self, map_filename): self.cmd_queue.put(('CMD_LAUNCH_NAV', {'filename': map_filename}))
    def stop_nav_system(self): self.cmd_queue.put(('CMD_STOP_ROS', None))
    def save_ros_map(self, filename): self.cmd_queue.put(('CMD_SAVE_MAP', {'filename': filename}))
    def send_nav_goal(self, x, y, yaw): self.cmd_queue.put(('CMD_PUB_GOAL', {'x': x, 'y': y, 'yaw': yaw}))
    def send_initial_pose(self, x, y, yaw): self.cmd_queue.put(('CMD_SET_POSE', {'x': x, 'y': y, 'yaw': yaw}))
    
    def toggle_view_mode(self): self.cmd_queue.put(('CMD_TOGGLE_VIEW', None))
    def send_vision_click(self, u, v): self.cmd_queue.put(('CMD_VISION_CLICK', {'u': u, 'v': v}))
    def trigger_force_running(self): self.cmd_queue.put(('CMD_FORCE_RUNNING', None))
    
    def send_joint_sequence(self, waypoints):
        self.cmd_queue.put(('CMD_EXECUTE_JOINT_SEQUENCE', waypoints))

    def _build_preset_sequence(self, action_names):
        is_right = (ARM_JOINT_IDS[0] > 20)
        if is_right: base_id = 22; roll_sign = -1.0
        else: base_id = 15; roll_sign = 1.0

        sequence_cmd = []
        for step_name in action_names:
            pose_cfg = PRESET_POSES[step_name]
            joints_map = {
                base_id + 0: math.radians(pose_cfg["pitch"]),
                base_id + 1: math.radians(pose_cfg["roll"] * roll_sign),
                base_id + 2: math.radians(pose_cfg["yaw"]),
                base_id + 3: math.radians(pose_cfg["elbow"]),
                base_id + 4: math.radians(pose_cfg["wrist"])
            }
            sequence_cmd.append({'joints': joints_map, 'duration': pose_cfg["duration"]})
        return sequence_cmd

    def trigger_arm_init_sequence(self):
        """初始化：Retract -> Lift -> Ready, 结束后保持"""
        seq = self._build_preset_sequence(["RETRACT", "LIFT", "READY"])
        
        # [修复] 修正指令名称为 CMD_EXECUTE_SEQUENCE
        self.cmd_queue.put(('CMD_EXECUTE_SEQUENCE', {
            'waypoints': seq,
            'post_mode': 'IK_HOLD'
        }))

    # core/worker.py

    def trigger_arm_exit_sequence(self):
        """
        退出/复位逻辑：执行反向收回序列 (LIFT -> RETRACT)
        并且保持在 IK_HOLD 状态，不切回 WALKING
        """
        # 构建反向序列：先抬起(LIFT)避免打到桌子，再收回(RETRACT)
        seq = self._build_preset_sequence(["LIFT", "RETRACT"])
        
        self.cmd_queue.put(('CMD_EXECUTE_SEQUENCE', {
            'waypoints': seq,
            'post_mode': 'IK_HOLD'  # 关键：执行完保持力矩，不泄力
        }))

    def switch_fsm_mode(self, fsm_id):
        self.cmd_queue.put(('CMD_SET_FSM', {'id': fsm_id}))

    def send_task_chain(self, task_data):
        """发送任务链给核心进程"""
        # task_data 格式: {'type': 'CHAIN', 'tasks': [...]}
        self.cmd_queue.put(('CMD_EXEC_TASK_CHAIN', task_data))

    def inject_signal(self, key, value):
        """发送模拟信号指令"""
        self.cmd_queue.put(('CMD_INJECT_SIGNAL', {'key': key, 'value': value}))
