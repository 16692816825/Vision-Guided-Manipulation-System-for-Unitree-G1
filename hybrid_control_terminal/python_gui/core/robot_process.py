# core/robot_process.py
import multiprocessing
import threading
import queue
import time
import traceback
import pathlib
import numpy as np
import math
import os
import sys
import re
from types import SimpleNamespace

from config import *
from utils import load_policy, make_env
from core.bridge import RobotBridge, MockBridge
from core.sim_engine import SimEngine
from core.input_mapper import InputMapper
from core.safety_layer import SafetyLayer
from core.recorder import TrajectoryRecorder
from core.l10_client import L10Client 
from core.trajectory import generate_cartesian_path
import boot
from config import ACT_NONE, ACT_GRASP, ACT_OPEN
from core.trajectory import generate_cartesian_path
from config import JOINTS, WAIST_JOINTS, ARM_JOINT_IDS
from core.ros_manager import RosManager
from core.sequencer import ActionSequencer
from core.signal_server import SignalServer
from ui.widgets_task import TASK_TYPE_MOVE, TASK_TYPE_WAIT # 确保能引用到常量
import json
import gc
try:
    from core.ros_bridge import RosBridgeThread
    ROS_AVAILABLE = True
except ImportError:
    print("[系统] 未检测到 ROS 环境或相关依赖，导航功能将不可用。")
    ROS_AVAILABLE = False

class RobotProcess(multiprocessing.Process):
    def __init__(self, cmd_queue, status_queue, args):
        super().__init__()
        self.cmd_queue = cmd_queue
        self.status_queue = status_queue
        self.args = args
        self.daemon = True
        
        self.ros_manager = None
        self.sequencer = None
        self.l10 = None
        self.ik_solver = None
        
        self.last_target_roll = 0.0  
        self.traj_post_mode = 'IK_HOLD' 
        # [新增] 任务执行上下文

        self.pending_task_action = None # 格式: {'filename': 'wave.json'}
        self.nav_goal_coords = None     # 格式: (x, y) 用于计算剩余距离
        self.nav_goal_yaw = None        # 目标偏航 (rad)
        # [新增] 信号监听服务
        self.signal_server = None
        
        # [新增] 任务链执行状态变量
        self.task_chain = []          # 任务列表
        self.current_task_idx = -1    # 当前执行到第几个
        self.chain_sub_state = 'IDLE' # 子状态: INIT, NAVIGATING, ARRIVED, ACTING, WAITING
        self.task_start_time = 0      # 用于计算超时
        # =========== [修改 1] 新增动作缓存字典 ===========
        self.action_cache = {} 
        # ===============================================

    def log(self, msg):
        try:
            self.status_queue.put_nowait({'type': 'LOG', 'msg': msg})
        except: pass 

    def _fix_yaml_image_path(self, yaml_path):
        """
        [加强版] 强制修复 YAML 文件中的 image 路径
        """
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            new_lines = []
            modified = False
            for line in lines:
                # 匹配 image: xxxx
                if line.strip().startswith('image:'):
                    # 获取原有的图片文件名 (不管它路径写的多离谱，只取文件名)
                    parts = line.strip().split(':')
                    if len(parts) >= 2:
                        old_val = parts[1].strip()
                        # 提取纯文件名 (e.g. "5566.pgm")
                        filename = os.path.basename(old_val)
                        
                        # 构造新行: image: 5566.pgm
                        new_line = f"image: {filename}\n"
                        
                        if new_line != line:
                            new_lines.append(new_line)
                            self.log(f"[AutoFix] YAML 路径已修正: {old_val} -> {filename}")
                            modified = True
                            continue
                new_lines.append(line)
            
            if modified:
                with open(yaml_path, 'w', encoding='utf-8') as f:
                    f.writelines(new_lines)
                # 强制刷新缓冲区
                os.sync()
                
        except Exception as e:
            self.log(f"[WARN] YAML 修复失败: {e}")

    def run(self):
        # =========== [深度优化 3] 提升进程优先级 ===========
        try:
            os.nice(-10) # 负值表示高优先级 (范围 -20 到 19)
            print("[System] 核心进程优先级已提升 (Nice -10)")
        except Exception as e:
            print(f"[System] 提权失败 (非 Root?): {e}")
        # =================================================
        self.log("核心进程启动...")
        
        # 1. 初始化 ROS
        self.ros_manager = RosManager(self.log)
        self.ros_manager.start_roscore() 

        ros_bridge = None
        if ROS_AVAILABLE:
            try:
                ros_bridge = RosBridgeThread()
                ros_bridge.start()
                self.log("ROS1 数据监听桥接已就绪")
            except Exception as e:
                self.log(f"ROS 桥接失败: {e}")
                ros_bridge = None
        # [新增] 启动 UDP 信号监听 (端口 12345)
        self.signal_server = SignalServer(port=12345)
        self.signal_server.start()
        self.log("UDP 信号监听服务已启动 (Port: 12345)")
        try:
            # 2. 初始化环境
            # model_path = pathlib.Path(self.args.model)
            # policy = None
            # if model_path.exists():
            #     policy = load_policy(model_path)
            # else:
            #     self.log(f"警告: 策略文件 {model_path} 不存在，RL功能受限")

            env = make_env(render=False, right_arm=self.args.right_arm)
            speed_obj = SimpleNamespace(dt=max(0.005, self.args.rate), mult=ARM_SPEED_LEVELS[1])
            sim = SimEngine(env, None, speed_obj)

            if not self.args.sim_only:
                self.l10 = L10Client(robot_ip="192.168.123.164") 
                self.log("L10 灵巧手客户端已连接")
            
            if self.args.sim_only:
                bridge = MockBridge()
                loco = None
            else:
                bridge = RobotBridge(self.args.iface, self.args.domain)
                loco = None
                if bridge.ok:
                    try:
                        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
                        loco = LocoClient()
                        loco.SetTimeout(1.0)
                        loco.Init()
                    except Exception as e:
                        self.log(f"Loco Init Err: {e}")
                else:
                    self.log("Bridge 未就绪，跳过 LocoClient 初始化")

            input_mapper = InputMapper()
            safety = SafetyLayer()
            recorder = TrajectoryRecorder()
            ik_solver = None # 直接设为 None，防止后续调用报错
            self.sequencer = ActionSequencer(loco, sim, bridge, self.status_queue, self.log, self.args.sim_only)

        except Exception as e:
            self.status_queue.put({'type': 'ERROR', 'msg': str(e)})
            traceback.print_exc()
            return
        # =========== [优化] 后台采样线程，避免主循环被阻塞 ===========
        cached_fsm_id = -1
        last_full_states = {}
        last_joint_q = {}
        last_vitals = {}
        state_poll_stop = threading.Event()
        fsm_poll_stop = threading.Event()

        def _state_poll_loop():
            nonlocal last_full_states, last_joint_q, last_vitals
            full_skip = 0
            while not state_poll_stop.is_set():
                if bridge.ok:
                    q_states = bridge.get_joint_states(minimal=True)
                    if q_states:
                        last_joint_q = q_states
                    if full_skip <= 0:
                        full_states = bridge.get_joint_states()
                        if full_states:
                            last_full_states = full_states
                        full_skip = 4
                    else:
                        full_skip -= 1
                    vitals = bridge.get_robot_vitals()
                    if vitals:
                        last_vitals = vitals
                state_poll_stop.wait(0.01) # 100Hz

        state_thread = None
        if bridge.ok:
            state_thread = threading.Thread(target=_state_poll_loop, daemon=True)
            state_thread.start()

        def _fsm_poll_loop():
            nonlocal cached_fsm_id
            while not fsm_poll_stop.is_set():
                if loco and not self.args.sim_only:
                    fid = boot.get_fsm_id(loco)
                    if fid is not None:
                        cached_fsm_id = fid
                fsm_poll_stop.wait(1.0)

        fsm_thread = None
        if loco and not self.args.sim_only:
            fsm_thread = threading.Thread(target=_fsm_poll_loop, daemon=True)
            fsm_thread.start()
        # ==========================================================
        # =========== [修改 2] 启动时预加载所有 JSON 动作文件 ===========
        self.log(">>> [System] 正在预加载动作文件到内存...")
        try:
            import glob # 确保导入 glob
            # 遍历 data_logger 目录
            for json_file in DATA_LOG_DIR.glob("*.json"):
                fname = json_file.name # 例如 "wave.json"
                
                # +++++++++++ [新增修正] 跳过非动作文件 +++++++++++
                if fname == "tasks.json":
                    continue
                # ++++++++++++++++++++++++++++++++++++++++++++++

                # 复用 Recorder 的静态方法进行解析
                traj_data = TrajectoryRecorder.load_trajectory(fname)
                if traj_data:
                    self.action_cache[fname] = traj_data
                
                # +++++++++++ [新增调试] 如果解析失败打印文件名 +++++++++++
                else:
                    print(f"[Warn] 预加载跳过无效文件: {fname}")
                # +++++++++++++++++++++++++++++++++++++++++++++++++++++

            self.log(f">>> [System] 预加载完成: 已缓存 {len(self.action_cache)} 个动作")
        except Exception as e:
            self.log(f"[WARN] 动作预加载失败: {e}")
        # ==========================================================

        app_state = 'IDLE'
        control_mode = 'WALKING' 
        arm_level = 1; walk_level = 1
        walk_mult = WALK_SPEED_LEVELS[1]
        target_weight = 0.0; current_weight = 0.0
        sim_x, sim_y = 0.0, 0.0
        
        replay_traj = []; replay_index = 0; replay_start_q = {}
        transition_steps = 0; standby_q = {}
        ik_target_q = {}; ik_start_q = {}; ik_progress = 0.0
        ik_steps_total = 100; ik_steps_cnt = 0
        filter_q = {}
        
        traj_buffer = []; traj_index = 0
        traj_segments = []; current_seg_idx = 0; seg_progress = 0

        last_map_sent_time = 0
        last_scan_sent_time = 0 # [新增] 用于控制雷达发送频率
        sent_zero_weight = True 
        loop_rate = 0.01 
        keep_running = True
        
        ik_target_pos_cache = {'x': 0.4, 'y': 0.0, 'z': 0.8}
        view_source = 'THIRD_PERSON' 
        vision_cursor = None         
        last_vision_pos = None 
        
        loop_cnt = 0 

        loco_last_cmd = (0.0, 0.0, 0.0)
        loco_last_send = 0.0
        loco_send_interval = 0.1
        loco_keepalive_interval = 0.5
        loco_cmd_eps = 0.01
        yaw_tolerance = math.radians(10.0)
        loco_cmd_lock = threading.Lock()
        loco_cmd_latest = None
        loco_ctrl_queue = queue.Queue()
        loco_cmd_event = threading.Event()
        loco_cmd_stop = threading.Event()

        def _loco_worker():
            nonlocal loco_cmd_latest
            while not loco_cmd_stop.is_set():
                try:
                    ctrl_cmd = loco_ctrl_queue.get(timeout=0.1)
                except queue.Empty:
                    ctrl_cmd = None

                if ctrl_cmd is not None:
                    cmd_name, cmd_args = ctrl_cmd
                    try:
                        if cmd_name == "SetFsmId":
                            loco.SetFsmId(*cmd_args)
                            send_label = "运控.SetFsmId"
                        elif cmd_name == "Start":
                            loco.Start()
                            send_label = "运控.Start"
                        else:
                            send_label = f"运控.{cmd_name}"
                    except Exception as e:
                        self.log(f"[WARN] 运控指令错误: {e}")
                        continue
                    continue

                loco_cmd_event.wait(0.1)
                loco_cmd_event.clear()
                with loco_cmd_lock:
                    cmd = loco_cmd_latest
                if cmd is None:
                    continue
                vx, vy, rot = cmd
                try:
                    if abs(vx) > 0 or abs(vy) > 0 or abs(rot) > 0:
                        loco.Move(vx, vy, rot)
                        send_label = "运控移动"
                    else:
                        loco.StopMove()
                        send_label = "运控停止"
                except Exception as e:
                    self.log(f"[WARN] 运控指令错误: {e}")
                    continue

        loco_thread = None
        if loco and not self.args.sim_only:
            loco_thread = threading.Thread(target=_loco_worker, daemon=True)
            loco_thread.start()

        def _enqueue_loco_ctrl(cmd_name, *cmd_args):
            if loco is None or self.args.sim_only:
                return
            loco_ctrl_queue.put((cmd_name, cmd_args))

        def _maybe_send_loco(vx, vy, rot, force=False):
            nonlocal loco_last_cmd, loco_last_send, loco_cmd_latest
            if loco is None or self.args.sim_only:
                return
            now = time.time()
            cmd_changed = (
                abs(vx - loco_last_cmd[0]) >= loco_cmd_eps or
                abs(vy - loco_last_cmd[1]) >= loco_cmd_eps or
                abs(rot - loco_last_cmd[2]) >= loco_cmd_eps
            )
            if not force:
                if cmd_changed and (now - loco_last_send) < loco_send_interval:
                    return
                if not cmd_changed and (now - loco_last_send) < loco_keepalive_interval:
                    return
            with loco_cmd_lock:
                loco_cmd_latest = (vx, vy, rot)
            loco_cmd_event.set()
            loco_last_cmd = (vx, vy, rot)
            loco_last_send = now

        def _angle_diff(a, b):
            diff = a - b
            while diff > math.pi:
                diff -= 2.0 * math.pi
            while diff < -math.pi:
                diff += 2.0 * math.pi
            return diff
        # =========== [修复] 必须在这里初始化变量 ===========
        self.last_wait_log_time = 0.0
        # ================================================
        while keep_running:
            # [探针 T0] 循环开始
            loop_start = time.time()
            # =========== [修复] 初始化所有探针，防止报错 ===========
            t0 = t1 = t2 = t3 = t4 = t5 = loop_start
            # ===================================================
            t0 = loop_start  # 记录起点

            pending_ik_track_data = None 

            while not self.cmd_queue.empty():
                try:
                    cmd_type, data = self.cmd_queue.get_nowait()

                    if cmd_type == 'CMD_QUIT':
                        keep_running = False; break
                    
                    elif cmd_type == 'CMD_IK_TRACK':
                        self.log("IK 已禁用，忽略 CMD_IK_TRACK")
                        continue 

                    elif cmd_type == 'KEY_UPDATE':
                        input_mapper.update_keys(data['key'], data['pressed'])

                    elif cmd_type == 'CMD_FORCE_RUNNING':
                        app_state = 'RUNNING'
                        self.log(">>> 状态同步: 机器人已在线 (FSM 200/500/801)")

                    # === 启停序列 ===
                    elif cmd_type == 'CMD_START_SQUAT':
                        if app_state == 'IDLE':
                            app_state = 'STARTING'; self.sequencer.start_squat(); app_state = 'RUNNING'
                    elif cmd_type == 'CMD_START_HANGER':
                        if app_state == 'IDLE':
                            app_state = 'STARTING'; self.sequencer.start_hanger(); app_state = 'RUNNING'
                            
                    # =========== [修改] 修复蹲姿关机逻辑 ===========
                    elif cmd_type == 'CMD_STOP_SQUAT':
                        if app_state in ['RUNNING', 'STARTING']:
                            # 1. 立即标记状态，停止 RL/IK 循环
                            app_state = 'STOPPING'
                            
                            # 2. 强制切回 WALKING 模式，防止逻辑残留
                            control_mode = 'WALKING' 
                            
                            # 3. 执行安全的关机序列
                            self.sequencer.stop_squat(current_weight)
                            
                            # 4. 状态复位
                            current_weight = 0.0
                            app_state = 'IDLE'
                    # =============================================
                            
                    elif cmd_type == 'CMD_STOP_HANGER':
                        if app_state in ['RUNNING', 'STARTING']:
                            app_state = 'STOPPING'; self.sequencer.stop_hanger(current_weight); current_weight = 0.0; app_state = 'IDLE'

                    # === ROS 管理 ===
                    elif cmd_type == 'CMD_LAUNCH_MAPPING':
                        if self.ros_manager.launch_ros("mapping.launch", package="fastlio"):
                            control_mode = 'NAVIGATING'; target_weight = 0.0; self.log(">>> [HongTu] 建图模式启动")
                    
                    elif cmd_type == 'CMD_LAUNCH_NAV':
                        map_name = data.get('filename')
                        if map_name:
                            current_root = os.getcwd()
                            maps_dir = os.path.join(current_root, "maps")
                            
                            pcd_path = os.path.join(maps_dir, f"{map_name}.pcd")
                            yaml_path = os.path.join(maps_dir, f"{map_name}.yaml")
                            
                            # [1] 强制修复 YAML
                            self._fix_yaml_image_path(yaml_path)

                            # [2] 参数名修正
                            # 之前 PCDReader 读到了 yaml，说明 'map_file' 这个变量被 3D 节点占用了。
                            # 2D map_server 可能需要 'yaml_file' 或者根本不认 'map_file'
                            # 策略：把 PCD 路径赋给 'map_file' (喂饱 3D 节点)
                            # 把 YAML 路径赋给 'yaml_file' 和 'map_file_2d' (希望能命中 2D 节点)
                            args_list = [
                                f"map_file:={pcd_path}",      # 3D 节点似乎在用这个名字
                                f"pcd_file:={pcd_path}",      # 备用
                                f"yaml_file:={yaml_path}",    # 2D 节点常用名
                                f"map_path:={yaml_path}"      # 备用
                            ]
                            
                            if self.ros_manager.launch_ros("navigation.launch", package="fastlio", args_list=args_list):
                                control_mode = 'NAVIGATING'; target_weight = 0.0; last_map_sent_time = 0
                                self.log(f">>> [HongTu] 导航引擎启动成功！")
                        else: self.log("错误: 未指定地图文件")

                    elif cmd_type == 'CMD_STOP_ROS':
                        self.ros_manager.stop_ros(); control_mode = 'WALKING'; self.status_queue.put({'type': 'NAV_STOPPED'})
                        self.log(">>> 导航系统已停止")
                    elif cmd_type == 'CMD_SAVE_MAP':
                        fname = data.get('filename', 'default_map')
                        maps_dir = os.path.abspath("maps"); os.makedirs(maps_dir, exist_ok=True)
                        full_path_base = os.path.join(maps_dir, fname)
                        if self.ros_manager.save_map_command(full_path_base):
                            self.log("正在自动退出建图模式..."); time.sleep(1.0)
                            self.ros_manager.stop_ros()
                            
                            # =========== [修复] 必须重置控制模式 ===========
                            control_mode = 'WALKING' 
                            target_weight = 0.0
                            # =============================================
                            
                            self.status_queue.put({'type': 'NAV_SAVED_AND_STOPPED'})
                    # [新增] 组合任务指令: 导航 -> 到达 -> 动作
                    elif cmd_type == 'CMD_START_TASK':
                        # 1. 解析数据
                        target_pose = data['pose']   # {'x':..., 'y':..., 'yaw':...}
                        action_file = data['action'] # "wave.json"
                        
                        # 2. 发布导航目标
                        if ros_bridge:
                            ros_bridge.publish_goal(target_pose['x'], target_pose['y'], target_pose['yaw'])
                            
                            # 3. 设置状态
                            app_state = 'RUNNING'
                            control_mode = 'NAVIGATING'
                            target_weight = 0.0
                            
                            # 4. 记录任务上下文 (用于后续的到达判定)
                            self.nav_goal_coords = (target_pose['x'], target_pose['y'])
                            self.nav_goal_yaw = target_pose.get('yaw', 0.0)
                            self.pending_task_action = action_file
                            
                            self.log(f"🚀 任务链启动: 前往目标 -> 待执行 [{action_file}]")
                        else:
                            self.log("错误: ROS 未连接，无法导航")
                    # ... (原有的 CMD_START_TASK 处理块) ...
                    
                    # === [新增] 任务链指令 ===
                    elif cmd_type == 'CMD_EXEC_TASK_CHAIN':
                        data_pack = data 
                        self.task_chain = data_pack.get('tasks', [])
                        
                        if not self.task_chain:
                            self.log("❌ 任务链为空")
                        else:
                            self.log(f"🚀 启动任务链 (共 {len(self.task_chain)} 步)")
                            
                            # === [新增] 启动前清空所有旧信号 ===
                            if self.signal_server:
                                self.signal_server.clear()
                                self.log("🧹 信号缓冲区已清空")
                            # =================================
                            
                            self.current_task_idx = 0
                            self.chain_sub_state = 'INIT'
                            control_mode = 'TASK_CHAIN_EXECUTION' 
                            app_state = 'RUNNING'
                            target_weight = 0.0

                    elif cmd_type == 'CMD_INJECT_SIGNAL':
                        key = data.get('key'); val = data.get('value')
                        if self.signal_server:
                            self.signal_server.inject_signal(key, val)
                            self.log(f"💉 注入信号: {key}={val}")
                    # ========================
                    # === 常规指令 ===
                    elif cmd_type == 'SET_ARM_SPD':
                        arm_level = data; speed_obj.mult = ARM_SPEED_LEVELS[arm_level]
                    elif cmd_type == 'SET_WALK_SPD':
                        walk_level = data; walk_mult = WALK_SPEED_LEVELS[walk_level]
                    elif cmd_type == 'CAM_CTRL':
                        sim.move_camera(data['action'], data['dx'], data['dy'])
                    elif cmd_type == 'CMD_VISION_CLICK':
                        vision_cursor = (data['u'], data['v']); self.log(f"视觉目标锁定: ({data['u']}, {data['v']})")
                    elif cmd_type == 'CMD_TOGGLE_VIEW':
                        if view_source == 'THIRD_PERSON': view_source = 'ROBOT_HEAD'; vision_cursor = None; self.log(">>> 切换至: 机器人第一人称视角")
                        else: view_source = 'THIRD_PERSON'; self.log(">>> 切换至: 仿真上帝视角")
                    elif cmd_type == 'CMD_PUB_GOAL':
                        if ros_bridge: 
                            ros_bridge.publish_goal(data['x'], data['y'], data['yaw'])
                            # [修复] 强制切回导航模式，否则如果在 WALKING 状态下发目标，机器人不会动
                            if app_state == 'RUNNING':
                                control_mode = 'NAVIGATING'
                                target_weight = 0.0 # 确保权重归零（纯运控）
                            self.log(f"发布导航目标: ({data['x']:.2f}, {data['y']:.2f}) -> Mode: NAVIGATING")
                    elif cmd_type == 'CMD_SET_POSE':
                        if ros_bridge: ros_bridge.publish_initial_pose(data['x'], data['y'], data['yaw']); self.log(f"设置初始位姿: ({data['x']:.2f}, {data['y']:.2f})")
                    
                    # === 示教/回放 ===
                    elif cmd_type == 'CMD_TEACH_START':
                        if app_state != 'RUNNING':
                            self.log("示教启动失败: 机器人未运行")
                        elif control_mode not in ['WALKING', 'TEACH_STANDBY']:
                            self.log(f"示教启动被拒绝: 当前模式 {control_mode}")
                        else:
                            control_mode = 'TEACHING'
                            target_weight = 1.0
                            recorder.start()
                            self.log(">>> 示教开始")
                    elif cmd_type == 'CMD_TEACH_STOP':
                        if control_mode == 'TEACHING':
                            success = recorder.stop_and_save(data['filename'])
                            if bridge.ok:
                                src = last_joint_q if last_joint_q else last_full_states
                                if not src:
                                    src = bridge.get_joint_states()
                                standby_q = {k: v['q'] for k, v in src.items() if k in ARM_JOINT_IDS}
                            else: standby_q = sim.get_motor_qpos()
                            control_mode = 'TEACH_STANDBY'; target_weight = 1.0; self.log(f"示教保存: {success}")
                    elif cmd_type == 'CMD_REPLAY_START':
                        if app_state != 'RUNNING':
                            self.log("回放启动失败: 机器人未运行")
                        elif control_mode not in ['WALKING', 'TEACH_STANDBY']:
                            self.log(f"回放启动被拒绝: 当前模式 {control_mode}")
                        else:
                            # =========== [修改 3] 优先读缓存 ===========
                            fname = data['filename']
                            # 补全后缀以匹配缓存 Key
                            key_name = fname if fname.endswith(".json") else fname + ".json"
                            
                            # 1. 尝试缓存
                            traj = self.action_cache.get(key_name)
                            
                            # 2. 缓存没命中 (比如刚录的)，则读硬盘并更新缓存
                            if not traj:
                                traj = recorder.load_trajectory(fname)
                                if traj: self.action_cache[key_name] = traj
                            # ==========================================

                            if traj:
                                self.traj_post_mode = 'IK_HOLD'
                                gc.disable()
                                replay_traj = traj; control_mode = 'TRANSITION'; target_weight = 1.0; transition_steps = 100 
                                if bridge.ok:
                                    src = last_joint_q if last_joint_q else last_full_states
                                    if not src:
                                        src = bridge.get_joint_states()
                                    replay_start_q = {k: v['q'] for k, v in src.items() if k in ARM_JOINT_IDS}
                                else: replay_start_q = sim.get_motor_qpos()
                                self.log(f"开始回放")
                    
                    # 柔性退出
                    elif cmd_type == 'CMD_EXIT_TEACH':
                        if control_mode in ['TEACH_STANDBY', 'TEACHING', 'REPLAYING', 'TRANSITION', 'IK_MOVING', 'IK_HOLD', 'IK_TRACKING', 'ARM_CONTROL', 'TRAJECTORY_FOLLOWING', 'SEQUENCE_EXECUTION']:
                            control_mode = 'WALKING'; target_weight = 0.0
                            for w_id in WAIST_JOINTS:
                                if w_id in sim._joint_map: sim.env.data.qpos[sim._joint_map[w_id]] = 0.0
                            if loco and not self.args.sim_only: _enqueue_loco_ctrl("Start")
                            self.log(">>> 柔性复位: 已切回行走模式 (Weight=0)")

                    # === IK 控制 ===
                    elif cmd_type == 'CMD_IK_MOVE':
                        self.traj_post_mode = 'IK_HOLD'
                        self.log("IK 已禁用，忽略 CMD_IK_MOVE")

                    elif cmd_type == 'CMD_IK_PREVIEW':
                        self.log("IK 已禁用，忽略 CMD_IK_PREVIEW")
                    
                    elif cmd_type == 'CMD_TRAJ_MOVE':
                        self.traj_post_mode = 'IK_HOLD'
                        self.log("IK 已禁用，忽略 CMD_TRAJ_MOVE")

                    elif cmd_type == 'CMD_HAND_ACT':
                        if self.l10:
                            action = data.get('action')
                            if action == 'GRASP':
                                self.l10.grasp(data.get('diameter', 30)); self.log(f"执行抓取: 直径 {data.get('diameter')}mm")
                            elif action == 'OPEN':
                                self.l10.open(); self.log("执行松开")

                    elif cmd_type == 'CMD_EXECUTE_SEQUENCE':
                        # 1. 解析数据
                        waypoints = []
                        if isinstance(data, dict) and 'waypoints' in data:
                            waypoints = data['waypoints']
                            self.traj_post_mode = data.get('post_mode', 'IK_HOLD')
                        else:
                            waypoints = data
                            self.traj_post_mode = 'IK_HOLD'

                        if app_state == 'RUNNING' and waypoints:
                            # [修复] 判断序列类型：是关节角度(joints) 还是 笛卡尔坐标(x,y,z)?
                            is_joint_seq = (len(waypoints) > 0 and 'joints' in waypoints[0])

                            if is_joint_seq:
                                # ===========================================
                                # 分支 A: 执行关节角度序列 (如 SAFE INIT)
                                # ===========================================
                                self.log(f"接收关节序列任务 ({len(waypoints)}段)...")
                                
                                # 获取当前关节角度作为起点
                                current_q = {}
                                if bridge.ok:
                                    src = last_joint_q if last_joint_q else last_full_states
                                    if not src:
                                        src = bridge.get_joint_states()
                                    current_q = {k: v['q'] for k, v in src.items() if k in ARM_JOINT_IDS}
                                else:
                                    current_q = {k: v for k,v in sim.get_motor_qpos().items() if k in ARM_JOINT_IDS}
                                
                                temp_segments = []
                                
                                # 遍历每一个关键帧
                                for wp in waypoints:
                                    target_map = wp['joints'] # 目标关节包
                                    duration = wp['duration']
                                    steps = int(max(duration, 0.5) / loop_rate) # 计算帧数
                                    
                                    frames = []
                                    # 生成插值帧
                                    for s in range(steps):
                                        progress = s / steps
                                        # 使用平滑曲线 (Smootherstep)
                                        alpha = progress * progress * (3 - 2 * progress)
                                        
                                        frame_q = {}
                                        for jid in ARM_JOINT_IDS:
                                            start_rad = current_q.get(jid, 0.0)
                                            # 兼容 int key 和 str key
                                            target_rad = target_map.get(jid)
                                            if target_rad is None: target_rad = target_map.get(str(jid), start_rad)
                                            
                                            frame_q[jid] = start_rad + (target_rad - start_rad) * alpha
                                        frames.append(frame_q)
                                    
                                    temp_segments.append({'frames': frames, 'action': 0})
                                    
                                    # 更新起点为当前目标的终点，以便计算下一段
                                    for jid in ARM_JOINT_IDS:
                                        val = target_map.get(jid)
                                        if val is None: val = target_map.get(str(jid), current_q.get(jid, 0.0))
                                        current_q[jid] = val

                                # 提交执行
                                traj_segments = temp_segments
                                current_seg_idx = 0; seg_progress = 0
                                control_mode = 'SEQUENCE_EXECUTION'; target_weight = 1.0
                                
                                # 关节模式下，不需要更新 last_target_roll 或可视化轨迹
                                self.log("关节序列规划完成，开始执行。")

                            else:
                                # ===========================================
                                # 分支 B: 执行笛卡尔 IK 序列 (原逻辑)
                                # ===========================================
                                self.log("IK 已禁用，仅支持关节序列执行")

                    elif cmd_type == 'CMD_VIS_CLEAR':
                        if hasattr(sim, 'set_trajectory_path'): sim.set_trajectory_path([]) 
                        self.log("可视化轨迹已清除")
                    
                    elif cmd_type == 'CMD_SET_FSM':
                        target_id = data['id']
                        if loco and not self.args.sim_only:
                            self.log(f"正在切换运控模式 -> FSM {target_id} ...")
                            _enqueue_loco_ctrl("SetFsmId", target_id)
                            cached_fsm_id = target_id
                        else:
                            self.log(f"[Sim] 模拟切换 FSM -> {target_id}")
                            cached_fsm_id = target_id

                except Exception as e:
                    self.log(f"CMD Error: {e}"); traceback.print_exc()
            
            # [探针 T1] 指令处理结束
            t1 = time.time()

            if pending_ik_track_data is not None:
                self.log("IK 已禁用，忽略 CMD_IK_TRACK")

            if bridge.ok:
                should_sync_arm = True
                # 判断是否需要仿真跟随真实手臂（在IK控制时不需要，因为仿真引导真机）
                if control_mode in ['IK_TRACKING', 'IK_MOVING', 'IK_HOLD', 'TRAJECTORY_FOLLOWING', 'ARM_CONTROL']: 
                    should_sync_arm = False
                elif app_state == 'RUNNING' and target_weight >= 0.99: 
                    should_sync_arm = False
                
                # =========== [修改] 极致优化同步逻辑 ===========
                # 1. 状态过滤：待机(IDLE)时不消耗算力去同步模型
                # 2. 全局降频：所有状态下仅每 5 帧 (20Hz) 读取一次状态并同步
                # 3. 状态缓存：避免每帧拉取 DDS 状态导致阻塞
                
                is_running_state = (app_state != 'IDLE')
                need_state_update = is_running_state and (loop_cnt % 5 == 0)
                if control_mode == 'TEACHING':
                    need_state_update = True
                
                if need_state_update and last_joint_q:
                    real_q_map = {k: v['q'] for k, v in last_joint_q.items()}
                    if is_running_state:
                        sim.sync_with_real(real_q_map, last_vitals.get('quat'), update_arms=should_sync_arm, root_pos_offset=(sim_x, sim_y))
                # =============================================
            # =========== [建议补充] 物理仿真结束时间点 ===========
            t2 = time.time() 
            # =================================================

            # [定位到这里] if app_state == 'RUNNING': 内部
            if app_state == 'RUNNING':
                kp_val = IK_KP if 'IK_KP' in globals() else 40.0 
                kd_val = IK_KD if 'IK_KD' in globals() else 2.0
                cmd_q = {}
                
                # ... (保留原有的权重平滑代码 diff = target_weight - current_weight ...) ...
                diff = target_weight - current_weight
                if abs(diff) > 0.005: current_weight += diff * 0.05
                else: current_weight = target_weight

                # ... (保留原有的模式切换检查 new_mode = input_mapper ...) ...
                new_mode = input_mapper.check_mode_toggle(control_mode)
                if new_mode:
                    control_mode = new_mode
                    if control_mode == 'WALKING': target_weight = 0.0; self.log(">>> 切换至: 行走模式")
                    elif control_mode == 'ARM_CONTROL': target_weight = 1.0; self.log(">>> 切换至: 手臂控制模式 (RL)")
                
                # ==============================================================
                # [核心修改区域 START] 键盘优先逻辑
                # ==============================================================
                
                # 1. 首先获取键盘输入 (无论当前是什么模式)
                vx_key, vy_key, rot_key, is_key_moving = input_mapper.get_walking_command(walk_mult)

                # 2. 优先级判断: 只要有键盘输入，强制接管
                if is_key_moving:
                    # 如果之前是在导航中，现在被打断了
                    if control_mode == 'NAVIGATING':
                        self.log(">>> [中断] 检测到键盘操作，已切回手动模式 (WALKING)")
                        control_mode = 'WALKING'
                        target_weight = 0.0
                        # 此时无需通知 ROS 停止，只需本地不再执行 ROS 速度即可

                    # 执行键盘指令
                    sim_x += vx_key * 0.02; sim_y += vy_key * 0.02
                    _maybe_send_loco(vx_key, vy_key, rot_key)
                    
                    # 保持 RL 更新以维持站立姿态
                    # if sim.policy: sim.update_rl()
                    cmd_q = sim.get_motor_qpos()

                # 3. 如果没按键，且处于导航模式 -> 听 ROS 的
                elif control_mode == 'NAVIGATING' and ros_bridge:
                    if cached_fsm_id != 500:
                        if loop_cnt % 100 == 0:
                            self.log(f"[WARN] 导航中！当前 FSM 为 {cached_fsm_id}，机器人可能不会移动！请切换至 500。")
                    
                    nav_vel = ros_bridge.get_cmd_vel()
                    vx, vy, rot = nav_vel
                    # 增加一些限幅和死区保护
                    vx = np.clip(vx, -0.7, 2.0); vy = np.clip(vy, -0.6, 0.6); rot = np.clip(rot, -0.5, 0.5)
                    if abs(vx) < 0.02: vx = 0
                    if abs(vy) < 0.02: vy = 0
                    if abs(rot) < 0.05: rot = 0
                    
                    # 仿真位姿更新
                    sim_yaw = sim.env.data.qpos[12] if hasattr(sim.env.data, 'qpos') else 0
                    c, s = np.cos(sim_yaw), np.sin(sim_yaw)
                    sim_x += (vx * c - vy * s) * 0.02; sim_y += (vx * s + vy * c) * 0.02
                    
                    if loco and not self.args.sim_only:
                        _maybe_send_loco(vx, vy, rot)
                    # ====================================================
                    # [新增] 到达检测与动作触发逻辑
                    # ====================================================
                    if self.pending_task_action is not None and self.nav_goal_coords is not None:
                        curr_pose = ros_bridge.robot_pose # [x, y, yaw]
                        
                        dx = self.nav_goal_coords[0] - curr_pose[0]
                        dy = self.nav_goal_coords[1] - curr_pose[1]
                        dist = math.sqrt(dx*dx + dy*dy)
                        yaw_err = 0.0
                        yaw_ok = True
                        if self.nav_goal_yaw is not None:
                            yaw_err = _angle_diff(self.nav_goal_yaw, curr_pose[2])
                            yaw_ok = abs(yaw_err) < yaw_tolerance
                        
                        cmd_speed = math.sqrt(nav_vel[0]**2 + nav_vel[1]**2 + nav_vel[2]**2)
                        
                        # 判定到达 (距离 < 0.20m 且 速度 < 0.1)
                        # 如果你是在原地测试，建议把 dist 阈值改大一点，比如 0.3
                        if dist < 0.20 and cmd_speed < 0.1 and yaw_ok:
                            self.log(f"✅ 到达目标点 (误差 {dist:.2f}m)，开始执行动作...")
                            
                            _maybe_send_loco(0.0, 0.0, 0.0, force=True)
                            
                            action_filename = self.pending_task_action
                            # =========== [修改 4] 优先读缓存 ===========
                            key_name = action_filename
                            if not key_name.endswith(".json"): key_name += ".json"

                            traj = self.action_cache.get(key_name)
                            if not traj:
                                traj = recorder.load_trajectory(action_filename)
                            # ==========================================
                            
                            if traj:
                                gc.disable()
                                replay_traj = traj
                                control_mode = 'TRANSITION' 
                                target_weight = 1.0         
                                transition_steps = 100
                                
                                if bridge.ok:
                                    src = last_joint_q if last_joint_q else last_full_states
                                    if not src:
                                        src = bridge.get_joint_states()
                                    replay_start_q = {k: v['q'] for k, v in src.items() if k in ARM_JOINT_IDS}
                                else:
                                    replay_start_q = sim.get_motor_qpos()
                                    
                                self.log(f"🎬 动作回放启动: {action_filename}")
                            else:
                                self.log(f"❌ 动作文件加载失败: {action_filename}")
                                control_mode = 'WALKING' 
                            
                            self.pending_task_action = None
                            self.nav_goal_coords = None
                            self.nav_goal_yaw = None
                    # if sim.policy: sim.update_rl()
                    cmd_q = sim.get_motor_qpos()
                
                # 4. 如果没按键，且处于行走模式 -> 停车
                elif control_mode == 'WALKING':
                    _maybe_send_loco(0.0, 0.0, 0.0)
                    # if sim.policy: sim.update_rl()
                    cmd_q = sim.get_motor_qpos()

                # ==============================================================
                # [核心修改区域 END] 
                # ==============================================================

                elif control_mode == 'ARM_CONTROL':
                    dx, dy, dz = input_mapper.get_arm_deltas()
                    if dx != 0 or dy != 0 or dz != 0:
                        curr_local = sim.get_goal_local()
                        new_local = curr_local + np.array([dx, dy, dz])
                        sim.set_goal_local(new_local[0], new_local[1], new_local[2])
                    # if sim.policy: sim.update_rl()
                    cmd_q = sim.get_motor_qpos()
                    for w_id in WAIST_JOINTS: cmd_q[w_id] = 0.0

                elif control_mode in ['IK_TRACKING', 'IK_MOVING', 'IK_HOLD']:
                    
                    # [优化1] 实时跟随：引入自适应滤波 + 高刚度
                    if control_mode == 'IK_TRACKING':
                        for jid, target_val in ik_target_q.items():
                            prev_val = filter_q.get(jid, target_val)
                            
                            # --- 动态 Alpha 算法 ---
                            # 计算当前指令与目标的差值
                            diff = abs(target_val - prev_val)
                            # 如果差值大(>0.1rad)，alpha接近0.8(极快)
                            # 如果差值小(<0.01rad)，alpha降至0.05(极滑)
                            # 0.03 是基础平滑度，8.0 是由于 diff 通常很小，需要放大倍数
                            dynamic_alpha = 0.03 + min(0.8, diff * 8.0)
                            
                            smooth_val = prev_val * (1.0 - dynamic_alpha) + target_val * dynamic_alpha
                            
                            filter_q[jid] = smooth_val
                            cmd_q[jid] = smooth_val
                            
                            if jid in sim._joint_map: 
                                sim.env.data.qpos[sim._joint_map[jid]] = smooth_val
                        
                        # [硬件参数优化] 提高刚度(Kp)提升响应速度，提高阻尼(Kd)防止急停抖动
                        kp_val = 60.0 
                        kd_val = 4.0 

                    # [优化2] 自动移动：改用 Smootherstep 曲线，起步更快
                    elif control_mode == 'IK_MOVING':
                        ik_steps_cnt += 1
                        progress = min(1.0, ik_steps_cnt / ik_steps_total)
                        
                        # 原来的 Cosine 比较肉: (1.0 - np.cos(np.pi * progress)) * 0.5
                        # 改用 Smootherstep: 起步加速更迅速，尾段减速依然平滑
                        t = progress
                        smooth_p = t * t * (3.0 - 2.0 * t)
                        
                        for jid in ARM_JOINT_IDS:
                            start = ik_start_q.get(jid, 0.0)
                            end = ik_target_q.get(jid, start)
                            val = start + (end - start) * smooth_p
                            cmd_q[jid] = val
                            if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = val
                        
                        # 使用全局配置的较硬参数 (需确保 config.py 中 IK_KP >= 60)
                        kp_val = IK_KP
                        kd_val = IK_KD
                        
                        if progress >= 1.0: control_mode = 'IK_HOLD'

                    # [优化3] 保持模式：保持高刚度
                    elif control_mode == 'IK_HOLD':
                        cmd_q = ik_target_q
                        for jid, rad in cmd_q.items():
                            if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = rad
                        kp_val = IK_KP
                        kd_val = IK_KD
                    
                    # 锁腰逻辑保持不变
                    for w_id in WAIST_JOINTS: cmd_q[w_id] = 0.0
                    # if sim.policy: sim.update_rl()

                elif control_mode == 'TRAJECTORY_FOLLOWING':
                    kp_val = IK_KP; kd_val = IK_KD
                    
                    if traj_index < len(traj_buffer):
                        cmd_q = traj_buffer[traj_index]
                        if current_weight < 0.8: cmd_q = traj_buffer[0]
                        else: traj_index += 1
                        for jid, rad in cmd_q.items():
                            if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = rad
                    else:
                        if self.traj_post_mode == 'WALKING':
                            control_mode = 'WALKING'; target_weight = 0.0; sent_zero_weight = False
                            self.log(">>> 退出序列完成，已切回行走模式 (WALKING)")
                            for w_id in WAIST_JOINTS: 
                                if w_id in sim._joint_map: sim.env.data.qpos[sim._joint_map[w_id]] = 0.0
                        else:
                            last_frame = traj_buffer[-1]; ik_target_q = last_frame.copy()
                            control_mode = 'IK_HOLD'; self.log("轨迹执行完毕，保持姿态")
                            self.traj_post_mode = 'IK_HOLD'
                    # [新增] 到达目标后，自动清除可视化轨迹
                        if hasattr(sim, 'set_trajectory_path'): 
                            sim.set_trajectory_path([])

                elif control_mode == 'SEQUENCE_EXECUTION':
                    if current_seg_idx < len(traj_segments):
                        current_seg = traj_segments[current_seg_idx]
                        frames = current_seg['frames']; action = current_seg['action']
                        if seg_progress < len(frames):
                            cmd_q = frames[seg_progress]
                            for jid, rad in cmd_q.items():
                                if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = rad
                            seg_progress += 1
                        else:
                            if seg_progress == len(frames): 
                                if action == ACT_GRASP:
                                    self.log(f"段 {current_seg_idx+1} 结束: 执行抓取...")
                                    if self.l10: self.l10.grasp(30)
                                    seg_progress += 1 
                                elif action == ACT_OPEN:
                                    self.log(f"段 {current_seg_idx+1} 结束: 执行张开...")
                                    if self.l10: self.l10.open()
                                    seg_progress += 1
                                else: current_seg_idx += 1; seg_progress = 0
                            else:
                                wait_frames = int(1.0 / loop_rate)
                                if seg_progress < len(frames) + 1 + wait_frames:
                                    cmd_q = frames[-1]
                                    for jid, rad in cmd_q.items():
                                        if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = rad
                                    seg_progress += 1
                                else: current_seg_idx += 1; seg_progress = 0
                    else:
                        self.log("序列执行完毕。")
                        if traj_segments:
                            last_frame = traj_segments[-1]['frames'][-1]
                            ik_target_q = last_frame.copy(); control_mode = 'IK_HOLD'
                        else: control_mode = 'IK_HOLD'
                        self.traj_post_mode = 'IK_HOLD'
                # ... (同级 elif 块) ...

                # === [新增] 任务链执行核心逻辑 ===
                elif control_mode == 'TASK_CHAIN_EXECUTION':
                    # 1. 检查是否全部完成
                    if self.current_task_idx >= len(self.task_chain):
                        self.log("🏁 任务链执行完毕")
                        control_mode = 'WALKING'
                        self.task_chain = []
                        continue

                    # 2. 获取当前任务
                    curr_task = self.task_chain[self.current_task_idx]
                    t_type = curr_task.get('type', TASK_TYPE_MOVE)
                    
                    # --- 子状态机 ---
                    if self.chain_sub_state == 'INIT':
                        self.log(f"👉 步骤 [{self.current_task_idx+1}] 类型: {t_type}")
                        if t_type == TASK_TYPE_MOVE:
                            pose = curr_task.get('pose', {})
                            tx, ty, tyaw = pose.get('x',0), pose.get('y',0), pose.get('yaw',0)
                            if ros_bridge:
                                ros_bridge.publish_goal(tx, ty, tyaw)
                                self.nav_goal_coords = (tx, ty) # 复用坐标记录
                                self.nav_goal_yaw = tyaw
                                self.chain_sub_state = 'NAVIGATING'
                                target_weight = 0.0
                            else:
                                self.log("⚠️ 无 ROS，跳过导航")
                                self.chain_sub_state = 'ARRIVED'
                                
                        elif t_type == TASK_TYPE_WAIT:
                            self.task_start_time = time.time()
                            self.chain_sub_state = 'WAITING'

                    elif self.chain_sub_state == 'NAVIGATING':
                        # 复用导航移动逻辑
                        if ros_bridge:
                            nav_vel = ros_bridge.get_cmd_vel()
                            vx, vy, rot = nav_vel
                            # 限幅
                            vx = np.clip(vx, -0.7, 2.0); vy = np.clip(vy, -0.4, 0.4); rot = np.clip(rot, -2.0, 2.0)
                            
                            # 执行移动
                            _maybe_send_loco(vx, vy, rot)
                            
                            # 仿真位置更新
                            sim_yaw = sim.env.data.qpos[12]
                            c, s = np.cos(sim_yaw), np.sin(sim_yaw)
                            sim_x += (vx * c - vy * s) * 0.02; sim_y += (vx * s + vy * c) * 0.02

                            # 到达检测
                            curr = ros_bridge.robot_pose
                            dist = math.sqrt((curr[0]-self.nav_goal_coords[0])**2 + (curr[1]-self.nav_goal_coords[1])**2)
                            spd = math.sqrt(vx**2 + vy**2 + rot**2)
                            yaw_err = 0.0
                            yaw_ok = True
                            if self.nav_goal_yaw is not None:
                                yaw_err = _angle_diff(self.nav_goal_yaw, curr[2])
                                yaw_ok = abs(yaw_err) < yaw_tolerance
                            
                            # 阈值：距离<0.2m 且 速度<0.1
                            if dist < 0.25 and spd < 0.1 and yaw_ok: 
                                self.log(f"✅ 到达节点 (误差 {dist:.2f}m)")
                                _maybe_send_loco(0.0, 0.0, 0.0, force=True)
                                self.chain_sub_state = 'ARRIVED'
                                self.nav_goal_yaw = None
                        
                        # RL 维持平衡
                        # if sim.policy: sim.update_rl()
                        cmd_q = sim.get_motor_qpos()

                    elif self.chain_sub_state == 'ARRIVED':
                        # 检查动作
                        act_file = curr_task.get('action')
                        if act_file and act_file != "None":
                            self.log(f"🎬 执行动作: {act_file}")
                            # =========== [修改 5] 优先读缓存 ===========
                            key_name = act_file
                            if not key_name.endswith(".json"): key_name += ".json"

                            traj = self.action_cache.get(key_name)
                            if not traj:
                                traj = recorder.load_trajectory(act_file)
                            # ==========================================

                            if traj:
                                gc.disable()
                                # === 关键：切出当前模式去执行动作 ===
                                replay_traj = traj
                                control_mode = 'TRANSITION' # 切入回放模式
                                target_weight = 1.0
                                transition_steps = 100
                                # 设置回放完后的“返程票”
                                self.traj_post_mode = 'TASK_CHAIN_NEXT' 
                                
                                if bridge.ok:
                                    src = last_joint_q if last_joint_q else last_full_states
                                    if not src:
                                        src = bridge.get_joint_states()
                                    replay_start_q = {k:v['q'] for k,v in src.items() if k in ARM_JOINT_IDS}
                                else: replay_start_q = sim.get_motor_qpos()
                                continue # 立即跳出，下一帧进入 TRANSITION
                            else:
                                self.log("❌ 动作文件无效")
                                self.chain_sub_state = 'DONE'
                        else:
                            self.chain_sub_state = 'DONE'

                    elif self.chain_sub_state == 'WAITING':
                        cond = curr_task.get('condition', {})
                        k, v_target = cond.get('key'), cond.get('val')
                        
                        # [修改] 获取带时间戳的信号数据
                        # 假设 self.task_start_time 是在这个任务进入 WAITING 状态瞬间赋值的 (time.time())
                        signal_data = self.signal_server.get_data_with_ts(k)
                        
                        v_curr = None
                        ts_curr = 0
                        
                        if signal_data:
                            v_curr = str(signal_data['val'])
                            ts_curr = signal_data['ts']
                        
                        # 保持原地不动 (RL update)
                        if loco: loco.StopMove()
                        #if sim.policy: sim.update_rl() # 如果你之前注释了这里，保持注释
                        cmd_q = sim.get_motor_qpos()

                        # [修改] 核心判断逻辑：
                        # 1. 值匹配
                        # 2. 信号的时间戳(ts_curr) 必须晚于 任务开始等待的时间(self.task_start_time)
                        #    这样就过滤掉了进入该任务之前发出的“脏信号”
                        if v_curr == str(v_target):
                            if ts_curr > self.task_start_time:
                                self.log(f"⚡ 信号匹配成功: {k}={v_curr} (延迟: {ts_curr - self.task_start_time:.2f}s)")
                                self.signal_server.pop_signal(k)
                                self.chain_sub_state = 'DONE'
                            else:
                                # 忽略过期信号的日志可以保留，或者也可以加时间限制
                                if loop_cnt % 100 == 0:
                                    # self.log(...) 
                                    pass

                        # =========== [修改 2] 优雅的日志打印逻辑 ===========
                        # 逻辑：当前时间 - 上次打印时间 > 5秒，才打印一次
                        current_ts = time.time()
                        if current_ts - self.last_wait_log_time > 5.0:
                            # 计算已经等待了多久
                            wait_duration = current_ts - self.task_start_time
                            self.log(f"⏳ 已等待 {wait_duration:.0f}秒... (目标: {k}={v_target})")
                            
                            # 更新上次打印时间
                            self.last_wait_log_time = current_ts
                        # =================================================
                            
                        
                        

                    elif self.chain_sub_state == 'DONE':
                        self.current_task_idx += 1
                        self.chain_sub_state = 'INIT'
                elif control_mode in ['TEACHING', 'REPLAYING', 'TEACH_STANDBY', 'TRANSITION']:
                    if control_mode == 'TEACHING':
                        joint_q = None
                        if bridge.ok:
                            src = last_joint_q if last_joint_q else last_full_states
                            if not src:
                                src = bridge.get_joint_states()
                            if src:
                                joint_q = {k: v['q'] for k, v in src.items()}
                        else:
                            joint_q = sim.get_motor_qpos()

                        if joint_q:
                            recorder.record_frame(joint_q)
                            cmd_q = {k: joint_q.get(k) for k in ARM_JOINT_IDS if k in joint_q}
                        else:
                            cmd_q = {}
                        kp_val = PASSIVE_KP; kd_val = PASSIVE_KD
                    elif control_mode == 'TEACH_STANDBY':
                        cmd_q = standby_q
                        for jid, rad in standby_q.items():
                            if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = rad
                        # if sim.policy: sim.update_rl()
                    elif control_mode == 'TRANSITION':
                        if replay_traj:
                            goal_q = replay_traj[0]['q']
                            progress = 1.0 - (transition_steps / 100.0)
                            for jid in ARM_JOINT_IDS:
                                start_rad = replay_start_q.get(jid, 0.0)
                                end_rad = goal_q.get(int(jid), 0.0)
                                val = start_rad + (end_rad - start_rad) * progress
                                cmd_q[jid] = val
                                if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = val
                            # if sim.policy: sim.update_rl()
                            kp_val = REPLAY_KP; kd_val = REPLAY_KD
                            transition_steps -= 1
                            if transition_steps <= 0: control_mode = 'REPLAYING'; replay_index = 0
                    # core/robot_process.py 约 500-530行之间

                    elif control_mode == 'REPLAYING':
                        if replay_index < len(replay_traj):
                            # ... (这部分保持不变，是正在回放时的逻辑) ...
                            frame = replay_traj[replay_index]
                            cmd_q = {int(k): v for k, v in frame['q'].items()}
                            for jid, rad in cmd_q.items():
                                if jid in sim._joint_map: sim.env.data.qpos[sim._joint_map[jid]] = rad
                            # if sim.policy: sim.update_rl() # 如果不需要仿真里的RL干扰，这行也可以注释掉
                            replay_index += 1
                            kp_val = REPLAY_KP; kd_val = REPLAY_KD
                        else:
                            # === [回放结束] ===
                            # =========== [深度优化 2.2] 动作结束，恢复并清理 GC ===========
                            gc.enable()
                            gc.collect() # 手动触发一次清理
                            # ==========================================================
                            # 1. 检查是否处于任务链模式
                            if self.traj_post_mode == 'TASK_CHAIN_NEXT':
                                self.log("🔄 动作完成，切断 RL 控制，返回任务链...")
                                
                                # [核心修改] 强制归零权重
                                # 这意味着：完全停止自定义控制，不运行 RL，不维持 IK
                                # 机器人将瞬间切换回 Unitree 原生运控状态
                                target_weight = 0.0 
                                sent_zero_weight = False # 强制发一次空指令以通过 sdk 归零
                                
                                # 切换状态
                                control_mode = 'TASK_CHAIN_EXECUTION'
                                self.chain_sub_state = 'DONE' # 标记当前动作节点完成
                                self.traj_post_mode = 'IK_HOLD' 
                            
                            else:
                                # 原有逻辑 (例如示教结束后的待命)
                                # 如果您在示教模式下也不想要 RL，可以把这里的 target_weight 也改为 0.0
                                # 但通常示教后需要保持姿态，所以这里保持原样或改为 IK_HOLD
                                control_mode = 'TEACH_STANDBY'
                                if replay_traj: standby_q = {int(k): v for k, v in replay_traj[-1]['q'].items()}

                if not self.args.sim_only and bridge.ok:
                    if current_weight > 0.01:
                        cmd_q = safety.enforce_waist_lock(cmd_q)
                        bridge.send_command(cmd_q, current_weight, kp=kp_val, kd=kd_val)
                    elif not sent_zero_weight:
                        bridge.send_command({}, 0.0)
                        sent_zero_weight = True
                    else: sent_zero_weight = False
            # [探针 T3] 核心控制计算(RL/IK/ROS)结束
            t3 = time.time()
            render_img = None
            # if self.status_queue.qsize() < 2: 
            #     if view_source == 'THIRD_PERSON': 
            #         # =========== [修改] 真机模式下禁止渲染仿真画面 ===========
            #         # 原代码: render_img = sim.get_render_frame(800, 600)
                    
            #         # 新逻辑: 仅在“纯仿真模式”下才消耗 CPU 去画 3D 图
            #         if self.args.sim_only:
            #             render_img = sim.get_render_frame(800, 600)
            #         else:
            #             # 真机模式下，这里设为 None，节省巨量 CPU 资源
            #             render_img = None 
            #         # =======================================================
            #     elif view_source == 'ROBOT_HEAD':
            #         if self.args.sim_only: render_img = sim.get_render_frame(640, 480, camera_name="head_camera")
            #         elif ros_bridge:
            #             real_img = ros_bridge.get_camera_image()
            #             if real_img is not None:
            #                 render_img = real_img; h, w, _ = render_img.shape
            #                 if vision_cursor is None: target_u, target_v = w // 2, h // 2; cross_color = (0, 255, 0)
            #                 else:
            #                     target_u, target_v = vision_cursor
            #                     target_u = max(0, min(target_u, w-1)); target_v = max(0, min(target_v, h-1))
            #                     cross_color = (0, 255, 255)
            #                 pos_base = ros_bridge.get_3d_pos_from_pixel(target_u, target_v)
            #                 import cv2
            #                 cv2.line(render_img, (target_u-15, target_v), (target_u+15, target_v), cross_color, 2)
            #                 cv2.line(render_img, (target_u, target_v-15), (target_u, target_v+15), cross_color, 2)
            #                 if pos_base is not None:
            #                     last_vision_pos = pos_base
            #                     info = f"XYZ: {pos_base[0]:.2f}, {pos_base[1]:.2f}, {pos_base[2]:.2f}"
            #                     cv2.putText(render_img, info, (target_u+10, target_v-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, cross_color, 1)
            #                 else:
            #                     cv2.putText(render_img, "NaN", (target_u+10, target_v-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            #             else:
            #                 render_img = np.zeros((480, 640, 3), dtype=np.uint8)
            #                 import cv2
            #                 cv2.putText(render_img, "WAITING FOR STREAM...", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            # [探针 T4] 图像渲染结束
            t4 = time.time()
            root_pos_curr = sim.env.data.qpos[0:3]
            goal_world = sim.get_goal_world() if hasattr(sim, 'get_goal_world') else sim.p_goal.copy()
            nav_data_pack = {}
            if ros_bridge:
                map_data, map_info, robot_pose, g_path, l_path, scan_data = ros_bridge.get_nav_data()
                nav_data_pack['robot_pose'] = robot_pose
                nav_data_pack['global_path'] = g_path
                nav_data_pack['local_path'] = l_path
                
                # [关键优化] 限制雷达数据发送频率到 10Hz (每 0.1s 发送一次)
                # 这样可以极大减轻 UI 线程的 json 解析和绘图压力
                current_time = time.time()
                if scan_data is not None and (current_time - last_scan_sent_time > 0.1):
                    nav_data_pack['scan'] = scan_data
                    last_scan_sent_time = current_time
                
                # 地图还是 2 秒发一次
                if map_data is not None and (current_time - last_map_sent_time > 2.0):
                    nav_data_pack['map'] = map_data
                    nav_data_pack['map_info'] = map_info
                    last_map_sent_time = current_time
            
            hand_force = [0]*5; hand_matrix = []
            if self.l10:
                if hasattr(self.l10, 'get_data'): hand_force, hand_matrix = self.l10.get_data()
                else: hand_force = self.l10.get_pressure()
            
            # =========== [修改] 通信降频：每 5 次循环发送一次 (100Hz -> 20Hz) ===========
            if loop_cnt % 5 == 0:
                if hasattr(sim, "update_kinematics"):
                    sim.update_kinematics()
                status_packet = {
                    'type': 'STATUS',
                    'app_state': app_state,
                    'fsm_id': cached_fsm_id, 
                    'mode': control_mode,
                    'weight': current_weight,
                    'arm_level': arm_level,
                    'walk_level': walk_level,
                    'sim_xy': (sim_x, sim_y),
                    'goal_relative': goal_world - root_pos_curr,
                    'p_goal': goal_world,
                    'p_hand': sim.get_fk_hand_pos(),
                    'vitals': last_vitals if bridge else {},
                    'joints': last_full_states if bridge else {},
                    'image': render_img,
                    'hand_force': hand_force,
                    'hand_matrix': hand_matrix,
                    'nav_data': nav_data_pack,
                    'vision_pos': last_vision_pos 
                }
                try:
                    # [优化] 如果队列满了，先扔掉旧的，确保发出去的是最新的
                    if self.status_queue.full():
                        try: self.status_queue.get_nowait()
                        except: pass
                    
                    self.status_queue.put_nowait(status_packet)
                except: pass
            # =========================================================================
            # [探针 T5] 通信打包结束
            t5 = time.time()
            elapsed = time.time() - loop_start
            sleep_time = loop_rate - elapsed
            if sleep_time > 0: time.sleep(sleep_time)
            
        state_poll_stop.set()
        fsm_poll_stop.set()
        loco_cmd_stop.set()
        loco_cmd_event.set()
        if state_thread: state_thread.join(timeout=1.0)
        if fsm_thread: fsm_thread.join(timeout=1.0)
        if loco_thread: loco_thread.join(timeout=1.0)
        self.signal_server.stop() # [新增]
        # 退出清理
        self.ros_manager.kill_all()
        try:
            if sim.renderer: sim.renderer.close()
        except: pass
        self.log("核心进程已安全退出。")
