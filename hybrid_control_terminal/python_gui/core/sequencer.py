# core/sequencer.py
import time
import numpy as np
import mujoco
from config import ARM_JOINT_IDS
import boot  # 引用你的 boot.py

class ActionSequencer:
    """
    专门负责执行阻塞式的长动作序列 (启动、关机、复位)
    并包含在动作执行期间维持 UI 刷新的同步逻辑
    """
    def __init__(self, loco, sim, bridge, status_queue, log_callback, sim_only=False):
        self.loco = loco
        self.sim = sim
        self.bridge = bridge
        self.status_queue = status_queue
        self.log = log_callback
        self.sim_only = sim_only

    # ==========================================================
    # 1. 启动序列
    # ==========================================================
    
    def start_squat(self):
        """执行蹲姿启动序列"""
        self.log("执行【蹲姿启动】序列...")
        
        if self.sim_only:
            self.sim.open_viewer()
            return

        if self.loco:
            self.loco.Damp()
            time.sleep(0.5)
            self.loco.Squat2StandUp() 
            
            # 这里的 300 步约等于 6秒 (假设 50Hz)
            self._wait_and_sync(300, "等待站立", current_app_state='STARTING') 

            self.loco.Start()
            time.sleep(1.0)
            
        self.sim.open_viewer()
        self.log("蹲姿启动完成")

    def start_hanger(self):
        """执行悬挂启动序列"""
        self.log("执行【悬挂启动】序列...")
        
        if self.sim_only:
            self.sim.open_viewer()
            return

        if self.loco:
            # 调用 boot.py 中的逻辑
            success = boot.run_hanger_start_logic(self.loco, log_callback=self.log)
            if not success:
                self.log("悬挂启动检测失败，请检查环境！")
                return

        self.sim.open_viewer()
        self.log("悬挂启动完成")

    # ==========================================================
    # 2. 停止/关机序列
    # ==========================================================

    # ==========================================================
    # 2. 停止/关机序列
    # ==========================================================

    def stop_squat(self, current_weight):
        """执行蹲姿关机序列 (安全增强版)"""
        self.log("执行【蹲姿关机】序列...")
        
        # 1. 停止位移，归零 RL 权重
        # 这步很关键，先让 RL 松手
        if self.loco: self.loco.StopMove()
        self._ramp_down_weight(current_weight)

        if self.loco:
            # === [新增] 安全检查与状态强制切换 ===
            try:
                # 获取当前 FSM ID
                current_fsm = boot.get_fsm_id(self.loco)
                self.log(f" -> 当前 FSM: {current_fsm}，准备切换至标准模式...")

                # 如果处于 500 (导航/运动) 或其他非标准状态，强制切回 801 (AI 模式)
                # 注意：G1 推荐切回 801 或 200 以响应趴下指令
                if current_fsm != 801:
                    self.loco.SetFsmId(801)
                    time.sleep(1.0) # 等待切换完成
                
                # 再次激活 Start，确保机器人双脚吸地站稳
                # 防止之前在示教模式下姿态奇怪，这里让运控接管并修正姿态
                self.loco.Start()
                time.sleep(1.5) # 等待姿态修正稳定

            except Exception as e:
                self.log(f"[WARN] 状态切换异常: {e}，尝试强行趴下")
            # =======================================

            # 2. 调用原生接口趴下
            self.log(" -> 正在趴下 (StandUp2Squat)...")
            self.loco.StandUp2Squat() 
            
            # 3. 等待趴下完成
            # 增加等待时间，确保趴到底
            self._wait_and_sync(250, "正在趴下", current_app_state='STOPPING')

            # 4. 切换至阻尼模式
            self.log(" -> ⬇️ 切换至阻尼模式 (Damp)")
            self.loco.Damp()
            time.sleep(0.5)

        self.log("✅ 蹲姿关机完成 (已泄力)")

    def stop_hanger(self, current_weight):
        """执行悬挂关机序列"""
        self.log("执行【悬挂关机】序列...")
        if self.loco: self.loco.StopMove()

        # 权重归零
        self._ramp_down_weight(current_weight)

        # 直接阻尼
        if self.loco:
            self.log("切换至阻尼模式 (Damp)...")
            self.loco.Damp()
            time.sleep(1.0)

        self.log("悬挂关机完成 (已泄力)")

    # ==========================================================
    # 3. 辅助逻辑 (权重平滑 & 状态同步)
    # ==========================================================

    def _ramp_down_weight(self, current_weight):
        """平滑降低 RL 权重至 0"""
        w = current_weight
        while w > 0.01:
            w *= 0.6
            if self.bridge and self.bridge.ok: 
                self.bridge.send_command({}, w)
            time.sleep(0.05)
        
        if self.bridge and self.bridge.ok: 
            self.bridge.send_command({}, 0.0)

    def _wait_and_sync(self, steps, msg_prefix, current_app_state='BUSY'):
        """
        [关键] 阻塞式等待，但保持与 UI 的状态同步 (画面更新、关节数据回传)
        防止在长动作期间 UI 卡死
        """
        for i in range(steps):
            loop_start = time.time()
            
            # 1. 偶尔打印进度
            if i % 50 == 0: 
                self.log(f"{msg_prefix}... {int(i/50)+1}/{int(steps/50)+1}")
            
            # 2. 同步真机状态到仿真
            joints_data = {}
            vitals_data = {}
            if self.bridge and self.bridge.ok: 
                full = self.bridge.get_joint_states()
                joints_data = full
                vitals_data = self.bridge.get_robot_vitals()
                q_map = {k: v['q'] for k, v in full.items()}
                if hasattr(self.sim, 'sync_with_real'): 
                    self.sim.sync_with_real(q_map, vitals_data.get('quat'))
            
            # 3. 渲染图像
            render_img = self.sim.get_render_frame(800, 600)
            
            # 获取位置与朝向 (计算 Yaw)
            root_pos_curr = self.sim.env.data.qpos[0:3].copy()
            root_quat = self.sim.env.data.qpos[3:7]
            mat = np.zeros(9); mujoco.mju_quat2Mat(mat, root_quat)
            sim_yaw = np.arctan2(mat[3], mat[0]) 

            goal_world = self.sim.get_goal_world() if hasattr(self.sim, 'get_goal_world') else self.sim.p_goal.copy()
            
            # 4. 构建状态包
            # 注意：在过渡期间，大部分高级数据 (nav, hand) 设为空或默认值
            status_packet = {
                'type': 'STATUS',
                'app_state': current_app_state,
                'fsm_id': -1,                    
                'mode': 'WALKING',               # 过渡期间默认为行走模式
                'weight': 0.0,                   # 过渡期间无 RL 权重
                'arm_level': 1,
                'walk_level': 1,
                
                'robot_base_pos': root_pos_curr,
                'robot_base_yaw': sim_yaw,
                'goal_relative': goal_world - root_pos_curr,
                'p_goal': goal_world,
                'p_hand': self.sim.get_fk_hand_pos(),
                
                'vitals': vitals_data,
                'joints': joints_data,
                'hand_force': [0]*5,    # 默认空数据
                'hand_matrix': [],      # 默认空数据
                'image': render_img,
                'nav_data': {},         # 默认空数据
                'vision_pos': None
            }
            
            try:
                # 队列防爆：如果满了先丢弃旧的
                if self.status_queue.full(): self.status_queue.get_nowait()
                self.status_queue.put_nowait(status_packet)
            except: pass
            
            # 5. 维持 50Hz 时序 (0.02s)
            elapsed = time.time() - loop_start
            if elapsed < 0.02: time.sleep(0.02 - elapsed)
