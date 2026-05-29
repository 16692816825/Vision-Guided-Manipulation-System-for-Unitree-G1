# core/sim_engine.py
import threading
import time
import numpy as np
from config import JOINTS, ARM_JOINT_IDS

try:
    import mujoco
except ImportError:
    print("[错误] 请安装 mujoco: pip install mujoco")

class SimEngine:
    def __init__(self, env, policy, speed_obj):
        self.env = env
        self.policy = policy
        self.speed = speed_obj
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.last_debug_print = 0  
        
        # [核心修改 1] 初始化目标点管理 (Body-Locked 模式)
        self.goal_pos_base = np.array([0.35, 0.0, 0.15]) 
        self.goal_pos_world_cache = np.array([0.0, 0.0, 0.0])
        
        # [新增] 轨迹可视化点列表
        self.visual_traj_points = [] 
        
        self.obs, _ = env.reset()
        self._joint_map = {} 
        self._init_joint_map()
        
        self.has_free_joint = (self.env.model.nq >= 30)
        self.foot_body_ids = []
        for name in ["left_ankle_roll_link", "right_ankle_roll_link", "left_foot", "right_foot", "left_toe", "right_toe"]:
             id = mujoco.mj_name2id(self.env.model, mujoco.mjtObj.mjOBJ_BODY, name)
             if id != -1: self.foot_body_ids.append(id)

        # --- 渲染与摄像机 ---
        self.renderer = None
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = [0.0, 0.0, 0.5] 
        self.camera.distance = 2.5
        self.camera.azimuth = 135 
        self.camera.elevation = -20
        
        self.enable_render = False
        self.render_width = 0
        self.render_height = 0
        self.MAX_WIDTH = 1920; self.MAX_HEIGHT = 1200
        
        self._update_mocap()

    def _init_joint_map(self):
        def get_adr(name): return int(self.env.model.jnt_qposadr[mujoco.mj_name2id(self.env.model, mujoco.mjtObj.mjOBJ_JOINT, name)])
        joint_names = {
            0: "left_hip_pitch_joint", 1: "left_hip_roll_joint", 2: "left_hip_yaw_joint", 3: "left_knee_joint", 4: "left_ankle_pitch_joint", 5: "left_ankle_roll_joint",
            6: "right_hip_pitch_joint", 7: "right_hip_roll_joint", 8: "right_hip_yaw_joint", 9: "right_knee_joint", 10: "right_ankle_pitch_joint", 11: "right_ankle_roll_joint",
            12: "waist_yaw_joint", 13: "waist_roll_joint", 14: "waist_pitch_joint",
            15: "left_shoulder_pitch_joint", 16: "left_shoulder_roll_joint", 17: "left_shoulder_yaw_joint", 18: "left_elbow_joint", 19: "left_wrist_roll_joint",
            22: "right_shoulder_pitch_joint", 23: "right_shoulder_roll_joint", 24: "right_shoulder_yaw_joint", 25: "right_elbow_joint", 26: "right_wrist_roll_joint"
        }
        try:
            for motor_id, j_name in joint_names.items():
                try: adr = get_adr(j_name); self._joint_map[motor_id] = adr
                except: pass
        except: pass

    def set_goal_local(self, x, y, z):
        with self.lock:
            self.goal_pos_base = np.array([x, y, z])
            self._update_mocap()

    def get_goal_local(self):
        return self.goal_pos_base.copy()
    
    def set_goal_world(self, x, y, z):
        with self.lock:
            self.goal_pos_world_cache = np.array([x, y, z])
            if self.env._goal_mid != -1:
                self.env.data.mocap_pos[self.env._goal_mid] = self.goal_pos_world_cache

    def get_goal_world(self):
        return self.goal_pos_world_cache.copy()

    def set_trajectory_path(self, points):
        """
        :param points: list of dict {'pos': [x,y,z], 'type': int} 
                       或者简单的 list of [x,y,z]
        """
        with self.lock:
            # 统一格式化为列表
            self.visual_traj_points = points

    def _update_mocap(self):
        if self.env._goal_mid == -1: return

        base_pos_world = self.env.data.qpos[0:3].copy()
        base_quat = self.env.data.qpos[3:7].copy()
        
        rot_mat = np.zeros(9)
        mujoco.mju_quat2Mat(rot_mat, base_quat)
        R_base_to_world = rot_mat.reshape(3, 3)
        
        offset_world = R_base_to_world @ self.goal_pos_base
        target_pos_world = base_pos_world + offset_world
        
        self.env.data.mocap_pos[self.env._goal_mid] = target_pos_world
        self.goal_pos_world_cache = target_pos_world

    def sync_with_real(self, real_qpos, real_quat=None, update_arms=True, root_pos_offset=(0.0, 0.0)):
        # [修改] 如果没有真实数据，直接返回
        if not real_qpos: return
        
        with self.lock:
            # 1. 仅保留数据赋值 (这是最轻量级的操作)
            for motor_id, rad in real_qpos.items():
                if not update_arms and motor_id in ARM_JOINT_IDS: continue
                if motor_id in self._joint_map:
                    self.env.data.qpos[self._joint_map[motor_id]] = rad
            
            # 2. 如果有自由度关节(底盘)，赋值位置
            if self.has_free_joint:
                if real_quat and len(real_quat) == 4: 
                    self.env.data.qpos[3:7] = real_quat
                self.env.data.qpos[0] = root_pos_offset[0]
                self.env.data.qpos[1] = root_pos_offset[1]
                # self.env.data.qpos[2] = 1.0 # Z轴高度可以写死，或者不更新
        return  

    def update_rl(self):
        with self.lock:
            self._update_mocap()
            mujoco.mj_forward(self.env.model, self.env.data)
            self.obs = self.env._get_obs()
            action, _ = self.policy.predict(self.obs, deterministic=True)
            action = np.clip(action * self.speed.mult, self.env.action_space.low, self.env.action_space.high)
            self.obs, _, _, _, _ = self.env.step(action)

    def update_kinematics(self):
        with self.lock:
            mujoco.mj_kinematics(self.env.model, self.env.data)

    def get_fk_hand_pos(self): return self.env._fk()
    def get_motor_qpos(self):
        if not hasattr(self.env, "_motor_qadr"):
            self.env._motor_qadr = {idx: int(self.env.model.jnt_qposadr[mujoco.mj_name2id(self.env.model, mujoco.mjtObj.mjOBJ_JOINT, mj_short + "_joint")]) for idx, _, mj_short in JOINTS}
        return {idx: float(self.env.data.qpos[adr]) for idx, adr in self.env._motor_qadr.items()}

    def open_viewer(self):
        self.enable_render = True

    def move_camera(self, action, dx, dy):
        with self.lock:
            if self.renderer is None: return
            try:
                mujoco.mjv_moveCamera(
                    self.env.model, action, dx, dy, self.renderer.scene, self.camera
                )
            except Exception as e: print(f"Cam Err: {e}")

    def get_render_frame(self, req_width, req_height):
        # 渲染彻底关闭，避免 GPU/CPU 开销和 OpenGL 初始化报错。
        return None
    def close(self):
        self.stop_evt.set()
        self.enable_render = False
        if self.renderer:
            self.renderer.close()
