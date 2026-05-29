# core/ik_solver.py
import numpy as np
import mujoco
import math
from scipy.spatial.transform import Rotation as R

class IKSolver:
    def __init__(self, model, data, arm_joint_names):
        self.model = model
        self.data = data
        
        self.is_right_arm = any("right" in name for name in arm_joint_names)
        prefix = "right" if self.is_right_arm else "left"
        print(f"[IKSolver] Initializing {prefix.upper()} (Anti-Jitter Mode)")

        # 查找关键 Body ID
        self.shoulder_link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}_shoulder_pitch_link")
        
        possible_names = [f"{prefix}_wrist_roll_rubber_hand", f"{prefix}_hand_base_link", f"{prefix}_wrist_roll"]
        self.wrist_link_id = next((mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) 
                                 for n in possible_names if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) != -1), -1)
        
        # 索引缓存
        self.qpos_ids = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in arm_joint_names])
        self.dof_ids = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in arm_joint_names])
        self.limits = np.array([model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in arm_joint_names])

        # 舒适姿态 (Rest Pose) - 用于零空间回归
        self.rest_pose = np.array([0.0, 0.3 if not self.is_right_arm else -0.3, 0.0, 1.2, 0.0])
        self.tcp_offset_local = np.array([0.23, 0.0, 0.0]) 

        # 物理限制
        self.max_arm_reach = 0.50 # 略微减小最大臂长，防止死磕奇异点
        
        # 权重矩阵 W (对角阵) - 降低腕部权重，让肩肘承担更多任务
        self.W = np.diag([1.2, 1.2, 1.0, 0.8]) # 前4个关节的权重

    def solve(self, target_pos_world, current_q=None, target_roll=0.0):
        if self.wrist_link_id == -1: return None
        
        # 1. 状态初始化
        if current_q is not None:
            self.data.qpos[self.qpos_ids] = current_q
        else:
            # 如果没有传入当前值，必须从 mujoco data 读取，否则会归零
            pass 

        max_steps = 30 # 减少迭代次数，避免过度计算
        active_indices = [0, 1, 2, 3] # 主要参与 IK 的关节 (肩3 + 肘1)
        wrist_idx = 4                 # 腕部单独处理

        # 2. 目标空间投影 (防止目标点超出物理极限导致乱算)
        mujoco.mj_kinematics(self.model, self.data)
        shoulder_pos = self.data.xpos[self.shoulder_link_id]
        target_vec = np.array(target_pos_world) - shoulder_pos
        target_dist = np.linalg.norm(target_vec)
        
        if target_dist > self.max_arm_reach:
            # 如果目标太远，投影到球面上，并往回缩一点点(0.99)，避免完全伸直进入奇异点
            target = shoulder_pos + (target_vec / target_dist) * (self.max_arm_reach * 0.995)
        else:
            target = np.array(target_pos_world)

        # 3. 迭代解算 (DLS + Null Space)
        for step in range(max_steps):
            mujoco.mj_kinematics(self.model, self.data)
            
            # --- FK 计算 ---
            w_pos = self.data.xpos[self.wrist_link_id]
            w_mat = self.data.xmat[self.wrist_link_id].reshape(3, 3)
            curr_tcp = w_pos + w_mat @ self.tcp_offset_local
            
            diff_vec = target - curr_tcp
            dist = np.linalg.norm(diff_vec)
            
            # [优化1] 提前退出阈值
            if dist < 0.001: 
                break 

            # [优化2] 误差钳位 (Error Clamping) - 消除启动抖动
            # 限制单步最大期望位移为 3cm，防止误差过大导致计算出的 dq 飞升
            max_step_dist = 0.03
            if dist > max_step_dist:
                scaled_diff = (diff_vec / dist) * max_step_dist
            else:
                scaled_diff = diff_vec # 接近时线性收敛
                
            # --- 雅可比计算 ---
            jac = np.zeros((6, self.model.nv))
            mujoco.mj_jac(self.model, self.data, jac[:3], jac[3:], curr_tcp, self.wrist_link_id)
            J = jac[:3, self.dof_ids[active_indices]] # 只取前4关节的位置雅可比
            
            # --- DLS (阻尼最小二乘) ---
            # [优化3] 自适应阻尼 - 接近奇异点时增加阻尼
            # JJT = J @ J.T
            # manip = np.sqrt(max(0, np.linalg.det(JJT))) # 可操作度
            # damping = 0.05 + (0.1 if manip < 0.02 else 0.0) # 简单版自适应
            
            # 使用更稳定的权重 DLS 公式: dq = W^-1 * J.T * (J * W^-1 * J.T + k^2 * I)^-1 * dx
            W_inv = np.linalg.inv(self.W)
            matrix_to_inv = J @ W_inv @ J.T + np.eye(3) * (0.02 ** 2) # 固定小阻尼 0.02
            
            try:
                inv_mat = np.linalg.inv(matrix_to_inv)
            except:
                inv_mat = np.eye(3) # 奇异保护
                
            dq_task = W_inv @ J.T @ inv_mat @ scaled_diff
            
            # --- 零空间控制 (Null Space) ---
            # 投影矩阵 P = I - J_pinv * J (这里简化近似)
            # 实际上 DLS 的投影比较复杂，这里用近似公式
            # J_dagger = W_inv * J.T * inv_mat
            J_dagger = W_inv @ J.T @ inv_mat
            null_proj = np.eye(4) - J_dagger @ J
            
            curr_q = self.data.qpos[self.qpos_ids[active_indices]]
            
            # A. 姿态吸引力 (Rest Pose Attraction)
            q_bias = 0.05 * (self.rest_pose[active_indices] - curr_q)
            
            # B. 关节限位斥力 (Limit Repulsion)
            q_repulsion = np.zeros(4)
            for i in range(4):
                q_val = curr_q[i]
                q_min, q_max = self.limits[active_indices[i]]
                margin = 0.15 # 增大缓冲区 0.15 rad
                
                if q_val < q_min + margin:
                    q_repulsion[i] = 0.2 * (q_min + margin - q_val)
                elif q_val > q_max - margin:
                    q_repulsion[i] = -0.2 * (q_val - (q_max - margin))
            
            q_null_force = q_bias + q_repulsion

            # [核心优化4] 零空间力淡出 (Null Space Fading) - 消除终点抖动
            # 当极其接近目标时 (dist < 1cm)，逐渐关掉零空间力
            # 这样手臂就不会在 "我想去目标" 和 "我想回舒适区" 之间打架了
            fade_factor = np.clip(dist / 0.02, 0.0, 1.0) 
            dq_null = null_proj @ (q_null_force * fade_factor)
            
            # --- 更新关节 ---
            dq_total = dq_task + dq_null
            
            # 限幅：防止单步突变
            dq_total = np.clip(dq_total, -0.15, 0.15)
            
            self.data.qpos[self.qpos_ids[active_indices]] += dq_total
            
            # 强制限位
            self.data.qpos[self.qpos_ids[active_indices]] = np.clip(
                self.data.qpos[self.qpos_ids[active_indices]], 
                self.limits[active_indices, 0], self.limits[active_indices, 1]
            )

        # --- 阶段 B: 腕部 Roll 独立控制 ---
        # 保持之前的逻辑，这部分通常不会引起笛卡尔位置抖动
        self.data.qpos[self.qpos_ids[wrist_idx]] = 0.0
        mujoco.mj_kinematics(self.model, self.data)
        current_mat = self.data.xmat[self.wrist_link_id].reshape(3, 3)
        
        # 计算当前的 Roll (相对于世界)
        # 注意：这里需要更稳健的计算方式，防止万向节死锁导致的数值跳变
        # 我们假设手腕的 X 轴是指向 TCP 的
        
        # 简单方案：直接用 scipy 计算差异
        current_world_roll = R.from_matrix(current_mat).as_euler('xyz')[0]
        
        diff = target_roll - current_world_roll
        # 归一化到 -pi ~ pi
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        
        w_lim = self.limits[wrist_idx]
        best_roll = np.clip(diff, w_lim[0], w_lim[1])
        
        # 检查是否因为 clip 导致反转更优 (比如 -3.1 -> +3.1)
        # 这里简单处理，直接赋值 clip 后的结果
        self.data.qpos[self.qpos_ids[wrist_idx]] = best_roll

        return self.data.qpos[self.qpos_ids].copy()

def rpy_to_mat(r, p, y):
    return R.from_euler('xyz', [r, p, y], degrees=False).as_matrix()