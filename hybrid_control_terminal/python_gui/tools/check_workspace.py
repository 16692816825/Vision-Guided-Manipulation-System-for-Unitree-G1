# tools/check_workspace_ik.py
import os
import sys
import numpy as np
import mujoco
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

# 引入项目根目录，以便导入 core 模块
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)

# 导入我们刚刚写好的新版 IK Solver
from core.ik_solver import IKSolver

class WorkspaceScanner:
    def __init__(self, xml_path, arm='left'):
        print(f"[Init] Loading model: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        self.arm_type = arm
        
        # 定义关节名称
        if arm == 'left':
            joint_names = [
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", 
                "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint"
            ]
        else:
            joint_names = [
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", 
                "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"
            ]
            
        # 初始化 IK 求解器
        self.solver = IKSolver(self.model, self.data, joint_names)
        
        # 用于防穿模检查的身体半径
        self.BODY_RADIUS = 0.15

    def check_reachability(self, target_pos):
        """
        测试单个点是否可达
        """
        # 1. 简单的几何过滤 (如果点在身体内部，直接判死刑，不需要算 IK)
        dist_xy = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
        if dist_xy < self.BODY_RADIUS:
            return False, "COLLISION_PRE"

        # 2. 运行 IK
        # 我们给一个中性的手腕角度 0.0，让 IK 自己去算前4轴
        # 使用上一帧的姿态热启动 (warm start) 会由 solver 内部的 self.data 自动处理
        q_sol = self.solver.solve(target_pos, user_rotation=0.0)
        
        if q_sol is None:
            return False, "IK_FAIL"

        # 3. 验证结果 (Forward Kinematics)
        # solve() 已经把结果写进 self.data 了，我们直接读
        mujoco.mj_kinematics(self.model, self.data)
        
        tip_id = self.solver.wrist_link_id
        w_pos = self.data.xpos[tip_id]
        w_mat = self.data.xmat[tip_id].reshape(3, 3)
        curr_tcp = w_pos + w_mat @ self.solver.tcp_offset_local
        
        # 计算误差
        error = np.linalg.norm(curr_tcp - target_pos)
        
        # 4. 判定标准: 误差 < 2cm
        if error < 0.02:
            return True, "OK"
        else:
            return False, "UNREACHABLE"

    def scan_volume(self, x_range, y_range, z_range, step=0.05):
        """
        扫描一个立方体区域
        """
        x_vals = np.arange(x_range[0], x_range[1], step)
        y_vals = np.arange(y_range[0], y_range[1], step)
        z_vals = np.arange(z_range[0], z_range[1], step)
        
        reachable_points = []
        unreachable_points = []
        
        total = len(x_vals) * len(y_vals) * len(z_vals)
        count = 0
        start_time = time.time()
        
        print(f"[Scan] Starting scan of {total} points for {self.arm_type.upper()} arm...")
        print(f"[Scan] Area: X{x_range}, Y{y_range}, Z{z_range}")

        # 重置机器人姿态到舒适位，避免初始状态就在奇怪的地方
        self.data.qpos[:] = 0
        if self.arm_type == 'left':
            # 左臂稍微抬起
            ids = self.solver.qpos_ids
            self.data.qpos[ids[1]] = 0.5 
            self.data.qpos[ids[3]] = 1.0 
        else:
            ids = self.solver.qpos_ids
            self.data.qpos[ids[1]] = -0.5
            self.data.qpos[ids[3]] = 1.0
            
        mujoco.mj_forward(self.model, self.data)

        for x in x_vals:
            for y in y_vals:
                for z in z_vals:
                    target = np.array([x, y, z])
                    success, reason = self.check_reachability(target)
                    
                    if success:
                        reachable_points.append(target)
                    else:
                        # 为了可视化不那么乱，我们可以只保留一部分不可达点，或者全部保留
                        # 这里只保留一部分用于画边界
                        unreachable_points.append(target)
                    
                    count += 1
                    if count % 500 == 0:
                        elapsed = time.time() - start_time
                        speed = count / elapsed
                        print(f"\rProgress: {count}/{total} ({count/total*100:.1f}%) - {speed:.1f} pts/s", end="")

        print("\n[Scan] Done.")
        return np.array(reachable_points), np.array(unreachable_points)

    def visualize(self, reachable, unreachable):
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. 绘制机器人参考
        # 画个简易的躯干和头
        ax.plot([0,0], [0,0], [0, 1.3], 'k-', linewidth=3, label='Robot Center')
        
        # 2. 绘制可达点 (绿色)
        if len(reachable) > 0:
            # 降采样显示，防止卡顿
            step = max(1, len(reachable) // 3000)
            pts = reachable[::step]
            ax.scatter(pts[:,0], pts[:,1], pts[:,2], c='green', s=5, alpha=0.5, label='Reachable')
            
        # 3. 绘制不可达点 (红色，半透明，量少一点)
        if len(unreachable) > 0:
            step = max(1, len(unreachable) // 1000)
            pts = unreachable[::step]
            ax.scatter(pts[:,0], pts[:,1], pts[:,2], c='red', s=2, alpha=0.1, label='Unreachable')

        ax.set_xlabel('X (Forward)')
        ax.set_ylabel('Y (Left/Right)')
        ax.set_zlabel('Z (Height)')
        ax.set_title(f'{self.arm_type.upper()} Arm Reachability (IK Verified)')
        
        # 设置坐标轴比例一致
        ax.set_box_aspect([1,1,1])
        
        # 视角调整
        ax.view_init(elev=30, azim=-120)
        
        plt.legend()
        plt.show()

if __name__ == "__main__":
    xml_path = os.path.join(project_root, "unitree_mujoco", "unitree_robots", "g1", "g1_23dof.xml")
    
    # === 这里选择你要测试的手臂 ===
    ARM_TO_TEST = 'left' 
    
    scanner = WorkspaceScanner(xml_path, arm=ARM_TO_TEST)
    
    # === 设定扫描范围 (米) ===
    # 我们特意把 Y 范围设大一点，覆盖到身体另一侧，来验证你的猜想
    # 左臂在左侧是 Y > 0，右侧是 Y < 0
    # 我们扫描 Y 从 0.8 (左边最远) 到 -0.5 (右边过中线)
    x_range = [0.1, 0.7]   # 前后: 10cm 到 70cm
    y_range = [-0.4, 0.6]  # 左右: -40cm(右侧) 到 60cm(左侧)
    z_range = [0.2, 0.9]   # 高度: 20cm 到 90cm
    
    good, bad = scanner.scan_volume(x_range, y_range, z_range, step=0.04) # 4cm 精度
    
    scanner.visualize(good, bad)
