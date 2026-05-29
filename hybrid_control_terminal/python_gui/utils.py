import pathlib
import numpy as np
import mujoco
from core.settings_manager import settings

# === 伪造的环境类 ===
class MockEnv:
    def __init__(self, right_arm=False):
        # 1. 加载模型 (必须有这一步，否则 IK 和 界面都无法显示)
        xml_path = settings.get("robot", "model_xml_path", "unitree_mujoco/unitree_robots/g1/g1_23dof.xml")
        try:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)
            print(f"[MockEnv] 成功加载模型: {xml_path}")
        except Exception as e:
            print(f"[MockEnv] 模型加载失败: {e}")
            self.model = None
            self.data = None

        # 2. 伪造 RL 需要的属性 (防止报错)
        # 动作空间范围
        self.action_space = type('obj', (object,), {'low': -1.0, 'high': 1.0})
        
        # 关节名称 (IK Solver 需要用到)
        prefix = "right" if right_arm else "left"
        self._controlled_joint_names = [
            f"{prefix}_shoulder_pitch_joint", f"{prefix}_shoulder_roll_joint",
            f"{prefix}_shoulder_yaw_joint", f"{prefix}_elbow_joint", f"{prefix}_wrist_roll_joint"
        ]
        
        # 目标点标记 ID (用于可视化)
        self._goal_mid = -1
        if self.model:
            goal_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "goal_body")
            if goal_bid != -1: 
                self._goal_mid = int(self.model.body_mocapid[goal_bid])

    def reset(self):
        # 返回假的观测数据
        return np.zeros(10), {}

    def step(self, action):
        # 既然没有物理步进，直接返回空
        return np.zeros(10), 0.0, False, False, {}

    def _get_obs(self):
        return np.zeros(10)

    def _fk(self):
        # 正运动学占位
        return np.array([0.5, 0.0, 0.5])

# === 导出函数 ===

def load_policy(path):
    print(f"[系统] ⚠️ 精简模式：已跳过策略加载")
    return None

def make_env(render: bool, right_arm: bool):
    print("[系统] ⚠️ 精简模式：正在初始化 Mock MuJoCo 环境...")
    return MockEnv(right_arm=right_arm)
