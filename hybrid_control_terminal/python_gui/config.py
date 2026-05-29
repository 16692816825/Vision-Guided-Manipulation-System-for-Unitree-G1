# config.py
import math
import pathlib  # <--- [关键修复] 必须导入这个库
from core.settings_manager import settings 
def deg_to_rad(degrees: float) -> float:
    return degrees * (math.pi / 180.0)

# 关节定义
JOINTS = [
    (15, "L shoulder-pitch", "left_shoulder_pitch"), (16, "L shoulder-roll", "left_shoulder_roll"),
    (17, "L shoulder-yaw", "left_shoulder_yaw"), (18, "L elbow", "left_elbow"),
    (19, "L wrist-roll", "left_wrist_roll"), (22, "R shoulder-pitch", "right_shoulder_pitch"),
    (23, "R shoulder-roll", "right_shoulder_roll"), (24, "R shoulder-yaw", "right_shoulder_yaw"),
    (25, "R elbow", "right_elbow"), (26, "R wrist-roll", "right_wrist_roll"),
]

WAIST_JOINTS = [12, 13, 14]
ARM_JOINT_IDS = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]
# 腿部主要关节 (髋+膝) 用于里程计
LEG_DRIVE_JOINTS = [0, 1, 3, 4, 6, 7, 9, 10]

# 速度档位 (0=慢, 1=中, 2=快)
ARM_SPEED_LEVELS = [0.3, 0.6, 1.2] 
WALK_SPEED_LEVELS = [1.0, 2.0, 3.0] 

# 基础速度参数
BASE_WALK_SPEED = 2
BASE_TURN_SPEED = 0.5
BASE_STRAFE_SPEED = 0.2
REACH_THRESHOLD = 0.03

# 运动学参数 (V5.2/V5.3)
MOVEMENT_EFFICIENCY = 0.85 
INERTIA_ALPHA = 0.08
VELOCITY_DEADZONE = 0.08
ODOMETRY_FACTOR = 0.025 
MOTION_DEADZONE = 0.5

# --- [V6.0 新增配置] ---

# 数据记录目录
DATA_LOG_DIR = pathlib.Path("data_logger")
# 自动创建目录，防止报错
DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)

# 示教模式参数
# 拖拽模式下的电机参数 (Kp=0 表示无刚度，Kd=1.5 提供轻微阻尼感，防止像面条一样乱甩)
PASSIVE_KP = 0.0
PASSIVE_KD = 1.5 

# 回放模式下的电机参数 (需要较硬的刚度来精准复现)
REPLAY_KP = 80.0
REPLAY_KD = 2.0

# [新增] IK 逆运动学控制参数
# 需要较高的刚度以保证到达精度
IK_KP = 60
IK_KD = 4
IK_ERROR_TOLERANCE = 0.01  # 允许的误差范围 (米)

DEFAULT_ARM_ORIGIN_OFFSET = [0.0, 0.0, 0.0] # 手臂基座相对于机器人基座的默认偏移 (米)
RL_OBSERVATION_FRAME = 'base'  # 强化学习观测坐标系 ('base' 或 'map')

# --- [新增] 视觉外参配置 (基于 CAD 图纸) ---
# 相机相对于 Base Link (腰部/骨盆) 的位置
# 注意：你需要确认 CAD 图的原点是否对应机器人的 Base Link
# 如果 CAD 原点是脚底，而 Base Link 是腰部，这里的 Z 需要减去腿长
# 这里假设 CAD 原点就是 Base Link
# 尝试从配置中获取，如果获取失败则使用默认值
# 正确写法: 直接获取具体的 key
CAM_OFFSET_X = settings.get("calibration", "cam_offset_x", 0.0476)
CAM_OFFSET_Y = settings.get("calibration", "cam_offset_y", 0.0)
CAM_OFFSET_Z = settings.get("calibration", "cam_offset_z", 0.4627)
CAM_PITCH_DEG = settings.get("calibration", "cam_pitch_deg", 42.0)
# --- [新增] 航点动作枚举 ---
ACT_NONE  = 0  # 无动作 (仅路过)
ACT_GRASP = 1  # 到达后抓取 (并等待)
ACT_OPEN  = 2  # 到达后张开 (并等待)


# --- [新增] G1 23-DOF 关节映射表 ---
# 格式: ID: "中文显示名称"
G1_JOINT_MAP = {
    # 左腿 (0-5)
    0: "左髋-Pitch",   1: "左髋-Roll",    2: "左髋-Yaw",
    3: "左膝-Knee",   4: "左踝-Pitch",  5: "左踝-Roll",
    
    # 右腿 (6-11)
    6: "右髋-Pitch",   7: "右髋-Roll",    8: "右髋-Yaw",
    9: "右膝-Knee",   10: "右踝-Pitch", 11: "右踝-Roll",
    
    # 腰部 (12) - 23DOF 只有 Yaw
    12: "腰部-Yaw",
    
    # 左臂 (15-19)
    15: "左肩-Pitch", 16: "左肩-Roll",  17: "左肩-Yaw",
    18: "左肘-Elbow", 19: "左腕-Roll",
    
    # 右臂 (22-26)
    22: "右肩-Pitch", 23: "右肩-Roll",  24: "右肩-Yaw",
    25: "右肘-Elbow", 26: "右腕-Roll"
}

# config.py (添加在末尾)

# --- 预设姿态库 (角度制) ---
# 格式: {关节名: 角度}
# 这里的 Roll 是绝对值，逻辑层会根据左右臂自动处理符号
PRESET_POSES = {
    "RETRACT": { # 收缩态
        "pitch": 13.99, "roll":16.33, "yaw": -4.67, "elbow": 46.53, "wrist": 0.0, 
        "duration": 2.5
    },
    "LIFT": {    # 抬高态
        "pitch": 60.0,  "roll": 15.0, "yaw": 0.0, "elbow": -20.0,  "wrist": 0.0,
        "duration": 1.5
    },
    "READY": {   # 预备态
        "pitch": 0.0,  "roll": 0.0,  "yaw": 30.0, "elbow": 0.0,  "wrist": 0.0,
        "duration": 2.0
    }
}
