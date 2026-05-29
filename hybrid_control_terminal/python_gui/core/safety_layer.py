# core/safety_layer.py
import numpy as np
from config import WAIST_JOINTS

class SafetyLayer:
    """
    安全层：拦截危险指令，保护硬件
    """
    def __init__(self):
        self.joint_limits = {} # 可以从xml加载，这里暂时留空
        self.max_delta_q = 0.2 # 每一帧关节最大突变弧度（防抽搐）

    def filter_arm_action(self, action, current_qpos):
        """
        对 RL 输出的动作进行安全过滤
        """
        # 1. 幅度限制：防止飞车
        safe_action = np.clip(action, -1.0, 1.0)
        
        # 2. (可选) 碰撞预测检测...
        
        return safe_action

    def check_hardware_health(self, bridge_vitals):
        """
        检查硬件健康状态
        返回: (is_safe, error_msg)
        """
        if not bridge_vitals:
            return True, ""
            
        # 温度保护
        if bridge_vitals.get('max_temp', 0) > 80:
            return False, "CRITICAL: Motor Overheat (>80C)"
            
        # 电压保护 (48V系统，低于38V可能欠压)
        if 0 < bridge_vitals.get('voltage', 48) < 38:
            return False, "WARNING: Low Voltage"
            
        return True, ""

    def enforce_waist_lock(self, command_q):
        """
        强制锁定腰部，防止手臂运动导致身体晃动
        """
        for j_id in WAIST_JOINTS:
            if j_id in command_q:
                command_q[j_id] = 0.0
        return command_q
