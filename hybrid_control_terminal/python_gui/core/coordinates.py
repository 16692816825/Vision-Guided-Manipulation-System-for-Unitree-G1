# core/coordinates.py
import numpy as np
from scipy.spatial.transform import Rotation as R

class CoordinateSystem:
    """
    坐标系统一管理类
    约定：
    1. 所有位置 pos 均为 np.array([x, y, z])
    2. 所有姿态 rot 均为 3x3 旋转矩阵 或 四元数 [x, y, z, w]
    """

    @staticmethod
    def get_transform_matrix(pos, quat):
        """根据位置和四元数构建 4x4 变换矩阵 T"""
        T = np.eye(4)
        T[:3, 3] = pos
        T[:3, :3] = R.from_quat(quat).as_matrix() # quat: [x, y, z, w]
        return T

    @staticmethod
    def map_to_base(pos_map, robot_pos_map, robot_yaw_map):
        """
        将 [地图坐标] 转换为 [基座相对坐标]
        用于：导航点击地图上的点 -> 告诉手臂要去抓那个点
        """
        # 构建机器人在地图中的变换矩阵 (仅2D平面移动)
        # R_map_base
        cos_t = np.cos(robot_yaw_map)
        sin_t = np.sin(robot_yaw_map)
        R = np.array([
            [cos_t, -sin_t, 0],
            [sin_t,  cos_t, 0],
            [0,      0,     1]
        ])
        
        # p_base = R.T * (p_map - p_robot)
        pos_rel = np.array(pos_map) - np.array(robot_pos_map)
        return R.T @ pos_rel

    @staticmethod
    def base_to_map(pos_base, robot_pos_map, robot_yaw_map):
        """
        将 [基座相对坐标] 转换为 [地图坐标]
        用于：可视化，将手臂的目标点画在地图上
        """
        cos_t = np.cos(robot_yaw_map)
        sin_t = np.sin(robot_yaw_map)
        R = np.array([
            [cos_t, -sin_t, 0],
            [sin_t,  cos_t, 0],
            [0,      0,     1]
        ])
        
        # p_map = R * p_base + p_robot
        return R @ np.array(pos_base) + np.array(robot_pos_map)