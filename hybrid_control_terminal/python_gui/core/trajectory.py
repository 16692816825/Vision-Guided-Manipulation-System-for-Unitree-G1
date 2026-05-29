# core/trajectory.py
import numpy as np
import math
from scipy.interpolate import CubicSpline # <--- 必须引入


def generate_cartesian_path(start_pos, end_pos, start_roll, end_roll, steps):
    path = []
    for i in range(steps):
        t = i / (steps - 1)
        s = (1.0 - math.cos(t * math.pi)) / 2.0
        
        current_pos = start_pos + (end_pos - start_pos) * s
        
        # [修改] 插值 Roll
        current_roll = start_roll + (end_roll - start_roll) * s
        
        # [修改] 返回字典 key: 'roll'
        path.append({
            'pos': current_pos,
            'roll': current_roll
        })
    return path

def generate_smooth_spline(points_list, freq=50):
    if len(points_list) < 2: return []

    key_pos = []
    key_roll = [] # [修改]
    times = [0.0]
    
    for i, p in enumerate(points_list):
        key_pos.append(p['pos'])
        key_roll.append(p['roll']) # [修改] 读取 roll
        if i > 0:
            times.append(times[-1] + p['duration'])
            
    key_pos = np.array(key_pos)
    key_roll = np.array(key_roll) # [修改]
    times = np.array(times)
    total_time = times[-1]
    
    cs_x = CubicSpline(times, key_pos[:, 0], bc_type='clamped')
    cs_y = CubicSpline(times, key_pos[:, 1], bc_type='clamped')
    cs_z = CubicSpline(times, key_pos[:, 2], bc_type='clamped')
    cs_roll = CubicSpline(times, key_roll, bc_type='clamped') # [修改]
    
    num_samples = int(total_time * freq)
    t_samples = np.linspace(0, total_time, num_samples)
    
    smooth_path = []
    for t in t_samples:
        pos = np.array([cs_x(t), cs_y(t), cs_z(t)])
        roll = float(cs_roll(t)) # [修改]
        smooth_path.append({'pos': pos, 'roll': roll}) # [修改]
        
    return smooth_path
