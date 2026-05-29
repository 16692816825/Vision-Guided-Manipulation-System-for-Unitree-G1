# core/recorder.py
import json
import time
import numpy as np
import pathlib
from config import DATA_LOG_DIR, ARM_JOINT_IDS

class TrajectoryRecorder:
    def __init__(self):
        self.buffer = []
        self.start_time = 0
        self.recording = False

    def start(self):
        self.buffer = []
        self.start_time = time.time()
        self.recording = True

    def record_frame(self, joint_states):
        """
        记录一帧数据
        joint_states: dict {motor_id: q_pos, ...}
        """
        if not self.recording: return
        
        # 只记录手臂关节
        frame = {
            't': time.time() - self.start_time,
            'q': {str(k): float(v) for k, v in joint_states.items() if k in ARM_JOINT_IDS}
        }
        self.buffer.append(frame)

    def stop_and_save(self, filename):
        self.recording = False
        if not self.buffer: return False
        
        path = DATA_LOG_DIR / f"{filename}.json"
        try:
            with open(path, 'w') as f:
                json.dump(self.buffer, f)
            return True
        except Exception as e:
            print(f"Save Error: {e}")
            return False

    @staticmethod
    def load_trajectory(filename):
        if not filename.endswith(".json"):
            filename += ".json"
            
        path = DATA_LOG_DIR / filename
        if not path.exists(): 
            print(f"[Recorder] 文件不存在: {path}")
            return None
        
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 1. 字典解包 (处理 {"frames": [...]})
            if isinstance(data, dict):
                for key in ["frames", "trajectory", "data", "points"]:
                    if key in data and isinstance(data[key], list):
                        data = data[key]
                        break
            
            if not isinstance(data, list):
                return None

            traj = []
            dt_default = 0.02 
            
            # [设置] 针对这一批特定文件的加速倍率
            # 2 = 2倍速, 3 = 3倍速。如果你觉得还慢，就改成 4
            flat_data_speedup = 8

            for i, frame in enumerate(data):
                if isinstance(frame, str):
                    try: frame = json.loads(frame)
                    except: continue
                if not isinstance(frame, dict): continue
                
                # --- 1. 获取/生成时间戳 ---
                if 't' in frame: t_val = frame['t']
                elif 'time' in frame: t_val = frame['time']
                else: t_val = i * dt_default 
                
                # --- 2. 提取关节数据 ---
                q_data = None
                
                # 情况A: 标准嵌套格式 (这种格式保持原速，不跳帧)
                if 'q' in frame: q_data = frame['q']
                elif 'joints' in frame: q_data = frame['joints']
                
                # 情况B: 扁平格式 (你的那一批文件)
                if q_data is None:
                    # 检查是不是包含关节ID键
                    if '15' in frame or '22' in frame or '0' in frame:
                        # === [核心修改] 只有在这里进行抽帧 ===
                        # 如果当前帧数不能被倍率整除，就跳过
                        if i % flat_data_speedup != 0:
                            continue
                        # ==================================
                        q_data = frame

                # --- 3. 数据转换 (保持不变) ---
                if q_data:
                    try:
                        q_converted = {}
                        for k, v in q_data.items():
                            if str(k).isdigit(): 
                                q_converted[int(k)] = float(v)
                        if q_converted:
                            traj.append({'t': t_val, 'q': q_converted})
                    except: pass

            if not traj:
                print("[Recorder Error] 无法解析数据，格式不匹配")
                return None

            #print(f"[Recorder] 成功加载 {len(traj)} 帧 (自动适配格式)")
            return traj

        except Exception as e:
            print(f"Load Error: {e}")
            return None
