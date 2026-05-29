# core/bridge.py
import threading
import time
import numpy as np
from config import WAIST_JOINTS, LEG_DRIVE_JOINTS

class RobotBridge:
    def __init__(self, iface, domain=0):
        self.ok = False
        self.last_state = None
        self._lock = threading.Lock()
        self._pub = None
        self._sub = None
        self._cmd = None
        self._crc = None

        try:
            from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.utils.crc import CRC
            
            # 初始化 DDS
            print(f"[RobotBridge] 正在初始化 DDS (网卡: {iface})...")
            ChannelFactoryInitialize(domain, iface)

            self._cmd = unitree_hg_msg_dds__LowCmd_()
            self._crc = CRC()
            
            self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self._pub.Init()
            
            self._sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._sub.Init(self._state_handler, 10)
            
            self.ok = True
            self._avg_vol_smooth = 0.0
            print("[RobotBridge] DDS 初始化成功！")
            
        except Exception as e:
            print(f"[RobotBridge] 初始化失败: {e}")
            self.ok = False
            
    def _state_handler(self, msg):
        with self._lock: self.last_state = msg

    def wait_for_state(self, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                if self.last_state: return True
            time.sleep(0.1)
        return False

    # [关键修复 1] 返回完整字典结构，解决 UI 报错
    def get_joint_states(self, minimal=False):
        if not self.ok or not self.last_state: return {}
        states = {}
        with self._lock:
            for i, m in enumerate(self.last_state.motor_state):
                states[i] = {'q': m.q}
                if minimal:
                    continue

                # [修复] 温度处理：处理可能是列表的情况
                raw_temp = m.temperature
                final_temp = 0
                
                # 如果是数字直接用
                if isinstance(raw_temp, (int, float)):
                    final_temp = int(raw_temp)
                # 如果是列表/数组，取第一个元素
                elif hasattr(raw_temp, '__len__') and len(raw_temp) > 0:
                    final_temp = int(raw_temp[0])
                
                states[i]['dq'] = m.dq
                states[i]['tau'] = m.tau_est
                states[i]['temp'] = final_temp # 使用处理后的温度
        return states

    def get_robot_vitals(self) -> dict:
        res = {'rpy': [0,0,0], 'quat': [1,0,0,0], 'voltage': 0, 'soc': 0, 'max_temp': 0}
        if not self.ok or not self.last_state: return res
        
        with self._lock:
            try:
                msg = self.last_state
                if hasattr(msg, 'imu_state'):
                    res['rpy'] = list(msg.imu_state.rpy)
                    res['quat'] = list(msg.imu_state.quaternion)
                
                max_t, tot_v, c = 0, 0, 0
                for m in msg.motor_state:
                    # 温度处理
                    t = m.temperature
                    if hasattr(t, '__len__') and len(t) > 0: val_t = int(t[0]) # 修复之前的列表问题
                    else: val_t = int(t)
                    if val_t > max_t: max_t = val_t
                    
                    # 累加电压 (过滤掉 0V 的无效数据)
                    if m.vol > 10: 
                        tot_v += m.vol
                        c += 1
                
                res['max_temp'] = int(max_t)
                
                if c > 0:
                    raw_avg_vol = tot_v / c
                    
                    # [优化 1] 平滑滤波 (Low-Pass Filter)
                    # 如果是第一次运行，直接赋值；否则取 98% 的旧值 + 2% 的新值
                    # 这样电压变化会非常平缓，彻底消除数字跳动
                    if self._avg_vol_smooth == 0.0:
                        self._avg_vol_smooth = raw_avg_vol
                    else:
                        self._avg_vol_smooth = self._avg_vol_smooth * 0.98 + raw_avg_vol * 0.02
                    
                    final_vol = self._avg_vol_smooth
                    res['voltage'] = final_vol
                    
                    # [优化 2] 调整电压映射范围 (根据 G1 实际表现调整)
                    # 0%   = 41.0V (保护电压)
                    # 100% = 53.5V (稍微降低满电门槛，之前是 54.6V)
                    v_min = 41.0
                    v_max = 53.5
                    
                    pct = (final_vol - v_min) / (v_max - v_min) * 100.0
                    res['soc'] = max(0, min(100, int(pct)))
                    
            except Exception as e: 
                print(f"Vitals Error: {e}")
                pass
        return res

    # [关键修复 3] 增强指令下发，支持示教模式的 Kp=0
    def send_command(self, arm_q, weight, kp=None, kd=None):
        if not self.ok: return
        with self._lock:
            # 混合权重: 1.0 表示完全由 LowCmd 控制
            self._cmd.motor_cmd[29].q = np.clip(weight, 0.0, 1.0)
            
            if weight > 0.01:
                for i, q in arm_q.items():
                    # 确保 ID 是整数
                    try: i = int(i)
                    except: continue
                    
                    if i >= 30: continue # 防止数组越界
                    
                    m = self._cmd.motor_cmd[i]
                    m.q = q
                    m.dq = 0
                    m.tau = 0
                    
                    # 动态 Kp/Kd 设置
                    if kp is not None:
                        m.kp = float(kp)
                    else:
                        m.kp = 60.0 if i in WAIST_JOINTS else 50.0
                        
                    if kd is not None:
                        m.kd = float(kd)
                    else:
                        m.kd = 2.0 if i in WAIST_JOINTS else 1.2
                        
            # CRC 校验
            if hasattr(self._crc, "Crc"): self._cmd.crc = self._crc.Crc(self._cmd)
            else: self._cmd.crc = self._crc.calculate_crc(self._cmd)
            
            self._pub.Write(self._cmd)

class MockBridge:
    def __init__(self):
        self.ok = True
        print("[MockBridge] 虚拟硬件接口 (Sim Mode)")
    def wait_for_state(self, t=1): return True
    def get_joint_states(self, minimal=False): 
        # [修复] Mock 也要返回完整结构
        if minimal:
            return {i: {'q': 0.0} for i in range(30)}
        return {i: {'q': 0.0, 'dq': 0.0, 'tau': 0.0, 'temp': 30} for i in range(30)}
    def get_robot_vitals(self): return {'rpy':[0,0,0],'quat':[1,0,0,0],'voltage':48.5,'soc':88,'max_temp':36}
    def send_command(self, q, w, kp=None, kd=None): pass
