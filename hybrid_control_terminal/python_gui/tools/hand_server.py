# tools/hand_server.py
import socket
import json
import time
import sys
import os
import threading

# 确保能引用到项目根目录下的 LinkerHand
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from LinkerHand.linker_hand_api import LinkerHandApi

# === 配置 ===
ROBOT_IP = "0.0.0.0"  # 监听所有网卡
ROBOT_PORT = 9999     # 接收指令端口
PC_IP = "192.168.123.161" # 【注意】请修改为你电脑的 IP 地址 (G1通常在123网段)
PC_PORT = 9998        # 发送数据端口
CAN_CHANNEL = "can0"  # 机器人内部通常是 can0

class HandServer:
    def __init__(self):
        print(f"[Server] 正在初始化 L10 (CAN: {CAN_CHANNEL})...")
        try:
            # 初始化 API
            self.hand = LinkerHandApi(hand_joint="L10", hand_type="left", can=CAN_CHANNEL)
            # 设置默认速度
            self.hand.set_speed([180, 250, 250, 250, 250])
            print("[Server] L10 初始化成功")
        except Exception as e:
            print(f"[Error] L10 初始化失败: {e}")
            sys.exit(1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ROBOT_IP, ROBOT_PORT))
        self.running = True

    def _calc_dynamic_pose(self, mm):
        """根据直径计算抓取姿态 (参考你的 dynamic_grasping.py)"""
        pose = [255, 70, 255, 255, 255, 255, 255, 255, 255, 120] # 基础准备姿态
        
        # 握拳基础
        base_fist = [60, 60, 25, 25, 25, 25, 255, 255, 255, 58]
        
        offset = int(mm * 2)
        final_pose = list(base_fist)
        final_pose[0] = min(255, 60 + offset) # 拇指
        final_pose[2] = min(255, 25 + offset) # 食指
        final_pose[3] = min(255, 25 + offset)
        final_pose[4] = min(255, 25 + offset)
        final_pose[5] = min(255, 25 + offset)
        
        return final_pose

    def feedback_loop(self):
        """循环读取压力并发送回 PC"""
        print("[Server] 启动压力回传线程...")
        while self.running:
            try:
                # 获取压力 (返回的是列表)
                # L10 通常没有矩阵传感器，如果是 L10，get_force 返回的是电流或简易压力
                # 这里假设 get_force 返回 [thumb, index, middle, ring, pinky] 或类似数据
                force_data = self.hand.get_force() 
                
                # 如果是 None，给个空数据
                if force_data is None: force_data = [0]*5
                
                # 某些版本 API 返回的是 tuple (force, approach...)
                if isinstance(force_data, tuple) or isinstance(force_data, list):
                    # 取第一个元素作为压力数据 (根据你的 get_force.py)
                    if len(force_data) > 0 and isinstance(force_data[0], (list, tuple)):
                        force_data = force_data[0]

                data = {
                    'type': 'FEEDBACK',
                    'force': list(force_data)
                }
                
                msg = json.dumps(data).encode('utf-8')
                self.sock.sendto(msg, (PC_IP, PC_PORT))
                
                time.sleep(0.1) # 10Hz 回传频率
            except Exception as e:
                print(f"Feedback error: {e}")
                time.sleep(1)

    def run(self):
        print(f"[Server] 等待指令 (Port {ROBOT_PORT})...")
        # 启动回传线程
        t = threading.Thread(target=self.feedback_loop)
        t.daemon = True
        t.start()

        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                cmd = json.loads(data.decode('utf-8'))
                
                action = cmd.get('action')
                print(f"[Recv] {action}")

                if action == 'OPEN':
                    # 张开
                    pose = [255, 70, 255, 255, 255, 255, 255, 255, 255, 255]
                    self.hand.finger_move(pose)
                
                elif action == 'GRASP':
                    # 动态抓取
                    mm = cmd.get('mm', 30)
                    pose = self._calc_dynamic_pose(mm)
                    self.hand.finger_move(pose)
                
                elif action == 'FIST':
                    # 握拳
                    pose = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
                    self.hand.finger_move(pose)

            except Exception as e:
                print(f"Loop error: {e}")

if __name__ == "__main__":
    server = HandServer()
    server.run()
