# core/l10_client.py
import socket
import json
import threading
import time
from core.settings_manager import settings
class L10Client:
    # [修改] __init__ 方法，移除默认参数中的硬编码 IP
    def __init__(self, robot_ip=None, robot_port=9999, local_port=None):
        
        # [逻辑] 优先使用传入参数，如果没有传入，则读取配置文件
        if robot_ip is None:
            robot_ip = settings.get("network", "robot_ip")
        
        if local_port is None:
            local_port = settings.get("network", "local_port")
            
        self.robot_addr = (robot_ip, robot_port)
        self.latest_force = [0, 0, 0, 0, 0]
        self.latest_matrix = []
        self.running = True
        
        print(f"[DEBUG] L10Client 正在启动... 目标机器人: {robot_ip}")
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 允许端口复用
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", local_port))
            self.sock.setblocking(False) 
            print(f"[DEBUG] L10Client 成功绑定本地端口: {local_port}")
        except Exception as e:
            print(f"[FATAL ERROR] L10Client 端口绑定失败！！！原因: {e}")
            self.running = False
            return

        self.t = threading.Thread(target=self._recv_loop)
        self.t.daemon = True
        self.t.start()

    def _recv_loop(self):
        print("[DEBUG] 接收线程已运行，等待数据...")
        last_print = 0
        while self.running:
            try:
                # 缓冲区 4096
                data, addr = self.sock.recvfrom(4096)
                
                # --- 调试点 1: 只要收到字节流就打印 (每秒打印一次防止刷屏) ---
                if time.time() - last_print > 1.0:
                    last_print = time.time()

                msg = json.loads(data.decode('utf-8'))
                if msg.get('type') == 'FEEDBACK':
                    self.latest_force = msg.get('force', [])
                    self.latest_matrix = msg.get('matrix', [])
                    
            except BlockingIOError:
                time.sleep(0.005)
            except Exception as e:
                print(f"[DEBUG] 数据解析错误: {e}")
                time.sleep(0.1)

    def get_data(self):
        return self.latest_force, self.latest_matrix

    def send_cmd(self, action, **kwargs):
        payload = {'action': action}
        payload.update(kwargs)
        try:
            data = json.dumps(payload).encode('utf-8')
            self.sock.sendto(data, self.robot_addr)
            print(f"[DEBUG] 发送指令: {action}")
        except Exception as e:
            print(f"[DEBUG] 发送失败: {e}")

    def grasp(self, diameter_mm): self.send_cmd('GRASP', mm=diameter_mm)
    def open(self): self.send_cmd('OPEN')
    def get_pressure(self): return self.latest_force
    def close(self): self.running = False
