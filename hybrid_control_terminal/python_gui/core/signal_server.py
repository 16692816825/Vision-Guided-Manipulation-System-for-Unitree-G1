#core/signal_server.py
import socket
import threading
import json
import time
class SignalServer:
    """
    外部信号监听服务 (UDP)
    职责：接收外部系统 (如 AGV 调度、PLC) 发来的 JSON 指令，
    存入内存供机器人轮询。
    code
    Code
    [更新] 引入时间戳机制，解决旧信号误触发问题。
    存储结构: self.latest_signals = { 'key': {'val': value, 'ts': timestamp} }
    """
    def __init__(self, port=12345):
        self.port = port
        self.running = False
        self.latest_signals = {} 
        self.lock = threading.Lock()
        self.socket = None
        self.thread = None

    def start(self):
        if self.running: return
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 绑定 0.0.0.0 接收所有网卡数据
            self.socket.bind(('0.0.0.0', self.port))
            self.socket.settimeout(1.0) # 设置超时方便退出循环
            
            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()
            # print(f"[SignalServer] 监听启动 (UDP Port: {self.port})")
        except Exception as e:
            print(f"[SignalServer] 启动失败: {e}")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.socket:
            self.socket.close()
        # print("[SignalServer] 服务已停止")

    def _loop(self):
        while self.running:
            try:
                # 缓冲区 1024 字节足够一般指令使用
                data, addr = self.socket.recvfrom(1024)
                text = data.decode('utf-8').strip()
                
                # 尝试解析 JSON
                try:
                    msg = json.loads(text)
                    if isinstance(msg, dict):
                        with self.lock:
                            now = time.time()
                            # 增量更新信号池，并附带时间戳
                            for k, v in msg.items():
                                self.latest_signals[k] = {'val': v, 'ts': now}
                            
                            # 记录最后更新时间 (元数据)
                            self.latest_signals['_last_update'] = {'val': now, 'ts': now}
                        
                except json.JSONDecodeError:
                    pass # 忽略非 JSON 数据
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[SignalServer] Error: {e}")
                time.sleep(1.0)

    def get_value(self, key):
        """
        [兼容旧接口] 获取某个信号的当前值 (不返回时间戳)
        """
        with self.lock:
            data = self.latest_signals.get(key)
            if data and isinstance(data, dict):
                return data.get('val')
            return None

    def get_data_with_ts(self, key):
        """
        [新接口] 获取信号及其时间戳
        返回: {'val': value, 'ts': timestamp} 或 None
        """
        with self.lock:
            return self.latest_signals.get(key)

    def inject_signal(self, key, value):
        """用于 UI 手动注入模拟信号 (带当前时间戳)"""
        with self.lock:
            self.latest_signals[key] = {'val': value, 'ts': time.time()}
            # print(f"[Signal] 手动注入: {key} = {value}")

    def clear(self):
        with self.lock:
            self.latest_signals.clear()

    def pop_signal(self, key):
        """取出并删除某个信号 (阅后即焚)"""
        with self.lock:
            if key in self.latest_signals:
                del self.latest_signals[key]
                # print(f"[Signal] 消费信号: {key}")