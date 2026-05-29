# tools/sender.py
import socket
import json

# 配置目标 IP 和 端口
# 如果是在同一台电脑测试，用 127.0.0.1
# 如果是另一台电脑给机器人发，填机器人的局域网 IP
TARGET_IP = "127.0.0.1"
TARGET_PORT = 12345

def send_signal(key, value):
    # 1. 构造字典数据
    data = {key: value}
    
    # 2. 转为 JSON 字符串
    json_str = json.dumps(data)
    
    # 3. 通过 UDP 发送
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(json_str.encode('utf-8'), (TARGET_IP, TARGET_PORT))
    print(f"已发送数据: {json_str} -> {TARGET_IP}:{TARGET_PORT}")

if __name__ == "__main__":
    # 发送符合任务条件的信号
    send_signal("status", "arrived")
