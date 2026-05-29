import time
import sys
# 引入 BrainCo 官方 SDK
from bc_stark_sdk import SerialDevice, Revo2

def test_hand(port_name):
    print(f"正在尝试连接灵巧手 (端口: {port_name})...")
    
    try:
        # 初始化串口，BrainCo 默认波特率通常是 115200
        dev = SerialDevice(port_name, baudrate=115200)
        hand = Revo2(dev)
        print("✅ 连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        print("请检查端口号是否正确，或者是否有 sudo 权限 (可以尝试加 sudo 运行脚本)")
        return

    print("\n--- 开始抓取测试 ---")
    
    # 1. 完全张开 (ID字典: 0拇指弯曲, 1拇指旋转, 2食指, 3中指, 4无名指, 5小指)
    # BrainCo 原生协议中，通常 0 是张开，100 是闭合 (具体取决于出厂标定)
    print("动作：张开手掌")
    hand.set_positions({0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0})
    time.sleep(2)
    
    # 2. 拇指对掌
    print("动作：大拇指对掌")
    hand.set_positions({1: 100}) 
    time.sleep(1)
    
    # 3. 四指和拇指收拢抓水瓶 (设为 70% 防止捏爆)
    print("动作：抓取水瓶")
    hand.set_positions({0: 70, 2: 70, 3: 70, 4: 70, 5: 70})
    time.sleep(3)
    
    # 4. 松开
    print("动作：松开水瓶")
    hand.set_positions({0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0})
    print("--- 测试结束 ---")

if __name__ == "__main__":
    # 默认尝试 ttyUSB0，如果不行可以通过命令行传 ttyUSB1, ttyUSB2 等
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    test_hand(port)
