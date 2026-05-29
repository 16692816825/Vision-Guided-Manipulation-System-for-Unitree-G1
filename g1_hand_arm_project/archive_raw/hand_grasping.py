import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize

# 导入宇树的消息类型
try:
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmds_, MotorCmd_
except ImportError:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorCmd_

class RevoHandDDSTester:
    def __init__(self):
        # 初始化发往左手的指令话题
        self.pub = ChannelPublisher("rt/brainco/left/cmd", MotorCmds_)
        self.pub.Init()

    def set_hand(self, positions):
        """
        positions: [大拇指, 大拇指旋转(对掌), 食指, 中指, 无名指, 小拇指]
        范围均在 0.0 (全开) ~ 1.0 (全关) 之间
        """
        msg = MotorCmds_()
        cmds = []
        
        for i in range(6):
            # 严格按照 IDL 要求的 7 个参数按顺序实例化：
            # mode(1:伺服), q(位置), dq(速度), tau(前馈力矩), kp, kd, reserve
            cmd = MotorCmd_(
                1, float(positions[i]), 1.0, 0.0, 0.0, 0.0, [0, 0, 0]
            )
            cmds.append(cmd)
            
        msg.cmds = cmds
        self.pub.Write(msg)
        print(f"[Hand] 已发送灵巧手指令: {positions}")

    def grasp_bottle(self):
        print("--- 开始测试 ---")
        # 1. 初始状态：全开
        self.set_hand([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(2)

        # 2. 准备抓取：大拇指旋转到掌心对面（对掌姿态）
        print("动作：大拇指对掌...")
        self.set_hand([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(1.5)

        # 3. 圆柱握紧（抓水瓶）：四指和大拇指同时弯曲
        print("动作：四指与大拇指收拢，抓取水瓶...")
        self.set_hand([0.7, 1.0, 0.7, 0.7, 0.7, 0.7])
        time.sleep(3)

        # 4. 松开水瓶
        print("动作：松开水瓶...")
        self.set_hand([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(1.5)
        print("--- 测试结束 ---")

if __name__ == "__main__":
    net_iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    print(f"正在使用网卡 {net_iface} 初始化 DDS...")
    
    ChannelFactoryInitialize(0, net_iface)
    tester = RevoHandDDSTester()
    
    # 稍微等一下 DDS 发现节点
    time.sleep(1.0) 
    tester.grasp_bottle()


