import sys
import asyncio

# === 引入模块 ===
try:
    from revo2_utils import *
except ImportError:
    print("错误：请在 ~/stark-serialport-example/python/revo2/ 目录下运行此脚本")
    sys.exit(1)

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class HandPowerOff:
    def __init__(self, iface, usb_port):
        self.iface = iface
        self.usb_port = usb_port
        self.low_state = None
        self.crc = CRC()
        self.slave_id = 0x7e

    def cb(self, msg):
        self.low_state = msg

    async def run(self):
        ChannelFactoryInitialize(0, self.iface)
        # 尝试关闭串口连接
        self.hand_client = await libstark.modbus_open(self.usb_port, libstark.Baudrate.Baud460800)
        
        self.pub_arm = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub_arm.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.cb, 10)

        print("[System] 正在捕获位置以锁定手臂...")
        while self.low_state is None:
            await asyncio.sleep(0.1)
        
        lock_q = {j: self.low_state.motor_state[j].q for j in range(35)}

        print("[Action] 正在切断灵巧手动力电源 (WEIGHT -> 0.0)...")

        # 持续发送 2 秒断电指令
        for _ in range(100):
            msg = unitree_hg_msg_dds__LowCmd_()
            for j in range(35):
                msg.motor_cmd[j].mode = 0x01
                msg.motor_cmd[j].q = lock_q[j] # 锁定当前位置
                msg.motor_cmd[j].kp = 20.0
                msg.motor_cmd[j].kd = 1.0
            
            # 【核心】将 29 号关节设为 0，切断末端 24V 供电
            msg.motor_cmd[29].q = 0.0 
            msg.motor_cmd[29].kp = 10.0
            msg.motor_cmd[29].kd = 1.0
            
            msg.crc = self.crc.Crc(msg)
            self.pub_arm.Write(msg)
            await asyncio.sleep(0.02)

        # 尝试发送软件释放指令
        try:
            await self.hand_client.set_finger_enables(self.slave_id, [False]*6)
            print("[Hand] 已发送软件去使能指令。")
        except:
            pass

        libstark.modbus_close(self.hand_client)
        print("[Done] 手臂已锁定，动力电已切断。手指现在应该处于自然松弛状态。")

if __name__ == "__main__":
    asyncio.run(HandPowerOff("eth0", "/dev/ttyUSB1").run())
