import sys
import time
import asyncio

# === 引入 BrainCo 官方模块 ===
try:
    from revo2_utils import *
except ImportError:
    print("错误：请在 ~/stark-serialport-example/python/revo2/ 目录下运行")
    sys.exit(1)

# === 引入 宇树 SDK 模块 ===
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class SafeReset:
    def __init__(self, iface, usb_port):
        self.iface = iface
        self.usb_port = usb_port
        self.low_state = None
        self.crc = CRC()
        self.slave_id = 0x7e

    def cb(self, msg):
        self.low_state = msg

    async def run(self):
        # 1. 建立连接
        ChannelFactoryInitialize(0, self.iface)
        self.hand_client = await libstark.modbus_open(self.usb_port, libstark.Baudrate.Baud460800)
        
        # 强制设置模式，避免 IllegalDataValue
        await self.hand_client.set_finger_unit_mode(self.slave_id, libstark.FingerUnitMode.Normalized)
        
        self.pub_arm = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub_arm.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.cb, 10)

        print("[System] 正在读取当前手臂姿态...")
        while self.low_state is None:
            await asyncio.sleep(0.1)
        
        # 记录当前原地位置，确保手臂不动
        current_q = {j: self.low_state.motor_state[j].q for j in range(35)}

        print("[Action] 正在原地锁定手臂并张开手指 (慢速)...")
        
        # 持续发送 3 秒复位指令
        for i in range(150):
            # A. 锁定手臂：发送当前实时读到的位置
            msg = unitree_hg_msg_dds__LowCmd_()
            for j in range(35):
                msg.motor_cmd[j].mode = 0x01
                msg.motor_cmd[j].q = current_q[j]
                msg.motor_cmd[j].kp = 20.0
                msg.motor_cmd[j].kd = 1.0
            
            msg.motor_cmd[29].q = 1.0 # 保持灵巧手供电
            msg.crc = self.crc.Crc(msg)
            self.pub_arm.Write(msg)

            # B. 缓慢张开手指
            # 使用 500 这个中值尝试唤醒，避开 0 或 1000 可能触发的非法值报错
            if i % 10 == 0:
                try:
                    # 分步发送，速度设为极慢的 100
                    await self.hand_client.set_finger_speeds(self.slave_id, [100]*6)
                    await asyncio.sleep(0.01)
                    # 尝试回复到中间位置 500，然后再完全张开
                    await self.hand_client.set_finger_positions(self.slave_id, [500, 500, 500, 500, 500, 500])
                except:
                    pass
            
            await asyncio.sleep(0.02)

        # 最后尝试全张开
        print("[Action] 尝试完全张开...")
        try:
            await self.hand_client.set_finger_positions(self.slave_id, [0, 0, 0, 0, 0, 0])
        except:
            pass
            
        libstark.modbus_close(self.hand_client)
        print("[Done] 复位尝试结束。请检查手指状态。")

if __name__ == "__main__":
    asyncio.run(SafeReset("eth0", "/dev/ttyUSB1").run())
