import sys
import time
import asyncio

# === 引入 BrainCo 官方模块 ===
try:
    from revo2_utils import *
except ImportError:
    print("错误：请在 ~/stark-serialport-example/python/revo2/ 目录下运行此脚本")
    sys.exit(1)

# === 引入 宇树 SDK 模块 ===
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# 机器人关节定义
LEFT_ARM = [15, 16, 17, 18, 19]
ALL_ARM = LEFT_ARM + [22, 23, 24, 25, 26]
WEIGHT_JOINT = 29 # 灵巧手 24V 动力使能关键关节

DT = 0.02
KP_ARM = 30.0 # 适中刚度
KD_ARM = 1.0

# 采用你提供的实测增量
FOLD_ARM_DELTA = {15: 1.00, 16: 0.20, 17: 0.00, 18: -1.80, 19: 0.00}
LIFT_FOLDED_ARM_DELTA = {15: -1.00, 16: 0.20, 17: 0.00, 18: -1.80, 19: 0.00}
UNFOLD_PREGRASP_DELTA = {15: -1.00, 16: 0.16, 17: 0.00, 18: -0.40, 19: 0.00}

class HandArmSynergyFinal:
    def __init__(self):
        self.low_state = None
        self.crc = CRC()
        self.slave_id = 0x7e # 左手 ID
        self.hand_client = None

    def cb(self, msg):
        self.low_state = msg

    async def wait_state(self):
        print("[System] 正在等待 LowState 数据...")
        while self.low_state is None:
            await asyncio.sleep(0.1)
        self.q0 = {j: self.low_state.motor_state[j].q for j in ALL_ARM}
        self.q_fold = {j: self.q0[j] + FOLD_ARM_DELTA.get(j, 0) for j in ALL_ARM}
        self.q_lift_folded = {j: self.q0[j] + LIFT_FOLDED_ARM_DELTA.get(j, 0) for j in ALL_ARM}
        self.q_unfold_pregrasp = {j: self.q0[j] + UNFOLD_PREGRASP_DELTA.get(j, 0) for j in ALL_ARM}
        print("[System] 位姿捕获成功")

    # ================= 灵巧手控制：分步写入绕过 SDK Bug =================
    async def set_hand_robust(self, positions, speed=600):
        """
        分步发送速度和位置，确保所有 6 个手指都能收到指令。
        Normalized 模式：0-1000。
        """
        try:
            # 1. 设置 6 指速度 (寄存器 941-946)
            await self.hand_client.set_finger_speeds(self.slave_id, [speed]*6)
            await asyncio.sleep(0.05)

            # 2. 设置 6 指位置 (寄存器 931-936)
            await self.hand_client.set_finger_positions(self.slave_id, positions)

            # 3. 读取反馈，观察后三位是否从 0 变为非零
            fb = await self.hand_client.get_finger_positions(self.slave_id)
            print(f"[Hand Status] 指令: {positions} | 实时位置反馈: {fb}")
        except Exception as e:
            print(f"[Hand Error] 控制异常: {e}")

    # ================= 手臂控制：锁定动力电 =================
    def write_arm(self, qmap, pwr_val=1.0):
        msg = unitree_hg_msg_dds__LowCmd_()
        # 强制开启灵巧手动力电源 (WEIGHT关节)
        msg.motor_cmd[WEIGHT_JOINT].q = float(pwr_val)
        msg.motor_cmd[WEIGHT_JOINT].kp = 10.0
        msg.motor_cmd[WEIGHT_JOINT].kd = 1.0

        for j in ALL_ARM:
            msg.motor_cmd[j].mode = 0x01 # 关键：确保控制权
            msg.motor_cmd[j].q = float(qmap[j])
            msg.motor_cmd[j].kp = KP_ARM
            msg.motor_cmd[j].kd = KD_ARM
            msg.motor_cmd[j].tau = 0.0
            msg.motor_cmd[j].dq = 0.0

        msg.crc = self.crc.Crc(msg)
        self.pub_arm.Write(msg)

    async def phase_arm(self, name, seconds, a, b, wa, wb):
        print(f"[Arm] 执行相位: {name}")
        steps = int(seconds / DT)
        for i in range(steps):
            r = (i + 1) / steps
            q = {j: (1-r)*a[j] + r*b[j] for j in ALL_ARM}
            w = (1-r)*wa + r*wb
            self.write_arm(q, w)
            await asyncio.sleep(DT)

    # ================= 业务主逻辑 =================
    async def run(self, hand_port):
        # 1. 硬件连接
        self.hand_client = await libstark.modbus_open(hand_port, libstark.Baudrate.Baud460800)
        await self.hand_client.set_finger_unit_mode(self.slave_id, libstark.FingerUnitMode.Normalized)

        self.pub_arm = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub_arm.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.cb, 10)
        await self.wait_state()

        # --- 执行动作 ---

        # A. 初始位：全力张开 (Normalized 模式 0 为张开)
        # 注意：如果发现手指动作反向，请将 0 和 1000 对调
        print("\n[Action] 动力上电并初始化位置...")
        await self.set_hand_robust([0, 0, 0, 0, 0, 0])
        await self.phase_arm("Initial Power On", 1.0, self.q0, self.q0, 0.0, 1.0)

        # B. 手臂移动阶段
        await self.phase_arm("fold arm", 3.0, self.q0, self.q_fold, 1.0, 1.0)
        await self.phase_arm("lift arm", 5.0, self.q_fold, self.q_lift_folded, 1.0, 1.0)
        await self.phase_arm("unfold to pregrasp", 4.0, self.q_lift_folded, self.q_unfold_pregrasp, 1.0, 1.0)

        # C. 灵巧手闭合抓取 (使用 1000 满行程)
        print("\n=== 执行深度抓取动作 ===")
        # 步骤 1: 大拇指旋转对掌 (Index 1)
        await self.set_hand_robust([0, 1000, 0, 0, 0, 0])
        await asyncio.sleep(1.0)
        # 步骤 2: 五指闭合
        await self.set_hand_robust([1000, 1000, 1000, 1000, 1000, 1000])
        await asyncio.sleep(3.0)

        # D. 复位返回
        await self.phase_arm("fold back", 4.0, self.q_unfold_pregrasp, self.q_lift_folded, 1.0, 1.0)
        print("松开手指...")
        await self.set_hand_robust([0, 0, 0, 0, 0, 0])
        await self.phase_arm("unfold back to initial", 3.0, self.q_fold, self.q0, 1.0, 1.0)

        libstark.modbus_close(self.hand_client)
        print("\n[Done] 全部流程完成。")

if __name__ == "__main__":
    net = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    usb = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB1"

    ChannelFactoryInitialize(0, net)
    try:
        asyncio.run(HandArmSynergyFinal().run(usb))
    except KeyboardInterrupt:
        pass


