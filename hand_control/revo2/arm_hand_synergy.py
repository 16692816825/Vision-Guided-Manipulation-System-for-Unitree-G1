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

# --- 机器人配置 (完全采用你提供的脚本参数) ---
LEFT_ARM = [15, 16, 17, 18, 19]
OTHER_ARM = [22, 23, 24, 25, 26]
ALL_ARM = LEFT_ARM + OTHER_ARM
WEIGHT_JOINT = 29

DT = 0.02
KP = 20.0 # 保持你实测成功的 20.0
KD = 1.0

# 采用你提供的实测增量
FOLD_ARM_DELTA = {15: 1.00, 16: 0.20, 17: 0.00, 18: -1.80, 19: 0.00}
LIFT_FOLDED_ARM_DELTA = {15: -1.00, 16: 0.20, 17: 0.00, 18: -1.80, 19: 0.00}
UNFOLD_PREGRASP_DELTA = {15: -1.00, 16: 0.16, 17: 0.00, 18: -0.40, 19: 0.00}

class HandArmSynergy:
    def __init__(self):
        self.low_state = None
        self.crc = CRC()
        self.slave_id = 0x7e # 左手 ID
        self.hand_client = None

    def cb(self, msg):
        self.low_state = msg

    async def wait_state(self):
        print("[System] 等待机器人状态 (LowState)...")
        while self.low_state is None:
            await asyncio.sleep(0.1)
        self.q0 = {j: self.low_state.motor_state[j].q for j in ALL_ARM}
        self.q_fold = {j: self.q0[j] + FOLD_ARM_DELTA.get(j, 0) for j in ALL_ARM}
        self.q_lift_folded = {j: self.q0[j] + LIFT_FOLDED_ARM_DELTA.get(j, 0) for j in ALL_ARM}
        self.q_unfold_pregrasp = {j: self.q0[j] + UNFOLD_PREGRASP_DELTA.get(j, 0) for j in ALL_ARM}
        print("[System] 位姿捕获成功")

    # ================= 灵巧手控制 (优化行程) =================
    async def set_hand_safe(self, positions, speed=400):
        """
        Normalized 模式：
        Index 0 (拇指弯曲): 建议 20-400
        Index 1 (拇指旋转): 建议 20-500 (0为内旋)
        Index 2-5 (四指): 建议 20-1000
        """
        speeds = [speed] * 6
        try:
            print(f"[Hand] 写入指令: {positions}")
            await self.hand_client.set_finger_positions_and_speeds(self.slave_id, positions, speeds)
        except Exception as e:
            print(f"[Hand] 报错: {e}")

    # ================= 手臂控制 (带 Mode=1 强制模式) =================
    def write_arm_cmd(self, qmap, weight_val):
        msg = unitree_hg_msg_dds__LowCmd_()
        msg.motor_cmd[WEIGHT_JOINT].q = float(weight_val)
        
        for j in ALL_ARM:
            msg.motor_cmd[j].mode = 0x01 # 关键：确保控制权
            msg.motor_cmd[j].q = float(qmap[j])
            msg.motor_cmd[j].kp = KP
            msg.motor_cmd[j].kd = KD
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
            self.write_arm_cmd(q, w)
            await asyncio.sleep(DT)

    # ================= 核心流程 =================
    async def run(self, hand_port):
        # 1. 初始化连接
        print(f"[System] 连接灵巧手 {hand_port}...")
        self.hand_client = await libstark.modbus_open(hand_port, libstark.Baudrate.Baud460800)
        await self.hand_client.set_finger_unit_mode(self.slave_id, libstark.FingerUnitMode.Normalized)
        
        self.pub_arm = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub_arm.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.cb, 10)

        await self.wait_state()

        # --- 动作序列 ---
        
        # A. 初始位：全张开 (使用 20 避开 0 零位保护)
        # 拇指旋转设为 500 (外旋位)，四指设为 20 (张开)
        await self.set_hand_safe([20, 500, 20, 20, 20, 20])

        # B. 手臂移动阶段 (按你提供的相位)
        await self.phase_arm("enable and hold both arms", 1.0, self.q0, self.q0, 0.0, 1.0)
        await self.phase_arm("fold forearm near upper arm", 3.0, self.q0, self.q_fold, 1.0, 1.0)
        await self.phase_arm("lift and move folded arm", 6.0, self.q_fold, self.q_lift_folded, 1.0, 1.0)
        await self.phase_arm("unfold forearm to pregrasp", 4.0, self.q_lift_folded, self.q_unfold_pregrasp, 1.0, 1.0)
        
        # C. 灵巧手抓取动作 (分两步，确保能抓紧)
        print("\n=== 执行大幅度抓取动作 ===")
        # 步骤 1: 大拇指旋转到对掌位置 (0)
        print("步骤 1: 大拇指对掌...")
        await self.set_hand_safe([20, 20, 20, 20, 20, 20])
        await asyncio.sleep(1.0)
        
        # 步骤 2: 五指全力闭合
        # 拇指弯曲限制在 400，四指推到 980 满行程
        print("步骤 2: 五指全力闭合...")
        await self.set_hand_safe([400, 20, 980, 980, 980, 980])
        
        await self.phase_arm("hold pregrasp and grabbing", 4.0, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)

        # D. 复位返回
        await self.phase_arm("fold forearm back", 4.0, self.q_unfold_pregrasp, self.q_lift_folded, 1.0, 1.0)
        await self.phase_arm("shoulder back while folded", 6.0, self.q_lift_folded, self.q_fold, 1.0, 1.0)
        await self.phase_arm("unfold back to initial", 3.0, self.q_fold, self.q0, 1.0, 1.0)
        
        print("松开手指...")
        await self.set_hand_safe([20, 500, 20, 20, 20, 20])
        await asyncio.sleep(1.0)

        await self.phase_arm("release arm_sdk", 1.0, self.q0, self.q0, 1.0, 0.0)

        libstark.modbus_close(self.hand_client)
        print("\n[Done] 流程结束")

if __name__ == "__main__":
    net_if = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    usb_port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB1"
    ChannelFactoryInitialize(0, net_if)
    asyncio.run(HandArmSynergy().run(usb_port))


