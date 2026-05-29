#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 上肢动作调试模块 (arm_debug.py)
==========================================
独立 DDS 进程，通过 stdin 行协议接收目标角度并平滑插值执行。

协议（每行一个 JSON）：
  {"angles": [13个浮点数]}  — 设置新目标角度（弧度）
  {"stop": true}                — 停止调试，平滑归零并释放 arm_sdk

运行方式：
  web_panel.py 通过 subprocess.Popen 启动本脚本，
  不导入 unitree_sdk2py 到 ROS2 主进程，避免 DDS 冲突。

依赖：
  pip install unitree-sdk2py numpy
"""

import math
import sys
import time
import json
import threading

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# 从 arm_common 导入共享常量
from arm_common import G1JointIndex, ARM_JOINTS, JOINT_LIMITS


# ---------- 默认归零姿态 ----------
DEFUALT_POSE = [0.0] * len(ARM_JOINTS)


class ArmDebugController:
    """上肢调试控制器 — 50Hz 平滑插值到目标角度。"""

    def __init__(self, control_dt: float = 0.02, kp: float = 60.0, kd: float = 1.5):
        self.control_dt = control_dt
        self.kp = kp
        self.kd = kd

        self._state_lock = threading.Lock()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

        self.first_low_state = False
        self.done = False

        # 当前角度 & 目标角度
        self._current_angles = list(DEFUALT_POSE)
        self._target_angles = list(DEFUALT_POSE)
        self._target_event = threading.Event()

        # 控制线程
        self._thread = None
        self._running = False

    # ---- DDS 初始化 ----
    def init_dds(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._state_handler, 10)

    def start_dds(self):
        self._thread = RecurrentThread(
            interval=self.control_dt, target=self._control_loop, name="arm_debug"
        )
        # 等待第一次关节状态
        _t0 = time.time()
        while not self.first_low_state:
            if time.time() - _t0 > 10.0:
                raise TimeoutError("未收到关节状态消息，请检查 DDS 通信和网络接口")
            time.sleep(0.1)
        # 读取当前真实角度作为起点
        with self._state_lock:
            if self.low_state is not None:
                self._current_angles = [
                    float(self.low_state.motor_state[j].q) for j in ARM_JOINTS
                ]
                self._target_angles = list(self._current_angles)
        self._running = True
        self._thread.Start()
        print("[arm_debug] DDS 已启动，当前角度已读取", file=sys.stderr)

    def stop_and_zero(self):
        """平滑归零并释放 arm_sdk。"""
        self._target_angles = list(DEFUALT_POSE)
        self._target_event.set()
        # 等待归零完成（最多 5 秒）
        _t0 = time.time()
        while self._running and time.time() - _t0 < 5.0:
            time.sleep(0.05)
        self._running = False
        if self._thread:
            # RecurrentThread 没有 stop，直接标记退出
            pass
        print("[arm_debug] 已归零并停止", file=sys.stderr)

    # ---- 内部方法 ----
    def _state_handler(self, msg: LowState_) -> None:
        with self._state_lock:
            self.low_state = msg
        if not self.first_low_state:
            self.first_low_state = True

    def _control_loop(self) -> None:
        if not self._running:
            return
        try:
            with self._state_lock:
                if self.low_state is None:
                    return

            # 平滑插值：每次循环移动剩余距离的 30%（低通效果）
            alpha = 0.3
            for i, joint in enumerate(ARM_JOINTS):
                cur = self._current_angles[i]
                tgt = self._target_angles[i]
                new_val = cur + alpha * (tgt - cur)
                lo, hi = JOINT_LIMITS.get(joint, (-math.pi, math.pi))
                new_val = float(np.clip(new_val, lo, hi))
                self._current_angles[i] = new_val

                self.low_cmd.motor_cmd[joint].q = new_val
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

            # 启用 arm_sdk
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)

        except Exception as e:
            print(f"[arm_debug] 控制循环异常: {e}", file=sys.stderr)

    def set_target(self, angles: list) -> None:
        """设置新目标角度（弧度）。"""
        if len(angles) != len(ARM_JOINTS):
            print(f"[arm_debug] 角度数量错误：期望 {len(ARM_JOINTS)}，收到 {len(angles)}", file=sys.stderr)
            return
        self._target_angles = [float(a) for a in angles]
        self._target_event.set()
        print(f"[arm_debug] 新目标: {self._target_angles}", file=sys.stderr)


# ---------- 预设姿势 ----------
def pose_reach_forward():
    return [
        -0.8,  0.5, -0.4,  0.15, -1.8,
        -0.8, -0.5,  0.4,  0.15,  1.8,
         0.0,  0.0,  0.0,
    ]

def pose_arms_up():
    return [
        -1.0,  0.7,  0.0,  0.6, -0.8,
        -1.0, -0.7,  0.0,  0.6,  0.8,
         0.0,  0.0,  0.0,
    ]

def pose_pray():
    return [
        -1.15, 0.5, -0.3,  0.3, -1.8,
        -1.15, -0.5,  0.3,  0.3,  1.8,
         0.0,  0.0,  0.0,
    ]

def pose_wave():
    return [
        -1.1,  0.55, -0.45,  0.2, -1.8,
        -1.1, -0.55,  0.45,  0.2,  1.8,
         0.0,  0.0,   0.0,
    ]

def pose_wave_body():
    return [
        -0.7,  0.7,  0.0,  0.6, -0.8,
        -0.7, -0.7,  0.0,  0.6,  0.8,
         0.0,  0.0,  0.0,
    ]

PRESETS = {
    "reach_forward": ("伸手接近", pose_reach_forward()),
    "arms_up":        ("抬起目标", pose_arms_up()),
    "pray":           ("夹紧保持", pose_pray()),
    "wave":           ("伸展下放", pose_wave()),
    "wave_body":      ("自然下垂", pose_wave_body()),
}


# ---------- 主循环 ----------
def main():
    # 解析网络接口参数
    iface = sys.argv[1] if len(sys.argv) > 1 else None
    if iface:
        ChannelFactoryInitialize(0, iface)
        print(f"[arm_debug] DDS 初始化: 网络接口={iface}", file=sys.stderr)
    else:
        ChannelFactoryInitialize(0)
        print("[arm_debug] DDS 初始化: 自动检测网络接口", file=sys.stderr)

    ctrl = ArmDebugController()
    ctrl.init_dds()
    ctrl.start_dds()

    print("[arm_debug] 就绪，等待指令...", file=sys.stderr)
    sys.stdout.flush()

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print(f"[arm_debug] JSON 解析失败: {line}", file=sys.stderr)
                continue

            if msg.get("stop"):
                print("[arm_debug] 收到停止指令", file=sys.stderr)
                ctrl.stop_and_zero()
                break

            angles = msg.get("angles")
            if angles is not None:
                ctrl.set_target(angles)

    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        ctrl.stop_and_zero()
        print("[arm_debug] 进程退出", file=sys.stderr)


if __name__ == "__main__":
    main()
