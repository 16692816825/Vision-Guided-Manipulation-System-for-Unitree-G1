#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 手臂控制演示脚本 — 手臂抬起 → 保持 → 放下 → 释放 arm_sdk 控制权。

通过 unitree_sdk2py 的 DDS 通道直接控制，不依赖 ROS2。

运行：
    python3 arm.py              # 自动检测网络接口
    python3 arm.py eth0         # 指定网络接口
"""

import math
import time
import sys
import threading
from typing import List

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

from arm_common import G1JointIndex, ARM_JOINTS, JOINT_LIMITS

_CONTROL_DT = 0.02
_TRANSITION_DUR = 3.0
_DEFAULT_KP = 60.0
_DEFAULT_KD = 1.5
_STATE_TIMEOUT = 10.0


class ArmDemoController:
    """G1 手臂控制演示 — 四阶段运动序列。"""

    def __init__(self) -> None:
        self.time_ = 0.0
        self.control_dt_ = _CONTROL_DT
        self.duration_ = _TRANSITION_DUR

        self.kp = _DEFAULT_KP
        self.kd = _DEFAULT_KD

        self._state_lock = threading.Lock()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state: LowState_ = None

        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False

        self.target_pos = [
            0.0,            math.pi / 2,  0.0,  math.pi / 2,  0.0,
            0.0,           -math.pi / 2,  0.0,  math.pi / 2,  0.0,
            0.0,            0.0,          0.0,
        ]

    def _clip_joint(self, joint: int, value: float) -> float:
        lo, hi = JOINT_LIMITS.get(joint, (-math.pi, math.pi))
        return float(np.clip(value, lo, hi))

    def _apply_joint_pd(self, targets: List[float], ratio: float) -> None:
        """对所有受控关节应用 PD 位置控制（插值到目标）。"""
        for i, joint in enumerate(ARM_JOINTS):
            with self._state_lock:
                cur_q = self.low_state.motor_state[joint].q
            self.low_cmd.motor_cmd[joint].q = self._clip_joint(joint, ratio * targets[i] + (1.0 - ratio) * cur_q)
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

    def _apply_current_pd(self, ratio: float) -> None:
        """对所有受控关节应用 PD 控制（插值到零位）。"""
        for i, joint in enumerate(ARM_JOINTS):
            with self._state_lock:
                cur_q = self.low_state.motor_state[joint].q
            self.low_cmd.motor_cmd[joint].q = self._clip_joint(joint, (1.0 - ratio) * cur_q)
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

    def Init(self) -> None:
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self) -> None:
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        _t0 = time.time()
        while not self.first_update_low_state:
            if time.time() - _t0 > _STATE_TIMEOUT:
                raise TimeoutError("未收到关节状态消息，请检查 DDS 通信和网络接口")
            time.sleep(0.1)

        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_) -> None:
        with self._state_lock:
            self.low_state = msg
        if not self.first_update_low_state:
            self.first_update_low_state = True

    def LowCmdWrite(self) -> None:
        try:
            with self._state_lock:
                if self.low_state is None:
                    return

            self.time_ += self.control_dt_

            if self.time_ < self.duration_:
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
                self._apply_current_pd(ratio)

            elif self.time_ < self.duration_ * 3:
                ratio = np.clip((self.time_ - self.duration_) / (self.duration_ * 2), 0.0, 1.0)
                self._apply_joint_pd(self.target_pos, ratio)

            elif self.time_ < self.duration_ * 6:
                ratio = np.clip((self.time_ - self.duration_ * 3) / (self.duration_ * 3), 0.0, 1.0)
                self._apply_current_pd(ratio)

            elif self.time_ < self.duration_ * 7:
                ratio = np.clip((self.time_ - self.duration_ * 6) / self.duration_, 0.0, 1.0)
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1 - ratio

            else:
                self.done = True

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
        except Exception as e:
            print(f"[arm] 控制循环异常: {e}")


if __name__ == '__main__':
    print("请确保机器人周围无障碍物。")
    input("按 Enter 继续...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    ctrl = ArmDemoController()
    ctrl.Init()
    ctrl.Start()

    while True:
        time.sleep(1)
        if ctrl.done:
            print("完成!")
            sys.exit(0)
