#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 人形机器人手臂多姿态演示
===========================
在 arm.py 基础上扩展，支持定义多个目标姿态，手臂在姿态之间平滑循环切换。

预设姿态：
  1. 双臂抬起（T-pose 变体）
  2. 双手合十（祈祷姿势）
  3. 挥手（右臂举起左右摆动）
  4. 双臂下垂归零

运行方式：
    python3 src/arm_multi_pose_demo.py              # 自动检测网络接口
    python3 src/arm_multi_pose_demo.py eth0         # 指定网络接口

依赖：
    pip install unitree_sdk2py numpy
"""

import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

import numpy as np

kPi = 3.141592654
kPi_2 = 1.57079632


class G1JointIndex:
    """G1 机器人关节索引常量（0-29）。"""
    # 左腿 (0-5)
    LeftHipPitch = 0;  LeftHipRoll = 1;  LeftHipYaw = 2
    LeftKnee = 3;  LeftAnklePitch = 4;  LeftAnkleRoll = 5
    # 右腿 (6-11)
    RightHipPitch = 6;  RightHipRoll = 7;  RightHipYaw = 8
    RightKnee = 9;  RightAnklePitch = 10;  RightAnkleRoll = 11
    # 腰部 (12-14)
    WaistYaw = 12;  WaistRoll = 13;  WaistPitch = 14
    # 左臂 (15-21)
    LeftShoulderPitch = 15;  LeftShoulderRoll = 16;  LeftShoulderYaw = 17
    LeftElbow = 18;  LeftWristRoll = 19;  LeftWristPitch = 20;  LeftWristYaw = 21
    # 右臂 (22-28)
    RightShoulderPitch = 22;  RightShoulderRoll = 23;  RightShoulderYaw = 24
    RightElbow = 25;  RightWristRoll = 26;  RightWristPitch = 27;  RightWristYaw = 28
    # 特殊索引
    kNotUsedJoint = 29  # arm_sdk 使能控制：q=1 启用, q=0 释放


# ======================================================================
# 关节索引顺序（与目标角度数组一一对应）
# ======================================================================
ARM_JOINTS = [
    G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.WaistYaw,
    G1JointIndex.WaistRoll,
    G1JointIndex.WaistPitch,
]

# ======================================================================
# 预设姿态定义
# ======================================================================
# 每个姿态是 13 个弧度值的列表，顺序同 ARM_JOINTS
# 布局：[左臂5, 右臂5, 腰部3]

def _pose_zeros():
    """零位：所有受控关节归零。"""
    return [0.0] * 13


def _pose_arms_up():
    """姿态 1："1"
    """
    return [
            -0.7,      0.7,  0.0,    0.5,  -0.8,
            -0.7,     -0.7,  0.0,    0.5,  0.8,
            0.0,      0.0,    0.0
    ]


def _pose_pray():
    """姿态 2："2"
    """
    k3_4_pi = kPi * 0.75
    return [
           -0.7,      0.65,  -0.23,   0.0,  -1.2,
           -0.7,     -0.65,   0.23,   0.0,   1.2,
            0.0,      0.0,    0.0
    ]


def _pose_wave():
    """姿态 3:"3"
    """
    return [
        -0.5,   0.6,   -0.25,    -0.45,  -1.2,
        -0.5,   -0.6,   0.25,    -0.45,   1.2, 
         0.0,   0.0,     0.0,
    ]


def _pose_reach_forward():
    """姿态 4："4"
    """
    return [
           -0.6,      0.58,   -0.2,   0.05,  -1.2,
           -0.6,     -0.58,    0.2,   0.05,   1.2,
            0.0,      0.0,    0.0
            ]
def _pose_wave_body():            
    """姿态 5 :"5"
    """
    return [
            -0.7,      0.7,    0.0,    0.6,  -0.8,
            -0.7,     -0.7,    0.0,    0.6,  0.8,
             0.0,      0.0,    0.0
    ]


# 姿态列表：名称 + 角度数组 + 保持时间(秒)
POSE_SEQUENCE = [
    ("1",      _pose_arms_up(),         2.0),
    ("2",      _pose_pray(),            1.0),
    ("3",      _pose_wave(),            2.0),
    ("4",      _pose_reach_forward(),   2.0),
    ("5",      _pose_wave_body(),       2.0),
]


# ======================================================================
# 多姿态控制器
# ======================================================================
class MultiPoseController:
    """G1 手臂多姿态循环控制器。

    控制流程：
    1. 初始化：归零并启用手臂 SDK
    2. 循环：依次切换到每个预设姿态 → 保持 → 切换下一个
    3. 结束：所有姿态演示完毕后归零并释放 arm_sdk

    每个姿态之间通过线性插值平滑过渡，过渡时间由 transition_time 控制。
    """

    def __init__(self, poses, transition_time=2.0, kp=60.0, kd=1.5):
        """
        Args:
            poses: 姿态列表，每项为 (名称, 角度数组, 保持时间)
            transition_time: 姿态之间过渡时间（秒）
            kp: 位置控制比例增益
            kd: 位置控制阻尼增益
        """
        self.poses = poses
        self.transition_time = transition_time
        self.kp = kp
        self.kd = kd
        self.control_dt = 0.02   # 50Hz

        # 运行时状态
        self.time = 0.0
        self.phase = 0           # 当前执行阶段索引
        self.done = False
        self.first_low_state = False

        # DDS 对象
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

        # 计算阶段时间表
        # 阶段 0: 初始归零 (transition_time)
        # 阶段 1+2i-1: 过渡到第i个姿态 (transition_time)
        # 阶段 1+2i: 保持第i个姿态 (hold_time)
        # 最后: 归零 + 释放
        self._build_timeline()

    def _build_timeline(self):
        """构建阶段时间表：每阶段 (起始时间, 结束时间, 阶段类型, 数据)。"""
        self.timeline = []
        t = 0.0

        # 阶段 0: 初始归零
        t_end = t + self.transition_time
        self.timeline.append((t, t_end, "init_zero", None))
        t = t_end

        # 逐个姿态：过渡 + 保持
        for name, angles, hold_time in self.poses:
            # 过渡到目标姿态
            t_end = t + self.transition_time
            self.timeline.append((t, t_end, "transition", (name, angles)))
            t = t_end
            # 保持目标姿态
            t_end = t + hold_time
            self.timeline.append((t, t_end, "hold", (name, angles)))
            t = t_end

        # 最后归零
        t_end = t + self.transition_time
        self.timeline.append((t, t_end, "final_zero", None))
        t = t_end

        # 释放 arm_sdk
        t_end = t + self.transition_time
        self.timeline.append((t, t_end, "release", None))
        t = t_end

        self.total_time = t

    def Init(self):
        """初始化 DDS 通信通道。"""
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._state_handler, 10)

    def Start(self):
        """等待关节状态就绪后启动控制线程。"""
        self.thread = RecurrentThread(
            interval=self.control_dt, target=self._control_loop, name="multi_pose"
        )
        while not self.first_low_state:
            time.sleep(0.1)
        self.thread.Start()

    def _state_handler(self, msg):
        """关节状态回调。"""
        self.low_state = msg
        if not self.first_low_state:
            self.first_low_state = True

    def _get_current_joint_angles(self):
        """从 lowstate 读取当前手臂关节角度。"""
        return [float(self.low_state.motor_state[j].q) for j in ARM_JOINTS]

    def _send_joint_cmd(self, target_angles):
        """向所有受控关节发送 PD 位置控制指令。"""
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1  # 启用 arm_sdk
        for i, joint in enumerate(ARM_JOINTS):
            self.low_cmd.motor_cmd[joint].q = target_angles[i]
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

    def _control_loop(self):
        """50Hz 控制回调 — 根据时间表执行当前阶段。"""
        self.time += self.control_dt

        if self.time >= self.total_time:
            self.done = True
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)
            return

        # 查找当前阶段
        phase_name = ""
        for t_start, t_end, ptype, pdata in self.timeline:
            if t_start <= self.time < t_end:
                phase_name = ptype
                ratio = np.clip((self.time - t_start) / (t_end - t_start), 0.0, 1.0)
                current = self._get_current_joint_angles()

                if ptype == "init_zero":
                    # 从当前位置归零
                    target = [(1.0 - ratio) * c for c in current]
                    self._send_joint_cmd(target)

                elif ptype == "transition":
                    # 从当前位置过渡到目标姿态
                    name, target_angles = pdata
                    target = [
                        ratio * t + (1.0 - ratio) * c
                        for t, c in zip(target_angles, current)
                    ]
                    self._send_joint_cmd(target)
                    # 首次进入该阶段时打印
                    if abs(self.time - t_start) < self.control_dt * 1.5:
                        self._log_pose(name, target_angles)

                elif ptype == "hold":
                    # 保持目标姿态不变
                    _, target_angles = pdata
                    self._send_joint_cmd(target_angles)

                elif ptype == "final_zero":
                    # 最终归零
                    target = [(1.0 - ratio) * c for c in current]
                    self._send_joint_cmd(target)
                    if abs(self.time - t_start) < self.control_dt * 1.5:
                        print("\n  [归零]")

                elif ptype == "release":
                    # 释放 arm_sdk 控制权
                    enable = 1.0 - ratio
                    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = enable
                    self._send_joint_cmd(current)
                    if abs(self.time - t_start) < self.control_dt * 1.5:
                        print("\n  [释放 arm_sdk]")

                break

        # CRC 校验 + 发送
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def _log_pose(self, name, _angles):
        """打印姿态切换信息。"""
        print(f"\n  >>> {name}")


# ======================================================================
# 主入口
# ======================================================================
if __name__ == '__main__':

    input("Press Enter to continue...")

    # 初始化 DDS
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # 创建控制器并启动
    ctrl = MultiPoseController(POSE_SEQUENCE)
    ctrl.Init()
    ctrl.Start()

    start = time.time()
    while True:
        time.sleep(1)
        elapsed = time.time() - start
        progress = min(ctrl.time / ctrl.total_time * 100, 100) if ctrl.total_time > 0 else 0
        print(f"\r  进度: {progress:5.1f}%  ({elapsed:.0f}s/{ctrl.total_time:.0f}s)", end="", flush=True)

        if ctrl.done:
            total = time.time() - start
            print(f"\r  进度: 100.0%  ({total:.0f}s/{ctrl.total_time:.0f}s)")
            print("\n  Done!")
            sys.exit(0)
