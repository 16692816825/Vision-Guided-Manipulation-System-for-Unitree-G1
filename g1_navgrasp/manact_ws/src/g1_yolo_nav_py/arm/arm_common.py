"""G1 手臂控制公共模块 — 共享常量和基类。

提供：
    G1JointIndex  — 关节索引常量
    ARM_JOINTS    — 受控关节列表（13 个）
    JOINT_LIMITS  — 关节角度安全限位
    BaseArmController — 手臂控制器基类（DDS 通信 + 状态管理 + timeline 调度）
"""

import math
import time
import threading
from typing import List

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

class G1JointIndex:
    """G1 机器人关节索引常量，对应 LowCmd/LowState 的 motor_cmd 数组下标。"""
    # 左腿 (0-5)
    LeftHipPitch = 0;  LeftHipRoll = 1;  LeftHipYaw = 2
    LeftKnee = 3;  LeftAnklePitch = 4;  LeftAnkleB = 4
    LeftAnkleRoll = 5;  LeftAnkleA = 5
    # 右腿 (6-11)
    RightHipPitch = 6;  RightHipRoll = 7;  RightHipYaw = 8
    RightKnee = 9;  RightAnklePitch = 10;  RightAnkleB = 10
    RightAnkleRoll = 11;  RightAnkleA = 11
    # 腰部 (12-14)
    WaistYaw = 12;  WaistRoll = 13;  WaistA = 13
    WaistPitch = 14;  WaistB = 14
    # 左臂 (15-21)
    LeftShoulderPitch = 15;  LeftShoulderRoll = 16;  LeftShoulderYaw = 17
    LeftElbow = 18;  LeftWristRoll = 19
    LeftWristPitch = 20;  LeftWristYaw = 21
    # 右臂 (22-28)
    RightShoulderPitch = 22;  RightShoulderRoll = 23;  RightShoulderYaw = 24
    RightElbow = 25;  RightWristRoll = 26
    RightWristPitch = 27;  RightWristYaw = 28
    # 特殊索引
    kNotUsedJoint = 29

# 受控关节列表（13 个：双臂 10 + 腰部 3）
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

# 关节角度安全限位（弧度），防止超限指令损坏电机
JOINT_LIMITS = {
    G1JointIndex.LeftShoulderPitch:  (-2.5, 2.5),
    G1JointIndex.LeftShoulderRoll:   (-1.5, 2.0),
    G1JointIndex.LeftShoulderYaw:    (-1.5, 1.5),
    G1JointIndex.LeftElbow:          (-1.5, 2.0),
    G1JointIndex.LeftWristRoll:      (-1.5, 1.5),
    G1JointIndex.RightShoulderPitch: (-2.5, 2.5),
    G1JointIndex.RightShoulderRoll:  (-2.0, 1.5),
    G1JointIndex.RightShoulderYaw:   (-1.5, 1.5),
    G1JointIndex.RightElbow:         (-1.5, 2.0),
    G1JointIndex.RightWristRoll:     (-1.5, 1.5),
    G1JointIndex.WaistYaw:           (-1.5, 1.5),
    G1JointIndex.WaistRoll:          (-0.5, 0.5),
    G1JointIndex.WaistPitch:         (-0.5, 0.5),
}

class BaseArmController:
    """手臂控制器基类 — DDS 通信 + 状态管理 + timeline 阶段调度。

    子类只需覆盖：
        _build_timeline() — 定义运动阶段序列
        _on_complete()    — 完成后的行为（默认：标记 done）
    """

    def __init__(self, poses: list, transition_time: float = 2.0, kp: float = 60.0, kd: float = 1.5) -> None:
        self.poses = poses
        self.transition_time = transition_time
        self.kp = kp
        self.kd = kd
        self.control_dt = 0.02  # 50Hz

        self.time = 0.0
        self.done = False
        self.first_low_state = False

        self._state_lock = threading.Lock()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

        self._build_timeline()

    def _build_timeline(self):
        """构建阶段时间表。子类必须覆盖。

        每阶段格式: (起始时间, 结束时间, 阶段类型, 数据)
        阶段类型: "init_zero", "transition", "hold", "final_zero", "release"
        """
        self.timeline = []
        self.total_time = 0.0

    def _on_complete(self):
        """timeline 执行完毕后的行为。子类可覆盖。"""
        self.done = True
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0

    def Init(self):
        """初始化 DDS 发布者和订阅者。"""
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._state_handler, 10)

    def Start(self):
        """等待关节状态就绪后启动 50Hz 控制线程。"""
        self.thread = RecurrentThread(
            interval=self.control_dt, target=self._control_loop, name="arm_ctrl"
        )
        _t0 = time.time()
        while not self.first_low_state:
            if time.time() - _t0 > 10.0:
                raise TimeoutError("未收到关节状态消息，请检查 DDS 通信和网络接口")
            time.sleep(0.1)
        self.thread.Start()

    def _state_handler(self, msg):
        with self._state_lock:
            self.low_state = msg
        if not self.first_low_state:
            self.first_low_state = True

    def _get_current_joint_angles(self):
        with self._state_lock:
            state = self.low_state
        if state is None:
            return [0.0] * len(ARM_JOINTS)
        return [float(state.motor_state[j].q) for j in ARM_JOINTS]

    def _send_joint_cmd(self, target_angles: List[float], enable_sdk: bool = True) -> None:
        """向所有受控关节发送 PD 位置控制指令（含角度限位）。"""
        if enable_sdk:
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
        for i, joint in enumerate(ARM_JOINTS):
            lo, hi = JOINT_LIMITS.get(joint, (-math.pi, math.pi))
            self.low_cmd.motor_cmd[joint].q = float(np.clip(target_angles[i], lo, hi))
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

    def _control_loop(self) -> None:
        try:
            self._do_control_step()
        except Exception as e:
            print(f"[arm] 控制循环异常: {e}")

    def _do_control_step(self) -> None:
        with self._state_lock:
            if self.low_state is None:
                return

        self.time += self.control_dt

        if self.time >= self.total_time:
            self._on_complete()
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)
            return

        for t_start, t_end, ptype, pdata in self.timeline:
            if t_start <= self.time < t_end:
                ratio = np.clip((self.time - t_start) / (t_end - t_start), 0.0, 1.0)
                current = self._get_current_joint_angles()

                if ptype == "init_zero":
                    target = [(1.0 - ratio) * c for c in current]
                    self._send_joint_cmd(target)

                elif ptype == "transition":
                    name, target_angles = pdata
                    target = [ratio * t + (1.0 - ratio) * c
                              for t, c in zip(target_angles, current)]
                    self._send_joint_cmd(target)
                    if abs(self.time - t_start) < self.control_dt * 1.5:
                        self._log_pose(name)

                elif ptype == "hold":
                    _, target_angles = pdata
                    self._send_joint_cmd(target_angles)

                elif ptype == "final_zero":
                    target = [(1.0 - ratio) * c for c in current]
                    self._send_joint_cmd(target)
                    if abs(self.time - t_start) < self.control_dt * 1.5:
                        print("\n  [归零]")

                elif ptype == "release":
                    enable = 1.0 - ratio
                    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = enable
                    self._send_joint_cmd(current, enable_sdk=False)
                    if abs(self.time - t_start) < self.control_dt * 1.5:
                        print("\n  [释放 arm_sdk]")

                break

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def _log_pose(self, name):
        """打印姿态切换信息。子类可覆盖。"""
        print(f"\n  >>> {name}")

def build_timeline(poses, transition_time, include_init_zero=True,
                   include_final_zero=True, include_release=True):
    """通用 timeline 构建器。

    Args:
        poses: [(名称, 角度列表, 保持时间), ...]
        transition_time: 每个过渡阶段的时长
        include_init_zero: 是否在开头加归零阶段
        include_final_zero: 是否在结尾加归零阶段
        include_release: 是否在最后释放 arm_sdk
    Returns:
        (timeline, total_time)
    """
    timeline = []
    t = 0.0

    if include_init_zero:
        t_end = t + transition_time
        timeline.append((t, t_end, "init_zero", None))
        t = t_end

    for name, angles, hold_time in poses:
        t_end = t + transition_time
        timeline.append((t, t_end, "transition", (name, angles)))
        t = t_end
        t_end = t + hold_time
        timeline.append((t, t_end, "hold", (name, angles)))
        t = t_end

    if include_final_zero:
        t_end = t + transition_time
        timeline.append((t, t_end, "final_zero", None))
        t = t_end

    if include_release:
        t_end = t + transition_time
        timeline.append((t, t_end, "release", None))
        t = t_end

    return timeline, t
