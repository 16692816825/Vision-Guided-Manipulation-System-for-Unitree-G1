import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

kPi = 3.141592654

class G1JointIndex:
    # 左腿
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    # 右腿
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    # 腰部
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    # 左臂
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    # 右臂
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    # 控制字
    kNotUsedJoint = 29

class Custom:
    def __init__(self):
        self.control_dt_ = 0.02
        self.transition_time = 1.0       # 动作1：抬臂时间
        self.hold_time = 5.0             # 动作1：保持时间
        self.release_time = 1.0          # 释放时间

        self.kp_high = 60.0
        self.kd_high = 1.5
        self.kp_low = 2.0
        self.kd_low = 0.1

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False
        self.time_ = 0.0

        # 左臂关节列表
        self.left_arm_joints = [
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
        ]
        self.right_arm_joints = [
            G1JointIndex.RightShoulderPitch,
            G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,
            G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,
        ]
        self.waist_joints = [
            G1JointIndex.WaistYaw,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistPitch,
        ]

        # 动作1：右臂抬起，手掌朝向脸部
        self.right_target_pos = {
            G1JointIndex.RightShoulderPitch: -0.8,
            G1JointIndex.RightShoulderRoll:  0.0,
            G1JointIndex.RightShoulderYaw:   -0.3,
            G1JointIndex.RightElbow:         0.5,
            G1JointIndex.RightWristRoll:     -0.3,
        }

        # 初始角度存储
        self.left_init_pos = {}
        self.right_init_pos = {}
        self.waist_init_pos = {}
        self.init_positions_recorded = False

    def Init(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while not self.first_update_low_state:
            time.sleep(1)
        self._record_initial_positions()
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if not self.first_update_low_state:
            self.first_update_low_state = True

    def _record_initial_positions(self):
        if self.low_state is None:
            return
        for jid in self.left_arm_joints:
            self.left_init_pos[jid] = self.low_state.motor_state[jid].q
        for jid in self.right_arm_joints:
            self.right_init_pos[jid] = self.low_state.motor_state[jid].q
        for jid in self.waist_joints:
            self.waist_init_pos[jid] = self.low_state.motor_state[jid].q
        self.init_positions_recorded = True

    def _set_joint_target(self, joint_id, q, dq=0.0, tau_ff=0.0, kp=None, kd=None):
        self.low_cmd.motor_cmd[joint_id].q = q
        self.low_cmd.motor_cmd[joint_id].dq = dq
        self.low_cmd.motor_cmd[joint_id].kp = kp if kp is not None else self.kp_high
        self.low_cmd.motor_cmd[joint_id].kd = kd if kd is not None else self.kd_high
        self.low_cmd.motor_cmd[joint_id].tau_ff = tau_ff

    def LowCmdWrite(self):
        if not self.init_positions_recorded:
            return

        self.time_ += self.control_dt_
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        # 阶段1：右臂抬起 → 手掌朝向脸部
        if self.time_ < self.transition_time:
            ratio = self.time_ / self.transition_time
            for jid in self.right_arm_joints:
                start_q = self.right_init_pos[jid]
                target_q = self.right_target_pos.get(jid, start_q)
                q_des = (1.0 - ratio) * start_q + ratio * target_q
                self._set_joint_target(jid, q_des)
            # 左臂和腰部保持初始位置
            for jid in self.left_arm_joints:
                self._set_joint_target(jid, self.left_init_pos[jid])
            for jid in self.waist_joints:
                self._set_joint_target(jid, self.waist_init_pos[jid])

        # 阶段2：保持手掌朝脸姿势
        elif self.time_ < self.transition_time + self.hold_time:
            for jid in self.right_arm_joints:
                target_q = self.right_target_pos.get(jid, self.right_init_pos[jid])
                self._set_joint_target(jid, target_q)
            for jid in self.left_arm_joints:
                self._set_joint_target(jid, self.left_init_pos[jid])
            for jid in self.waist_joints:
                self._set_joint_target(jid, self.waist_init_pos[jid])

        # 阶段3：柔顺释放
        elif self.time_ < self.transition_time + self.hold_time + self.release_time:
            release_progress = (self.time_ - (self.transition_time + self.hold_time)) / self.release_time
            current_kp = self.kp_high * (1.0 - release_progress) + self.kp_low * release_progress
            current_kd = self.kd_high * (1.0 - release_progress) + self.kd_low * release_progress
            for jid in self.right_arm_joints + self.left_arm_joints + self.waist_joints:
                current_q = self.low_state.motor_state[jid].q
                self._set_joint_target(jid, current_q, kp=current_kp, kd=current_kd)
        else:
            self.done = True
            print("全部动作完成！")

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

if __name__ == '__main__':
    print("警告：确保机器人周围安全！")
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:
        time.sleep(1)
        if custom.done:
            print("程序正常结束")
            sys.exit(0)
