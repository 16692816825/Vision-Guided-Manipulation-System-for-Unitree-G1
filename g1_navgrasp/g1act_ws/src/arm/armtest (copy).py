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
    LeftWristPitch = 20   # 23DOF 无效
    LeftWristYaw = 21     # 23DOF 无效
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
        self.transition_time = 2.0   # 左臂抬起时间
        self.hold_time = 15.0        # 保持姿势时间
        self.release_time = 3.0      # 平滑释放时间

        self.kp_high = 60.0
        self.kd_high = 1.5
        self.kp_low = 2.0            # 最终极低刚度
        self.kd_low = 0.1

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False
        self.time_ = 0.0

        # 左臂关节列表（23DOF 去掉 LeftWristPitch/Yaw）
        self.left_arm_joints = [
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
        ]
        # 右臂和腰部保持不变
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

        # ----- 左臂目标姿势：手掌面向脸部 -----
        # 注意：LeftWristRoll 的正负需要根据实际机器人方向调整
        self.left_target_pos = {
            G1JointIndex.LeftShoulderPitch: -0.7,   # 抬臂
            G1JointIndex.LeftShoulderRoll:  -0.1,    # 适当外展，使手掌更容易朝向脸
            G1JointIndex.LeftShoulderYaw:   -0.3,
            G1JointIndex.LeftElbow:         -0.5,    # 屈肘
            G1JointIndex.LeftWristRoll:     -0.6,    # 旋转手腕，使掌心向内（朝向自己）
            # 如果您的机器人有腕俯仰/偏摆，可添加：
            # G1JointIndex.LeftWristPitch:   0.1,
            # G1JointIndex.LeftWristYaw:     0.0,
        }

        # 存储初始角度
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
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0  # 始终启用

        # 阶段1：左臂抬起
        if self.time_ < self.transition_time:
            ratio = self.time_ / self.transition_time
            for jid in self.left_arm_joints:
                start_q = self.left_init_pos[jid]
                target_q = self.left_target_pos.get(jid, start_q)
                q_des = (1.0 - ratio) * start_q + ratio * target_q
                self._set_joint_target(jid, q_des)
            for jid, init_q in self.right_init_pos.items():
                self._set_joint_target(jid, init_q)
            for jid, init_q in self.waist_init_pos.items():
                self._set_joint_target(jid, init_q)

        # 阶段2：保持姿势20秒
        elif self.time_ < self.transition_time + self.hold_time:
            for jid in self.left_arm_joints:
                target_q = self.left_target_pos.get(jid, self.left_init_pos[jid])
                self._set_joint_target(jid, target_q)
            for jid, init_q in self.right_init_pos.items():
                self._set_joint_target(jid, init_q)
            for jid, init_q in self.waist_init_pos.items():
                self._set_joint_target(jid, init_q)
            elapsed = self.time_ - self.transition_time
            if int(elapsed * 10) % 20 == 0:
                print(f"保持姿势中... {elapsed:.1f} / {self.hold_time:.0f} 秒")

        # 阶段3：平滑释放（刚度降低到极低）
        elif self.time_ < self.transition_time + self.hold_time + self.release_time:
            release_progress = (self.time_ - (self.transition_time + self.hold_time)) / self.release_time
            current_kp = self.kp_high * (1.0 - release_progress) + self.kp_low * release_progress
            current_kd = self.kd_high * (1.0 - release_progress) + self.kd_low * release_progress
            for jid in self.left_arm_joints + self.right_arm_joints + self.waist_joints:
                current_q = self.low_state.motor_state[jid].q
                self._set_joint_target(jid, current_q, kp=current_kp, kd=current_kd)
            if int(release_progress * 10) % 5 == 0:
                print(f"平滑释放中... 当前刚度 kp={current_kp:.1f}")

        else:
            self.done = True
            print("动作完成，手臂处于极低刚度柔顺状态。")

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

if __name__ == '__main__':
    print("警告：请确保机器人周围无障碍物。")
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
            print("程序正常结束，手臂已柔顺。")
            sys.exit(0)
