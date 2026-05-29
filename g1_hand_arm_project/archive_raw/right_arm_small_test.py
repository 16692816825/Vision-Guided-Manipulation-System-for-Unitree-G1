
import time

import sys

import numpy as np



from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from unitree_sdk2py.utils.crc import CRC



RIGHT_ARM = [22, 23, 24, 25, 26]

WEIGHT = 29



KP = 25.0

KD = 1.0

DT = 0.02



MOVE_TIME = 2.0

HOLD_TIME = 0.5

BACK_TIME = 2.0

RELEASE_TIME = 1.0



# 只让右腕 roll 轻微转动，约 8.6 度

DELTA = {

    26: 0.15,

}



class RightArmSmallTest:

    def __init__(self):

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

        self.low_state = None

        self.crc = CRC()

        self.init_q = None

        self.target_q = None



    def state_cb(self, msg):

        self.low_state = msg



    def wait_state(self):

        print("waiting lowstate ...")

        while self.low_state is None:

            time.sleep(0.1)

        self.init_q = {j: self.low_state.motor_state[j].q for j in RIGHT_ARM}

        self.target_q = dict(self.init_q)

        for j, d in DELTA.items():

            self.target_q[j] = self.init_q[j] + d



        print("initial right arm:")

        for j in RIGHT_ARM:

            print(j, "q =", round(self.init_q[j], 4), "target =", round(self.target_q[j], 4))



    def write_cmd(self, q_map, weight):

        self.low_cmd.motor_cmd[WEIGHT].q = weight



        for j in RIGHT_ARM:

            self.low_cmd.motor_cmd[j].tau = 0.0

            self.low_cmd.motor_cmd[j].q = float(q_map[j])

            self.low_cmd.motor_cmd[j].dq = 0.0

            self.low_cmd.motor_cmd[j].kp = KP

            self.low_cmd.motor_cmd[j].kd = KD



        self.low_cmd.crc = self.crc.Crc(self.low_cmd)

        self.pub.Write(self.low_cmd)



    def interp(self, a, b, r):

        return {j: (1.0 - r) * a[j] + r * b[j] for j in RIGHT_ARM}



    def run_phase(self, name, seconds, start_q, end_q, start_weight, end_weight):

        print(name)

        steps = int(seconds / DT)

        for i in range(steps):

            r = (i + 1) / steps

            q = self.interp(start_q, end_q, r)

            w = (1.0 - r) * start_weight + r * end_weight

            self.write_cmd(q, w)

            time.sleep(DT)



    def run(self):

        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)

        self.pub.Init()



        self.sub = ChannelSubscriber("rt/lowstate", LowState_)

        self.sub.Init(self.state_cb, 10)



        self.wait_state()



        # 先接管但保持当前位置

        self.run_phase("enable arm_sdk and hold current", 1.0, self.init_q, self.init_q, 0.0, 1.0)



        # 小幅动作

        self.run_phase("move wrist slightly", MOVE_TIME, self.init_q, self.target_q, 1.0, 1.0)



        # 短暂停留

        self.run_phase("hold", HOLD_TIME, self.target_q, self.target_q, 1.0, 1.0)



        # 回到初始角

        self.run_phase("move back", BACK_TIME, self.target_q, self.init_q, 1.0, 1.0)



        # 释放 arm_sdk

        self.run_phase("release arm_sdk", RELEASE_TIME, self.init_q, self.init_q, 1.0, 0.0)



        print("Done.")



if __name__ == "__main__":

    net = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    print("network =", net)

    print("WARNING: right wrist will move slightly. Keep clear.")

    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, net)

    RightArmSmallTest().run()

