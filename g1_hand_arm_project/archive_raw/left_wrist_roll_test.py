import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

ALL_ARM = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]
WEIGHT = 29
DT = 0.02
KP = 12.0
KD = 1.2

TEST_JOINT = 19
DELTA = 0.25

class WristRollTest:
    def __init__(self):
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

    def cb(self, msg):
        self.low_state = msg

    def wait_state(self):
        while self.low_state is None:
            time.sleep(0.1)
        self.q0 = {j: self.low_state.motor_state[j].q for j in ALL_ARM}
        self.q1 = dict(self.q0)
        self.q1[TEST_JOINT] = self.q0[TEST_JOINT] + DELTA
        print("q19 start =", round(self.q0[19], 4), "target =", round(self.q1[19], 4))

    def write(self, qmap, weight):
        self.low_cmd.motor_cmd[WEIGHT].q = float(weight)
        for j in ALL_ARM:
            self.low_cmd.motor_cmd[j].tau = 0.0
            self.low_cmd.motor_cmd[j].q = float(qmap[j])
            self.low_cmd.motor_cmd[j].dq = 0.0
            self.low_cmd.motor_cmd[j].kp = KP
            self.low_cmd.motor_cmd[j].kd = KD
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def interp(self, a, b, r):
        return {j: (1-r)*a[j] + r*b[j] for j in ALL_ARM}

    def phase(self, name, seconds, a, b, wa, wb):
        print(name)
        steps = int(seconds / DT)
        for i in range(steps):
            r = (i + 1) / steps
            q = self.interp(a, b, r)
            w = (1-r)*wa + r*wb
            self.write(q, w)
            time.sleep(DT)

    def run(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.cb, 10)
        self.wait_state()

        self.phase("enable", 1.0, self.q0, self.q0, 0.0, 1.0)
        self.phase("wrist roll positive", 2.0, self.q0, self.q1, 1.0, 1.0)
        self.phase("hold", 0.5, self.q1, self.q1, 1.0, 1.0)
        self.phase("back", 2.0, self.q1, self.q0, 1.0, 1.0)
        self.phase("release", 1.0, self.q0, self.q0, 1.0, 0.0)
        print("Done.")

if __name__ == "__main__":
    net = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    ChannelFactoryInitialize(0, net)
    WristRollTest().run()
