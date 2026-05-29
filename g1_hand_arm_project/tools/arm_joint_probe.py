# unitree_sdk2py path shim for organized project layout
from pathlib import Path
_THIS_FILE = Path(__file__).resolve()
for _candidate in [_THIS_FILE.parent, *_THIS_FILE.parents]:
    if (_candidate / "unitree_sdk2py").exists():
        import sys as _sys
        if str(_candidate) not in _sys.path:
            _sys.path.insert(0, str(_candidate))
        break


import argparse

import time

import numpy as np



from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from unitree_sdk2py.utils.crc import CRC



ARM5_JOINTS = [15,16,17,18,19,22,23,24,25,26]

WEIGHT = 29

DT = 0.02

KP = 20.0

KD = 1.0



class Probe:

    def __init__(self, joint, delta):

        self.joint = joint

        self.delta = delta

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

        self.low_state = None

        self.crc = CRC()



    def cb(self, msg):

        self.low_state = msg



    def wait_state(self):

        while self.low_state is None:

            time.sleep(0.1)

        self.init_q = {j: self.low_state.motor_state[j].q for j in ARM5_JOINTS}

        self.target_q = dict(self.init_q)

        self.target_q[self.joint] += self.delta

        print("probe joint =", self.joint, "delta =", self.delta)

        print("init =", round(self.init_q[self.joint], 4), "target =", round(self.target_q[self.joint], 4))



    def write(self, qmap, weight):

        self.low_cmd.motor_cmd[WEIGHT].q = float(weight)

        for j in ARM5_JOINTS:

            self.low_cmd.motor_cmd[j].tau = 0.0

            self.low_cmd.motor_cmd[j].q = float(qmap[j])

            self.low_cmd.motor_cmd[j].dq = 0.0

            self.low_cmd.motor_cmd[j].kp = KP

            self.low_cmd.motor_cmd[j].kd = KD

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)

        self.pub.Write(self.low_cmd)



    def interp(self, a, b, r):

        return {j: (1-r)*a[j] + r*b[j] for j in ARM5_JOINTS}



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



        self.phase("enable and hold all arms", 1.0, self.init_q, self.init_q, 0.0, 1.0)

        self.phase("move one joint", 1.5, self.init_q, self.target_q, 1.0, 1.0)

        self.phase("hold", 0.3, self.target_q, self.target_q, 1.0, 1.0)

        self.phase("move back", 1.5, self.target_q, self.init_q, 1.0, 1.0)

        self.phase("release", 1.0, self.init_q, self.init_q, 1.0, 0.0)

        print("Done.")



if __name__ == "__main__":

    ap = argparse.ArgumentParser()

    ap.add_argument("--net", default="eth0")

    ap.add_argument("--joint", type=int, required=True)

    ap.add_argument("--delta", type=float, default=0.05)

    args = ap.parse_args()



    print("WARNING: one arm joint will move slightly.")

    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, args.net)

    Probe(args.joint, args.delta).run()

