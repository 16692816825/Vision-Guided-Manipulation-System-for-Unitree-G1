# unitree_sdk2py path shim for organized project layout
from pathlib import Path
_THIS_FILE = Path(__file__).resolve()
for _candidate in [_THIS_FILE.parent, *_THIS_FILE.parents]:
    if (_candidate / "unitree_sdk2py").exists():
        import sys as _sys
        if str(_candidate) not in _sys.path:
            _sys.path.insert(0, str(_candidate))
        break


import time

import sys



from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from unitree_sdk2py.utils.crc import CRC



ARM5_JOINTS = [15,16,17,18,19,22,23,24,25,26]

WEIGHT = 29

DT = 0.02

KP = 25.0

KD = 1.0



class HoldTest:

    def __init__(self):

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

        self.low_state = None

        self.crc = CRC()



    def cb(self, msg):

        self.low_state = msg



    def wait_state(self):

        print("waiting lowstate ...")

        while self.low_state is None:

            time.sleep(0.1)

        self.q0 = {j: self.low_state.motor_state[j].q for j in ARM5_JOINTS}

        print("captured arm q:")

        for j in ARM5_JOINTS:

            print(j, round(self.q0[j], 4))



    def write(self, weight):

        self.low_cmd.motor_cmd[WEIGHT].q = float(weight)

        for j in ARM5_JOINTS:

            self.low_cmd.motor_cmd[j].tau = 0.0

            self.low_cmd.motor_cmd[j].q = float(self.q0[j])

            self.low_cmd.motor_cmd[j].dq = 0.0

            self.low_cmd.motor_cmd[j].kp = KP

            self.low_cmd.motor_cmd[j].kd = KD



        self.low_cmd.crc = self.crc.Crc(self.low_cmd)

        self.pub.Write(self.low_cmd)



    def ramp(self, seconds, w0, w1):

        steps = int(seconds / DT)

        for i in range(steps):

            r = (i + 1) / steps

            w = (1-r)*w0 + r*w1

            self.write(w)

            time.sleep(DT)



    def hold(self, seconds, w):

        steps = int(seconds / DT)

        for _ in range(steps):

            self.write(w)

            time.sleep(DT)



    def run(self):

        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)

        self.pub.Init()



        self.sub = ChannelSubscriber("rt/lowstate", LowState_)

        self.sub.Init(self.cb, 10)



        self.wait_state()



        print("ramp weight 0 -> 1")

        self.ramp(1.0, 0.0, 1.0)



        print("hold 2s")

        self.hold(2.0, 1.0)



        print("ramp weight 1 -> 0")

        self.ramp(1.0, 1.0, 0.0)



        print("Done.")



if __name__ == "__main__":

    net = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    print("network =", net)

    print("WARNING: arm_sdk will hold both arms at current posture.")

    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, net)

    HoldTest().run()

