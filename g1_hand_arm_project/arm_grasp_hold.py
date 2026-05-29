# unitree_sdk2py path shim for organized project layout
from pathlib import Path
_THIS_FILE = Path(__file__).resolve()
for _candidate in [_THIS_FILE.parent, *_THIS_FILE.parents]:
    if (_candidate / "unitree_sdk2py").exists():
        import sys as _sys
        if str(_candidate) not in _sys.path:
            _sys.path.insert(0, str(_candidate))
        break

import os
import sys
import time
import subprocess

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

LEFT_ARM = [15, 16, 17, 18, 19]
OTHER_ARM = [22, 23, 24, 25, 26]
ALL_ARM = LEFT_ARM + OTHER_ARM
WEIGHT = 29

DT = 0.02
KP = 20.0
KD = 1.0

FLAG = "/tmp/g1_open_retract.flag"

HAND_DIR = "/home/unitree/stark-serialport-example/python/revo2"
HAND_SCRIPT = f"{HAND_DIR}/left_hand_safe_once.py"

FOLD_ARM_DELTA = {
    15: 1.00,
    16: 0.20,
    17: 0.00,
    18: -1.80,
    19: 0.00,
}

LIFT_FOLDED_ARM_DELTA = {
    15: -1.00,
    16: 0.20,
    17: 0.00,
    18: -1.80,
    19: 0.00,
}

UNFOLD_PREGRASP_DELTA = {
    15: -1.00,
    16: 0.16,
    17: 0.00,
    18: -0.60,
    19: -0.25,
}

def run_left_hand(pose):
    print("left hand:", pose)
    subprocess.run(
        ["python3", HAND_SCRIPT, pose],
        cwd=HAND_DIR,
        check=True,
    )

class ArmGraspHold:
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

        self.q0 = {j: self.low_state.motor_state[j].q for j in ALL_ARM}
        self.q_fold = dict(self.q0)
        self.q_lift_folded = dict(self.q0)
        self.q_unfold_pregrasp = dict(self.q0)

        for j, d in FOLD_ARM_DELTA.items():
            self.q_fold[j] = self.q0[j] + d
        for j, d in LIFT_FOLDED_ARM_DELTA.items():
            self.q_lift_folded[j] = self.q0[j] + d
        for j, d in UNFOLD_PREGRASP_DELTA.items():
            self.q_unfold_pregrasp[j] = self.q0[j] + d

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
        return {j: (1 - r) * a[j] + r * b[j] for j in ALL_ARM}

    def phase(self, name, seconds, a, b, wa, wb):
        print(name)
        steps = max(1, int(seconds / DT))
        for i in range(steps):
            r = (i + 1) / steps
            q = self.interp(a, b, r)
            w = (1 - r) * wa + r * wb
            self.write(q, w)
            time.sleep(DT)

    def hold_until_release_command(self):
        print("holding grasp pose")
        print("run this in another terminal when ready:")
        print(f"  touch {FLAG}")

        while not os.path.exists(FLAG):
            self.write(self.q_unfold_pregrasp, 1.0)
            time.sleep(DT)

        os.remove(FLAG)

    def run(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()

        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.cb, 10)

        self.wait_state()

        run_left_hand("open")

        self.phase("enable and hold", 1.0, self.q0, self.q0, 0.0, 1.0)
        self.phase("fold forearm near upper arm", 2.5, self.q0, self.q_fold, 1.0, 1.0)
        self.phase("lift and move folded arm", 4.5, self.q_fold, self.q_lift_folded, 1.0, 1.0)
        self.phase("unfold forearm to bottle", 3.0, self.q_lift_folded, self.q_unfold_pregrasp, 1.0, 1.0)

        run_left_hand("thumb_open_max")
        time.sleep(0.3)
        run_left_hand("thumb_ready")
        time.sleep(0.4)
        run_left_hand("bottle")

        self.hold_until_release_command()

        run_left_hand("open")
        time.sleep(0.5)

        self.phase("fold forearm back", 3.0, self.q_unfold_pregrasp, self.q_lift_folded, 1.0, 1.0)
        self.phase("shoulder back while folded", 4.5, self.q_lift_folded, self.q_fold, 1.0, 1.0)
        self.phase("unfold back to initial", 2.5, self.q_fold, self.q0, 1.0, 1.0)
        self.phase("release arm_sdk", 1.0, self.q0, self.q0, 1.0, 0.0)

        print("Done.")

if __name__ == "__main__":
    net = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    print("network =", net)
    print("WARNING: arm will reach bottle and hold. Keep clear.")
    input("Press Enter to continue...")
    ChannelFactoryInitialize(0, net)
    ArmGraspHold().run()
