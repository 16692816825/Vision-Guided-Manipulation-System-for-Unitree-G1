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
import subprocess



from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from unitree_sdk2py.utils.crc import CRC



# 实测修正：15-19 对应机器人左手臂主动作，22-26 对应机器人右手臂

LEFT_ARM = [15, 16, 17, 18, 19]

OTHER_ARM = [22, 23, 24, 25, 26]

ALL_ARM = LEFT_ARM + OTHER_ARM

WEIGHT = 29



DT = 0.02

KP = 20.0

KD = 1.0

HAND_DIR = "/home/unitree/stark-serialport-example/python/revo2"
HAND_SCRIPT = "left_hand_safe_once.py"

def run_left_hand(pose):
    print("left hand pose:", pose)
    subprocess.check_call(["python3", HAND_SCRIPT, pose], cwd=HAND_DIR)



# 小幅预抓取增量，先非常保守

# 若方向不对，后面改符号

FOLD_ARM_DELTA = {
    15: 1.00,
    16: 0.32,   # Ôö¼ÓÍâÀ©£¬±Ü¿ª´óÍÈ
    17: 0.00,
    18: -1.80,
    19: 0.00,
}



LIFT_FOLDED_ARM_DELTA = {
    15: -1.00,
    16: 0.32,   # ±£³ÖÍâÀ©
    17: 0.00,
    18: -1.80,
    19: 0.00,
}



UNFOLD_PREGRASP_DELTA = {
    15: -1.00,
    16:  0.08,   # ×¥È¡Ê±´ó±ÛÄÚÊÕ£¬ÈÃÊÖÕÆ¸ü¿¿½üÆ¿Éí
    17:  0.00,
    18: -0.60,   # ±£³ÖÄã¸Õµ÷ºÃµÄ¸ß¶È
    19: -0.25,
}

LIFT_BOTTLE_DELTA = {
    15: -1.00,
    16:  0.08,
    17:  0.00,
    18: -1.05,   # after grasp, fold the forearm upward before returning
    19: -0.25,
}

RETRACT_OUT_DELTA = {
    15: 1.00,    # »ØÊÕÊ±±£³ÖºÍ fold ÀàËÆµÄ´ó±ÛºóÊÕ
    16: 0.48,    # »ØÊÕÊ±Ã÷ÏÔÍâÀ©£¬±Ü¿ª´óÍÈ
    17: 0.00,
    18: -1.80,   # Ð¡±ÛÕÛÆð£¬Ëõ¶Ì°ë¾¶
    19: 0.00,
}



class LeftPregraspSmallTest:

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

        self.q_lift_bottle = dict(self.q0)

        self.q_retract_out = dict(self.q0)



        for j, d in FOLD_ARM_DELTA.items():

            self.q_fold[j] = self.q0[j] + d



        for j, d in LIFT_FOLDED_ARM_DELTA.items():

            self.q_lift_folded[j] = self.q0[j] + d



        for j, d in UNFOLD_PREGRASP_DELTA.items():

            self.q_unfold_pregrasp[j] = self.q0[j] + d

        for j, d in LIFT_BOTTLE_DELTA.items():

            self.q_lift_bottle[j] = self.q0[j] + d

        for j, d in RETRACT_OUT_DELTA.items():

            self.q_retract_out[j] = self.q0[j] + d



        print("captured q0 / q_fold / q_lift_folded / q_unfold_pregrasp:")

        for j in ALL_ARM:

            print(

                j,

                "q0 =", round(self.q0[j], 4),

                "fold =", round(self.q_fold[j], 4),

                "lift_folded =", round(self.q_lift_folded[j], 4),

                "unfold_pregrasp =", round(self.q_unfold_pregrasp[j], 4),
                "lift_bottle =", round(self.q_lift_bottle[j], 4),
                "retract_out =", round(self.q_retract_out[j], 4),

            )



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

        # ³õÊ¼»¯£ºÕÅ¿ª×óÊÖ£¬½Ó¹ÜÉÏÖ«
        run_left_hand("open")
        time.sleep(0.5)

        self.phase("enable and hold both arms", 1.0, self.q0, self.q0, 0.0, 1.0)

        # -------- µÚÒ»´Î£ºÈ¥×¥Æ¿ --------
        self.phase("pick: fold forearm near upper arm", 2.2, self.q0, self.q_fold, 1.0, 1.0)
        self.phase("pick: lift and move folded arm", 4.2, self.q_fold, self.q_lift_folded, 1.0, 1.0)
        self.phase("pick: unfold forearm to bottle", 3.0, self.q_lift_folded, self.q_unfold_pregrasp, 1.0, 1.0)

        self.phase("pick: hold before grasp", 0.5, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)
        run_left_hand("thumb_open_max")
        time.sleep(0.4)
        run_left_hand("thumb_ready")
        time.sleep(0.5)
        run_left_hand("bottle")
        self.phase("pick: hold after grasp", 1.0, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)

        self.phase("pick: lift forearm with bottle", 2.0, self.q_unfold_pregrasp, self.q_lift_bottle, 1.0, 1.0)
        self.phase("pick: hover with bottle lifted", 3.0, self.q_lift_bottle, self.q_lift_bottle, 1.0, 1.0)
        self.phase("pick: lower bottle back to release position", 2.0, self.q_lift_bottle, self.q_unfold_pregrasp, 1.0, 1.0)

        self.phase("place: hold before release", 0.5, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)
        run_left_hand("open")
        self.phase("place: hold after release", 1.0, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)

        self.phase("place: fold forearm back after release", 3.0, self.q_unfold_pregrasp, self.q_lift_folded, 1.0, 1.0)
        self.phase("place: retract outward after release", 4.2, self.q_lift_folded, self.q_retract_out, 1.0, 1.0)
        self.phase("place: final empty-hand back to initial", 3.0, self.q_retract_out, self.q0, 1.0, 1.0)

        self.phase("release arm_sdk", 1.0, self.q0, self.q0, 1.0, 0.0)

        print("Done pick-place.")


if __name__ == "__main__":

    net = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    print("network =", net)

    print("WARNING: robot LEFT arm will move slightly; both shoulders may correct posture.")

    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, net)

    LeftPregraspSmallTest().run()
