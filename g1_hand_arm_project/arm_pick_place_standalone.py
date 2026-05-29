# unitree_sdk2py path shim for organized project layout
from pathlib import Path
import argparse
import asyncio
import sys
import time

_THIS_FILE = Path(__file__).resolve()
for _candidate in [_THIS_FILE.parent] + list(_THIS_FILE.parents):
    if (_candidate / "unitree_sdk2py").exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

HAND_SDK_DIR = Path("/home/unitree/stark-serialport-example/python/revo2")
if str(HAND_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(HAND_SDK_DIR))

from revo2_utils import *  # noqa: F403
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
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

HAND_PORT = "/dev/ttyUSB1"
HAND_SLAVE_ID = 0x7E
HAND_SPEEDS = [300] * 6
THUMB_AUX_INDEX = 1
THUMB_AUX_READY_MIN = 500
REQUIRE_THUMB_AUX_READY = True

# Revo2 left hand order used by the SDK: [thumb, thumb_aux, index, middle, ring, pinky].
# The target behavior requires thumb_aux to rotate the thumb perpendicular before closing.
HAND_POSES = {
    "open": [0, 0, 0, 0, 0, 0],
    "thumb_open_max": [0, 0, 0, 0, 0, 0],
    "thumb_ready": [0, 1000, 0, 0, 0, 0],
    "bottle": [180, 850, 480, 560, 540, 420],
}

SAFE_START_Q = {
    15: 0.290546,
    16: 0.130748,
    17: 0.015687,
    18: 0.980525,
    19: 0.086035,
    22: 0.290581,
    23: -0.142648,
    24: -0.000635,
    25: 0.974725,
    26: -0.106983,
}
SAFE_START_TOLERANCE_RAD = 0.18
SAFE_RETURN_SECONDS = 6.0
SAFE_RETURN_HOLD_SECONDS = 0.5
SAFE_RETURN_MAX_DELTA_RAD = 2.4

FOLD_ARM_DELTA = {
    15: 1.00,
    16: 0.32,
    17: 0.00,
    18: -1.80,
    19: 0.00,
}

LIFT_FOLDED_ARM_DELTA = {
    15: -1.00,
    16: 0.32,
    17: 0.00,
    18: -1.80,
    19: 0.00,
}

UNFOLD_PREGRASP_DELTA = {
    15: -1.00,
    16: 0.08,
    17: 0.00,
    18: -0.70,
    19: -0.25,
}

LIFT_BOTTLE_DELTA = {
    15: -1.00,
    16: 0.08,
    17: 0.00,
    18: -1.15,
    19: -0.25,
}

RETRACT_OUT_DELTA = {
    15: 1.00,
    16: 0.48,
    17: 0.00,
    18: -1.80,
    19: 0.00,
}


def safe_start_qmap(current):
    target = dict(current)
    for joint, q in SAFE_START_Q.items():
        target[joint] = float(q)
    return target


def safe_start_delta_report(qmap):
    deltas = {joint: float(qmap[joint]) - float(SAFE_START_Q[joint]) for joint in SAFE_START_Q}
    max_joint = max(deltas, key=lambda joint: abs(deltas[joint]))
    return deltas, max_joint, abs(deltas[max_joint])


def is_near_safe_start(qmap, tolerance=SAFE_START_TOLERANCE_RAD):
    _, _, max_delta = safe_start_delta_report(qmap)
    return max_delta <= float(tolerance)


class StandalonePickPlace:
    def __init__(self, hand_port=HAND_PORT):
        self.hand_port = hand_port
        self.hand_client = None
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()
        self.current_qmap = None
        self.arm_control_started = False

    def cb(self, msg):
        self.low_state = msg

    async def connect_hand(self):
        self.hand_client = await libstark.modbus_open(self.hand_port, libstark.Baudrate.Baud460800)  # noqa: F405
        if not self.hand_client:
            raise RuntimeError(f"failed to open left hand serial port {self.hand_port}")

        info = await self.hand_client.get_device_info(HAND_SLAVE_ID)
        if not info:
            raise RuntimeError(f"failed to get left hand info, id=0x{HAND_SLAVE_ID:02x}")

        print("Left hand:", info.description)
        await self.hand_client.set_finger_unit_mode(HAND_SLAVE_ID, libstark.FingerUnitMode.Normalized)  # noqa: F405

    def close_hand(self):
        if self.hand_client is not None:
            libstark.modbus_close(self.hand_client)  # noqa: F405
            self.hand_client = None

    async def send_hand_pose(self, name, wait_s=2.0):
        if self.hand_client is None:
            raise RuntimeError("left hand serial is not connected")
        if name not in HAND_POSES:
            raise ValueError(f"unknown hand pose: {name}")

        target = HAND_POSES[name]
        print(f"hand pose {name} target={target}")
        await self.hand_client.set_finger_positions_and_speeds(HAND_SLAVE_ID, target, HAND_SPEEDS)
        await asyncio.sleep(wait_s)
        feedback = None
        try:
            feedback = await self.hand_client.get_finger_positions(HAND_SLAVE_ID)
            print(f"hand pose {name} target={target} feedback={list(feedback)}")
        except Exception as exc:
            print(f"hand pose {name} feedback read failed: {exc}")
        if name == "thumb_ready" and REQUIRE_THUMB_AUX_READY:
            self.validate_thumb_aux_ready(feedback)
        return feedback

    def validate_thumb_aux_ready(self, feedback):
        if feedback is None:
            raise RuntimeError("thumb_ready failed: cannot read ThumbAux feedback")
        thumb_aux = int(feedback[THUMB_AUX_INDEX])
        if thumb_aux < THUMB_AUX_READY_MIN:
            raise RuntimeError(
                "thumb_ready failed: ThumbAux did not reach the perpendicular pre-grasp target; "
                f"feedback[{THUMB_AUX_INDEX}]={thumb_aux}, required>={THUMB_AUX_READY_MIN}"
            )

    def build_targets(self, q0):
        self.q0 = dict(q0)
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

    def wait_state(self):
        print("waiting lowstate ...")
        while self.low_state is None:
            time.sleep(0.1)

        q0 = {j: self.low_state.motor_state[j].q for j in ALL_ARM}
        self.build_targets(q0)

    def ensure_safe_start(self):
        deltas, max_joint, max_delta = safe_start_delta_report(self.q0)
        print(
            "safety precheck:",
            "max_delta_joint =", max_joint,
            "max_delta_rad =", round(max_delta, 4),
        )
        if is_near_safe_start(self.q0):
            print("safety precheck: arm is already near safe start")
            return

        if max_delta > SAFE_RETURN_MAX_DELTA_RAD:
            raise RuntimeError(
                "current arm pose is too far from safe start for automatic return; "
                f"joint {max_joint} delta={deltas[max_joint]:+.4f} rad"
            )

        q_safe = safe_start_qmap(self.q0)
        self.phase(
            "safety: slowly return arms to safe start",
            SAFE_RETURN_SECONDS,
            self.q0,
            q_safe,
            1.0,
            1.0,
        )
        self.phase(
            "safety: hold safe start",
            SAFE_RETURN_HOLD_SECONDS,
            q_safe,
            q_safe,
            1.0,
            1.0,
        )
        self.build_targets(q_safe)

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
        self.current_qmap = dict(qmap)

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
        self.current_qmap = dict(b)

    def q_distance(self, a, b):
        return max(abs(float(a[j]) - float(b[j])) for j in ALL_ARM)

    async def abort_return(self, cause):
        print("abort_return:", cause)
        if self.hand_client is not None:
            try:
                await self.send_hand_pose("open", 1.0)
            except Exception as exc:
                print("abort_return: failed to open hand:", exc)

        if not self.arm_control_started:
            return

        current = self.current_qmap or self.q0
        if self.q_distance(current, self.q0) <= 0.05:
            self.phase("abort: release arm_sdk", 1.0, self.q0, self.q0, 1.0, 0.0)
            return

        self.phase("abort: fold forearm back from current pose", 3.0, current, self.q_lift_folded, 1.0, 1.0)
        self.phase("abort: retract outward after failed pre-grasp", 4.0, self.q_lift_folded, self.q_retract_out, 1.0, 1.0)
        self.phase("abort: final empty-hand back to initial", 3.0, self.q_retract_out, self.q0, 1.0, 1.0)
        self.phase("abort: release arm_sdk", 1.0, self.q0, self.q0, 1.0, 0.0)

    async def run(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()

        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.cb, 10)

        self.wait_state()
        await self.connect_hand()
        try:
            await self.send_hand_pose("open", 2.0)
            time.sleep(0.5)

            self.phase("enable and hold both arms", 1.0, self.q0, self.q0, 0.0, 1.0)
            self.arm_control_started = True
            self.ensure_safe_start()

            self.phase("pick: fold forearm near upper arm", 2.2, self.q0, self.q_fold, 1.0, 1.0)
            self.phase("pick: lift and move folded arm", 4.2, self.q_fold, self.q_lift_folded, 1.0, 1.0)
            self.phase("pick: unfold forearm to bottle", 3.0, self.q_lift_folded, self.q_unfold_pregrasp, 1.0, 1.0)

            self.phase("pick: hold before grasp", 0.5, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)
            await self.send_hand_pose("thumb_open_max", 1.0)
            time.sleep(0.4)
            await self.send_hand_pose("thumb_ready", 2.0)
            time.sleep(0.5)
            await self.send_hand_pose("bottle", 2.0)
            self.phase("pick: hold after grasp", 1.0, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)

            self.phase("pick: lift forearm with bottle", 2.0, self.q_unfold_pregrasp, self.q_lift_bottle, 1.0, 1.0)
            self.phase("pick: hover with bottle lifted", 3.0, self.q_lift_bottle, self.q_lift_bottle, 1.0, 1.0)
            self.phase("pick: lower bottle back to release position", 2.0, self.q_lift_bottle, self.q_unfold_pregrasp, 1.0, 1.0)

            self.phase("place: hold before release", 0.5, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)
            await self.send_hand_pose("open", 2.0)
            self.phase("place: hold after release", 1.0, self.q_unfold_pregrasp, self.q_unfold_pregrasp, 1.0, 1.0)

            self.phase("place: fold forearm back after release", 3.0, self.q_unfold_pregrasp, self.q_lift_folded, 1.0, 1.0)
            self.phase("place: retract outward after release", 4.2, self.q_lift_folded, self.q_retract_out, 1.0, 1.0)
            self.phase("place: final empty-hand back to initial", 3.0, self.q_retract_out, self.q0, 1.0, 1.0)

            self.phase("release arm_sdk", 1.0, self.q0, self.q0, 1.0, 0.0)
            self.arm_control_started = False
            print("Done standalone pick-place.")
        except Exception as exc:
            await self.abort_return(exc)
            raise
        finally:
            self.close_hand()


async def hand_only_test(hand_port=HAND_PORT):
    runner = StandalonePickPlace(hand_port=hand_port)
    await runner.connect_hand()
    try:
        sequence = [
            ("open", 5.0),
            ("thumb_ready", 5.0),
            ("open", 5.0),
            ("bottle", 5.0),
            ("open", 5.0),
        ]
        for pose, wait_s in sequence:
            await runner.send_hand_pose(pose, wait_s)
    finally:
        runner.close_hand()


def parse_args():
    parser = argparse.ArgumentParser(description="Standalone Unitree G1 + Revo2 left-hand fixed pick-place flow.")
    parser.add_argument("net", nargs="?", default="eth0", help="DDS network interface, default: eth0")
    parser.add_argument("--hand-port", default=HAND_PORT, help=f"left Revo2 serial port, default: {HAND_PORT}")
    parser.add_argument("--hand-only-test", action="store_true", help="test Revo2 hand poses only; does not move arm")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.hand_only_test:
        print("WARNING: left Revo2 hand will move; robot arm will not move.")
        input("Press Enter to continue...")
        asyncio.run(hand_only_test(args.hand_port))
    else:
        print("network =", args.net)
        print("WARNING: robot LEFT arm will move; both shoulders may correct posture.")
        input("Press Enter to continue...")
        ChannelFactoryInitialize(0, args.net)
        asyncio.run(StandalonePickPlace(hand_port=args.hand_port).run())
