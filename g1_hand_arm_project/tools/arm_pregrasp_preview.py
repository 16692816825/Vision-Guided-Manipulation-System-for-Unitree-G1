#!/usr/bin/env python3
"""
Move the left arm to the current pre-grasp posture, hold briefly, and return.

This is an arm preview tool for mapping small pre-grasp corrections. It does not
close the Revo2 hand, does not pick up the bottle, and releases arm_sdk by
holding the current command while smoothly lowering the arm_sdk weight.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List


LEFT_ARM = [15, 16, 17, 18, 19]
OTHER_ARM = [22, 23, 24, 25, 26]
ALL_ARM = LEFT_ARM + OTHER_ARM
WEIGHT = 29

DT = 0.02
KP = 20.0
KD = 1.0

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


def init_sdk_path() -> None:
    this_file = Path(__file__).resolve()
    for candidate in [this_file.parent, *this_file.parents]:
        if (candidate / "unitree_sdk2py").exists():
            if str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
            return


def parse_joint_offsets(items: Iterable[str]) -> Dict[int, float]:
    offsets: Dict[int, float] = {}
    for item in items:
        if not item:
            continue
        for part in str(item).split(","):
            part = part.strip()
            if not part:
                continue
            if "=" not in part:
                raise ValueError(f"offset must be JOINT=DELTA, got: {part}")
            joint_text, delta_text = part.split("=", 1)
            offsets[int(joint_text.strip())] = float(delta_text.strip())
    return offsets


def apply_limited_offsets(
    base: Dict[int, float],
    offsets: Dict[int, float],
    allowed_joints: List[int],
    max_abs_delta: float,
) -> Dict[int, float]:
    q = dict(base)
    allowed = set(allowed_joints)
    limit = abs(float(max_abs_delta))
    for joint, delta in offsets.items():
        if int(joint) not in allowed:
            raise ValueError(f"joint {joint} is not allowed for pre-grasp offset")
        if abs(float(delta)) > limit:
            raise ValueError(
                f"joint {joint} offset {delta} exceeds safety limit {limit}"
            )
        q[int(joint)] = float(q[int(joint)]) + float(delta)
    return q


def _target_from_delta(q0: Dict[int, float], delta_map: Dict[int, float]) -> Dict[int, float]:
    target = dict(q0)
    for joint, delta in delta_map.items():
        target[joint] = float(q0[joint]) + float(delta)
    return target


def build_targets(
    q0: Dict[int, float],
    pregrasp_offsets: Dict[int, float] | None = None,
    max_abs_offset: float = 0.08,
) -> Dict[str, Dict[int, float]]:
    q_fold = _target_from_delta(q0, FOLD_ARM_DELTA)
    q_lift_folded = _target_from_delta(q0, LIFT_FOLDED_ARM_DELTA)
    q_unfold_pregrasp = _target_from_delta(q0, UNFOLD_PREGRASP_DELTA)
    if pregrasp_offsets:
        q_unfold_pregrasp = apply_limited_offsets(
            q_unfold_pregrasp,
            pregrasp_offsets,
            allowed_joints=LEFT_ARM,
            max_abs_delta=max_abs_offset,
        )
    return {
        "q0": dict(q0),
        "q_fold": q_fold,
        "q_lift_folded": q_lift_folded,
        "q_unfold_pregrasp": q_unfold_pregrasp,
    }


def run_left_hand_open() -> None:
    subprocess.run(
        ["python3", HAND_SCRIPT, "open"],
        cwd=HAND_DIR,
        check=True,
    )


class ArmPregraspPreview:
    def __init__(self, sdk, offsets: Dict[int, float], max_abs_offset: float, hold_seconds: float):
        self.sdk = sdk
        self.offsets = offsets
        self.max_abs_offset = max_abs_offset
        self.hold_seconds = hold_seconds
        self.low_cmd = sdk["default_cmd"]()
        self.low_state = None
        self.crc = sdk["CRC"]()

    def cb(self, msg):
        self.low_state = msg

    def wait_state(self):
        print("waiting lowstate ...")
        while self.low_state is None:
            time.sleep(0.1)
        q0 = {joint: self.low_state.motor_state[joint].q for joint in ALL_ARM}
        self.targets = build_targets(q0, self.offsets, self.max_abs_offset)
        print("captured arm q:")
        for joint in ALL_ARM:
            print(joint, round(q0[joint], 4))
        if self.offsets:
            print("pregrasp offsets:", self.offsets)

    def write(self, qmap: Dict[int, float], weight: float):
        self.low_cmd.motor_cmd[WEIGHT].q = float(weight)
        for joint in ALL_ARM:
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].q = float(qmap[joint])
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].kp = KP
            self.low_cmd.motor_cmd[joint].kd = KD

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def interp(self, a: Dict[int, float], b: Dict[int, float], r: float):
        return {joint: (1.0 - r) * a[joint] + r * b[joint] for joint in ALL_ARM}

    def phase(self, name: str, seconds: float, a, b, wa: float, wb: float):
        print(name)
        steps = max(1, int(float(seconds) / DT))
        for i in range(steps):
            r = float(i + 1) / float(steps)
            q = self.interp(a, b, r)
            w = (1.0 - r) * float(wa) + r * float(wb)
            self.write(q, w)
            time.sleep(DT)

    def hold(self, seconds: float, qmap):
        print(f"hold pregrasp preview for {seconds:.1f}s")
        steps = max(1, int(float(seconds) / DT))
        for _ in range(steps):
            self.write(qmap, 1.0)
            time.sleep(DT)

    def run(self):
        self.pub = self.sdk["ChannelPublisher"]("rt/arm_sdk", self.sdk["LowCmd"])
        self.pub.Init()
        self.sub = self.sdk["ChannelSubscriber"]("rt/lowstate", self.sdk["LowState"])
        self.sub.Init(self.cb, 10)

        self.wait_state()

        q0 = self.targets["q0"]
        q_fold = self.targets["q_fold"]
        q_lift = self.targets["q_lift_folded"]
        q_pregrasp = self.targets["q_unfold_pregrasp"]

        self.phase("enable and hold", 1.0, q0, q0, 0.0, 1.0)
        self.phase("fold forearm near upper arm", 2.5, q0, q_fold, 1.0, 1.0)
        self.phase("lift and move folded arm", 4.5, q_fold, q_lift, 1.0, 1.0)
        self.phase("unfold forearm to pregrasp preview", 3.0, q_lift, q_pregrasp, 1.0, 1.0)
        self.hold(self.hold_seconds, q_pregrasp)
        self.phase("fold forearm back", 3.0, q_pregrasp, q_lift, 1.0, 1.0)
        self.phase("shoulder back while folded", 4.5, q_lift, q_fold, 1.0, 1.0)
        self.phase("unfold back to initial", 2.5, q_fold, q0, 1.0, 1.0)
        self.phase("release arm_sdk", 1.0, q0, q0, 1.0, 0.0)
        print("Done.")


def load_sdk():
    init_sdk_path()
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    return {
        "ChannelFactoryInitialize": ChannelFactoryInitialize,
        "ChannelPublisher": ChannelPublisher,
        "ChannelSubscriber": ChannelSubscriber,
        "LowCmd": LowCmd_,
        "LowState": LowState_,
        "default_cmd": unitree_hg_msg_dds__LowCmd_,
        "CRC": CRC,
    }


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Preview the left-arm pre-grasp posture and return safely."
    )
    parser.add_argument("--net", default="eth0")
    parser.add_argument(
        "--offset",
        action="append",
        default=[],
        help="Pregrasp joint offset, e.g. --offset 16=0.03 or --offset 16=0.03,18=-0.02",
    )
    parser.add_argument("--max-abs-offset", type=float, default=0.08)
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    parser.add_argument("--no-hand", action="store_true", help="Do not command the hand open first")
    parser.add_argument("--dry-run", action="store_true", help="Validate arguments without DDS control")
    parser.add_argument("--yes", action="store_true", help="Skip the interactive confirmation prompt")
    return parser


def run(args) -> int:
    offsets = parse_joint_offsets(args.offset)
    apply_limited_offsets(
        {joint: 0.0 for joint in ALL_ARM},
        offsets,
        allowed_joints=LEFT_ARM,
        max_abs_delta=args.max_abs_offset,
    )

    print("WARNING: left arm will move to pregrasp preview, hold, then return.")
    print("This script will not close the hand and will not pick the bottle.")
    print(f"network = {args.net}")
    print(f"offsets = {offsets}")
    print(f"hold_seconds = {float(args.hold_seconds):.1f}")
    if args.dry_run:
        print("dry-run only; no DDS control")
        return 0

    if not args.yes:
        input("Press Enter to continue...")

    if not args.no_hand:
        run_left_hand_open()

    sdk = load_sdk()
    sdk["ChannelFactoryInitialize"](0, args.net)
    ArmPregraspPreview(
        sdk=sdk,
        offsets=offsets,
        max_abs_offset=float(args.max_abs_offset),
        hold_seconds=max(0.2, float(args.hold_seconds)),
    ).run()
    return 0


def main():
    return run(build_arg_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
