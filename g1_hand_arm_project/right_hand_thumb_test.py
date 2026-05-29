from pathlib import Path
import argparse
import asyncio
import sys


HAND_SDK_DIR = Path("/home/unitree/stark-serialport-example/python/revo2")
if str(HAND_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(HAND_SDK_DIR))

from revo2_utils import *  # noqa: F403


HAND_PORT = "/dev/ttyUSB2"
HAND_SLAVE_ID = 0x7F
HAND_SPEEDS = [300] * 6
THUMB_AUX_INDEX = 1
THUMB_AUX_READY_MIN = 500
CONNECT_RETRIES = 3
CONNECT_RETRY_DELAY_S = 0.5

# Revo2 hand order used by the SDK: [thumb, thumb_aux, index, middle, ring, pinky].
# Target diagnostic sequence: open -> thumb_ready -> bottle -> open.
HAND_POSES = {
    "open": [0, 0, 0, 0, 0, 0],
    "thumb_ready": [0, 1000, 0, 0, 0, 0],
    "bottle": [180, 850, 480, 560, 540, 420],
}


class RightHandThumbTest:
    def __init__(
        self,
        hand_port=HAND_PORT,
        require_thumb_feedback=True,
        connect_retries=CONNECT_RETRIES,
        connect_retry_delay_s=CONNECT_RETRY_DELAY_S,
    ):
        self.hand_port = hand_port
        self.require_thumb_feedback = require_thumb_feedback
        self.connect_retries = connect_retries
        self.connect_retry_delay_s = connect_retry_delay_s
        self.hand_client = None

    async def connect(self):
        last_exc = None
        for attempt in range(1, self.connect_retries + 1):
            try:
                self.hand_client = await libstark.modbus_open(self.hand_port, libstark.Baudrate.Baud460800)  # noqa: F405
                if not self.hand_client:
                    raise RuntimeError(f"failed to open right hand serial port {self.hand_port}")

                info = await self.hand_client.get_device_info(HAND_SLAVE_ID)
                if not info:
                    raise RuntimeError(f"failed to get right hand info, id=0x{HAND_SLAVE_ID:02x}")

                print("Right hand:", info.description)
                await self.hand_client.set_finger_unit_mode(HAND_SLAVE_ID, libstark.FingerUnitMode.Normalized)  # noqa: F405
                return
            except Exception as exc:
                last_exc = exc
                print(f"connect attempt {attempt}/{self.connect_retries} failed: {exc}")
                self.close()
                if attempt < self.connect_retries:
                    await asyncio.sleep(self.connect_retry_delay_s)

        raise RuntimeError(f"failed to connect right hand after {self.connect_retries} attempts: {last_exc}")

    def close(self):
        if self.hand_client is not None:
            libstark.modbus_close(self.hand_client)  # noqa: F405
            self.hand_client = None

    async def send_pose(self, name, wait_s):
        if self.hand_client is None:
            raise RuntimeError("right hand serial is not connected")
        if name not in HAND_POSES:
            raise ValueError(f"unknown hand pose: {name}")

        target = HAND_POSES[name]
        print(f"right hand pose {name} target={target}")
        await self.hand_client.set_finger_positions_and_speeds(HAND_SLAVE_ID, target, HAND_SPEEDS)
        await asyncio.sleep(wait_s)

        feedback = None
        try:
            feedback = await self.hand_client.get_finger_positions(HAND_SLAVE_ID)
            print(f"right hand pose {name} target={target} feedback={list(feedback)}")
        except Exception as exc:
            print(f"right hand pose {name} feedback read failed: {exc}")

        if name == "thumb_ready" and self.require_thumb_feedback:
            self.validate_thumb_aux_ready(feedback)
        return feedback

    def validate_thumb_aux_ready(self, feedback):
        if feedback is None:
            raise RuntimeError("thumb_ready failed: cannot read right ThumbAux feedback")
        thumb_aux = int(feedback[THUMB_AUX_INDEX])
        if thumb_aux < THUMB_AUX_READY_MIN:
            raise RuntimeError(
                "thumb_ready failed: right ThumbAux did not reach the perpendicular target; "
                f"feedback[{THUMB_AUX_INDEX}]={thumb_aux}, required>={THUMB_AUX_READY_MIN}"
            )

    async def run(self, wait_open, wait_thumb, wait_grasp):
        await self.connect()
        try:
            print("sequence: open -> thumb_ready -> bottle -> open")
            await self.send_pose("open", wait_open)
            await self.send_pose("thumb_ready", wait_thumb)
            await self.send_pose("bottle", wait_grasp)
            await self.send_pose("open", wait_open)
        finally:
            self.close()


def parse_args():
    parser = argparse.ArgumentParser(description="Right Revo2 thumb pre-grasp diagnostic; robot arm will not move.")
    parser.add_argument("--hand-port", default=HAND_PORT, help=f"right Revo2 serial port, default: {HAND_PORT}")
    parser.add_argument("--wait-open", type=float, default=3.0, help="seconds to wait after open pose")
    parser.add_argument("--wait-thumb", type=float, default=5.0, help="seconds to wait after thumb_ready pose")
    parser.add_argument("--wait-grasp", type=float, default=4.0, help="seconds to wait after bottle pose")
    parser.add_argument("--connect-retries", type=int, default=CONNECT_RETRIES, help="right hand connection attempts")
    parser.add_argument("--connect-retry-delay", type=float, default=CONNECT_RETRY_DELAY_S, help="seconds between retries")
    parser.add_argument(
        "--continue-on-thumb-feedback-fail",
        action="store_true",
        help="continue to bottle pose even if ThumbAux feedback does not reach the threshold",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print("WARNING: right Revo2 hand will move; robot arm will not move.")
    input("Press Enter to continue...")
    tester = RightHandThumbTest(
        hand_port=args.hand_port,
        require_thumb_feedback=not args.continue_on_thumb_feedback_fail,
        connect_retries=args.connect_retries,
        connect_retry_delay_s=args.connect_retry_delay,
    )
    asyncio.run(tester.run(args.wait_open, args.wait_thumb, args.wait_grasp))
