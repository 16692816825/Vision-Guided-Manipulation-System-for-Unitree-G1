import asyncio
import sys
from revo2_utils import *


SLAVE_ID = 0x7e
PORT = "/dev/ttyUSB1"
BAUDRATE = libstark.Baudrate.Baud460800

# Order: [thumb, thumb_aux, index, middle, ring, pinky]
POSES = {
    "open": [0, 0, 0, 0, 0, 0],
    "thumb_open_max": [0, 0, 0, 0, 0, 0],
    "thumb_ready": [0, 1000, 0, 0, 0, 0],
    "bottle": [180, 850, 480, 560, 540, 420],
}

SPEEDS = [300] * 6


async def send_pose(client, name, wait_s):
    positions = POSES[name]
    print("send pose:", name, positions)
    await client.set_finger_positions_and_speeds(SLAVE_ID, positions, SPEEDS)
    await asyncio.sleep(wait_s)


async def main():
    pose = sys.argv[1] if len(sys.argv) > 1 else "open"
    valid = list(POSES.keys()) + ["grasp"]
    if pose not in valid:
        print("usage: python3 left_hand_safe_once.py " + "|".join(valid))
        sys.exit(1)

    client = await libstark.modbus_open(PORT, BAUDRATE)
    if not client:
        print(f"failed to open {PORT}")
        sys.exit(1)

    try:
        info = await client.get_device_info(SLAVE_ID)
        if not info:
            print(f"failed to get left hand info, id={SLAVE_ID}")
            sys.exit(1)

        print("Left hand:", info.description)
        await client.set_finger_unit_mode(SLAVE_ID, libstark.FingerUnitMode.Normalized)

        if pose == "grasp":
            await send_pose(client, "thumb_open_max", 1.0)
            await send_pose(client, "thumb_ready", 2.0)
            await send_pose(client, "bottle", 2.0)
        else:
            await send_pose(client, pose, 2.0)
    finally:
        libstark.modbus_close(client)


if __name__ == "__main__":
    asyncio.run(main())
