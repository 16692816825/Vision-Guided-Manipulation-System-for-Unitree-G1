import asyncio
import sys
from revo2_utils import *

SLAVE_ID = 0x7e
PORT = "/dev/ttyUSB1"
BAUDRATE = libstark.Baudrate.Baud460800

# Ë³Ðò£º[thumb, thumb_aux, index, middle, ring, pinky]
POSES = {
    "open": [0, 0, 0, 0, 0, 0],

    # ÏÈÈÃ´óÄ´Ö¸ÍêÈ«ÕÅ¿ª
    "thumb_open_max": [0, 0, 0, 0, 0, 0],

    # ÔÙÈÃ´óÄ´Ö¸×ªµ½½Ó½ü´¹Ö±ÓÚÊÖÕÆ
    "thumb_ready": [80, 1000, 0, 0, 0, 0],

    # »Øµ½Ç°Ò»¸ö·½°¸£º²»ÔÙÏÈÇá´¥ÔÙ¶þ´Î·¢Á¦£¬Ö±½ÓÐÎ³ÉÎÕÆ¿×ËÌ¬
    "bottle": [320, 700, 560, 560, 540, 540],
}

SPEEDS = {
    "open": [250, 250, 250, 250, 250, 250],
    "thumb_open_max": [220, 220, 220, 220, 220, 220],
    "thumb_ready": [180, 180, 180, 180, 180, 180],
    "bottle": [180, 180, 180, 180, 180, 180],
}

async def send_pose(client, name, wait_s):
    positions = POSES[name]
    speeds = SPEEDS[name]
    print("send", name, positions, "speeds", speeds)
    await client.set_finger_positions_and_speeds(SLAVE_ID, positions, speeds)
    await asyncio.sleep(wait_s)

async def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "sequence"

    if mode not in ["sequence"] + list(POSES.keys()):
        print("usage: python3 left_hand_grasp_pose_test.py sequence|open|thumb_open_max|thumb_ready|bottle")
        sys.exit(1)

    client = await libstark.modbus_open(PORT, BAUDRATE)
    if not client:
        print("failed to open", PORT)
        sys.exit(1)

    try:
        info = await client.get_device_info(SLAVE_ID)
        print("Left hand:", info.description)

        await client.set_finger_unit_mode(SLAVE_ID, libstark.FingerUnitMode.Normalized)

        if mode == "sequence":
            await send_pose(client, "open", 0.8)
            await send_pose(client, "thumb_open_max", 0.5)
            await send_pose(client, "thumb_ready", 0.8)
            await send_pose(client, "bottle", 4.0)
        else:
            await send_pose(client, mode, 2.0)

    finally:
        libstark.modbus_close(client)

if __name__ == "__main__":
    asyncio.run(main())
