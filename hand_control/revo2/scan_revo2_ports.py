import asyncio
import sys
from revo2_utils import *

PORTS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]
IDS = [0x7e, 0x7f, 1, 2, 126, 127]
BAUDS = [
    ("460800", libstark.Baudrate.Baud460800),
    ("115200", libstark.Baudrate.Baud115200),
]

async def try_one(port, baud_name, baud, sid):
    client = None
    try:
        client = await libstark.modbus_open(port, baud)
        if not client:
            print("FAIL open", port, baud_name, "id", sid)
            return
        info = await client.get_device_info(sid)
        if info:
            print("FOUND port =", port, "baud =", baud_name, "id =", sid, "hex =", hex(sid))
            print("  info =", info.description)
    except Exception as e:
        print("no", port, baud_name, "id", sid, ":", str(e).splitlines()[0])
    finally:
        if client:
            try:
                libstark.modbus_close(client)
            except Exception:
                pass

async def main():
    for port in PORTS:
        for baud_name, baud in BAUDS:
            for sid in IDS:
                await try_one(port, baud_name, baud, sid)

if __name__ == "__main__":
    asyncio.run(main())
