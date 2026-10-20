#!/usr/bin/env python3
"""cansend-like frame injector for the UDP software CAN bus.

Examples:
    ./scripts/vbus_send.py 123#DEADBEEF          # 11-bit ID
    ./scripts/vbus_send.py 18DAF110#0102         # 29-bit ID (8 hex digits)
    ./scripts/vbus_send.py 110#0002500000000000 -r 10 -i 0.05   # repeat 10x every 50 ms
"""

import argparse
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "simulator"))
from vecu_sim.udp_can_bus import DEFAULT_GROUP, DEFAULT_PORT, CanFrame, UdpCanBus  # noqa: E402


def parse_frame(text: str) -> CanFrame:
    if "#" not in text:
        raise ValueError("expected <id>#<data>, e.g. 123#DEADBEEF")
    can_id, data = text.split("#", 1)
    return CanFrame(int(can_id, 16), bytes.fromhex(data), extended=len(can_id) == 8)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("frame", help="<id>#<data> in can-utils notation")
    parser.add_argument("--group", default=DEFAULT_GROUP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("-r", "--repeat", type=int, default=1)
    parser.add_argument("-i", "--interval", type=float, default=0.0, help="seconds between repeats")
    args = parser.parse_args()

    try:
        frame = parse_frame(args.frame)
    except ValueError as error:
        parser.error(str(error))
    with UdpCanBus(args.group, args.port) as bus:
        for i in range(args.repeat):
            bus.send(frame)
            if i + 1 < args.repeat:
                time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    sys.exit(main())
