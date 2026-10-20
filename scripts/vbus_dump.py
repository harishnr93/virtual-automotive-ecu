#!/usr/bin/env python3
"""candump-like monitor for the UDP software CAN bus.

can-utils (candump, cansend) only work on SocketCAN interfaces. When the ECUs run on the
UDP fallback bus, use this tool instead. Output format matches 'candump -L':
    (1697812345.123456) vbus0 100#1027000000000003

Examples:
    ./scripts/vbus_dump.py                      # print all frames until Ctrl+C
    ./scripts/vbus_dump.py -f 100,200 -n 20     # only IDs 0x100/0x200, stop after 20
    ./scripts/vbus_dump.py -l trace.log -t 5    # record 5 s to a file
"""

import argparse
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "simulator"))
from vecu_sim.udp_can_bus import DEFAULT_GROUP, DEFAULT_PORT, UdpCanBus  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--group", default=DEFAULT_GROUP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("-f", "--filter", help="comma-separated hex IDs, e.g. 100,200")
    parser.add_argument("-n", "--count", type=int, default=0, help="stop after N frames")
    parser.add_argument("-t", "--time", type=float, default=0.0, help="stop after T seconds")
    parser.add_argument("-l", "--log", help="also write frames to this file")
    args = parser.parse_args()

    wanted = {int(x, 16) for x in args.filter.split(",")} if args.filter else None
    log = open(args.log, "w", encoding="ascii") if args.log else None
    end = time.monotonic() + args.time if args.time > 0 else None
    count = 0
    try:
        with UdpCanBus(args.group, args.port) as bus:
            while end is None or time.monotonic() < end:
                frame = bus.recv(timeout=0.2)
                if frame is None or (wanted is not None and frame.can_id not in wanted):
                    continue
                line = f"({time.time():017.6f}) vbus0 {frame.candump()}"
                print(line, flush=True)
                if log:
                    log.write(line + "\n")
                count += 1
                if args.count and count >= args.count:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
