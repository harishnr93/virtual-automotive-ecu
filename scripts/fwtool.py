#!/usr/bin/env python3
"""Firmware image tool for the virtual ECU: build, inspect and (for tests) damage images.

    ./scripts/fwtool.py build --version 0.7.0 --out /tmp/fw_070.bin
    ./scripts/fwtool.py build --version 0.8.0 --security 2 --size 131072 --out /tmp/fw_080.bin
    ./scripts/fwtool.py build --version 0.9.0 --defect --out /tmp/fw_bad.bin   # hangs after boot
    ./scripts/fwtool.py inspect /tmp/fw_070.bin
    ./scripts/fwtool.py inspect /tmp/ecu_flash.bin --slot A                   # ECU flash file
    ./scripts/fwtool.py corrupt /tmp/fw_070.bin --offset 1000 --out /tmp/fw_corrupt.bin

The payload is deterministic filler standing in for program code. Integrity is protected by
CRC-32 only: this detects accidental corruption, it does NOT prove who built the image.
"""

import argparse
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "simulator"))
from vecu_sim import firmware_image as fw  # noqa: E402

CRUISE_ECU_HARDWARE_ID = 0x0000CC01


def cmd_build(args) -> int:
    version = fw.parse_version(args.version)
    header = fw.ImageHeader(hardware_id=args.hardware_id, version=version, security_version=args.security,
                            build_time=int(time.time()) if args.build_time is None else args.build_time,
                            flags=fw.FLAG_SIMULATED_DEFECT_HANG if args.defect else 0)
    seed = (version[0] << 16) | (version[1] << 8) | version[2]
    image = fw.build_image(header, fw.make_payload(args.size, seed))
    pathlib.Path(args.out).write_bytes(image)
    print(f"built {args.out}: version {header.version_text} security {header.security_version} "
          f"hw 0x{header.hardware_id:08X} payload {header.payload_size} B crc32 0x{header.payload_crc32:08X} "
          f"image {len(image)} B{' DEFECT' if args.defect else ''}")
    return 0


def cmd_inspect(args) -> int:
    data = pathlib.Path(args.file).read_bytes()
    if args.slot:
        offset = fw.SLOT_ADDRESS["AB".index(args.slot)]
        data = data[offset:offset + fw.SLOT_SIZE]
    try:
        header = fw.verify_image(data)
    except fw.ImageFormatError as error:
        print(f"INVALID: {error.error}")
        return 1
    print(f"valid image: version {header.version_text} security {header.security_version} "
          f"hw 0x{header.hardware_id:08X} payload {header.payload_size} B crc32 0x{header.payload_crc32:08X} "
          f"build_time {header.build_time} flags 0x{header.flags:X}")
    return 0


def cmd_corrupt(args) -> int:
    data = bytearray(pathlib.Path(args.file).read_bytes())
    if not 0 <= args.offset < len(data):
        print(f"offset {args.offset} outside the image ({len(data)} B)")
        return 2
    data[args.offset] ^= 1 << args.bit
    pathlib.Path(args.out).write_bytes(bytes(data))
    print(f"flipped bit {args.bit} of byte {args.offset}: {args.out}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build", help="build an image")
    build.add_argument("--version", required=True, help="MAJOR.MINOR.PATCH")
    build.add_argument("--security", type=int, default=1, help="security version (anti-rollback), default 1")
    build.add_argument("--hardware-id", type=lambda s: int(s, 0), default=CRUISE_ECU_HARDWARE_ID)
    build.add_argument("--size", type=int, default=64 * 1024, help="payload size in bytes (default 65536)")
    build.add_argument("--build-time", type=int, default=None)
    build.add_argument("--defect", action="store_true", help="TEST ONLY: image makes the Control task hang")
    build.add_argument("--out", required=True)
    inspect = sub.add_parser("inspect", help="verify and print an image (or a slot of an ECU flash file)")
    inspect.add_argument("file")
    inspect.add_argument("--slot", choices=["A", "B"])
    corrupt = sub.add_parser("corrupt", help="flip one bit (test images)")
    corrupt.add_argument("file")
    corrupt.add_argument("--offset", type=int, required=True)
    corrupt.add_argument("--bit", type=int, default=0, choices=range(8))
    corrupt.add_argument("--out", required=True)
    args = parser.parse_args()
    return {"build": cmd_build, "inspect": cmd_inspect, "corrupt": cmd_corrupt}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
