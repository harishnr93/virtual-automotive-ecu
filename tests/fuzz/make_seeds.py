#!/usr/bin/env python3
"""Writes the seed corpora of the fuzz targets (tests/fuzz/corpus/<target>/).

Seeds are small, *valid* examples of each input format, so the fuzzer starts deep inside the
parsers instead of having to discover magic numbers and CRCs by chance. They are generated
here (not hand-edited) so their meaning stays reviewable:

    python3 tests/fuzz/make_seeds.py [--nvm-seed FILE]   # FILE: an NVM file written by the ECU
"""

import argparse
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "simulator"))
from vecu_sim import firmware_image as fw  # noqa: E402
from vecu_sim.can_bus import CanFrame  # noqa: E402
from vecu_sim.udp_can_bus import encode  # noqa: E402


def isotp_record(dt_ms: int, dlc: int, data: bytes, send: bool = False, functional: bool = False) -> bytes:
    flags = (0x80 if send else 0) | (dt_ms & 0x7F)
    return bytes([flags, (0x80 if functional else 0) | dlc]) + data.ljust(8, b"\xcc")


def uds(*requests: bytes, flags: int = 0) -> bytes:
    return bytes([flags]) + b"".join(bytes([len(r)]) + r for r in requests)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nvm-seed", type=pathlib.Path)
    args = parser.parse_args()
    seeds = {
        "isotp": {
            "single_frame": isotp_record(1, 8, bytes([0x02, 0x10, 0x03])),
            "first_and_consecutive": isotp_record(1, 8, bytes([0x10, 0x0B, 0x2E, 0xF1, 0x90, 1, 2, 3])) +
            isotp_record(2, 8, bytes([0x21, 4, 5, 6, 7, 8, 9, 10])),
            "send_then_flow_control": isotp_record(1, 0, bytes([0x00, 0x40]), send=True) +
            isotp_record(1, 3, bytes([0x30, 0x00, 0x00])) + isotp_record(200, 3, bytes([0x30, 0x02, 0x05])),
        },
        "uds_server": {
            "sessions_and_reads": uds(b"\x10\x03", b"\x22\xf1\x90\x01\x00", b"\x19\x02\xff", b"\x3e\x80",
                                      b"\x14\xff\xff\xff"),
            "programming_download": uds(b"\x10\x03", b"\x10\x02", b"\x31\x01\xff\x00", b"\x34\x00\x44" + bytes(7) + b"\x10",
                                        b"\x36\x01" + bytes(8), b"\x36\x02" + bytes(8), b"\x37", b"\x11\x01"),
            "functional": uds(b"\x3e\x00", b"\x10\x01", flags=1),
        },
        "udp_wire": {
            "vehicle_dynamics": encode(CanFrame(0x100, bytes([0x40, 0x1f, 0, 0, 1, 0, 0, 0x30])), 0x12345678),
            "extended_id": encode(CanFrame(0x18DAF110, bytes([1, 2, 3]), True), 7),
        },
        "firmware": {
            "valid_image_slot_a": b"\x01" + fw.build_image(fw.ImageHeader(0x0000CC01, (1, 2, 3)), fw.make_payload(300, 5)),
            "valid_image_slot_b": b"\x00" + fw.build_image(fw.ImageHeader(0x0000CC01, (0, 7, 0)), fw.make_payload(64, 9)),
            "header_only": b"\x03" + fw.serialize_header(fw.ImageHeader(0x0000CC01, (9, 9, 9), payload_size=10)),
        },
        "nvm": {},
    }
    if args.nvm_seed:
        seeds["nvm"]["ecu_written"] = args.nvm_seed.read_bytes()
    for target, files in seeds.items():
        directory = HERE / "corpus" / target
        directory.mkdir(parents=True, exist_ok=True)
        for name, content in files.items():
            (directory / name).write_bytes(content)
        print(f"{target}: {len(files)} seed(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
