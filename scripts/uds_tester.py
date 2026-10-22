#!/usr/bin/env python3
"""Command-line UDS tester for the virtual cruise control ECU.

Each positional argument is one command; commands run in order on one connection
(so a session change applies to the following commands):

    ./scripts/uds_tester.py info
    ./scripts/uds_tester.py live
    ./scripts/uds_tester.py "session extended" "read-did F18C 0200"
    ./scripts/uds_tester.py dtc "dtc-count 08" clear-dtc
    ./scripts/uds_tester.py "reset hard" "read-did 0202"
    ./scripts/uds_tester.py "raw 22 F1 90" "raw 3E 00"
    ./scripts/uds_tester.py "functional 3E00"
    ./scripts/uds_tester.py "benchmark 200"

Commands: info, live, session default|extended, read-did DID..., dtc [MASK], dtc-count MASK,
dtc-supported, clear-dtc [GROUP], reset hard|soft, tester-present, raw HEX...,
functional HEX, benchmark N
"""

import argparse
import pathlib
import shlex
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "simulator"))
from vecu_sim.can_bus import open_bus  # noqa: E402
from vecu_sim.isotp import IsoTpEndpoint, IsoTpError  # noqa: E402
from vecu_sim.udp_can_bus import DEFAULT_GROUP, DEFAULT_PORT  # noqa: E402
from vecu_sim.uds_client import (DIDS, DTCS, NRC_NAMES, NegativeResponseError, UdsClient,  # noqa: E402
                                 decode_did, dtc_to_text, status_to_text)

SESSIONS = {"default": 0x01, "extended": 0x03}
RESETS = {"hard": 0x01, "soft": 0x03}
STATE_NAMES = {0: "Off", 1: "Standby", 2: "Active", 3: "Override", 4: "Fault"}


def show_dids(client, dids):
    for did, raw in client.read_data_by_identifier(*dids).items():
        d = DIDS[did]
        value = decode_did(did, raw)
        extra = f" ({STATE_NAMES.get(value)})" if did == 0x0101 else ""
        print(f"  0x{did:04X} {d.name:<24} = {value}{(' ' + d.unit) if d.unit else ''}{extra}")


def show_dtcs(entries):
    if not entries:
        print("  (none)")
    for code, status in entries:
        print(f"  {dtc_to_text(code)}  0x{code:06X}  status=0x{status:02X} [{status_to_text(status)}]  "
              f"{DTCS.get(code, '')}")


def run_command(client, isotp, words):
    cmd, args = words[0], words[1:]
    if cmd == "info":
        show_dids(client, [0xF190, 0xF187, 0xF189, 0xF186])
    elif cmd == "live":
        show_dids(client, [0x0100, 0x0101, 0x0102, 0x0103, 0x0104, 0x0201, 0x0202])
    elif cmd == "session":
        p2, p2_star = client.diagnostic_session_control(SESSIONS[args[0]])
        print(f"  session={args[0]} P2={p2} ms P2*={p2_star} ms")
    elif cmd == "read-did":
        show_dids(client, [int(a, 16) for a in args])
    elif cmd == "dtc":
        show_dtcs(client.read_dtc_by_status_mask(int(args[0], 16) if args else 0xFF))
    elif cmd == "dtc-count":
        print(f"  {client.read_dtc_count(int(args[0], 16))} DTC(s) match mask 0x{int(args[0], 16):02X}")
    elif cmd == "dtc-supported":
        show_dtcs(client.read_supported_dtcs())
    elif cmd == "clear-dtc":
        client.clear_diagnostic_information(int(args[0], 16) if args else 0xFFFFFF)
        print("  cleared")
    elif cmd == "reset":
        client.ecu_reset(RESETS[args[0]])
        print(f"  {args[0]} reset accepted")
    elif cmd == "tester-present":
        client.tester_present()
        print("  ok")
    elif cmd == "raw":
        rsp = client.request(bytes.fromhex("".join(args)))
        print(f"  response: {rsp.hex(' ').upper()}")
    elif cmd == "functional":
        payload = bytes.fromhex("".join(args))
        isotp.send(payload, 0x7DF)
        rsp = isotp.recv(0.3)
        print(f"  response: {rsp.hex(' ').upper() if rsp else '(none)'}")
    elif cmd == "benchmark":
        n = int(args[0]) if args else 100
        for label, dids in (("single-frame 22 0100", [0x0100]), ("multi-frame 22 F190", [0xF190])):
            times = []
            for _ in range(n):
                client.read_data_by_identifier(*dids)
                times.append(client.last_response_time_ms)
            times.sort()
            print(f"  {label:<22} n={n} min={times[0]:.2f} median={statistics.median(times):.2f} "
                  f"p95={times[int(0.95 * (n - 1))]:.2f} max={times[-1]:.2f} ms")
    else:
        raise ValueError(f"unknown command {cmd!r}")
    if client.last_response_time_ms is not None and cmd not in ("benchmark", "functional"):
        print(f"  (response time {client.last_response_time_ms:.2f} ms)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("commands", nargs="+")
    parser.add_argument("--bus", default="auto", choices=["auto", "udp", "socketcan"])
    parser.add_argument("--iface", default="vcan0")
    parser.add_argument("--udp-group", default=DEFAULT_GROUP)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    bus, description = open_bus(args.bus, args.iface, args.udp_group, args.udp_port)
    isotp = IsoTpEndpoint(bus, tx_id=0x7E0, rx_id=0x7E8)
    client = UdsClient(isotp)
    print(f"[tester] bus={description} request=0x7E0 response=0x7E8")
    status = 0
    try:
        for command in args.commands:
            words = shlex.split(command)
            print(f"> {command}")
            client.last_response_time_ms = None
            try:
                run_command(client, isotp, words)
            except NegativeResponseError as error:
                print(f"  negative response: 0x{error.nrc:02X} {NRC_NAMES.get(error.nrc, '')}")
                status = 1
            except (TimeoutError, IsoTpError) as error:
                print(f"  error: {error}")
                status = 2
    finally:
        bus.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
