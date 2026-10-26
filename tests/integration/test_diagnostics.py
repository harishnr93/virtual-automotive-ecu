"""System test of UDS diagnostics: cruise ECU + vehicle simulator (fault injection) + tester.

Three processes on one isolated bus. The C++ ISO-TP/UDS stack in the ECU is exercised by the
independent Python ISO-TP/UDS implementation in the tester. Real faults (lost speed message,
invalid speed signal) are injected by the simulator and must appear as DTCs.
"""

import json
import os
import pathlib
import random
import re
import statistics
import subprocess
import sys
import time

import pytest

from vecu_sim.isotp import IsoTpEndpoint
from vecu_sim.udp_can_bus import UdpCanBus
from vecu_sim.uds_client import DIDS, NegativeResponseError, UdsClient, decode_did

REPO = pathlib.Path(__file__).resolve().parents[2]
BIN_DIR = pathlib.Path(os.environ.get("VECU_BIN_DIR", REPO / "build" / "bin"))
ECU = BIN_DIR / "vecu_cruise_ecu"
SCENARIO = REPO / "config" / "scenarios" / "cruise_sensor_fault.json"

pytestmark = pytest.mark.skipif(not ECU.exists(), reason=f"{ECU} not built")

DTC_SPEED_MISSING = 0xC12187
DTC_SPEED_IMPLAUSIBLE = 0x403164
DTC_DRIVER_INPUTS_MISSING = 0xC14087
CONFIRMED = 0x08


def wait_for(predicate, timeout, interval=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
    return None


def expect_nrc(client, request, nrc):
    with pytest.raises(NegativeResponseError) as error:
        client.request(bytes(request))
    assert error.value.nrc == nrc, f"request {bytes(request).hex()}"


@pytest.fixture(scope="module")
def system():
    port = random.randint(40000, 59999)
    ecu = subprocess.Popen([str(ECU), "--bus", "udp", "--udp-port", str(port), "--duration", "22"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    time.sleep(0.5)
    sim = subprocess.Popen([sys.executable, "-m", "vecu_sim.simulator", "--scenario", str(SCENARIO), "--bus", "udp",
                            "--udp-port", str(port)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                           env={**os.environ, "PYTHONPATH": str(REPO / "simulator")})
    sim_start = time.monotonic()
    bus = UdpCanBus(port=port)
    isotp = IsoTpEndpoint(bus)
    client = UdpTimingClient(isotp)
    yield {"client": client, "isotp": isotp, "sim_start": sim_start, "ecu": ecu, "sim": sim}
    bus.close()
    sim.communicate(timeout=60)
    _, ecu_log = ecu.communicate(timeout=60)
    print(ecu_log[-1500:])
    times = client.times
    if times:
        print(f"[diag] response times over {len(times)} requests: min={min(times):.2f} "
              f"median={statistics.median(times):.2f} max={max(times):.2f} ms")


class UdpTimingClient(UdsClient):
    """Records every response time."""

    def __init__(self, isotp):
        super().__init__(isotp)
        self.times = []

    def request(self, payload, functional=False, expect_response=True):
        try:
            return super().request(payload, functional, expect_response)
        finally:
            if expect_response and self.last_response_time_ms is not None:
                self.times.append(self.last_response_time_ms)
                self.last_response_time_ms = None


def sim_time(system):
    return time.monotonic() - system["sim_start"]


def test_full_diagnostic_session(system):
    client, isotp = system["client"], system["isotp"]

    # --- identification (multi-frame responses) --------------------------------------------
    ident = client.read_data_by_identifier(0xF190, 0xF187, 0xF189)
    assert decode_did(0xF190, ident[0xF190]) == "VECUSIM0000000001"
    assert decode_did(0xF187, ident[0xF187]) == "VECU-CC-0001"
    assert decode_did(0xF189, ident[0xF189]) == "00.06.00"  # version of the booted image (Phase 6)

    # --- negative responses and session rules ----------------------------------------------
    expect_nrc(client, [0x22, 0xF1, 0x8C], 0x31)  # serial number: extended session only
    expect_nrc(client, [0x11, 0x03], 0x7E)        # soft reset: extended session only
    expect_nrc(client, [0x10, 0x02], 0x7E)        # programming session: only from extended (Phase 6)
    expect_nrc(client, [0x2E, 0xF1, 0x90], 0x11)  # WriteDataByIdentifier: not implemented
    expect_nrc(client, [0x22, 0xF1], 0x13)

    # --- functional addressing (0x7DF) -----------------------------------------------------
    isotp.send(bytes([0x3E, 0x00]), 0x7DF)
    assert isotp.recv(0.3) == bytes([0x7E, 0x00])
    isotp.send(bytes([0x2E, 0x00]), 0x7DF)
    assert isotp.recv(0.3) is None  # NRC 0x11 is suppressed for functional requests

    # --- live data while cruising ----------------------------------------------------------
    assert wait_for(lambda: decode_did(0x0101, client.read_data_by_identifier(0x0101)[0x0101]) == 2, 4.0), \
        "cruise never reported Active through DID 0x0101"
    live = {d: decode_did(d, v) for d, v in client.read_data_by_identifier(0x0100, 0x0102, 0x0104).items()}
    assert 75.0 <= live[0x0100] <= 85.0 and live[0x0102] == 80 and live[0x0104] == 0

    # --- every DID answers with exactly the length the tester expects ---------------------
    client.diagnostic_session_control(0x03)
    for did, description in DIDS.items():
        assert len(client.read_data_by_identifier(did)[did]) == description.length

    # --- S3 timeout: no request for > 5 s returns the ECU to the default session ----------
    assert decode_did(0xF186, client.read_data_by_identifier(0xF186)[0xF186]) == 0x03
    time.sleep(5.6)
    assert decode_did(0xF186, client.read_data_by_identifier(0xF186)[0xF186]) == 0x01

    # --- DTC from the lost speed message (simulator mutes 0x100 at t = 5.0 s) ------------
    assert sim_time(system) > 6.0
    dtcs = dict(client.read_dtc_by_status_mask(CONFIRMED))
    assert DTC_SPEED_MISSING in dtcs, dtcs
    # recovered: confirmed / failed this cycle / failed since clear, but no longer testFailed
    assert dtcs[DTC_SPEED_MISSING] == 0x2E
    assert DTC_SPEED_IMPLAUSIBLE not in dtcs

    # --- DTC from the invalid speed signal (t = 11.0 s) -----------------------------------
    def implausible_confirmed_and_over():
        status = dict(client.read_dtc_by_status_mask(CONFIRMED)).get(DTC_SPEED_IMPLAUSIBLE)
        # Wait until the fault has ended (testFailed = 0); clearing while the monitor still
        # fails would immediately re-confirm the DTC.
        return status is not None and (status & 0x01) == 0
    assert wait_for(implausible_confirmed_and_over, 8.0, 0.1)
    assert client.read_dtc_count(CONFIRMED) == 2

    # --- clear all DTCs --------------------------------------------------------------------
    client.clear_diagnostic_information(0xFFFFFF)
    assert client.read_dtc_count(CONFIRMED) == 0
    statuses = dict(client.read_supported_dtcs())
    assert statuses[DTC_SPEED_MISSING] & CONFIRMED == 0

    # --- ECU hard reset (simulated) --------------------------------------------------------
    client.diagnostic_session_control(0x03)
    client.ecu_reset(0x01)
    after = client.read_data_by_identifier(0x0202, 0xF186)
    assert decode_did(0x0202, after[0x0202]) == 1
    assert decode_did(0xF186, after[0xF186]) == 0x01  # reset returns to the default session

    # --- response time (P2 server = 50 ms) -------------------------------------------------
    times = sorted(client.times)
    print(f"[diag] p95={times[int(0.95 * (len(times) - 1))]:.2f} ms max={times[-1]:.2f} ms n={len(times)}")
    assert times[int(0.95 * (len(times) - 1))] < 50.0

    # --- both communication DTCs when the simulator stops sending --------------------------
    # The ECU outlives the 16 s scenario: 0x100 (100 ms) and 0x110 (250 ms) time out, so
    # U0121-87 and U0140-87 must both be reported as currently failing (testFailed = bit 0).
    system["sim"].wait(timeout=30)

    def both_messages_missing():
        failing = dict(client.read_dtc_by_status_mask(0x01))
        return DTC_SPEED_MISSING in failing and DTC_DRIVER_INPUTS_MISSING in failing
    assert wait_for(both_messages_missing, 3.0, 0.1)
    assert decode_did(0x0101, client.read_data_by_identifier(0x0101)[0x0101]) == 4  # Fault
    assert decode_did(0x0104, client.read_data_by_identifier(0x0104)[0x0104]) in (1, 2)


def test_ecu_log_shows_reset_and_no_transport_errors(system):
    system["sim"].communicate(timeout=60)
    _, log = system["ecu"].communicate(timeout=60)
    system["ecu_log"] = log
    assert "ECU_RESET type=hard" in log
    summary = dict(item.split("=", 1) for item in re.search(r"\] SUMMARY (ecu=.*)", log).group(1).split())
    assert summary["isotp_errors"] == "0"
    assert int(summary["uds_requests"]) >= 30
