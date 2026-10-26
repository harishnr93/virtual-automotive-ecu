"""System tests of the firmware update (Phase 6): real ECU process, flash file, UDS over the
software CAN bus, images built by the independent Python implementation (vecu_sim.firmware_image).

  1. successful update -> trial boot -> confirmation -> persists across a process restart
  2. rejected images (corrupt, downgrade, wrong hardware, security rollback) and protocol errors
  3. interrupted update: ECU killed (SIGKILL) in the middle of the download -> old image boots
  4. faulty new firmware: watchdog exits, an external supervisor restarts the ECU, the
     bootloader counts the failed trial boots and rolls back after 3 attempts
  5. the programming session is refused while driving and accepted at standstill

Measured timings are printed (pytest -s) and recorded in docs/phase6_firmware_update.md.
"""

import os
import pathlib
import random
import signal
import subprocess
import sys
import time

import pytest

from vecu_sim import firmware_image as fw
from vecu_sim.isotp import IsoTpEndpoint, IsoTpError
from vecu_sim.udp_can_bus import UdpCanBus
from vecu_sim.uds_client import FirmwareCheckError, NegativeResponseError, UdsClient, SESSIONS

REPO = pathlib.Path(__file__).resolve().parents[2]
BIN_DIR = pathlib.Path(os.environ.get("VECU_BIN_DIR", REPO / "build" / "bin"))
ECU = BIN_DIR / "vecu_cruise_ecu"

pytestmark = pytest.mark.skipif(not ECU.exists(), reason=f"{ECU} not built")

HW = 0x0000CC01
DTC_FIRMWARE = 0xF00096
CONFIRMED = 0x08
WATCHDOG_EXIT = 75


# --------------------------------------------------------------------------- helpers ----

def start_ecu(port, flash, *extra, duration=60):
    args = [str(ECU), "--bus", "udp", "--udp-port", str(port), "--duration", str(duration), "--flash", str(flash),
            *extra]
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)


def stop(ecu, expected=0):
    """Ends the ECU (SIGTERM = orderly shutdown) and checks the exit code; a sanitizer report
    in the sanitizer builds also makes the exit code non-zero."""
    if ecu.poll() is None:
        ecu.send_signal(signal.SIGTERM)
    _, log = ecu.communicate(timeout=30)
    assert ecu.returncode == expected, log[-4000:]
    return log


def image(version, security=1, hw=HW, size=8 * 1024, defect=False):
    header = fw.ImageHeader(hardware_id=hw, version=version, security_version=security, build_time=1790000000,
                            flags=fw.FLAG_SIMULATED_DEFECT_HANG if defect else 0)
    return fw.build_image(header, fw.make_payload(size, (version[0] << 16) | (version[1] << 8) | version[2]))


def wait_ready(client, timeout=5.0):
    """Waits until the ECU answers; returns the running software version (DID F189)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return client.read_data_by_identifier(0xF189)[0xF189].decode()
        except TimeoutError:
            time.sleep(0.1)
    raise TimeoutError("ECU did not answer")


def fw_status(client):
    raw = client.read_data_by_identifier(0xF189, 0x0400, 0x0401, 0x0402)
    return {"version": raw[0xF189].decode(), **fw.decode_boot_status(raw[0x0400]),
            "update": fw.decode_update_status(raw[0x0401]), **fw.decode_versions(raw[0x0402])}


@pytest.fixture
def env(tmp_path):
    port = random.randint(40000, 59999)
    bus = UdpCanBus(port=port)
    client = UdsClient(IsoTpEndpoint(bus))
    yield port, tmp_path / "flash.bin", client
    bus.close()


# ------------------------------------------------------------ 1. successful update ----

def test_update_trial_confirmation_and_persistence(env):
    port, flash, client = env
    ecu = start_ecu(port, flash, "--confirm-after", "1.5")
    try:
        assert wait_ready(client) == "00.06.00"
        # Cross-implementation check: the factory image the C++ ECU wrote into its flash file
        # verifies with the independent Python implementation.
        slot_a = flash.read_bytes()[fw.SLOT_ADDRESS[0]:fw.SLOT_ADDRESS[0] + fw.SLOT_SIZE]
        factory = fw.verify_image(slot_a)
        assert factory.version == (0, 6, 0) and factory.hardware_id == HW
        assert slot_a[64:64 + factory.payload_size] == fw.make_payload(factory.payload_size, 0x000600)

        new = image((0, 7, 0), size=64 * 1024)
        timing = client.flash_image(new)
        status = wait_ready(client) and fw_status(client)
        assert status["version"] == "00.07.00"
        assert (status["running_slot"], status["state_b"], status["boot_attempts"]) == ("B", "Trial", 1)
        assert status["last_boot_event"] == "TrialBoot" and status["state_a"] == "Confirmed"

        confirmed_at = None
        start = time.monotonic()
        while time.monotonic() - start < 5.0:
            if fw_status(client)["state_b"] == "Confirmed":
                confirmed_at = time.monotonic() - start
                break
            time.sleep(0.1)
        assert confirmed_at is not None, "trial image was not confirmed"
        # Never before the healthy period (a race once confirmed a trial image right after the reset).
        assert confirmed_at >= 1.0, f"confirmed only {confirmed_at:.2f} s after the reset"
        print(f"\n[firmware] {len(new)} B image: erase {timing['erase_s'] * 1000:.1f} ms, download "
              f"{timing['download_s']:.2f} s ({timing['throughput_kib_s']:.1f} KiB/s), check "
              f"{timing['check_s'] * 1000:.1f} ms, total {timing['total_s']:.2f} s; confirmed "
              f"{confirmed_at:.2f} s after reset (--confirm-after 1.5)")
    finally:
        log = stop(ecu)
    assert "FW_CONFIRMED version=00.07.00" in log
    print("[firmware] " + next(line for line in log.splitlines() if "FW_SUMMARY" in line).split("] ")[-1])

    # A new process = power cycle: the confirmed image boots normally.
    ecu = start_ecu(port, flash)
    try:
        assert wait_ready(client) == "00.07.00"
        status = fw_status(client)
        assert (status["last_boot_event"], status["state_b"]) == ("Normal", "Confirmed")
        assert (status["slot_a"], status["slot_b"]) == ("00.06.00", "00.07.00")
    finally:
        stop(ecu)


# ------------------------------------------------------------ 2. rejected images ----

def test_invalid_images_and_protocol_errors_are_rejected(env):
    port, flash, client = env
    ecu = start_ecu(port, flash)
    try:
        assert wait_ready(client) == "00.06.00"
        corrupt = bytearray(image((0, 7, 0)))
        corrupt[5000] ^= 0x10  # payload bit flip
        header_corrupt = bytearray(image((0, 7, 0)))
        header_corrupt[13] ^= 0x01  # version byte changed after the header CRC was computed
        cases = [(bytes(corrupt), "PayloadCrcMismatch"), (bytes(header_corrupt), "HeaderCrcMismatch"),
                 (image((0, 5, 0)), "VersionNotNewer"), (image((0, 6, 0)), "VersionNotNewer"),
                 (image((0, 7, 0), hw=0x0000BB02), "WrongHardware"),
                 (image((0, 7, 0), security=0), "SecurityVersionRollback")]
        for data, expected in cases:
            with pytest.raises(FirmwareCheckError) as error:
                client.flash_image(data)
            assert error.value.reason == expected
            status = fw_status(client)
            assert status["update"]["state"] == "Failed" and status["update"]["last_image_error"] == expected
            assert status["state_b"] == "Invalid" and status["version"] == "00.06.00"
            client.diagnostic_session_control(SESSIONS["default"])

        # Protocol errors (each NRC is what the tester sees).
        def nrc(payload):
            with pytest.raises(NegativeResponseError) as error:
                client.request(payload)
            return error.value.nrc

        assert nrc(bytes([0x34, 0x00, 0x44, 0, 0, 0, 0, 0, 0, 0x20, 0])) == 0x7F  # default session
        assert nrc(bytes([0x10, 0x02])) == 0x7E                                    # not from default
        client.diagnostic_session_control(SESSIONS["extended"])
        client.diagnostic_session_control(SESSIONS["programming"])
        assert nrc(bytes([0x19, 0x0A])) == 0x7F                                    # no DTC services
        assert nrc(bytes([0x34, 0x00, 0x44, 0, 0, 0, 0, 0, 0, 0x20, 0])) == 0x70  # not erased
        assert nrc(bytes([0x36, 0x01, 0xAA])) == 0x24                              # no download active
        client.routine_control(0xFF00)
        assert nrc(bytes([0x34, 0x00, 0x44, 0, 0, 0, 0, 0x7F, 0, 0, 0])) == 0x31  # larger than a slot
        assert client.request_download(0, 1024) == 0x0FFF
        assert nrc(bytes([0x36, 0x02, 0xAA])) == 0x73                              # counter must start at 1
        assert nrc(bytes([0x37])) == 0x24                                          # incomplete
        assert nrc(bytes([0x31, 0x01, 0xFF, 0x01])) == 0x24                        # check before exit
        client.ecu_reset(0x01)
        assert wait_ready(client) == "00.06.00"
    finally:
        log = stop(ecu)
    assert "BOOT event=Normal" in log and "TrialBoot" not in log


# ------------------------------------------------------- 3. interrupted update ----

def test_power_loss_during_download_keeps_the_old_image(env):
    port, flash, client = env
    ecu = start_ecu(port, flash)
    new = image((0, 7, 0), size=48 * 1024)
    try:
        assert wait_ready(client) == "00.06.00"

        def kill_halfway(done, total):
            if done > total // 2 and ecu.poll() is None:
                ecu.kill()  # SIGKILL: no shutdown code runs, like pulling the power
                ecu.wait()

        with pytest.raises((TimeoutError, IsoTpError)):  # no flow control / no response
            client.flash_image(new, progress=kill_halfway)
    finally:
        stop(ecu, expected=-signal.SIGKILL)

    ecu = start_ecu(port, flash)
    try:
        assert wait_ready(client) == "00.06.00"
        status = fw_status(client)
        assert status["last_boot_event"] == "Normal" and status["running_slot"] == "A"
        assert status["state_b"] == "Receiving"  # half-written: never bootable
        # The tester retries: the update completes normally.
        client.flash_image(new)
        assert wait_ready(client) == "00.07.00"
        assert fw_status(client)["state_b"] == "Trial"
    finally:
        stop(ecu)


def test_tester_disappearing_mid_download_aborts_via_s3_timeout(env):
    port, flash, client = env
    ecu = start_ecu(port, flash)
    try:
        assert wait_ready(client) == "00.06.00"
        client.diagnostic_session_control(SESSIONS["extended"])
        client.diagnostic_session_control(SESSIONS["programming"])
        client.routine_control(0xFF00)
        client.request_download(0, 20000)
        client.transfer_data(1, bytes(4093))
        time.sleep(5.6)  # S3 = 5 s without requests
        status = fw_status(client)
        assert status["update"]["state"] == "Failed" and status["update"]["bytes_received"] == 4093
        assert client.read_data_by_identifier(0xF186)[0xF186][0] == 0x01  # back in the default session
    finally:
        log = stop(ecu)
    assert "FW_DOWNLOAD_ABORTED" in log


# ---------------------------------------------------- 4. faulty firmware -> rollback ----

def test_faulty_firmware_is_rolled_back_after_three_failed_boots(env):
    port, flash, client = env
    ecu = start_ecu(port, flash, "--watchdog-action", "exit", "--confirm-after", "5")
    try:
        assert wait_ready(client) == "00.06.00"
        client.flash_image(image((0, 9, 0), defect=True))
        attempts = []
        # External supervisor (here: the test) restarts the ECU after each watchdog exit, like a
        # hardware watchdog reset; the bootloader runs on every start.
        for _ in range(4):
            status = wait_ready(client) and fw_status(client)
            attempts.append((status["version"], status["last_boot_event"], status["boot_attempts"]))
            if status["last_boot_event"] == "Rollback":
                break
            assert ecu.wait(timeout=10) == WATCHDOG_EXIT
            ecu.stderr.read()
            ecu = start_ecu(port, flash, "--watchdog-action", "exit", "--confirm-after", "5")
        print(f"\n[firmware] boots after flashing the defective image: {attempts}")
        assert attempts[:3] == [("00.09.00", "TrialBoot", 1), ("00.09.00", "TrialBoot", 2),
                                ("00.09.00", "TrialBoot", 3)]
        assert attempts[3][:2] == ("00.06.00", "Rollback")
        status = fw_status(client)
        assert (status["running_slot"], status["state_a"], status["state_b"]) == ("A", "Confirmed", "Invalid")
        assert dict(client.read_supported_dtcs())[DTC_FIRMWARE] & CONFIRMED
        time.sleep(2.0)  # the old image runs normally: no further watchdog exit
        assert ecu.poll() is None
    finally:
        log = stop(ecu)
    assert "BOOT event=Rollback" in log and "WATCHDOG" in log


# ------------------------------------------------- 5. programming only at standstill ----

def test_programming_session_only_at_standstill_with_bus_traffic(env):
    port, flash, client = env
    ecu = start_ecu(port, flash)
    sim = subprocess.Popen([sys.executable, "-m", "vecu_sim.simulator", "--scenario",
                            str(REPO / "config" / "scenarios" / "workshop_flash.json"), "--bus", "udp",
                            "--udp-port", str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           env={**os.environ, "PYTHONPATH": str(REPO / "simulator")})
    try:
        assert wait_ready(client) == "00.06.00"
        deadline = time.monotonic() + 5.0
        while client.read_data_by_identifier(0x0101)[0x0101][0] != 2:  # cruise Active
            assert time.monotonic() < deadline, "cruise did not engage"
            time.sleep(0.1)
        client.diagnostic_session_control(SESSIONS["extended"])
        with pytest.raises(NegativeResponseError) as error:
            client.diagnostic_session_control(SESSIONS["programming"])
        assert error.value.nrc == 0x22  # conditionsNotCorrect: vehicle moving
        client.diagnostic_session_control(SESSIONS["default"])

        deadline = time.monotonic() + 15.0
        while int.from_bytes(client.read_data_by_identifier(0x0100)[0x0100], "big") > 50:  # > 0.5 km/h
            assert time.monotonic() < deadline, "vehicle did not stop"
            time.sleep(0.2)
        timing = client.flash_image(image((0, 7, 0), size=32 * 1024))  # with sensor traffic on the bus
        assert wait_ready(client) == "00.07.00"
        print(f"\n[firmware] flashing at standstill with simulator traffic: {timing['throughput_kib_s']:.1f} KiB/s")
    finally:
        sim.terminate()
        sim.wait(timeout=10)
        log = stop(ecu)
    assert "UDS programming session refused" in log
