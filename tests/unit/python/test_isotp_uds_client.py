"""Python ISO-TP and UDS client, tested against an in-memory bus (no sockets)."""

import queue
import threading

import pytest

from vecu_sim.isotp import IsoTpEndpoint, IsoTpError, decode_st_min
from vecu_sim.udp_can_bus import CanFrame
from vecu_sim.uds_client import NegativeResponseError, UdsClient, dtc_to_text, status_to_text


class Wire:
    """Two connected in-memory bus ends; each end receives what the other sends."""

    class End:
        def __init__(self):
            self.inbox = queue.Queue()
            self.peer = None
            self.sent = []

        def send(self, frame):
            self.sent.append(frame)
            self.peer.inbox.put(frame)

        def recv(self, timeout=None):
            try:
                return self.inbox.get(timeout=timeout)
            except queue.Empty:
                return None

    def __init__(self):
        self.a, self.b = Wire.End(), Wire.End()
        self.a.peer, self.b.peer = self.b, self.a


def pair(**tester_kwargs):
    wire = Wire()
    tester = IsoTpEndpoint(wire.a, tx_id=0x7E0, rx_id=0x7E8, **tester_kwargs)
    ecu = IsoTpEndpoint(wire.b, tx_id=0x7E8, rx_id=0x7E0)
    return wire, tester, ecu


def test_st_min_decoding_matches_cpp():
    assert decode_st_min(0x00) == 0 and decode_st_min(0x7F) == pytest.approx(0.127)
    assert decode_st_min(0xF1) == pytest.approx(100e-6) and decode_st_min(0xFA) == pytest.approx(0.127)


def test_single_frame_layout_matches_cpp_test_vector():
    wire, tester, _ = pair()
    tester.send(bytes([0x22, 0xF1, 0x90]))
    assert wire.a.sent[0] == CanFrame(0x7E0, bytes.fromhex("0322F190CCCCCCCC"))


@pytest.mark.parametrize("size", [8, 20, 300, 4095])
def test_multi_frame_round_trip(size):
    wire, tester, ecu = pair(block_size=4, st_min=0)
    payload = bytes(i & 0xFF for i in range(size))
    received = {}
    receiver = threading.Thread(target=lambda: received.setdefault("msg", tester.recv(2.0)))
    receiver.start()
    ecu.send(payload)
    receiver.join(5)
    assert received["msg"] == payload
    first = wire.b.sent[0]
    assert first.data[0] >> 4 == 1 and ((first.data[0] & 0xF) << 8 | first.data[1]) == size
    flow_controls = [f for f in wire.a.sent if f.data[0] >> 4 == 3]
    consecutive = (size - 6 + 6) // 7
    assert len(flow_controls) == 1 + (consecutive - 1) // 4  # BS = 4


def test_wrong_sequence_number_is_detected():
    _, tester, _ = pair()
    tester.bus.inbox.put(CanFrame(0x7E8, bytes([0x10, 0x10, 1, 2, 3, 4, 5, 6])))
    tester.bus.inbox.put(CanFrame(0x7E8, bytes([0x22, 7, 8, 9, 10, 11, 12, 13])))
    with pytest.raises(IsoTpError, match="sequence"):
        tester.recv(0.5)


def test_flow_control_timeout_and_overflow():
    wire, tester, _ = pair(n_bs=0.05)
    with pytest.raises(IsoTpError, match="N_Bs"):
        tester.send(bytes(20))
    wire.a.inbox.put(CanFrame(0x7E8, bytes([0x32, 0, 0]).ljust(8, b"\xCC")))
    with pytest.raises(IsoTpError, match="overflow"):
        tester.send(bytes(20))


def test_functional_requests_must_be_single_frames():
    wire, tester, _ = pair()
    tester.send(bytes([0x3E, 0x00]), functional_id=0x7DF)
    assert wire.a.sent[0].can_id == 0x7DF
    with pytest.raises(ValueError):
        tester.send(bytes(10), functional_id=0x7DF)


class StubServer(threading.Thread):
    """Answers requests from a dict {request: [responses...]}."""

    def __init__(self, end, answers):
        super().__init__(daemon=True)
        self.isotp = IsoTpEndpoint(end, tx_id=0x7E8, rx_id=0x7E0)
        self.answers = answers
        self.start()

    def run(self):
        while True:
            request = self.isotp.recv(5.0)
            if request is None:
                return
            for response in self.answers.get(request, []):
                self.isotp.send(response)


def test_uds_client_parses_responses_and_raises_nrcs():
    wire = Wire()
    StubServer(wire.b, {
        bytes([0x22, 0x01, 0x00, 0x01, 0x01]): [bytes([0x62, 0x01, 0x00, 0x1F, 0x40, 0x01, 0x01, 0x02])],
        bytes([0x10, 0x03]): [bytes([0x50, 0x03, 0x00, 0x32, 0x01, 0xF4])],
        bytes([0x19, 0x02, 0x08]): [bytes([0x59, 0x02, 0x7F, 0xC1, 0x21, 0x87, 0x2F])],
        bytes([0x11, 0x03]): [bytes([0x7F, 0x11, 0x78]), bytes([0x7F, 0x11, 0x7E])],  # pending, then NRC
    })
    client = UdsClient(IsoTpEndpoint(wire.a), p2_client=1.0)
    assert client.read_data_by_identifier(0x0100, 0x0101) == {0x0100: b"\x1f\x40", 0x0101: b"\x02"}
    assert client.diagnostic_session_control(0x03) == (50, 5000)
    assert client.read_dtc_by_status_mask(0x08) == [(0xC12187, 0x2F)]
    with pytest.raises(NegativeResponseError) as error:
        client.ecu_reset(0x03)
    assert error.value.nrc == 0x7E
    assert client.last_response_time_ms is not None
    with pytest.raises(TimeoutError):
        UdsClient(IsoTpEndpoint(wire.a), p2_client=0.05).request(bytes([0x3E, 0x00]))


def test_client_flushes_stale_frames_before_a_request():
    wire = Wire()
    for _ in range(5):  # stale traffic queued while the tester was idle
        wire.a.inbox.put(CanFrame(0x7E8, bytes([0x02, 0x7E, 0x00]).ljust(8, b"\xCC")))

    def flush():
        n = 0
        while wire.a.recv(timeout=0.0) is not None:
            n += 1
        return n
    wire.a.flush = flush
    StubServer(wire.b, {bytes([0x22, 0x01, 0x00]): [bytes([0x62, 0x01, 0x00, 0x1F, 0x40])]})
    client = UdsClient(IsoTpEndpoint(wire.a), p2_client=1.0)
    assert client.read_data_by_identifier(0x0100) == {0x0100: b"\x1f\x40"}
    assert wire.a.inbox.empty()


def test_udp_bus_counts_kernel_drops_and_flushes(tmp_path):
    import os
    from vecu_sim.udp_can_bus import UdpCanBus
    port = 40000 + (os.getpid() % 20000)
    with UdpCanBus(port=port) as rx, UdpCanBus(port=port) as tx:
        for _ in range(20000):
            tx.send(CanFrame(0x123, b"\x01"))
        import time
        time.sleep(0.1)
        queued = rx.flush()
        tx.send(CanFrame(0x123, b"\x02"))  # drops are reported with the next queued frame
        assert rx.recv(timeout=1.0) == CanFrame(0x123, b"\x02")
        assert queued > 0 and rx.drops > 0
        assert queued + rx.drops + 1 == 20001


def test_dtc_formatting():
    assert dtc_to_text(0xC12187) == "U0121-87"
    assert dtc_to_text(0x403164) == "C0031-64"
    assert dtc_to_text(0xF00049) == "U3000-49"
    assert status_to_text(0x09) == "testFailed|confirmedDTC"
