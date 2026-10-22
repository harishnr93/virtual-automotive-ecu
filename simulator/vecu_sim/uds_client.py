"""UDS (ISO 14229-1) tester client for the virtual cruise control ECU.

The DID and DTC descriptions below play the role of an ODX/CDD diagnostic description:
the tester needs them to interpret responses. tests/integration/test_diagnostics.py checks
them against the running ECU.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .isotp import IsoTpEndpoint

FUNCTIONAL_ID = 0x7DF

NRC_NAMES = {
    0x10: "generalReject", 0x11: "serviceNotSupported", 0x12: "subFunctionNotSupported",
    0x13: "incorrectMessageLengthOrInvalidFormat", 0x14: "responseTooLong", 0x21: "busyRepeatRequest",
    0x22: "conditionsNotCorrect", 0x24: "requestSequenceError", 0x31: "requestOutOfRange",
    0x33: "securityAccessDenied", 0x78: "requestCorrectlyReceivedResponsePending",
    0x7E: "subFunctionNotSupportedInActiveSession", 0x7F: "serviceNotSupportedInActiveSession",
}


@dataclass(frozen=True)
class Did:
    name: str
    length: int
    kind: str          # "ascii" | "uint"
    scale: float = 1.0
    unit: str = ""


DIDS: Dict[int, Did] = {
    0xF190: Did("VIN", 17, "ascii"),
    0xF187: Did("SparePartNumber", 12, "ascii"),
    0xF189: Did("SoftwareVersion", 8, "ascii"),
    0xF18C: Did("EcuSerialNumber", 10, "ascii"),       # extended session only
    0xF186: Did("ActiveDiagnosticSession", 1, "uint"),
    0x0100: Did("VehicleSpeed", 2, "uint", 0.01, "km/h"),
    0x0101: Did("CruiseState", 1, "uint"),
    0x0102: Did("CruiseTargetSpeed", 1, "uint", 1.0, "km/h"),
    0x0103: Did("ThrottleCommand", 2, "uint", 0.01, "%"),
    0x0104: Did("CruiseFaultReason", 1, "uint"),
    0x0200: Did("DeadlineMissesTotal", 4, "uint"),      # extended session only
    0x0201: Did("UptimeSeconds", 4, "uint", 1.0, "s"),
    0x0202: Did("ResetCount", 2, "uint"),
}

DTCS = {
    0xC12187: "U0121-87 VehicleDynamics (0x100) message missing",
    0xC14087: "U0140-87 DriverInputs (0x110) message missing",
    0x403164: "C0031-64 vehicle speed signal plausibility failure",
    0xF00049: "U3000-49 cruise controller internal failure",
}

DTC_STATUS_BITS = ["testFailed", "testFailedThisOperationCycle", "pendingDTC", "confirmedDTC",
                   "testNotCompletedSinceLastClear", "testFailedSinceLastClear",
                   "testNotCompletedThisOperationCycle", "warningIndicatorRequested"]


def dtc_to_text(code: int) -> str:
    """0xC12187 -> 'U0121-87' (SAE J2012 display format)."""
    letter = "PCBU"[(code >> 22) & 0x3]
    return f"{letter}{(code >> 20) & 0x3}{(code >> 16) & 0xF:X}{(code >> 8) & 0xFF:02X}-{code & 0xFF:02X}"


def status_to_text(status: int) -> str:
    return "|".join(name for bit, name in enumerate(DTC_STATUS_BITS) if status & (1 << bit)) or "-"


class NegativeResponseError(Exception):
    def __init__(self, service: int, nrc: int) -> None:
        self.service = service
        self.nrc = nrc
        super().__init__(f"NRC 0x{nrc:02X} {NRC_NAMES.get(nrc, 'unknown')} for service 0x{service:02X}")


class UdsClient:
    def __init__(self, isotp: IsoTpEndpoint, p2_client: float = 0.5, p2_star_client: float = 5.5) -> None:
        # P2 client = server P2 (50 ms) + generous margin for a non-real-time host.
        self.isotp = isotp
        self.p2_client = p2_client
        self.p2_star_client = p2_star_client
        self.last_response_time_ms: Optional[float] = None

    def request(self, payload: bytes, functional: bool = False, expect_response: bool = True) -> Optional[bytes]:
        payload = bytes(payload)
        start = time.perf_counter()
        self.isotp.send(payload, FUNCTIONAL_ID if functional else None)
        if not expect_response:
            return None
        timeout = self.p2_client
        while True:
            response = self.isotp.recv(timeout)
            if response is None:
                raise TimeoutError(f"no response to service 0x{payload[0]:02X} within {timeout:.2f} s")
            if response[0] == 0x7F and len(response) >= 3 and response[1] == payload[0]:
                if response[2] == 0x78:  # response pending: wait up to P2*
                    timeout = self.p2_star_client
                    continue
                self.last_response_time_ms = (time.perf_counter() - start) * 1000.0
                raise NegativeResponseError(response[1], response[2])
            if response[0] == payload[0] + 0x40:
                self.last_response_time_ms = (time.perf_counter() - start) * 1000.0
                return response
            # unrelated response (e.g. late answer to an earlier request): keep waiting

    # ------------------------------------------------------------------- services ----
    def diagnostic_session_control(self, session: int) -> Tuple[int, int]:
        rsp = self.request(bytes([0x10, session]))
        return int.from_bytes(rsp[2:4], "big"), int.from_bytes(rsp[4:6], "big") * 10  # P2, P2* in ms

    def ecu_reset(self, reset_type: int) -> None:
        self.request(bytes([0x11, reset_type]))

    def tester_present(self, suppress: bool = False, functional: bool = False) -> None:
        self.request(bytes([0x3E, 0x80 if suppress else 0x00]), functional=functional, expect_response=not suppress)

    def read_data_by_identifier(self, *dids: int) -> Dict[int, bytes]:
        request = bytes([0x22]) + b"".join(d.to_bytes(2, "big") for d in dids)
        rsp = self.request(request)
        result, i = {}, 1
        while i < len(rsp):
            did = int.from_bytes(rsp[i:i + 2], "big")
            if did not in DIDS:
                raise ValueError(f"response contains unknown DID 0x{did:04X}")
            length = DIDS[did].length
            result[did] = rsp[i + 2:i + 2 + length]
            i += 2 + length
        return result

    def read_dtc_count(self, mask: int) -> int:
        rsp = self.request(bytes([0x19, 0x01, mask]))
        return int.from_bytes(rsp[4:6], "big")

    def _dtc_list(self, rsp: bytes) -> List[Tuple[int, int]]:
        body = rsp[3:]
        return [(int.from_bytes(body[i:i + 3], "big"), body[i + 3]) for i in range(0, len(body) - 3, 4)]

    def read_dtc_by_status_mask(self, mask: int) -> List[Tuple[int, int]]:
        return self._dtc_list(self.request(bytes([0x19, 0x02, mask])))

    def read_supported_dtcs(self) -> List[Tuple[int, int]]:
        return self._dtc_list(self.request(bytes([0x19, 0x0A])))

    def clear_diagnostic_information(self, group: int = 0xFFFFFF) -> None:
        self.request(bytes([0x14]) + group.to_bytes(3, "big"))


def decode_did(did: int, raw: bytes):
    description = DIDS[did]
    if description.kind == "ascii":
        return raw.decode("ascii")
    value = int.from_bytes(raw, "big") * description.scale
    return value if description.scale != 1.0 else int(value)
