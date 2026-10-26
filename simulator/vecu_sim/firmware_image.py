"""Firmware image format of the virtual ECU — an independent Python implementation.

Written from the format description (docs/phase6_firmware_update.md), not generated from the
C++ code, so the C++ ECU and this tool check each other: an image built here must be
accepted by the ECU, and the factory image the ECU writes to its flash file must parse here.

Layout (little-endian, 64-byte header followed by the payload):

    offset size field
       0    4  magic "VFWI"
       4    2  header version (1)
       6    2  header size (64)
       8    4  hardware id
      12    3  version major, minor, patch      (15: reserved, 0)
      16    4  security version (anti-rollback counter)
      20    4  payload size
      24    4  payload CRC-32
      28    8  build time (seconds since 1970)
      36    1  verification method (0 = CRC-32, 1 = signature: reserved)   (37: reserved)
      38    2  signature size (0)
      40    4  flags (bit 0: simulated defect, test images only)
      44   16  reserved, 0
      60    4  header CRC-32 over bytes 0..59

CRC-32 detects accidental corruption (transmission errors, interrupted writes, bit flips).
It is NOT authentication: anyone can build an image with a matching CRC.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import Tuple

MAGIC = b"VFWI"
HEADER_VERSION = 1
HEADER_SIZE = 64
FLAG_SIMULATED_DEFECT_HANG = 0x01
VERIFICATION_CRC32 = 0
VERIFICATION_SIGNATURE = 1

# Flash layout of the ECU (FlashLayout in include/firmware/boot_control.hpp).
SLOT_SIZE = 256 * 1024
SLOT_ADDRESS = (0x00000, 0x40000)

_HEADER = struct.Struct("<4sHHI3BxIIIQBxHI16s")  # bytes 0..59
assert _HEADER.size == 60

# Enumerations as reported by DIDs 0x0400/0x0401 (values fixed in the C++ headers).
IMAGE_ERRORS = ["None", "TooSmall", "BadMagic", "UnsupportedHeaderVersion", "HeaderCrcMismatch",
                "PayloadSizeInvalid", "PayloadCrcMismatch", "WrongHardware", "VersionNotNewer",
                "SecurityVersionRollback", "UnsupportedVerification", "ReadError"]
SLOT_STATES = ["Empty", "Receiving", "Pending", "Trial", "Confirmed", "Invalid"]
BOOT_EVENTS = ["Normal", "FirstProvisioning", "TrialBoot", "Rollback", "FallbackCorrupt", "NoValidImage"]
UPDATE_STATES = ["Idle", "Erased", "Downloading", "Downloaded", "Verified", "Failed"]


def _name(table, index: int) -> str:
    return table[index] if 0 <= index < len(table) else f"unknown({index})"


class ImageFormatError(ValueError):
    def __init__(self, error: str) -> None:
        self.error = error
        super().__init__(error)


@dataclass
class ImageHeader:
    hardware_id: int
    version: Tuple[int, int, int]
    security_version: int = 1
    payload_size: int = 0
    payload_crc32: int = 0
    build_time: int = 0
    verification_method: int = VERIFICATION_CRC32
    signature_size: int = 0
    flags: int = 0

    @property
    def version_text(self) -> str:
        return "%02d.%02d.%02d" % self.version

    @property
    def image_size(self) -> int:
        return HEADER_SIZE + self.payload_size + self.signature_size


def parse_version(text: str) -> Tuple[int, int, int]:
    """'0.7.0' or '00.07.00' -> (0, 7, 0)."""
    parts = tuple(int(p) for p in text.split("."))
    if len(parts) != 3 or not all(0 <= p <= 255 for p in parts):
        raise ValueError(f"invalid version {text!r} (expected MAJOR.MINOR.PATCH, each 0..255)")
    return parts  # type: ignore[return-value]


def serialize_header(header: ImageHeader) -> bytes:
    body = _HEADER.pack(MAGIC, HEADER_VERSION, HEADER_SIZE, header.hardware_id, *header.version,
                        header.security_version, header.payload_size, header.payload_crc32, header.build_time,
                        header.verification_method, header.signature_size, header.flags, bytes(16))
    return body + struct.pack("<I", zlib.crc32(body))


def parse_header(data: bytes) -> ImageHeader:
    if len(data) < HEADER_SIZE:
        raise ImageFormatError("TooSmall")
    if data[:4] != MAGIC:
        raise ImageFormatError("BadMagic")
    (_, header_version, header_size, hardware_id, major, minor, patch, security, payload_size, payload_crc,
     build_time, method, signature_size, flags, _) = _HEADER.unpack_from(data)
    if header_version != HEADER_VERSION or header_size != HEADER_SIZE:
        raise ImageFormatError("UnsupportedHeaderVersion")
    if zlib.crc32(data[:60]) != struct.unpack_from("<I", data, 60)[0]:
        raise ImageFormatError("HeaderCrcMismatch")
    return ImageHeader(hardware_id, (major, minor, patch), security, payload_size, payload_crc, build_time,
                       method, signature_size, flags)


def make_payload(size: int, seed: int) -> bytes:
    """Deterministic stand-in for program code (xorshift32, same sequence as the C++ ECU)."""
    state = seed & 0xFFFFFFFF or 0x9E3779B9
    out = bytearray(size)
    for i in range(size):
        state ^= (state << 13) & 0xFFFFFFFF
        state ^= state >> 17
        state ^= (state << 5) & 0xFFFFFFFF
        out[i] = state & 0xFF
    return bytes(out)


def build_image(header: ImageHeader, payload: bytes) -> bytes:
    header.payload_size = len(payload)
    header.payload_crc32 = zlib.crc32(payload)
    header.verification_method = VERIFICATION_CRC32
    header.signature_size = 0
    return serialize_header(header) + payload


def verify_image(data: bytes, max_size: int = SLOT_SIZE) -> ImageHeader:
    """Parses and checks an image (header CRC, size, payload CRC); raises ImageFormatError."""
    header = parse_header(data)
    if header.payload_size == 0 or header.image_size > max_size or header.image_size > len(data):
        raise ImageFormatError("PayloadSizeInvalid")
    if header.verification_method != VERIFICATION_CRC32:
        raise ImageFormatError("UnsupportedVerification")
    if zlib.crc32(data[HEADER_SIZE:HEADER_SIZE + header.payload_size]) != header.payload_crc32:
        raise ImageFormatError("PayloadCrcMismatch")
    return header


def decode_boot_status(raw: bytes) -> dict:
    """DID 0x0400."""
    return {"running_slot": "AB"[raw[0]] if raw[0] < 2 else None,
            "state_a": _name(SLOT_STATES, raw[1]), "state_b": _name(SLOT_STATES, raw[2]),
            "boot_attempts": raw[3], "max_boot_attempts": raw[4],
            "last_boot_event": _name(BOOT_EVENTS, raw[5]), "rejected_image_error": _name(IMAGE_ERRORS, raw[6])}


def decode_update_status(raw: bytes) -> dict:
    """DID 0x0401."""
    return {"state": _name(UPDATE_STATES, raw[0]), "last_image_error": _name(IMAGE_ERRORS, raw[1]),
            "bytes_received": int.from_bytes(raw[2:6], "big")}


def decode_versions(raw: bytes) -> dict:
    """DID 0x0402."""
    return {"slot_a": "%02d.%02d.%02d" % tuple(raw[0:3]), "slot_b": "%02d.%02d.%02d" % tuple(raw[3:6]),
            "security_version": int.from_bytes(raw[6:10], "big")}
