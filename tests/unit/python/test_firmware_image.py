"""Python firmware image implementation (vecu_sim.firmware_image)."""

import zlib

import pytest

from vecu_sim import firmware_image as fw


def build(version=(1, 2, 3), size=1000, **kwargs):
    header = fw.ImageHeader(hardware_id=0x0000CC01, version=version, build_time=1790000000, **kwargs)
    return fw.build_image(header, fw.make_payload(size, 7))


def test_round_trip():
    image = build(security_version=4, flags=fw.FLAG_SIMULATED_DEFECT_HANG)
    header = fw.verify_image(image)
    assert len(image) == fw.HEADER_SIZE + 1000 == header.image_size
    assert (header.version, header.version_text, header.security_version) == ((1, 2, 3), "01.02.03", 4)
    assert header.flags == fw.FLAG_SIMULATED_DEFECT_HANG and header.payload_crc32 == zlib.crc32(image[64:])


def test_payload_generator_matches_the_cpp_ecu():
    # Golden value: CRC-32 of the C++ factory payload (makePayload(65536, 0x000600)), as
    # written by vecu_cruise_ecu into slot A and logged by fwtool inspect (Phase 6 docs).
    assert zlib.crc32(fw.make_payload(65536, 0x000600)) == 0x98C3C505
    assert fw.make_payload(16, 0) == fw.make_payload(16, 0x9E3779B9)  # seed 0 is replaced


def test_every_header_bit_flip_is_detected():
    good = build(size=64)
    for byte in range(fw.HEADER_SIZE):
        for bit in range(8):
            bad = bytearray(good)
            bad[byte] ^= 1 << bit
            with pytest.raises(fw.ImageFormatError):
                fw.parse_header(bytes(bad))


@pytest.mark.parametrize("offset", [64, 500, 1063])
def test_payload_corruption_is_detected(offset):
    bad = bytearray(build())
    bad[offset] ^= 0x80
    with pytest.raises(fw.ImageFormatError) as error:
        fw.verify_image(bytes(bad))
    assert error.value.error == "PayloadCrcMismatch"


def test_size_limits_and_truncation():
    with pytest.raises(fw.ImageFormatError, match="TooSmall"):
        fw.parse_header(b"VFWI")
    with pytest.raises(fw.ImageFormatError, match="PayloadSizeInvalid"):
        fw.verify_image(build()[:-1])  # truncated download
    with pytest.raises(fw.ImageFormatError, match="PayloadSizeInvalid"):
        fw.verify_image(build(), max_size=512)


def test_crc32_is_not_authentication():
    # Anyone can change the payload and recompute both CRCs: the image is "valid" again.
    # Detecting this needs a cryptographic signature (verification method 1, not implemented).
    image = bytearray(build())
    image[100] ^= 0xFF
    header = fw.parse_header(bytes(image))
    tampered = fw.build_image(header, bytes(image[64:]))
    assert fw.verify_image(tampered).version == (1, 2, 3)


def test_versions_and_status_decoders():
    assert fw.parse_version("0.7.0") == (0, 7, 0) and fw.parse_version("00.07.00") == (0, 7, 0)
    with pytest.raises(ValueError):
        fw.parse_version("1.256.0")
    assert fw.decode_boot_status(bytes([1, 4, 3, 2, 3, 2, 0]))["running_slot"] == "B"
    assert fw.decode_update_status(bytes([4, 6, 0, 1, 0, 0]))["bytes_received"] == 65536
    assert fw.decode_versions(bytes([0, 6, 0, 0, 7, 0, 0, 0, 0, 2]))["security_version"] == 2
