"""Python DBC codec: must read the project DBC and agree bit-for-bit with the C++ codec."""

import pytest

from vecu_sim.dbc import Database, default_dbc_path

DB = Database.load(default_dbc_path())

MOTOROLA_DBC = """
BO_ 291 Test: 8 Node
 SG_ Big16 : 7|16@0+ (1,0) [0|65535] "" Node
 SG_ Nibbles : 19|8@0+ (1,0) [0|255] "" Node
"""


def test_reads_project_matrix():
    assert {m.name for m in DB.messages.values()} == {"VehicleDynamics", "DriverInputs", "CruiseControlStatus"}
    dynamics = DB["VehicleDynamics"]
    assert (dynamics.frame_id, dynamics.dlc, dynamics.cycle_time_ms, dynamics.timeout_ms) == (0x100, 8, 20, 100)
    assert DB["CruiseControlStatus"].signals["CruiseState"].choices[4] == "Fault"
    assert DB["CruiseControlStatus"].signals["CruiseFaultReason"].choices[1] == "SpeedSignalTimeout"


def test_decodes_frame_recorded_from_cpp_ecu():
    # Frame from a Phase 1 candump trace of vecu_sensor_ecu: 100#1400DA0A01000001
    values = DB.by_id(0x100).decode(bytes.fromhex("1400DA0A01000001"))
    assert values["VehicleSpeed"] == pytest.approx(0.20)
    assert values["LongitudinalAccel"] == pytest.approx(2.778)
    assert values["VehicleSpeedValid"] == 1
    assert values["AliveCounter"] == 1


def test_encoding_matches_cpp_reference_vectors():
    # Same vectors as tests/unit/test_can_signal.cpp (CanSignalPhysical.*).
    data = DB["VehicleDynamics"].encode({"VehicleSpeed": 123.45, "LongitudinalAccel": -1.234})
    assert data[0:4] == bytes([0x39, 0x30, 0x2E, 0xFB])


def test_motorola_layout_matches_cpp():
    message = Database.from_text(MOTOROLA_DBC)["Test"]
    data = message.encode({"Big16": 0x1234})
    assert data[0:2] == bytes([0x12, 0x34])
    # DBC "19|8@0+": MSB at bit 19 (byte 2, bit 3), continuing into byte 3.
    data = message.encode({"Nibbles": 0xAB})
    assert data[2:4] == bytes([0x0A, 0xB0])
    assert message.decode(data)["Nibbles"] == 0xAB


def test_values_are_clamped_and_rounded():
    status = DB["CruiseControlStatus"]
    decoded = status.decode(status.encode({"ThrottleCommand": 150.0, "CruiseTargetSpeed": 80.4}))
    assert decoded["ThrottleCommand"] == pytest.approx(100.0)
    assert decoded["CruiseTargetSpeed"] == 80


def test_round_trip_all_messages():
    for message in DB.messages.values():
        values = {name: (s.minimum + s.maximum) / 2 for name, s in message.signals.items()}
        decoded = message.decode(message.encode(values))
        for name, signal in message.signals.items():
            assert decoded[name] == pytest.approx(values[name], abs=signal.factor), name


def test_rejects_errors():
    with pytest.raises(KeyError):
        DB["DriverInputs"].encode({"NoSuchSignal": 1})
    with pytest.raises(ValueError):
        DB["DriverInputs"].decode(bytes(4))
    with pytest.raises(ValueError):
        DB["DriverInputs"].encode({"AcceleratorPedal": float("nan")})
