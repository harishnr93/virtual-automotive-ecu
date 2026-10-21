"""Physics checks of the vehicle model against hand-derived values."""

import math

import pytest

from vecu_sim.vehicle_model import VehicleModel, VehicleParams

DT = 0.01
P = VehicleParams()

# Golden values, also asserted by the C++ test CruiseClosedLoop.PlantMatchesPythonGoldenValues:
# they keep tests/unit/support/longitudinal_plant.hpp and vehicle_model.py in sync.
GOLDEN_FULL_THROTTLE_10S_KPH = 82.90919684950765
GOLDEN_COAST_10S_FROM_100_KPH = 90.19733428509716


def run(model, seconds, throttle=0.0, brake=0.0, grade=0.0):
    for _ in range(int(round(seconds / DT))):
        model.step(throttle, brake, grade, DT)
    return model


def test_golden_values_shared_with_cpp_plant():
    assert run(VehicleModel(), 10, throttle=100).speed_kph == pytest.approx(GOLDEN_FULL_THROTTLE_10S_KPH, abs=1e-9)
    coast = VehicleModel(speed_kph=100)
    assert run(coast, 10).speed_kph == pytest.approx(GOLDEN_COAST_10S_FROM_100_KPH, abs=1e-9)


def test_road_load_throttle_holds_speed():
    model = VehicleModel(speed_kph=80)
    hold = model.road_load_throttle_pct(80)
    v = 80 / 3.6
    expected = 100 * (0.5 * P.air_density * P.drag_coefficient * P.frontal_area_m2 * v * v
                      + P.mass_kg * P.gravity * P.rolling_resistance) / P.max_drive_force_n
    assert hold == pytest.approx(expected)
    model.settle_drive_force(hold)
    assert run(model, 20, throttle=hold).speed_kph == pytest.approx(80, abs=1e-6)


def test_initial_coast_deceleration_equals_resistance_over_effective_mass():
    model = VehicleModel(speed_kph=100)
    model.step(0, 0, 0, DT)
    v = 100 / 3.6
    resistance = 0.5 * P.air_density * P.drag_coefficient * P.frontal_area_m2 * v * v + P.mass_kg * P.gravity * P.rolling_resistance
    assert model.accel_mps2 == pytest.approx(-resistance / (P.mass_kg * P.inertia_factor), rel=1e-9)


def test_top_speed_is_power_limited():
    model = run(VehicleModel(), 300, throttle=100)
    v = model.speed_mps
    drive = P.max_power_w / v
    resistance = 0.5 * P.air_density * P.drag_coefficient * P.frontal_area_m2 * v * v + P.mass_kg * P.gravity * P.rolling_resistance
    assert drive == pytest.approx(resistance, rel=0.01)  # equilibrium reached
    assert 180 < model.speed_kph < 220


def test_grade_increases_required_throttle():
    model = VehicleModel()
    flat = model.road_load_throttle_pct(90)
    uphill = model.road_load_throttle_pct(90, grade_pct=4)
    theta = math.atan(0.04)
    available = min(P.max_drive_force_n, P.max_power_w / (90 / 3.6))  # power-limited at 90 km/h: 3600 N
    extra = 100 * P.mass_kg * P.gravity * (math.sin(theta) + P.rolling_resistance * (math.cos(theta) - 1)) / available
    assert uphill - flat == pytest.approx(extra)


def test_brake_stops_vehicle_and_never_reverses():
    model = run(VehicleModel(speed_kph=50), 10, brake=100)
    assert model.speed_kph == 0.0
    run(model, 5, grade=10)  # standing on a hill: no rolling back in this model
    assert model.speed_kph == 0.0


def test_drive_force_follows_first_order_lag():
    model = VehicleModel(speed_kph=50)
    target = model.available_force_n()
    run(model, P.drive_time_constant_s, throttle=100)
    assert model.drive_force_n / target == pytest.approx(1 - math.exp(-1), abs=0.02)
