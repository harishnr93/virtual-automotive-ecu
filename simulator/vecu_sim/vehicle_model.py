"""Simplified longitudinal vehicle dynamics.

Point-mass model along the direction of travel:

    m_eff · dv/dt = F_drive − F_aero − F_roll − F_grade − F_brake

    F_drive : actuator force, first-order lag towards  u_throttle · min(F_max, P_max / v)
    F_aero  = ½ · ρ · c_d · A · v²
    F_roll  = m · g · c_r · cos θ          (only while moving)
    F_grade = m · g · sin θ                (θ = atan(grade % / 100))
    F_brake = u_brake · F_brake,max        (only while moving)
    m_eff   = m · λ                        (λ: rotating-mass factor)

Integrated with explicit Euler at a fixed step (10 ms). Speed is limited to v >= 0
(no reversing, no rolling back on hills).

Assumptions / limitations (deliberately simple, see docs/phase3_cruise_control.md):
no gearbox or engine map (power limit only), no tyre slip, no wind, no driveline
elasticity, constant mass, brake force not speed-dependent. Good enough to exercise a
cruise controller with realistic orders of magnitude; not a validated vehicle model.

The same equations are implemented in C++ for fast controller tests
(tests/unit/support/longitudinal_plant.hpp); a golden-value test keeps them in sync.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    mass_kg: float = 1500.0
    inertia_factor: float = 1.05
    drag_coefficient: float = 0.30
    frontal_area_m2: float = 2.2
    air_density: float = 1.225
    rolling_resistance: float = 0.010
    max_drive_force_n: float = 4000.0
    max_power_w: float = 90000.0
    max_brake_force_n: float = 9000.0
    drive_time_constant_s: float = 0.4
    gravity: float = 9.81


class VehicleModel:
    def __init__(self, params: VehicleParams = VehicleParams(), speed_kph: float = 0.0) -> None:
        self.p = params
        self.speed_mps = speed_kph / 3.6
        self.drive_force_n = 0.0
        self.accel_mps2 = 0.0
        self.distance_m = 0.0

    @property
    def speed_kph(self) -> float:
        return self.speed_mps * 3.6

    def available_force_n(self) -> float:
        return min(self.p.max_drive_force_n, self.p.max_power_w / max(self.speed_mps, 1.0))

    def resistance_n(self, grade_pct: float = 0.0) -> float:
        theta = math.atan(grade_pct / 100.0)
        v = self.speed_mps
        aero = 0.5 * self.p.air_density * self.p.drag_coefficient * self.p.frontal_area_m2 * v * v
        rolling = self.p.mass_kg * self.p.gravity * self.p.rolling_resistance * math.cos(theta) if v > 0 else 0.0
        return aero + rolling + self.p.mass_kg * self.p.gravity * math.sin(theta)

    def road_load_throttle_pct(self, speed_kph: float, grade_pct: float = 0.0) -> float:
        """Throttle that holds `speed_kph` constant (steady state)."""
        saved = self.speed_mps
        self.speed_mps = speed_kph / 3.6
        try:
            return 100.0 * self.resistance_n(grade_pct) / self.available_force_n()
        finally:
            self.speed_mps = saved

    def settle_drive_force(self, throttle_pct: float) -> None:
        """Puts the drive actuator in steady state (e.g. vehicle already cruising at start)."""
        self.drive_force_n = self.available_force_n() * throttle_pct / 100.0

    def step(self, throttle_pct: float, brake_pct: float, grade_pct: float, dt: float) -> None:
        throttle = min(max(throttle_pct, 0.0), 100.0) / 100.0
        brake = min(max(brake_pct, 0.0), 100.0) / 100.0
        self.drive_force_n += (throttle * self.available_force_n() - self.drive_force_n) * dt / self.p.drive_time_constant_s
        braking = brake * self.p.max_brake_force_n if self.speed_mps > 0 else 0.0
        net = self.drive_force_n - self.resistance_n(grade_pct) - braking
        new_speed = max(0.0, self.speed_mps + net / (self.p.mass_kg * self.p.inertia_factor) * dt)
        self.accel_mps2 = (new_speed - self.speed_mps) / dt
        self.distance_m += 0.5 * (self.speed_mps + new_speed) * dt
        self.speed_mps = new_speed
