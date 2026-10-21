# Phase 3 — Vehicle Simulation and Cruise Control

This phase closes the loop: a Python vehicle simulator and the C++ cruise control ECU
exchange CAN frames in real time, and the controller holds a desired speed. It continues
[Phase 2](phase2_scheduler.md).

---

## 1. Engineering objective

1. A simplified **longitudinal vehicle model** with a driver/road scenario, running as its own
   process on the CAN bus.
2. A **PID speed controller** with saturation and anti-windup, and a **cruise state machine**
   with brake/accelerator override, sensor-timeout handling and a safe fallback.
3. Keep the **control algorithm separate** from communication and scheduling.
4. **Validate** the controller: deterministic closed-loop tests in C++ and real-time
   closed-loop tests over CAN, with measured KPIs.

---

## 2. Concepts

### 2.1 The control loop
```
 driver's desired speed ─► setpoint ramp ─► PID + feedforward ─► throttle % ─► vehicle ─► speed
                                               ▲                                            │
                                               └──────────── measured speed (CAN 0x100) ◄───┘
```
The controller compares the setpoint with the measured speed (**error**) and computes a throttle
command. The vehicle reacts; the new speed comes back over CAN; this repeats every 10 ms.

### 2.2 PID in one paragraph
- **P** (proportional): throttle proportional to the current error. Fast, but alone it leaves a
  steady-state error, because some throttle is always needed to overcome drag.
- **I** (integral): accumulates the error over time and keeps growing until the error is zero.
  It learns the throttle needed to hold speed (road load, grade, extra mass).
- **D** (derivative): reacts to the rate of change. Implemented (on the measurement, filtered) but
  set to **0** here: tuning runs showed no benefit for this slow, well-damped plant.

### 2.3 Saturation and windup
The throttle command is limited to **0 … 60 %** (a comfort limit, about 1.5 m/s² at 60 km/h).
While the output is saturated, a naive integrator keeps growing ("windup") and later causes a
large overshoot. **Anti-windup by clamping** freezes the integrator while the output is limited
in the direction the error pushes. The unit test `AntiWindupShortensRecoveryFromSaturation`
measures the difference: after 5 s at saturation, a controller without anti-windup needs more than
10 s to leave saturation; with clamping it needs **≤ 2 steps (20 ms)**. Back-calculation is also
implemented and tested.

### 2.4 Setpoint ramp and feedforward
A target jump from 60 to 80 km/h is not applied at once: the internal setpoint ramps at
**3 km/h/s (≈ 0.83 m/s²)** for comfort. While ramping, the controller adds **acceleration
feedforward**, the throttle needed for that acceleration (`m·λ·a / F_max`). The PID then only
corrects model errors. Tuning experiment (same model, 60 → 80 km/h):

| Variant | Overshoot | Settling (±0.5 km/h) |
|---|---|---|
| PI, setpoint ramp, no feedforward | 1.94 km/h | 19.7 s |
| PI + acceleration feedforward | **0.77 km/h** | **8.0 s** |

### 2.5 Bumpless transfer
At engagement the integrator starts from the driver's current pedal position, and the setpoint
starts from the current speed, so the throttle does not jump. After an accelerator override the
integrator is held and resumed from its stored value (the road-load estimate).

### 2.6 State machine and safety rules
```
 Off ─(main switch)─► Standby ─(SET edge, 30–180 km/h, no brake)─► Active ⇄ Override (pedal > 5 %)
                         ▲  ◄──────── brake / cancel ───────────────┘  │
                         └──── 500 ms healthy ◄── Fault ◄── any input fault (from any state)
```
| Rule | Reason |
|---|---|
| Throttle only in **Active**; every other state outputs 0 % | Defined safe state |
| Brake or CANCEL disengages immediately (Active and Override) | Driver always overrides |
| Accelerator above 5 % → Override (cruise throttle 0, driver in control) | Driver may go faster |
| Fault on: speed timeout, speed invalid, driver-input timeout, stale inputs (> 100 ms), controller fault | Never control on bad data |
| Fault exits only after 500 ms of healthy inputs, and only to **Standby** | No automatic re-engagement |
| Controller fault (NaN, invalid dt/config) is latched | Requires an ECU restart (UDS reset in Phase 4) |

These rules are inspired by ISO 26262 *practice* (safe state, fault reaction, no silent
recovery). This is not a safety-certified design and no hazard analysis was performed.

### 2.7 Defence in depth on the actuator side
The simulator plays the role of the powertrain ECU and applies the cruise command only if the
ECU reports **Active**, the command is **fresh (≤ 100 ms)** and the brake is not pressed:
`applied throttle = max(driver pedal, valid cruise command)`.

---

## 3. Vehicle model

```
m·λ·dv/dt = F_drive − ½ρ c_d A v² − m g c_r cos θ − m g sin θ − F_brake
F_drive: first-order lag (τ = 0.4 s) towards u · min(F_max, P_max / v)
```

| Parameter | Value | | Parameter | Value |
|---|---|---|---|---|
| mass m | 1500 kg | | max. drive force F_max | 4000 N |
| rotating-mass factor λ | 1.05 | | max. power P_max | 90 kW |
| drag coefficient c_d | 0.30 | | max. brake force | 9000 N |
| frontal area A | 2.2 m² | | drive lag τ | 0.4 s |
| rolling resistance c_r | 0.010 | | integration | explicit Euler, 10 ms |

Derived checks (unit-tested): about 8.7 % throttle holds 80 km/h on a flat road; top speed is
power-limited at about 200 km/h; a 4 % grade at 90 km/h needs an extra ≈ 16 % throttle.

**Assumptions and limitations:** point mass; no gearbox, engine map, tyre slip, wind or
driveline elasticity; constant mass; brake force independent of speed; no rolling backwards.
Explicit Euler at 10 ms is adequate here (the fastest time constant is 0.4 s). The model has
realistic orders of magnitude but is **not validated against a real vehicle**.

The same equations exist in C++ (`tests/unit/support/longitudinal_plant.hpp`) for fast
controller tests. Golden values (10 s at full throttle → 82.90919684950765 km/h; 10 s coasting
from 100 km/h → 90.19733428509716 km/h) are asserted in **both** languages to 1e-9 km/h.

---

## 4. Architecture

```
 Python process: vecu_sim.simulator             C++ process: vecu_cruise_ecu
 ┌───────────────────────────────┐              ┌─────────────────────────────────────────────┐
 │ Scenario (JSON) → driver/road │  0x100 20 ms │ CanRx ─► Com_20ms: decode, supervise,        │
 │ VehicleModel (10 ms steps)    │ ───────────► │          build CruiseInputs ─┐               │
 │ DBC codec (config/*.dbc)      │  0x110 50 ms │                              ▼ SharedSignal  │
 │ actuator arbitration          │ ───────────► │ Control_10ms: CruiseController.step()        │
 │ CSV / SVG / KPI report        │ ◄─────────── │          (src/control: no CAN, no threads)   │
 └───────────────────────────────┘  0x200 20 ms │ Com_20ms: encode 0x200 ◄─ SharedSignal       │
                                                └─────────────────────────────────────────────┘
```

| File | Content |
|---|---|
| `include/control/pid_controller.hpp` | PID: saturation, rate limit, clamping/back-calculation anti-windup, derivative on measurement + filter, feedforward, bumpless reset, NaN protection |
| `include/control/cruise_controller.hpp` | `CruiseInputs` → `CruiseController::step()` → `CruiseOutputs`; state machine, setpoint ramp, feedforward |
| `apps/cruise_ecu/main.cpp` | glue: bus signals ↔ `CruiseInputs/Outputs`; enum values checked against the DBC with `static_assert` |
| `simulator/vecu_sim/dbc.py` | DBC reader and codec (the simulator uses the same DBC as the ECU) |
| `simulator/vecu_sim/can_bus.py` | Python SocketCAN + `open_bus()` (auto → vcan0 or UDP bus) |
| `simulator/vecu_sim/vehicle_model.py` | vehicle dynamics |
| `simulator/vecu_sim/scenario.py` | JSON scenarios: levels, button pulses, fault windows |
| `simulator/vecu_sim/simulator.py` | real-time loop, actuator arbitration, recording, CLI |
| `simulator/vecu_sim/analysis.py` | KPIs: rise/settling/overshoot, disturbance, fault and override reaction |
| `simulator/vecu_sim/svg_plot.py` | dependency-free SVG plots |
| `config/scenarios/*.json` | 5 scenarios |
| `scripts/run_closed_loop.sh` | runs ECU + simulator, writes `logs/closed_loop/*` |

**CAN matrix change:** `CruiseFaultReason` (bits 32–35 of 0x200, values 0–5) was added to the
DBC and to `vehicle_signals.hpp` so the reason for a Fault is visible on the bus. The
DBC-consistency unit test confirms that both sides still match.

**Python dependencies:** none beyond the standard library. matplotlib was not usable on this
machine (it requires numpy ≥ 1.23, the system has 1.21.5), hence the small SVG writer.

---

## 5. Commands

```bash
cd ~/harish_ws/vir_auto_ecu
cmake -S . -B build -DVECU_WARNINGS_AS_ERRORS=ON && cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure            # 150 tests, ~50 s

# Closed-loop runs (ECU + simulator), results in logs/closed_loop/
./scripts/run_closed_loop.sh config/scenarios/cruise_step_response.json
./scripts/run_closed_loop.sh config/scenarios/cruise_sensor_fault.json
./scripts/run_closed_loop.sh                          # 80 s full demo

# Only the deterministic controller tests (prints measured KPIs)
./build/bin/vecu_unit_tests --gtest_filter='Pid*:Cruise*'

# Simulator alone (e.g. against an ECU started in another terminal)
PYTHONPATH=simulator python3 -m vecu_sim.simulator --scenario config/scenarios/cruise_grade.json \
    --csv run.csv --svg run.svg --report run.json
```

Writing a new scenario: copy a file from `config/scenarios/`. `accelerator_pct: "hold"` starts
the car at the throttle that holds its initial speed. Available faults:
`mute_vehicle_dynamics`, `mute_driver_inputs`, `invalid_speed`.

---

## 6. Expected behaviour

ECU log of the sensor-fault scenario (real run):
```
CRUISE_STATE Standby -> Active target_kph=80.0 speed_kph=80.0
RX_TIMEOUT id=0x100 silence_ms=111.8 previous=Ok
CRUISE_STATE Active -> Fault reason=SpeedSignalTimeout
RX_RECOVERED id=0x100 gap_ms=520.4
CRUISE_STATE Fault -> Standby                      # 500 ms healthy, NOT back to Active
CRUISE_STATE Standby -> Active target_kph=80.0 speed_kph=78.0   # driver presses SET again
CRUISE_STATE Active -> Fault reason=SpeedInvalid
```
At engagement you will see a short `Active -> Override -> Active`: in the scenarios, the driver
presses SET while still holding the pedal at road load (about 6 % > 5 % threshold), then releases
it. A real cruise system behaves the same way.

Plots of real runs (speed and target, throttle, cruise state as background colour):
- [Full demo](images/phase3_full_demo.svg)
- [Step response 60 → 80 km/h](images/phase3_step_response.svg)
- [Sensor faults](images/phase3_sensor_fault.svg)

---

## 7. Tests and measured results

### 7.1 New tests (37 C++ test cases, 35 Python unit tests, 8 closed-loop system tests; CTest total 150)

| Suite | Type | What it proves |
|---|---|---|
| `PidConfig.*`, `PidController.*` | unit | P/I/D terms, derivative on measurement + filter, feedforward, saturation, rate limit, anti-windup (none vs clamping vs back-calculation, measured), bumpless reset, NaN/dt protection, invalid config |
| `CruiseControllerTest.*` | unit | every state transition; edge detection; target clamping; brake/accelerator override; integrator hold; fault priority; 500 ms recovery debounce incl. restart on intermittent fault; recovery only to Standby; latched controller fault |
| `CruiseClosedLoop.*` | unit (closed loop, deterministic) | step response, coasting deceleration, grade rejection, +20 % mass robustness, fault reaction, C++/Python model golden values |
| `test_dbc.py` | Python unit | DBC parsing; decodes a frame recorded from the C++ ECU; same byte vectors as the C++ codec (Intel and Motorola) |
| `test_vehicle_model.py` | Python unit | road load, coast-down, power-limited top speed, grade, braking, actuator lag, golden values |
| `test_scenario_and_simulator.py` | Python unit | scenario semantics, transmission schedule, alive counters, fault injection, actuator arbitration, KPI formulas, SVG structure |
| `test_closed_loop.py` | system (real-time, 4 scenarios in parallel) | step response, grade, overrides, faults; communication health (no alive errors, no RX overruns, simulator never late) |

Verification (2026-10-21): GCC 11 `-Werror`, ASan + UBSan and Clang 14 ThreadSanitizer builds,
**150/150 CTest entries passed** in each (2 vcan tests skipped inside the C++ suites);
cppcheck and clang-tidy: no findings.

### 7.2 Deterministic closed loop (C++, `CruiseClosedLoop.*`)

| Case | Overshoot | Settling ±0.5 km/h | Final error | Max throttle |
|---|---|---|---|---|
| 60 → 80 km/h | 0.77 km/h | 8.01 s | 0.001 km/h | 48.2 % |
| 100 → 80 km/h (coasting) | 0.00 km/h | 22.27 s | 0.000 km/h | 8.7 % |
| 60 → 80 km/h, mass 1800 kg (model error +20 %) | 0.78 km/h | 8.85 s | 0.000 km/h | 52.1 % |
| 4 % grade at 90 km/h | max drop 1.40 km/h | back in band after 8.61 s | — | — |

Deceleration is slow because the cruise controller has no brake actuator: it can only coast
(about 1 km/h/s at 100 km/h).

### 7.3 Real-time closed loop over CAN (ECU process + simulator process)

Values from real runs on the UDP software bus; the two runs reported here (manual script run and
integration-test run) agreed within the precision shown where both exist.

| Scenario | Measured |
|---|---|
| Step 60 → 80 km/h | rise (10–90 %) 5.03 s, overshoot 0.88 km/h, settling 8.15 s, steady-state error 0.032 km/h, max throttle 49.8 % |
| 4 % grade at 90 km/h | max deviation 1.43 km/h, recovery 8.55–8.57 s |
| Accelerator override | Override seen on the bus 40 ms after the pedal press |
| Brake | disengaged to Standby 40 ms after the brake press |
| 0x100 lost for 0.5 s | Fault + throttle 0 after **120 ms** (100 ms timeout + COM period), reason `SpeedSignalTimeout`, recovered to Standby |
| Speed invalid for 0.3 s | Fault + throttle 0 after **40 ms**, reason `SpeedInvalid`, recovered to Standby |
| Full demo: engage at 51.8 → 80 km/h | overshoot 1.65 km/h, settling 12.42 s |
| Full demo: 80.1 → 100 km/h | overshoot 0.46 km/h, settling 6.67 s |
| Full demo: 4 % grade at 100 km/h | max deviation 1.58 km/h, recovery 9.42 s |
| Simulator pacing | 0 late steps in all runs; max step lateness 0.97–3.74 ms |

The over-the-bus results match the deterministic C++ results within ~0.1 km/h and ~0.15 s; the
difference is the communication and scheduling delay (about 20–40 ms) in the real loop.
Reaction times are quantised by the message cycles (0x110: 50 ms, 0x200: 20 ms) and the
simulator's 10 ms recording step.

**Observed weakness:** when the driver engages while accelerating hard (40 % pedal in the full
demo), the bumpless start puts 40 % into the integrator. That is far above the ~5 % road load,
and the excess causes the larger overshoot (1.65 km/h). Possible improvement: initialise the
integrator with the pedal minus the acceleration share, estimated from the `LongitudinalAccel`
signal.

---

## 8. Limitations and possible improvements

- Vehicle model simplifications (Section 3); parameters are plausible, not identified from data.
- No brake actuation by the cruise controller (coast-down only); no adaptive cruise / distance control.
- Gains were tuned for this model; the +20 % mass test shows margin, not formal robustness.
- The feedforward uses a nominal mass. An online mass/grade estimator could replace it.
- Integrator initialisation when engaging under strong acceleration (Section 7.3).
- The simulator runs in real time on a non-real-time OS (pacing verified, but not guaranteed).
- Fault handling here is local to the cruise function. Phase 5 adds central fault management and
  DTC storage, and Phase 4 adds a UDS ECU reset that can clear latched controller faults.
