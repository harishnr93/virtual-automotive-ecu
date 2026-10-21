# Phase 1 — Development Environment and Virtual CAN

This document explains **what** was built in Phase 1, **why** it was built that way, and
**how** to build, run and test it. It is written for readers who are learning automotive
embedded software architecture.

---

## 1. Engineering objective

Create the foundation that every later phase depends on:

1. A reproducible build (CMake, strict compiler warnings, tests, static analysis).
2. A **CAN communication stack** in C++17:
   frame model → signal packing (DBC-compatible) → transport backends → supervision.
3. **Two virtual ECUs** running as separate Linux processes that exchange periodic CAN
   messages, detect communication loss, and recover from it.

---

## 2. Automotive concepts used in this phase

### 2.1 ECU
An **Electronic Control Unit** is a small computer in a vehicle (engine, brakes, airbag,
cruise control …). A modern car has 50–150 of them. Each ECU reads inputs, runs control
software periodically, and talks to other ECUs over in-vehicle networks.
Here, each ECU is a **Linux process** — a simulation of an ECU, not a real one.

### 2.2 CAN bus
**Controller Area Network** (ISO 11898) is the most common in-vehicle network.

| Property | Meaning |
|---|---|
| Broadcast | Every node receives every frame; nodes filter by identifier. There are no addresses. |
| Identifier | 11-bit (standard) or 29-bit (extended). It names the *content*, not the sender, and also sets the priority (lower ID wins arbitration). |
| Payload | Classic CAN: 0–8 bytes. The length is the **DLC** (Data Length Code). |
| Periodic | Most messages are sent cyclically (e.g. every 10/20/100 ms), so receivers notice when a sender stops. |

A frame written in can-utils notation: `100#1400DA0A01000001` → ID `0x100`, 8 data bytes.

### 2.3 Signals and the DBC file
Bytes alone mean nothing. A **signal** is a group of bits inside a frame with a scaling
rule:

```
physical value = raw value × factor + offset
```

Example (`VehicleSpeed`, 16 bits, factor 0.01): raw `0x3039` = 12345 → 123.45 km/h.

The industry describes all messages and signals in a **DBC file** (the "CAN matrix").
Ours is [`config/vehicle_can.dbc`](../config/vehicle_can.dbc). Tools such as
`cantools`, SavvyCAN or Vector CANdb++ can open it.

Two bit layouts exist:
- **Intel / little-endian** (`@1` in DBC): start bit = least significant bit.
- **Motorola / big-endian** (`@0`): start bit = most significant bit, using DBC's
  "sawtooth" bit numbering. Both are implemented and unit-tested; the project matrix uses Intel.

### 2.4 Communication supervision
Safety-relevant receivers must notice when data is missing or stale:

| Mechanism | Detects | Implementation |
|---|---|---|
| **Message timeout** (deadline monitoring) | Sender stopped / bus disconnected | `MessageTimeoutMonitor` |
| **Alive counter** (rolling counter, 0…15) | Lost frames, repeated/frozen data | `AliveCounterChecker` |
| **DLC check** | Wrong message layout | in each ECU's receive handler |
| **Range check** | Physically impossible value | `decodePhysical()` returns `OutOfRange` |

These correspond to ideas from AUTOSAR COM deadline monitoring and AUTOSAR E2E
protection; this project implements them in a simplified, educational form.

### 2.5 Linux SocketCAN and vcan
**SocketCAN** is the Linux kernel's CAN API: a CAN interface is used through ordinary
sockets. **vcan** is a virtual CAN driver: it creates an interface (`vcan0`) that behaves
like a CAN bus without hardware. `can-utils` (`candump`, `cansend`) work on such interfaces.

---

## 3. Environment findings (measured on this machine)

| Item | Result |
|---|---|
| OS / kernel | Ubuntu 22.04.5 LTS on WSL2, kernel `6.6.87.2-microsoft-standard-WSL2` |
| Compiler / build | GCC 11.4.0, CMake 3.22.1, GDB 12.1 |
| Python / tests | Python 3.10.12, pytest 6.2.5, GoogleTest 1.11 (libgtest-dev) |
| Static analysis | cppcheck 2.7, clang-tidy 14 |
| SocketCAN core | `can.ko`, `can-raw.ko` present; AF_CAN raw sockets can be created |
| **vcan driver** | **Not available**: `# CONFIG_CAN_VCAN is not set`, no `vcan.ko` |
| can-utils | Not installed (requires `sudo apt install can-utils`) |

**Consequence:** on the stock WSL2 kernel, `vcan0` cannot be created. Building a custom
WSL2 kernel would work but is a heavy, fragile step. Instead the project provides a
**software-only CAN bus with the same application-facing interface** (Section 4.3),
selected automatically. On kernels that do have vcan (native Ubuntu, GitHub Actions
runners), the real SocketCAN backend is used without any code change.

Run `./scripts/check_env.sh` to repeat these checks.

---

## 4. Architecture

```
 ┌──────────────────────┐            ┌──────────────────────┐
 │  vecu_sensor_ecu     │            │  vecu_cruise_ecu     │   apps/  (composition roots)
 │  synthetic vehicle   │            │  Phase-1 COM stub    │
 └─────────┬────────────┘            └──────────┬───────────┘
           │ signals (km/h, %, bool)            │
 ┌─────────▼────────────────────────────────────▼───────────┐
 │ Signal layer:  vehicle_signals.hpp (CAN matrix = DBC)    │   include/can, src/can
 │                can_signal (pack/unpack, scale, range)    │
 │ Supervision:   MessageTimeoutMonitor, AliveCounter       │
 ├──────────────────────────────────────────────────────────┤
 │ ICanChannel  (open / send / receive / stats)             │ ◄── the abstraction boundary
 ├───────────────┬────────────────────┬─────────────────────┤
 │SocketCanChannel│  UdpCanChannel    │  VirtualCanBus      │
 │ vcan0 / can0   │  multicast on lo  │  in-process (tests) │
 └───────────────┴────────────────────┴─────────────────────┘
```

### 4.1 Repository layout (Phase 1 content)

| Path | Content |
|---|---|
| `include/core`, `src/core` | `clock.hpp` (monotonic time), `logger`, `UniqueFd` (RAII fd), `RingBuffer` |
| `include/can`, `src/can` | frame, status codes, `ICanChannel`, 3 backends, factory, signal codec, CAN matrix, timeout monitor, alive counter, candump log writer |
| `apps/` | `vecu_sensor_ecu`, `vecu_cruise_ecu`, shared option parsing and the Phase 1 loop |
| `config/vehicle_can.dbc` | CAN matrix |
| `simulator/vecu_sim/udp_can_bus.py` | Python client of the software CAN bus |
| `scripts/` | environment check, vcan setup, demo, bus monitor/injector, static analysis |
| `tests/unit`, `tests/integration` | GoogleTest + pytest |
| `include|src/{control,diagnostics,faults,watchdog,firmware}` | empty placeholders for later phases |

**Structure adjustment (justified):** an `apps/` directory was added. Executables are
*composition roots* — they only wire libraries together. Keeping `main()` functions out
of `src/` lets every library be linked into tests without dragging in a `main()`.

### 4.2 The CAN matrix

| ID | Message | Sender → Receiver | Cycle | Rx timeout |
|---|---|---|---|---|
| `0x100` | VehicleDynamics | VehicleSim → CruiseECU | 20 ms | 100 ms |
| `0x110` | DriverInputs | VehicleSim → CruiseECU | 50 ms | 250 ms |
| `0x200` | CruiseControlStatus | CruiseECU → VehicleSim | 20 ms | 100 ms |
| `0x7E0/0x7E8` | (reserved for UDS, Phase 4) | | | |

Signals (all Intel byte order):

| Message | Signal | Bits | Factor / offset | Range | Unit |
|---|---|---|---|---|---|
| 0x100 | VehicleSpeed | 0–15 | 0.01 / 0 | 0…300 | km/h |
| 0x100 | LongitudinalAccel (signed) | 16–31 | 0.001 / 0 | −20…20 | m/s² |
| 0x100 | VehicleSpeedValid | 32 | 1 / 0 | 0…1 | |
| 0x100/0x110/0x200 | AliveCounter | 56–59 | 1 / 0 | 0…15 | |
| 0x110 | AcceleratorPedal | 0–7 | 0.5 / 0 | 0…100 | % |
| 0x110 | BrakePedalPressed / CruiseMainSwitch / CruiseSetRequest / CruiseCancelRequest | 8 / 9 / 10 / 11 | 1 / 0 | 0…1 | |
| 0x110 | DriverSetSpeed | 16–23 | 1 / 0 | 0…200 | km/h |
| 0x200 | ThrottleCommand | 0–15 | 0.01 / 0 | 0…100 | % |
| 0x200 | CruiseState (0 Off, 1 Standby, 2 Active, 3 Override, 4 Fault) | 16–18 | 1 / 0 | 0…4 | |
| 0x200 | CruiseTargetSpeed | 24–31 | 1 / 0 | 0…200 | km/h |
| 0x200 | CruiseFaultReason (added in Phase 3; 0 None … 5 ControllerFault) | 32–35 | 1 / 0 | 0…15 | |

The C++ tables in `vehicle_signals.hpp` are `constexpr` (read-only memory, like flash on
an MCU). The unit test `VehicleSignals.MatchesDbcFile` parses the DBC and fails if the two
ever diverge, which prevents ECUs being built from different matrix versions.

### 4.3 The three CAN backends

| Backend | When | How it works | Fidelity |
|---|---|---|---|
| `SocketCanChannel` | `vcan0`/`can0` exists | `CAN_RAW` socket bound to the interface | Real Linux CAN stack; works with can-utils |
| `UdpCanChannel` | stock WSL2 (fallback) | each node joins multicast group `239.255.0.1:47000` on `127.0.0.1`, TTL 0 | Broadcast + per-sender ordering; **no** arbitration, bit timing, error frames, bus-off |
| `VirtualCanBus` | unit tests | in-process, mutex + fixed-size queues | Deterministic; models RX-queue overruns |

`--bus auto` (default) tries SocketCAN first and falls back to UDP, logging why:
```
[WARN ] [can] SocketCAN interface 'vcan0' unavailable: NotAvailable (No such device); falling back to UDP software CAN bus
```

UDP datagram layout (24 bytes, little-endian) — identical in C++ and Python:
`"VCAN" | version | flags | dlc | reserved | sender_id(4) | can_id(4) | data(8)`.
The random `sender_id` lets a node discard its own looped-back frames, matching SocketCAN's
default behaviour.

### 4.4 The two virtual ECUs

**`vecu_sensor_ecu`** (stand-in for the Phase 3 Python vehicle simulator)
- Every 20 ms sends `0x100` from a synthetic profile (0→100 km/h in 10 s and back).
- Every 50 ms sends `0x110` (cruise main switch on after 1 s, set speed 80 km/h).
- Supervises `0x200` from the cruise ECU.
- Fault injection: `--mute-tx START:END` stops `0x100` for a time window.

**`vecu_cruise_ecu`** (Phase 1 = communication skeleton)
- Receives and validates `0x100` and `0x110` (DLC, alive counter, value range, timeout).
- Every 20 ms sends `0x200` with `CruiseState` and `ThrottleCommand = 0 %`.
- State: `Off` → `Standby` when inputs are present and the main switch is on;
  `Fault` while any input message is timed out. **No control yet** — PID and the full
  state machine come in Phase 3.

In Phase 1 both ran a simple single-threaded loop: wait for CAN frames until the next 10 ms
tick, then run the cyclic function.

> **Update (Phase 2):** this loop has been replaced by the periodic task scheduler, a CAN
> reception thread and 10/20/100 ms tasks — see [phase2_scheduler.md](phase2_scheduler.md).
> The timeout-detection and loop-lateness values in Section 8 were measured with the Phase 1 loop.

---

## 5. Design decisions

| Decision | Reason |
|---|---|
| `ICanChannel` interface + factory | Applications never see Linux headers; backend can change (vcan, UDP, test bus, later an RTOS CAN driver) without touching application code. |
| Status codes, no exceptions in the runtime | Typical for embedded/safety code: every error path is visible at the call site; `[[nodiscard]]` forces callers to handle results. |
| RAII (`UniqueFd`, `unique_ptr<FILE>`) | Sockets and files are closed on every path, including early returns. |
| Fixed-size containers (`std::array`, `RingBuffer`, template-sized monitor) | No heap allocation in the receive/supervision path; bounded memory. |
| `constexpr` signal tables | Zero start-up cost, read-only, compile-time checked. |
| Monotonic clock for all timing | Wall-clock jumps (NTP) must not trigger or hide timeouts. |
| Time passed into supervision logic | `MessageTimeoutMonitor` is pure logic → deterministic tests with no sleeps. |
| Non-blocking `send()` | An ECU task must never stall on a full transmit queue; it returns `TxBufferFull`. |
| First-timeout (1000 ms) vs. cyclic timeout | Gives other ECUs time to start (cf. AUTOSAR "first timeout"). |
| Alive counter re-sync after timeout | After a communication loss the counter jump is expected, not a separate error. |
| candump-format trace files | Logs can be processed with standard tools (`canplayer`, `log2asc`). |

---

## 6. Build, run and test (exact WSL commands)

```bash
cd ~/harish_ws/vir_auto_ecu

# 0) (optional) install tools not present on this machine
sudo apt update && sudo apt install -y can-utils     # only useful with a vcan-capable kernel

# 1) check the environment
./scripts/check_env.sh

# 2) configure + build (strict warnings as errors)
cmake -S . -B build -DVECU_WARNINGS_AS_ERRORS=ON
cmake --build build -j"$(nproc)"

# 3) run all tests (C++ unit + integration, Python unit + multi-process)
ctest --test-dir build --output-on-failure
ctest --test-dir build -L unit            # only unit tests
ctest --test-dir build -L integration     # only integration tests

# 4) run the two ECUs for 5 seconds (logs + CAN traces in logs/)
./scripts/run_demo.sh 5

# 5) or run them manually in two terminals
./build/bin/vecu_cruise_ecu                     # terminal 1
./build/bin/vecu_sensor_ecu                     # terminal 2 (Ctrl+C to stop both)

# 6) watch the bus (UDP fallback) — like candump
./scripts/vbus_dump.py                          # terminal 3
./scripts/vbus_dump.py -f 200 -n 10             # only 0x200, 10 frames
./scripts/vbus_send.py 123#DEADBEEF             # inject a frame (like cansend)

# 7) fault injection: stop 0x100 between 2 s and 3 s, watch the cruise ECU react
./build/bin/vecu_cruise_ecu --duration 5 &
./build/bin/vecu_sensor_ecu --duration 5 --mute-tx 2:3

# 8) memory/UB checking build
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DVECU_ENABLE_SANITIZERS=ON
cmake --build build-asan -j"$(nproc)" && ctest --test-dir build-asan

# 9) static analysis
./scripts/run_static_analysis.sh build
```

### With a vcan-capable kernel (native Linux, CI, custom WSL2 kernel)
```bash
sudo ./scripts/setup_vcan.sh                     # creates vcan0
candump -td vcan0 &                              # can-utils monitor
./build/bin/vecu_cruise_ecu --bus socketcan &
./build/bin/vecu_sensor_ecu --bus socketcan --duration 5
```
The SocketCAN integration tests run automatically once `vcan0` exists (otherwise they are
reported as *skipped*, not passed).

---

## 7. Expected behaviour

Cruise ECU console (shortened):
```
[WARN ] [can] SocketCAN interface 'vcan0' unavailable ... falling back to UDP software CAN bus
[INFO ] [cruise_ecu] RX_FIRST id=0x100 after_ms=19.9
[INFO ] [cruise_ecu] RX_FIRST id=0x110 after_ms=49.8
[INFO ] [cruise_ecu] CRUISE_STATE Off -> Standby                  # main switch on at t=1 s
[INFO ] [cruise_ecu] SUMMARY ecu=cruise_ecu bus=vbus0 tx=150 tx_failed=0 rx_0x100=149 rx_0x110=59 ... timeouts=0
```
With `--mute-tx` on the sensor:
```
[WARN ] [cruise_ecu] RX_TIMEOUT id=0x100 silence_ms=100.4 previous=Ok
[INFO ] [cruise_ecu] CRUISE_STATE Standby -> Fault
[INFO ] [cruise_ecu] RX_RECOVERED id=0x100 gap_ms=...
[INFO ] [cruise_ecu] CRUISE_STATE Fault -> Standby
```

---

## 8. Test inventory and measured results

| Suite | Type | What it proves |
|---|---|---|
| `RingBuffer.*` | unit | FIFO order, wrap-around, overflow policy |
| `CanFrame.*` | unit | ID/DLC limits, equality, candump formatting |
| `CanSignalRaw.*` / `CanSignalPhysical.*` | unit | Intel + Motorola bit layout with known vectors, exhaustive round-trip of all positions/lengths, scaling, offset, signed values, clamping, NaN rejection, out-of-range detection |
| `VehicleSignals.*` | unit | C++ matrix == DBC file; no overlapping signals; timeout ≥ 3 × cycle |
| `UdpWireFormat.*` + `python_unit` | unit | C++ and Python agree byte-for-byte on a shared reference vector; malformed datagrams rejected |
| `MessageTimeoutMonitor.*` | unit | first timeout, exact deadline, single event per timeout, recovery — with simulated time |
| `AliveCounter.*` | unit | wrap-around, repeated, skipped, invalid, resync |
| `VirtualCanBus.*` | unit | broadcast semantics, no self-reception, overrun counting, cross-thread wake-up, lifetime safety |
| `CanLogWriter.*` | unit | candump log format, RAII file handling, unwritable path |
| `UdpCanChannel.*`, `CanChannelFactory.*` | integration | real sockets: bidirectional exchange, broadcast, ordering, timeouts, configuration errors, auto fallback |
| `SocketCanChannel.*` | integration | error handling always; vcan exchange **skipped** when vcan0 is absent |
| `python_integration_two_ecus` | integration | two ECU **processes**: frame counts, zero alive errors, timeout + recovery with fault injection, C++ → Python decoding |

Results recorded on 2026-10-20 on the machine described in Section 3:

- `ctest`: **72 tests, 70 passed, 2 skipped** (vcan tests), 0 failed.
- Same 72 tests under AddressSanitizer + UndefinedBehaviorSanitizer: no findings.
- `cppcheck` and `clang-tidy` (profile in `.clang-tidy`): no remaining findings.
- Timeout detection, 10 runs, `0x100` muted, configured timeout 100 ms, 10 ms supervision
  tick: measured silence at detection **100.1 – 110.2 ms**.
- Phase 1 loop tick lateness (maximum per 2 s run, same 10 runs): **0.70 – 2.03 ms**.

These numbers describe a non-real-time OS (WSL2) on a desktop PC. They are *observations*,
not guarantees; Phase 2 adds systematic scheduler timing measurement.

---

## 9. Limitations and possible improvements

- **No vcan on stock WSL2.** The UDP bus reproduces broadcast semantics, not the physical
  layer (no arbitration, error frames, bus-off, bit timing, bus-load limits).
- **Not real-time.** Linux/WSL2 scheduling can delay ticks; nothing here is a hard
  real-time guarantee.
- **Logging in the loop.** Trace writes use buffered stdio in the ECU loop; a later phase
  can move this to a background thread.
- **Classic CAN only.** CAN FD (64-byte payloads) and remote frames are out of scope.
- **Single-threaded loop.** Replaced in Phase 2 by the multi-rate scheduler.
- **Signal tables hand-written.** A future improvement is generating `vehicle_signals.hpp`
  from the DBC (the consistency test already guards against drift).
- Optional: build a WSL2 kernel with `CONFIG_CAN_VCAN=m` to use can-utils directly.
