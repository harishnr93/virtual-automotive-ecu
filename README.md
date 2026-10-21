# virtual-automotive-ecu

A hardware-free automotive embedded software platform in C++17 and Python. It simulates
several ECUs that communicate over a virtual CAN network, and it covers periodic
scheduling, cruise control, UDS diagnostics, fault management, watchdog supervision,
firmware-update simulation and automated validation.

> **Scope note.** This is an educational, simulation-only project. It runs on Linux/WSL2
> without any microcontroller or CAN hardware. It borrows ideas from AUTOSAR Classic,
> ISO 14229 (UDS), ISO 15765-2 (ISO-TP) and ISO 26262 *as engineering practice*. It does
> not claim compliance, certification or hard real-time behaviour.

## Status

| Phase | Content | Status |
|---|---|---|
| 1 | Environment, CAN stack, two virtual ECUs, tests | ✅ done |
| 2 | Periodic ECU scheduler (10/20/100 ms), timing measurement | ✅ done |
| 3 | Python vehicle simulator, PID cruise control | ⏳ planned |
| 4 | ISO-TP + UDS diagnostic server | ⏳ planned |
| 5 | Fault management, DTCs, software watchdog | ⏳ planned |
| 6 | Firmware update simulation (CRC32, A/B partitions, rollback) | ⏳ planned |
| 7 | Regression suite, performance report, CI, documentation | ⏳ planned |

## What works today (Phases 1–2)

- **CAN abstraction** `ICanChannel` with three interchangeable backends:
  - Linux **SocketCAN** (`vcan0` / real `can0`)
  - a **UDP-multicast software CAN bus**, used automatically when the kernel has no vcan
    driver (as on stock WSL2)
  - an **in-process bus** for deterministic unit tests
- **DBC-compatible signal codec**: Intel and Motorola bit layouts, factor/offset scaling,
  signed values, range clamping and checks. The CAN matrix in [`config/vehicle_can.dbc`](config/vehicle_can.dbc)
  is checked against the C++ tables by a unit test.
- **Communication supervision**: message timeout monitoring with a first timeout,
  4-bit alive counters, DLC and range checks.
- **Two virtual ECUs** (separate processes): a sensor ECU publishing vehicle data and a
  cruise ECU communication skeleton that supervises its inputs and enters `Fault` on
  communication loss.
- **Periodic task scheduler**: one thread per task with rate-monotonic priorities.
  - **Timing:** absolute-time releases with no drift.
  - **Overruns:** handled OSEK-style, with skipped activations counted.
  - **Detection:** deadline misses and execution-budget overruns.
  - **Measurement:** execution time, start latency, jitter and response time for every
    activation, reported as a table, a summary line per task, or JSON.
- **Priorities:** `SCHED_FIFO` priorities are requested only when asked and fall back
  to normal scheduling when not permitted. The ECU reports which policy it actually got.
- **Inter-task data exchange**: `SharedSignal<T>` with timestamps and sequence numbers, so
  stale data can be detected. Priority-inheritance mutexes prevent unbounded priority inversion.
- **OS abstraction layer** (`os_port`): threads, absolute sleep and a PI mutex. A
  FreeRTOS/Zephyr port would replace only this layer.
- **Event-driven CAN reception**: an RX thread, the equivalent of a receive interrupt,
  feeds a bounded queue that the COM task drains.
- **ECU task sets:**
  - cruise ECU: Control_10ms, Com_20ms, Diag_100ms
  - sensor ECU: Com_10ms, Diag_100ms
- **Tooling**:
  - candump-format CAN traces
  - Python `vbus_dump.py` / `vbus_send.py` for the software bus
  - `vecu_sched_bench` scheduler benchmark
  - environment check and demo script
- **Quality**:
  - 113 GoogleTest and pytest tests
  - strict warnings with `-Werror`
  - ASan/UBSan and Clang ThreadSanitizer builds
  - cppcheck and clang-tidy

## Quick start (WSL2 / Ubuntu 22.04)

```bash
sudo apt install -y build-essential cmake libgtest-dev python3-pytest   # once
./scripts/check_env.sh

cmake -S . -B build -DVECU_WARNINGS_AS_ERRORS=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

./scripts/run_demo.sh 5          # two ECUs talk for 5 s; logs + timing report
./scripts/vbus_dump.py           # (other terminal) watch the bus like candump
./build/bin/vecu_sched_bench --duration 10   # measure scheduler timing
```

## Architecture

```
 vecu_sensor_ecu        vecu_cruise_ecu          ← applications: 10/20/100 ms tasks
        │                      │
 PeriodicScheduler · SharedSignal · timing analysis · os_port   ← include/core
 CAN RX thread + queue · signal codec · CAN matrix · supervision ← include/can
        │                      │
 ───────────── ICanChannel (open/send/receive) ─────────────
   SocketCAN (vcan0)  │  UDP software bus  │  in-memory test bus
```

## Measured results

Recorded on WSL2 (kernel 6.6.87.2, 16 logical CPUs), GCC 11.4 / Clang 14. These are observations on a
non-real-time OS, not guarantees.

| Measurement | Result |
|---|---|
| Test suite (2026-10-21) | 113 tests: 111 passed, 2 skipped (need `vcan0`), 0 failed |
| ASan + UBSan / Clang ThreadSanitizer runs | no findings / no data races |
| cppcheck / clang-tidy | no remaining findings |
| Timeout detection for `0x100` (configured 100 ms; Phase 1 loop, 10 runs) | 100.1 – 110.2 ms after the last frame |
| Scheduler, light load, 10 s (`SCHED_OTHER`) | 0 deadline misses in 1599 activations; start latency mean 266–534 µs, max 1.7 ms |
| Scheduler + 32 CPU-stress threads, 10 s | 2 deadline misses (10 ms task); max start latency 11–14 ms → needs RT priorities |
| Overloaded 10 ms task (12 ms work), 10 s | all 832 activations flagged as misses, 167 releases dropped, other tasks unaffected |

Details and method: [docs/phase2_scheduler.md](docs/phase2_scheduler.md#8-tests-and-measured-results).

## Repository layout

```
include/ src/      libraries: core (OS port, scheduler, timing, logger), can (stack, RX thread, supervision)
                   control/ diagnostics/ faults/ watchdog/ firmware/  ← later phases
apps/              ECU executables (composition roots)
simulator/         Python package (software-bus client; vehicle simulator in Phase 3)
config/            CAN matrix (DBC)
tests/unit, tests/integration   GoogleTest + pytest
scripts/           environment check, vcan setup, demo, bus tools, static analysis
docs/              per-phase engineering documentation
```

## Documentation

- [Phase 1 — Development environment and virtual CAN](docs/phase1_virtual_can.md)
- [Phase 2 — ECU scheduler](docs/phase2_scheduler.md)

## License

MIT, see [LICENSE](LICENSE).
