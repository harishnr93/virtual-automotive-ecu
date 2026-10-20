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
| 2 | Periodic ECU scheduler (10/20/100 ms), timing measurement | ⏳ planned |
| 3 | Python vehicle simulator, PID cruise control | ⏳ planned |
| 4 | ISO-TP + UDS diagnostic server | ⏳ planned |
| 5 | Fault management, DTCs, software watchdog | ⏳ planned |
| 6 | Firmware update simulation (CRC32, A/B partitions, rollback) | ⏳ planned |
| 7 | Regression suite, performance report, CI, documentation | ⏳ planned |

## What works today (Phase 1)

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
- **Tooling**: candump-format CAN traces, Python `vbus_dump.py` / `vbus_send.py` for the
  software bus, environment check, demo script.
- **Quality**: GoogleTest and pytest, 72 tests. Strict warnings with `-Werror`,
  ASan/UBSan build, cppcheck and clang-tidy.

## Quick start (WSL2 / Ubuntu 22.04)

```bash
sudo apt install -y build-essential cmake libgtest-dev python3-pytest   # once
./scripts/check_env.sh

cmake -S . -B build -DVECU_WARNINGS_AS_ERRORS=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

./scripts/run_demo.sh 5          # two ECUs talk for 5 s; logs in logs/
./scripts/vbus_dump.py           # (other terminal) watch the bus like candump
```

## Architecture (Phase 1)

```
 vecu_sensor_ecu        vecu_cruise_ecu          ← applications (apps/)
        │                      │
 signal codec · CAN matrix · timeout monitor · alive counter   ← include/can, src/can
        │                      │
 ───────────── ICanChannel (open/send/receive) ─────────────
   SocketCAN (vcan0)  │  UDP software bus  │  in-memory test bus
```

## Measured results (Phase 1)

Recorded on 2026-10-20 on WSL2 (kernel 6.6.87.2), GCC 11.4. These are observations on a
non-real-time OS, not guarantees.

| Measurement | Result |
|---|---|
| Test suite | 72 tests: 70 passed, 2 skipped (need `vcan0`), 0 failed |
| ASan + UBSan run of the same suite | no findings |
| cppcheck / clang-tidy | no remaining findings |
| Timeout detection for `0x100` (configured 100 ms, 10 runs) | 100.1 – 110.2 ms after the last frame |
| Max lateness of the 10 ms loop tick (10 runs × 2 s) | 0.70 – 2.03 ms |

## Repository layout

```
include/ src/      libraries: core (clock, logger, RAII), can (stack + supervision)
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

## License

MIT, see [LICENSE](LICENSE).
