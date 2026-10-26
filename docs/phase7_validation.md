# Phase 7 — Validation, CI and Documentation

This phase turns the six previous phases into a project that **anyone can build, test, measure
and review with one command each**, locally or in GitHub Actions. It also documents the result:
architecture, decisions, measured performance, and which requirement is verified by which test.
It continues [Phase 6](phase6_firmware_update.md).

---

## 1. Engineering objective

| Plan item | Covered by |
|---|---|
| Automated regression tests | `scripts/run_regression.sh`: GCC `-Werror`, ASan + UBSan, Clang TSan, cppcheck + clang-tidy, with a summary and JUnit reports. System tests are split into one labelled CTest entry per area |
| Measure system performance | `scripts/run_performance.py` writes [performance_report.md](performance_report.md) from one measurement run; new tool `vecu_can_latency` for the CAN latency that was not yet measured |
| Architecture diagrams | [architecture.md](architecture.md): context, layers, tasks and data flow, sequences, state machines (Mermaid) |
| Document engineering decisions | [engineering_decisions.md](engineering_decisions.md): 16 decisions with problem, alternatives and price |
| Professional README | rewritten [README](../README.md): what it is, results, how to run, honest scope |
| GitHub Actions CI | [`.github/workflows/ci.yml`](../.github/workflows/ci.yml): the regression stages as jobs, a SocketCAN job, a manual performance job |

---

## 2. Concepts

### 2.1 Regression testing
A *regression* is something that used to work and is broken by a later change. Protection comes
from running **all** tests after **every** change, automatically. The tests must therefore be:
- **fast enough** to run often: the whole suite takes about 3 minutes
- **deterministic**: the same code gives the same result, so a red result means a real problem
- **diagnosable**: a failure names the broken area

Before this phase, all Python system tests ran as one CTest entry (`python_integration`). Now each
area is its own entry with labels, so a report says `system_firmware_update FAILED` and you can run
one area alone:

```bash
ctest --test-dir build -L firmware     # everything labelled firmware
ctest --test-dir build -L system       # all multi-process system tests
ctest --test-dir build -L unit         # deterministic logic tests only (~3 s with -j16)
```
The system tests check real-time behaviour, so each runs **alone** (`RUN_SERIAL`): nothing else
runs in parallel, not even the unit tests (see 7.4 for why).

### 2.2 Several builds of the same code
| Build | Finds |
|---|---|
| GCC, `-Wall -Wextra -Wconversion … -Werror` | suspicious code at compile time |
| AddressSanitizer + UndefinedBehaviorSanitizer | out-of-bounds access, use after free, leaks, signed overflow, invalid shifts, at run time |
| ThreadSanitizer (Clang) | data races between threads |
| cppcheck, clang-tidy | bug patterns and style issues without running the code |

The system tests start the *sanitized* ECU binaries (each build directory runs its own `bin/`) and
check their exit codes, so a sanitizer report inside an ECU process fails the test.

### 2.3 Continuous integration
CI runs the regression stages on a fresh machine for every push and pull request. That proves the
build does not depend on anything that exists only on the developer's machine: it needs only
packages that are installed explicitly, and only files in the repository. The workflow uses
`ubuntu-22.04`, the same distribution and compilers (GCC 11, Clang 14) as the development
environment.

### 2.4 Measuring performance honestly
- **Measure, don't estimate.** Every value in the performance report comes from one run of the
  script, stored with the date, host, compiler and commit.
- **Report distributions, not single numbers.** Medians show the typical case, and p99 and
  maximum show the outliers that matter for deadlines.
- **Say what is included.** For example, the CAN latency is `send()` → kernel → receiver wake-up;
  it does *not* include the wire time of a physical bus.
- **No guarantees on a general-purpose OS.** Linux/WSL2 without real-time priorities or CPU
  isolation has no worst-case bound. The scheduler table under CPU stress shows how much the
  numbers move.

### 2.5 Traceability
For each requirement: where is it implemented, which test proves it, and where is the evidence?
Section 5 answers this for every requirement of the project specification.

---

## 3. Files

| File | Content |
|---|---|
| `scripts/run_regression.sh` | regression runner (the CI stages locally), logs + JUnit in `logs/regression/` |
| `scripts/run_performance.py` | measurement run → `docs/performance_report.md` + `logs/performance/results.json` |
| `apps/can_latency/main.cpp` | `vecu_can_latency`: one-way CAN latency with optional background traffic |
| `tests/integration/test_can_latency.py` | the latency tool must deliver every probe (values are reported, not asserted) |
| `tests/CMakeLists.txt` | system tests as labelled CTest entries, each run serially |
| `.github/workflows/ci.yml` | GitHub Actions workflow |
| `docs/architecture.md`, `docs/engineering_decisions.md`, `docs/performance_report.md` | documentation |
| `README.md` | rewritten |

---

## 4. Test suite

| Level | Tool | Count | What it tests | Time |
|---|---|---|---|---|
| C++ unit | GoogleTest | 241 | pure logic with synthetic time and an in-memory bus: codecs, supervision, scheduler arithmetic, PID, cruise state machine, ISO-TP, UDS, fault manager, DTCs, NVM, watchdog, firmware | ~3 s* |
| C++ integration | GoogleTest | 24 | real OS resources: UDP sockets, SocketCAN (when `vcan0` exists), real threads and timing | ~1 s* |
| Python unit | pytest | 59 | DBC parser, vehicle model, scenarios, ISO-TP / UDS client, UDP bus, firmware image | ~1 s |
| System | pytest + real processes | 33 in 7 areas | ECU processes + simulator + tester on isolated buses: CAN, scheduler, closed loop, diagnostics, faults and watchdog, firmware update, CAN latency | ~2.7 min (serial) |

\* with `ctest -j16`. The whole suite takes ~2.8 min, almost all of it the serial system tests.

CTest shows **273 entries**: the GoogleTest cases one by one, `python_unit` as one entry, and the
7 system areas.

Test categories required by the specification:

| Category | Tests |
|---|---|
| Unit tests | `vecu_unit_tests`, `tests/unit/python` |
| Integration tests | `vecu_integration_tests`, `system_*` |
| CAN message tests | `CanFrame`, `CanSignalRaw/Physical`, `VehicleSignals`, `UdpWireFormat`, `VirtualCanBus`, `CanLogWriter`, `CanReceiver`, `MessageTimeoutMonitorTest`, `AliveCounter`, `UdpCanChannel`, `SocketCanChannel`, `test_dbc.py`, `test_udp_can_bus.py`, `system_two_ecu_communication`, `system_can_latency` |
| PID controller tests | `PidController`, `PidConfig`, `CruiseClosedLoop`, `system_closed_loop` |
| Diagnostic protocol tests | `IsoTp`, `IsoTpStMin`, `UdsServerTest`, `UdsProgrammingTest`, `test_isotp_uds_client.py`, `system_diagnostics` |
| Fault injection tests | `FaultManagerTest`, `WatchdogTest`, `CruiseControllerTest` (fault cases), `system_fault_management` (injected stall, overrun, hang, NaN, signal faults), `system_closed_loop` (sensor faults) |
| Scheduler timing tests | `TaskTimingMonitor*`, `ComputeNextRelease`, `SchedulerConfig`, `PeriodicScheduler` (real threads), `system_scheduler_bench` |
| Firmware update recovery tests | `FirmwareTest` (power loss at 21 points, rollback), `RamFlash`, `system_firmware_update` (SIGKILL, defective image) |

---

## 5. Requirements traceability

Requirement → implementation → verification. "Evidence" points to the measured result.

### A. Virtual CAN communication
| Requirement | Implementation | Verified by |
|---|---|---|
| SocketCAN with vcan0; check vcan support, software fallback | `SocketCanChannel`, `UdpCanChannel`, `createCanChannel` (auto), `scripts/check_env.sh` | `SocketCanChannel.*` (skipped without vcan; CI job `socketcan`), `UdpCanChannel.*`, `CanChannelFactory.*` |
| C++ transmitter and receiver | `ICanChannel`, `CanReceiver` thread | `UdpCanChannel.*`, `CanReceiver.*`, `system_two_ecu_communication` |
| Encoding / decoding, identifiers, signal definitions, DBC | `can_signal`, `vehicle_signals`, `config/vehicle_can.dbc` | `CanSignal*`, `VehicleSignals.*` (C++ table = DBC), `test_dbc.py` |
| Periodic transmission | Com tasks of both ECUs | `system_two_ecu_communication` (frame counts and periods) |
| Timeout detection | `MessageTimeoutMonitor`, alive counters | `MessageTimeoutMonitorTest.*`, `AliveCounter.*`, system tests; evidence: [performance report §4](performance_report.md) |
| Logging | `CanLogWriter` (candump format) | `CanLogWriter.*` |
| Error handling | `CanStatus`, rejected / overrun counters, `SO_RXQ_OVFL` | `UdpCanChannel.KernelQueueDropsAreCountedAsOverruns`, `UdpWireFormat.*` |

### B. Scheduler
| Requirement | Implementation | Verified by |
|---|---|---|
| 10 / 20 / 100 ms tasks, priorities, safe synchronisation | `PeriodicScheduler`, `os_port` (PI mutex), `SharedSignal` | `PeriodicScheduler.*`, `SharedSignal.*`, `PiMutex.*`, TSan build |
| Execution time, deadline misses, jitter | `TaskTimingMonitor`, timing report | `TaskTimingMonitor*`, `system_scheduler_bench`; evidence: performance report §1, §5 |
| No hard real-time claim; RTOS-ready | `os_port` abstraction | documentation ([Phase 2](phase2_scheduler.md)) |

### C. Vehicle dynamics simulator
| Requirement | Implementation | Verified by |
|---|---|---|
| Speed, acceleration, throttle, brake, resistance, driver requests | `simulator/vecu_sim/vehicle_model.py`, `scenario.py` | `test_vehicle_model.py`, `test_scenario_and_simulator.py`; assumptions in [Phase 3](phase3_cruise_control.md) |
| Exchange over the communication abstraction | `simulator.py` (UDP / SocketCAN) | `system_closed_loop` |

### D. Cruise control ECU
| Requirement | Implementation | Verified by |
|---|---|---|
| PID, saturation, anti-windup | `PidController` | `PidController.*`, `PidConfig.*` |
| State machine, brake override, sensor timeout, safe fallback | `CruiseController` | `CruiseControllerTest.*`, `CruiseClosedLoop.*`, `system_closed_loop`; evidence: performance report §3, §4 |
| Algorithm separated from communication and scheduling | `vecu_control` has no CAN / thread / clock dependency | build structure ([architecture](architecture.md#2-software-layers)) |

### E. Diagnostics
| Requirement | Implementation | Verified by |
|---|---|---|
| 0x10, 0x11, 0x22, 0x19, 0x14, 0x3E; sessions; positive / negative responses | `UdsServer` | `UdsServerTest.*`, `system_diagnostics`; evidence: performance report §6 |
| ISO-TP | `IsoTpEndpoint` (C++), `isotp.py` | `IsoTp.*`, `test_isotp_uds_client.py` |
| Scope documented | [Phase 4](phase4_diagnostics.md), [Phase 6](phase6_firmware_update.md#210-the-uds-programming-sequence) | — |

### F. Fault management
| Requirement | Implementation | Verified by |
|---|---|---|
| Sensor timeout, invalid values, CAN timeout, deadline misses, controller faults | monitors in `cruise_ecu.cpp`, table `cruise_faults.hpp` | `system_fault_management`, `system_closed_loop` |
| DTC storage, state transitions, recovery | `FaultManager`, `DtcStore`, NVM | `FaultManagerTest.*`, `DtcStoreTest.*`, `DtcAging.*`, `NvmTest.*` |
| Deterministic fault injection | synthetic-time tests, `--inject`, simulator faults | as above |

### G. Watchdog
| Requirement | Implementation | Verified by |
|---|---|---|
| Missing heartbeat, execution timeout, stalled application | `WatchdogSupervisor` | `WatchdogTest.*` |
| Configurable supervision, simulated recovery, not a hardware watchdog | per-task limits, `--watchdog-action` | `system_fault_management` (stall → reset, hang → exit 75 → restart); evidence: performance report §4 |

### H. Firmware update
| Requirement | Implementation | Verified by |
|---|---|---|
| Metadata, CRC32, version check, state machine, partitions, interrupted update, rollback | `vecu_firmware`, `FirmwareService` | `FirmwareImage.*`, `FirmwareTest.*`, `UdsProgrammingTest.*`, `test_firmware_image.py`, `system_firmware_update`; evidence: performance report §7 |
| CRC32 is not authentication; extensible for signatures | `IImageVerifier` | `test_crc32_is_not_authentication`, `FirmwareImage.SignatureMethodIsReservedNotSilentlyAccepted` |

### I. Measurements required by the specification
| Measurement | Where |
|---|---|
| Task execution time, scheduling jitter | performance report §1 (benchmark) and §5 (ECU tasks) |
| CAN message latency | performance report §2 |
| Controller response | performance report §3 |
| Fault detection latency | performance report §4 |
| Diagnostic response time | performance report §6 |

### Software engineering requirements
| Requirement | How it is met / checked |
|---|---|
| Modular C++17, clear interfaces, RAII | 7 libraries with one-way dependencies ([architecture](architecture.md#2-software-layers)); file descriptors, threads and locks owned by RAII classes |
| Minimal dynamic allocation in time-critical paths | fixed-capacity containers on the task path: ring buffers, CAN queues, DTC store, DID/routine tables, ISO-TP buffers. Allocation happens at start-up; logging can allocate (strings) |
| Thread-safe communication | `SharedSignal`, bounded queues, atomics; verified by the TSan build |
| Defensive programming, error handling | status enums instead of exceptions on the task path, range checks, counters for every dropped item |
| No unnecessary globals | state lives in objects owned by `main()`; the only mutable globals are the signal-handler flag and the logger's level and output lock |
| CMake, warnings, static analysis, reproducible builds | `-Werror` builds, cppcheck + clang-tidy, CI on a fresh runner |
| MISRA / ISO 26262 | *inspired* practices only (status codes instead of exceptions, no RTTI use, explicit casts, `-Wconversion`, fixed-width types); **no compliance or certification is claimed** |

---

## 6. Commands (WSL2)

```bash
# Everything CI does, locally (~10 min): three builds, all tests, static analysis
./scripts/run_regression.sh
./scripts/run_regression.sh --quick              # GCC build + all tests (~3.5 min)
./scripts/run_regression.sh --quick --label firmware

# Performance report (~1.5 min, needs build/): rewrites docs/performance_report.md
./scripts/run_performance.py

# Individual measurements
./build/bin/vecu_can_latency --count 5000 --background-fps 2000
./build/bin/vecu_sched_bench --duration 10 --stress 16
```

CI runs on GitHub after a push. Locally, the workflow is emulated by section 7.2.

---

## 7. Results (2026-10-26, WSL2)

### 7.1 Regression run
`./scripts/run_regression.sh`, final run (13:40), exit code 0:
```
gcc     PASS   100% tests passed, 0 tests failed out of 273 (172s)
asan    PASS   100% tests passed, 0 tests failed out of 273 (173s)
tsan    PASS   100% tests passed, 0 tests failed out of 273 (173s)
static  PASS   cppcheck and clang-tidy: no findings
```
The three runs before it did **not** pass. Their failures led to the fixes in 7.4.

### 7.2 CI emulation in a clean copy
The workflow's build-test matrix was emulated in a copy containing only the files a commit would
include (`git ls-files -co --exclude-standard`, no build output). Each configuration was configured,
built and tested from scratch, limited to 4 CPUs with `taskset -c 0-3` (the size of a GitHub-hosted
runner):

| Configuration | Compiler warnings | Tests | Test time |
|---|---|---|---|
| GCC 11, RelWithDebInfo, `-Werror` | 0 | 273 / 273 passed | 167 s |
| ASan + UBSan, Debug | 0 | 273 / 273 passed | 171 s |
| Clang 14 TSan, Debug | 0 | 273 / 273 passed | 171 s |

The closed-loop test took 24 s in each configuration, so its host-stall retry was not needed.
The workflow file itself was checked by parsing it as YAML. It has not run on GitHub (section 8).

### 7.3 Performance
See [performance_report.md](performance_report.md) (generated 2026-10-26 13:53). Highlights:
- **CAN latency** (software bus, one-way): median 80 µs, p99 217 µs, max 602 µs; 0 of 5000 lost.
  With 2000 background frames/s it was *lower* (median 59 µs): on an idle bus the receiving
  thread's wake-up from sleep dominates. This was observed in every run; the cause was not
  investigated further.
- **Scheduler:** 0 deadline misses in all runs. With 16 CPU-stress threads, start latencies reach
  9.4 ms and the 10 ms task's execution time reaches 6.9 ms (preemption by the stress threads).
  That is close to the 10 ms period, which is why real-time priorities matter (`--realtime`).
- **ECU tasks while cruising:** Control_10ms needs 6 µs on average (max 106 µs); the largest
  start latency was 3.5 ms.
- **Run-to-run variation is visible.** Brake → disengage was 30 ms here and 40 ms in an earlier
  run; speed-spike detection was 70 ms here and 100 ms earlier. Both depend on where the event
  falls within the 20/50 ms message cycles and task periods. The report is therefore dated and
  tied to one run.

### 7.4 Problems found during this phase
- **Race in the trial-image confirmation (Phase 6 code).** The first full regression failed:
  right after a firmware update a new image was already `Confirmed` instead of `Trial`. Cause:
  the ECU marked the image as trial inside `boot()` and only afterwards told the Diag task that a
  new boot had happened. If the Diag task ran in between, it measured the "healthy period" from
  the original start-up and confirmed the untested image at once. Fix: `FirmwareService` records
  the boot time under the same lock as the trial state and decides about confirmation itself
  (`confirmTrialIfHealthy`). The system test now also asserts that confirmation never comes
  before the healthy period. The failure could not be reproduced on demand; after the fix, 8 of 8
  repetitions under 16 CPU-stress threads passed, plus all later full runs.
- **Test scheduling disturbed real-time tests.** After splitting the system tests into separate
  CTest entries, the closed-loop test ran in parallel with hundreds of unit-test process starts
  and all four simulators stalled for ~12 ms at the same instant. Fix: system tests run serially
  (`RUN_SERIAL`).
- **Whole-host stalls of WSL2.** Even running alone, the closed-loop test later failed in the
  sanitizer stages: all simulators were 10–18 ms late at one instant, at a different time in
  each run, while every product assertion passed. The pace check protects the *validity of the
  measurement*. Following the project's policy (behaviour strict, environment-quality checks get
  one documented retry), the closed-loop run is repeated once when a simulator was late, and the
  retry is reported as a pytest warning. No limit was relaxed.
- **CAN latency was never measured before** (Phase 1 measured only timeout detection). The new
  tool `vecu_can_latency` closes this gap.
- **Static analysis** of the new tool: two cppcheck style findings and two clang-tidy findings
  were fixed. One cppcheck "condition always false" is a false positive (`createCanChannel` does
  return nullptr on failure) and is suppressed inline with a reason, like the two existing
  suppressions.
- The regression script exits non-zero on any failed stage (checked: exit code 1 with a failing
  stage, 0 when all pass).

---

## 8. Limitations and possible improvements
- **The CI workflow has not run on GitHub yet.** It was checked by YAML parsing and by emulating
  its steps locally (7.2), but GitHub-specific behaviour is unverified until the first push: the
  runner kernel, the `vm.mmap_rnd_bits` step, the availability of the `vcan` module, and timing
  on shared runners.
- **SocketCAN code paths have never been executed.** WSL2 has no `vcan`. The CI job `socketcan`
  is the first chance, and it is marked informational because the module may not be loadable there.
- **The Mermaid diagrams were not rendered locally** (no Node.js). They use basic syntax that
  GitHub supports, but the first rendering on GitHub is still to be checked.
- **Timing-based system tests on shared CI runners** can fail when a runner is heavily loaded.
  The limits have margins, and the 4-CPU emulation passed (7.2), but there is no guarantee.
- No code-coverage measurement yet (gcov/lcov would be the free option), no fuzzing of the
  protocol parsers (ISO-TP, UDS, DBC, image header), and no long-duration soak test.
- Performance numbers are from one host and one run. Variance between runs is visible in the
  report's maxima but not characterised statistically.
