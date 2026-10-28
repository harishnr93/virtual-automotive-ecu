# Engineering Decisions

The important design decisions of the project, each with the problem it solves, the
alternatives that were considered, and the price paid. Details are in the phase documents
linked under each entry.

| # | Decision | Area |
|---|---|---|
| 1 | [UDP multicast software CAN bus as a fallback for missing vcan](#1-udp-multicast-software-can-bus-as-a-fallback-for-missing-vcan) | CAN |
| 2 | [One CAN interface, three implementations](#2-one-can-interface-three-implementations) | CAN |
| 3 | [A dedicated CAN reception thread with bounded queues](#3-a-dedicated-can-reception-thread-with-bounded-queues) | CAN / scheduling |
| 4 | [One thread per periodic task, absolute release times, OS port](#4-one-thread-per-periodic-task-absolute-release-times-os-port) | Scheduling |
| 5 | [Timestamped shared signals between tasks](#5-timestamped-shared-signals-between-tasks) | Scheduling |
| 6 | [Control algorithm as pure logic](#6-control-algorithm-as-pure-logic) | Control |
| 7 | [The vehicle model lives in Python, with shared golden values](#7-the-vehicle-model-lives-in-python-with-shared-golden-values) | Simulation |
| 8 | [Own ISO-TP and UDS implementations, twice](#8-own-iso-tp-and-uds-implementations-twice) | Diagnostics |
| 9 | [Fault manager modelled on DEM / FiM concepts](#9-fault-manager-modelled-on-dem--fim-concepts) | Faults |
| 10 | [Software watchdog with three reactions](#10-software-watchdog-with-three-reactions) | Watchdog |
| 11 | [A/B firmware slots with trial boot](#11-ab-firmware-slots-with-trial-boot) | Firmware |
| 12 | [CRC-32 for integrity, a strategy interface for authenticity](#12-crc-32-for-integrity-a-strategy-interface-for-authenticity) | Firmware |
| 13 | [Test strategy: deterministic core, real processes on top](#13-test-strategy-deterministic-core-real-processes-on-top) | Validation |
| 14 | [Timing is measured and reported, never asserted as a guarantee](#14-timing-is-measured-and-reported-never-asserted-as-a-guarantee) | Validation |
| 15 | [Test-only fault injection in the product binary](#15-test-only-fault-injection-in-the-product-binary) | Validation |
| 16 | [Machine-readable log lines as a test interface](#16-machine-readable-log-lines-as-a-test-interface) | Validation |
| 17 | [Traces use a steady wall clock](#17-traces-use-a-steady-wall-clock) | Tooling |
| 18 | [Result figures without plotting libraries](#18-result-figures-without-plotting-libraries) | Tooling |

---

### 1. UDP multicast software CAN bus as a fallback for missing vcan
**Problem.** The stock WSL2 kernel has no `vcan` module, so Linux SocketCAN cannot be used without
building a custom kernel.
**Decision.** A software bus: every node joins the IPv4 multicast group 239.255.0.1 on the loopback
interface (TTL 0, never leaves the host). A 24-byte datagram carries one CAN frame and a per-channel
sender id, so a node does not receive its own frames, just as with SocketCAN. `--bus auto` uses
`vcan0` when it exists and the UDP bus otherwise.
**Alternatives.** A custom WSL2 kernel (setup effort, not reproducible for others); a central broker
process (one more process that can fail, extra latency).
**Price.** No bit timing, arbitration, ACK/error frames or bus-off. Kernel drops are possible under
extreme load; they are counted (`SO_RXQ_OVFL`) and never silent. → [Phase 1](phase1_virtual_can.md)

### 2. One CAN interface, three implementations
**Decision.** `ICanChannel` (open, send, receive with timeout, statistics) is implemented by
SocketCAN, the UDP bus and an in-memory bus. Everything above it is identical.
**Why.** The unit tests run without sockets and timing (in-memory bus), the system tests run on
real kernel sockets, and a real `can0` would work without code changes. It mirrors the AUTOSAR
split between the driver and CanIf.

### 3. A dedicated CAN reception thread with bounded queues
**Decision.** One high-priority thread blocks in `receive()` and pushes frames into bounded queues
(128 frames): one for COM and one for diagnostics. The periodic tasks never block on a socket.
**Why.** A periodic task that blocks in I/O loses its timing. A full queue counts an overrun
instead of blocking the receiver.
**Price.** One more thread; a frame waits up to one task period in the queue. The ISO-TP block size
of the ECU (32) is chosen so that a tester cannot overflow the diagnostic queue ([Phase 6](phase6_firmware_update.md#211-flow-control-how-fast-can-we-download)).

### 4. One thread per periodic task, absolute release times, OS port
**Decision.** Each task (10/20/100 ms) has its own thread that sleeps until an **absolute** release
time (`clock_nanosleep(TIMER_ABSTIME)`), so errors do not accumulate. Behaviour follows OSEK:
- an activation limit of 1: a late task skips releases instead of queueing them
- rate-monotonic priorities
- a priority-inheritance mutex

All OS calls are behind `os_port`.
**Alternatives.** A single-threaded cyclic executive (simple, but one overrun delays every task);
`timerfd` + epoll (more complex, no benefit here).
**Price.** Under Linux/WSL2 this is soft real time: measured start latencies are hundreds of
microseconds, and milliseconds under load ([performance report](performance_report.md)). An RTOS
port would replace `os_port`, not the scheduler. → [Phase 2](phase2_scheduler.md)

### 5. Timestamped shared signals between tasks
**Decision.** Tasks exchange data only through `SharedSignal<T>`: a value, a timestamp and a
sequence number, guarded by a priority-inheritance mutex.
**Why.** The reader can tell how old the data is. The control task treats inputs older than 100 ms
as stale, and the COM task never transmits a control output older than 40 ms. A task that stops
is therefore detected through the data itself, independently of the watchdog.

### 6. Control algorithm as pure logic
**Decision.** `CruiseController` and the PID take an input struct and a time step and return an
output struct. They have no CAN, threads, clock or globals.
**Why.** The same code is tested with synthetic time (exact, fast), in a C++ closed loop against a
plant model, and runs unchanged in the ECU. It also makes the safety-relevant behaviour reviewable:
- anti-windup
- output limits
- never re-engaging automatically after a fault

→ [Phase 3](phase3_cruise_control.md)

### 7. The vehicle model lives in Python, with shared golden values
**Decision.** The longitudinal vehicle model and the scenarios are in Python. A small C++ copy of the
model is used only for controller unit tests; both are checked against the same golden values.
**Why.** Scenarios, analysis and plots are quick to write in Python, and the ECU is tested over the
real bus. Shared golden values keep the two models from drifting apart.

### 8. Own ISO-TP and UDS implementations, twice
**Decision.** ISO-TP and a UDS server (a subset of ISO 14229-1) are implemented in C++ for the ECU,
and independently in Python for the tester (no `python-can` / `udsoncan`). The firmware image
format is likewise implemented twice.
**Why.** There are no extra dependencies, every byte is understood, and two independent
implementations catch each other's mistakes. A shared misunderstanding of the standard would not
be caught; the documentation states the implemented scope precisely for that reason.
→ [Phase 4](phase4_diagnostics.md)

### 9. Fault manager modelled on DEM / FiM concepts
**Decision.**
- Monitors report pass/fail; a fault manager debounces (time or counter based), qualifies faults
  and recovers them after a configurable condition.
- Latched faults are released only by a new operation cycle.
- The manager drives DTC status bits, aging and function inhibition.
- DTCs are persisted in an NVM file with a CRC-32 and an atomic write (temp file, `fsync`,
  `rename`).

**Why.** Separating detection, qualification and reaction is the core idea of AUTOSAR DEM/FiM. The
configuration is one table that can be reviewed. → [Phase 5](phase5_fault_management.md)

### 10. Software watchdog with three reactions
**Decision.** The scheduler reports activation start and end to a supervisor, which checks the
heartbeat, the execution time and application stalls. On expiry, the COM task first transmits safe
values. What happens next is configurable:
- an in-process reset (recover once the task resumes; 200 ms probation; at most 3 resets per
  operation cycle)
- a process exit with code 75 (an external supervisor restarts the ECU)
- nothing (stay in the safe state)

**Price.** It is a software watchdog in the same process: it cannot catch a frozen OS, and a hung
thread cannot be recovered in-process. That is why the exit mode exists; it stands in for a
hardware watchdog reset.

### 11. A/B firmware slots with trial boot
**Decision.**
- Updates go into the slot that is not running.
- Every decision is persisted before it takes effect.
- A new image boots as a trial and is confirmed only after a healthy period; after 3 failed boots
  the bootloader rolls back.
- Every boot re-verifies the image.

**Alternatives.** Overwriting in place from a separate bootloader (half the flash, but no runnable
image while flashing).
**Price.** Twice the flash. → [Phase 6](phase6_firmware_update.md)

### 12. CRC-32 for integrity, a strategy interface for authenticity
**Decision.** CRC-32 protects the image header and payload against accidental corruption. The
verification sits behind `IImageVerifier`, and the header carries the verification method and a
signature size.
**Why it is not enough.** A CRC has no secret, so anyone can make a modified image valid. A real
product needs a signature, SecurityAccess and secure boot. The extension path is documented, and a
test shows that tampering plus a recomputed CRC passes.

### 13. Test strategy: deterministic core, real processes on top
**Decision.** Two layers:
- **Logic tests** use synthetic time and an in-memory bus: exact, fast, no flakiness. They cover
  debounce boundaries, rollback after exactly 3 boots, power loss at 21 points, and the PID.
- **System tests** start real ECU processes, the simulator and the tester on an isolated UDP bus
  per test (random port), and check behaviour over the bus.

Every C++ change also runs under ASan + UBSan and ThreadSanitizer (Clang: GCC 11's TSan reports
false positives on `pthread_cond_clockwait`), plus cppcheck and clang-tidy. `run_regression.sh`
and the CI run the same stages.

### 14. Timing is measured and reported, never asserted as a guarantee
**Decision.** Behaviour assertions are strict. Timing assertions have margins that reflect the
requirement, not the speed of this host. For example, a lost speed message must be detected in
100–250 ms (the timeout is 100 ms), and a brake must disengage the cruise control within 150 ms.
The single timing-*quality* check (the scheduler benchmark under light load) gets **one**
documented retry, because a whole-host stall was observed. Performance values come only from
`scripts/run_performance.py` runs and are published with date, host and commit.
**Why.** WSL2 is not a real-time OS. Claims are limited to what was measured.

### 15. Test-only fault injection in the product binary
**Decision.** `--inject` (task stall, overrun, hang, controller NaN) and the image's "simulated
defect" flag exist in the ECU binary and are inactive unless explicitly requested.
**Why.** Fault reactions can be tested end to end, including real process restarts.
**Price.** A production build should compile them out; this is documented as a limitation.

### 16. Machine-readable log lines as a test interface
**Decision.** Key events are logged as single lines with `key=value` fields, which tests parse:
`SUMMARY`, `SCHED_SUMMARY`, `FAULT`, `BOOT`, `FW_SUMMARY`, `WATCHDOG_EXPIRED … t=…s`.
**Why.** System tests can check internal reactions and their timing without debug interfaces.
**Lesson.** This is a contract. Adding a non-numeric field to `SUMMARY` broke the parsers in
Phase 6, so firmware data got its own line.

### 17. Traces use a steady wall clock
**Problem.** A CAN trace showed 0.85 s without frames although none were lost: WSL2 had stepped the
system clock forward while re-syncing with Windows, so the wall-clock timestamps jumped.
**Decision.** CAN traces and the ECU's epoch anchor use `core::steadyWallMicros()`: the wall clock
read once, advanced by the monotonic clock. Timing *decisions* already used only the monotonic clock.
**Price.** Over a long run the trace can drift from the real wall clock by the corrections it
ignores; intervals inside the trace are always right. → [Results gallery](results_gallery.md#a-bug-the-figures-found)

### 18. Result figures without plotting libraries
**Decision.** A small SVG toolkit (`svg_charts.py`) instead of matplotlib. The installed matplotlib
did not even import (it needs a newer numpy), and the project avoids dependencies.
**Rules:**
- one reference palette, with dark mode via `prefers-color-scheme`
- one y-axis per panel
- labels that never overlap
- every figure drawn from a real run, with the date in the subtitle

**Price.** Static images only, without hover or zoom; the numbers are also in the performance
report. → [Results gallery](results_gallery.md)

