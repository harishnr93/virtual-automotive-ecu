# Phase 5 — Fault Management and Watchdog

This phase makes the ECU **detect, qualify, report, react to and recover from faults**, and
adds a **software watchdog** that supervises the tasks themselves. It continues
[Phase 4](phase4_diagnostics.md).

> **Scope note.** The concepts follow AUTOSAR (DEM, FiM, WdgM, NvM) and ISO 14229 / ISO 26262
> *practice*, in simplified form. This is an educational design: there was no hazard analysis,
> no safety goals, no FMEA, and no certification. The watchdog is a **software** supervisor
> inside the same process, not a hardware watchdog.

---

## 1. Engineering objective

Implement and test fault handling for:

| Plan item | Covered by |
|---|---|
| Sensor signal timeout | `SpeedSignalStale`: the sender's alive counter stops (data not refreshed) |
| Invalid sensor values | `SpeedSignalInvalid`: validity bit, range, physical gradient |
| CAN communication timeout | `SpeedMessageTimeout`, `DriverInputsTimeout` |
| Task deadline misses | `TaskDeadlineMiss` (scheduler counters) |
| Controller faults | `ControllerFault` (latched) |
| DTC storage | DTC memory with aging, persisted in an NVM simulation with CRC32 |
| Fault state transitions / recovery conditions | fault manager state machine, per-fault debounce and recovery |
| Deterministic fault-injection tests | synthetic-time unit tests + time-triggered injections in system tests |
| Watchdog: missing heartbeat, execution timeout, stalled application | `WatchdogSupervisor` |
| Configurable supervision, simulated ECU recovery | per-task limits, `--watchdog-action reset/exit/none` |

---

## 2. Concepts

### 2.1 Fault, error, failure — and why we debounce
A **monitor** (a "test") checks something periodically, for example "was message 0x100 received in
the last 100 ms?". A single bad result can be noise. **Debouncing** turns raw results into a
*qualified* decision:

- **Time-based debounce:** the monitor must fail continuously for `fail_time` before the fault is
  qualified, and pass continuously for `pass_time` (the **recovery condition**) before it heals.
- **Counter-based debounce** (AUTOSAR DEM style): every failed report adds `increment`, every
  passed report subtracts `decrement`. The fault qualifies at `fail_threshold` and heals at
  `pass_threshold`. This tolerates sporadic failures but catches frequent ones.

### 2.2 Fault state machine
```
            failed           debounce done                passed           recovery done
NotTested ─────────► PreFailed ───────────► Failed ───────────► PrePassed ──────────────► Passed
                        │  passed (glitch)    ▲  failed again  │
                        └──► Passed          └────────────────┘
```
`Failed` and `PrePassed` count as **active**. A **latched** fault stays `Failed` until the next
**operation cycle**: an ECU start, a hard reset, or a watchdog reset.

### 2.3 Two reaction levels (signal vs. fault)
| Level | When | What | Example |
|---|---|---|---|
| **Signal-level** (immediate, no debounce) | every received frame | do not use a bad value; keep the last plausible one | a speed sample that jumps 30 km/h in 20 ms is discarded |
| **Fault-level** (qualified) | after debounce | DTC, function inhibition, recovery tracking | 40 ms of implausible speed → `SpeedSignalInvalid` → cruise inhibited → recovers after 500 ms of plausible data |

This separation mirrors AUTOSAR: COM/E2E invalidate signals, DEM qualifies events, and FiM
inhibits functions.

### 2.4 Function inhibition
Faults configured with `inhibits_function` block the cruise function while they are active
(`CruiseInputs.function_inhibited` → state `Fault`, reason `FunctionInhibited`). After a fault
heals, the cruise controller still waits its own 500 ms, and then returns to **Standby**, never
directly to Active. The driver must engage again.

### 2.5 DTC life cycle across operation cycles
- A qualified failure sets `testFailed`, `pending` and `confirmed` (confirmation threshold 1).
- A complete operation cycle without failure clears `pending`.
- **Aging:** a confirmed DTC that does not fail for **3 complete operation cycles** loses
  `confirmed`. `testFailedSinceLastClear` keeps the history.
- The DTC memory, counters and the last reset reason are stored in **NVM**, so they survive
  restarts (Section 3.3).

### 2.6 Software watchdog (alive and deadline supervision)
Every task activation reports *start* and *end* to the supervisor, through lock-free atomics.
A separate thread checks every 20 ms:

| Check | Violation if | Limit used here |
|---|---|---|
| Alive (heartbeat) | no completed activation for `heartbeat_timeout` | 3 × task period |
| Execution time | one activation running longer than `max_execution` | 2 × task period |
| Stalled application | *all* supervised tasks violate at the same time | — (expires immediately) |

A violation makes the entity **Failed**. More than `failed_tolerance = 2` consecutive failed
cycles make it **Expired**, which stays latched until recovery. The main thread also checks that
the watchdog thread itself keeps running ("who watches the watchdog"; on a real ECU this is the
hardware watchdog).

### 2.7 Safe state and recovery after a watchdog expiration
```
 Expired ──► SAFE STATE: Com_20ms transmits Fault / 0 % itself (Control may be the stalled task),
   │                      the cruise function is inhibited, DTC U3000-47
   ├── action "exit":  write NVM (reason = Watchdog), terminate with exit code 75
   │                    -> an external supervisor restarts the process (a real reset)
   ▼
 stalled task resumes ──► action "reset": simulated ECU reset (application re-initialised,
   │                       new operation cycle); max. 3 per cycle, then permanent safe state
   ▼
 200 ms healthy probation ──► leave safe state (driver must re-engage)
```
An in-process reset cannot interrupt a thread that is stuck. The ECU therefore stays in the safe
state until the task responds again. With a **permanent hang**, it stays in the safe state; the
`exit` action handles that case with a real process restart.

### 2.8 Stale control output (second, independent mechanism)
`Control_10ms` writes its output every 10 ms with a timestamp. If `Com_20ms` finds the output older
than **40 ms**, it transmits `Fault / 0 %` (reason 8 `ControlOutputStale`) by itself. This is
faster than the watchdog (41 ms vs. 60 ms measured, Section 8) and works even if the watchdog
thread failed.

---

## 3. Architecture

```
 Monitors (in their tasks)                         Fault manager (thread-safe)          Reactions
 ──────────────────────────                        ───────────────────────────          ─────────
 Com_20ms:  0x100 / 0x110 timeout ───┐             debounce (time / counter)            DTC status (DtcStore)
            speed validity/range/    ├─ report() ─► state machine, recovery,     ───►   function inhibition
            gradient, alive counter  │             latched faults, event memory         (CruiseInputs)
 Control_10ms: controller fault ─────┤                      │                            fault events -> log
 Diag_100ms: deadline misses ────────┤                      ▼                            DIDs 0x0300...
 Watchdog thread: expiration ────────┘             NVM (Diag_100ms, shutdown,
                                                   watchdog exit) — CRC32, atomic
 Scheduler ──observer──► WatchdogSupervisor ◄── check() every 20 ms (Watchdog thread, prio 95)
```

| File | Content |
|---|---|
| `include/faults/fault_manager.hpp` | debouncing, state machine, latching, inhibition, event memory (32 events), clear hook |
| `include/faults/dtc_store.hpp` | ISO 14229 status bits + aging + import for NVM |
| `include/faults/nvm_storage.hpp` | binary NVM block: magic, version, records, CRC32; atomic write |
| `include/core/crc32.hpp` | CRC-32 (IEEE), compile-time table; reused by Phase 6 |
| `include/watchdog/watchdog_supervisor.hpp` | alive/execution supervision, tolerance, stall detection |
| `include/core/periodic_scheduler.hpp` | new `ITaskObserver` hook (activation start/end) |
| `apps/common/ecu_runtime.cpp` | watchdog thread, recovery sequence, emergency exit, supervisor liveness |
| `apps/common/fault_injector.hpp` | **test-only** injections: stall, overrun, hang, controller NaN |
| `apps/cruise_ecu/cruise_faults.hpp` | the fault configuration table (Section 3.1) |
| `apps/cruise_ecu/cruise_ecu.cpp` | monitors, signal invalidation, NVM handling, watchdog callbacks |
| `simulator/vecu_sim/*` | new simulator faults `speed_spike`, `freeze_alive`; tester flush and drop count |

**Structure change (justified):** `apps/cruise_ecu/main.cpp` had grown to ~500 lines. It is now
split into `cruise_faults.hpp` (configuration), `cruise_ecu.hpp/.cpp` (application) and a
20-line `main.cpp`.

### 3.1 Fault configuration

| Fault | DTC | Monitor | Qualify | Recover | Latched | Inhibits |
|---|---|---|---|---|---|---|
| SpeedMessageTimeout | U0121-87 `C12187` | 0x100 timeout (100 ms) | immediate | 500 ms received | no | yes |
| DriverInputsTimeout | U0140-87 `C14087` | 0x110 timeout (250 ms) | immediate | 500 ms received | no | yes |
| SpeedSignalInvalid | C0031-64 `403164` | validity bit, range, ≤ 100 km/h/s gradient | 40 ms | 500 ms plausible | no | yes |
| SpeedSignalStale | U0121-82 `C12182` | alive counter repeated | 3 frames (counter +34/−20) | 5 frames | no | yes |
| TaskDeadlineMiss | U3000-48 `F00048` | new misses per 100 ms window | 2 windows (+50/−10) | 10 windows (1 s) | no | yes |
| ControllerFault | U3000-49 `F00049` | controller output | immediate | next operation cycle | **yes** | yes |
| WatchdogExpired | U3000-47 `F00047` | watchdog thread | immediate | next operation cycle | **yes** | yes |
| NvmCorrupted | U3000-44 `F00044` | NVM load at start | immediate | next operation cycle | **yes** | no |

### 3.2 New diagnostic data (UDS 0x22)
| DID | Content |
|---|---|
| 0x0300 | active faults, bit *n* = fault id *n* (table order above) |
| 0x0301 | watchdog status (0 Ok, 1 Failed, 2 Expired) + number of expirations (u16) |
| 0x0302 | last reset reason (0 PowerOn, 1 UDS hard, 2 UDS soft, 3 watchdog) |
| 0x0303 | operation cycle counter |
| 0x0304 | NVM load status at start (0 Ok, 1 not found, 2 corrupted, 3 I/O error) |
| 0x0202 | reset counter (now persistent across restarts) |

`CruiseFaultReason` on CAN 0x200 gained 6 `FunctionInhibited`, 7 `WatchdogSafeState` and
8 `ControlOutputStale` (value table in the DBC). The software version DID is now `00.05.00`.

### 3.3 NVM simulation
A binary file (`--nvm FILE`): magic `VNVM`, version, operation cycles, reset counter, last reset
reason, all DTC records (code, status, occurrence, aging), CRC-32. It is written by `Diag_100ms`
when the DTC memory changed (at most once per second), at shutdown, and before a watchdog exit.
It is always written to `FILE.tmp` + `fsync` + `rename`, so a crash never leaves a torn block. A
corrupt block (bad magic/version/size/CRC) is detected at start: the ECU starts with an empty fault
memory, sets DTC U3000-44 and rewrites a valid block.

### 3.4 Fault injection (test only)
| Option | Effect |
|---|---|
| `--inject stall:TASK:AT:DUR` | the task blocks once for DUR s at AT s |
| `--inject overrun:TASK:AT:DUR:MS` | every activation in [AT, AT+DUR) runs MS ms longer |
| `--inject hang:TASK:AT` | the task blocks until shutdown |
| `--inject controller-nan:AT` | one control step receives a NaN time step |
| simulator `speed_spike` / `freeze_alive` | +30 km/h on the transmitted speed / alive counter stops |

Injections are inactive unless given, and logged as `FAULT_INJECTION` (with "TEST MODE" at start).
A production build would compile them out.

---

## 4. Design decisions

| Decision | Reason |
|---|---|
| Separate signal-level reaction and fault qualification | Bad values are never used (no debounce delay on safety), while DTCs and inhibition are not triggered by single glitches. |
| Debounce/recovery as data (`cruise_faults.hpp`), `constexpr` | Calibratable like an ECU dataset; reviewable in one table; no code change to tune. |
| Fault manager forwards only *qualified* states to the DTC memory, on every report | An active fault re-fails its DTC immediately after a tester clear (ISO 14229 intent). |
| Recovery only to Standby, latched faults until a new operation cycle | No automatic re-engagement after a fault; a latched internal fault needs a reset. |
| Watchdog in its own highest-priority thread, lock-free heartbeats | A supervisor must not depend on what it supervises; tasks are never blocked by it. |
| Safe-state transmission by Com_20ms | If Control stalls, its last output is frozen and must not keep driving the throttle. |
| Stale-output check in COM in addition to the watchdog | Diverse, independent mechanisms; the measured faster path. |
| `exit` action with exit code 75 | A real reset needs a process restart (a stuck thread cannot be killed safely); the NVM records why. |
| NVM: CRC32 + atomic rename + version field | Detects corruption; never a torn write; format can evolve. |
| Kernel drop counting (`SO_RXQ_OVFL`) on the socket backends | Lost frames become visible as `rx_overruns` instead of disappearing silently (Section 8.4). |

---

## 5. Commands

```bash
cd ~/harish_ws/vir_auto_ecu
cmake --build build -j"$(nproc)" && ctest --test-dir build      # 226 tests (~2 min)

# Full fault sequence (as in the system test): run the ECU with NVM and injections ...
./build/bin/vecu_cruise_ecu --duration 35 --nvm logs/nvm.bin \
    --inject overrun:Com_20ms:14.5:1.0:25 --inject stall:Control_10ms:21.0:0.4 \
    --inject controller-nan:27.0 &
# ... plus the vehicle simulator with signal faults
PYTHONPATH=simulator python3 -m vecu_sim.simulator --scenario config/scenarios/fault_management.json \
    --csv logs/faults.csv --svg logs/faults.svg
# ... and look at the fault memory with the tester (around t = 28 s also "reset hard")
./scripts/uds_tester.py dtc-supported "read-did 0300 0301 0302 0303 0304"

# Watchdog with a real process restart (exit code 75), then inspect the reset reason
./build/bin/vecu_cruise_ecu --duration 10 --nvm logs/nvm.bin --inject hang:Control_10ms:1.0 \
    --watchdog-action exit; echo "exit code $?"
./build/bin/vecu_cruise_ecu --duration 3 --nvm logs/nvm.bin &   # restart: NVM_LOADED ... last_reset=Watchdog
sleep 1; ./scripts/uds_tester.py "read-did 0302 0202" "dtc 08"

# Only the deterministic fault-handling unit tests
./build/bin/vecu_unit_tests --gtest_filter='FaultManager*:Watchdog*:Nvm*:DtcAging*:Crc32*'
```

---

## 6. Expected behaviour (real ECU log, abbreviated)

```
FAULT_INJECTION stall task=Control_10ms t=21.000s duration_ms=400
CONTROL_OUTPUT_STALE age_ms=51.3 t=21.041s                       # COM blocks the frozen output
WATCHDOG_EXPIRED entity=Control_10ms violation=ExecutionTimeout detail_ms=60.4 t=21.061s
SAFE_STATE entered: watchdog expired (Control_10ms, ExecutionTimeout)
FAULT WatchdogExpired NotTested -> Failed t=21.061s
WATCHDOG_RESUMED entity=Control_10ms after_ms=359.8
ECU_RESET type=watchdog (simulated: application state re-initialised, process keeps running)
WATCHDOG_HEALTHY supervision ok after recovery
SAFE_STATE left: supervision healthy after recovery (driver must re-engage)
CRUISE_STATE Fault -> Standby
```

On the bus (simulator recording), the cruise status went directly from Active / 6.64 % to
Fault / 0 % with reason 8 and then 7, and no cruise throttle was applied while it showed reasons 7/8.

---

## 7. Tests

| Suite | Type | What it proves |
|---|---|---|
| `FaultManagerTest.*` (11) | unit, synthetic time | time debounce (exact boundary), glitch rejection, recovery condition incl. relapse, counter debounce, latching across cycles, non-inhibiting faults, re-fail after clear, event memory order and overflow |
| `WatchdogTest.*` (7) | unit, synthetic time | healthy operation, missing heartbeat → Failed → Expired after tolerance (exact time), execution timeout, transient recovery, stalled application, latched Expired until reset, configuration limits |
| `Crc32.*`, `DtcAging.*`, `DtcStoreImport.*`, `NvmTest.*` (10) | unit | CRC check value 0xCBF43926; aging after 3 clean cycles and reset by a new failure; NVM round trip, **every single bit flip detected**, truncation, foreign file, I/O error, atomic temp-file handling |
| `CruiseControllerTest.FunctionInhibition*` (2), `UdsServerTest.ClearHandler*` (1) | unit | inhibition → Fault until released + 500 ms; specific reason wins; clear hook |
| `UdpCanChannel.ConcurrentSendersAreSafe` (made realistic), `.KernelQueueDropsAreCountedAsOverruns` (new) | integration | concurrent transmitters at 2× full-bus rate; 20 000-frame flood: **received + counted drops = sent** |
| `test_fault_management.py` (9) | system (ECU + simulator + tester) | all fault classes injected, qualified, reported as DTCs, recovered; safe state on the bus; latched fault cleared only by a UDS hard reset; NVM persistence and **aging across real restarts**; NVM corruption detected and repaired; watchdog **process restart** with persisted reason; permanent hang keeps the safe state and shuts down without deadlock |
| Python unit (`speed_spike`, `freeze_alive`, tester flush, drop counting) | unit | simulator fault types and tester robustness |

---

## 8. Measured results (2026-10-25, WSL2, real processes)

### 8.1 Fault detection and recovery
| Fault | Measured |
|---|---|
| Speed spike +30 km/h | cruise `Fault` on the bus **80–100 ms** after the spike started (40 ms debounce + frame/task cycles); spike samples discarded |
| Frozen alive counter | cruise `Fault` on the bus after **100 ms** (3 stale frames) |
| Com_20ms overrun (+25 ms per activation) | `TaskDeadlineMiss` qualified **202 ms** after the overrun began; healed **1.10 s** after it ended (configured: 10 clean windows = 1 s) |
| Control_10ms stall (400 ms) | stale output blocked on the bus after **41 ms**; watchdog expired after **60 ms**; task resumed after 360 ms → watchdog reset → healthy after 200 ms probation |
| Control_10ms permanent hang + `exit` | watchdog expired after **61 ms**; process terminated with code 75 after **1.10 s** total run time (hang at 1.0 s); after restart: reason `Watchdog`, reset count 1, U3000-47 confirmed |
| Controller NaN | `ControllerFault` latched within one 10 ms step; cleared only by UDS `11 01` |
| NVM corruption | detected at start (status 2), U3000-44 confirmed, valid block rewritten (CRC verified with Python `zlib.crc32`) |

### 8.2 DTC memory across restarts
After the full fault run, a restart showed all five fault DTCs still confirmed, reset count 2,
reason `PowerOn` (clean shutdown) and operation cycle 4. One more restart (cycle 5): the speed-spike
DTC aged out (status `0x60`: confirmed cleared, `testFailedSinceLastClear` kept), while the
controller DTC, which failed one cycle later, remained confirmed (`0x28`).

### 8.3 Verification
GCC 11 `-Werror` fresh build: 0 warnings. **226 CTest entries** pass in the fresh GCC build, the GCC, ASan + UBSan
and Clang ThreadSanitizer builds (2 vcan cases skipped inside the C++ suites).

### 8.4 Problems found during verification
- **UDP bus drops were invisible.** Under TSan, a concurrent-send test lost frames: the receiver
  could not keep up with an unrealistically fast burst, and the kernel dropped datagrams without any
  counter showing it. Fix: `SO_RXQ_OVFL` drop counting in both socket backends (as `rx_overruns`,
  also in `CAN_RX_SUMMARY channel_drops=`), a larger receive buffer, a test paced at a realistic bus
  load, and a flood test proving exact accounting. Note: the kernel reports drops *with the next
  received frame*.
- **Tester lost responses after idle time (latent since Phase 4).** While the Python tester waited
  (e.g. the 5.6 s S3 test), vehicle traffic filled its receive queue (1454 drops measured), and the
  next response could be dropped. Fix: the UDS client discards stale traffic before each request
  (as real testers do); the drop count is visible.
- **Host stalls.** One full-suite run showed a ~45 ms whole-host stall: all three benchmark tasks
  late at the same moment, with releases correctly accounted as skipped. Behaviour assertions stay
  strict; the light-load *timing-quality* check of the benchmark now gets one documented retry
  (8 of 8 immediate repeats were clean, max latency < 1 ms).
- Test-code bugs fixed: watchdog unit tests that fed heartbeats "from the future"; a pytest
  helper whose name started with `test`; a tautological NVM size assertion.

---

## 9. Limitations and possible improvements
- **Software watchdog only:** it shares the process and the host with the tasks. A real ECU needs an
  independent hardware watchdog (and window watchdog), which WdgM would trigger.
- **In-process reset** re-initialises application state but not memory, threads or the OS; the
  `exit` action plus an external supervisor is the closer analogue of a real reset.
- No freeze-frame (snapshot) data, extended data records, DTC priority or displacement, warning
  indicator (MIL), or fault-tolerant-time analysis per hazard.
- Gradient plausibility uses one fixed physical limit; a sustained offset longer than ~0.3 s would
  eventually pass the gradient check (it becomes "plausible"), so it would need a second sensor for
  cross-checking.
- NVM is a file; there is no flash-wear model, no redundant copies and no write-time budget (the
  write is rate-limited to once per second and runs in the lowest-priority task).
- Fault injection hooks exist in the production binary (inactive without options); a release
  build should compile them out.
