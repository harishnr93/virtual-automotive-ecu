# Phase 2 — ECU Scheduler

This document explains the periodic task scheduler: the concepts behind it, how it is
built, how it was tested, and what was measured. It continues
[Phase 1](phase1_virtual_can.md).

---

## 1. Engineering objective

Replace the Phase 1 single-threaded loop with an ECU-style **periodic task scheduler**:

- 10 ms control, 20 ms communication and 100 ms diagnostic tasks;
- fixed task priorities where the OS permits them;
- measurement of execution time, start latency, scheduling jitter and response time;
- deadline-miss and execution-budget detection;
- safe data exchange between tasks;
- an abstraction that allows an RTOS backend later.

**Non-goal:** hard real-time. Linux on WSL2 cannot guarantee deadlines. The scheduler
*measures* timing and *detects* violations; it does not promise to avoid them.

---

## 2. Concepts (in simple terms)

### 2.1 Periodic tasks
ECU software is mostly a set of functions that must run again and again at a fixed rate:
read sensors and compute the controller every 10 ms, handle communication every 20 ms,
check diagnostics every 100 ms. Each of these is a **task** with a **period** T.

Task k is **released** (becomes ready) at `r_k = epoch + offset + k·T`. Computing the next
release from the *grid* rather than from "now" means errors never accumulate: after
1000 activations the 10 ms task is still exactly on its grid. This needs an **absolute**
sleep (`clock_nanosleep(..., TIMER_ABSTIME)`), not "sleep 10 ms".

### 2.2 Timing quantities measured for every activation

```
 release r ──── start s ═════ execution ═════ end e
 |<- start latency ->|<--- execution time --->|
 |<------------- response time -------------->|      deadline miss if response > D
```

| Quantity | Meaning | Why it matters |
|---|---|---|
| **Start latency** (release jitter) | How late the OS started the task after its release | Delays every output of the task |
| **Execution time** | How long the task body ran (includes preemption) | Basis for CPU load and the budget check |
| **Response time** | Release → end | Compared with the deadline D |
| **Period jitter** | Change of the start interval: \|(s_k − s_{k−1}) − T\| | Irregular sampling degrades control loops |
| **Deadline miss** | Response time > D (default D = T) | The task's result came too late |
| **Budget overrun** | Execution time > configured budget | Early warning of a runaway or overloaded task |
| **Skipped activation** | A release that was dropped because the task was still running | Lost work, see 2.4 |

### 2.3 Priorities and rate-monotonic scheduling
With fixed-priority preemptive scheduling (as in OSEK/AUTOSAR OS or FreeRTOS), the highest-
priority ready task always runs. **Rate-monotonic** priority assignment (shorter period →
higher priority) is optimal for this model, so the platform uses:

| Thread | Period | Priority |
|---|---|---|
| CAN RX (event-driven, "interrupt") | — | 90 |
| 10 ms task | 10 ms | 80 |
| 20 ms task | 20 ms | 70 |
| 100 ms task | 100 ms | 50 |

`checkRateMonotonic()` warns at start-up if a configuration violates this rule.

On Linux, such priorities only take effect with the `SCHED_FIFO` real-time policy, which
needs a privilege (see Section 3). Without it, threads fall back to `SCHED_OTHER` (the
normal fair scheduler) and the report says so: **`policy=SCHED_OTHER`**.

### 2.4 Overruns: what happens when a task is too slow
If a task runs longer than its period, its next release passes while it is still running.
The scheduler applies the OSEK/AUTOSAR rule of an **activation limit of 1**:

- one pending activation is kept and starts immediately (late);
- further releases during the overrun are **dropped and counted** (`skipped`);
- the period grid (phase) is preserved — no burst of catch-up activations.

Example (T = 10 ms, task runs 35 ms from t=0): releases at 10, 20, 30 ms happened; the one at
30 ms runs immediately, 10 and 20 ms are counted as skipped.

### 2.5 Sharing data between tasks safely
Tasks run in different threads. If the 20 ms task writes a struct while the 10 ms task reads
it, the reader may see half old/half new values ("torn data").
`SharedSignal<T>` solves this (like an AUTOSAR RTE sender-receiver port):

- the writer replaces the value under a mutex; readers get a complete copy;
- each sample carries a **timestamp** and **sequence number** → the reader can detect stale
  data (the cruise ECU enters `Fault` if its inputs are older than 100 ms);
- only trivially copyable types are allowed, so copying cannot allocate or throw.

### 2.6 Priority inversion and priority inheritance
If a low-priority task holds a mutex that a high-priority task needs, and a medium-priority
task preempts the low one, the high-priority task waits for the medium one — **priority
inversion** (this caused resets on the Mars Pathfinder lander in 1997). The platform's
`os::PiMutex` uses the **priority-inheritance** protocol: the lock holder temporarily inherits
the waiter's priority. All task-shared data (SharedSignal, RX queue, statistics) use it.

### 2.7 Event-driven reception
On a microcontroller, a CAN **receive interrupt** copies each frame into a queue, and a periodic
COM task processes the queue later. Phase 2 reproduces this split:

```
bus ─► CanReceiver thread (prio 90) ─► CanRxQueue (128 frames, bounded) ─► Com task
       timestamps each frame            overruns counted                     decode + supervise
```

---

## 3. Environment findings (measured on this machine)

| Check | Result | Consequence |
|---|---|---|
| `ulimit -r` (RLIMIT_RTPRIO) | `0` | User may not use real-time priorities |
| `sched_setscheduler(SCHED_FIFO)` | `EPERM` | `--realtime` falls back to `SCHED_OTHER` (reported) |
| `/proc/sys/kernel/sched_rt_runtime_us` | 950000 | RT throttling active (relevant once RT is enabled) |
| Wake-up latency of `clock_nanosleep` (10 ms period, 300 samples) | mean ≈ 440–490 µs, p99 ≈ 0.9–1.0 ms | Comes from WSL2's virtualised timers |
| Same with timer slack reduced to 1 ns | mean ≈ 440–460 µs | No meaningful gain → **not** adopted |
| GCC 11 ThreadSanitizer | aborts on kernel 6.6 (ASLR) and lacks a `pthread_cond_clockwait` interceptor | Run TSan with `setarch -R` (automatic) and **Clang 14** |

---

## 4. Architecture

```
 apps/  cruise_ecu: Control_10ms · Com_20ms · Diag_100ms     sensor_ecu: Com_10ms · Diag_100ms
          │ SharedSignal<Inputs> / SharedSignal<ControlOutput>
 apps/common/EcuPlatform  (channel, trace, CanReceiver + CanRxQueue, PeriodicScheduler, report)
          │
 include/core/periodic_scheduler.hpp   tasks, epoch barrier, activation policy, statistics
 include/core/timing_stats.hpp         pure timing formulas (unit-tested with synthetic time)
 include/core/timing_report.hpp        table, SCHED_SUMMARY lines, JSON
 include/core/shared_signal.hpp        inter-task data exchange
          │
 include/core/os_port.hpp  ◄── OS abstraction: Thread, sleepUntil, PiMutex, lockProcessMemory
 src/core/os_port_posix.cpp    (POSIX/Linux implementation; an RTOS port replaces only this)
```

### 4.1 RTOS portability
Only `os_port.hpp` + `os_port_posix.cpp` touch pthreads. A FreeRTOS port would map:

| Platform API | POSIX (this project) | FreeRTOS equivalent |
|---|---|---|
| `os::Thread::start` | `pthread_create` + `SCHED_FIFO` attributes | `xTaskCreate(..., priority, ...)` |
| `os::sleepUntil` | `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` | `vTaskDelayUntil` |
| `os::PiMutex` | `pthread_mutex` + `PTHREAD_PRIO_INHERIT` | `xSemaphoreCreateMutex` (has priority inheritance) |
| `os::lockProcessMemory` | `mlockall` | not needed (no virtual memory) |
| start barrier | `std::condition_variable` | event group / task notification |

### 4.2 Scheduler life cycle
1. **Init phase:** `addTask()` for every task. All memory is allocated here; configuration is
   validated (period > 0, 0 ≤ offset < period, deadline ≤ period, priority 1…99, unique name,
   max. 8 tasks).
2. `start()`: creates one thread per task; each waits on a **condition-variable barrier**. The
   scheduler then sets a common **epoch** (now + 20 ms) and releases all of them, so all task
   phases refer to the same time zero.
3. **Run:** each thread loops `sleepUntil(release) → run → measure → next release`.
   Statistics are updated under a `PiMutex` (a few hundred nanoseconds). Counters used by
   the diagnostic task are lock-free atomics.
4. `stop()`: sets a flag; every task finishes its current activation and exits after its next
   wake-up (≤ one period), then all threads are joined.

Reading full statistics copies the monitor under the lock and sorts samples *outside* it, so
reporting never blocks a task for long.

### 4.3 ECU task layout

**Cruise ECU**

| Task | Period / offset | Prio | Budget | Work |
|---|---|---|---|---|
| CanRx thread | event | 90 | — | receive, timestamp, enqueue |
| Control_10ms | 10 ms / 0 | 80 | 2 ms | read inputs, stale-data check, cruise state (PID in Phase 3) |
| Com_20ms | 20 ms / 1 ms | 70 | 5 ms | drain RX queue, decode, alive/DLC/range checks, timeout monitoring, publish inputs, transmit 0x200 |
| Diag_100ms | 100 ms / 3 ms | 50 | 20 ms | report new deadline misses, status every 1 s |

**Sensor ECU:** Com_10ms (prio 80; sends 0x100 every 2nd and 0x110 every 5th activation,
like AUTOSAR COM cycle counters) and Diag_100ms (prio 50).

Offsets spread the tasks inside the period so they do not all wake up at the same instant.

---

## 5. Design decisions

| Decision | Reason |
|---|---|
| One thread per task (not one loop) | Preemption by priority, like an RTOS; a slow low-priority task cannot delay a fast one when RT priorities are granted. |
| Absolute release grid | No drift; releases are exactly reproducible (verified by a test). |
| Pure `TaskTimingMonitor` + `computeNextRelease` | All formulas tested deterministically with synthetic timestamps. |
| Activation limit 1 with skip counting | Matches OSEK/AUTOSAR semantics; no catch-up bursts after an overrun. |
| Request RT, fall back, **report** | Works for every user; never silently pretends to be real-time. |
| Priority-inheritance mutexes | Bounds priority inversion once RT priorities are used. |
| Bounded queues/windows, allocation only at init | Predictable memory; no heap use in the periodic path of the scheduler. |
| Statistics: Welford mean/stddev + last 8192 samples for percentiles | O(1) per activation; percentiles only computed when reporting. |
| Timer-slack tuning not adopted | Measured: no meaningful effect on WSL2 (Section 3). |
| Type-safe report formatting (`ostringstream`) | Removed printf-style varargs flagged by clang-tidy. |

---

## 6. Commands

```bash
cd ~/harish_ws/vir_auto_ecu
cmake -S . -B build -DVECU_WARNINGS_AS_ERRORS=ON && cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

# Scheduler benchmark (prints a timing table + SCHED_SUMMARY lines)
./build/bin/vecu_sched_bench --duration 10
./build/bin/vecu_sched_bench --duration 10 --stress 32           # CPU contention
./build/bin/vecu_sched_bench --duration 10 --load-10ms 12000     # overload the 10 ms task
./build/bin/vecu_sched_bench --duration 10 --json logs/timing.json
./build/bin/vecu_sched_bench --duration 10 --realtime --lock-memory   # RT if permitted

# The two ECUs now run on the scheduler; timing report printed at exit
./scripts/run_demo.sh 5
./build/bin/vecu_cruise_ecu --duration 5 --timing-report logs/cruise_timing.json

# Thread-safety check (data races) — use Clang; setarch -R is applied automatically
CXX=clang++ cmake -S . -B build-tsan-clang -DCMAKE_BUILD_TYPE=Debug -DVECU_ENABLE_TSAN=ON
cmake --build build-tsan-clang -j"$(nproc)" && ctest --test-dir build-tsan-clang

# Memory errors / undefined behaviour
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DVECU_ENABLE_SANITIZERS=ON
cmake --build build-asan -j"$(nproc)" && ctest --test-dir build-asan
```

### Enabling real-time priorities (optional, not verified on this machine)
Requires `sudo`, which was not available during development, so this path is **untested**
here; the code handles both outcomes and the tests accept either.

```bash
# allow your user SCHED_FIFO priorities up to 99 (then log out of WSL: wsl --shutdown)
echo "$USER - rtprio 99" | sudo tee /etc/security/limits.d/99-vecu-rtprio.conf
echo "$USER - memlock unlimited" | sudo tee -a /etc/security/limits.d/99-vecu-rtprio.conf
# alternative for one binary:
sudo setcap cap_sys_nice,cap_ipc_lock+ep build/bin/vecu_sched_bench
```
Even with SCHED_FIFO, WSL2 runs inside a Hyper-V VM on Windows; latency is still bounded by
the host, so results remain soft real-time.

---

## 7. Expected behaviour

Cruise ECU start-up line:
```
[cruise_ecu] started: tasks=3 policy=SCHED_OTHER rx_thread=SCHED_OTHER memory_locked=0 (Ctrl+C to stop)
```
With `--realtime` but no privilege, the line adds
`(real-time requested but not permitted -> fallback)`.

At exit, a table and one machine-readable line per task:
```
task           period prio policy        acts dlmiss  skip budget | exec_avg  exec_p99  exec_max |  lat_avg   lat_p99   lat_max |  jit_max
Control_10ms     10.0   80 SCHED_OTHER    499      0     0      0 |      2.0       3.4     177.2 |    321.5     665.2     842.2 |    658.3
SCHED_SUMMARY task=Control_10ms period_ms=10.000 prio=80 policy=SCHED_OTHER activations=499 deadline_misses=0 ...
```
A deadline miss during operation is reported by the diagnostic task:
`DEADLINE_MISS task=Control_10ms new=1 total=1` (this information feeds the fault manager in Phase 5).

---

## 8. Tests and measured results

### 8.1 New tests (41 added; total 113)

| Suite | Type | What it proves |
|---|---|---|
| `DurationStats.*`, `SampleWindow.*` | unit | mean/stddev (Welford), nearest-rank percentiles, bounded window |
| `TaskTimingMonitor.*` | unit | latency/execution/response/jitter formulas, deadline miss on response time, budget check — synthetic time |
| `ComputeNextRelease.*` | unit | activation policy: no overrun, exact boundary, 1 pending activation, dropped activations with phase kept |
| `OsPort.*`, `OsThread.*`, `PiMutex.*` | unit | steady_clock = CLOCK_MONOTONIC, absolute sleep, thread naming, RT request/fallback, PI protocol, mutual exclusion |
| `SharedSignal.*` | unit | sequence/timestamps; 200 000 concurrent writes with no torn read |
| `SchedulerConfig.*`, `TimingReport.*` | unit | config validation, init-phase rules, rate-monotonic check, report formats |
| `CanRxQueue.*`, `CanReceiver.*` | unit | bounded FIFO with overrun count; RX thread timestamps and preserves order; prompt stop |
| `PeriodicScheduler.*` | integration | activation counts; **exact** release grid with offsets from a common epoch; deadline-miss + skip detection under overload; budget overruns; prompt stop/restart; stats while running; consistent producer/consumer data exchange |
| `test_scheduler_bench.py` | integration | rates over 3 s, JSON report, overload detection, RT fallback |
| `test_two_ecu_communication.py` (extended) | integration | ECUs run their task sets at the right rates without RX queue overruns |

### 8.2 Verification runs (2026-10-21, WSL2 kernel 6.6.87.2, 16 logical CPUs)

| Build | Result |
|---|---|
| GCC 11, `-Werror` | 113 tests: 111 passed, 2 skipped (need vcan0), 0 failed |
| GCC 11 ASan + UBSan | same, no findings |
| Clang 14 ThreadSanitizer | same, **no data races** |
| cppcheck + clang-tidy | no findings |

Bugs found and fixed by this verification:
- **Race in `os::Thread::start`** (found by the ASan build's different timing): the thread was
  named by its creator after `pthread_create`, racing with the thread body. Fixed: the thread
  names itself before running its entry function.
- **Unused member** `TaskTimingMonitor::period_` (found by Clang `-Wunused-private-field`). Removed.
- **Test parser bug**: the integration test's `SUMMARY` regex also matched the new
  `SCHED_SUMMARY` lines. Fixed by anchoring the pattern.

### 8.3 Scheduler benchmark (10 s each, `SCHED_OTHER`, real measurements)

Task loads (busy-wait): 10 ms → 100 µs, 20 ms → 200 µs, 100 ms → 500 µs unless stated.

| Scenario | Task | Activations | Deadline misses | Skipped | Start latency mean / p99 / max [µs] | Period jitter max [µs] |
|---|---|---|---|---|---|---|
| A: light load | 10 ms | 999 | 0 | 0 | 466.6 / 883.2 / 1088.4 | 711.3 |
| | 20 ms | 500 | 0 | 0 | 533.5 / 1154.1 / 1681.5 | 1000.3 |
| | 100 ms | 100 | 0 | 0 | 266.0 / 697.0 / 865.3 | 675.0 |
| B: + 32 spinning threads (16 CPUs) | 10 ms | 999 | **2** | 0 | 448.2 / 4067.8 / 11162.0 | 10987.9 |
| | 20 ms | 500 | 0 | 0 | 675.3 / 6165.9 / 14140.3 | 13976.4 |
| | 100 ms | 100 | 0 | 0 | 1492.4 / 12129.1 / 13534.9 | 13390.5 |
| C: 10 ms task overloaded (12 ms work) | 10 ms | 832 | **832** | **167** | 5006.5 / 9896.3 / 10008.2 | 7993.2 |
| | 20 ms | 500 | 0 | 0 | 711.5 / 1227.3 / 1392.6 | 792.0 |
| | 100 ms | 100 | 0 | 0 | 1928.4 / 3757.8 / 3799.4 | 2995.7 |

Observations:
- **A:** no misses; ~0.3–0.5 ms mean wake-up latency, consistent with the raw
  `clock_nanosleep` measurement — this is the WSL2 floor, not scheduler overhead.
- **B:** without real-time priorities, CPU contention pushes worst-case latency above 10 ms and
  the 10 ms task misses deadlines. This is exactly the situation SCHED_FIFO priorities exist for.
- **C:** every activation of the overloaded task misses its deadline and 167 releases are
  dropped (832 + 167 = 999 releases); the other tasks are unaffected because each has its own
  thread on a multi-core host.

### 8.4 ECU task timing (two-ECU demo, 5 s)

| ECU / task | Activations | Misses | Exec mean / max [µs] | Start latency mean / max [µs] |
|---|---|---|---|---|
| cruise Control_10ms | 499 | 0 | 2.0 / 177.2 | 321.5 / 842.2 |
| cruise Com_20ms | 250 | 0 | 98.9 / 226.7 | 182.9 / 375.1 |
| cruise Diag_100ms | 50 | 0 | 6.7 / 92.4 | 1058.2 / 3639.9 |
| sensor Com_10ms | 499 | 0 | 103.2 / 334.3 | 205.6 / 913.0 |
| sensor Diag_100ms | 50 | 0 | 8.0 / 81.8 | 304.4 / 1082.1 |

Estimated CPU utilisation: cruise ECU 0.52 %, sensor ECU 1.04 %. CAN RX: 0 errors, 0 queue
overruns. Timeout detection with the COM supervision now in the 20 ms task: measured
101.8 ms for a 100 ms timeout in the integration test run.

All values are single-run observations on a desktop PC under WSL2 — not guarantees.

---

## 9. Limitations and possible improvements

- **Soft real-time only.** No PREEMPT_RT kernel, no CPU isolation, RT priorities not granted to
  this user; WSL2 adds hypervisor timer latency (~0.4 ms mean).
- **RT path untested here.** The SCHED_FIFO code path is implemented and handled by the tests,
  but could not be exercised without `sudo` (Section 6).
- **Stop latency** is up to one period of the slowest task (100 ms), because tasks sleep on
  absolute time; acceptable for shutdown.
- **Logging** from tasks (state changes, timeouts) formats strings and writes to stderr; fine
  for events, but a production system would use a lock-free log queue drained by a low-priority task.
- **Percentiles** cover the most recent 8192 activations of each task (82 s for a 10 ms task).
- **Execution time** is wall-clock time between start and end, so it includes time the task was
  preempted. A per-thread CPU clock (`CLOCK_THREAD_CPUTIME_ID`) could separate the two.
- Possible next steps: CPU affinity (pin tasks to a core), a cyclic-executive backend for
  comparison, and exporting the JSON reports to plots (Phase 7).
