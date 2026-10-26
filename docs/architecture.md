# Architecture

This document describes the platform as a whole: what runs where, which module depends on
which, how data moves through an ECU in one control cycle, and the state machines behind the
behaviour. The per-phase documents explain each part in depth. Engineering decisions are
collected in [engineering_decisions.md](engineering_decisions.md).

Diagrams are written in [Mermaid](https://mermaid.js.org/), which GitHub renders directly.

---

## 1. System context

Everything runs as ordinary Linux processes on one machine (WSL2). The processes talk only over
the CAN abstraction: Linux SocketCAN (`vcan0`) where the kernel supports it, otherwise the UDP
software CAN bus.

```mermaid
flowchart LR
    subgraph host["Linux / WSL2 host (no ECU hardware)"]
        sim["Vehicle simulator<br/>(Python: longitudinal model,<br/>driver, scenarios, fault injection)"]
        tester["UDS tester<br/>(Python: uds_tester.py,<br/>firmware flashing)"]
        cruise["vecu_cruise_ecu<br/>(C++17: cruise control, UDS,<br/>fault management, watchdog,<br/>firmware update)"]
        sensor["vecu_sensor_ecu<br/>(C++17: test traffic)"]
        bus(["Virtual CAN bus<br/>SocketCAN vcan0 or UDP multicast 239.255.0.1"])
        flash[("flash file<br/>A/B slots")]
        nvm[("NVM file<br/>DTC memory")]
    end
    sim <-->|"0x100, 0x110 / 0x200"| bus
    tester <-->|"ISO-TP 0x7E0 / 0x7E8, 0x7DF"| bus
    cruise <--> bus
    sensor <--> bus
    cruise --- flash
    cruise --- nvm
```

**What is simulated and what is real.** The software is real C++/Python that runs, schedules,
encodes frames, diagnoses and verifies images. Four things are simulated:
- the hardware: there is no microcontroller, CAN controller or flash chip
- the bus timing: there is no bit timing or arbitration
- the vehicle: it is a mathematical model
- the "firmware": an image is data that decides the ECU's version and behaviour

Linux is not a real-time OS, so all timing figures are measurements, not guarantees.

---

## 2. Software layers

```mermaid
flowchart TB
    subgraph apps["Applications (apps/): composition roots, no reusable logic"]
        cruiseapp["cruise_ecu<br/>CruiseEcu, FirmwareService,<br/>fault table"]
        sensorapp["sensor_ecu"]
        tools["sched_bench, can_latency"]
        common["common: EcuPlatform (CAN I/O, scheduler,<br/>watchdog thread), options, fault injector"]
    end
    subgraph services["Libraries (include/ + src/)"]
        control["vecu_control<br/>PID, cruise state machine<br/>(no CAN, no threads, no clock)"]
        diag["vecu_diagnostics<br/>ISO-TP, UDS server"]
        faults["vecu_faults<br/>fault manager, DTC memory, NVM"]
        wdg["vecu_watchdog<br/>supervision"]
        fwlib["vecu_firmware<br/>image, flash model, boot record,<br/>bootloader, update manager"]
        canlib["vecu_can<br/>frames, signal codec, CAN matrix,<br/>RX thread, timeout and alive supervision"]
        core["vecu_core<br/>OS port (threads, absolute sleep, PI mutex),<br/>periodic scheduler, timing statistics, logger, CRC-32"]
    end
    subgraph transports["ICanChannel implementations"]
        socketcan["SocketCanChannel<br/>(vcan0 / can0)"]
        udp["UdpCanChannel<br/>(software bus)"]
        mem["VirtualCanBus<br/>(in-memory, unit tests)"]
    end
    cruiseapp --> control & diag & faults & fwlib & common
    sensorapp --> common
    tools --> canlib & core
    common --> canlib & wdg
    diag --> canlib & faults
    faults --> core
    wdg --> core
    fwlib --> core
    canlib --> core
    canlib --> socketcan & udp & mem
```

Rules behind this structure:
- **The control algorithm is pure logic.** `vecu_control` depends on nothing but the standard
  library. It gets inputs as a struct and returns outputs, so the same code runs in unit tests
  with a synthetic clock, in the C++ closed-loop test plant, and in the real ECU.
- **One interface for all buses.** Everything above `ICanChannel` is identical for SocketCAN,
  the UDP bus and the in-memory test bus.
- **Protocol and application are separate.** The UDS server does not know about flash or
  cruise control. The application plugs in DIDs, routines, a download handler and session
  hooks.
- **Composition happens only in `apps/`.** `main()` and the `CruiseEcu` class wire libraries
  together, and all logic worth testing lives in the libraries.
- **The OS port is the replacement point for an RTOS.** Threads, absolute sleep and the
  priority-inheritance mutex are behind `os_port`, and the scheduler uses only that.

---

## 3. Inside the cruise ECU: threads, tasks and data flow

| Thread / task | Priority* | Period | Work |
|---|---|---|---|
| Watchdog | 95 | 20 ms | checks heartbeats, execution times and stalls of all tasks; safe state and recovery |
| CanRx | 90 | event | blocking receive; routes frames to the COM queue or the diagnostic queue |
| Control_10ms | 80 | 10 ms | cruise state machine + PID → throttle command |
| Dcm_10ms | 75 | 10 ms | ISO-TP + UDS server, firmware download |
| Com_20ms | 70 | 20 ms | decode + check received signals, timeout/alive supervision, transmit 0x200 |
| Diag_100ms | 50 | 100 ms | deadline-miss monitor, fault events, NVM write, trial-image confirmation |

\* `SCHED_FIFO` priorities when started with `--realtime` and the permission to use them;
otherwise all run as `SCHED_OTHER` and the priorities are only recorded.

```mermaid
flowchart LR
    bus(["CAN bus"]) --> rx["CanRx thread"]
    rx -->|"0x100, 0x110"| q1[["RX queue (128)"]]
    rx -->|"0x7E0, 0x7DF"| q2[["diag RX queue (128)"]]
    q1 --> com["Com_20ms<br/>decode, plausibility,<br/>timeouts, alive counters"]
    com -->|"SharedSignal: CruiseInputs"| ctl["Control_10ms<br/>CruiseController"]
    ctl -->|"SharedSignal: CruiseOutputs"| com
    com -->|"0x200 CruiseControlStatus"| bus
    q2 --> dcm["Dcm_10ms<br/>ISO-TP + UDS"]
    dcm -->|"0x7E8"| bus
    com & ctl & dcm & diagt["Diag_100ms"] -->|"report()"| fm["FaultManager<br/>+ DtcStore"]
    fm -->|"function inhibited"| ctl
    dcm -->|"download, routines"| fws["FirmwareService"] --> flash[("flash")]
    diagt --> nvm[("NVM")]
    sched["PeriodicScheduler"] -.->|"activation start/end"| wd["Watchdog thread"]
    wd -.->|"safe state / reset"| com
```

Tasks never share mutable state directly. They exchange data through:
- **`SharedSignal<T>`**: a value with a timestamp and a sequence number, guarded by a
  priority-inheritance mutex. The reader can see how old the data is, which is the basis of the
  stale-input and stale-output checks.
- **Bounded queues**: the CAN RX queues hold 128 frames; when full they count the overrun
  instead of blocking.
- **Thread-safe services**: the fault manager and the firmware service each hold one internal
  mutex.

### 3.1 One control cycle

```mermaid
sequenceDiagram
    participant S as Simulator
    participant B as CAN bus
    participant R as CanRx thread
    participant C as Com_20ms
    participant K as Control_10ms
    S->>B: 0x100 VehicleDynamics (speed, valid, alive counter), every 20 ms
    B->>R: frame
    R->>R: push into RX queue (no processing in the RX thread)
    C->>R: pop all queued frames
    C->>C: decode, range + gradient check, alive counter, timeout monitor
    C->>K: write CruiseInputs (SharedSignal, timestamped)
    K->>K: read inputs, check age, state machine + PID
    K->>C: write CruiseOutputs (SharedSignal, timestamped)
    C->>C: check output age (stale output means safe values)
    C->>B: 0x200 CruiseControlStatus (throttle, state, target, reason)
    B->>S: frame, the model integrates throttle into speed
```

### 3.2 A diagnostic request (multi-frame response)

```mermaid
sequenceDiagram
    participant T as Tester
    participant D as Dcm_10ms (ISO-TP + UDS)
    T->>D: SF 03 22 F1 90 (ReadDataByIdentifier VIN)
    Note over D: next Dcm cycle, at most 10 ms later
    D->>T: FF 10 14 62 F1 90 ... (First Frame, 20 bytes)
    T->>D: FC 30 00 00 (continue, no limits)
    Note over D: next Dcm cycle
    D->>T: CF 21 ... and CF 22 ...
```

### 3.3 Firmware update

```mermaid
sequenceDiagram
    participant T as Tester
    participant U as UDS server
    participant F as FirmwareService / UpdateManager
    participant B as Bootloader
    T->>U: 10 03, then 10 02 (programming, only at standstill)
    T->>U: 31 01 FF00 (erase)
    U->>F: erase the slot not running, persist Receiving first
    T->>U: 34 (RequestDownload, size)
    U-->>T: 74 20 0FFF (max. 4095 bytes per block)
    loop blocks of 4093 bytes, ISO-TP block size 32
        T->>U: 36 counter + data
        U->>F: program flash
    end
    T->>U: 37 (transfer exit)
    T->>U: 31 01 FF01 (check)
    U->>F: verify header CRC, payload CRC, hardware, version, security version
    F-->>T: status 00 OK (slot Pending, active for the next boot)
    T->>U: 11 01 (hard reset)
    U->>B: boot: Pending becomes Trial (attempt 1), record stored, image verified
    Note over B: healthy for confirm-after seconds: Confirmed.<br/>3 failed boots: Invalid, then rollback.
```

---

## 4. CAN matrix

Defined in [`config/vehicle_can.dbc`](../config/vehicle_can.dbc); the C++ signal table and the
Python DBC parser are checked against each other by tests.

| ID | Message | Sender → receiver | Cycle | Timeout | Content |
|---|---|---|---|---|---|
| 0x100 | VehicleDynamics | simulator → cruise ECU | 20 ms | 100 ms | speed (0.01 km/h), acceleration, speed valid, alive counter |
| 0x110 | DriverInputs | simulator → cruise ECU | 50 ms | 250 ms | pedals, main switch, set/cancel, desired speed, alive counter |
| 0x200 | CruiseControlStatus | cruise ECU → simulator | 20 ms | (supervised by the simulator) | throttle command, cruise state, target speed, fault reason, alive counter |
| 0x7E0 / 0x7E8 | UDS physical request / response | tester ↔ cruise ECU | on request | P2 50 ms, S3 5 s | ISO-TP |
| 0x7DF | UDS functional request | tester → all | on request | | single frame only |

---

## 5. State machines

### 5.1 Cruise control (`CruiseController`)

```mermaid
stateDiagram-v2
    [*] --> Off
    Off --> Standby: main switch on
    Standby --> Off: main switch off
    Standby --> Active: set request and speed 30..180 km/h, no brake
    Active --> Standby: brake or cancel
    Active --> Override: accelerator pedal above 5 percent
    Override --> Active: accelerator released, resume from current speed
    Override --> Standby: brake or cancel
    Active --> Off: main switch off
    Override --> Off: main switch off
    Standby --> Fault: relevant fault
    Active --> Fault: relevant fault
    Override --> Fault: relevant fault
    Off --> Fault: driver inputs lost, stale inputs, controller fault
    Fault --> Standby: healthy for 0.5 s and main switch on (never back to Active)
    Fault --> Off: healthy for 0.5 s and main switch off
```
In `Fault`, the throttle command is 0 % and the reason is transmitted in 0x200. Relevant faults
include speed timeout or invalid, driver-input timeout, stale inputs, a controller fault, and
function inhibition by the fault manager, the watchdog safe state, the programming session or
bootloader mode.

### 5.2 Fault qualification (`FaultManager`, per fault)

```mermaid
stateDiagram-v2
    [*] --> NotTested
    NotTested --> PreFailed: test failed
    NotTested --> Passed: test passed
    PreFailed --> Failed: debounce complete (time or counter)
    PreFailed --> Passed: passed again (glitch)
    Failed --> PrePassed: test passed (not for latched faults)
    PrePassed --> Failed: failed again
    PrePassed --> Passed: recovery condition met
    Passed --> PreFailed: test failed
```
`Failed` and `PrePassed` count as active (DTC `testFailed`, function inhibition). A latched fault
leaves `Failed` only at the next operation cycle (start, hard reset, watchdog reset).

### 5.3 Software watchdog (per supervised task)

```mermaid
stateDiagram-v2
    [*] --> Ok
    Ok --> Failed: missed heartbeat, execution too long or application stalled
    Failed --> Ok: next check fine (within tolerance)
    Failed --> Expired: violation persists beyond the tolerance (2 checks)
    Expired --> [*]: safe state, then in-process reset, process exit (code 75) or stay
```
After an in-process reset, the ECU leaves the safe state only after 200 ms of healthy
supervision. After 3 watchdog resets in one operation cycle it stays in the safe state.

### 5.4 Firmware slot (`BootRecord`)

```mermaid
stateDiagram-v2
    [*] --> Empty
    Empty --> Receiving: erase routine (persisted before erasing)
    Confirmed --> Receiving: erase routine (only the slot not running)
    Invalid --> Receiving: erase routine
    Receiving --> Pending: download complete and check passed
    Receiving --> Invalid: check failed
    Pending --> Trial: next boot (attempt counter 1)
    Trial --> Trial: boot without confirmation (attempt + 1)
    Trial --> Confirmed: healthy period
    Trial --> Invalid: more than 3 boots, rollback to the other slot
    Confirmed --> Invalid: corrupt at boot, fallback to the other slot
```

---

## 6. Where to look

| Topic | Code | Document |
|---|---|---|
| CAN abstraction, signal codec, supervision | `include/can`, `src/can` | [Phase 1](phase1_virtual_can.md) |
| Scheduler, OS port, timing | `include/core`, `src/core` | [Phase 2](phase2_scheduler.md) |
| PID, cruise state machine, vehicle model | `include/control`, `simulator/vecu_sim` | [Phase 3](phase3_cruise_control.md) |
| ISO-TP, UDS | `include/diagnostics` | [Phase 4](phase4_diagnostics.md) |
| Fault manager, DTCs, NVM, watchdog | `include/faults`, `include/watchdog` | [Phase 5](phase5_fault_management.md) |
| Firmware update | `include/firmware`, `apps/cruise_ecu/firmware_service.*` | [Phase 6](phase6_firmware_update.md) |
| Validation, CI, measurements | `tests`, `scripts`, `.github/workflows` | [Phase 7](phase7_validation.md), [performance report](performance_report.md) |
