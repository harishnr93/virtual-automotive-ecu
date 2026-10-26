# Phase 4 — UDS Diagnostics over ISO-TP

This phase gives the cruise control ECU a diagnostic interface: a tester can identify the
ECU, read live data, read and clear fault memory (DTCs), switch sessions and reset the ECU.
It continues [Phase 3](phase3_cruise_control.md).

> **Scope note.** This is an educational UDS server *inspired by* ISO 14229-1 with an ISO-TP
> transport inspired by ISO 15765-2. It implements a selected subset (Section 7) and has not
> been tested for conformance against the standards or against a commercial tester.

---

## 1. Engineering objective

1. An **ISO-TP transport** that segments messages up to 4095 bytes into CAN frames, with flow control.
2. A **UDS server** for services 0x10, 0x11, 0x14, 0x19, 0x22 and 0x3E, with positive and negative
   responses, request validation and diagnostic session handling.
3. A **DTC memory** with ISO 14229 status bits, fed by the ECU's monitors (timeouts, plausibility,
   controller fault).
4. A **tester** (Python) to use and validate all of it, and measured diagnostic response times.

---

## 2. Concepts

### 2.1 Why a transport layer?
A UDS response such as the VIN is 20 bytes, while a Classic CAN frame carries 8.
**ISO-TP** splits a message into frames and reassembles it:

```
 tester (0x7E0)                                  ECU (0x7E8)
   SF  03 22 F1 90 CC CC CC CC   ─────────►      "read VIN" fits into one Single Frame
                                 ◄─────────  FF  10 14 62 F1 90 56 45 43   First Frame: 0x014 = 20 bytes
   FC  30 00 00 CC CC CC CC CC   ─────────►      Flow Control: continue, BS = 0, STmin = 0
                                 ◄─────────  CF  21 55 53 49 4D 30 30 30   Consecutive Frame, SN 1
                                 ◄─────────  CF  22 30 30 30 30 30 30 31   SN 2 -> "VECUSIM0000000001"
```

| Frame | First byte | Content |
|---|---|---|
| Single Frame (SF) | `0L` | L = length 1–7, then data |
| First Frame (FF) | `1L LL` | 12-bit total length (8–4095), then 6 data bytes |
| Consecutive Frame (CF) | `2N` | sequence number N = 1, 2 … 15, 0, 1 … then 7 data bytes |
| Flow Control (FC) | `3S BS ST` | S: 0 continue / 1 wait / 2 overflow; BS: frames per block (0 = all); STmin: gap between CFs |

The **receiver controls the pace** (BS, STmin). Timeouts protect both sides: the sender waits
at most **N_Bs = 1000 ms** for a flow control, the receiver at most **N_Cr = 1000 ms** for the next CF.

### 2.2 UDS in one paragraph
A request starts with a **service identifier (SID)**. A positive response returns SID + 0x40.
A negative response is `7F <SID> <NRC>`, where the **negative response code** tells the tester
why the request failed. Many services have a **sub-function**; setting bit 7 of it
(`suppressPosRspMsgIndicationBit`) asks the server not to send a positive response.

### 2.3 Sessions
| Session | Entered by | What it allows here | Leaves by |
|---|---|---|---|
| default (0x01) | power-up, reset, `10 01`, S3 timeout | identification, live data, DTCs, hard reset | — |
| extended (0x03) | `10 03` | additionally: serial number, scheduler health DID, soft reset | `10 01`, reset, **S3 = 5 s without requests** |
| programming (0x02) | — | not implemented in Phase 4; added in [Phase 6](phase6_firmware_update.md#210-the-uds-programming-sequence) (from extended only) | — |

`3E 00` (**TesterPresent**) keeps a non-default session alive. Any request restarts the S3 timer.

### 2.4 Physical and functional addressing
- **Physical** `0x7E0 → 0x7E8`: one tester to one ECU, any length.
- **Functional** `0x7DF`: one tester to all ECUs, single frames only. Responses still come back on 0x7E8.
  For functional requests the server **suppresses** NRCs 0x11, 0x12, 0x31, 0x7E and 0x7F, so that
  ECUs which do not support a request stay silent.

### 2.5 DTCs and status bits
A DTC is 3 bytes: a 2-byte SAE J2012 code plus a failure-type byte. Example: `U0121-87` is
encoded as `0xC12187`. Each DTC carries a status byte:

| Bit | Name | Set when | Cleared when |
|---|---|---|---|
| 0 | testFailed | the monitor currently fails | the monitor passes |
| 1 | testFailedThisOperationCycle | failed in this cycle | new operation cycle |
| 2 | pendingDTC | failed | a full cycle without failure |
| 3 | confirmedDTC | failed (Phase 4: immediately) | ClearDiagnosticInformation |
| 4 | testNotCompletedSinceLastClear | after clear | first test result |
| 5 | testFailedSinceLastClear | failed | clear |
| 6 | testNotCompletedThisOperationCycle | new cycle / clear | first test result in the cycle |
| 7 | warningIndicatorRequested | not supported (availability mask 0x7F) | — |

Phase 4 confirms a DTC on its first failure. Debouncing, confirmation counters, aging and
persistent storage are part of the fault manager in Phase 5.

---

## 3. Architecture

```
                         ┌────────────────────────── vecu_cruise_ecu ──────────────────────────┐
 tester 0x7E0/0x7DF ───► │ CanRx thread ─ routing ─┬─► diag RX queue ─► Dcm_10ms (prio 75)       │
                         │  (CanIf-like)           │      IsoTpEndpoint (0x7E8 / 0x7E0)          │
                         │                         │      UdsServer ── DID readers ── SharedSignal│
 tester ◄── 0x7E8 ────── │                         │                └─ DtcStore (thread-safe) ◄─┐│
                         │                         └─► COM RX queue ─► Com_20ms ── monitors ─────┤│
                         │                                             Control_10ms ─ ctrl fault ┘│
                         └──────────────────────────────────────────────────────────────────────┘
```

| File | Content |
|---|---|
| `include/diagnostics/isotp.hpp` | `IsoTpEndpoint`: SF/FF/CF/FC, BS, STmin, FC.Wait limit, N_Bs/N_Cr, padding, fixed 4095-byte buffers |
| `include/diagnostics/uds_server.hpp` | `UdsServer`: services, NRC rules, sessions, S3, suppress bit, functional rules, DID registry, deferred reset |
| `include/diagnostics/diag_config.hpp` | CAN identifiers 0x7E0 / 0x7DF / 0x7E8 |
| `include/faults/dtc_store.hpp` | `DtcStore`: ISO 14229 status bits, operation cycle, thread-safe (PI mutex) |
| `include/can/can_receiver.hpp` | new `addRoute()`: diagnostic IDs go to their own queue |
| `apps/cruise_ecu/main.cpp` | `Dcm_10ms` task, DIDs, DTC monitors, simulated reset |
| `simulator/vecu_sim/isotp.py` | independent Python ISO-TP (tester side) |
| `simulator/vecu_sim/uds_client.py` | UDS client + DID/DTC descriptions (an ODX/CDD-like role) |
| `scripts/uds_tester.py` | command-line tester |

**Separation of concerns:** `IsoTpEndpoint` and `UdsServer` are pure logic. Time is passed in,
frames leave through a callback, and they know nothing about threads or sockets, so all protocol
rules are unit-tested with exact frame and byte expectations. The ECU glue code is ~150 lines.

**Two independent implementations:** the tester's ISO-TP/UDS stack in Python shares no code with
the ECU's C++ stack. Their interoperation in the system tests is evidence that both follow the
same rules, rather than the same bugs.

The DBC file has no new messages: diagnostic frames are transport PDUs, not signal-based
messages (its comment now names the three diagnostic identifiers).

### 3.1 Design decisions

| Decision | Reason |
|---|---|
| **Own ISO-TP implementation** instead of the Linux `can-isotp` kernel module | This WSL2 kernel is built without it (`# CONFIG_CAN_ISOTP is not set`, checked at the start of the phase), and a kernel module could not run over the UDP software bus anyway. Writing it also exposes every protocol rule to unit tests. |
| ISO-TP and UDS as **pure state machines** (time passed in, frames out via callback) | Deterministic tests with exact byte expectations; no threads or sockets inside; portable to an RTOS. |
| **Dedicated `Dcm_10ms` task** (prio 75) with its own RX queue, fed by CAN-ID routing in the RX thread | Diagnostics never delay vehicle-signal processing, and vice versa (similar to AUTOSAR CanIf → CanTp/Dcm vs. Com). Priority below the 10 ms control task: control output wins over a tester. |
| Fixed buffers (3 × 4095 bytes per endpoint, response buffer), DID registry with fixed capacity | No heap use after initialisation. |
| DID values read through `SharedSignal` snapshots | The Dcm task never touches data owned by other tasks directly. |
| `DtcStore` thread-safe (PI mutex) | Written by Com_20ms/Control_10ms monitors, read and cleared by Dcm_10ms. |
| Reset executed **after** the positive response | ISO 14229-1 requires the response before the reset; the server only records the request. |
| Session restrictions **per sub-function and per DID** (soft reset, serial number, scheduler DID) | Shows NRC 0x7E and session-dependent 0x31 without inventing artificial service restrictions. |
| Independent Python tester | Interoperability between two implementations is stronger evidence than a loop-back of one. |

---

## 4. ECU diagnostic content

### 4.1 Data identifiers (0x22)

| DID | Name | Length | Encoding | Sessions |
|---|---|---|---|---|
| F190 | VIN | 17 | ASCII `VECUSIM0000000001` (simulated) | all |
| F187 | Spare part number | 12 | ASCII `VECU-CC-0001` | all |
| F189 | ECU software version | 8 | ASCII `00.04.00` | all |
| F18C | ECU serial number | 10 | ASCII `SIM-000001` | **extended** |
| F186 | Active diagnostic session | 1 | 0x01 / 0x03 | all |
| 0100 | Vehicle speed | 2 | unsigned, 0.01 km/h | all |
| 0101 | Cruise state | 1 | 0 Off … 4 Fault | all |
| 0102 | Cruise target speed | 1 | km/h | all |
| 0103 | Throttle command | 2 | unsigned, 0.01 % | all |
| 0104 | Cruise fault reason | 1 | as CAN signal CruiseFaultReason | all |
| 0200 | Deadline misses (all tasks) | 4 | unsigned | **extended** |
| 0201 | Uptime | 4 | seconds | all |
| 0202 | Reset counter | 2 | unsigned | all |

Up to 8 DIDs per request. Unsupported DIDs are left out of the response; if none is supported,
the server sends NRC 0x31.

### 4.2 DTCs

| DTC | Code | Monitor (test) | Fails when |
|---|---|---|---|
| U0121-87 | 0xC12187 | Com_20ms timeout monitor | VehicleDynamics (0x100) not received for 100 ms |
| U0140-87 | 0xC14087 | Com_20ms timeout monitor | DriverInputs (0x110) not received for 250 ms |
| C0031-64 | 0x403164 | Com_20ms on each 0x100 | speed out of range or `VehicleSpeedValid = 0` |
| U3000-49 | 0xF00049 | Control_10ms | latched cruise controller fault |

A monitor reports no result before its message was received for the first time, which is why
DTCs start as `0x50` (test not completed).

End-to-end coverage (system test with the real simulator): U0121-87 (muted 0x100), C0031-64
(invalid speed), and U0121-87 + U0140-87 together when the simulator stops. **U3000-49 is covered
at unit level only:** the controller fault latches only on NaN, invalid time step or invalid
configuration, which cannot be produced through the validated bus inputs. For the same reason,
"a reset clears a latched controller fault" is shown only at unit level (the reset re-creates the
controller object).

### 4.3 ECU reset (0x11) — simulated
`11 01` (hard reset, any session) and `11 03` (soft reset, extended session only) are answered
first; the reset is executed afterwards, as ISO 14229 requires. The simulation re-initialises
the application state (controller, communication supervision, alive counters), returns to the
default session and increments DID 0x0202. A hard reset also starts a new DTC operation cycle.
**The process does not restart and no memory is lost**: there is no bootloader or real reset
hardware in this platform.

---

## 5. Commands

```bash
cd ~/harish_ws/vir_auto_ecu
cmake --build build -j"$(nproc)" && ctest --test-dir build      # 194 tests

# terminal 1: ECU (optionally with the vehicle simulator, see Phase 3)
./build/bin/vecu_cruise_ecu --duration 120
./scripts/run_closed_loop.sh config/scenarios/cruise_sensor_fault.json   # alternative: with faults

# terminal 2: tester — each argument is one command, run in order on one connection
./scripts/uds_tester.py info live
./scripts/uds_tester.py "session extended" "read-did F18C 0200"
./scripts/uds_tester.py dtc-supported "dtc 08" clear-dtc
./scripts/uds_tester.py "reset hard" "read-did 0202"
./scripts/uds_tester.py "raw 22 F1 90" "raw 10 02" "functional 3E00"
./scripts/uds_tester.py "benchmark 300"

# frame-level view on the software bus (like candump)
./scripts/vbus_dump.py -f 7E0,7DF,7E8
```

---

## 6. Expected behaviour (real tester output)

```
> info
  0xF190 VIN                      = VECUSIM0000000001
  0xF187 SparePartNumber          = VECU-CC-0001
  0xF189 SoftwareVersion          = 00.04.00
  0xF186 ActiveDiagnosticSession  = 1
> read-did F18C
  negative response: 0x31 requestOutOfRange          # extended session only
> session extended
  session=extended P2=50 ms P2*=5000 ms
> reset soft
  soft reset accepted
> read-did F186 0202
  0xF186 ActiveDiagnosticSession  = 1                # reset returned to the default session
  0x0202 ResetCount               = 1
> raw 10 02
  negative response: 0x12 subFunctionNotSupported    # programming session: Phase 6
> functional 2E00
  response: (none)                                   # NRC 0x11 suppressed for functional requests
```

After the simulator muted 0x100 for 0.5 s and later sent an invalid speed, `dtc 08` lists
`U0121-87` and `C0031-64` with status `0x2E`: confirmed, pending, failed this cycle and failed
since the last clear, but not currently failing.

---

## 7. Implemented vs. out of scope

| Area | Implemented | Not implemented |
|---|---|---|
| ISO-TP | normal addressing, Classic CAN, SF/FF/CF/FC, BS, STmin (ms and 100–900 µs), FC.Wait with limit, FC.Overflow, N_Bs/N_Cr, SN check, padding, 4095 bytes | extended/mixed addressing, CAN FD (> 4095 bytes, larger frames), N_As/N_Ar transmit confirmation, functional multi-frame |
| 0x10 | default, extended; P2/P2* in response; S3 timeout | programming session (Phase 6), safetySystem session |
| 0x11 | hard reset, soft reset (extended only), simulated | keyOffOnReset, rapid power shutdown |
| 0x14 | all DTCs (0xFFFFFF), single DTC | DTC groups (powertrain/chassis…), memory selection |
| 0x19 | 0x01, 0x02, 0x0A | snapshots, extended data, other report types |
| 0x22 | up to 8 DIDs, session-dependent DIDs | dynamically defined DIDs (0x2C) |
| 0x3E | 0x00 with suppress bit | — |
| General | NRCs sent: 0x11, 0x12, 0x13, 0x14, 0x31, 0x7E; suppress bit; functional NRC suppression | NRC 0x22 conditionsNotCorrect and 0x7F serviceNotSupportedInActiveSession are defined but never sent (no service has a run-time condition, and all six services are available in both sessions); SecurityAccess (0x27), WriteDataByIdentifier (0x2E), RoutineControl (0x31), IO control (0x2F), CommunicationControl (0x28), ControlDTCSetting (0x85), NRC 0x78 response pending from the server, DoIP, OBD services |

---

## 8. Tests and measured results

### 8.1 New tests (44 C++ test cases, 11 Python unit tests, 2 system tests; CTest total 194)

| Suite | What it proves |
|---|---|
| `IsoTp*` (14) | exact frame bytes for SF/FF/CF/FC; 4095-byte transfer with sequence wrap and BS = 8 (frame count checked); STmin pacing; FC.Wait restart and limit; FC.Overflow; N_Bs and N_Cr timeouts; wrong SN; new SF interrupting a reception; invalid frames ignored; send-length checks; driver TX failure |
| `DtcStoreTest.*` (7) | status bit transitions on fail/pass, occurrence counter, clear single/all, operation cycle aging, status-mask filtering |
| `UdsServerTest.*` (22) | every service, every NRC path, P2/P2* encoding, suppress bit, S3 timeout, deferred reset, session-restricted DIDs and sub-functions, functional NRC suppression, responseTooLong |
| `test_isotp_uds_client.py` (11) | Python ISO-TP against the same byte vectors as the C++ tests; 8–4095-byte transfers with BS; errors; UDS client parsing, NRC and 0x78 handling |
| `test_diagnostics.py` (2) | 3 processes (ECU + simulator + tester): identification, NRCs, functional addressing, live data while cruising, every DID length, S3 timeout, DTCs from real injected faults with exact status 0x2E, clear, hard reset, response time, U0121 + U0140 failing when the simulator stops |
| `UdpCanChannel.ConcurrentSendersAreSafe` | two threads transmit 2 × 300 frames on one channel (as Com_20ms and Dcm_10ms do): all received, none corrupted; also run under ThreadSanitizer |

Verification (2026-10-22, after the review): **194/194 CTest entries passed** in the GCC 11 `-Werror`, ASan + UBSan and
Clang 14 ThreadSanitizer builds (2 vcan cases skipped inside the C++ suites); cppcheck and
clang-tidy: no findings.

### 8.2 Diagnostic response time (tester-measured, request sent → complete response received)

| Condition | Request | n | min | median | p95 | max |
|---|---|---|---|---|---|---|
| ECU idle | `22 0100` (single frame) | 100 | 9.09 | 9.98 | 10.55 | 10.83 ms |
| ECU idle | `22 F190` (multi-frame response) | 100 | 19.38 | 19.99 | 20.53 | 20.89 ms |
| ECU cruising with simulator | `22 0100` | 300 | 9.20 | 9.98 | 10.34 | 10.57 ms |
| ECU cruising with simulator | `22 F190` | 300 | 19.26 | 19.97 | 20.42 | 20.76 ms |
| System test, mixed requests | all | 64 | 0.52 | 9.86 | 19.97 (p95) | 30.06 ms |

All responses are within the server's P2 = 50 ms.

**Why 10 / 20 / 30 ms?** The diagnostic stack runs in the 10 ms `Dcm_10ms` task. Back-to-back
requests arrive just after an activation and wait for the next one (~10 ms). A multi-frame
response needs one more activation to process the tester's flow control (~20 ms), and a
multi-frame *request* adds another (~30 ms). Improvement options: process ISO-TP frames in the
RX thread, or run the Dcm task at 1–5 ms.

### 8.3 Impact on the ECU (25 s cruising, 601 UDS requests)
| Task | Activations | Deadline misses | Exec mean / max |
|---|---|---|---|
| Control_10ms | 2499 | 0 | 4.1 / 206.0 µs |
| Dcm_10ms | 2499 | 0 | 70.6 / 488.2 µs |
| Com_20ms | 1250 | 0 | 118.3 / 3706.7 µs |
| Diag_100ms | 250 | 0 | 10.1 / 168.7 µs |

The control result in the same run was unchanged from Phase 3 (step 60 → 80 km/h: overshoot
0.88 km/h, settling 8.16 s). ISO-TP errors: 0. Alive errors: 0.

### 8.4 Problems found during verification and the Phase 4 review
- **Test race (found in the ASan run):** the system test cleared DTCs as soon as `C0031-64` was
  confirmed, while the injected fault was still active, so the DTC was immediately confirmed
  again. A correct tester waits until `testFailed` = 0 before clearing; the test now does.
- **One-off host stall:** in one of eight full-suite runs, the Python simulator process stalled
  for 164 ms (16 late 10 ms steps), and the Phase 3 test correctly failed, because a stalled
  simulator invalidates the closed-loop KPIs. Four repeated runs afterwards showed 0 late steps
  (max lateness 0.9–4.6 ms). In the same session pytest reported an impossible wall-clock
  duration, which points to a WSL host/clock hiccup. The simulator now records *when* late
  steps happen (`late_at_s`) to make such events diagnosable.
- **Review against the plan (2026-10-22):** the scope table claimed NRCs 0x22 and 0x7F, which the
  server never sends (corrected); the Phase 4 doc lacked the design-decision section required
  for every phase (added, including why ISO-TP is self-implemented); several code comments
  were stale after adding the Dcm task (task list, single-transmitter claim on `CanIo`,
  validation order in `uds_server.hpp`, accessor name); a debug log printed the SID with CAN-ID
  formatting; U0140-87 had no end-to-end test; the "concurrent `send()`" contract was neither
  documented nor tested. All of these were fixed; the test suite grew to 194 entries.

---

## 9. Limitations and possible improvements
- Response time is quantised by the 10 ms Dcm period (Section 8.2).
- DTCs are confirmed on the first failure, with no debouncing, aging or persistence → Phase 5.
- ECU reset is simulated in-process (Section 4.3).
- No security access: in a real ECU, the extended-session functions and resets would be
  protected (0x27); out of scope here.
- One request at a time: a request arriving while a multi-frame response is still being sent is
  dropped (counted), not answered with NRC 0x21 busyRepeatRequest.
- Not checked against a commercial tester or a conformance test suite.
