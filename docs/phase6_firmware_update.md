# Phase 6 — Firmware Update Simulation

This phase lets a tester **reprogram the cruise ECU over UDS**. It also makes sure the ECU
**always keeps a working image**, whatever goes wrong: a corrupt download, a wrong or older
image, power lost in the middle, or new firmware that crashes. It continues
[Phase 5](phase5_fault_management.md).

> **Scope note.** The design follows common automotive practice: A/B partitions, trial boot
> with rollback, and the UDS download services of ISO 14229-1. It is simplified for an
> educational simulation, and it does not claim compliance with ISO 14229, ISO 26262 or any
> cybersecurity standard.
>
> **Integrity is protected by CRC-32 only.** A CRC detects *accidental* corruption. It does
> **not** authenticate an image: anyone can build an image with a valid CRC (section 2.3). The
> design leaves a defined place for a cryptographic signature (section 9.2), but no signature
> is implemented.

---

## 1. Engineering objective

| Plan item | Covered by |
|---|---|
| Firmware image metadata | 64-byte image header: hardware id, version, security version, size, CRC, build time, verification method, flags; protected by its own CRC |
| CRC32 integrity verification | header CRC and payload CRC: checked after download (routine `FF01`) **and at every boot** |
| Version checking | update policy: correct hardware, newer version, security version not lower than the running one (anti-rollback) |
| Update state machine | `UpdateManager`: Idle → Erased → Downloading → Downloaded → Verified / Failed |
| Simulated flash partitions | NOR-flash model (erase = 0xFF, programming only clears bits) with slots A and B and a boot record stored twice |
| Recovery from an interrupted update | the target slot is marked `Receiving` *before* erasing, so it is never bootable; power loss injected at every point (unit test) and as a real SIGKILL (system test) |
| Rollback to the previous valid image | trial boot: a new image must prove itself, or after 3 failed boots the bootloader returns to the old one. A tester can also roll back explicitly (routine `0201`) |
| CRC32 is not authentication | section 2.3, plus a unit test that tampers with an image and recomputes the CRCs |
| Extensible for signature verification | `IImageVerifier` strategy, `verification_method` and `signature_size` header fields; section 9.2 |

---

## 2. Concepts

### 2.1 What a firmware update must guarantee
On a workshop tester or over the air, an update can fail in many ways: the transfer is
corrupted, the wrong file is chosen, the battery dies halfway, or the new software has a bug
that only shows after boot. The one rule that matters most:

> **At any moment, a valid, runnable image must exist and the bootloader must be able to find it.**

Everything in this phase serves that rule: two slots, never erasing the running one, persisting
decisions before acting on them, and verifying images at every boot instead of trusting them.

### 2.2 Image = header + payload
```
 ┌────────────── 64-byte header ──────────────┐┌──────── payload ────────┐
 │ "VFWI" │ hw id │ version │ security ver │ ││ program code (here: a   │
 │ size │ payload CRC │ build time │ method │ ││ deterministic filler)   │
 │ flags │ reserved │ header CRC             ││                         │
 └────────────────────────────────────────────┘└─────────────────────────┘
```
| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `VFWI` |
| 4 / 6 | 2 / 2 | header version (1) / header size (64) |
| 8 | 4 | hardware id: which ECU hardware the image is built for (`0x0000CC01`) |
| 12 | 3 | version major.minor.patch (byte 15 reserved) |
| 16 | 4 | security version: anti-rollback counter |
| 20 / 24 | 4 / 4 | payload size / payload CRC-32 |
| 28 | 8 | build time (seconds since 1970) |
| 36 | 1 | verification method: 0 = CRC-32, 1 = signature (reserved, rejected today) |
| 38 | 2 | signature size (0) |
| 40 | 4 | flags; bit 0 = *simulated defect* (test images only) |
| 44 | 16 | reserved (0) |
| 60 | 4 | header CRC-32 over bytes 0–59 |

All fields are little-endian. The header has its own CRC, so a flipped bit in the *size* or
*version* fields is caught before those fields are trusted. The unit tests flip **every single
bit of the header** (512 cases) and expect a rejection each time.

**What "running an image" means here.** The ECU executable is always the same binary. The image
in flash is *data*, and its header decides what the ECU reports and how it behaves:
- its version (DID `F189`)
- its security version (the anti-rollback limit)
- the defect flag, which makes the Control task hang 1 s after boot. It stands in for "new firmware with a bug".

Everything around the image is real and persistent in the flash file: storage, verification,
boot decisions, trial counting and rollback.

### 2.3 CRC-32 detects accidents — it does not authenticate
A CRC is a checksum: a short value computed from the data so that typical *accidental*
changes produce a different value. That covers single bit flips, burst errors, and a
truncated or half-written image. It is the right tool against transmission errors and
interrupted writes.

It is useless against **intentional** changes. The algorithm is public and has no secret, so an
attacker who modifies the payload simply recomputes the payload CRC and the header CRC, and the
image is "valid" again. The Python test `test_crc32_is_not_authentication` does exactly that.
Proving *who* built an image needs a **digital signature**: the image is signed with a private
key that only the manufacturer holds, and the ECU verifies it with the matching public key.
Without that key, nobody can produce a valid signature. This project does **not** implement
signatures. Section 9.2 shows where they plug in.

Consequence: in this simulation, **anyone who can send UDS requests can flash any image** that
has the right hardware id and a newer version. There is also no SecurityAccess (0x27). Both
would be mandatory on a real ECU.

### 2.4 Version checks and anti-rollback
`checkCompatibility()` applies three rules, in this order:
1. **Hardware id** must match. A file for another ECU is rejected (`WrongHardware`).
2. **Security version** must be ≥ the running image's. It is raised when a security bug is
   fixed, so a known-vulnerable older image can never be installed again
   (`SecurityVersionRollback`).
3. **Version** must be newer than the running one (`VersionNotNewer`). A workshop mode
   (`allow_same_or_older_version`) can relax this rule, but never rule 2.

The explicit **rollback** (routine `0201`, or the automatic one after failed boots) is not a
download. It returns to the image that already ran successfully on this ECU, so it is allowed.

### 2.5 Simulated NOR flash
Real program flash differs from a file in two ways, and the model reproduces both:
- **Erase** works on whole sectors (here 4 KiB) and sets every byte to `0xFF`.
- **Programming** can only change bits from 1 to 0. Writing `0x5A` over `0xA5` fails with
  `NotErased`, so an area must be erased before it is rewritten.

`RamFlash` keeps the memory in RAM and can **inject a power loss** after N programmed bytes: the
bytes before the cut are written, the rest are not, and the device is "off" until
`powerCycle()`. `FileFlash` keeps the memory in a file (`--flash FILE`), so the content survives
an ECU restart just as flash survives a power cycle.

### 2.6 A/B slots
```
 0x00000 ┌───────────────┐
         │ slot A 256 KiB│  factory image 0.6.0 (provisioned on a blank flash)
 0x40000 ├───────────────┤
         │ slot B 256 KiB│  updates alternate between A and B
 0x80000 ├───────────────┤
         │ boot record 0 │  4 KiB sector
 0x81000 ├───────────────┤
         │ boot record 1 │  4 KiB sector
 0x82000 └───────────────┘
```
An update is always written into the slot the ECU is **not running from**. The running image is
never touched, so the old image is still there if anything fails.

### 2.7 The boot record, stored twice
The boot record (32 bytes, with its own CRC) holds:
- which slot is active
- the state and version of each slot
- the boot attempt counter
- the last boot event

Changing it means erasing and rewriting a sector, so a power loss *during* that write would
destroy it. The record is therefore stored in two sectors with a **sequence number**:
- each write goes to the *older* copy, with sequence + 1
- at boot, the valid copy with the highest sequence wins

A torn write leaves the other, older copy intact. Unit test:
`PowerLossDuringRecordWriteKeepsPreviousRecord`.

### 2.8 Slot states and the update state machine
```
            erase routine                    check OK
 Empty/Confirmed/Invalid ──► Receiving ──(download, check)──► Pending ──(reset)──► Trial
                               │  never bootable                                 │      │
                               └──── check failed ──► Invalid    confirmed ◄──────┘      │ 3 failed boots
                                                                 (healthy)               ▼
                                                                                      Invalid → rollback
```
The **order of steps** is what makes an interrupted update harmless:
1. Persist `Receiving` for the target slot, **then** erase it. From this moment the slot is
   never bootable, even if it later contains a perfect copy of a valid image.
2. Download in blocks.
3. Verify the whole image in flash (header CRC, size, payload CRC, compatibility).
4. Only then persist `Pending` and make the slot active **for the next boot**.

If power is lost before step 4, the record still says `Receiving` and the old image boots.

`UpdateManager` states: `Idle → Erased → Downloading → Downloaded → Verified`; any error leads
to `Failed`. Leaving the programming session or resetting **aborts** a download in progress.

Two safety rules (found during integration, see 8.4):
- **The target is always the slot the ECU is *not* running from.** This holds even after a
  successful check has already made the other slot active, so a second download before the
  reset cannot erase the running image.
- **No new download while the running image is still a trial.** Its partner slot holds the only
  confirmed fallback (`NotPossible` → NRC 0x22).

### 2.9 Trial boot, confirmation, rollback
A verified image is not trusted yet: it may contain a bug that only shows at run time.
- At the next boot, the bootloader turns `Pending` into **`Trial`**, sets `boot_attempts = 1`, and
  **stores the record before starting the application**.
- If the application runs healthily for `--confirm-after` seconds (default 5 s; healthy means no
  watchdog safe state), the Diag task **confirms** it: the slot becomes `Confirmed` and the
  counter is cleared.
  The healthy period is measured from the boot time recorded by `FirmwareService` under the same
  lock as the trial state, so it can never start before the boot. A race found in Phase 7 once
  allowed exactly that ([Phase 7, 7.4](phase7_validation.md#74-problems-found-during-this-phase)).
- If the ECU resets before confirming (watchdog, crash, power loss), the next boot increments
  the counter. When the count exceeds `max_boot_attempts = 3`, the image is marked `Invalid` and
  the bootloader **rolls back** to the other, confirmed slot. It reports `U3000-96` (fault
  `FirmwareRollback`).

The bootloader also **re-verifies the active image at every boot**. If it has become corrupt
(e.g. bit rot), it falls back to the other confirmed image (`FallbackCorrupt`). If no image is
valid at all, the ECU starts in **bootloader mode**: the cruise function is disabled, but UDS
works, so it can be reflashed.

### 2.10 The UDS programming sequence
```
 10 03            extended session
 10 02            programming session  (only from extended; only at standstill → else NRC 0x22)
 31 01 FF 00      routine eraseMemory: inactive slot ("Receiving", erased)
 34 00 44 <addr 4> <size 4>   RequestDownload  → 74 20 0F FF  (max. 4095 bytes per 0x36 request)
 36 01 <4093 B>   TransferData, block counter 1, 2, … 255, 0, 1 …   → 76 <counter>
 …
 37               RequestTransferExit
 31 01 FF 01      routine check image → 71 01 FF 01 <00 = ok | 01 = failed> <image error>
 11 01            ECU hard reset → bootloader → trial boot of the new image
```
The routine identifiers `FF00`/`FF01` follow the ones ISO 14229-1 lists for erase memory and
checking programming dependencies. A repeated `0x36` with the same counter (the tester did not
get the response) is acknowledged without writing twice.

| Service | Sessions | NRCs used |
|---|---|---|
| `10 02` programming | from extended only | 0x7E (from default), 0x22 (moving / cruise active) |
| `31 01` routines | per routine | 0x31 (unknown / wrong session), 0x22, 0x24, 0x72, 0x12 (stop/results not implemented) |
| `34` RequestDownload | programming | 0x7F, 0x13, 0x31 (format / size), 0x70 (not erased), 0x22 (already active) |
| `36` TransferData | programming | 0x7F, 0x13, 0x24 (no download), 0x73 (counter), 0x71 (more data than requested), 0x72 (flash error) |
| `37` RequestTransferExit | programming | 0x7F, 0x13, 0x24 (incomplete) |
| `14`, `19` (DTC services) | **not** in programming | 0x7F |

New DIDs (readable in all sessions):

| DID | Bytes | Content |
|---|---|---|
| `F189` | 8 | software version **of the booted image** (was a constant until Phase 5) |
| `0400` | 7 | running slot (0 = A, 1 = B, 0xFF = none), state A, state B, boot attempts, max attempts, last boot event, rejected image error |
| `0401` | 6 | update state, last image error, bytes received (u32) |
| `0402` | 10 | version of slot A, version of slot B, running security version (u32) |

### 2.11 Flow control: how fast can we download?
One `0x36` request carries up to 4095 bytes: 585 CAN frames. The ECU's diagnostic receive queue
holds 128 frames and the Dcm task drains it every 10 ms. Before this phase, the ECU's flow-control
frame said "BS = 0" (send everything), so a tester would overflow the queue and the transfer
would fail.

The ECU now announces **block size 32**: after 32 frames the tester waits for the next
flow-control frame, which the ECU sends once its Dcm task has consumed them. The expected
throughput is therefore 32 × 7 bytes per 10 ms ≈ 21.9 KiB/s. The measured value is 19.8 KiB/s
(section 8). This is exactly the trade-off real ECUs tune with BS/STmin.

---

## 3. Architecture

```
 tester (Python): uds_client.flash_image() / uds_tester.py flash / fwtool.py
        │ ISO-TP (BS 32)
 vecu_cruise_ecu ── Dcm_10ms: UdsServer ──(IDownloadHandler, routines, session hooks)──► FirmwareService
                    Diag_100ms: trial confirmation                                       (apps/cruise_ecu)
                    Control_10ms: inhibited in programming session / bootloader mode          │
                    reset (UDS 11 01, watchdog) ──► boot()                                    ▼
 ─────────────────────────────────────── vecu_firmware library (no UDS, no threads) ────────────────
   UpdateManager (state machine) · Bootloader (trial / rollback / fallback) · BootControl (A/B record)
   firmware_image (header, CRC, policy, IImageVerifier) · IFlash: RamFlash (tests) / FileFlash (--flash)
```

| File | Content |
|---|---|
| `include/firmware/firmware_image.hpp` | header format, `buildImage`, `checkCompatibility`, `IImageVerifier` / `Crc32Verifier`, `verifyImageInFlash` |
| `include/firmware/flash_memory.hpp` | `IFlash`, `RamFlash` (power-loss injection, bit flips, erase counters), `FileFlash` |
| `include/firmware/boot_control.hpp` | flash layout, slot states, boot record with two copies |
| `include/firmware/bootloader.hpp` | boot decision: provisioning, trial, rollback, fallback, no valid image |
| `include/firmware/update_manager.hpp` | erase / download / exit / check / rollback / abort |
| `include/diagnostics/uds_server.hpp` | programming session, 0x31, 0x34/0x36/0x37, `IDownloadHandler`, session guard and change hook |
| `apps/cruise_ecu/firmware_service.*` | glue: UDS ↔ firmware library, DIDs, routines, timing measurement, one mutex |
| `simulator/vecu_sim/firmware_image.py` | **independent** Python implementation of the format (written from section 2.2, not translated from C++) |
| `scripts/fwtool.py` | build / inspect / corrupt images, inspect a slot of the ECU flash file |
| `config/scenarios/workshop_flash.json` | vehicle drives, then stops: programming refused, then accepted |

The UDS server knows nothing about flash, and the firmware library knows nothing about UDS or
threads. Each is tested on its own; `FirmwareService` is the only place where they meet.

---

## 4. Design decisions

- **A/B slots instead of "bootloader overwrites the application in place".** In-place updates
  leave no runnable image while flashing, and need a bootloader that can always reflash.
  A/B costs twice the flash but makes every failure recoverable. It is the approach used for
  over-the-air updates.
- **Programming inside the application.** Many classic ECUs jump to a separate bootloader for the
  programming session. With A/B slots the running application can write the *other* slot
  safely. The bootloader logic still runs at every (simulated) reset.
- **Persist first, act second.** `Receiving` before erase, `Pending` after verification, and
  `Trial` + attempt counter before the application starts. Every decision survives a power loss
  at any point.
- **Verify at every boot, not only after download.** Flash content can change: bit rot, or a
  defective write that the download check did not see.
- **Confirmation by a healthy period** is the simplest observable criterion. A real product would
  confirm after a self-test, e.g. all monitors passed and communication established.
- **Two independent implementations of the image format** (C++ and Python). Each one finds
  the other's mistakes: the system test parses the factory image that the C++ ECU wrote, using
  the Python code.
- **`IImageVerifier` strategy.** The bootloader and update manager call `verifier.verify()` and do
  not care whether it computes a CRC or checks a signature (section 9.2).
- **Standstill guard via the session hook.** The UDS server offers a generic
  "may I change session?" callback. The ECU decides that programming needs speed ≤ 1 km/h and no
  active cruise control. The cruise function is inhibited for the whole programming session.

---

## 5. Commands (WSL2)

```bash
cmake --build build -j"$(nproc)"

# Terminal 1: cruise ECU with a persistent flash file (first start provisions factory image 0.6.0)
./build/bin/vecu_cruise_ecu --flash /tmp/ecu_flash.bin --confirm-after 3

# Terminal 2: build an update, flash it, watch the trial and its confirmation
./scripts/fwtool.py build --version 0.7.0 --out /tmp/fw_070.bin
./scripts/uds_tester.py fw-status "flash /tmp/fw_070.bin" fw-status
sleep 4; ./scripts/uds_tester.py fw-status          # state_b=Confirmed

# Rejections (the old image keeps running)
./scripts/fwtool.py build --version 0.5.0 --out /tmp/fw_old.bin            # VersionNotNewer
./scripts/fwtool.py corrupt /tmp/fw_070.bin --offset 5000 --out /tmp/fw_bad.bin   # PayloadCrcMismatch
./scripts/fwtool.py build --version 0.8.0 --hardware-id 0xBB02 --out /tmp/fw_hw.bin # WrongHardware
./scripts/uds_tester.py "flash /tmp/fw_bad.bin"

# Explicit rollback to the previous confirmed image
./scripts/uds_tester.py "session extended" rollback "reset hard" fw-status

# Defective firmware: restart loop like a hardware watchdog reset (Ctrl+C to stop)
./scripts/fwtool.py build --version 0.9.0 --defect --out /tmp/fw_defect.bin
while true; do ./build/bin/vecu_cruise_ecu --flash /tmp/ecu_flash.bin --watchdog-action exit; \
  [ $? -eq 75 ] || break; done        # terminal 1
./scripts/uds_tester.py "flash /tmp/fw_defect.bin"                   # terminal 2; 3 trial boots, then Rollback

# Inspect the flash file offline
./scripts/fwtool.py inspect /tmp/ecu_flash.bin --slot A

# Tests of this phase only
./build/bin/vecu_unit_tests --gtest_filter='FirmwareImage*:RamFlash*:FileFlash*:FirmwareTest*:UdsProgramming*'
PYTHONPATH=simulator python3 -m pytest -s tests/unit/python/test_firmware_image.py tests/integration/test_firmware_update.py
```

---

## 6. Expected behaviour (real ECU log, abbreviated)

Successful update (`--confirm-after 2`):
```
BOOT event=Normal app_valid=1 slot=A version=00.06.00 security_version=1 state_a=Confirmed state_b=Empty attempts=0/3
UDS_SESSION extended
UDS_SESSION programming
FW_ERASE slot=B result=Ok ms=4.34
FW_DOWNLOAD_START address=0x0 size=65600 result=Ok
FW_DOWNLOAD_DONE bytes=65600 s=3.24 kib_per_s=19.78 max_block_ms=0.081
FW_CHECK result=Ok image_error=None ms=2.72 version=00.07.00 (activated for the next reset as trial image)
UDS_SESSION default
ECU_RESET type=hard (simulated: application state re-initialised, process keeps running)
BOOT event=TrialBoot app_valid=1 slot=B version=00.07.00 security_version=1 state_a=Confirmed state_b=Trial attempts=1/3
FW_CONFIRMED version=00.07.00 slot=B                       (2.05 s later)
```
Defective image with the restart loop: the boots seen by the tester (system test output) were
`(00.09.00, TrialBoot, 1)`, `(00.09.00, TrialBoot, 2)`, `(00.09.00, TrialBoot, 3)`,
`(00.06.00, Rollback, 0)`. Each trial ends with `FW_DEFECT … Control task hangs`, a watchdog
expiration and exit code 75. After the rollback, U3000-96 is confirmed and slot B is `Invalid`.

---

## 7. Tests

| Suite | Type | What it proves |
|---|---|---|
| `FirmwareImage.*` (7) | unit | header round trip; **all 512 single-bit header flips detected**; malformed headers; payload flips at first/middle/last byte; compatibility rules incl. anti-rollback in workshop mode; signature method rejected, never silently accepted |
| `RamFlash.*`, `FileFlash.*` (3) | unit | NOR semantics (erase to 0xFF, 1→0 only, alignment, range); power loss cuts a write at an exact byte; file flash persists across reopen |
| `FirmwareTest.*` (20) | unit, deterministic | record copies alternate (wear spread); torn record write and corrupt newest copy fall back to the older copy; provisioning; trial → confirm; **rollback after 3 unconfirmed boots**; fallback from a corrupt active image; no valid image → flashing still possible; rejected download keeps the old image; downgrade / hardware / security rollback; block counter rules, retransmission, wrap 255 → 0; abort; **power loss at 21 points across the whole update: always a bootable image, new only if fully verified**; the update never targets the running slot; no update during a trial; rollback discards a pending image |
| `UdsProgrammingTest.*` (11) | unit (fake handler) | session transitions and guard; per-session service availability; routines; complete download; format and length checks; handler NRCs passed through; maximum block length; download aborted by session change, S3 timeout and reset |
| `test_firmware_image.py` (9) | Python unit | round trip, all header bit flips, payload corruption, truncation; payload generator equals the C++ one (golden CRC); **CRC is not authentication** (tamper + recompute passes) |
| `test_firmware_update.py` (6) | system: real ECU process, flash file, UDS over the bus | update → trial → confirmation → persists across a restart; Python verifies the C++ factory image in the flash file; 6 invalid images and 9 protocol errors rejected with the right NRC/reason; **SIGKILL during download** → old image boots, retry succeeds; tester vanishes → S3 aborts the download; **defective image → 3 watchdog restarts → rollback** with DTC; programming refused while driving, flashing at standstill with sensor traffic |

Changed Phase 4 expectations: `10 02` from the default session now gives 0x7E instead of
0x12 (programming exists, but only via the extended session). `F189` now reports the booted
image (00.06.00).

---

## 8. Measured results (2026-10-26, WSL2, real processes)

### 8.1 Update timing
| Measurement | Result |
|---|---|
| 64 KiB image (65 600 B), tester view | erase 13–14 ms, download **3.23–3.24 s = 19.8 KiB/s**, check 4–10 ms, total 3.3 s |
| Same, with the vehicle simulator sending sensor traffic | 19.6 KiB/s |
| Theoretical limit with BS 32 and a 10 ms Dcm cycle | 21.9 KiB/s (measured: 90 %) |
| ECU-side durations (file flash) | erase of a 256 KiB slot 3.2–4.3 ms; write of one 4093-byte block ≤ 0.16 ms; check (CRC of 64 KiB + record write with fsync) 2.7–3.1 ms; trial boot 3.2–3.3 ms; normal boot 0.18 ms; first provisioning 6.4 ms |
| Dcm_10ms during a complete update | 0 deadline misses; **3 budget overruns** (budget 3 ms; erase, check and reset/boot); maximum execution 8.5 ms (watchdog execution limit 20 ms) |
| Confirmation of a trial image | 1.59 s after the reset with `--confirm-after 1.5` (checked every 100 ms) |

### 8.2 Failure handling
| Scenario | Result |
|---|---|
| Payload bit flip / header bit flip / 0.5.0 / same version / wrong hardware / security version 0 | rejected by `FF01` with `PayloadCrcMismatch` / `HeaderCrcMismatch` / `VersionNotNewer` ×2 / `WrongHardware` / `SecurityVersionRollback`; slot B `Invalid`, 00.06.00 keeps running |
| ECU killed (SIGKILL) at ~50 % of a 48 KiB download | restart: `BOOT event=Normal`, 00.06.00, slot B `Receiving`; the retried update succeeded |
| Tester disappears after the first block | S3 timeout after 5 s: back to the default session, download `Failed`, 4093 bytes received |
| Defective image 0.9.0 with `--watchdog-action exit` + restart loop | 3 trial boots, each ending in exit code 75; the 4th boot rolled back to 00.06.00; U3000-96 confirmed; the old image then ran without further watchdog exits |
| Programming requested while the cruise control was active at 50 km/h | NRC 0x22; accepted after the vehicle had stopped |

### 8.3 Verification
GCC 11 `-Werror`: 0 warnings. **267 CTest entries** pass in a fresh GCC build and in the ASan + UBSan
and Clang ThreadSanitizer builds (2 vcan cases skipped inside the C++ suites, as before). The
firmware system tests also check the ECU exit code, so a sanitizer report inside the ECU process
fails them. cppcheck and clang-tidy: no findings.

### 8.4 Problems found during development
- **Design flaw: a second download could erase the running image.** After a successful check,
  the new slot becomes "active" before the reset. A second download then picked the *other* slot,
  which is the running one, and would have erased the only confirmed image. A related case:
  flashing while a trial image runs would erase its only fallback. Fix: `UpdateManager` knows the
  running slot, always targets the other one, and refuses updates during a trial. Three new unit
  tests.
- **ISO-TP flow control was not ready for large requests.** With BS 0, a 4 KiB TransferData would
  overflow the 128-frame diagnostic queue. The ECU now announces BS 32 (section 2.11).
- **Machine-readable log contract.** Adding the version string `00.06.00` to the `SUMMARY` line
  broke the integration tests, which parse every numeric-looking value as a number. Firmware data
  now has its own `FW_SUMMARY` line.
- **Static analysis** flagged a convoluted branch (dead store, cloned branch) in the bootloader's
  "no valid image" path. It was simplified and the path got its own unit test.
- GCC 11 `-Wstringop-overread` false positive in `buildImage` (`vector::insert`): replaced by a
  pre-sized vector and `std::copy`.
- Test-side: an ECU killed mid-download shows up at the tester as an ISO-TP N_Bs timeout (no
  flow control), not as a UDS P2 timeout. The test accepts both.

---

## 9. Limitations and possible improvements

### 9.1 Limitations
- **No authenticity, no access control.** CRC-32 only, and no SecurityAccess (0x27): anyone on the
  bus can flash an image with the right hardware id and a newer version (section 2.3).
- **Anti-rollback minimum comes from the running image.** A real ECU stores the minimum security
  version in tamper-resistant, monotonic storage (e.g. OTP fuses), so it cannot be lowered by
  any flash content.
- **Flash timing is file I/O**, much faster than real NOR flash: real sector erases take tens to
  hundreds of ms. A real ECU answers long jobs with NRC 0x78 (response pending) and runs them in a
  background flash-driver job instead of inside the Dcm task (here: budget overruns, section 8.1).
- **Images are data, not code** (section 2.2). There is no real jump to a bootloader, no separate
  bootloader binary that could itself be updated, and no secure-boot chain.
- **Power-loss model:** writes are cut at a byte boundary. Real flash can leave cells in undefined
  states; those would also be caught by the CRCs, but are not simulated.
- UDS subset: `dataFormatIdentifier` 0x00 only (no compression or encryption), address must be 0
  (slot-relative), no `0x35` upload, no `0x38` file transfer, no routine stop/results.
- Confirmation criterion is a healthy period, not an application self-test. `max_boot_attempts`
  is fixed at 3.

### 9.2 How a signature would be added
The extension points exist; nothing in the bootloader or update state machine changes:
1. **Image:** set `verification_method = 1`, append the signature after the payload
   (`signature_size` bytes, e.g. 64 for Ed25519 or ECDSA P-256). Sign *header (with
   `signature_size` set) + payload*, so no field can be changed without invalidating the signature.
2. **Verifier:** implement `IImageVerifier` (`method() = Signature`): hash header + payload from
   flash in chunks and verify the signature with the manufacturer's **public key**. The key would
   be compiled into the bootloader, ideally in immutable or write-protected storage. Use a
   reviewed library (e.g. libsodium or mbedTLS) rather than custom cryptography.
3. **Wiring:** pass that verifier instead of `Crc32Verifier` to `Bootloader` and `UpdateManager`.
   They already reject images whose method does not match the verifier, so after the switch an
   unsigned (CRC-only) image is refused automatically. A unit test already shows the reverse case:
   a signature image is rejected by the CRC verifier, never silently accepted.
4. **Around it:** SecurityAccess (0x27) before the programming session, a monotonic security
   version counter, key revocation, and a secure-boot chain that verifies the bootloader itself.
   CRC-32 can stay as a fast transfer check, but it is never the security mechanism.
