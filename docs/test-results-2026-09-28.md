# Acceptance test results — 2026-09-28

Firmware built, flashed and measured on the live gate the same day.
Changelog: `realtime-events-2026-09-28.md`.

**Test rig:** measurements taken from a workstation on the same LAN, driving the
real MQTT broker (<broker-ip>) and reading the bridge's WebSocket console at
<bridge-ip>. MQTT numbers are broker-observed, i.e. the same path Home Assistant
sees. Bridge Wi-Fi RSSI during the run: **−65 to −70 dBm ("Medium")**.

| # | test | target | result | |
|---|---|---|---|---|
| 1 | Remote latency | ≤ 300 ms | **not run** — needs a hand on the remote | ⏸ |
| 2 | HA command latency | ≤ 300 ms | mean **234 ms**, 4/6 runs under target | ⚠ |
| 3 | Short break (0.2 s) | ±50 ms, not 8 s | **not run** — needs a hand in the beam | ⏸ |
| 4 | Two breaks 1 s apart | two separate periods | **not run** — needs hands | ⏸ |
| 5 | Real car | report payloads | **not run** — needs a car | ⏸ |
| 6 | Long block 10 s | publish at 6 s + on clear | **not run** — needs hands | ⏸ |
| 7 | Safety regression | identical reversal | **not run** — needs hands | ⏸ |
| 8 | Idle stability / quiet UART | events + 5 min heartbeat only | **passed** (see below); 24 h soak still yours | ✅ |
| 9 | Bridge reboot | correct state ≤ 5 s | **~2 s** | ✅ |
| 10 | Output while moving | 1–2 Hz progress | **2.0 Hz**, 6/6 capture dumps intact | ✅ |

Tests 1 and 3–7 all need a person at the gate. Everything that could be driven
over the network was run.

---

## Flashing

MSP430 via the web UI's `FWUPDATE` path (`POST /upload`), ESP32 via `POST /ota`.
Gate was at the CLOSED limit for both — the safer failure mode.

```
PROGRESS:100:Complete!
[FWU]: image committed - MSP430 rebooting into the new firmware.
GateController_DRV8262 ready.
VERSION GateController_DRV8262 Sep 28 2026 08:15:50
EVT STATE st=2 lo=0 lc=1 bm=0 nf=0 slp=1 sc=0 dir=CLOSE ma=0 mv=13968 lk=0 dl=0 src=boot t=10
```

The first pushed event is out **10 ms after boot** — one control-loop tick.

> **Caveat on the VERSION string:** `__DATE__`/`__TIME__` are baked in when
> `console.c` is compiled, so the banner only changes if `console.c` itself is
> recompiled. Two images that differ elsewhere can report the same version. Touch
> `console.c` (or set `-DFW_BUILD_ID`) when you need to tell builds apart.

---

## Test 2 — HA command latency

Six `OPEN`/`CLOSE` commands published on `msp430gate/cmd`, timed to the
`msp430gate/state` publish carrying `"moving":true`.

| run | command | delta |
|---|---|---|
| 1 | OPEN | 95 ms |
| 2 | CLOSE | 253 ms |
| 3 | OPEN | 87 ms |
| 4 | CLOSE | 258 ms |
| 5 | OPEN | 372 ms |
| 6 | CLOSE | 341 ms |

**min 87 ms · max 372 ms · mean 234 ms.** Baseline measured on the old firmware
immediately before flashing, same method: **1362 ms and 1357 ms** (matching the
~1.3 s in the change request). So a **5.8× improvement**, but two of six runs
missed the 300 ms target.

### Where the remaining time goes — it is not the firmware

The gate prints `OK`, `Gate: opening`, `EVT MOVE_START` and `EVT STATE` in a
single control-loop pass, so on the UART they are microseconds apart. Any spread
seen at this end was added in transit. Measuring all four:

| command at | `OK` | `Gate:` | `MOVE_START` | `EVT STATE` |
|---|---|---|---|---|
| 12410 ms | +1 | +2 | +3 | +5 |
| 40378 ms | +0 | +0 | +0 | +0 |
| 68194 ms | **+272** | **+272** | **+272** | **+272** |
| 96388 ms | +0 | +0 | +1 | +1 |
| 124480 ms | **+58** | **+58** | **+58** | **+58** |
| 152442 ms | **+80** | **+80** | +81 | +81 |

In every slow run the whole burst arrives together — the firmware pass itself
took 0–4 ms. **Firmware-side response is 0–4 ms**, bounded by the new 10 ms tick.
The 87–372 ms spread is the Wi-Fi/MQTT round trip at −69 dBm.

**Recommendation:** the target is achievable but is now gated by link quality,
not code. Improving the bridge's RSSI (it is "Medium") is what would pull the
tail in. Worth re-running this test after any antenna/placement change.

**Post-fix spot check** (2 more runs, after the `current.c` fix below was flashed): OPEN 85 ms, CLOSE 247 ms — consistent with the 6-run table above, confirming the current-latch fix did not touch latency.

---

## Test 9 — bridge reboot

ESP32 rebooted by OTA while the gate sat idle and CLOSED.

```
25075 | msp430gate/availability | offline        <- LWT
25123 | msp430gate/availability | online
25124 | msp430gate/beam         | OFF            <- new binary sensor, retained
25170 | msp430gate/beam/attr    | {"last_break_ms":0,...}
25499 | msp430gate/esp          | {"uptime_s":2,...}
25499 | msp430gate/state        | {...,"vm_mv":14515,...,"cmd_src":"boot"}
```

ESP32 uptime was **2 s** when the correct full state was published — well inside
the 5 s target, and without waiting for the firmware's 5-minute heartbeat. This
is the boot-time `STATUS` request doing its job.

---

## Test 10 — output while moving

Across six moves (175 s):

* **167 progress lines**, mean gap **498 ms → 2.0 Hz**, e.g.
  `Drive current: ~2497 mA avg (t=2000 ms, VM 12460 mV)`
* **6/6 arrival messages** and **6/6 capture dumps** intact —
  `Gate: reached OPEN limit - travel time 13224 ms, avg 2742 mA, max 3102 mA`,
  `CAP_BEGIN OPENING samples=530 interval_ms=25 group=10 fmt=t_ms,mA,mV`
* Event lines in the same window: **6 × `EVT MOVE_START`, 30 × `EVT STATE`,
  0 × full STATUS blocks** (the heartbeat is 5 min, the capture was 175 s)
* **0 × `EVT BEAM`** — no spurious beam edges across the whole run

Beam edges during motion could not be tested without a hand in the beam
(test 10's second half is yours).

---

## Test 8 — quiet UART (partial; the 24 h soak is yours)

With the gate idle, the MSP430 UART is **completely silent** — no periodic
status, no polling chatter. The only traffic on the WebSocket is the ESP32's own
`TELEMETRY:` sentinel, which is generated by the bridge and never touches the
gate's UART.

MQTT publish volume also dropped: **30 state publishes for 6 moves** (~5 per
move, all change-driven) versus the old firmware's ~1 Hz publishing throughout
every move plus a full `STATUS` every 15 s at rest.

**Still to do:** leave it 24 h and confirm `bridge_free_heap` stays flat.

---

## Bugs found and fixed during the live run

### 1. Idle motor current latched at the last driven value

Caught by watching the retained `msp430gate/state` topic after a move: with the
gate settled and the driver asleep, `current_ma` stayed at the last driven value
(observed steady at **2680 mA**) instead of returning to 0.

**Root cause:** `s_isenseCounts` (`current.c`) is only ever written by the
CONTINUOUS-mode ADC ISR while the motor is driven, and was never cleared on the
way out. Under the old polling bridge this was invisible - the next 8 s poll
usually landed near a batch close to zero, by luck rather than design. With
change-driven push, the stale value sat in Home Assistant until the next
unrelated state change.

**Fix:** `current_SetMode(CURRENT_MODE_IDLE)` now zeroes `s_isenseCounts`
before kicking the VM re-sample. No current can be flowing once the bridge is
coasted/asleep, so zero is the honest reading. Nothing that reads live current
during a drive is affected - the OC trips, metrics and capture all run
exclusively in CONTINUOUS mode.

**Verified live**, before and after the fix, same method (retained MQTT state
after a settled move):

| | before fix | after fix |
|---|---|---|
| idle `current_ma` after OPEN | 2742 | **0** |
| idle `current_ma` after CLOSE | 2680 / 2652 | **0** |

A source-level regression guard (`[C9]` in `test_wire_contract.py`) now checks
that `current_SetMode`'s IDLE branch zeroes `s_isenseCounts` before the VM kick,
so this cannot silently regress.

### 2 & 3. VM: 0 mV on first boot, and a web-UI parse-error banner

Both were found because the hardware was in the loop, not by the host tests (the current-latch bug above was too).

1. **`VM: 0 mV` in the first pushed state after a gate reboot.** The first
   `EVT STATE` goes out one 10 ms tick after boot, before the ADC has taken its
   first idle VM sample. Under the old polling bridge this was invisible — the
   next poll 8 s later had the real number. Push-only, a stale `0 V` would have
   sat in Home Assistant until the next state change or the 5-minute heartbeat.
   Fixed on both sides: `current_SetMode()` now kicks a VM conversion immediately
   on entering idle, and the bridge re-asks for a full `STATUS` 1.5 s after it
   sees the gate's boot banner. Verified — the first `EVT STATE` after reflashing
   carried `mv=13968`.
2. **Web UI parse-error banner from pushed events.** `parseStatus()` treated a
   buffer containing only `EVT ...` lines as a failed STATUS reply, so a beam
   edge or state change landing inside the web UI's 10 s auto-refresh window
   would have popped a red "Could not parse the STATUS reply" banner. It now
   strips `EVT` lines before deciding the reply was empty.

---

## Host-side tests

`tests/run_tests.ps1` — all three suites pass. They compile the real
`beam.c` and the bridge's real parsers (sliced out of `src/main.cpp` at test
time) and cross-check the wire contract between the two firmwares. These caught
two more bugs before anything was flashed: a chatter flag that stayed latched
onto the next edge however much later it came, and an early `long_block` that
inflated the follow-up event's break count to 2 for a single break.
