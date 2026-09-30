# Real-time status + beam/vehicle detection — 2026-09-28

Implements the Home Assistant firmware change request across both firmwares:

| | |
|---|---|
| MSP430 gate controller | `msp430-gate-controller/` |
| ESP32-C3 bridge | `esp32-wifi-bridge/` |
| HA device id | `msp430gate_<mac6>` |

---

## 1. Root cause — confirmed, and it was the one in the request

The request's read was right, with one correction: the ~8 s interval was not the
MSP430 polling itself, it was **the ESP32 asking the MSP430 for `STATUS` every
8 s at rest** (`MQTT_POLL_MS_IDLE`, 1 s while moving). The MSP430 never spoke
unprompted at all. That single interval explains every symptom in the request:

* **Remote-started moves 2.8–5.8 s late** — a random point inside the 8 s window.
* **HA-commanded moves ~1.0 s** — a separate 4 s `CMD_SETTLE_MS` window already
  forced 1 s polling right after a relayed command, so those landed inside 1 s.
* **Beam breaks missed or exactly 8.1 s long** — the beam was never its own
  message. It was one field inside whatever `STATUS` frame happened to go out
  next, so a break shorter than the poll interval either fell between two polls
  or was stretched to exactly one poll period.
* **Beam changes sharing a timestamp with unrelated state changes** — same cause:
  one frame carried both.

There was no 8 s latch anywhere in the firmware. Nothing to remove.

A second, smaller contributor was the MSP430's tick order: `gate_fsm_Tick()` ran
*before* `console_Service()`, so a command that arrived during a tick was only
acted on by the state machine on the **next** tick — up to 50 ms at rest.

**UART TX was already non-blocking** (interrupt-driven ring buffer in `uart.c`),
so the request's "if TX is a blocking printf" hypothesis did not apply. One real
problem was found in that area and fixed — see §2, capture dump.

---

## 2. MSP430 changes

### New: pushed event lines (`gate_fsm.c`, `beam.c`)
Full spec in [`SERIAL_PROTOCOL.md`](../msp430-gate-controller/docs/SERIAL_PROTOCOL.md) §5.0. Summary:

```
EVT STATE st=3 lo=0 lc=0 bm=0 nf=0 slp=0 sc=0 dir=OPEN ma=1840 mv=12960 lk=0 dl=0 src=remote t=987654
EVT BEAM b=1 t=123456 dur=0 n10=1
EVT BEAM b=0 t=123656 dur=200 n10=1
EVT BEAMACT ev=vehicle_pass pat=axle_pair n=2 tot=1453 gs=1 t=123456789 brk=142,131 gap=1180
EVT MOVE_START dir=OPEN src=remote st=3 t=987654
```

* `EVT STATE` is emitted the moment any **discrete** field changes. ~130 bytes,
  one line, ~11 ms on the wire — versus ~50 ms for a whole `STATUS` block.
* `EVT MOVE_START` is emitted in the **same control-loop pass that energises the
  motor**, from every path: HA/MQTT/web command, RF remote, wired button, and
  auto-reverse.
* The photo beam is **excluded** from the `EVT STATE` change test on purpose —
  it has its own, higher-priority line, so a hand wave does not drag a full state
  line onto the wire.

### New: `beam.c` / `beam.h` — beam reporting and classification
Pure observation. It reads the same debounced `input_Level(IN_PHOTOBEAM)` the
safety logic reads and never writes anything the state machine looks at.

* Every edge reported immediately with its real duration. No latch, no minimum
  hold.
* Classification window opens on the first blocked edge, closes after
  `BEAM_AXLE_GAP_MAX_MS` (3500 ms) clear. Then one `EVT BEAMACT`:
  * `vehicle_pass` / `axle_pair` — ≥2 breaks in 30–1000 ms with a 300–3500 ms gap
  * `vehicle_pass` / `body` — one break in 700–6000 ms
  * `short_break` — anything else (person, dog)
  * `long_block` — published **as soon as** a break reaches 6000 ms, then again
    on clear with the total. Both report `n=1`.
* Thresholds are `#define`s in `gate_config.h` (`BEAM_*`), **not** nvconfig
  params. Deliberate: the nvconfig blob is version-stamped and bumping the
  version **re-seeds every saved parameter back to factory defaults**, which
  would throw away the gate's tuned OC / ramp / soft-end values. Tune them from
  the `msp430gate/diag/beam_edges` topic and rebuild.
* Chatter guard: >10 edges in one second ⇒ edge lines throttle to one per 100 ms
  with `chat=1`. Safety path unaffected.

### Timing
* **Idle tick 50 ms → 10 ms** (`TICK_IDLE_MS`, `TICK_IDLE_TACCR0` 15624 → 3124).
  Beam edges are now seen within 10 ms and timed to ±10 ms instead of ±50 ms.
  Cost: the CPU leaves LPM0 100×/s instead of 20×/s — each wake is tens of µs,
  so <1 % duty on top of an LPM0 baseline that already runs SMCLK at 20 MHz.
* **`console_Service()` now runs BEFORE `gate_fsm_Tick()`** — a command received
  this tick is executed this tick, not the next one. This also removes the race
  the ESP32's `CMD_SETTLE_MS` workaround existed to paper over.
* `IDLE_VM_SAMPLE_TICKS` 20 → 100 so VM sampling stays ~1 Hz.
* `led_Service()` fault blink is now accumulated in **real milliseconds**. It
  counted raw ticks with a hard-coded 50 ms/1 ms split, which would have silently
  become a 2 s "fast" blink at the new tick rate.
* New `board_UptimeMs()` — a real ms clock accumulated in the Timer_A2 ISR.
  `g_tickCount` could not be used: its ticks are not constant duration.

### Photo-beam debounce 30 ms → 10 ms (`PHOTOBEAM_DEBOUNCE_MS`)
**This is the one change that touches the safety path, and it is deliberate.**
`input_Level(IN_PHOTOBEAM)` is the single source of truth for both the reversal
logic and the new reporting, so beam-triggered reversal now reacts ~20 ms
*sooner* than before. That is strictly in the safe direction. It is needed
because `BEAM_AXLE_BREAK_MIN_MS` is 30 ms and a 30 ms debounce would swallow the
shortest real tyre break. Set `-DPHOTOBEAM_DEBOUNCE_MS=30` to restore the exact
previous reversal timing, at the cost of missing the shortest tyre breaks.

### Quieter UART
* No periodic output at rest at all. One full `STATUS` block every 5 min
  (`STATUS_HEARTBEAT_MS`) as a staleness backstop, plus `EVT` lines on change.
* `STATUS` is now also the **status request** — the bridge sends it at boot and
  on every MQTT reconnect and gets an immediate full block.
* `INPUT: TOGGLE button pressed` / `INPUT: REMOTE toggle asserted` moved behind
  `-DGATE_DEBUG=1`, off by default. `EVT MOVE_START src=` replaces them.
* Progress lines while moving stay at ~2 Hz and gained elapsed time and VM:
  `Drive current: ~1840 mA avg (t=3500 ms, VM 12960 mV)`.

### Capture-dump UART headroom (`current.c`)
The end-of-move CSV dump filled the 4 KB TX ring to within 220 bytes of full, so
a beam edge landing mid-dump could sit ~350 ms behind it at 115200 baud. The dump
now never queues more than `CAPTURE_DUMP_MAX_QUEUED` (512) bytes at a time,
bounding that to ~45 ms. The dump is UART-rate-limited either way, so it takes no
longer overall.

### Idle motor current no longer latches (`current.c`)
Found live on the bench: with change-driven push, `current_ma` in Home Assistant
stuck at the last driven value (observed steady at 2680 mA) once the gate
settled and the driver slept, instead of returning to 0. `s_isenseCounts` is
only ever written by the CONTINUOUS-mode ADC ISR and was never cleared on the
way out - the old polling bridge usually happened to see a near-zero batch on
its next 8 s poll, which hid it. `current_SetMode(CURRENT_MODE_IDLE)` now
zeroes it explicitly. Verified live before/after (`test-results-2026-09-28.md`).

### Command source (`cmd_src_t`)
`inputs.c` now stores an origin alongside every queued command. `gate_fsm.c`
reports it as `src=`: `boot`, `ha`, `remote`, `wired_button`, `auto_reverse`,
`obstruction_stop`, `fault`.

### Build
Flash 27 762 / 32 640 bytes (4 878 free). RAM unchanged at 10 010 / 18 432.

---

## 3. ESP32 bridge changes

* **All polling removed.** `MQTT_POLL_MS_IDLE`, `MQTT_POLL_MS_MOVING`,
  `MQTT_FULL_STATUS_MS` and the `CMD_SETTLE_MS` fast-poll window are gone.
  `mqtt_gate_task` is now purely a publisher, woken by `uart_rx_task` when a
  pushed line is parsed.
* `STATUS` is sent exactly twice: at boot and on every MQTT reconnect. Plus an
  11-minute (`MQTT_SANITY_POLL_MS`) stale-link backstop — two missed firmware
  heartbeats.
* **Beam edges publish inline from `uart_rx_task`**, not via a task hand-off.
  Its stack went 6144 → 8192 to cover the extra JSON. Gate-state changes still
  hand off, because their payload is much larger.
* All state topics remain **retained**, so HA restarts pick up the last values
  straight from the broker. The `beamact` event topic is deliberately **not**
  retained — a retained event replays on every HA restart and would re-trigger
  automations.
* **Legacy-firmware fallback (`s_gate_pushes`).** The gate's own `STATUS` reply
  identifies the build: the pushing one prints an `Uptime:` line. Until that is
  seen, the bridge polls at `MQTT_LEGACY_POLL_MS` (2 s). This keeps HA alive if
  the ESP32 is updated before the MSP430, or if an MSP430 reflash rolls back.

### New MQTT topics

| topic | retained | contents |
|---|---|---|
| `msp430gate/beam` | yes | `ON` / `OFF` |
| `msp430gate/beam/attr` | yes | `{"last_break_ms":…,"last_edge_uptime_ms":…,"breaks_last_10s":…,"chatter":…}` |
| `msp430gate/beamact` | **no** | `{"event_type":…,"pattern":…,"breaks":…,"break_ms":[…],"gap_ms":[…],"total_ms":…,"gate_state":…,"uptime_ms":…}` |
| `msp430gate/diag/beam_edges` | no | `{"edge":"blocked"\|"clear","uptime_ms":…,"dur_ms":…}` — one per raw edge, no HA entity |

`msp430gate/state` gained one field: `"cmd_src"`. Nothing was renamed or removed.

### New HA entities

| entity | type | notes |
|---|---|---|
| `binary_sensor.msp430_gate_controller_gate_photo_beam_blocked` | binary_sensor | `device_class: occupancy`, ON = blocked, JSON attributes topic |
| `event.msp430_gate_controller_gate_beam_activity` | event | `event_types: vehicle_pass, short_break, long_block` |
| `sensor.msp430_gate_controller_last_command_source` | sensor | `ha` / `remote` / `wired_button` / `auto_reverse` / `obstruction_stop` / `fault` / `boot` |

Entity ids follow the same naming pattern as the existing entities (device name
+ entity name). If HA assigns something different on your install, rename it in
the UI — the discovery `object_id` is unchanged in form from the existing ones.

**Kept unchanged:** cover, state sensor, both limits, moving, lockout, low
battery, supply voltage, motor current, stall count, cycle count, all last-move
metrics, bridge diagnostics, the four buttons, the existing
`sensor.msp430_gate_controller_gate_photo_beam` (Clear/Blocked — now fed from the
same real-time value), and MQTT availability (LWT/birth).

---

## 3a. Measured results

Flashed and measured on the live gate the same day — see
`test-results-2026-09-28.md` for the full tables. Headlines:

* HA command -> `moving:true`: **1362 ms before, mean 234 ms after** (6 runs,
  min 87 / max 372). The firmware pass itself is **0-4 ms**; the rest is the
  Wi-Fi/MQTT round trip at -69 dBm, proven by the fact that `OK`,
  `Gate: opening`, `EVT MOVE_START` and `EVT STATE` — all emitted in one
  control-loop pass — arrive together even on the slow runs.
* Bridge reboot -> correct retained state: **~2 s** (target 5 s).
* Progress lines while moving: **2.0 Hz**; 6/6 arrival messages and capture
  dumps intact.
* Idle UART: **silent**. 30 state publishes for 6 moves, all change-driven.
* First pushed event after a gate reboot: **10 ms**, one tick.

Everything needing a hand at the gate (remote, beam, car, safety regression) is
still outstanding — see the acceptance test plan.

## 4. Host-side tests

`tests/run_tests.ps1` — three suites, all passing:

1. **`test_beam.c`** compiles the *real* `beam.c` with stubs and drives it with
   the acceptance-test waveforms (200 ms wave, two waves 1 s apart, axle pair,
   body, car+trailer, 10 s long block, 20 ms glitch, chatter, rolling counter,
   window gate-state context).
2. **`test_esp_parsers.cpp`** compiles the bridge's parsers *sliced verbatim out
   of `src/main.cpp`* by `gen_esp_test.py`, so it cannot drift from shipping
   code. Checks field extraction, lockout text, retain flags, the chatter rate
   limit, JSON payload shape, and that the existing arrival/VERSION/STATUS paths
   still work.
3. **`test_wire_contract.py`** reads both firmwares and asserts that every `key=`
   the bridge parses is one the gate actually emits, that every emitted
   `event_type` is declared in HA discovery, and that no event body contains a
   `;` (the message terminator).

Two real bugs were caught by these and fixed before anything shipped: the chatter
flag stayed latched onto the next edge however much later it came, and the early
`long_block` inflated the follow-up event's break count to 2 for a single break.

---

## 5. Limitations and things the hardware cannot do

* **The beam is not power-gated in any state.** P8.6 is a plain input with an
  internal pull-up, sampled every tick in every gate state (`inputs_Poll()` runs
  unconditionally). There is nothing to enable and no current to pay. The
  observed Sep 18 event with the gate closed is consistent with that.
* **P8.6 has no pin-change interrupt** — on the MSP430F5338 only P1/P2 do. Beam
  edges are therefore sampled, not interrupt-driven. The 10 ms idle tick is what
  bounds edge latency and duration accuracy to ±10 ms. Going below that means
  moving the beam to a P1/P2 pin (a wiring change) or raising the tick rate
  further.
* **Existing fail-safe behaviour is unchanged and still not fail-safe:** with
  `PHOTOBEAM_PULL_UP=1` + `PHOTOBEAM_ACTIVE_HIGH=0`, a disconnected or unpowered
  sensor floats HIGH and reads "beam clear", so the gate will still close on a
  dead sensor. Out of scope here, but worth knowing given HA is about to use the
  beam as a "don't close" condition. Fixing it means an external ~4.7k pull-down
  at the P8.6 connector.
* **Beam thresholds are compile-time**, not MQTT `number` entities — see §2 for
  why (an nvconfig version bump wipes the gate's tuned parameters).
* **Flash the MSP430 first.** If the ESP32 is updated first it falls back to 2 s
  polling until it sees the new firmware, which works but is not quiet.
