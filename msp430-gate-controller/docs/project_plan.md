# GateController_DRV8262 — Implementation Plan (Rev 3)

> **Historical design document.** This is the Rev 3 implementation plan the
> firmware was built from. The architecture still matches, but several numbers
> have moved since: the idle tick is now 10 ms (not 50 ms), the IPROPI sense
> resistance is 1650 ohm (two 3.3 k in parallel), and the over-current defaults
> and blanking time were re-tuned from real capture data. `gate_config.h` and
> `nvconfig.c` are the source of truth for current values.

Swinging-gate operator firmware for **MSP430F5338IPZ + DRV8262** (single H-bridge, PWM/IN-IN mode, one gate motor, 12 V battery supply).

---

## 1. Context

Rev 2 replaced the CSV log with live human-readable UART output and added a flash-persisted,
UART-configurable parameter system. This revision (Rev 3) responds to engineering review on four
points: **Rev 2's "just busy-poll, power doesn't matter" call was wrong** for a battery product
and is reversed here; the ADC sampling rate is legacy EVM baggage and is cut 25×; two UL 325 edge
cases were under-specified (opening-side obstacles, low battery) and are now fully defined; and
the stack/ISR discipline around the new console is now an explicit, checkable rule rather than an
implied one.

### 1.1 Changes applied in this revision

| # | Feedback | Resolution |
| --- | --- | --- |
| 1 | Dynamic sleep, not busy-poll | Two tick regimes: **1 ms / LPM0** while the motor is moving, ramping, or braking; **50 ms / LPM0** while at rest (idle, fault, or lockout). See §7.1 for why LPM0 (not LPM3) is used in both regimes, with quantified current estimates and an LPM3 stretch-goal path |
| 2 | 25 kHz ADC is EVM legacy | ADC trigger rate cut from 25 kHz → **1 kHz**; averaging window cut from 48 → **8 samples** (8 ms); PWM carrier itself is unchanged (still 25 kHz, inaudible) |
| 3a | Opening-side obstacle undefined | Now a mandatory, hardcoded path: over-current during `ST_OPENING` → immediate ramp-to-0 → optional brief backoff (`OPENING_OBSTACLE_BACKOFF_MS`, default 250 ms) → `ST_STOPPED_MIDTRAVEL`, always logged as a fault |
| 3b | Low-battery lockout missing | New `ST_LOW_BATTERY`: VM < 10.8 V for 500 ms blocks **new** moves (does not abort one in progress); recovers at 11.2 V (hysteresis) |
| 3c | Hard OC ceiling must survive blanking | Two-tier over-current: existing soft trip (~3.5 A) stays blanked for 300 ms; new **hard trip (~4.5 A) is always active**, including during blanking |
| 4 | Stack/ISR/RX discipline | Formalized as explicit rules in §9.4: one static `dbg_Printf` scratch buffer, **no print/console call is ever made from an ISR**, and the RX line buffer has a hard length limit with an explicit `ERR LINE_TOO_LONG` reply on overflow (never silent corruption) |

---

## 2. Current code — what is reused vs. replaced

Unchanged from Rev 2 (see the mapping table there) with one addition: `board.c`'s Timer_A2 setup
and `current.c`'s Timer_A0/ADC12 setup are no longer static one-time configurations — both are now
switched at runtime between an active profile and an idle profile (§7.1, §7.2).

| Existing in `main.c` | Disposition |
| --- | --- |
| `clock_Init()` | **Reuse** → `board.c` |
| `gpio_Init()` | **Rewrite** → `board.c` (pull-up inputs incl. fail-safe photo beam; RST/NMI untouched — plain reset) |
| `watchdog_Init()` | **Reuse** → `board.c` |
| `ta2_Init()` + `TIMER2_A0_VECTOR` | **Rewrite**: now dynamic — `board_SetTickRate(TICK_ACTIVE\|TICK_IDLE)` reprograms the period; CPU still sleeps in LPM0 after each tick either way (§7.1) |
| `pwmTimerConfig()` | **Reuse, simplified** → `drv8262.c`; halted (`MC_0`) whenever the driver is asleep |
| `adcTimerConfig()` / `adc12_Init()` / `ADC12_VECTOR` / `UpdateipropiReading()` | **Rewrite** → `current.c`: trigger rate 25 kHz→1 kHz, averaging 48→8 samples, plus an idle on-demand single-shot mode |
| `RMPCNTL_t` / ramp functions | **Replaced** by the single Q8 slew-rate accumulator from Rev 2 §7.1 (unchanged in this revision) |
| `drv8262_init()` | **Reuse** → `drv8262.c`; DAC code per Rev 2 §7.2 |
| `uartInit()` / `uartTxByte()` / `USCI_A1_VECTOR` | **Rewrite** → `uart.c` (115200, rings, ISR — **ISR touches only raw bytes, never formatting/printing**, §9.4) |
| `NMI_ISR` | **Deleted** — pin 96 is a plain hardware reset |
| `Trap_ISR` | **Reuse** → `board.c` |

---

## 3. Hardware constraints & corrected pin map

Unchanged from Rev 2 — see that section for the full pin table (photo beam confirmed on P8.6,
active-high-broken with a fail-safe pull-up; limit switches on P7.5/P7.6; RST/NMI restored to a
plain hardware reset). No pin assignments changed in this revision.

---

## 4. Target architecture — modules

Same 10 translation units + 2 headers + `main.c` as Rev 2. Three modules gain new
responsibilities; nothing is added or removed from the list.

### 4.3 `board.c/.h` — BSP (updated)
Adds **`board_SetTickRate(tick_rate_t rate)`** — `TICK_ACTIVE` reprograms Timer_A2 for a 1 ms
period (SMCLK/8, `TACCR0 = 2499`); `TICK_IDLE` reprograms it for 50 ms (SMCLK/64 via the extra
timer-divider stage, `TACCR0 = 15624`). Both stay on **SMCLK** (not ACLK) so the UART keeps
working at either rate (§7.1). Adds **`board_SleepUntilTick(void)`** — the single place that
issues `__bis_SR_register(LPM0_bits | GIE)`; called once per super-loop pass instead of never
sleeping (reverses the Rev 2 "no sleep" call).

### 4.4 `drv8262.c/.h` — driver (updated)
`drv8262_Sleep(true)` now also halts Timer_B0 (`MC_0`) — no point toggling PWM outputs while the
driver itself is powered down. `drv8262_Sleep(false)` re-arms it before the next move.

### 4.5 `current.c/.h` — ADC owner (updated)
Adds **`current_SetMode(CURRENT_MODE_CONTINUOUS | CURRENT_MODE_IDLE)`**:
- `CONTINUOUS` (active motion): Timer_A0 triggers ADC12 at **1 kHz** (was 25 kHz); 8-sample
  (was 48) swept-phase average, refreshed every 8 ms; feeds `metrics_*` and both OC thresholds.
- `IDLE` (at rest): Timer_A0 halted; a single software-triggered (`ADC12SC`) VM conversion is
  taken roughly once per second (driven off the 50 ms idle tick, every 20th tick) for the
  low-battery check (§6.3) — no continuous sampling needed since the motor isn't drawing current.
Adds the second, always-armed threshold: `current_HardOcTrip()` returns true whenever the
(still 8 ms-averaged) current exceeds `OC_HARD_LIMIT_COUNTS`, checked by `gate_fsm` unconditionally
— including while `tBlank > 0` — unlike the existing soft `OC_TRIP_COUNTS` check.

### 4.8 `console.c/.h` — debug output + command line (hardened, §9.4)
No new public API, but three now-explicit invariants (checked in code review, not just intent):
1. `dbg_Printf` formats into **one static** scratch buffer (no per-call stack allocation).
2. **No ISR anywhere in the project calls `dbg_Print`, `dbg_Printf`, or any `console_*`
   function.** ISRs only push raw bytes/counts and set flags.
3. The RX line buffer (`CONSOLE_RX_LINE_MAX`, 40 bytes) is a hard cap; a line that doesn't
   terminate within the cap sets an overflow flag, and on eventually seeing `\r`/`\n` the console
   discards the line and replies `ERR LINE_TOO_LONG` rather than acting on truncated input.

Everything else in §4 (types, config, inputs, uart, nvconfig, gate_fsm, main) is unchanged from
Rev 2 except `main.c`'s loop, which now sleeps (§4.11 below).

### 4.11 `main.c` — super-loop (updated: sleeps again)
```c
int main(void) {
    board_Init(); uart_Init(); nvconfig_Init(); drv8262_Init();
    current_Init(); inputs_Init();
    inputs_Poll();
    gate_fsm_Init(input_Level(IN_LIMIT_OPEN), input_Level(IN_LIMIT_CLOSED));
    dbg_Print("Gate controller ready.\r\n");
    __bis_SR_register(GIE);
    for (;;) {
        wdt_Kick();
        if (SamplesCollected) current_Service();
        if (timerUpdate)      drv8262_Service();
        if (tickFlag) {
            tickFlag = 0;
            inputs_Poll();
            gate_fsm_Tick();      /* may call board_SetTickRate/current_SetMode/drv8262_Sleep */
            console_Service();
        }
        board_SleepUntilTick();   /* LPM0 + GIE; wakes on the next tick or any other IRQ */
    }
}
```

---

## 5. Header comment (add these two bullets to the Rev 2 block in `gate_fsm.c`)

```c
 *   * LOW BATTERY: if supply voltage stays below 10.8 V for 500 ms while at rest,
 *     new moves are blocked until it recovers above 11.2 V (hysteresis) — a move
 *     already in progress is not aborted by this check.
 *   * TWO-TIER OVER-CURRENT: a soft trip (~3.5 A) is blanked for 300 ms after
 *     start to ride through inrush; a hard trip (~4.5 A) is always active, even
 *     during that blanking window, as a fast backstop below the DRV8262's own
 *     ~4.7 A hardware current limit.
 *   * POWER: the MCU runs a 1 ms control tick (LPM0 sleep between ticks) only
 *     while the motor is moving, ramping, or braking; at rest it drops to a
 *     50 ms tick to save the 12 V battery.
```
(The rest of the Rev 2 header comment is unchanged; keep "Forward = OPEN, reverse = CLOSE" etc.)

---

## 6. Gate state machine

Same 13-state shape as Rev 2 plus one new state. Three transition paths are hardened per the
feedback; everything else (normal cycle, dual-limit lockout, driver-fault handling, stall
timeout, lockout-clear-by-any-button) is unchanged from Rev 2 §6.

### 6.1 New state: `ST_LOW_BATTERY`

| State | Meaning | Motor |
| --- | --- | --- |
| `ST_LOW_BATTERY` | VM confirmed below cutoff while at rest | coast, `drv8262_Sleep(true)` |

Entered from any **resting** state (`ST_IDLE_OPEN`, `ST_IDLE_CLOSED`, `ST_STOPPED_MIDTRAVEL`,
`ST_UNKNOWN`) when `vmMv < VM_LOW_CUTOFF_MV` (10800) has held for `VM_LOW_CONFIRM_MS` (500 ms) —
sampled via the idle-mode ~1 Hz VM read (§4.5). **Not** checked while a moving state is active —
a move already underway is allowed to finish; the check only ever gates the *start* of a new one.

Transitions:
- `vmMv >= VM_LOW_RECOVER_MV` (11200) held stably → back to the correct resting state per current
  limit-switch levels (`ST_IDLE_OPEN`/`ST_IDLE_CLOSED`/`ST_UNKNOWN`); `log "Battery recovered
  (Vmm mV) — resuming normal operation"`.
- any `CMD_*` while still low → rejected, `log CMD_REJECT(LOW_BATTERY)`.
- `dualLimit` / `nFault` still override and take priority (safety faults outrank a battery
  warning) → `ST_LOCKOUT` / `ST_DRIVER_FAULT` as usual.
- D12 uses a **slow blink** here, distinct from lockout's fast blink and stall's solid-on, so the
  three fault classes are distinguishable at a glance:

  | D12 pattern | Meaning |
  | --- | --- |
  | solid on | `ST_STALL_FAULT` |
  | fast blink | `ST_LOCKOUT` |
  | slow blink | `ST_LOW_BATTERY` |

### 6.2 Opening-side obstacle — now mandatory and explicit

`ST_OPENING`, on `ocTrip` (soft or hard, see §6.3) or (if ever enabled) `beamBroken`:
1. **Immediately** command `drv8262_SetDuty(0)` — ramp-to-zero starts this tick, no delay.
2. `log "OBSTACLE detected while OPENING (over-current, N mA) — halted"`.
3. → `ST_SOFT_STOP(next = ST_STOPPED_MIDTRAVEL, backoff = OPENING_OBSTACLE_BACKOFF_MS)`.
4. Once duty reaches 0: if `OPENING_OBSTACLE_BACKOFF_MS > 0`, drive briefly in `DIR_CLOSE` at
   `HOMING_DUTY_PCT` for that duration (a short, gentle nudge to relieve mechanical load — this is
   *not* the full 0.75 s UL 325 reversal used on the closing side; opening-side obstacles are
   typically the gate hitting a physical stop, not a person being pinched) — then coast.
5. → `ST_STOPPED_MIDTRAVEL`; requires a new command to resume (no auto-retry).

This replaces Rev 2's `OPENING_OBSTACLE_REVERSE_MS` toggle — the halt-and-log behavior is now
unconditional; only the small backoff duration is configurable (`SET_OPENING_BACKOFF <ms>`,
default 250, 0 = pure hard stop).

**`ST_HOMING` obstacle handling is now direction-aware** (previously uniform in Rev 2): if
`homeDir == DIR_OPEN`, an obstacle uses the same halt-and-backoff path as `ST_OPENING`; if
`homeDir == DIR_CLOSE`, it uses the full reverse-toward-open path as `ST_CLOSING` — homing was
incorrectly treating both directions the same in Rev 2.

### 6.3 Two-tier over-current (feeds every obstacle/stall check)

```
ocSoft = (isenseCounts > OC_TRIP_COUNTS)      && (tBlank == 0)   /* ~3.5 A, blanked   */
ocHard = (isenseCounts > OC_HARD_LIMIT_COUNTS)                    /* ~4.5 A, unblanked */
ocTrip = ocSoft || ocHard
```
`ocHard` is evaluated every tick in every moving state regardless of `tBlank`, so a locked-rotor
or short-circuit condition at power-on is caught well inside the 300 ms blanking window instead
of waiting it out. Both feed the same `ocTrip` signal already used throughout `ST_OPENING` /
`ST_CLOSING` / `ST_HOMING` — the log line distinguishes which tier fired
(`"over-current (soft)"` vs `"over-current (HARD)"`) since a hard trip is a stronger signal of a
real fault (locked rotor, short) worth surfacing distinctly for diagnosis.

### 6.4 Full state table (updated)

| State | Meaning | Motor | Tick rate |
| --- | --- | --- | --- |
| `ST_INIT` | first tick after peripheral init | coast | active (brief) |
| `ST_IDLE_OPEN` / `ST_IDLE_CLOSED` | at rest at a limit | brake 1 s (active tick) → coast + sleep (idle tick) | active during brake, then idle |
| `ST_OPENING` / `ST_CLOSING` | driving | PWM `RUN_DUTY_PCT` | active |
| `ST_SOFT_STOP` | ramp → 0 | ramp down | active |
| `ST_STOPPED_MIDTRAVEL` | stopped between limits | coast + sleep | idle |
| `ST_OBSTACLE_REVERSE` | UL 325 reversal (closing-side only) | PWM `REVERSE_DUTY_PCT` | active |
| `ST_HOMING` | slow move, direction-aware obstacle handling | PWM `HOMING_DUTY_PCT` | active |
| `ST_UNKNOWN` | neither limit; awaiting command | coast + sleep | idle |
| `ST_STALL_FAULT` | timeout / thrash | coast + sleep | idle |
| `ST_DRIVER_FAULT` | nFAULT asserted (awake) | coast; brief active bursts to retry | idle (active only during a retry pulse) |
| `ST_LOCKOUT` | dual-limit / persistent nFAULT / 5 stalls | coast + sleep | idle |
| **`ST_LOW_BATTERY`** | VM below cutoff at rest | coast + sleep | idle |

### 6.5 Safety requirement → state map (updated)

| # | Requirement | Where it lives |
| --- | --- | --- |
| 1 | Obstacle reversal (UL 325, closing) | `ST_OBSTACLE_REVERSE` from `ST_CLOSING`/homing-toward-close |
| 1b | **Obstacle halt (opening)** | `ST_OPENING`/homing-toward-open → immediate ramp-to-0 + optional 250 ms backoff → `ST_STOPPED_MIDTRAVEL` (§6.2) |
| 2 | Max-drive-duration / stall timeout | unchanged from Rev 2 |
| 3 | Mid-travel power-up / unknown | unchanged |
| 4 | Dual limit-switch fault | unchanged |
| 5 | Soft start / soft stop | unchanged |
| 6 | DRV8262 hardware fault line | unchanged |
| 7 | Debounce + inrush constants | unchanged, plus: |
| 7b | **Hard OC ceiling survives blanking** | `ocHard` check, always active (§6.3) |
| 8 | **Low-battery lockout** | `ST_LOW_BATTERY` (§6.1) |
| 9 | **Dynamic MCU power** | tick-rate switching (§7.1) |

---

## 7. Timing/scheduling & current-sense math

### 7.1 Dynamic tick / sleep architecture (reverses Rev 2's "never sleep" call)

**Why LPM0 in both regimes, not LPM3 for idle:** LPM3 stops SMCLK/DCO entirely, which is also the
UART's bit-rate clock. The console must stay usable **at any time** — including while the gate is
sitting idle at the gate post, laptop plugged in — so SMCLK (and therefore the DCO) has to keep
running. LPM0 stops only the CPU/MCLK while leaving SMCLK/ACLK/peripherals live, which is
sufficient for the win being asked for: the CPU load isn't "always executing," it's "asleep
between ticks," and slowing the tick from continuous (Rev 2) to 50 ms already removes the
overwhelming majority of the wasted current. A true LPM3 idle mode is noted as an optional
stretch goal below with its cost made explicit, rather than silently adopted despite the UART
conflict.

| Regime | Timer_A2 config | CPU sleep | Used during |
| --- | --- | --- | --- |
| `TICK_ACTIVE` | SMCLK/8, `TACCR0=2499` → **1 ms** | LPM0 between ticks | `ST_OPENING/CLOSING/SOFT_STOP/OBSTACLE_REVERSE/HOMING`, and the post-arrival brake sub-phase of the idle states |
| `TICK_IDLE` | SMCLK/64 (extra divider stage — verify exact bit field against the family user's guide during implementation, §12), `TACCR0=15624` → **50 ms** | LPM0 between ticks | `ST_IDLE_OPEN/CLOSED` (post-brake), `ST_STOPPED_MIDTRAVEL`, `ST_UNKNOWN`, `ST_STALL_FAULT`, `ST_DRIVER_FAULT` (between retries), `ST_LOCKOUT`, `ST_LOW_BATTERY` |

`gate_fsm` calls `board_SetTickRate()` / `current_SetMode()` / `drv8262_Sleep()` together on the
relevant state transitions — entering any moving state (or the brake sub-phase) switches to
`TICK_ACTIVE` + `CURRENT_MODE_CONTINUOUS` + driver awake; completing the brake and settling into
a resting state switches to `TICK_IDLE` + `CURRENT_MODE_IDLE` + `drv8262_Sleep(true)`.

**Quantified win (order-of-magnitude — confirm exact figures against the F5338 datasheet, §12):**

| Scheme | CPU state at rest | Est. average current | Est. daily draw (23.9 h idle + ~0.5 min moves) |
| --- | --- | --- | --- |
| Rev 2 (busy-poll, never sleeps) | active @ ~20 MHz continuously | ~4–6 mA | ~100–140 mAh/day (matches the ~120 mAh/day figure in the feedback) |
| **Rev 3 (this revision): 50 ms tick, LPM0** | asleep >99.9% of the time; SMCLK/DCO still running for UART | ~0.3–0.5 mA average | **~7–12 mAh/day** (≈10–15× better) |
| Stretch goal: 50 ms tick, **LPM3** | asleep, SMCLK/DCO also stopped | ~1–2 µA average | ~0.03–0.05 mAh/day (another ~10×) — **cost:** UART/console is unusable while asleep; would need a wake-and-resync scheme (e.g. requiring a button press before typing commands) that is not designed here |

**Ramp/PWM servicing** (`drv8262_Service`, Rev 2 §7.1) is unaffected by which tick regime is
active — it's only ever invoked while `TICK_ACTIVE` is selected, since that's the only time the
motor is being driven.

### 7.2 ADC sampling rate (cut 25×, feedback #2)

- Timer_A0 ADC12 trigger: **25 kHz → 1 kHz** (`TASSEL_2 | ID_3`, `TACCR0 = 2499`, i.e. the same
  divide-by-8 SMCLK tap already used elsewhere — one less clock configuration to keep track of).
- Averaging window: **48 → 8 samples** (8 ms), still swept across the (unchanged, 25 kHz) PWM
  carrier phase to avoid aliasing at a fixed point in the switching cycle — the sweep step just
  advances once per 1 kHz trigger instead of once per 25 kHz trigger, and still fully decorrelates
  over an 8 ms/8-sample window.
- This is a **25× reduction in `ADC12_VECTOR` interrupts/sec** (25000/s → 1000/s), directly
  addressing the ISR-overhead concern, with no DMA needed at this rate — noted as an available
  future optimization (MSP430F5xx has a DMA controller) if profiling ever shows 1 kHz is still
  too much overhead, which is not expected.
- PWM carrier frequency is **unchanged** (still 25 kHz) — the feedback was specifically about
  ADC sampling, and 25 kHz PWM is standard practice (inaudible, good ripple).
- The hard-OC check (§6.3) uses this same 8 ms-averaged value, not a raw per-sample check in the
  ISR — 8 ms of extra latency vs. an ISR-level check is a good trade against keeping all
  formatting/decision logic out of interrupt context (§9.4), and is still far faster than the
  300 ms blanking window it's meant to preempt. Flagged in §12 as revisit-if-needed only if bench
  testing shows a given motor's locked-rotor current rises faster than 8 ms allows for.
- Idle mode (§4.5) doesn't use this trigger at all — a single on-demand `ADC12SC` conversion
  once/second is enough for the low-battery check.

### 7.3 Current-sense math

Unchanged from Rev 2 (`R_IPROPI = 3.3 kΩ`, `A_IPROPI = 212 µA/A`, `I_motor(mA) ≈ counts × 1.152`
assuming `ADC_VREF_MV = 3300` — see §12 to confirm the actual `VeREF+` wiring). New: the second
threshold below.

### 7.4 Software timers (all in milliseconds, `TICK_MS` no longer a single constant — see §7.1)

| Timer | Default | Purpose |
| --- | --- | --- |
| `OC_BLANK_MS` | 300 | inrush blanking for the **soft** OC trip only |
| `MAX_DRIVE_DURATION` | 13000 | stall watchdog, opening/closing |
| `HOMING_TIMEOUT_MS` | 30000 | stall watchdog, homing |
| `OBSTACLE_REVERSE_MS` | 750 | UL 325 reverse duration (closing-side obstacles) |
| **`OPENING_OBSTACLE_BACKOFF_MS`** | **250** | brief nudge after an opening-side obstacle halt (0 = disable) |
| `POST_ARRIVAL_BRAKE_MS` | 1000 | brake pulse on arrival, before sleeping the driver |
| `DEADTIME_DIR_CHANGE_MS` | 150 | coast between direction reversals |
| `NFAULT_RETRY_GAP_MS` | 50 | spacing between fault-clear attempts |
| `INPUT_DEBOUNCE_MS` | 30 | buttons + limit switches + photo beam |
| `LOCKOUT_CLEAR_STABLE_MS` | 2000 | fault must be gone this long before a button clears lockout |
| **`VM_LOW_CONFIRM_MS`** | **500** | VM must stay below cutoff this long before `ST_LOW_BATTERY` |
| `HEARTBEAT_MS` | 500 | D6 ≈1 Hz (only while `TICK_ACTIVE`; at 50 ms idle tick, toggle every 10th tick instead) |
| `LOG_PERIODIC_MS` | 1000 | VM print cadence while moving; also the idle VM sample cadence (§4.5) |

---

## 8. Runtime-tunable parameters (`nvconfig`) — additions

All Rev 2 parameters (§8 there) are unchanged. New in Rev 3:

| Console name | Meaning | Units | Default |
| --- | --- | --- | --- |
| `OC_HARD_LIMIT` | always-on hard over-current trip (survives blanking) | mA | 4500 (≈3906 counts) |
| `OPENING_BACKOFF` | brief nudge after an opening-side obstacle halt | ms | 250 |
| `VM_LOW_CUTOFF` | battery voltage below which new moves are blocked | mV | 10800 |
| `VM_LOW_RECOVER` | battery voltage above which `ST_LOW_BATTERY` clears | mV | 11200 |
| `VM_LOW_CONFIRM_TIME` | how long VM must stay low before entering `ST_LOW_BATTERY` | ms | 500 |

`OC_TRIP_CURRENT` (Rev 2, the soft/blanked trip, default 3500 mA) is unchanged and remains
strictly below `OC_HARD_LIMIT` — the console should reject a `SET_OC_TRIP_CURRENT` or
`SET_OC_HARD_LIMIT` that would invert this ordering (min/max validation in the parameter table,
§9 of Rev 2, now cross-checks these two against each other rather than just a fixed range).

---

## 9. Remaining open items

Carried forward from Rev 2 §12, plus:
- Confirm the F5338's LPM0 current draw at the operating DCO/SMCLK frequency from the datasheet,
  to replace the order-of-magnitude estimate in §7.1 with a real number.
- Confirm the exact Timer_A extra-divider register/bit field (for the SMCLK/64 idle-tick divide)
  against the MSP430 5xx/6xx family user's guide during implementation.
- Battery chemistry: the 10.8 V / 11.2 V thresholds assume a lead-acid-style discharge curve; if
  the pack is a different chemistry (e.g. LiFePO4), these values should be revisited.
- Bench-confirm the 8 ms-averaged hard-OC check reacts fast enough for the actual motor's
  locked-rotor current rise time; move to an ISR-level raw-sample check only if it doesn't
  (keeping the "no printing from ISRs" rule intact either way — that check would just set a flag).
- (Unchanged from Rev 2) spin-direction confirmation, `VeREF+` wiring confirmation, OC
  calibration, flash segment size/layout, R5/R8/R13/R16 depopulation check.

---

## 10. Risks — additions

All Rev 2 risks (§13 there) still apply. New:
- **UART availability while idle is a deliberate trade, not an oversight** — LPM0 (not LPM3) was
  chosen specifically to keep the console live at all times; if battery life ever needs to beat
  the ~7–12 mAh/day estimate, moving to LPM3 requires designing a wake-and-resync scheme for the
  UART first (§7.1) — don't flip to LPM3 without also solving that.
- **Tick-rate switching must not race the ramp.** `board_SetTickRate(TICK_IDLE)` should only be
  issued after `drv8262_DutyIsZero()` and the brake has completed — switching to a 50 ms tick
  mid-ramp would make the soft-stop far coarser than intended.
- **Idle-mode VM sampling cadence and `ST_LOW_BATTERY`'s 500 ms confirm window interact** — at a
  1 Hz idle VM sample rate, confirming "500 ms below cutoff" only has ~1 sample of margin; if
  bench testing shows this is too coarse/noisy, either sample VM a bit more often while idle
  (still cheap since the CPU is awake every 50 ms anyway) or lengthen `VM_LOW_CONFIRM_TIME`.
- **Two OC thresholds must stay ordered** (`OC_TRIP_CURRENT < OC_HARD_LIMIT < ~4.7 A hardware
  backstop`) — enforce in `nvconfig`'s validation, not just by convention (§8).

---

## 11. Implementation order — additions

Insert after Rev 2 step 7 (`drv8262.c` bring-up):
- **7a. `board.c` tick-rate switching** — implement and bench-verify `board_SetTickRate()` in
  isolation (toggle it manually from a console command during bring-up, confirm both periods with
  a scope on a spare GPIO toggled in the tick ISR) before wiring it into `gate_fsm`.
- **7b. `current.c` idle mode** — verify the on-demand VM conversion works correctly interleaved
  with the continuous IPROPI mode, and that switching modes doesn't desync `Adcresults`.

Insert after Rev 2 step 10 (`gate_fsm.c` states):
- **10a.** Add `ST_LOW_BATTERY` and the direction-aware homing obstacle path; bench-test with a
  bench supply lowered below 10.8 V and confirm new moves are blocked but an in-progress move
  (started just before lowering the supply) is allowed to finish.
- **10b.** Bench-test the opening-side obstacle path (block the arm while opening) and confirm
  the halt + optional backoff + `STOPPED_MIDTRAVEL` sequence, distinct from the closing-side full
  reversal.

---

## 12. Verification — additions

All Rev 2 bench tests (§15 there) still apply, with the "no sleep" assumption removed. New tests:

| Scenario | Stimulus | Pass criteria |
| --- | --- | --- |
| Idle current draw | Let the gate sit idle; measure MCU/board supply current | Drops to the estimated LPM0-idle range (§7.1) within one tick after settling; console still responds to a command at any time |
| Tick-rate switch | Command a move from idle, then let it complete | Scope/GPIO toggle confirms 50 ms→1 ms on move start, 1 ms→50 ms after the post-arrival brake completes |
| ADC rate | Scope the ADC12 trigger or count ISR entries over 1 s while moving | ≈1000 conversions/sec, not 25000 |
| Opening obstacle | Block the arm while opening | Immediate ramp-to-0, "halted" log line (not "reversing"), optional 250 ms backoff, settles in `STOPPED_MIDTRAVEL` |
| Homing obstacle, open direction | Block the arm during homing-toward-open | Same halt+backoff path as opening, not a full reversal |
| Homing obstacle, close direction | Block the arm during homing-toward-close | Full reverse-toward-open path, as closing |
| Hard OC survives blanking | Force >4.5 A within the first 300 ms after a start | Trips immediately (does not wait for `OC_BLANK_MS` to expire) — soft trip alone would not fire yet |
| Low battery blocks new moves | Lower the bench supply below 10.8 V for >500 ms, then command a move | Move is rejected (`CMD_REJECT(LOW_BATTERY)`), D12 slow-blinks |
| Low battery doesn't abort a move | Start a move, then lower the supply below 10.8 V mid-travel | The move completes normally; `ST_LOW_BATTERY` is only entered afterward, from the resting state |
| Battery recovery hysteresis | Raise the supply back above 11.2 V | `ST_LOW_BATTERY` clears to the correct resting state; commands work again |
| Console under load | Send `LIST`/`SET_*` while the gate is idle (50 ms tick) and while it's moving (1 ms tick) | Responds promptly in both regimes — confirms the "UART always available" design goal |
| RX overflow | Send a line longer than `CONSOLE_RX_LINE_MAX` without a terminator, then terminate it | `ERR LINE_TOO_LONG`, no corruption, next command works normally |
| ISR discipline (code review, not bench) | Grep the codebase for `dbg_Print`/`console_` calls inside any `#pragma vector` function | Zero matches |
