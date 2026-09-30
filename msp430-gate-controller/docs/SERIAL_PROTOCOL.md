# GateController_DRV8262 — Serial Protocol

Reference for whatever parses this device's UART (the ESP32, and any AI helping
with it). Written to be machine-parseable: fixed framing, stable prefixes,
explicit grammar. Keep this file in sync with the firmware when serial output
changes.

- Firmware source of truth: `console.c` (framing, commands), `current.c`
  (`capture_DumpService`), `gate_fsm.c` (event lines), `main.c` (boot banner).
- Last protocol change: **2026-09-10** — added the `STATUS_IF_CHANGED` command;
  the capture dump now includes a final end-of-move sample. See Changelog at bottom.

---

## 1. Physical link

| Property | Value |
|---|---|
| Baud | 115200 |
| Format | 8N1 (8 data bits, no parity, 1 stop) |
| Flow control | none |
| Peripheral | MSP430F5338 UCA1 |

---

## 2. Framing — the one rule

**Every logical message the firmware emits ends with a line containing only `;`.**

```
<message body, one or more CRLF-terminated lines>
;\r\n
```

- The `;` line is the end-of-message marker. Buffer incoming lines; when you
  receive a line that is exactly `;`, the buffered lines are one complete
  message — parse them, then clear the buffer.
- **No message body ever contains a `;`.** A `;` only ever appears as the
  marker. (Firmware enforces this: descriptions/labels use `-` `,` `.` `/`
  instead.)
- Lines are `\r\n`-terminated. Do not rely on `\r` vs `\n` individually.
- A message may be a single line (`OK`) or many lines (`STATUS`, `LIST`, a
  capture dump). Either way there is exactly one `;` line, at the end.
- Empty/whitespace-only received command lines produce no output at all (no
  `;`). `TIME:` sync lines also produce no output.

### Optional timestamp prefix

Once the host has sent a `TIME:` sync (section 7), the firmware prepends
`[<unix_epoch_seconds>] ` to every message body line (not to the `;` marker
line). Strip a leading `^\[\d+\] ` before matching body text.

```
[1757280120] Gate: opening
;
```

---

## 3. Host → device: commands

Send one command per line, terminated by `\r`, `\n`, or `;` (all three
accepted). Case-insensitive. Max line length 40 chars (overflow →
`ERR LINE_TOO_LONG`).

| Command | Reply (before the `;` marker) |
|---|---|
| `STATUS` | multi-line status block (section 5) |
| `STATUS_IF_CHANGED` | the same status block, but **only if** a discrete field (state, the four digital inputs, last direction, driver sleep, lockout reason, stall count) changed since the last `STATUS` / `STATUS_IF_CHANGED`. If nothing changed: **no output at all** (no body, no `;`). Live current / VM are not part of the change test. A plain `STATUS` re-baselines it. |
| `LIST` | one `  <NAME> = <val> <units> (default <d>)` line per param |
| `LIST_DETAILS` | per param: `<NAME> (<units>)` then `    <one-line description>` |
| `GET_<NAME>` | bare integer, or `ERR UNKNOWN_PARAM` |
| `SET_<NAME> <int>` | `OK`, or `ERR BAD_VALUE` / `ERR UNKNOWN_PARAM` / `ERR OUT_OF_RANGE` / `ERR ORDERING (...)` |
| `SAVE` | `Saved.` |
| `DEFAULTS` | `Reset to defaults in RAM - send SAVE to persist.` |
| `VERSION` | `VERSION <version string>` (section 6) |
| `OPEN` / `CLOSE` / `TOGGLE` / `STOP` | `OK` (injects the operator command; the move itself emits async events) |
| `BSL` | `BSL: jumping to bootloader ...` then the app is gone (section 8) |
| `FWUPDATE` | `FWUPDATE: RAM updater armed - send the image now, app halting.` then raw binary mode (section 8) |
| `HELP` or `?` | the command list |
| anything else | `ERR UNKNOWN_COMMAND (try HELP)` |

`<NAME>` values come from `LIST`. All params are unsigned 16-bit integers.

Moves cannot be commanded except via `OPEN`/`CLOSE`/`TOGGLE`/`STOP` (or the
physical buttons / RF remote).

---

## 4. Device → device replies vs. async events

Two kinds of message arrive unsolicited-looking:

1. **Command replies** — emitted synchronously right after you send a command,
   in order. The next `;` after your command closes its reply.
2. **Async events** — emitted any time by the gate state machine / inputs. Each
   is its own `;`-terminated message. They can appear between your command and
   its reply if something physical happens at that instant; match on the body
   text, not on position.

---

## 5. Device → host: async event messages

All are single-line unless noted. `<n>` = unsigned integer.

### 5.0 Pushed `EVT` lines (firmware 2026-09-28 and later)

The device now **pushes** every change on its own; a host does **not** need to
poll. Each `EVT` line is a normal single-line message (own `;` terminator,
`[epoch]` prefix once time-synced), is emitted in the same control-loop pass as
the thing it reports, and is stable/machine-parseable — parse by `key=` token,
not by position. Unknown keys must be ignored so fields can be added later.

`t=` is the device's free-running **uptime in milliseconds** (`board_UptimeMs()`,
wraps at ~49.7 days). It is not wall-clock time; use the `[epoch]` prefix for
that.

```
EVT STATE st=<n> lo=<0|1> lc=<0|1> bm=<0|1> nf=<0|1> slp=<0|1> sc=<n>
          dir=<OPEN|CLOSE> ma=<n> mv=<n> lk=<n> dl=<0|1> src=<source> t=<n>
```
Emitted whenever any **discrete** field changes. One line, ~130 bytes — it
carries everything a `STATUS` block does except the human-readable text.

| key | meaning |
|---|---|
| `st` | `gate_state_t` index (same numbering as `State=` in `STATUS`) |
| `lo` / `lc` | OPEN (P7.5) / CLOSED (P7.6) limit, 1 = at that limit |
| `bm` | photo beam broken |
| `nf` | nFAULT asserted |
| `slp` | DRV8262 asleep |
| `sc` | stall count |
| `dir` | last drive direction |
| `ma` / `mv` | sensed motor current (mA) / VM rail (mV) |
| `lk` | `lockout_reason_t`: 0 none, 1 dual limit, 2 nFAULT persist, 3 repeat stall — only meaningful when `st=12` |
| `dl` | both limit inputs read active right now (the "will lock out" warning) |
| `src` | what caused the current situation — see the source list below |

The **photo beam is deliberately excluded** from the change test: beam edges get
their own, higher-priority line (below) so a wave of a hand does not drag a whole
state line onto the wire.

```
EVT BEAM b=<0|1> t=<n> dur=<n> n10=<n> [chat=1]
```
One line per debounced beam edge, emitted before anything else that pass.
`b=1` blocked, `b=0` clear. `dur` is the length of the break that just ended
(0 on a blocked edge). `n10` is the rolling count of breaks in the last 10 s.
There is **no latch and no minimum hold** — a 200 ms break reports as ~200 ms.
`chat=1` appears only while the beam is chattering (more than
`BEAM_CHATTER_EDGES_PER_S` edges in one second, e.g. sun glare, rain, a bug on
the lens); while it is set, edge lines are throttled to one per
`BEAM_CHATTER_MIN_GAP_MS`. The safety logic reads the debounced level directly
and is never affected by this throttle.

```
EVT BEAMACT ev=<type> pat=<pattern> n=<n> tot=<n> gs=<n> t=<n>
            brk=<d1,d2,...> gap=<g1,g2,...>
```
One line per classified beam-activity window. A window opens on the first
blocked edge and closes once the beam has been clear for `BEAM_AXLE_GAP_MAX_MS`.

- `ev` — `vehicle_pass`, `short_break` or `long_block`
- `pat` — `axle_pair` (low beam, two or more tyre breaks with a wheelbase-sized
  gap), `body` (high beam, one long break), or `none`
- `n` — number of breaks in the window (a trailer simply adds more)
- `tot` — total window duration in ms
- `gs` — `gate_state_t` **when the window opened**. A single beam cannot tell
  which way a car is travelling, so this is the context a host needs.
- `brk` / `gap` — comma-separated break durations and inter-break gaps, or `-`
  when empty. Capped at `BEAM_MAX_BREAKS` entries; `n` is still the true count.

`long_block` is published **twice**: once as soon as a break reaches
`BEAM_BODY_BREAK_MAX_MS` (something is standing in the beam *now*, `brk=-`), and
again when it finally clears, carrying the total duration. Both report `n=1`.

```
EVT MOVE_START dir=<OPEN|CLOSE> src=<source> st=<n> t=<n>
```
Emitted in the **same control-loop pass that energises the motor**, whatever
started the move. The `EVT STATE` line for that pass follows immediately and
carries `src=` too, so a host may simply use this as a nudge.

**`src` vocabulary** (`[a-z_]+`): `boot`, `ha` (any command arriving over the
console link — Home Assistant, MQTT or the web UI), `remote` (RF receiver,
P7.4), `wired_button` (OPEN / CLOSE / TOGGLE), `auto_reverse` (firmware-initiated
obstacle reversal), `obstruction_stop` (over-current stop while driving toward
OPEN), `fault` (stall, nFAULT, lockout, low battery).

**Heartbeat and status request.** With nothing happening the device prints one
full `STATUS` block every `STATUS_HEARTBEAT_MS` (5 min) and nothing else. That is
a backstop, not the main path: a host that has just (re)started should send
`STATUS` to resync immediately rather than wait for it.

**Detecting this firmware.** The `STATUS` block of a pushing build contains an
`Uptime:` line (see §5 STATUS reply). An older build does not — a host can use
that to fall back to polling.

### Boot (in order, 3–4 messages)
```
                                  (blank line)
GateController_DRV8262 ready.
;
VERSION GateController_DRV8262 <Mmm dd yyyy> <hh:mm:ss><build id>
;
*** TEST MODE: ... ***            (only in TEST_MODE builds)
;
```
Seeing the boot banner + `VERSION` again = the device reset (power cycle, or the
BOR at the end of a successful `FWUPDATE`). This is how you confirm which image
is running.

### Inputs
```
INPUT: REMOTE toggle asserted        (GATE_DEBUG builds only)
INPUT: TOGGLE button pressed         (GATE_DEBUG builds only)
```
These are off by default from 2026-09-28 — `EVT MOVE_START src=...` carries the
origin now, which is what they were for. Build with `-DGATE_DEBUG=1` to restore.

### Motion — normal
```
Gate: opening
Gate: closing
Gate: close rejected - photo beam broken
Gate: soft end-of-travel at <n> ms - duty -> <n>%
Drive current: ~<n> mA avg (t=<n> ms, VM <n> mV)   (every ~0.5 s while driving)
```
Arrival (ONE message, two lines):
```
Gate: reached OPEN limit - travel time <n> ms, avg <n> mA, max <n> mA
Peak ramp-up current: <n> mA
;
```
(`OPEN`/`CLOSED` accordingly.)

### Motion — obstacle / stop / fault
```
OBSTACLE detected while OPENING (over-current<optional " HARD">, <n> mA) - halted
OBSTACLE detected while CLOSING (over-current<optional " HARD">, <n> mA) - reversing toward OPEN
OBSTACLE detected while CLOSING (photo beam) - reversing toward OPEN
Gate: <source> while opening - immediate stop
Gate: hard over-current during reversal - stopped mid-travel
Gate: reversal complete, stopped mid-travel - awaiting command
Gate: reversal reached a limit
Gate: stall - MAX_DRIVE_DURATION exceeded, motor disabled (count=<n>)
Gate: retrying after stall (homing)
Gate: homing aborted by <source> - immediate stop
FAULT: <OPEN|CLOSED> limit still engaged <n> ms after starting to drive away - motor not moving, leads reversed, or stuck switch
DRV8262 nFAULT asserted
LOCKOUT: <n> consecutive stalls - power cycle required
```
`<source>` is one of: `OPEN button`, `CLOSE button`, `TOGGLE button/remote`,
`STOP command`, `command`.

### Battery
```
Battery low (<n> mV) - new moves blocked
Battery recovered (<n> mV) - resuming normal operation
```

### Lockout clear progress
```
LOCKOUT: fault gone and stable - cleared
  clear: fault gone - hold <n> ms more to auto-clear
  clear: fault condition still present
```

### STATUS reply (multi-line, one message)
```
Gate: <state text>
  OPEN   limit (Arm-Retracted, P7.5) = <0|1>   <- 1 here means gate is at the OPEN end
  CLOSED limit (Arm-Extended,  P7.6) = <0|1>   <- 1 here means gate is at the CLOSED end
  (limit inputs: internal pull-up, read as active-<HIGH|LOW> ...)
  photo beam broken = <0|1>   nFAULT = <0|1>
State=<int>  lastDir=<OPEN|CLOSE>
Current: <n> mA (<n> counts)  VM: <n> mV
Stall count: <n>  Driver asleep: <0|1>
Uptime: <n> ms  src=<source>  beam10s=<n>  lastbreak=<n> ms
[LOCKED OUT: <reason>]                 (only if locked out)
[Both limit switches read ACTIVE - will lock out]   (only if both limits active)
;
```

---

## 6. VERSION string

```
VERSION GateController_DRV8262 <__DATE__> <__TIME__><FW_BUILD_ID>
```
e.g. `VERSION GateController_DRV8262 Sep  8 2026 14:03:11 g1a2b3c4`

- `<__DATE__>` is C `__DATE__` form: `Mmm d yyyy` (day space-padded to width 2,
  so there can be two spaces: `Sep  8 2026`).
- `<FW_BUILD_ID>` is empty unless the build was done with
  `-DFW_BUILD_ID='" <id>"'` (leading space included in the macro).
- Printed at boot and in response to the `VERSION` command.

---

## 7. Time sync (host → device)

```
TIME:<unix_epoch_seconds>\r\n      (or ';'-terminated)
```
No reply, no `;`. After this the device free-runs a seconds counter and prefixes
message bodies with `[<epoch>] ` (section 2). Re-send periodically to correct
drift. `TIME:` with no digits is accepted and ignored.

---

## 8. Per-move capture dump

Emitted automatically once, at the end of every drive (open / close / home /
obstacle-reverse), if at least one sample was recorded. Paced against the UART
buffer, so it can take ~1 s and spans many lines but never drops bytes.

```
CAP_BEGIN <label> samples=<n> interval_ms=<ms> group=<g> fmt=t_ms,mA,mV
<record>~<record>~...~<record>          up to <g> records, joined by '~'
<record>~<record>~...
...
CAP_END
;
```

- `<label>` ∈ `OPENING` | `CLOSING` | `HOMING` (an obstacle reverse continues
  the trace under the label of the move it interrupted).
- `<record>` = `<t_ms>,<mA>,<mV>` — all unsigned integers. `t_ms` is
  `index * interval_ms` (evenly spaced, starts at 0). The **last** record is an
  extra sample taken at the instant the move ended (so the stall-into-the-limit
  peak is not cut off); its `t_ms` is nominal and may be up to one
  `interval_ms` early relative to the real sample instant.
- `mA` = sensed motor current (8 ms boxcar average at the sample instant).
  `mV` = motor supply rail (VM) at the sample instant.
- `samples` = total record count. Rows carry `group` records each except the
  last. Records are in time order.
- Parsing: between `CAP_BEGIN` and `CAP_END`, take only lines whose first
  character is a digit; split on `~`, then split each piece on `,`. Ignore any
  other line in that span (a concurrent async event can land there).
- Window covered = `samples * interval_ms`. Buffer is finite
  (`CAPTURE_SAMPLES`); a longer move is truncated (you get the start, which is
  the interesting part).

Example:
```
CAP_BEGIN CLOSING samples=23 interval_ms=25 group=10 fmt=t_ms,mA,mV
0,44,12180~25,71,12176~50,120,12170~75,410,12130~100,980,12010~125,2102,11840~150,3176,11700~175,2400,11760~200,1800,11820~225,1500,11860
250,1300,11890~275,1200,11900~300,1180,11905~325,1170,11908~350,1165,11910~375,1162,11911~400,1160,11912~425,1160,11912~450,1160,11913~475,1159,11913
500,1159,11913~525,1159,11913~550,1160,11913
CAP_END
;
```

---

## 9. FWUPDATE / BSL — raw binary, framing OFF

After the `FWUPDATE` reply line and a ~10 ms gap, the link leaves ASCII/`;`
framing entirely and speaks the raw self-flash protocol until the device
resets. Do not apply sections 2–5 during this window.

Wire protocol (ESP32 → MSP430, little-endian), from `console.c`:
- MSP430 emits `0xC2` (diag) then `0xCC` when the RAM updater is ready.
- Data frame: `0xA5, addrLo, addrHi, len(1..128), data[len], crcLo, crcHi`
  - reply `0x06` = written+verified, `0x15` = rejected (bad CRC / verify fail /
    out of range / non-ascending) → resend the frame.
  - CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection) over
    `addrLo..data`.
  - Frames ascending, never spanning a 512-byte segment; addresses in
    `0x8000..0xFFFF` only.
- Commit: `0x5B` → reply `0x06`, then software BOR into the new image.
- On timeout (host silent ~7 s) the MSP430 does a silent BOR back to the app.

After the BOR, normal framing resumes with the boot banner (section 5) — read
the `VERSION` line to confirm the new image.

`BSL` is similar: after `BSL: jumping to bootloader at 0x1000 - app UART goes
dark, BSL is now on P1.1/P1.2 @ 9600 8N1` the application is gone and the
on-chip ROM BSL owns the line at 9600 8N1.

---

## 10. Suggested parser loop (pseudocode)

```
buffer = []
for each line L (CRLF-split) from the UART:
    L = strip_trailing_CR_LF(L)
    if in_fw_update_mode: handle raw bytes; continue
    if L == ";":
        msg = buffer; buffer = []
        dispatch(msg)          # msg is a list of body lines, no ';'
        continue
    strip optional leading "[<digits>] " from L
    buffer.append(L)

dispatch(msg):
    first = msg[0]
    if first.startswith("CAP_BEGIN "): parse_capture(msg)      # msg[-1] is "CAP_END"
    elif first == "OK" or first == "Saved." : ack
    elif first.startswith("ERR "): error(first)
    elif first.startswith("VERSION "): version(first)
    elif first.startswith("Gate: ") or first.startswith("INPUT: ")
         or first.startswith("OBSTACLE ") or first.startswith("Drive current: ")
         or first.startswith("FAULT: ") or first.startswith("LOCKOUT: ")
         or first.startswith("Battery "): event(msg)
    elif first == "GateController_DRV8262 ready.": device_booted()
    elif first.startswith("Gate: ") and len(msg) > 1: arrival_or_status(msg)
    elif is_status_block(msg): status(msg)
    elif first matches /^\s+\w+ = \d+/: list_reply(msg)
    elif first matches /^-?\d+$/: get_reply(int(first))
    else: log_unrecognized(msg)
```

---

## Changelog

- **2026-09-28**
  - **The device now pushes state instead of waiting to be polled.** New
    single-line `EVT STATE`, `EVT BEAM`, `EVT BEAMACT` and `EVT MOVE_START`
    messages (§5.0), emitted in the same control-loop pass as the event they
    report. A host should stop polling `STATUS`/`STATUS_IF_CHANGED` and consume
    these instead; both commands still work unchanged.
  - `STATUS` is now also the **status request**: a host sends it at boot and on
    every reconnect and gets an immediate full block. The device additionally
    prints one unprompted `STATUS` every 5 minutes as a staleness backstop.
  - `STATUS` gained an `Uptime: <n> ms  src=... beam10s=... lastbreak=... ms`
    line. Its presence is how a host tells a pushing build from an older one.
  - Photo beam: reported per edge with real durations, no latch, ~10 ms
    debounce (was 30 ms), and classified into `vehicle_pass` / `short_break` /
    `long_block` on the device, which is where the precise timing lives.
  - Idle tick 50 ms -> 10 ms, and the console is now serviced *before* the FSM
    each tick, so a command is executed in the tick it arrives rather than the
    next one.
  - `INPUT: ...` lines moved behind `-DGATE_DEBUG=1` (off by default).
  - `Drive current:` progress lines gained elapsed time and VM.

- **2026-09-10**
  - Added `STATUS_IF_CHANGED` — a poll-friendly `STATUS` that stays silent while
    the discrete gate state is unchanged (live current / VM excluded from the
    test). Intended for a fast bridge poll; send a plain `STATUS` on a slow
    cadence to re-baseline and as a keepalive.
  - Per-move capture dump now appends one final sample captured at move-end, so
    the trace reaches the end-of-travel current peak. `samples=` counts it.
  - New tunable params (from `LIST`): `RAMP_START_DUTY` (%, launch duty for an
    open/close before the ramp) and `OC_GRACE_TIME` (ms, window from move start
    where both OC trips are ignored). `OC_BLANK_TIME` max raised to 20000 ms.

- **2026-09-08**
  - Added end-of-message framing: every message now ends with a lone `;` line.
    Multi-line replies (`STATUS`, `LIST`, `LIST_DETAILS`, arrival stats) and the
    capture dump emit exactly one `;` at the end.
  - Guaranteed no `;` in any message body (rewrote a handful of descriptions).
  - `;` is now also accepted as an inbound command-line terminator.
  - Capture dump: CSV records are grouped `<=CAPTURE_DUMP_GROUP` per line, joined
    by `~`; header gains `group=` and `fmt=`; the standalone `t_ms,mA,mV` header
    line was removed.
  - Added `VERSION` command + a `VERSION ...` line in the boot banner.
  - Capture `mV` column is now live during a move (VM interleaved into the ADC
    sequence).
