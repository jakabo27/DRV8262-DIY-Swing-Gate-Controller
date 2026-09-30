# Home Assistant via MQTT

> **Design notes from the first MQTT phase.** The topic/entity design and the
> "5 concerns" review below are still accurate, but the *polling* described in
> "How status gets to MQTT" was replaced on 2026-09-28: the MSP430 now pushes
> `EVT` lines the moment anything changes and the bridge publishes straight from
> the parse (HA latency went from ~1.4 s to ~0.23 s). For the current topic and
> entity list see [`../../home-assistant/README.md`](../../home-assistant/README.md);
> for the change itself see [`../../docs/realtime-events-2026-09-28.md`](../../docs/realtime-events-2026-09-28.md).

**Phase 1: IMPLEMENTED** (in `src/main.cpp`, section "Home Assistant / MQTT bridge").
Phase 2 items below are still just ideas.

## What it does

The flasher connects to the Mosquitto broker and:

- **publishes gate status** — FSM state (+ decoded name), limit switches, photo
  beam, nFAULT, motor current, VM rail, stall count, driver-asleep, last
  direction, lockout reason — as one retained JSON doc on `msp430gate/state`;
- **accepts `OPEN` / `CLOSE` / `TOGGLE` / `STOP`** on `msp430gate/cmd`;
- is **auto-discovered by Home Assistant** as one device: a **gate cover** plus
  sensors/binary_sensors.

## Config (compile-time `#define` in main.cpp)

| macro | value |
|---|---|
| `MQTT_URI` | `mqtt://<broker-ip>:1883` (set in `src/secrets.h`) |
| `MQTT_USERNAME` | *(your broker user, in `src/secrets.h`)* |
| `MQTT_PASSWORD` | *(your broker password, in `src/secrets.h`)* |
| `MQTT_BASE` | `msp430gate` |
| `MQTT_DISC_PREFIX` | `homeassistant` |
| `MQTT_POLL_MS_IDLE` / `_MOVING` | 8000 / 1000 |
| `MQTT_HEARTBEAT_MS` | 30000 |

The `mqtt` component is not in ESP-IDF 6.x core — it's pulled as the managed
component `espressif/mqtt` via `src/idf_component.yml` (needs internet on the
first build, then cached in `managed_components/`).

## How status gets to MQTT (single-reader design)

`uart_rx_task` stays the **only** reader of `UART_PORT`. It now also assembles
whole messages (the MSP430 firmware terminates each reply with **`;`**; a
content-complete `State=…Driver asleep:` block and a 1200-byte cap are fallbacks)
and hands each to `gate_rx_message()`, which:

- parses a STATUS block → updates `g_gate_status` (under `s_status_mux`) → gives
  `s_status_sem` and nudges `mqtt_gate_task`;
- or, on a fault keyword (`OBSTACLE`, `LOCKED OUT`, `stall`, `homing`, …), just
  nudges `mqtt_gate_task` to re-poll now.

`mqtt_gate_task`:

1. waits for `WIFI_CONNECTED_BIT` (broker unreachable in fallback-AP-only mode),
   derives `s_dev_id = "msp430gate_<mac6>"`, starts the client (LWT
   `msp430gate/availability=offline`, `buffer.size=2048`);
2. every cycle: if connected & not `s_bsl_active`/`s_fwu_busy`, send `STATUS`,
   wait ≤1.5 s on `s_status_sem`, publish `msp430gate/state` **iff it changed**
   or `MQTT_HEARTBEAT_MS` elapsed;
3. next wait = **1 s while the gate is moving, 8 s at rest**, and
   `ulTaskNotifyTake` returns early whenever `uart_rx_task` or an inbound command
   nudges it → **state changes publish within ~1 s**, idle traffic stays light.

All writers (`STATUS` poll, web console `DATA:`, `TIME:` sync, MQTT commands) go
through `app_uart_write()` which holds `s_uart_mux` for the write — see concern 1.

## Topics

| topic | dir | retain | payload |
|---|---|---|---|
| `msp430gate/availability` | pub | yes | `online` / `offline` (LWT) |
| `msp430gate/state` | pub | yes | `{"state":12,"state_name":"ST_LOCKOUT","cover_state":"stopped","moving":false,"low_batt":false,"open_limit":true,"closed_limit":true,"beam_broken":false,"nfault":0,"current_ma":0,"vm_mv":12311,"stall_count":0,"driver_asleep":true,"last_dir":"CLOSE","lockout":"…"}` — on change / 30 s heartbeat |
| `msp430gate/lastmove` | pub | yes | `{"dir":"OPEN","travel_ms":8200,"avg_ma":1160,"max_ma":3200,"peak_ma":4100,"cycles":142,"ts":"2026-09-08T19:30:00+00:00"}` — parsed from the arrival message; `cycles` persisted in NVS (`gatestats`) |
| `msp430gate/esp` | pub | yes | `{"rssi":-58,"uptime_s":12345,"free_heap":38000,"temp_c":42.5}` — ESP32 bridge diagnostics, ~every 30 s (`temp_c` = ESP32-C3 internal die temp, cached by `telemetry_task`) |
| `msp430gate/cmd` | sub | — | `OPEN` \| `CLOSE` \| `TOGGLE` \| `STOP` (case-insensitive); ignored while a BSL/flash/OTA session owns the UART |

`cover_state` is computed on the ESP32 (`gate_cover_state()`): state 1→`open`,
2→`closed`, 3→`opening`, 4→`closing`, **everything else→`stopped`** — so the HA
cover never goes `unknown`. Fault detail is carried by the `lockout` /
`state_name` fields and the lockout binary_sensor.

## HA discovery entities

Retained; re-sent on (re)connect but **rate-limited to once / 20 s** so a flapping
Wi-Fi link can't storm it. A `force_pub` flag pushes one fresh state right after
each (re)discovery. One shared `device` (`ids:["msp430gate_<mac6>"]`, plus
`"sw":"<MSP430 VERSION string>"` once the gate firmware reports it — a version
change forces an immediate re-publish). Every entity carries `uniq_id`
(`<devid>_<key>`) + full `object_id` so HA lets you rename / assign areas. The
cover uses **no `stat_*`/`opt` overrides** — `cover_state` already emits HA's exact
default words. Buffers: `cfg.buffer.size = 2048`, every `snprintf` guarded against
truncation, discovery payload buf 960 B, state buf 640 B.

**~26 entities, published with a 15 ms gap each (~0.4 s total):**

| key | component | notes |
|---|---|---|
| `gate` | **cover** | `dev_cla:gate`, `pl_open/pl_cls/pl_stop` → `msp430gate/cmd`, `val_tpl:{{ value_json.cover_state }}` (open/closed/opening/closing/stopped) |
| `state` | sensor | `{{ value_json.state_name }}` (`ST_*`) |
| `moving` | binary_sensor | `dev_cla:moving` (state 3/4/5/7/8) |
| `low_batt` | binary_sensor | `dev_cla:battery` (state 13 = `ST_LOW_BATTERY`) |
| `open_limit` / `closed_limit` | binary_sensor | template → `ON`/`OFF` (HA defaults) |
| `beam` | **sensor** | `Blocked` / `Clear` text (moved off `binary_sensor`/`problem`; old config is cleared on discovery) |
| `lockout` | binary_sensor | `dev_cla:problem`, `json_attr_t` exposes the reason; also lights on the pre-lockout "both limits active" line |
| `current` | sensor | mA, `stat_cla:measurement` |
| `vm` | sensor | `dev_cla:voltage`, V (`vm_mv/1000`) |
| `stalls` | sensor | stall count |
| `lm_travel` / `lm_avg` / `lm_max` / `lm_peak` | sensor | last-move travel time (ms) + avg/max/peak-inrush current (mA) — from `msp430gate/lastmove`. **Trend these for predictive maintenance.** |
| `lm_dir` / `lm_ts` | sensor | last-move direction; `dev_cla:timestamp` when it completed |
| `cycles` | sensor | `stat_cla:total_increasing`, NVS-persisted move counter |
| `esp_rssi` / `esp_uptime` / `esp_heap` / `esp_temp` | sensor | `entity_category:diagnostic` — from `msp430gate/esp` (`esp_temp` = `device_class:temperature`, °C) |
| `clear_fault` / `btn_open` / `btn_close` / `btn_stop` | **button** | `payload_press` → `msp430gate/cmd` |

`msp430gate/lastmove` republishes on each completed move (and after rediscovery);
`msp430gate/esp` every ~30 s. `cycles` is loaded from NVS at boot, incremented and
committed on each arrival message (~20 ms flash write, once per gate cycle).

## Testing

```
mosquitto_sub -h <broker-ip> -u <user> -P <password> -t 'msp430gate/#' -v
mosquitto_sub -h <broker-ip> -u <user> -P <password> -t 'homeassistant/#' -v
mosquitto_pub -h <broker-ip> -u <user> -P <password> -t msp430gate/cmd -m OPEN
```
HA → Settings → Devices → "MSP430 Gate Controller".

## Phase 2 ideas

- Tunable params (`LIST`) as HA `number` entities (`command_topic` → `SET_<NAME>`)
  + a `SAVE` button + a "Clear fault" button (sends `STOP`).
- Diagnostic sensors (RSSI / uptime / heap / `net_mode`) — data already in the
  `TELEMETRY:` stream (`entity_category: diagnostic`).
- Share the firmware STATUS parse with the web UI via a `GATESTATE:{json}` WS
  sentinel so the browser stops doing its own `STATUS` parsing.
- Move broker host/creds/base-topic to NVS + a web settings page.
- MQTT over TLS.

## Review of the 5 raised concerns (verdicts)

1. **UART mutex** — *real, for the RX side.* Two readers on one UART corrupt each
   other. Fix chosen: keep `uart_rx_task` the *sole* reader (it demuxes STATUS
   out of the stream), and put every *writer* behind `s_uart_mux` via
   `app_uart_write()`. A full request/response mutex was avoided so a 1.5 s
   status wait never blocks the web console.
2. **`unique_id`** — *real.* Added `uniq_id` + `obj_id` (MAC-suffixed) to every
   discovery payload; without it HA entities are unmanageable.
3. **Discovery buffer / rate** — *half real.* `buffer.size` is the per-message
   serialize buffer, not a ring of all pending, so it can't "overflow" from 9
   queued messages — **but** a single ~700 B cover payload is close to the 1024
   default, so `buffer.size = 2048` is a genuine fix. A 25 ms `vTaskDelay`
   between discovery publishes is cheap insurance and is included.
4. **Cover state mapping** — *real.* Unmapped enums render `unknown`. Handled by
   computing `cover_state` on the ESP32 with a `stopped` default for every
   non-clean state, so the template is a trivial field read.
5. **400 ms idle vs pattern detect** — *concern valid, not applicable here.* The
   400 ms wait was a browser thing. Firmware-side we key on the `;` terminator
   (event-driven, no delay) with a content-complete fallback. `uart_rx_task`
   already reads every byte continuously for the console relay, so
   `uart_enable_pattern_det_baud` would add an event-queue driver reinstall for
   no gain.
