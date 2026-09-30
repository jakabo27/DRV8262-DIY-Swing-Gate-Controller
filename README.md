# DRV8262 DIY Swing Gate Controller

A swing-gate opener I built from scratch to replace a relay-based controller. It drives the gate's actuator with Texas Instruments' **DRV8262** integrated H-bridge, so every move has a soft start and a soft stop instead of full power until a limit switch cuts it off. An **MSP430** runs the gate and all of its safety logic. An **ESP32** adds a web dashboard, Home Assistant integration, and wireless firmware updates for both chips.

![MSP430F5338](https://img.shields.io/badge/MCU-MSP430F5338-c00?style=flat-square)
![DRV8262](https://img.shields.io/badge/motor%20driver-TI%20DRV8262-c00?style=flat-square)
![ESP32-C3](https://img.shields.io/badge/Wi--Fi-ESP32--C3-555?style=flat-square)
![Home Assistant](https://img.shields.io/badge/Home%20Assistant-MQTT%20discovery-18bcf2?style=flat-square)
![License: MIT](https://img.shields.io/badge/license-MIT-555?style=flat-square)

## Engineering Skills & Competencies

| Domain | Technologies & Key Implementations |
|---|---|
| **Motor Drive & Power** | TI DRV8262 (Single H-Bridge, 25mΩ, 25kHz PWM), trapezoidal ramp profiling, active short-braking, inrush suppression |
| **Mixed-Signal Sensing** | Analog current mirror ($I_{PROPI}$), phase-swept ADC sampling (anti-aliasing against PWM), 1024-point 25ms telemetry capture |
| **Real-Time & Safety Firmware** | TI MSP430F5338 (Bare-metal C), 1ms deterministic control loop, LPM0 low-power scheduling, 3-tier overcurrent protection, fail-safe FSM |
| **Systems & IoT Architecture** | Dual-MCU distributed architecture, Seeed XIAO ESP32-C3 (ESP-IDF C++), MQTT auto-discovery, WebSockets, change-gated events |
| **Diagnostics & Bootloaders** | Custom RAM-relocated IAP bootloader (CRC-16 framing), dual-slot ESP32 OTA, live register-level pipeline recovery watchdog |
| **Testing & Tooling** | Host-side PC unit tests, Python wire-contract AST/regex CI, Saleae logic analyzer debugging, automated CSV telemetry plotting |

## Relay vs. PWM: same gate, same motor

<!-- 🎬 HERO GIF: when media/video/relay-vs-pwm-stop.gif exists (see "Claude deleteme.md" → Video), uncomment the next line.
![Side by side: the relay-driven gate coasts about 1.5ft past its closed position and oscillates back, while the PWM-driven gate eases in and stops at the limit](media/video/relay-vs-pwm-stop.gif)
-->

<!-- 🎬 FULL VIDEO: on github.com, edit this README and drag relay-vs-pwm-soft-stop.mp4 (≤ 10MB) onto the editor.
     GitHub uploads it and inserts a https://github.com/user-attachments/assets/... link. Replace this whole comment
     with that link, alone on its line, and GitHub shows a video player here. -->

**The original relay controller** ran the motor at full power until the limit switch opened the relay. A heavy swing gate is still at full speed at that moment, so its momentum carried it about 1.5ft too far, and it then oscillated slowly back into place. **This controller** eases the motor to 60% near the end of travel, winds down to zero at the limit, and holds the arm with a 1s short-brake. The gate arrives slowly and stops at the limit, with almost no overshoot.

## Why an integrated H-bridge instead of relays

A relay is either on or off. The DRV8262 switches the motor with MOSFETs at 25kHz, so the firmware can set any speed from 0 to 100% and change it gradually. That makes the soft stop possible, and it also brings current sensing and protection built into the same chip.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="media/charts/pwm-close-profile-dark.svg">
  <img alt="Motor current during a real close: a 7.3A peak at launch, about 2.5 to 3A at full speed, and a drop to about 1A after the duty is eased to 60% at 9.8s, ahead of the closed limit at 14.6s. The commanded-duty panel below compares the PWM profile (75% launch, ramp to 100%, ease to 60%, brake) with a relay's 100% then off." src="media/charts/pwm-close-profile-light.svg">
</picture>

*A real close, recorded by the controller itself (it captures current and voltage every 25ms during every move). [Raw data](media/data/close-2026-09-28.csv) · [chart script](tools/plot_capture.py)*

| | Relay control | DRV8262 PWM (this project) |
|---|---|---|
| **Speed** | Full power or off | Anything from 0 to 100%, changed smoothly |
| **Starting** | Full voltage at once: an inrush spike, a jerk, and a sag on the battery | Launches at 75% and ramps to 100% over 2.5s |
| **Stopping** | Power is cut at the limit switch while the gate is still at full speed. On this gate that meant coasting about 1.5ft too far, then oscillating back into place | Eases to 60% before the limit, winds to zero, then short-brakes the motor to hold the arm: almost no overshoot |
| **Current sensing** | Needs an added shunt resistor and amplifier, or a Hall sensor | Built in: the IPROPI pin outputs a copy of the motor current, so an ADC pin and a resistor are all it takes |
| **Over-current protection** | A fuse | The chip's own current regulation and shutdown, plus two firmware trip levels |
| **Obstacle detection** | Needs extra hardware | The same current signal: a jump while closing means something is in the way, so the gate reverses |
| **Reversing direction** | Two relays or a DPDT, timed so they never short the supply | Two logic pins, with shoot-through protection in the chip |
| **Wear and noise** | Contacts arc on every DC motor break, and they click | Solid-state and silent (25kHz is above human hearing) |

Every step of that motion profile is a runtime setting (`SET_SOFT_END_CLOSE 9500`, `SAVE`), so tuning it for a different gate takes a web form, not a recompile.

## The build

<p align="center">
  <img src="media/hardware/controller-installed.webp" width="560" alt="The finished controller on a 3D-printed mounting plate at the gate post: the DRV8262VEVM with its heatsink on top, and below it a breadboard with the XIAO ESP32-C3, a four-button test board, a 5V supply, a PC817 optocoupler board and terminal blocks for the field wiring">
</p>

*The finished controller, on a custom 3D-modeled and printed mounting plate (with a cover) at the gate post. The DRV8262VEVM (top) carries the DRV8262 under the heatsink and the MSP430F5338 on the left. The breadboard (bottom) holds the XIAO ESP32-C3 (left), a four-button test board, a 5V supply, a PC817 optocoupler board translating the 12V field inputs down to 3.3V to protect the MCU digital logic (right), and terminal blocks for the limit switches, photo beam and RF-remote receiver. There's also a [labeled photo of the bench version](media/hardware/breadboard-prototype.png) that shows what each terminal is for.*

The base is TI's **DRV8262VEVM** evaluation module, in the DDV package variant: the driver's thermal pad is on *top* of the chip, so the heatsink clips straight onto it instead of the heat going out through the board. The EVM already carries an MSP430F5338, which became the gate's controller. The gate runs on a 12V battery.

## What it does

- **Soft start and soft stop on every move**, as shown above.
- **Obstacle reversal while closing**, from either a current spike or the photo beam. **Halt and back off while opening**, since an obstacle on the way open is almost always the arm reaching its mechanical stop.
- **Three levels of over-current protection**: a firmware soft trip that ignores the launch surge, a firmware hard trip that is always armed, and the DRV8262's own hardware limit.
- **Safety checks that don't need the network**: stall timeout, a check that the starting limit switch actually releases, a lockout if both limit switches read active at once, low-battery lockout with hysteresis, and driver-fault recovery. The MSP430 handles all of these by itself.
- **Photo-beam vehicle detection.** Every beam break is timed to ±10ms and classified as a car, a person or animal, or a long block. Home Assistant uses that to close the gate right after a car goes through.
- **Per-move health data.** Travel time, average and peak current, and inrush for every move, plus a full 25ms current/voltage capture. Current creeping up over weeks is an early warning for a sticking hinge or a worn actuator.
- **Sleeps between ticks**: a 1ms control loop while moving, 10ms at rest, and the motor driver powered down between moves. The serial console still answers at any time.
- **25 runtime-tunable parameters**, saved in the MSP430's flash, adjustable from the dashboard.
- **Pushes events instead of being polled.** A state change reaches Home Assistant in about 0.23s (it used to take 1.4s).
- **Silicon errata mitigation (ADC pipeline watchdog).** Live register debugging on the bench revealed an MSP430 Timer_A/ADC pipeline stall condition. The firmware includes an active watchdog that forces a pipeline restart if samples delay >30ms, ensuring overcurrent protection never goes blind while moving.
- **Wireless firmware updates for both chips**, including the MSP430, whose ROM bootloader entry doesn't work on this board. [How →](#the-hard-part-updating-firmware-from-the-study-bypassing-a-dead-bootloader)

## Architecture

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="media/diagrams/architecture-dark.svg">
  <img alt="How it fits together. A phone or laptop browser talks HTTP and WebSocket to the ESP32-C3 Wi-Fi bridge, and Home Assistant reaches it through an MQTT broker. The bridge (web dashboard, wireless flasher, MQTT) owns the one UART link to the MSP430F5338 at 115200 8N1. The MSP430 runs the state machine and all safety logic, the 25kHz PWM motion profile, and current and voltage sensing. It reads the limit switches, photo beam and RF remote, drives the DRV8262 H-bridge on IN1/IN2, and reads the motor current back through IPROPI. The DRV8262 drives the gate actuator from the 12V battery." src="media/diagrams/architecture-light.svg">
</picture>

The split is deliberate. The MSP430 owns everything real-time and everything safety-related, and it works the same with the ESP32 unplugged. The ESP32 is a service layer: if Wi-Fi drops or the bridge reboots, the gate doesn't notice.

## Web dashboard

<p align="center">
  <img src="media/screenshots/web-dashboard.png" width="720" alt="The web dashboard right after an open: gate controls, status tiles (open at limit, photo beam clear, 14.4V supply, driver asleep), and the last move's motor current and supply voltage, 12.72s long with a 7.3A launch peak and the current dropping off at the 60% soft end">
</p>

The ESP32 serves a single-page dashboard. This is it right after an open: live status, and the current and battery voltage the MSP430 recorded during the move, where you can see the launch peak and the current dropping away at the soft end. Further down are the full parameter table with per-row set and reset, a live serial console, and firmware upload for both chips (screenshots in [`esp32-wifi-bridge/`](esp32-wifi-bridge/)).

## The hard part: updating firmware from the study (bypassing a dead bootloader)

MSP430s normally take new firmware through the ROM bootstrap loader: hold `TEST`, pulse `RST`, and the chip listens for a flashing protocol. On this board's MSP430F5338 that entry sequence never works. That was a problem, because the gate is out in the weather at the end of the driveway, and I wanted to push firmware updates from the comfort of my study.

So the **running gate firmware updates itself via a custom RAM-resident In-Application Programming (IAP) loader**. You drop a new image onto the ESP32's web page. The ESP32 sends `FWUPDATE`, and the MSP430 copies a small updater into RAM, stops the gate application, and takes the image over its normal serial console in CRC-16 checked frames, acknowledging each one. It rewrites its own flash segment by segment (leaving the saved settings alone) and reboots into the new build. The ESP32 also has the standard BSL entry paths as fallbacks for boards where they do work, and it updates itself over Wi-Fi through a two-slot OTA partition table. Full write-up: [`esp32-wifi-bridge/README.md`](esp32-wifi-bridge/README.md#the-hard-part-flashing-an-msp430-whose-bootloader-doesnt-work).

## Measured

| | Result |
|---|---|
| Home Assistant command → `moving` shown in HA | **234ms** mean (was 1362ms with polling) |
| Firmware's share of that | 0–4ms. The rest is the Wi-Fi/MQTT round trip |
| Bridge reboot → correct state back in HA | ~2s |
| Current capture per move | 1024 samples × 25ms: current and battery voltage |
| Photo-beam edge timing | ±10ms |
| Host-side tests | Build the real `beam.c` and the bridge's real parsers on a PC, and check both firmwares agree on every field of the serial protocol ([`tests/`](tests/)) |

Details: [`docs/test-results-2026-09-28.md`](docs/test-results-2026-09-28.md).

## Testing & Verification

A robust test harness was used to validate the embedded logic, utilizing both hardware and software tooling:
- **Host-side CI**: The `tests/` directory compiles the bare-metal C (e.g. `beam.c`) on a PC host to test against synthetic waveforms (e.g., car + trailer profiles, 20ms glitches, chatter).
- **Wire-Contract Checks**: Python AST and regex linters verify that the ESP32 bridge parsers strictly match the MSP430 event emissions so there is zero protocol drift between the two codebases.
- **Hardware Debugging**: Used a Saleae logic analyzer on the bench to precisely time the PWM slewing, optocoupler input delay, and the dual-MCU UART sync.

## Repository layout

| Folder | What's in it |
|---|---|
| [`msp430-gate-controller/`](msp430-gate-controller/) | The gate firmware: state machine, motion profile, current sensing, safety logic. A Code Composer Studio project |
| [`esp32-wifi-bridge/`](esp32-wifi-bridge/) | ESP32-C3 firmware: web dashboard, MQTT/Home Assistant, wireless flashing. A PlatformIO project |
| [`home-assistant/`](home-assistant/) | Entity reference and the gate automations |
| [`tests/`](tests/) | Host-side tests that cross-check the two firmwares' serial protocol |
| [`tools/`](tools/) | `plot_capture.py` turns a move capture into the chart above. `architecture-diagram/` is the source for the architecture drawing |
| [`docs/`](docs/) | Write-ups: the switch from polling to pushed events, and its test results |
| `media/` | Photos, screenshots, charts, video |

## Home Assistant integration and gate automations

![The MSP430 Gate Controller device page in Home Assistant: gate cover and buttons, limit switches, photo beam, motor current, supply voltage, cycle count and last-move sensors, plus the related automations and recent activity](media/screenshots/ha-device-page.png)

The bridge registers itself with MQTT discovery as one device, **MSP430 Gate Controller**, with a `cover` and 28 more entities: limit switches, photo beam, a beam-activity event (`vehicle_pass` / `short_break` / `long_block`), live current and battery voltage, per-move health stats, a cycle counter, open/close/stop buttons, and bridge diagnostics. No YAML is needed on the Home Assistant side.

On top of that, two automations run the gate day to day:

```mermaid
flowchart LR
    subgraph OPEN ["Driveway Gate: Open"]
        direction TB
        A(["Phone enters a 900m zone<br/>around home"]) --> B{"Driving?<br/>car Bluetooth / Android Auto /<br/>iPhone activity"}
        B -- yes --> C["wait until the gate isn't moving,<br/>then OPEN"]
        C --> D["car seen inside<br/>(camera or beam)"] --> E["open the garage"]
        X(["Camera sees a car<br/>leaving from inside"]) --> C
    end
    subgraph CLOSE ["Driveway Gate: Auto Close"]
        direction TB
        F(["Beam: vehicle_pass<br/>(+10s to clear the arc)"]) --> S
        G(["Open 3min or more and<br/>nobody is driving in"]) --> S
        H(["10-min hard cap"]) --> S
        S{"gate stopped, beam clear,<br/>swing area clear?"} -- yes --> P["CLOSE"]
    end
```

The main rule: the firmware treats *any* command during a move as "stop now", so the automations never press a button while the gate is moving. Every close is also re-checked (beam, camera zone, hold-open switch) immediately before the button press. In a replay against 10 days of real traffic, the new logic would have cut the median time the gate stands open from 3.5 minutes to under one. The generalized YAML, a placeholder-to-entity table, and flowcharts for each branch are in [`home-assistant/`](home-assistant/).

## Safety note

This is a personal project, and it hasn't been certified to UL 325 or any other gate-operator standard. Automatic gates can crush and kill. The obstacle reversal here is a backstop, not something to rely on: keep people and pets clear of a moving gate, test the reversal regularly, and if you build something like this, give it a photo beam at the very least.

## License

MIT. See [`LICENSE`](LICENSE).
