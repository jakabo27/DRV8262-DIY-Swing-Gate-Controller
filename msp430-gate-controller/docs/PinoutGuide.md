# DRV8262VEVM Hardware & Firmware Programming Pinout Guide

This document maps the physical interconnections of the **DRV8262VEVM** evaluation module, tracking signal routing across the **DRV8262 Motor Driver (U7)**, the **MSP430F5338IPZ Microcontroller (U1)** host, the **J4/J8 Isolation Jumpers**, and the **C2000 LaunchPad (LAUNCHXL-F280025C) Connectors (J1/J2)**.

---

# DRV8262 DDV Technical Guide: Single H-Bridge Mode Capabilities

## Power & Hardware Configuration
* **Package Architecture**: Features a top-side exposed thermal pad (HTSSOP-44) optimized for direct heat sink attachment.
* **Single H-Bridge Pin Tie-In**: Tied into Single H-bridge mode at startup by setting pin `MODE1 = High`. Outputs `OUT1` and `OUT3` are connected in parallel, as are `OUT2` and `OUT4`.
* **Current & Resistance**: Reduces total conduction resistance to 25 mΩ per side (50 mΩ high-side + low-side combined). Delivers up to 32 A peak current and up to 20 A RMS continuously over an operating range of 4.5 V to 60 V.

## Control Modes & Dynamic Speed PWM
* **Control Interface Selection (`MODE2` pin)**:
  * **PWM Mode (IN/IN)** (`MODE2` Floating or connected to `DVDD`): Uses `IN1` and `IN2` for independent high-side/low-side drive control. Supports Forward (1,0), Reverse (0,1), High-Side Slow Decay Brake (1,1), and Coast / Hi-Z (0,0) states.
  * **PH/EN Mode** (`MODE2 = GND`): Sets `IN1` as the Enable/PWM input for speed control and `IN2` as Phase for direction control.
* **Dynamic PWM Deceleration**: Duty cycle modulation on the PWM inputs allows smooth motor deceleration (e.g., slowing down motor speed as it approaches a Hall-sensor trigger point or mechanical limit switch). The driver automatically handles high-side/low-side dead-time insertion (300 ns typical) to prevent shoot-through.

## Analog Current Sensing (`IPROPI`)
* **Pin Setup**: Requires tying pins `IPROPI1` and `IPROPI2` together in Single H-bridge mode.
* **Current Feedback Mirror**: Sources an analog feedback current proportional to the current flowing through the high-side MOSFETs with a mirror gain $A_{IPROPI} = 212\ \mu\text{A/A}$ ($\pm 4\%$ accuracy from 40% to 100% rated load).
* **ADC Voltage Conversion**: Connecting an external resistor $R_{IPROPI}$ to ground converts the feedback current into a proportional signal voltage for an external MCU ADC:
  $$V_{IPROPI} = I_{PROPI} \times R_{IPROPI}$$
* **Monitored Operating States**: Continuously reflects motor current during Forward Drive, Reverse Drive, and High-Side Slow Decay (brake) modes.

## Current Limit Regulation & VREF Control
* **Pin Setup**: Requires tying pins `VREF1` and `VREF2` together.
* **Trip Current Threshold ($I_{TRIP}$)**: An internal comparator compares $V_{IPROPI}$ against $V_{VREF}$. The current limit threshold is determined by:
  $$I_{TRIP} \times A_{IPROPI} = \frac{V_{VREF}}{R_{IPROPI}}$$
* **Dynamic VREF Adjustment**: Modulating the voltage applied to `VREF` (via a DAC or a low-pass filtered PWM signal from the controller) allows on-the-fly adjustment of the motor current limit during operation.
* **Chopping & Decay Customization**:
  * **Off-Time Pin (`TOFF`)**: Configures fixed PWM off-time to 7 µs, 16 µs, 24 µs, or 32 µs.
  * **Decay Mode Pin (`DECAY`)**: Configures current chopping decay method to Slow Decay (brake), Smart Tune Dynamic Decay, or 30% Fast / 70% Slow Mixed Decay.

## Fault Indication & Safety Protection
* **`nFAULT` Indication Pin (Pin 41)**: Open-drain active-low output requiring a pull-up resistor to `DVDD`, `VCC`, or an external rail. Asserts low upon detecting VM Undervoltage Lockout (UVLO), Charge Pump Undervoltage (CPUV), Overcurrent Protection (OCP), or Thermal Shutdown (OTSD).
* **Fault Recovery Mode (`OCPM` Pin)**:
  * **Latch-Off (`OCPM = Low`)**: Driver outputs disable and remain latched off following an OCP or OTSD event until cleared.
  * **Auto-Retry (`OCPM = High`)**: Driver automatically attempts recovery after $t_{RETRY} \approx 4.1\text{ ms}$.
* **`nSLEEP` Control & Fault Reset**: Pulling `nSLEEP` low puts the device into a low-power 3 µA sleep mode. Sending a brief pulse of 20 µs to 40 µs on `nSLEEP` clears latched faults without placing the IC into full sleep mode.

## Additional Functional Features
* **Spread Spectrum Clocking**: Integrates triangular frequency modulation on the internal digital clock and charge pump to suppress radiated EMI peaks.
* **Logic Power Supplies**: Includes an internal 5 V `DVDD` regulator. An optional external supply on `VCC` (3.05 V to 5.5 V) powers internal logic directly to bypass VM regulator power dissipation in high-temperature applications.

--

## 1. Core Signal Mapping (DRV8262 ↔ MSP430 ↔ C2000 LaunchPad)

The following list shows how every control and telemetry pin of the DRV8262 motor driver maps to the MSP430 host MCU and the C2000 external headers. 
*Note: Paths assume default shunts/jumpers are installed across the respective left and right pins of J4 and J8.*

***All MSP430 pin/port values and C2000 `J1`/`J2` pin numbers in this section were rebuilt from the board netlist (`DRV8262VEVM NEtlist.txt`) and supersede every earlier revision. The earlier revision had assumed a tidy sequential `P4.0`–`P4.7` / `P8.0`–`P8.6` mapping that the netlist does not bear out.***

*   **IN1 (Control Input 1 / Phase 1 / Enable)**
    *   **DRV8262 Pin:** Pin 26 (`ENABLE/IN1`)
    *   **Jumper Path:** `J8` (Pin 4 ↔ Pin 3) to net `MSP_ENABLE/IN1/PH1`
    *   **MSP430 Host MCU:** Pin 51, Port `P4.1 / TB0.1`
    *   **C2000 Breakout:** `J1` Pin 9 (Net `C2K_ENABLE/IN1/PH1`)
    *   **LED Indicator Connection:** None on this input. The `DRV_ENABLE` LED **D9 (Yellow)** is a separate, firmware-driven pin — MSP430 Pin 36 (`P1.2 / TA0.1`); see Section 2.

*   **IN2 (Control Input 2 / Phase 2 / Direction)**
    *   **DRV8262 Pin:** Pin 27 (`DIR/IN2`)
    *   **Jumper Path:** `J8` (Pin 6 ↔ Pin 5) to net `MSP_DIR/IN2/PH2`
    *   **MSP430 Host MCU:** Pin 52, Port `P4.2 / TB0.2`
    *   **C2000 Breakout:** `J2` Pin 5 (Net `C2K_DIR/IN2/PH2`)
    *   **LED Indicator Connection:** None

*   **IN3 (Control Input 3 / Phase 3 / Step)**
    *   **DRV8262 Pin:** Pin 28 (`IN3/Reserved`) via net `STEP/IN3/RSVD`
    *   **Jumper Path:** `J8` (Pin 8 ↔ Pin 7) to net `MSP_STEP/IN3/PH3`
    *   **MSP430 Host MCU:** Pin 53, Port `P4.3 / TB0.3`
    *   **C2000 Breakout:** `J2` Pin 20 (Net `C2K_STEP/IN3/PH3`)
    *   **LED Indicator Connection:** None

*   **IN4 (Control Input 4 / Phase 4 / SCLK)**
    *   **DRV8262 Pin:** Pin 29 (`IN4/Reserved`) via net `SCLK/M1/IN4/RSVD`
    *   **Jumper Path:** `J8` (Pin 10 ↔ Pin 9) to net `MSP_M1/SCLK/IN4/PH4`
    *   **MSP430 Host MCU:** Pin 54 (`P4.4 / TB0.4`) **and** Pin 62 (`P8.4 / UCB1CLK / UCA1STE`) — the net ties both MCU pins together. Pin 50 (`P4.0 / TB0.0`) is also on the net but only through the 330 kΩ pulldown `R3` (`MSP_M1_PD`), so it is a bias tap, not a drive point.
    *   **C2000 Breakout:** `J1` Pin 13 (Net `C2K_M1/SCLK/IN4/PH4`)
    *   **LED Indicator Connection:** None

*   **nSLEEP (Device Sleep Mode - Active Low)**
    *   **DRV8262 Pin:** Pin 25 (`SLEEP`) via net `nSLEEP`
    *   **Jumper Path:** `J8` (Pin 2 ↔ Pin 1) to net `MSP_nSLEEP`
    *   **MSP430 Host MCU:** Pin 41, Port `P1.7 / TA0.2`
    *   **C2000 Breakout:** `J2` Pin 7 (Net `C2K_nSLEEP`)
    *   **LED Indicator Connection:** None

*   **nFAULT (Fault Indication Output - Active Low, Open-Drain)**
    *   **DRV8262 Pin:** Pin 41 (`FAULT`) via net `nFAULT`
    *   **Jumper Path:** `J4` (Pin 16 ↔ Pin 15) to net `MSP_nFAULT`
    *   **MSP430 Host MCU:** Pin 24, Port `P2.7 / P2MAP7`
    *   **C2000 Breakout:** `J1` Pin 15 (Net `C2K_nFAULT`)
    *   **LED Indicator Connection:** The driver-side `nFAULT` net drives **D7 (Super Red)** through R112; with the `J4` 15↔16 shunt installed the same net reaches MSP430 Pin 24 as a readable input.

*   **nHOME / OCPM (Indexer Home / Overcurrent Protection Mode Status Output)**
    *   **DRV8262 Pin:** Pin 40 (`OCPM`) via net `nHOME/OCPM`
    *   **Jumper Path:** `J4` (Pin 14 ↔ Pin 13) to net `MSP_nHOME/OCPM`
    *   **MSP430 Host MCU:** Pin 38, Port `P1.4 / TA0.3`
    *   **C2000 Breakout:** `J2` Pin 1 (Net `C2K_nHOME/OCPM`)
    *   **LED Indicator Connection:** None on this input. The `INDEXER HOME` LED **D11 (Yellow)** is a *separate*, firmware-driven pin — MSP430 Pin 39 (`P1.5 / TA0.4`); see Section 2.

*   **TOFF (Off-Time Configuration Control Input)**
    *   **DRV8262 Pin:** Pin 32 (`TOFF`) via net `RSVD/TOFF/EN3`
    *   **Jumper Path:** `J8` (Pin 16 ↔ Pin 15) to net `MSP_TOFF/RSVD/EN3`
    *   **MSP430 Host MCU:** Pin 22, Port `P2.5 / P2MAP5`. Pin 23 (`P2.6 / P2MAP6`) is also on the net through the 330 kΩ pulldown `R2` (`MSP_TOFF_PD`) — a bias tap, not a drive point.
    *   **C2000 Breakout:** `J2` Pin 3 (Net `C2K_TOFF/RSVD/EN3`)
    *   **LED Indicator Connection:** None

*   **DECAY / SDI (Decay Mode Control Input 0 / SPI Data Input)**
    *   **DRV8262 Pin:** Pin 30 (`DECAY`) via net `SDI/DECAY0/EN1/DECAY`
    *   **Jumper Path:** `J8` (Pin 12 ↔ Pin 11) to net `MSP_DECAY0/SDI/DECAY/EN1`
    *   **MSP430 Host MCU:** Pin 65, Port `P8.5 / UCB1SIMO / UCB1SDA`
    *   **C2000 Breakout:** `J2` Pin 12 (Net `C2K_DECAY0/SDI/DECAY/EN1`)
    *   **LED Indicator Connection:** None

*   **DECAY1 / SDO (Decay Mode Control Input 1 / SPI Data Output)**
    *   **DRV8262 Pin:** Pin 31 (`Reserved`) via net `SDO/DECAY1/EN2/RSVD`
    *   **Jumper Path:** `J8` (Pin 14 ↔ Pin 13) to net `MSP_DECAY1/SDO/RSVD/EN2`
    *   **MSP430 Host MCU:** Pin 66, Port `P8.6 / UCB1SOMI / UCB1SCL` (netlist: `MSP_DECAY1/SDO/RSVD/EN2` = `J8.13` → `U1.66`; supersedes the prior revision's Pin 59 / `P8.1`)
    *   **C2000 Breakout:** `J2` Pin 14 (Net `C2K_DECAY1/SDO/RSVD/EN2`)
    *   **LED Indicator Connection:** None
    *   **GateController firmware use:** repurposed as a digital input — see Section 5.2 (use TBD).

*   **nSCS / VREF1 / M0 (SPI Chip Select / Voltage Reference Input 1 / Mode Select Input 0)**
    *   **DRV8262 Pin:** Pin 33 (`VREF1/VREF`) via net `nSCS/M0/EN4/VREF1/VREF`
    *   **Jumper Path:** `J4` (Pin 2 ↔ Pin 1) to net `MSP_M0/nSCS/VREF1/EN4` (VREF source further gated by `J25`/`J26`)
    *   **MSP430 Host MCU:** Pin 3, Port `P6.6 / CB6 / A6 / DAC0` (the `DAC0` function fits a firmware-driven `VREF1`). Pin 59 (`P8.1 / UCB1STE / UCA1CLK`, net `MSP_nSCS`) also reaches this node through the series resistor `R54`.
    *   **C2000 Breakout:** `J2` Pin 18 (Net `C2K_M0/nSCS/VREF1/EN4`)
    *   **LED Indicator Connection:** None
    *   **GateController firmware use (active path, this build):** the EVM is jumpered `J25` + `J6` instead of `J4` + `J26` above - that routes `VREF1` through the `VREF/VREF2` net, via `J6`, to **MSP430 Pin 4, Port `P6.7 / CB7 / A7 / DAC1`** (a second, independent DAC channel, `DAC12_1`). `drv8262.c` drives `VREF1` from there (console parameter `VREF_CURRENT_REGULATION`) - see its file-header comment and `gate_config.h`. `P6.6`/`DAC12_0` above is left as a fixed, unused GPIO-high leftover on this build.

*   **MODE1 / MODE (Mode Selection Input 1)**
    *   **DRV8262 Pin:** Pin 39 (`MODE1`) via net `MODE/MODE1`
    *   **Jumper Path:** `J4` (Pin 12 ↔ Pin 11) to net `MSP_MODE/MODE1`
    *   **MSP430 Host MCU:** Pin 67, Port `P8.7`
    *   **C2000 Breakout:** `J2` Pin 13 (Net `C2K_MODE/MODE1`)
    *   **LED Indicator Connection:** None

*   **MODE2 / IPROPI4 (Mode Selection Input 2 / Proportional Current Output 4)**
    *   **DRV8262 Pin:** Pin 38 (`MODE2`) via net `MODE2/IPROPI4`
    *   **Jumper Path:** `J4` (Pin 4 ↔ Pin 3), then series `R39` (1 kΩ) to net `MSP_MODE2/IPROPI4`
    *   **MSP430 Host MCU:** Pin 100, Port `P6.3 / CB3 / A3`
    *   **C2000 Breakout:** `J1` Pin 18 (Net `C2K_MODE2/IPROPI4`)
    *   **LED Indicator Connection:** None

*   **IPROPI3 (Proportional Current Sense Output 3)**
    *   **DRV8262 Pin:** Pin 37 via net `RSVD/IPROPI3/IPROPI2/IPROPI`
    *   **Jumper Path:** `J4` (Pin 6 ↔ Pin 5), then series `R40` (1 kΩ) to net `MSP_IPROPI2/IPROPI3`
    *   **MSP430 Host MCU:** Pin 99, Port `P6.2 / CB2 / A2`
    *   **C2000 Breakout:** `J1` Pin 16 (Net `C2K_IPROPI2/IPROPI3`)
    *   **LED Indicator Connection:** None

*   **IPROPI2 (Proportional Current Sense Output 2)**
    *   **DRV8262 Pin:** Pin 36 via net `RSVD/IPROPI2/IPROPI1/IPROPI`
    *   **Jumper Path:** `J4` (Pin 8 ↔ Pin 7), then series `R41` (1 kΩ) to net `MSP_IPROPI1/IPROPI2`
    *   **MSP430 Host MCU:** Pin 98, Port `P6.1 / CB1 / A1` (netlist: `MSP_IPROPI1/IPROPI2` → `U1.98`. This resolves the earlier Pin 66 conflict — Pin 66 belongs to `DECAY1/SDO`.)
    *   **C2000 Breakout:** `J1` Pin 14 (Net `C2K_IPROPI1/IPROPI2`)
    *   **LED Indicator Connection:** None

*   **IPROPI1 / RSVD (Proportional Current Sense Output 1 / Reserved on the driver)**
    *   **DRV8262 Pin:** Pin 35 (`Reserved`) via net `RSVD/IPROPI1`
    *   **Jumper Path:** `J4` (Pin 10 ↔ Pin 9), then series `R42` (1 kΩ) to net `MSP_RSVD/IPROPI1`
    *   **MSP430 Host MCU:** Pin 97, Port `P6.0 / CB0 / A0`
    *   **C2000 Breakout:** `J1` Pin 12 (Net `C2K_RSVD/IPROPI1`)
    *   **LED Indicator Connection:** None
    *   **GateController firmware use:** repurposed as a digital input (CLOSE button) — see Section 5.2.

---

## 2. Dedicated MSP430 MCU Firmware Indicators (LED Mapping)

These pins belong exclusively to the MSP430 user interface software infrastructure on the EVM layout. Use these GPIO registers to toggle board status flags.

*Note: The D-number, color, and series-resistor references below were read directly off the EVM's "LEDS" schematic block and supersede any earlier revision of this document.*

*   **EVM_STATUS Indicator**
    *   **MCU Pin:** Pin 34, Port `P1.0 / TA0CLK / ACLK`
    *   **LED Physical Asset:** **D6 (Green)** via series resistor R114 (330Ω)
    *   **GateController Firmware Behavior:** Heartbeat — toggle on/off once per second.
*   **GUI_STATUS Indicator**
    *   **MCU Pin:** Pin 35, Port `P1.1 / TA0.0`
    *   **LED Physical Asset:** **D8 (Green)** via series resistor R113 (330Ω)
    *   **GateController Firmware Behavior:** Indicates photo beam state — ON if the beam is good, OFF if broken.
*   **DRV_ENABLE Indicator**
    *   **MCU Pin:** Pin 36, Port `P1.2 / TA0.1`
    *   **LED Physical Asset:** **D9 (Yellow)** via series resistor R115 (330Ω)
    *   **GateController Firmware Behavior:** Indicates the Arm Extended limit switch state.
*   **ACCEL/DECEL Indicator**
    *   **MCU Pin:** Pin 37, Port `P1.3 / TA0.2`
    *   **LED Physical Asset:** **D10 (Yellow)** via series resistor R111 (330Ω)
    *   **GateController Firmware Behavior:** Indicates the Arm Retracted limit switch state.
*   **INDEXER_HOME Status**
    *   **MCU Pin:** Pin 39, Port `P1.5 / TA0.4` (netlist: `INDEXER HOME` → `U1.39`; was wrongly listed as Pin 38 / `P1.4` — that pin is the `nHOME/OCPM` driver input, Section 1)
    *   **LED Physical Asset:** **D11 (Yellow)** via series resistor R116 (330Ω)
    *   **GateController Firmware Behavior:** Lights up when any button input (`A4`/`A3`/`A2`) is pressed.
*   **EN4 Indicator**
    *   **MCU Pin:** Pin 40, Port `P1.6 / TA0.1` (netlist: `EN4` → `U1.40`; was wrongly listed as Pin 39 / `P1.5`)
    *   **LED Physical Asset:** **D12 (Yellow)** via series resistor R117 (330Ω)
    *   **GateController Firmware Behavior:** Indicates a fault or stall condition.
*   **S1 push-button — reset only, NOT a firmware GPIO**
    *   Tactile switch `S1` connects through selection header `J27`. With the default shunt on `J27` Pins 1–2, `S1` pulls `RST/NMI/SBWTDIO` (MSP430 Pin 96) low — it is the **reset button**. `J27` Pin 3 (net `PUSH`) is the unrouted alternate position.
    *   There is **no `BUTTON` net** on this board. Earlier revisions placed `S1` on `P1.6` (Pin 40); that pin is actually the `EN4` LED (above).

**Non-GPIO LED indicators (not firmware-controlled):**

*   **D5 (Green)** — wired directly to `MCU_3V3` via R110 (560Ω); always on whenever the board has power. Not connected to any MCU pin.
*   **D7 (Super Red)** — driven by the driver-side `nFAULT` net (DRV8262 open-drain FAULT output) via R112 (330Ω); lights automatically on a driver fault. With the `J4` 15↔16 shunt installed the same net also reaches MSP430 Pin 24 (`P2.7 / P2MAP7`, net `MSP_nFAULT`) as a firmware-readable input, but the LED itself is not driven by firmware.

---

## 3. High-Value Analog Telemetry Breakouts

The EVM includes conditioned voltage dividers that feed VM (Supply Voltage) and the full phase output bridge states directly into the ADC blocks of both the MSP430 host MCU and the C2000 header interfaces for true real-time observation:

| Monitored Parameter | EVM Net Name | MSP430 ADC Pin / Channel | C2000 Breakout Header Connection |
| :--- | :--- | :--- | :--- |
| **Motor Supply Voltage (VM)** | `VSEN_VM` | Pin 1 — `P6.4 / CB4 / A4` (channel **A4**), direct | `J1` Pin 3 |
| **Bridge Output Phase A1** | `VSEN_AOUT1` | Pin 5 — `P7.4 / CB8 / A12` (channel **A12**) via series `R5` | `C2K_AOUT1` via `R7` — no header pin (see note) |
| **Bridge Output Phase A2** | `VSEN_AOUT2` | Pin 6 — `P7.5 / CB9 / A13` (channel **A13**) via series `R8` | `C2K_AOUT2` via `R10` — no header pin (see note) |
| **Bridge Output Phase B1** | `VSEN_BOUT1` | Pin 7 — `P7.6 / CB10 / A14` (channel **A14**) via series `R13` | `C2K_BOUT1` via `R12` — no header pin (see note) |
| **Bridge Output Phase B2** | `VSEN_BOUT2` | Pin 8 — `P7.7 / CB11 / A15` (channel **A15**) via series `R16` | `C2K_BOUT2` via `R15` — no header pin (see note) |

*Netlist-derived (supersedes the earlier "channel TBD" note). Key points:*

* *`VSEN_VM` lands directly on `U1.1` = `P6.4` (channel `A4`), and also on C2000 `J1` Pin 3. The four IPROPI current-sense outputs occupy channels `A0`–`A3` (`P6.0`–`P6.3`, Section 1), so `A4` is the first free channel for `VSEN_VM` — consistent.*
* ***The four phase-output sense signals share the exact pins used by the Section 5.1 button test points.*** *`VSEN_AOUT1` / `VSEN_AOUT2` / `VSEN_BOUT1` / `VSEN_BOUT2` reach `P7.4`–`P7.7` (channels `A12`–`A15`) only through series resistors `R5` / `R8` / `R13` / `R16`, and those same nets (`A12`–`A15`) are the `A1`–`A4` test points. `R5`/`R8`/`R13`/`R16` (route to MSP430) and `R7`/`R10`/`R12`/`R15` (route to C2000) are a mutually-exclusive population option. **For GateController's button use of `A1`–`A4`, `R5`/`R8`/`R13`/`R16` must be unpopulated** — confirm against the assembly-variant BOM.*
* *In the netlist the `C2K_AOUT1` / `C2K_AOUT2` / `C2K_BOUT1` / `C2K_BOUT2` nets terminate only at `R7` / `R10` / `R12` / `R15` — no `J1`/`J2` header pin — so the C2000 breakout of these four phase-sense signals is not wired on this assembly.*

---

## 4. Onboard Emulation Interconnects (ezFET)
The micro-B USB connection routes through an independent isolation controller module (**U2, MSP430F5528**) acting as the onboard **ezFET** debug probe.
*   **Target UART Interface:**
    *   `EZFET_UARTTXD` ↔ `MCU_UARTRXD` (MSP430 Pin 61, `P8.3` via local logic paths)
    *   `EZFET_UARTRXD` ↔ `MCU_UARTTXD` (MSP430 Pin 60, `P8.2` via local logic paths)
*   **Spy-Bi-Wire Debug Interconnect:**
    *   `TST/SBWTCK` routes to target programming headers and `J27` Selection Block.
    *   `RST/SBWTDIO` routes directly to the primary reset network.

---

## 5. GateController Digital Inputs (Buttons & Limit Switches)

The GateController firmware repurposes several EVM test points and header jumpers as digital inputs (momentary buttons and limit switches). Unless noted otherwise, none of these breakouts has an external pull-up or pull-down resistor populated, so every input pin must be configured with the **MSP430's internal pull-up** enabled (`PxREN` set, `PxOUT` set) so the contact reads low when pressed / closed.

*Revision note: the `A2`, `A3`, and `A4` usages below were reassigned in this revision — the earlier OPEN / CLOSE / TOGGLE mapping on those three test points no longer applies.*

### 5.1 Port 7 ADC-channel test points (A1–A4)

These four test points break out `P7.4`–`P7.7` (ADC channels `A12`–`A15`). Port 7 supports the internal pull-up/pull-down natively (`P7REN`), so all four inputs are fine as-is with no external resistor needed.

*Hardware caveat (from the netlist): the same four nets (`A12`–`A15`) are also the analog phase-sense taps `VSEN_AOUT1` / `VSEN_AOUT2` / `VSEN_BOUT1` / `VSEN_BOUT2`, connected through series resistors `R5` / `R8` / `R13` / `R16`. For these pins to work as clean digital button inputs, `R5`/`R8`/`R13`/`R16` must be unpopulated (they are a mutually-exclusive option with the C2000-side `R7`/`R10`/`R12`/`R15`). See Section 3.*

*   **Test Point A4**
    *   **MSP430 Host MCU:** Pin 8, Port `P7.7/CB11/A15/DAC1` (Net `A15`)
    *   **GPIO Configuration:** Input, internal pull-up enabled (`P7REN.7` + `P7OUT.7`)
    *   **GateController Use:** TOGGLE button (Open / Stop / Close)
*   **Test Point A3**
    *   **MSP430 Host MCU:** Pin 7, Port `P7.6/CB10/A14/DAC0` (Net `A14`)
    *   **GPIO Configuration:** Input, internal pull-up enabled (`P7REN.6` + `P7OUT.6`)
    *   **GateController Use:** Arm Extended limit-switch input
*   **Test Point A2**
    *   **MSP430 Host MCU:** Pin 6, Port `P7.5/CB9/A13` (Net `A13`)
    *   **GPIO Configuration:** Input, internal pull-up enabled (`P7REN.5` + `P7OUT.5`)
    *   **GateController Use:** Arm Retracted limit-switch input
*   **Test Point A1**
    *   **MSP430 Host MCU:** Pin 5, Port `P7.4/CB8/A12` (Net `A12`)
    *   **GPIO Configuration:** Input, internal pull-up enabled (`P7REN.4` + `P7OUT.4`)
    *   **GateController Use:** REMOTE input — functionally identical to the TOGGLE button (Open / Stop / Close)

### 5.2 Additional test-point and header-jumper inputs

*   **TP13**
    *   **MSP430 Host MCU:** Pin 2, Port `P6.5 / CB5 / A5` (net `NetTP13_1`) — netlist-confirmed (`TP13.1` → `U1.2`).
    *   **GPIO Configuration:** Input, internal pull-up enabled (`P6REN.5` + `P6OUT.5`)
    *   **GateController Use:** OPEN button
*   **Bottom-header jumper, 4th position from the bottom** — net `RSVD/IPROPI1` (DRV8262 Pin 35, a RSVD pin on the driver)
    *   **Jumper Path:** `J4` (Pin 10 ↔ Pin 9), then series `R42` (1 kΩ) to net `MSP_RSVD/IPROPI1`
    *   **MSP430 Host MCU:** Pin 97, Port `P6.0 / CB0 / A0` — netlist-confirmed (`MSP_RSVD/IPROPI1` = `J4.9` / `R42` → `U1.97`). Same signal is listed in Section 1 as `IPROPI1 / RSVD`.
    *   **GPIO Configuration:** Input, internal pull-up enabled (`P6REN.0` + `P6OUT.0`). A 0.1 µF cap (`C11`) sits on this node to ground — mild RC filtering, fine for a button. Channel `A0` is **not** used by the `VSEN_*` telemetry (that starts at `A4`), so there is no analog-vs-digital conflict here.
    *   **GateController Use:** CLOSE button
*   **Top-header jumper, 2nd position from the bottom** — net `SDO/DECAY1/EN2/RSVD` (DRV8262 Pin 31, a RSVD pin on the driver)
    *   **Jumper Path:** `J8` (Pin 14 ↔ Pin 13) to net `MSP_DECAY1/SDO/RSVD/EN2`
    *   **MSP430 Host MCU:** Pin 66, Port `P8.6 / UCB1SOMI / UCB1SCL` — netlist-confirmed (`MSP_DECAY1/SDO/RSVD/EN2` = `J8.13` → `U1.66`).
    *   **GPIO Configuration:** Input, internal pull-up enabled (`P8REN.6` + `P8OUT.6`)
    *   **GateController Use:** TBD
*   **Tactile switch `S1`** is **not** available as a firmware input — it is wired through `J27` to the reset line (`RST/NMI/SBWTDIO`, Pin 96). See Section 2.

*Note: Test point silkscreen labels `A1`–`A4` are distinct from the MSP430 ADC channel names. Section 3's telemetry uses channels `A4` and `A12`–`A15`; each `A1`–`A4` test-point pin is itself ADC-capable (channels `A12`–`A15`).*

*Note: Ports P6, P7, and P8 (`P6.x`/`P7.x`/`P8.x`) do not support edge-triggered GPIO interrupts (`PxIFG`/`PxIE`) on this device — only Ports P1–P4 do. For inputs on these ports (all of Section 5.1, plus TP13 and both header jumpers in 5.2), the firmware must either move the signal to a P1–P4 pin or poll `PxIN` on a timer / ADC-conversion-complete cadence rather than relying on a pin interrupt.*