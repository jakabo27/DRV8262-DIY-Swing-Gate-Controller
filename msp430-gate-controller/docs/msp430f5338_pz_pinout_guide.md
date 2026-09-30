# MSP430F5338IPZ (100-Pin PZ Package) Pin Capabilities & Programming Reference

> **Device:** Texas Instruments MSP430F5338  
> **Package:** 100-Pin LQFP (PZ Package)  
> **Target Audience:** Embedded Software Engineers & AI Code Generation / LLM Context Prompts  
> **Document Purpose:** Complete pinout breakdown, internal resistor configuration, timer mapping, peripheral capability matrix, and programming code snippets.

---

## 1. Quick Overview & Hardware Capability Matrix

### General GPIO & Internal Resistor Rules
* **Pull-Up / Pull-Down Resistors:** **ALL** general-purpose digital I/O pins (Ports P1 through P9, PJ, and Port U) feature individually software-selectable internal **pull-up or pull-down resistors** (~20–50 kΩ).
  * Enable/Disable: Controlled by `PxREN` register.
  * Direction Selection (Up/Down): Controlled by `PxOUT` register (`1` = Pull-Up, `0` = Pull-Down).
* **Port Interrupts (PxIFG / PxIE):** **ONLY Ports P1, P2, P3, and P4** support external hardware GPIO interrupts with configurable edge detection (`PxIES`). Ports P5, P6, P7, P8, P9, PJ, and PU do **NOT** have edge-triggered GPIO interrupts.
* **Port Mapping Controller (PMAP):** **Port P2 (P2.0–P2.7)** features full reconfigurable pin mapping. Default functions can be dynamically remapped via the PMAP controller registers (`P2MAP0`–`P2MAP7`).
* **Drive Strength:** All GPIOs support full or reduced drive strength configuration via `PxDS` registers to mitigate EMI.

---

## 2. Pin-by-Pin Capability Table (Pins 1–100)

| Pin # | Pin Name | I/O | Internal Pull Up/Down | GPIO Interrupt? | Analog Capabilities | Timer Functions | Communication & System Functions | Notes & Remapping |
| :---: | :--- | :---: | :---: | :---: | :--- | :--- | :--- | :--- |
| **1** | P6.4/CB4/A4 | I/O | Yes (`P6REN.4`) | No | ADC12 Ch A4, Comp_B CB4 | — | — | — |
| **2** | P6.5/CB5/A5 | I/O | Yes (`P6REN.5`) | No | ADC12 Ch A5, Comp_B CB5 | — | — | — |
| **3** | P6.6/CB6/A6/DAC0 | I/O | Yes (`P6REN.6`) | No | ADC12 Ch A6, Comp_B CB6, DAC12_0 Output | — | — | — |
| **4** | P6.7/CB7/A7/DAC1 | I/O | Yes (`P6REN.7`) | No | ADC12 Ch A7, Comp_B CB7, DAC12_1 Output | — | — | — |
| **5** | P7.4/CB8/A12 | I/O | Yes (`P7REN.4`) | No | ADC12 Ch A12, Comp_B CB8 | — | — | — |
| **6** | P7.5/CB9/A13 | I/O | Yes (`P7REN.5`) | No | ADC12 Ch A13, Comp_B CB9 | — | — | — |
| **7** | P7.6/CB10/A14/DAC0 | I/O | Yes (`P7REN.6`) | No | ADC12 Ch A14, Comp_B CB10, DAC12_0 Output | — | — | — |
| **8** | P7.7/CB11/A15/DAC1 | I/O | Yes (`P7REN.7`) | No | ADC12 Ch A15, Comp_B CB11, DAC12_1 Output | — | — | — |
| **9** | P5.0/VREF+/VeREF+ | I/O | Yes (`P5REN.0`) | No | ADC Ext. Ref Output/Input (+) | — | — | Shared with ADC Reference |
| **10** | P5.1/VREF-/VeREF- | I/O | Yes (`P5REN.1`) | No | ADC Ext. Ref Output/Input (-) | — | — | Shared with ADC Reference |
| **11** | AVCC1 | Power | No | No | Analog Power Supply (3.3V) | — | — | Connect to 1.8V–3.6V (filtered) |
| **12** | AVSS1 | Ground | No | No | Analog Ground | — | — | Ground reference |
| **13** | XIN | In | No | No | — | — | XT1 Low/High Freq Crystal Input | Low/High Freq Osc input |
| **14** | XOUT | Out | No | No | — | — | XT1 Low/High Freq Crystal Output | Low/High Freq Osc output |
| **15** | AVSS2 | Ground | No | No | Analog Ground | — | — | Ground reference |
| **16** | P5.6/ADC12CLK/DMAE0 | I/O | Yes (`P5REN.6`) | No | ADC12 Conversion Clock Output | — | DMA Ext. Trigger 0 Input (`DMAE0`) | Multi-function GPIO |
| **17** | P2.0/P2MAP0 | I/O | Yes (`P2REN.0`) | **Yes** | — | — | Default: USCI_B0 STE / USCI_A0 CLK | **Mappable (P2MAP0)** |
| **18** | P2.1/P2MAP1 | I/O | Yes (`P2REN.1`) | **Yes** | — | — | Default: USCI_B0 SIMO/SDA | **Mappable (P2MAP1)** |
| **19** | P2.2/P2MAP2 | I/O | Yes (`P2REN.2`) | **Yes** | — | — | Default: USCI_B0 SOMI/SCL | **Mappable (P2MAP2)** |
| **20** | P2.3/P2MAP3 | I/O | Yes (`P2REN.3`) | **Yes** | — | — | Default: USCI_B0 CLK / USCI_A0 STE | **Mappable (P2MAP3)** |
| **21** | P2.4/P2MAP4 | I/O | Yes (`P2REN.4`) | **Yes** | — | — | Default: USCI_A0 TXD / SIMO | **Mappable (P2MAP4)** |
| **22** | P2.5/P2MAP5 | I/O | Yes (`P2REN.5`) | **Yes** | — | — | Default: USCI_A0 RXD / SOMI | **Mappable (P2MAP5)** |
| **23** | P2.6/P2MAP6 | I/O | Yes (`P2REN.6`) | **Yes** | — | — | Default: None | **Mappable (P2MAP6)** |
| **24** | P2.7/P2MAP7 | I/O | Yes (`P2REN.7`) | **Yes** | — | — | Default: None | **Mappable (P2MAP7)** |
| **25** | DVCC1 | Power | No | No | Digital Power Supply | — | — | Main VCC supply |
| **26** | DVSS1 | Ground | No | No | Digital Ground | — | — | Main Ground |
| **27** | VCORE | Power | No | No | Core Regulated Voltage | — | — | Connect 470nF cap to ground |
| **28** | P5.2 | I/O | Yes (`P5REN.2`) | No | — | — | — | General Purpose Digital I/O |
| **29** | DVSS | Ground | No | No | Digital Ground | — | — | Ground reference |
| **30** | DNC | — | No | No | — | — | Do Not Connect | Leave pin open |
| **31** | P5.3 | I/O | Yes (`P5REN.3`) | No | — | — | — | General Purpose Digital I/O |
| **32** | P5.4 | I/O | Yes (`P5REN.4`) | No | — | — | — | General Purpose Digital I/O |
| **33** | P5.5 | I/O | Yes (`P5REN.5`) | No | — | — | — | General Purpose Digital I/O |
| **34** | P1.0/TA0CLK/ACLK | I/O | Yes (`P1REN.0`) | **Yes** | — | TA0 Clock Input (`TA0CLK`) | ACLK Output | System Clock Out / Timer In |
| **35** | P1.1/TA0.0 | I/O | Yes (`P1REN.1`) | **Yes** | — | TA0 CCR0 Cap/Comp (`TA0.0`) | BSL Transmit Output (`BSL_TX`) | Bootstrap Loader TX |
| **36** | P1.2/TA0.1 | I/O | Yes (`P1REN.2`) | **Yes** | — | TA0 CCR1 Cap/Comp (`TA0.1`) | BSL Receive Input (`BSL_RX`) | Bootstrap Loader RX |
| **37** | P1.3/TA0.2 | I/O | Yes (`P1REN.3`) | **Yes** | — | TA0 CCR2 Cap/Comp (`TA0.2`) | — | — |
| **38** | P1.4/TA0.3 | I/O | Yes (`P1REN.4`) | **Yes** | — | TA0 CCR3 Cap/Comp (`TA0.3`) | — | — |
| **39** | P1.5/TA0.4 | I/O | Yes (`P1REN.5`) | **Yes** | — | TA0 CCR4 Cap/Comp (`TA0.4`) | — | — |
| **40** | P1.6/TA0.1 | I/O | Yes (`P1REN.6`) | **Yes** | — | TA0 CCR1 Cap/Comp (`TA0.1`) | — | Alternate PWM Output |
| **41** | P1.7/TA0.2 | I/O | Yes (`P1REN.7`) | **Yes** | — | TA0 CCR2 Cap/Comp (`TA0.2`) | — | Alternate PWM Output |
| **42** | P3.0/TA1CLK/CBOUT | I/O | Yes (`P3REN.0`) | **Yes** | Comp_B Output (`CBOUT`) | TA1 Clock Input (`TA1CLK`) | — | Timer Clock / Comp Out |
| **43** | P3.1/TA1.0 | I/O | Yes (`P3REN.1`) | **Yes** | — | TA1 CCR0 Cap/Comp (`TA1.0`) | — | — |
| **44** | P3.2/TA1.1 | I/O | Yes (`P3REN.2`) | **Yes** | — | TA1 CCR1 Cap/Comp (`TA1.1`) | — | — |
| **45** | P3.3/TA1.2 | I/O | Yes (`P3REN.3`) | **Yes** | — | TA1 CCR2 Cap/Comp (`TA1.2`) | — | — |
| **46** | P3.4/TA2CLK/SMCLK | I/O | Yes (`P3REN.4`) | **Yes** | — | TA2 Clock Input (`TA2CLK`) | SMCLK Output | System Clock Out / Timer In |
| **47** | P3.5/TA2.0 | I/O | Yes (`P3REN.5`) | **Yes** | — | TA2 CCR0 Cap/Comp (`TA2.0`) | — | — |
| **48** | P3.6/TA2.1 | I/O | Yes (`P3REN.6`) | **Yes** | — | TA2 CCR1 Cap/Comp (`TA2.1`) | — | — |
| **49** | P3.7/TA2.2 | I/O | Yes (`P3REN.7`) | **Yes** | — | TA2 CCR2 Cap/Comp (`TA2.2`) | — | — |
| **50** | P4.0/TB0.0 | I/O | Yes (`P4REN.0`) | **Yes** | — | TB0 CCR0 Cap/Comp (`TB0.0`) | — | Timer_B Output |
| **51** | P4.1/TB0.1 | I/O | Yes (`P4REN.1`) | **Yes** | — | TB0 CCR1 Cap/Comp (`TB0.1`) | — | Timer_B Output |
| **52** | P4.2/TB0.2 | I/O | Yes (`P4REN.2`) | **Yes** | — | TB0 CCR2 Cap/Comp (`TB0.2`) | — | Timer_B Output |
| **53** | P4.3/TB0.3 | I/O | Yes (`P4REN.3`) | **Yes** | — | TB0 CCR3 Cap/Comp (`TB0.3`) | — | Timer_B Output |
| **54** | P4.4/TB0.4 | I/O | Yes (`P4REN.4`) | **Yes** | — | TB0 CCR4 Cap/Comp (`TB0.4`) | — | Timer_B Output |
| **55** | P4.5/TB0.5 | I/O | Yes (`P4REN.5`) | **Yes** | — | TB0 CCR5 Cap/Comp (`TB0.5`) | — | Timer_B Output |
| **56** | P4.6/TB0.6 | I/O | Yes (`P4REN.6`) | **Yes** | — | TB0 CCR6 Cap/Comp (`TB0.6`) | — | Timer_B Output |
| **57** | P4.7/TB0OUTH/SVMOUT| I/O | Yes (`P4REN.7`) | **Yes** | — | TB0 High Impedance (`TB0OUTH`) | SVM Output (`SVMOUT`) | Supply Voltage Monitor Out |
| **58** | P8.0/TB0CLK | I/O | Yes (`P8REN.0`) | No | — | TB0 Clock Input (`TB0CLK`) | — | Timer_B External Clock |
| **59** | P8.1/UCB1STE/UCA1CLK| I/O | Yes (`P8REN.1`) | No | — | — | USCI_B1 SPI STE / USCI_A1 CLK | SPI Slave Enable / UART Clock |
| **60** | P8.2/UCA1TXD/UCA1SIMO| I/O| Yes (`P8REN.2`) | No | — | — | USCI_A1 TXD (UART) / SIMO (SPI) | Dedicated USCI_A1 TX |
| **61** | P8.3/UCA1RXD/UCA1SOMI| I/O| Yes (`P8REN.3`) | No | — | — | USCI_A1 RXD (UART) / SOMI (SPI) | Dedicated USCI_A1 RX |
| **62** | P8.4/UCB1CLK/UCA1STE| I/O | Yes (`P8REN.4`) | No | — | — | USCI_B1 CLK / USCI_A1 STE | SPI Clock / STE |
| **63** | DVSS2 | Ground | No | No | Digital Ground | — | — | Ground reference |
| **64** | DVCC2 | Power | No | No | Digital Power Supply | — | — | Power supply rail |
| **65** | P8.5/UCB1SIMO/UCB1SDA| I/O| Yes (`P8REN.5`) | No | — | — | USCI_B1 SIMO (SPI) / SDA (I2C) | Dedicated USCI_B1 I2C Data |
| **66** | P8.6/UCB1SOMI/UCB1SCL| I/O| Yes (`P8REN.6`) | No | — | — | USCI_B1 SOMI (SPI) / SCL (I2C) | Dedicated USCI_B1 I2C Clock |
| **67** | P8.7 | I/O | Yes (`P8REN.7`) | No | — | — | — | General Purpose Digital I/O |
| **68** | P9.0 | I/O | Yes (`P9REN.0`) | No | — | — | — | General Purpose Digital I/O |
| **69** | P9.1 | I/O | Yes (`P9REN.1`) | No | — | — | — | General Purpose Digital I/O |
| **70** | P9.2 | I/O | Yes (`P9REN.2`) | No | — | — | — | General Purpose Digital I/O |
| **71** | P9.3 | I/O | Yes (`P9REN.3`) | No | — | — | — | General Purpose Digital I/O |
| **72** | P9.4 | I/O | Yes (`P9REN.4`) | No | — | — | — | General Purpose Digital I/O |
| **73** | P9.5 | I/O | Yes (`P9REN.5`) | No | — | — | — | General Purpose Digital I/O |
| **74** | P9.6 | I/O | Yes (`P9REN.6`) | No | — | — | — | General Purpose Digital I/O |
| **75** | P9.7 | I/O | Yes (`P9REN.7`) | No | — | — | — | General Purpose Digital I/O |
| **76** | VSSU | Ground | No | No | Port U / LDO Ground | — | — | LDO Ground |
| **77** | PU.0 | I/O | Yes (`PUCTL`) | No | — | — | Port U Digital I/O 0 | Powered by LDOO rail |
| **78** | NC | — | No | No | — | — | No Connect | Leave unconnected |
| **79** | PU.1 | I/O | Yes (`PUCTL`) | No | — | — | Port U Digital I/O 1 | Powered by LDOO rail |
| **80** | LDOI | Power | No | No | — | — | Integrated LDO Input | Up to 5.5V LDO input |
| **81** | LDOO | Power | No | No | — | — | Integrated LDO Output | 3.3V Regulated Output |
| **82** | NC | — | No | No | — | — | No Connect | Leave unconnected |
| **83** | AVSS3 | Ground | No | No | Analog Ground | — | — | Ground reference |
| **84** | P7.2/XT2IN | I/O | Yes (`P7REN.2`) | No | — | — | XT2 High Freq Crystal Input | High Freq Osc input |
| **85** | P7.3/XT2OUT | I/O | Yes (`P7REN.3`) | No | — | — | XT2 High Freq Crystal Output | High Freq Osc output |
| **86** | VBAK | Power | No | No | — | — | Backup Subsystem Capacitor Pin | Connect capacitor CBAK |
| **87** | VBAT | Power | No | No | — | — | Backup Domain Battery Power Input| RTC backup power |
| **88** | P5.7/RTCCLK | I/O | Yes (`P5REN.7`) | No | — | — | Real-Time Clock Output (`RTCCLK`) | RTC Clock Out |
| **89** | DVCC3 | Power | No | No | Digital Power Supply | — | — | Power supply rail |
| **90** | DVSS3 | Ground | No | No | Digital Ground | — | — | Ground reference |
| **91** | TEST/SBWTCK | In | Pull-Down | No | — | — | JTAG Test Mode / Spy-Bi-Wire Clock| Programming pin |
| **92** | PJ.0/TDO | I/O | Yes (`PJREN.0`) | No | — | — | JTAG Test Data Out | Port J Digital I/O |
| **93** | PJ.1/TDI/TCLK | I/O | Yes (`PJREN.1`) | No | — | — | JTAG Test Data In / Clock | Port J Digital I/O |
| **94** | PJ.2/TMS | I/O | Yes (`PJREN.2`) | No | — | — | JTAG Test Mode Select | Port J Digital I/O |
| **95** | PJ.3/TCK | I/O | Yes (`PJREN.3`) | No | — | — | JTAG Test Clock | Port J Digital I/O |
| **96** | RST/NMI/SBWTDIO| I/O | **Pull-Up** | No | — | — | Reset / Non-Maskable Interrupt / SBW Data | System Reset (Active Low) |
| **97** | P6.0/CB0/A0 | I/O | Yes (`P6REN.0`) | No | ADC12 Ch A0, Comp_B CB0 | — | — | — |
| **98** | P6.1/CB1/A1 | I/O | Yes (`P6REN.1`) | No | ADC12 Ch A1, Comp_B CB1 | — | — | — |
| **99** | P6.2/CB2/A2 | I/O | Yes (`P6REN.2`) | No | ADC12 Ch A2, Comp_B CB2 | — | — | — |
| **100**| P6.3/CB3/A3 | I/O | Yes (`P6REN.3`) | No | ADC12 Ch A3, Comp_B CB3 | — | — | — |

---

## 3. Peripheral Cross-Reference & Grouping

### A. Timer Module Pin Allocations
The MSP430F5338 has four 16-bit Timer modules (TA0, TA1, TA2, TB0):

1. **Timer_A0 (TA0):** 5 Capture/Compare Registers + Clock Input
   * `TA0CLK`: Pin 34 (`P1.0`)
   * `TA0.0`: Pin 35 (`P1.1`)
   * `TA0.1`: Pin 36 (`P1.2`) & Pin 40 (`P1.6`)
   * `TA0.2`: Pin 37 (`P1.3`) & Pin 41 (`P1.7`)
   * `TA0.3`: Pin 38 (`P1.4`)
   * `TA0.4`: Pin 39 (`P1.5`)
2. **Timer_A1 (TA1):** 3 Capture/Compare Registers + Clock Input
   * `TA1CLK`: Pin 42 (`P3.0`)
   * `TA1.0`: Pin 43 (`P3.1`)
   * `TA1.1`: Pin 44 (`P3.2`)
   * `TA1.2`: Pin 45 (`P3.3`)
3. **Timer_A2 (TA2):** 3 Capture/Compare Registers + Clock Input
   * `TA2CLK`: Pin 46 (`P3.4`)
   * `TA2.0`: Pin 47 (`P3.5`)
   * `TA2.1`: Pin 48 (`P3.6`)
   * `TA2.2`: Pin 49 (`P3.7`)
4. **Timer_B0 (TB0):** 7 Capture/Compare Registers + Clock Input
   * `TB0CLK`: Pin 58 (`P8.0`)
   * `TB0.0`: Pin 50 (`P4.0`)
   * `TB0.1`: Pin 51 (`P4.1`)
   * `TB0.2`: Pin 52 (`P4.2`)
   * `TB0.3`: Pin 53 (`P4.3`)
   * `TB0.4`: Pin 54 (`P4.4`)
   * `TB0.5`: Pin 55 (`P4.5`)
   * `TB0.6`: Pin 56 (`P4.6`)
   * `TB0OUTH`: Pin 57 (`P4.7`)

---

### B. Serial Communication Interfaces (USCI)

#### USCI_A0 & USCI_B0 (Port-Mapped on Port P2)
Port P2 features the **Port Mapping Controller (PMAP)**. By default, USCI_A0 and USCI_B0 are mapped as follows:
* **USCI_A0:**
  * `UCA0TXD` / `UCA0SIMO`: Pin 21 (`P2.4`)
  * `UCA0RXD` / `UCA0SOMI`: Pin 22 (`P2.5`)
  * `UCA0CLK`: Pin 17 (`P2.0`)
  * `UCA0STE`: Pin 20 (`P2.3`)
* **USCI_B0:**
  * `UCB0SIMO` / `UCB0SDA`: Pin 18 (`P2.1`)
  * `UCB0SOMI` / `UCB0SCL`: Pin 19 (`P2.2`)
  * `UCB0CLK`: Pin 20 (`P2.3`)
  * `UCB0STE`: Pin 17 (`P2.0`)

#### USCI_A1 & USCI_B1 (Fixed Pins on Port P8)
* **USCI_A1:**
  * `UCA1TXD` / `UCA1SIMO`: Pin 60 (`P8.2`)
  * `UCA1RXD` / `UCA1SOMI`: Pin 61 (`P8.3`)
  * `UCA1CLK`: Pin 59 (`P8.1`)
  * `UCA1STE`: Pin 62 (`P8.4`)
* **USCI_B1:**
  * `UCB1SIMO` / `UCB1SDA`: Pin 65 (`P8.5`)
  * `UCB1SOMI` / `UCB1SCL`: Pin 66 (`P8.6`)
  * `UCB1CLK`: Pin 62 (`P8.4`)
  * `UCB1STE`: Pin 59 (`P8.1`)

---

### C. Analog Peripherals (ADC12, DAC12, Comparator_B)

* **ADC12_A Inputs:**
  * `A0`–`A3`: Pins 97, 98, 99, 100 (`P6.0`–`P6.3`)
  * `A4`–`A7`: Pins 1, 2, 3, 4 (`P6.4`–`P6.7`)
  * `A12`–`A15`: Pins 5, 6, 7, 8 (`P7.4`–`P7.7`)
  * Ext. Reference (+/-): Pins 9 (`P5.0`), 10 (`P5.1`)
  * ADC Clock Output: Pin 16 (`P5.6`)
* **DAC12 Outputs:**
  * `DAC12.0`: Pin 3 (`P6.6`) or Pin 7 (`P7.6`)
  * `DAC12.1`: Pin 4 (`P6.7`) or Pin 8 (`P7.7`)
* **Comparator_B Inputs:**
  * `CB0`–`CB3`: Pins 97, 98, 99, 100 (`P6.0`–`P6.3`)
  * `CB4`–`CB7`: Pins 1, 2, 3, 4 (`P6.4`–`P6.7`)
  * `CB8`–`CB11`: Pins 5, 6, 7, 8 (`P7.4`–`P7.7`)
  * `CBOUT`: Pin 42 (`P3.0`)

---

## 4. Programming Code Snippets & AI Context Rules

### A. How to Configure Pull-Up / Pull-Down Resistors (Bare-Metal C)

```c
#include <msp430.h>

void setup_gpio(void) {
    // Example: Configure P1.1 as Input with Internal Pull-Up Resistor
    P1DIR &= ~BIT1;             // Set P1.1 as Input (0 = Input, 1 = Output)
    P1REN |= BIT1;              // Enable Internal Resistor on P1.1
    P1OUT |= BIT1;              // Select Pull-Up (1 = Pull-Up, 0 = Pull-Down)

    // Example: Configure P1.2 as Input with Internal Pull-Down Resistor
    P1DIR &= ~BIT2;             // Set P1.2 as Input
    P1REN |= BIT2;              // Enable Internal Resistor on P1.2
    P1OUT &= ~BIT2;             // Select Pull-Down

    // Example: Configure P4.0 as Output Driven High
    P4DIR |= BIT0;              // Set P4.0 as Output
    P4OUT |= BIT0;              // Drive High
}
```

### B. How to Configure External Hardware Interrupts (Port P1–P4 Only)

```c
void setup_p1_interrupt(void) {
    P1DIR &= ~BIT3;             // P1.3 as Input
    P1REN |= BIT3;              // Enable pull resistor
    P1OUT |= BIT3;              // Pull-up resistor

    P1IES |= BIT3;              // Interrupt Edge Select: High-to-Low transition
    P1IFG &= ~BIT3;             // Clear interrupt flag before enabling
    P1IE  |= BIT3;              // Enable P1.3 interrupt

    __enable_interrupt();       // Enable global interrupts
}

// Port 1 Interrupt Service Routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void) {
    switch(__even_in_range(P1IV, P1IV_P1IFG7)) {
        case P1IV_P1IFG3:      // P1.3 triggered
            // User code here
            break;
        default:
            break;
    }
}
```

### C. Port Mapping Controller (PMAP) Unlock & Reconfiguration

```c
void remap_p2_pins(void) {
    PMAPKEYID = PMAPKEY;        // Unlock Port Mapping Controller

    // Example: Remap P2.0 to PM_UCA0TXD and P2.1 to PM_UCA0RXD
    P2MAP0 = PM_UCA0TXD;
    P2MAP1 = PM_UCA0RXD;

    PMAPKEYID = 0;              // Lock Port Mapping Controller

    // Enable Secondary Pin Function on P2.0 & P2.1
    P2SEL |= (BIT0 | BIT1);
}
```

---

## 5. Summary Rules for AI Code Generators
When prompting AI models (e.g. Copilot, ChatGPT, Claude) to write MSP430F5338 C code, enforce these guidelines:
1. **Interrupt Availability:** NEVER attempt to enable GPIO interrupts (`PxIE`) on Ports P5, P6, P7, P8, P9, PJ, or PU. Only `P1`, `P2`, `P3`, and `P4` have hardware interrupt capability.
2. **Pull Resistor Setup:** Always set both `PxREN` (Enable) and `PxOUT` (Direction select) when configuring internal pull-up/pull-down resistors.
3. **Secondary Functions:** Set `PxSEL` bit to `1` to activate peripheral functions (UART, SPI, Timers, ADC, etc.) instead of standard GPIO mode.
4. **Port Mapping Lock:** Port P2 remapping requires writing `PMAPKEY` (0x2D52) to `PMAPKEYID` before modifying `P2MAPx` registers, and clearing `PMAPKEYID` afterward.
5. **VCORE Capacitor:** Ensure `VCORE` (Pin 27) is never connected to external power supplies; it requires a 470 nF decoupling capacitor to ground.
