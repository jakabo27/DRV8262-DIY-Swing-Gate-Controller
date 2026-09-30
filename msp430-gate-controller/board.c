/*******************************************************************************
 * board.c - see board.h. RST/NMI (pin 96) is left completely untouched here -
 * no SYSNMI/NMIIE code anywhere in this project - so it stays a normal
 * hardware reset (project_plan.md Rev 2 Sec 1.1 Q3).
 ******************************************************************************/
#include <msp430.h>
#include "board.h"
#include "gate_config.h"

volatile uint8_t  g_tickFlag  = 0;
volatile uint32_t g_tickCount = 0;

static tick_rate_t     s_tickRate = TICK_ACTIVE;
static bool             s_tickRateInited = false;
static fault_led_pattern_t s_faultPattern = FAULT_LED_OFF;
static uint16_t         s_heartbeatAccumMs = 0;   /* ISR-owned: heartbeat is tick-driven, not FSM-driven */
static uint16_t         s_faultBlinkAccumMs = 0;  /* fault-LED blink phase, in real ms */
static volatile uint32_t s_uptimeMs = 0;          /* ISR-owned: real ms since boot */

/* Port 1 bit assignments (LEDs + nHOME/OCPM + nSLEEP) */
#define LED_HEARTBEAT_BIT     BIT0   /* P1.0 D6 */
/* P1.1 (was D8 GUI_STATUS) and P1.2 (was D9 DRV_ENABLE) are the MSP430 ROM-BSL
 * UART pins (P1.1 = BSL TX, P1.2 = BSL RX) and are wired to the ESP32 BSL
 * programmer. They are left as high-Z inputs - firmware must NOT drive them, or
 * it fights the ESP32's line driver. D8/D9 retired; R113/R115 depopulated on
 * this assembly so the LEDs no longer load the BSL lines. */
#define LED_LIMIT_OPEN_BIT    BIT3   /* P1.3 D10 */
#define NHOME_OCPM_BIT        BIT4   /* P1.4 - input, Hi-Z (OCPM auto-retry select) */
#define LED_BUTTON_BIT        BIT5   /* P1.5 D11 */
#define LED_FAULT_BIT         BIT6   /* P1.6 D12 */
#define NSLEEP_BIT            BIT7   /* P1.7 */

/* Port 2 (TOFF network + nFAULT) */
#define TOFF_CTRL_BIT         BIT5   /* P2.5 - driven                          */
#define TOFF_PD_BIT           BIT6   /* P2.6 - left as input; on-board bias tap */
#define NFAULT_BIT            BIT7   /* P2.7 - input                           */

/* Port 4 (motor PWM legs; rest unused) */
#define IN1_BIT               BIT1   /* P4.1 / TB0.1 */
#define IN2_BIT               BIT2   /* P4.2 / TB0.2 */

/* Port 6 (MODE2, IPROPI1/2, VM, OPEN/CLOSE buttons, VREF drive) */
#define BTN_CLOSE_BIT         BIT0   /* P6.0 */
#define IPROPI1_BIT           BIT1   /* P6.1 / A1 - ADC input, NO pull (discrete R_IPROPI on EVM) */
#define IPROPI2_BIT           BIT2   /* P6.2 / A2 - ADC input, NO pull (discrete R_IPROPI on EVM) */
#define MODE2_BIT             BIT3   /* P6.3 - GPIO, deliberately NOT in P6SEL  */
#define VM_SENSE_BIT          BIT4   /* P6.4 / A4 */
#define BTN_OPEN_BIT          BIT5   /* P6.5 */
/* P6.6 (DAC0 net -> J4/J26 VREF path, unused on this build) is driven HIGH as
 * plain GPIO - a fixed ~3.3 V reference for whichever EVM VREF-select jumper
 * would land there. P6.7 (net -> J6 "MSP_VREF/VREF2" path, the one actually
 * jumpered - J25+J6) is DAC12_1, owned by drv8262.c and runtime-adjustable
 * (VREF_CURRENT_REGULATION) when VREF_DAC_ENABLE (gate_config.h) is 1. While
 * that's 0 (current default), P6.7 is driven HIGH here too - the original
 * fixed ~3.3 V behavior, byte-for-byte. See drv8262.c file header. */
#if VREF_DAC_ENABLE
#define VREF_DRIVE_BITS      BIT6            /* P6.6 only - P6.7 is DAC12_1 */
#else
#define VREF_DRIVE_BITS      (BIT6 | BIT7)   /* both fixed GPIO-high - DAC path disabled */
#endif

/* Port 7 (limit switches, TOGGLE, REMOTE) */
#define LIM_REMOTE_BIT        BIT4   /* P7.4 REMOTE */
#define LIM_OPEN_BIT          BIT5   /* P7.5 Arm-Retracted = OPEN */
#define LIM_CLOSED_BIT        BIT6   /* P7.6 Arm-Extended  = CLOSED */
#define BTN_TOGGLE_BIT        BIT7   /* P7.7 TOGGLE */

/* Port 8 (UART, DECAY0, MODE1, photo beam) */
#define UART_TXD_BIT          BIT2   /* P8.2 UCA1TXD */
#define UART_RXD_BIT          BIT3   /* P8.3 UCA1RXD */
#define DECAY0_BIT            BIT5   /* P8.5 */
#define PHOTOBEAM_BIT         BIT6   /* P8.6 - input, internal pull per PHOTOBEAM_PULL_UP (default UP) */
#define MODE1_BIT             BIT7   /* P8.7 */

/* Bounded poll of a PMM SVS/SVM delay flag, used by setVCoreUp() below. The
 * un-bounded while() spins this replaces (from the TI reference) can wedge the
 * CPU forever on a marginal cold-supply ramp - and board_Init() now arms the
 * watchdog *before* this runs, so a timeout here simply lets that watchdog
 * reset and retry instead of the board sitting dark. ~20000 iterations is
 * >100x the ~150 us max SVS settle even at a slow bring-up MCLK, and always
 * well inside the 250 ms watchdog. */
static bool pmmWait(uint16_t flag)
{
    uint16_t to = 0;
    while ((PMMIFG & flag) == 0u) {
        if (++to >= 20000u) { return false; }
    }
    return true;
}

/* Step VCORE up exactly one PMMCOREV level. Adapted from TI's F5xx_F6xx_Core_Lib
 * HAL_PMM.c reference (the one shipped in DriverLib/MSP430Ware), NOT the
 * simplified sequence printed in the Family Guide (SLAU208) appendix - that
 * simplified version is a documented real hang risk in the field (TI E2E:
 * "SetVCoreUp() function included in Family Guide hangs on SVSMHDLYIFG",
 * seen on F5172/F5310/F67771A) because it never masks PMM interrupts or
 * clears stale PMMIFG flags before polling them, so a flag can be consumed
 * by hardware/an ISR before the while() ever sees it set - an infinite hang
 * at boot, which is worse than the un-stepped single-level jump this
 * replaces. This version backs up/masks PMMRIE and clears PMMIFG up front,
 * exactly as the reference does; the delay-flag polls are additionally
 * bounded via pmmWait() so a stuck flag can't hang boot. */
static bool setVCoreUp(uint8_t level)
{
    uint16_t pmmRieBackup, svsmhCtlBackup, svsmlCtlBackup;

    PMMCTL0_H = PMMPW_H;                                       /* unlock PMM regs */
    pmmRieBackup = PMMRIE;
    PMMRIE &= (uint16_t)~(SVMHVLRPE | SVSHPE | SVMLVLRPE | SVSLPE | SVMHVLRIE |
                          SVMHIE | SVSMHDLYIE | SVMLVLRIE | SVMLIE | SVSMLDLYIE);
    svsmhCtlBackup = SVSMHCTL;
    svsmlCtlBackup = SVSMLCTL;
    PMMIFG = 0;

    SVSMHCTL = SVMHE | SVSHE | (uint16_t)(SVSMHRRL0 * level);
    if (!pmmWait(SVSMHDLYIFG)) { PMMRIE = pmmRieBackup; PMMCTL0_H = 0x00; return false; }
    PMMIFG &= (uint16_t)~SVSMHDLYIFG;
    if ((PMMIFG & SVMHIFG) == SVMHIFG) {
        /* requested level isn't reachable (SVM high-side fault) - roll back */
        SVSMHCTL = svsmhCtlBackup;
        (void)pmmWait(SVSMHDLYIFG);
        PMMIFG &= (uint16_t)~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
        PMMRIE = pmmRieBackup;
        PMMCTL0_H = 0x00;
        return false;
    }

    SVSMHCTL |= (uint16_t)(SVSHRVL0 * level);
    if (!pmmWait(SVSMHDLYIFG)) { PMMRIE = pmmRieBackup; PMMCTL0_H = 0x00; return false; }
    PMMIFG &= (uint16_t)~SVSMHDLYIFG;

    PMMCTL0_L = (uint8_t)(PMMCOREV0 * level);                  /* commit the new VCore level */

    SVSMLCTL = SVMLE | (uint16_t)(SVSMLRRL0 * level) | SVSLE | (uint16_t)(SVSLRVL0 * level);
    if (!pmmWait(SVSMLDLYIFG)) { PMMRIE = pmmRieBackup; PMMCTL0_H = 0x00; return false; }
    PMMIFG &= (uint16_t)~SVSMLDLYIFG;

    /* restore whichever of the backed-up SVS/SVM settings weren't part of the
     * level fields just programmed (matches the reference bit-for-bit) */
    SVSMLCTL &= (uint16_t)(SVSLRVL0 | SVSLRVL1 | SVSMLRRL0 | SVSMLRRL1 | SVSMLRRL2);
    svsmlCtlBackup &= (uint16_t)~(SVSLRVL0 | SVSLRVL1 | SVSMLRRL0 | SVSMLRRL1 | SVSMLRRL2);
    SVSMLCTL |= svsmlCtlBackup;
    SVSMHCTL &= (uint16_t)(SVSHRVL0 | SVSHRVL1 | SVSMHRRL0 | SVSMHRRL1 | SVSMHRRL2);
    svsmhCtlBackup &= (uint16_t)~(SVSHRVL0 | SVSHRVL1 | SVSMHRRL0 | SVSMHRRL1 | SVSMHRRL2);
    SVSMHCTL |= svsmhCtlBackup;

    {
        uint16_t to = 0;
        while (((PMMIFG & SVSMLDLYIFG) == 0) && ((PMMIFG & SVSMHDLYIFG) == 0)) {
            if (++to >= 20000u) { break; }
        }
    }
    PMMIFG &= (uint16_t)~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
    PMMRIE = pmmRieBackup;
    PMMCTL0_H = 0x00;                                          /* re-lock PMM regs */
    return true;
}

static void setVCore(uint8_t level)
{
    uint8_t actLevel = (uint8_t)(PMMCTL0 & PMMCOREV_3);
    bool ok = true;
    level = (uint8_t)(level & PMMCOREV_3);
    while (ok && actLevel < level) {
        ok = setVCoreUp((uint8_t)(actLevel + 1u));
        actLevel = (uint8_t)(PMMCTL0 & PMMCOREV_3);
    }
}

static void clock_Init(void)
{
    /* Move ACLK onto REFO and clear the power-up oscillator faults BEFORE the
     * PMM step below. The watchdog armed at the top of board_Init() is
     * ACLK-sourced, and ACLK powers up on the (unpopulated) XT1 - getting it
     * onto REFO now means that if setVCore() ever stalls on a marginal cold
     * supply ramp, ACLK is still counting and the watchdog can actually fire.
     * MCLK/SMCLK reset to DCOCLKDIV (never XT1), so they need nothing here. */
    UCSCTL3  = SELREF_2;                   /* DCO FLL reference = REFO       */
    /* Direct assignment, not |=: UCSCTL4 resets to a nonzero value (SELM/SELS
     * default to DCOCLKDIV) - ORing SELS_3 into that leftover SELS field would
     * produce SELS_7 (XT2CLK), which is unpopulated on this board and trips
     * an oscillator fault. */
    UCSCTL4  = SELA_2 | SELS_3 | SELM_3;   /* ACLK = REFO, SMCLK = MCLK = DCO */
    UCSCTL7 &= (uint16_t)~(XT2OFFG | XT1LFOFFG | XT1HFOFFG | DCOFFG);
    SFRIFG1 &= (uint16_t)~OFIFG;

    setVCore(3);                           /* core voltage 1.85 V, stepped 0->1->2->3, before MCLK > 16 MHz */

    /* Canonical MSP430F5xx FLL bring-up (SLAU208 UCS chapter + TI UCS code
     * examples). The previous version skipped every step here and trusted a
     * fixed __delay_cycles() with no lock check - on a cold power-up that could
     * leave the DCO parked near the top of the DCORSEL_6 band (~39 MHz, ~2x the
     * F5338 ceiling at VCORE=3), which is the too-fast heartbeat and the
     * runs-a-few-seconds-then-dark cold boot.
     *   1. hold the FLL off (SCG0) while its control registers change
     *   2. force the DCO to its lowest tap so it can only slew UP into lock
     *   3. release the FLL and wait the worst-case settle time
     *   4. clear the latched osc-fault flags and don't leave clock_Init() until
     *      OFIFG stays clear (bounded - a truly dead clock can't wedge us here,
     *      the watchdog is the final backstop). */
    __bis_SR_register(SCG0);
    UCSCTL0  = 0x0000;                     /* DCO = 0, MOD = 0              */
    UCSCTL1  = DCORSEL_6;                  /* DCO range containing 20 MHz   */
    UCSCTL2  = FLLD_0 | 609;               /* FLLD /1, FLLN 609 -> 610 * 32768 ~= 20 MHz */
    __bic_SR_register(SCG0);

    __delay_cycles(625000);               /* 32 * 32 * 20e6 / 32768 - worst-case FLL settle */

    {
        uint16_t guard = 0;
        do {
            UCSCTL7 &= (uint16_t)~(XT2OFFG | XT1LFOFFG | XT1HFOFFG | DCOFFG);
            SFRIFG1 &= (uint16_t)~OFIFG;
        } while ((SFRIFG1 & OFIFG) && (++guard < 20000u));
    }
}

static void gpio_Init(void)
{
    /* Port 1: LEDs, nHOME/OCPM (Hi-Z input), nSLEEP. Start nSLEEP low (asleep)
     * until drv8262_Init() wakes the driver. */
    P1OUT = 0;
    /* P1.1/P1.2 deliberately absent: ROM-BSL UART lines shared with the ESP32
     * programmer - stay inputs, Hi-Z, no pull (P1REN = 0). */
    P1DIR = LED_HEARTBEAT_BIT | LED_LIMIT_OPEN_BIT | LED_BUTTON_BIT |
            LED_FAULT_BIT | NSLEEP_BIT;
    P1REN = 0;
    P1IE  = 0;

    /* Port 2: TOFF_CTRL driven, TOFF_PD left as input (on-board bias tap sets
     * the TOFF time together with TOFF_CTRL's level), nFAULT input. */
    P2OUT = 0;
    P2DIR = TOFF_CTRL_BIT;
    P2REN = 0;

    /* Port 3: entirely unused on this board for the gate controller - safe
     * outputs, driven low. */
    P3OUT = 0;
    P3DIR = 0xFF;
    P3SEL = 0;

    /* Port 4: IN1/IN2 default to GPIO-low (coast); drv8262.c enables P4SEL on
     * whichever leg is active once a nonzero duty is commanded. IN3/IN4 and the
     * remaining test points are unused in single-H-bridge PWM mode - outputs,
     * driven low (matches the DRV8262VEVM reference firmware). */
    P4OUT = 0;
    P4DIR = 0xFF;
    P4SEL = 0;

    /* Port 5: the ADC uses AVCC/AVSS as its reference, so P5.1 (VeREF-) and the
     * rest are unused - safe outputs low. EXCEPT P5.0: on the EVM's J6 VREF path
     * "MSP_VREF/VREF2" can land on P5.0, so drive it HIGH as a fixed ~3.3 V
     * DRV8262 VREF source (harmless if J6 routes the ref off a different pin -
     * see the Port 6 VREF_DRIVE_BITS note and drv8262.c). */
    P5OUT = BIT0;
    P5DIR = 0xFF;
    P5SEL = 0;

    /* Port 6: CLOSE/OPEN buttons (always internal pull-UP, active-LOW - real
     * momentary switches in every mode), IPROPI1/IPROPI2 (ADC input, NO pull -
     * discrete R_IPROPI on the EVM), MODE2 (GPIO, deliberately excluded from
     * P6SEL), VM sense (analog), and P6.6 (+ P6.7 while VREF_DAC_ENABLE=0)
     * driven HIGH as a fixed DRV8262 VREF (VREF_DRIVE_BITS). When
     * VREF_DAC_ENABLE=1, P6.7 is left alone here instead - drv8262_Init()
     * configures it as DAC12_1, which takes over the pin on its own once
     * enabled. */
    P6OUT = BTN_CLOSE_BIT | BTN_OPEN_BIT | VREF_DRIVE_BITS;
    P6DIR = MODE2_BIT | VREF_DRIVE_BITS;
    P6SEL = IPROPI1_BIT | IPROPI2_BIT | VM_SENSE_BIT;   /* P6.1/P6.2 = ADC in, no pull; P6.6 stays GPIO.
                                                            P6.7 needs no P6SEL bit even when
                                                            VREF_DAC_ENABLE=1 - DAC12_1 takes the pin
                                                            over on its own once drv8262.c enables it. */
    P6REN = BTN_CLOSE_BIT | BTN_OPEN_BIT;

    /* Port 7: two limit switches, TOGGLE button, REMOTE - all internal-pull
     * inputs (P7REN below), P7OUT bit set = pull-UP, clear = pull-DOWN.
     *   Production (!TEST_MODE): limits + TOGGLE pull-UP. The limit inputs are
     *   switch -> optocoupler -> MSP430; the opto output transistor is
     *   effectively open-drain (only pulls LOW), so the pull-UP is mandatory
     *   for the pin to ever read HIGH - it stays pull-UP regardless of
     *   LIMIT_SWITCH_ACTIVE_HIGH (that flag only changes the logical reading in
     *   inputs.c rawActive()).
     *   Bench (TEST_MODE): only TOGGLE + REMOTE pulled UP; LIMITS pulled DOWN
     *   (not wired - poke HIGH to assert).
     *   REMOTE (P7.4): ALWAYS internal pull-UP, every build/mode - open-drain
     *   opto output can only sink. Logical "pressed" level is REMOTE_ACTIVE_HIGH,
     *   applied in inputs.c rawActive().
     * P7SEL left clear (Rev 2/3: R5/R8/R13/R16 must be depopulated for clean
     * digital reads - project_plan.md Sec 3/10). */
#if TEST_MODE
    P7OUT = LIM_REMOTE_BIT | BTN_TOGGLE_BIT;
#else
    P7OUT = LIM_REMOTE_BIT | LIM_OPEN_BIT | LIM_CLOSED_BIT | BTN_TOGGLE_BIT;
#endif
    P7DIR = 0x0F;                              /* unused low nibble as outputs      */
    P7SEL = 0;
    P7REN = LIM_REMOTE_BIT | LIM_OPEN_BIT | LIM_CLOSED_BIT | BTN_TOGGLE_BIT;

    /* Port 8: UART, DECAY0, MODE1, photo beam. P8.6 internal pull is enabled
     * (P8REN) and its direction is set by PHOTOBEAM_PULL_UP (gate_config.h):
     * P8OUT bit SET = pull-UP (default - the open-drain photo-eye/opto can only
     * sink, so it needs the pull-UP to read HIGH), CLEAR = pull-DOWN. The
     * logical "beam broken" level is a separate knob, PHOTOBEAM_ACTIVE_HIGH,
     * applied in inputs.c rawActive(). The other P8 output bits stay LOW. */
#if PHOTOBEAM_PULL_UP
    P8OUT = PHOTOBEAM_BIT;                     /* P8.6 internal pull-UP   */
#else
    P8OUT = 0;                                 /* P8.6 internal pull-DOWN */
#endif
    P8DIR = UART_TXD_BIT | DECAY0_BIT | MODE1_BIT | BIT0 | BIT1 | BIT4;
    P8SEL = UART_TXD_BIT | UART_RXD_BIT;
    P8REN = PHOTOBEAM_BIT;

    /* Port 9: entirely unused. */
    P9OUT = 0;
    P9DIR = 0xFF;
    P9SEL = 0;

    /* RST/NMI (pin 96) intentionally untouched - stays a plain hardware reset. */
}

static void watchdog_Init(void)
{
    WDTCTL = WDT_ARST_250;                 /* 250 ms watchdog, ACLK-sourced   */
}

/* Timer_A2: dynamic tick. TASSEL_2 (SMCLK) is used at BOTH rates (never ACLK)
 * so the UART keeps working even while idle - project_plan.md Rev 3 Sec 7.1. */
void board_SetTickRate(tick_rate_t rate)
{
    if (s_tickRateInited && rate == s_tickRate) {
        return;
    }
    TA2CTL = TASSEL_2 | ID_3 | MC_0 | TACLR;   /* stop, SMCLK/8 base divider   */
    if (rate == TICK_ACTIVE) {
        TA2EX0  = TICK_ACTIVE_EXDIV;
        TA2CCR0 = TICK_ACTIVE_TACCR0;
    } else {
        TA2EX0  = TICK_IDLE_EXDIV;
        TA2CCR0 = TICK_IDLE_TACCR0;
    }
    TA2CCTL0 = CCIE;
    TA2CTL  |= MC_1 | TACLR;
    s_tickRate = rate;
    s_tickRateInited = true;
}

uint16_t board_GetTickMs(void)
{
    return (s_tickRate == TICK_ACTIVE) ? TICK_ACTIVE_MS : TICK_IDLE_MS;
}

/* Plain volatile read, NOT interrupt-protected - see the comment on the
 * declaration in board.h. An earlier version of this masked interrupts around
 * the 32-bit load for textbook atomicity; that briefly disabled interrupts on
 * every single tick during a drive (beam_Tick() calls this every 1 ms while
 * moving), which is exactly the wrong place to add ANY interrupt latency: the
 * current-sense ADC (current.c) depends on its ISR firing and being promptly
 * serviced by current_Service() to keep re-arming, every ~1 ms, for as long as
 * the motor is driving - and that pipeline feeds ocSoftTrip()/ocHardTrip(),
 * the obstacle/over-current safety logic. Reporting-only code must never be
 * allowed to contend with that for CPU/interrupt time. */
uint32_t board_UptimeMs(void)
{
    return s_uptimeMs;
}

void board_SleepUntilTick(void)
{
    __bis_SR_register(LPM0_bits | GIE);
}

void wdt_Kick(void)
{
    WDTCTL = WDT_ARST_250;
}

void board_Init(void)
{
    /* Arm (do NOT hold) the watchdog from the very first instruction: a wedged
     * clock/PMM bring-up now self-resets in 250 ms instead of sitting dark
     * forever with the WDT held. WDT_ARST_250 is ACLK-sourced and ACLK
     * auto-falls back to REFO here (no XT1 fitted), so it counts immediately. */
    WDTCTL = WDT_ARST_250;
    clock_Init();
    gpio_Init();
    /* Let the P7 limit-input internal pull-ups charge the pin + wiring
     * capacitance before anything (inputs_Init) latches them - see
     * BOOT_PULL_SETTLE_CYCLES. Well inside the 250 ms watchdog. */
    __delay_cycles(BOOT_PULL_SETTLE_CYCLES);
    board_SetTickRate(TICK_ACTIVE);        /* main.c drops to idle once settled */
    watchdog_Init();                       /* refresh now that SMCLK/ACLK are known-good */
}

/* ---------------------------------------------------------------- LEDs --- */
static volatile uint8_t *led_PortOut(led_id_t id)
{
    (void)id;
    return &P1OUT;                          /* every LED is on Port 1 */
}

static uint8_t led_Bit(led_id_t id)
{
    switch (id) {
        case LED_HEARTBEAT:    return LED_HEARTBEAT_BIT;
        case LED_LIMIT_OPEN:   return LED_LIMIT_OPEN_BIT;
        case LED_BUTTON:       return LED_BUTTON_BIT;
        case LED_FAULT:        return LED_FAULT_BIT;
        /* LED_BEAM_OK (P1.1) and LED_LIMIT_CLOSED (P1.2) retired - now BSL UART
         * pins. Fall through to 0 so led_Set()/led_Toggle() are no-ops. */
        default:                return 0;
    }
}

void led_Set(led_id_t id, bool on)
{
    uint8_t bit = led_Bit(id);
    if (bit == 0) { return; }   /* retired LED id - do not touch the port */
    if (on) { *led_PortOut(id) |= bit; } else { *led_PortOut(id) &= (uint8_t)~bit; }
}

void led_Toggle(led_id_t id)
{
    uint8_t bit = led_Bit(id);
    if (bit == 0) { return; }
    *led_PortOut(id) ^= bit;
}

void led_SetFaultPattern(fault_led_pattern_t pattern)
{
    s_faultPattern = pattern;
    s_faultBlinkAccumMs = 0;
    if (pattern == FAULT_LED_OFF) {
        led_Set(LED_FAULT, false);
    } else if (pattern == FAULT_LED_SOLID) {
        led_Set(LED_FAULT, true);
    }
}

/* Called once per tick from main.c. The blink half-period is expressed in real
 * milliseconds and accumulated from board_GetTickMs(), so D12 blinks at the
 * same visible rate in either tick regime. (It used to count raw ticks with a
 * hard-coded 50 ms/1 ms split, which silently became a 2 s "fast" blink once
 * the idle tick moved to 10 ms.) */
void led_Service(void)
{
    uint16_t halfPeriodMs;
    switch (s_faultPattern) {
        case FAULT_LED_FAST_BLINK: halfPeriodMs = 200u;  break;
        case FAULT_LED_SLOW_BLINK: halfPeriodMs = 1000u; break;
        default: return; /* OFF/SOLID need no periodic service */
    }
    s_faultBlinkAccumMs = (uint16_t)(s_faultBlinkAccumMs + board_GetTickMs());
    if (s_faultBlinkAccumMs >= halfPeriodMs) {
        s_faultBlinkAccumMs = 0u;
        led_Toggle(LED_FAULT);
    }
}

/* --------------------------------------------------------- DRV8262 pins --- */
void drvpin_SetSleep(bool assertLow)
{
    if (assertLow) { P1OUT &= (uint8_t)~NSLEEP_BIT; } else { P1OUT |= NSLEEP_BIT; }
}

void drvpin_SetMode1(bool high)
{
    if (high) { P8OUT |= MODE1_BIT; } else { P8OUT &= (uint8_t)~MODE1_BIT; }
}

void drvpin_SetMode2(bool high)
{
    if (high) { P6OUT |= MODE2_BIT; } else { P6OUT &= (uint8_t)~MODE2_BIT; }
}

void drvpin_SetDecay0(bool high)
{
    if (high) { P8OUT |= DECAY0_BIT; } else { P8OUT &= (uint8_t)~DECAY0_BIT; }
}

void drvpin_SetToffCtrl(bool high)
{
    if (high) { P2OUT |= TOFF_CTRL_BIT; } else { P2OUT &= (uint8_t)~TOFF_CTRL_BIT; }
}

bool drvpin_ReadNFault(void)
{
    return (P2IN & NFAULT_BIT) == 0;
}

bool drvpin_ReadNHome(void)
{
    return (P1IN & NHOME_OCPM_BIT) == 0;
}

/* ------------------------------------------------------------- raw inputs --- */
bool pin_RawRead(input_id_t id)
{
    switch (id) {
        case IN_LIMIT_OPEN:    return (P7IN & LIM_OPEN_BIT) != 0;
        case IN_LIMIT_CLOSED:  return (P7IN & LIM_CLOSED_BIT) != 0;
        case IN_BTN_TOGGLE:    return (P7IN & BTN_TOGGLE_BIT) != 0;
        case IN_BTN_OPEN:      return (P6IN & BTN_OPEN_BIT) != 0;
        case IN_BTN_CLOSE:     return (P6IN & BTN_CLOSE_BIT) != 0;
        case IN_REMOTE:        return (P7IN & LIM_REMOTE_BIT) != 0;
        case IN_PHOTOBEAM:     return (P8IN & PHOTOBEAM_BIT) != 0;
        case IN_NFAULT:        return (P2IN & NFAULT_BIT) != 0;
        case IN_NHOME:         return (P1IN & NHOME_OCPM_BIT) != 0;
        default:               return false;
    }
}

/*******************************************************************************
 * ISRs owned by board.c
 ******************************************************************************/
#pragma vector = TIMER2_A0_VECTOR
__interrupt void TimerA2_ISR(void)
{
    g_tickCount++;
    g_tickFlag = 1;
    s_uptimeMs += board_GetTickMs();   /* real-ms clock - see board_UptimeMs() */

    /* Heartbeat D6 (P1.0) is advanced here, off the raw tick, so it keeps
     * blinking regardless of super-loop / FSM state - including the 50 ms idle
     * ("low power") tick regime, and even if the super-loop stalls. P1OUT ^= is
     * a single XOR.B, atomic against the super-loop's BIS.B/BIC.B on the other
     * P1 bits. Half-period = HEARTBEAT_TOGGLE_MS (gate_config.h) - change it,
     * rebuild and reflash to give an image a distinctive blink rate. */
    s_heartbeatAccumMs = (uint16_t)(s_heartbeatAccumMs + board_GetTickMs());
    if (s_heartbeatAccumMs >= HEARTBEAT_TOGGLE_MS) {
        s_heartbeatAccumMs = 0u;
        P1OUT ^= LED_HEARTBEAT_BIT;
    }

    __bic_SR_register_on_exit(LPM0_bits);
}

/* Catch-all for every unused interrupt vector on this device. */
#pragma vector = DAC12_VECTOR, RTC_VECTOR, PORT4_VECTOR, PORT3_VECTOR, \
                 TIMER2_A1_VECTOR, USCI_B1_VECTOR, PORT2_VECTOR, \
                 TIMER1_A1_VECTOR, TIMER1_A0_VECTOR, DMA_VECTOR, LDO_PWR_VECTOR, \
                 TIMER0_A1_VECTOR, TIMER0_A0_VECTOR, USCI_B0_VECTOR, USCI_A0_VECTOR, \
                 WDT_VECTOR, TIMER0_B1_VECTOR, TIMER0_B0_VECTOR, COMP_B_VECTOR, \
                 UNMI_VECTOR, SYSNMI_VECTOR, PORT1_VECTOR
__interrupt void Trap_ISR(void)
{
    /* Unexpected interrupt - spin here so the debugger can catch it. */
    __no_operation();
}
