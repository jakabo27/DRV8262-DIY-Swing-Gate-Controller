/*******************************************************************************
 * drv8262.c - see drv8262.h. Owns Timer_B0 (25 kHz PWM) and P4.1/P4.2 (IN1/IN2).
 * No Timer_B0 ISR is needed: the ramp is advanced from the 1 ms tick (not the
 * PWM period), and PWM itself free-runs in hardware once TBCCRx is written.
 ******************************************************************************/
#include <msp430.h>
#include "drv8262.h"
#include "board.h"
#include "gate_config.h"
#include "nvconfig.h"

#define IN1_BIT BIT1   /* P4.1 / TB0.1 */
#define IN2_BIT BIT2   /* P4.2 / TB0.2 */

/* DRV8262 VREF1 (hardware current-limit reference) CAN BE runtime-adjustable
 * on this build, gated by VREF_DAC_ENABLE (gate_config.h) - currently 0
 * (disabled) pending bench verification, so VREF1 is the original fixed
 * ~3.3 V GPIO reference for now (see board.c gpio_Init / VREF_DRIVE_BITS);
 * everything below describes the VREF_DAC_ENABLE=1 behavior.
 *
 * Path: DRV8262 pin 33 (VREF1/VREF) -> J25 -> VREF/VREF2 net -> J6 -> MSP430
 * pin 4 = P6.7. P6.7 is also DAC12_1 (a second, independent DAC channel from
 * the P6.6/DAC12_0 net that J4+J26 would reach instead) - once
 * DAC12_1CTL0.DAC12AMPx is nonzero the DAC takes over P6.7 regardless of
 * P6SEL/P6DIR, so board.c no longer needs to drive this pin as GPIO (see its
 * VREF_DRIVE_BITS comment; P6.6 is left as a dead/fixed leftover for the
 * unused J4+J26 route either way).
 *
 * DAC12SREF has no AVCC option on the F5338 (checked directly against
 * msp430f5338.h - only Vref+/Veref+ encodings exist), so the reference comes
 * from the internal REF module instead: REFVSEL_0 (1.5 V) with DAC12_1 gain =
 * 3x (DAC12IR cleared) gives
 *   V_VREF1 = 3 * 1.5 V * code/4095, clamped at AVCC (~3.3 V) above code ~3003
 * i.e. I_TRIP = V_VREF1 / (A_IPROPI * R_IPROPI) tops out at ~9.4 A - matching
 * both the original fixed setup and the ADC current-sense ceiling (both ride
 * the same AVCC rail through R_IPROPI/ISENSE_UA_PER_A). See gate_config.h
 * (DAC_REF_MV, DAC_CODE_PER_MA_Q16) and drv8262_SetCurrentLimitMa() below.
 * This all depends on the J25+J6 jumpering - if the EVM is ever rejumpered to
 * J4+J26 instead, VREF1 disconnects from the DAC and reverts to whatever
 * P6.6/DAC12_0 is doing (harmless, just inert again). The ADC-based OC_TRIP /
 * OC_HARD_LIMIT checks in gate_fsm.c remain the software-side backstop under
 * this hardware ceiling either way. */

static gate_dir_t s_activeDir = DIR_OPEN;
static int32_t    s_rampQ8    = 0;     /* 0..25600, current commanded duty      */
static int32_t    s_targetQ8  = 0;
static uint16_t   s_stepQ8    = 21;    /* per-tick slew limit, ~1.2 s default   */
static uint16_t   s_stopStepQ8 = 0;   /* nonzero -> overrides s_stepQ8 for a fast wind-down before braking */
static uint16_t   s_rampTimeMs = NVCFG_DEFAULT_RAMP_TIME_MS;  /* last RAMP_TIME; kept so StartRamp can span it */
static uint8_t    s_lastDuty  = 0;     /* 0-100, the duty actually applied      */
static bool        s_asleep   = true;

/* Per-tick Q8 slew for a full 0..100% sweep in 'ms' (>=1). */
static uint16_t stepFromMs(uint16_t ms)
{
    uint16_t s;
    if (ms == 0u) { ms = 1u; }
    s = (uint16_t)(25600UL / ms);
    return (s == 0u) ? 1u : s;
}

static void applyDuty(uint8_t duty)
{
    uint8_t activeBit = (s_activeDir == DIR_OPEN) ? IN1_BIT : IN2_BIT;
    uint8_t otherBit  = (s_activeDir == DIR_OPEN) ? IN2_BIT : IN1_BIT;

    P4SEL &= (uint8_t)~otherBit;
    P4OUT &= (uint8_t)~otherBit;
    if (s_activeDir == DIR_OPEN) { TB0CCR2 = 0; } else { TB0CCR1 = 0; }

    if (duty == 0) {
        P4SEL &= (uint8_t)~activeBit;
        P4OUT &= (uint8_t)~activeBit;
        if (s_activeDir == DIR_OPEN) { TB0CCR1 = 0; } else { TB0CCR2 = 0; }
    } else if (duty >= PWM_PERIOD_COUNTS) {
        P4SEL &= (uint8_t)~activeBit;
        P4OUT |= activeBit;                 /* glitch-free full-on via GPIO */
        if (s_activeDir == DIR_OPEN) { TB0CCR1 = PWM_PERIOD_TBCCR0; } else { TB0CCR2 = PWM_PERIOD_TBCCR0; }
    } else {
        P4OUT &= (uint8_t)~activeBit;
        P4SEL |= activeBit;
        if (s_activeDir == DIR_OPEN) { TB0CCR1 = duty; } else { TB0CCR2 = duty; }
    }
    s_lastDuty = duty;
}

void drv8262_Init(void)
{
    /* Timer_B0: 25 kHz PWM carrier (SMCLK/8/100 - ID_3 is the /8 prescaler;
     * without it this counts at the full 20 MHz SMCLK rate, giving 200 kHz).
     * OUTMOD_7 (reset/set) gives a direct, non-inverted duty->on-time mapping
     * (CCRx counts = high time out of 100), unlike the EVM's original OUTMOD_3.
     * Left halted; drv8262_Sleep(false) starts it on the first move. */
    TB0CTL   = TBSSEL_2 | ID_3 | MC_0 | TBCLR;
    TB0CCR0  = PWM_PERIOD_TBCCR0;
    TB0CCTL1 = OUTMOD_7;
    TB0CCTL2 = OUTMOD_7;
    TB0CCR1  = 0;
    TB0CCR2  = 0;

    drvpin_SetMode1(true);        /* MODE1 = HIGH: single H-bridge (OUT1||OUT3, OUT2||OUT4) */
    drvpin_SetMode2(true);        /* MODE2 = HIGH: PWM (IN/IN) mode                         */
    drvpin_SetToffCtrl(true);     /* TOFF = 16 us (TOFF_PD left as an input bias tap)        */
    drvpin_SetDecay0(false);      /* DECAY = slow decay                                      */

#if VREF_DAC_ENABLE
    /* Bring up the shared REF module + DAC12_1 once (see the file-header
     * comment above): 1.5 V internal reference, REFMSTR so REFCTL0 owns it
     * outright (the ADC12 conversions elsewhere all use ADC12SREF_0 = AVCC/
     * AVSS, never the REF module, so there's no contention). Bounded poll on
     * REFGENBUSY, same idiom as board.c's pmmWait() - never hang past the
     * watchdog on a stuck flag. Left enabled permanently, including whenever
     * the DRV8262 itself sleeps (drvpin_SetSleep) - re-settling on every wake
     * would add a variable delay before VREF1 is trustworthy right as the
     * motor starts driving. */
    REFCTL0 = REFMSTR | REFVSEL_0 | REFON;
    {
        uint16_t to = 0;
        while ((REFCTL0 & REFGENBUSY) && (++to < 20000u)) { }
    }
    DAC12_1CTL0 = DAC12SREF_0 | DAC12AMP_5 | DAC12ENC;  /* Vref+, 3x gain (DAC12IR=0), direct load */
#endif /* VREF_DAC_ENABLE - disabled: VREF1 stays board.c's fixed GPIO drive */

    s_activeDir = DIR_OPEN;
    s_rampQ8 = 0;
    s_targetQ8 = 0;
    s_lastDuty = 0;
    drv8262_SetRampTimeMs(g_cfg.rampTimeMs);
    drv8262_SetCurrentLimitMa(g_cfg.vrefCurrentRegulationMa);

    s_asleep = true;
    drvpin_SetSleep(true);        /* start asleep - the FSM wakes it for the first move */
}

void drv8262_SetDir(gate_dir_t dir)
{
    s_stopStepQ8 = 0;
    s_activeDir = dir;
    P4SEL &= (uint8_t)~(IN1_BIT | IN2_BIT);
    P4OUT &= (uint8_t)~(IN1_BIT | IN2_BIT);
    TB0CCR1 = 0;
    TB0CCR2 = 0;
}

void drv8262_SetDuty(uint8_t pct)
{
    s_stopStepQ8 = 0;                    /* a new commanded speed uses the normal ramp rate */
    s_stepQ8 = stepFromMs(s_rampTimeMs); /* restore the full-range slew - undo any one-shot StartRamp rate */
    if (pct > 100u) { pct = 100u; }
    s_targetQ8 = (int32_t)pct * 256;
}

/* Launch a fresh open/close: jump the commanded duty straight to 'startPct'
 * (the ramp floor) and slew from there up to 'targetPct' over the WHOLE
 * RAMP_TIME, so starting higher makes the gate move sooner without just
 * finishing the ramp early. The reduced slew is one-shot: the next
 * drv8262_SetDuty() (soft end-of-travel, soft stop) restores the normal
 * full-range rate, so the wind-down still takes RAMP_TIME. */
void drv8262_StartRamp(uint8_t startPct, uint8_t targetPct)
{
    int32_t startQ8, spanQ8;

    s_stopStepQ8 = 0;
    if (startPct  > 100u)      { startPct  = 100u; }
    if (targetPct > 100u)      { targetPct = 100u; }
    if (startPct  > targetPct) { startPct  = targetPct; }   /* never launch above target */

    startQ8    = (int32_t)startPct  * 256;
    s_targetQ8 = (int32_t)targetPct * 256;
    if (startQ8 > s_rampQ8) { s_rampQ8 = startQ8; }         /* jump up to the floor only, never down */

    spanQ8 = s_targetQ8 - s_rampQ8;
    if (spanQ8 < 0) { spanQ8 = -spanQ8; }
    {
        uint16_t ms = (s_rampTimeMs == 0u) ? 1u : s_rampTimeMs;
        uint32_t st = (uint32_t)spanQ8 / ms;
        s_stepQ8 = (st == 0u) ? 1u : (uint16_t)st;
    }
    applyDuty((uint8_t)(s_rampQ8 >> 8));                     /* apply the floor now, not on the next tick */
}

void drv8262_SetRampTimeMs(uint16_t ms)
{
    s_rampTimeMs = (ms == 0u) ? 1u : ms;
    s_stepQ8 = stepFromMs(s_rampTimeMs);
}

/* Wind the commanded duty down to 0 over roughly 'ms', independent of the
 * normal ramp rate, so a stop at a hard limit doesn't slam from full drive
 * straight into a short-brake (that dumps the winding energy as a big current
 * spike). Call drv8262_Service() each tick until drv8262_DutyIsZero(), then
 * drv8262_Brake(). Cleared by SetDuty/Coast/Brake/SetDir. */
void drv8262_BeginStop(uint16_t ms)
{
    if (ms == 0u) { ms = 1u; }
    s_stopStepQ8 = (uint16_t)(25600UL / ms);
    if (s_stopStepQ8 == 0u) { s_stopStepQ8 = 1u; }
    s_targetQ8 = 0;
}

/* True once the soft ramp has caught up to the commanded duty (move is at
 * speed) - used to gate steady-state current metrics past the ramp-up. */
bool drv8262_AtTarget(void)
{
    return s_rampQ8 == s_targetQ8;
}

void drv8262_Coast(void)
{
    P4SEL &= (uint8_t)~(IN1_BIT | IN2_BIT);
    P4OUT &= (uint8_t)~(IN1_BIT | IN2_BIT);
    TB0CCR1 = 0;
    TB0CCR2 = 0;
    s_rampQ8 = 0;
    s_targetQ8 = 0;
    s_stopStepQ8 = 0;
    s_lastDuty = 0;
}

void drv8262_Brake(void)
{
    P4SEL &= (uint8_t)~(IN1_BIT | IN2_BIT);
    P4OUT |= (IN1_BIT | IN2_BIT);
    TB0CCR1 = 0;
    TB0CCR2 = 0;
    s_rampQ8 = 0;
    s_targetQ8 = 0;
    s_stopStepQ8 = 0;
    s_lastDuty = 0;
}

void drv8262_Service(void)
{
    uint16_t step = (s_stopStepQ8 != 0u) ? s_stopStepQ8 : s_stepQ8;
    int32_t diff = s_targetQ8 - s_rampQ8;
    if (diff > (int32_t)step) { diff = (int32_t)step; }
    else if (diff < -(int32_t)step) { diff = -(int32_t)step; }
    s_rampQ8 += diff;
    applyDuty((uint8_t)(s_rampQ8 >> 8));
}

bool drv8262_DutyIsZero(void)
{
    return s_lastDuty == 0u;
}

void drv8262_SetCurrentLimitMa(uint16_t ma)
{
#if VREF_DAC_ENABLE
    /* mA -> DAC12_1DAT code, Q16 fixed point (see gate_config.h DAC_CODE_PER_MA_Q16
     * for the derivation). Clamped to the 12-bit code range defensively - the
     * console/nvconfig layer already range-checks to 9400 mA max, which maps
     * well under 4095, but this guards any other caller. Above the AVCC clip
     * point (~code 3003) the write still lands, it just has no further effect
     * on V_VREF1 - see the file-header comment. */
    uint32_t code = ((uint32_t)ma * DAC_CODE_PER_MA_Q16) >> 16;
    if (code > 4095u) { code = 4095u; }
    DAC12_1DAT = (uint16_t)code;
#else
    /* Disabled for now (see gate_config.h VREF_DAC_ENABLE) - VREF1 stays
     * board.c's original fixed ~3.3 V GPIO reference. VREF_CURRENT_REGULATION
     * is still stored/range-checked by nvconfig, just inert here, exactly like
     * the old CURRENT_LIMIT was before this feature existed. */
    (void)ma;
#endif
}

void drv8262_ClearFault(void)
{
    /* Zero the commanded duty *before* restarting the timer below: nFAULT can
     * still be asserted at this point (gate_fsm only confirms it clear on a
     * later tick) - without this, a successful clear pulse would resume PWM
     * at whatever duty was in effect when the fault hit, unramped, for up to
     * one 50 ms idle tick before the FSM catches up and coasts it. */
    drv8262_Coast();
    drvpin_SetSleep(true);
    __delay_cycles(DRV_FAULT_PULSE_CYCLES);
    drvpin_SetSleep(false);
    __delay_cycles(DRV_WAKE_DELAY_CYCLES);
    /* Restart Timer_B0 unconditionally: if the driver had been put to sleep
     * (TB0CTL halted, see drv8262_Sleep), setting s_asleep=false here without
     * restarting the PWM timer would leave it halted permanently - a later
     * drv8262_Sleep(false) call would then no-op (asleep==s_asleep already)
     * and never restart it, so the motor could never move again. */
    TB0CTL = TBSSEL_2 | ID_3 | MC_1 | TBCLR;
    s_asleep = false;
}

bool drv8262_ReadNFaultRaw(void)
{
    return drvpin_ReadNFault();
}

void drv8262_Sleep(bool asleep)
{
    if (asleep == s_asleep) { return; }
    if (asleep) {
        drv8262_Coast();
        TB0CTL = TBSSEL_2 | ID_3 | MC_0 | TBCLR;   /* halt the PWM timer entirely */
        drvpin_SetSleep(true);
        s_asleep = true;
    } else {
        drvpin_SetSleep(false);
        __delay_cycles(DRV_WAKE_DELAY_CYCLES);
        TB0CTL = TBSSEL_2 | ID_3 | MC_1 | TBCLR;   /* restart the PWM timer      */
        s_asleep = false;
    }
}

bool drv8262_IsAsleep(void)
{
    return s_asleep;
}
