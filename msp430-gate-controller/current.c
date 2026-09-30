/*******************************************************************************
 * current.c - see current.h. Owns Timer_A0 (ADC trigger) and ADC12. Only one
 * of {continuous IPROPI, idle VM} is ever configured/enabled at a time, so
 * there is no reconfiguration race between them.
 ******************************************************************************/
#include <msp430.h>
#include <stdbool.h>
#include "current.h"
#include "gate_config.h"
#include "console.h"
#include "uart.h"
#include "board.h"

/* Ping-pong pair, not a single buffer - see the ISR/current_Service() header
 * comment below for why (2026-09-28: root-caused a live current-sense freeze
 * to this). */
static volatile uint16_t s_raw1[2][ADC_AVG_SAMPLES];
static volatile uint16_t s_raw2[2][ADC_AVG_SAMPLES];
static volatile uint8_t  s_sampleIdx = 0;
static volatile uint8_t  s_writeBuf = 0;    /* which half the ISR is currently filling */
static volatile uint8_t  s_readyBuf = 0;    /* which half current_Service() should read */
static volatile uint8_t  s_samplesReady = 0;
static uint16_t s_isenseCounts = 0;

static current_mode_t   s_mode = CURRENT_MODE_CONTINUOUS;
static volatile uint16_t s_vmRaw = 0;
static volatile uint8_t  s_vmReady = 0;
static uint16_t s_vmMv = 0;
static uint16_t s_idleTickCounter = 0;

/* Watchdog for a live current-sense stall (see the big header comment above
 * ADC12_ISR further down) - board_UptimeMs() of the last batch current_Service()
 * successfully consumed, while in CONTINUOUS mode. */
static uint32_t s_lastBatchMs = 0;

static uint16_t s_peakRampCounts = 0;
static uint32_t s_ssSumCounts = 0;
static uint16_t s_ssSampleCount = 0;
static uint16_t s_ssMaxCounts = 0;

static uint16_t countsToMa(uint16_t counts)
{
    return (uint16_t)(((uint32_t)counts * MA_PER_COUNT_Q8) >> 8);
}

static uint16_t rawToMv(uint16_t raw)
{
    /* VSEN_VM divider + pin-voltage conversion folded into VM_MV_PER_COUNT_Q8.
     * Clamped as a safety net for a disconnected sense input pinning full-scale. */
    uint32_t mv = ((uint32_t)raw * VM_MV_PER_COUNT_Q8) >> 8;
    return (mv > 0xFFFFu) ? 0xFFFFu : (uint16_t)mv;
}

/* The full TA0+ADC12 CONTINUOUS-mode bring-up. Split out of current_SetMode()
 * so the stall-recovery watchdog in current_Service() can re-run exactly the
 * same sequence without duplicating it - see the ADC12_ISR header comment for
 * why a recovery path exists at all. */
static void armContinuous(void)
{
    ADC12CTL0 &= ~ADC12ENC;
    ADC12CTL1  = ADC12CONSEQ_3 | ADC12SHS_1;              /* repeat-seq, Timer_A0.1 trigger */
    ADC12MCTL0 = ADC12INCH_1 | ADC12SREF_0;               /* A1 = IPROPI1 */
    ADC12MCTL1 = ADC12INCH_2 | ADC12SREF_0;               /* A2 = IPROPI2 */
    ADC12MCTL2 = ADC12EOS | ADC12INCH_4 | ADC12SREF_0;    /* A4 = VM sense, end of seq */
    ADC12IE    = ADC12IE2;                                /* IRQ after the last (VM) conv */
    s_sampleIdx = 0;
    s_samplesReady = 0;
    s_writeBuf = 0;
    s_readyBuf = 0;

    TA0CTL   = TASSEL_2 | ID_3 | MC_0 | TACLR;            /* SMCLK/8 = 2.5 MHz */
    TA0CCR0  = ADC_TRIGGER_TACCR0;                        /* 1 kHz (was 25 kHz) */
    TA0CCR1  = 2;
    TA0CCTL1 = OUTMOD_7;
    TA0CTL  |= MC_1 | TACLR;

    ADC12CTL0 |= ADC12ENC;

    s_lastBatchMs = board_UptimeMs();   /* re-baseline the stall watchdog */
}

void current_SetMode(current_mode_t mode)
{
    ADC12CTL0 &= ~ADC12ENC;

    if (mode == CURRENT_MODE_CONTINUOUS) {
        armContinuous();
    } else {
        TA0CTL = TASSEL_2 | ID_3 | MC_0 | TACLR;              /* halt - no continuous trigger needed */
        ADC12CTL1  = ADC12SHP;                                 /* single-channel, single-conversion */
        ADC12MCTL0 = ADC12INCH_4 | ADC12SREF_0;               /* A4 = VM sense */
        ADC12IE    = ADC12IE0;
        s_idleTickCounter = 0;
        s_vmReady = 0;
    }
    s_mode = mode;

    if (mode == CURRENT_MODE_IDLE) {
        /* IPROPI is only sampled in CONTINUOUS mode, and s_isenseCounts was
         * never cleared on the way out - so it latched whatever the final ADC
         * batch happened to hold and reported that as the live motor current
         * for as long as the gate sat idle. Observed on the bench as a steady
         * "2680 mA" with the driver asleep. No current can be flowing once the
         * bridge is coasted/asleep, so the honest reading is zero. (The old
         * polled bridge usually showed 0 here, but only because the last batch
         * before the switch happened to land near zero - it was luck, not
         * design.) Nothing reads this while driving-mode logic is active: the
         * OC trips, metrics and capture all run only in CONTINUOUS. */
        s_isenseCounts = 0;

        /* Entering IDLE used to wait a whole IDLE_VM_SAMPLE_TICKS period (~1 s)
         * before the first VM conversion, so anything read in that window saw
         * "VM: 0 mV". That was invisible while a poller asked again 8 s later;
         * now that state is PUSHED on change, a stale 0 would sit in Home
         * Assistant until the next event. Kick one conversion immediately. */
        current_RequestVm();
    }
}

void current_Init(void)
{
    ADC12CTL0 = ADC12ON;
    current_SetMode(CURRENT_MODE_CONTINUOUS);
}

void current_RequestVm(void)
{
    if (s_mode != CURRENT_MODE_IDLE) { return; }
    ADC12CTL0 |= ADC12ENC | ADC12SC;
}

void current_TickIdle(void)
{
    if (s_mode != CURRENT_MODE_IDLE) { return; }
    if (++s_idleTickCounter >= IDLE_VM_SAMPLE_TICKS) {
        s_idleTickCounter = 0;
        current_RequestVm();
    }
}

void current_Service(void)
{
    if (s_mode == CURRENT_MODE_CONTINUOUS) {
        if (s_samplesReady) {
            uint32_t sum = 0;
            uint8_t i;
            uint8_t buf = s_readyBuf;   /* the ISR has already moved on to the OTHER
                                          * half by the time s_samplesReady went up -
                                          * see the ISR - so this half is stable to
                                          * read here with no race. */
            for (i = 0; i < ADC_AVG_SAMPLES; i++) {
                sum += s_raw1[buf][i];
                sum += s_raw2[buf][i];
            }
            s_isenseCounts = (uint16_t)(sum / (2u * ADC_AVG_SAMPLES));
            s_vmMv = rawToMv(s_vmRaw);          /* VM is interleaved into the seq now */
            s_samplesReady = 0;
            s_lastBatchMs = board_UptimeMs();
            /* ADC12ENC is re-armed INSIDE the ISR now, not here - see the ISR
             * header comment for why re-arming from the main loop was the bug. */
        } else {
            /* STALL WATCHDOG - a batch normally completes every ~8 ms
             * (ADC_AVG_SAMPLES=8 @ 1 kHz). Root-caused this project's live
             * current-sense freeze to TA0 itself no longer counting (confirmed
             * on the bench with a register dump: ADC12ENC stayed set, TA0R
             * stopped advancing, s_sampleIdx stuck mid-batch) - closing the
             * ADC12ENC-toggle race (re-arming inside the ISR, above) did NOT
             * stop it recurring, so something deeper than that specific
             * MSP430F5xx erratum is at play. Rather than ship known-blind
             * over-current protection while that gets fully root-caused, treat
             * a batch that is dramatically overdue as the pipeline having
             * wedged and force it back to life: over-current/obstacle safety
             * depends on this staying live for as long as the motor drives. */
            uint32_t now = board_UptimeMs();
            if ((uint32_t)(now - s_lastBatchMs) > ADC_STALL_RECOVERY_MS) {
                dbg_Printf("FAULT: current-sense stalled %lu ms - ADC/TA0 pipeline reset\r\n",
                           (unsigned long)(now - s_lastBatchMs));
                armContinuous();
            }
        }
    } else {
        if (s_vmReady) {
            s_vmMv = rawToMv(s_vmRaw);
            s_vmReady = 0;
        }
    }
}

uint16_t current_IsenseCounts(void)     { return s_isenseCounts; }
uint16_t current_IsenseMilliAmps(void)  { return countsToMa(s_isenseCounts); }
uint16_t current_VmMilliVolts(void)     { return s_vmMv; }

void metrics_OnMotorStart(void)
{
    s_peakRampCounts = 0;
    s_ssSumCounts = 0;
    s_ssSampleCount = 0;
    s_ssMaxCounts = 0;
}

/* rampUpActive: true while the commanded duty is still climbing to target (the
 * whole soft-start ramp) - the peak seen then is reported as "Peak ramp-up
 * current" (the real breakaway surge, which lands near ramp completion, not in
 * the old 800 ms blanking window). accumSteady: true once at target - feeds the
 * steady-state avg/max, excluding both the ramp-up and the wind-down tail. */
void metrics_Tick(bool rampUpActive, bool accumSteady)
{
    uint16_t s = s_isenseCounts;
    if (rampUpActive) {
        if (s > s_peakRampCounts) { s_peakRampCounts = s; }
    } else if (accumSteady) {
        s_ssSumCounts += s;
        s_ssSampleCount++;
        if (s > s_ssMaxCounts) { s_ssMaxCounts = s; }
    }
}

void metrics_Snapshot(uint16_t *peakRampMa, uint16_t *ssAvgMa, uint16_t *ssMaxMa)
{
    uint16_t ssAvgCounts = s_ssSampleCount ? (uint16_t)(s_ssSumCounts / s_ssSampleCount) : 0u;
    if (peakRampMa) { *peakRampMa = countsToMa(s_peakRampCounts); }
    if (ssAvgMa)     { *ssAvgMa    = countsToMa(ssAvgCounts); }
    if (ssMaxMa)     { *ssMaxMa    = countsToMa(s_ssMaxCounts); }
}

/* ================= per-move data capture (see current.h) ================= */
typedef struct { uint16_t ma; uint16_t mv; } cap_sample_t;
static cap_sample_t s_cap[CAPTURE_SAMPLES];
static uint16_t s_capN = 0;              /* samples recorded this move          */
static uint16_t s_capAccumMs = 0;
static bool     s_capActive = false;
static const char *s_capLabel = "MOVE";

static bool     s_capDump = false;      /* a dump is queued / in progress      */
static uint16_t s_capDumpIdx = 0;
static uint8_t  s_capDumpPhase = 0;     /* 1 header, 2 rows, 3 footer          */

void capture_Start(const char *label)
{
    if (s_capDump) { s_capActive = false; return; }  /* previous dump still draining */
    s_capLabel   = label;
    s_capN       = 0;
    s_capAccumMs = CAPTURE_INTERVAL_MS;  /* record on the very first tick */
    s_capActive  = true;
}

void capture_Tick(uint16_t dt)
{
    if (!s_capActive || s_capN >= CAPTURE_SAMPLES) { return; }
    s_capAccumMs = (uint16_t)(s_capAccumMs + dt);
    if (s_capAccumMs < CAPTURE_INTERVAL_MS) { return; }
    s_capAccumMs = (uint16_t)(s_capAccumMs - CAPTURE_INTERVAL_MS);
    s_cap[s_capN].ma = countsToMa(s_isenseCounts);
    s_cap[s_capN].mv = s_vmMv;
    s_capN++;
}

void capture_Finish(void)
{
    s_capActive = false;
    /* Record one last sample at the instant the move ends. The end-of-move
     * event (limit reached, OC trip, stop) lands in the partial interval AFTER
     * the final capture_Tick(), so without this the trace stops just short of
     * the stall-into-the-limit current peak. Its t_ms (index * interval) is up
     * to one interval optimistic - acceptable for a visual trace. */
    if (s_capN > 0u && s_capN < CAPTURE_SAMPLES && !s_capDump) {
        s_cap[s_capN].ma = countsToMa(s_isenseCounts);
        s_cap[s_capN].mv = s_vmMv;
        s_capN++;
    }
    if (s_capN > 0u && !s_capDump) {
        s_capDump      = true;
        s_capDumpIdx   = 0;
        s_capDumpPhase = 1;
    }
}

/* Emit the capture as a small framed block, paced against the UART TX ring so
 * nothing is dropped. Spans as many super-loop passes as needed (the gate is
 * stopped by the time this runs). Wire format (see SERIAL_PROTOCOL.md):
 *
 *   CAP_BEGIN <label> samples=<n> interval_ms=<ms> group=<g> fmt=t_ms,mA,mV
 *   <up to g records joined by '~'>            one such line per group
 *   ...
 *   CAP_END
 *   ;                                          end-of-message marker
 *
 * Each record is "t_ms,mA,mV". Lines between CAP_BEGIN and CAP_END that do not
 * start with a digit are unrelated async messages - ignore them when parsing. */
void capture_DumpService(void)
{
    if (!s_capDump) { return; }

    if (s_capDumpPhase == 1u) {
        if (uart_TxFree() < 100u) { return; }
        dbg_PrintfRaw("CAP_BEGIN %s samples=%u interval_ms=%u group=%u fmt=t_ms,mA,mV\r\n",
                      s_capLabel, (unsigned)s_capN, (unsigned)CAPTURE_INTERVAL_MS,
                      (unsigned)CAPTURE_DUMP_GROUP);
        s_capDumpPhase = 2u;
        return;
    }
    if (s_capDumpPhase == 2u) {
        while (s_capDumpIdx < s_capN) {
            uint16_t g;
            /* Two limits, both needed: room for one full group line (~220 B),
             * AND never leave more than CAPTURE_DUMP_MAX_QUEUED bytes sitting
             * in the TX ring. The dump used to fill the 4 KB ring to within
             * 220 bytes of full, which put a photo-beam edge emitted mid-dump
             * ~350 ms behind it on the wire. The UART is the bottleneck either
             * way, so capping the queue costs the dump nothing overall. */
            if (uart_TxFree() < 220u) { return; }
            if (uart_TxFree() < (uint16_t)(UART_TX_RING_LEN - CAPTURE_DUMP_MAX_QUEUED)) { return; }
            for (g = 0u; g < CAPTURE_DUMP_GROUP && s_capDumpIdx < s_capN; g++) {
                if (g != 0u) { dbg_PrintRaw("~"); }
                dbg_PrintfRaw("%u,%u,%u",
                              (unsigned)((uint32_t)s_capDumpIdx * CAPTURE_INTERVAL_MS),
                              (unsigned)s_cap[s_capDumpIdx].ma,
                              (unsigned)s_cap[s_capDumpIdx].mv);
                s_capDumpIdx++;
            }
            dbg_PrintRaw("\r\n");
        }
        s_capDumpPhase = 3u;
        return;
    }
    if (uart_TxFree() < 16u) { return; }
    dbg_PrintRaw("CAP_END\r\n;\r\n");
    s_capDumpPhase = 0u;
    s_capDump = false;
}

/* current_Service()/ADC12_ISR handshake - PING-PONG, and the re-arm moved
 * INTO the ISR (2026-09-28) ------------------------------------------------
 *
 * Root cause of a live current-sense freeze (motor current locks to one value
 * partway into every drive - confirmed on the bench with a temporary register
 * dump: ADC12ENC stayed set, but TA0R itself stopped advancing, s_sampleIdx
 * stuck mid-batch, s_samplesReady stuck at 0 - i.e. the ADC's hardware trigger
 * pipeline had wedged): this matches TI's documented MSP430F5xx erratum class
 * ADC38/ADC42 - "external ADC trigger without toggling ENC might prevent
 * further conversions... ensure ADC12CTL0.ADC12ENC is always toggled before
 * providing any new external trigger."
 *
 * The old code DID toggle ENC (ISR clears it, current_Service() sets it back)
 * - but with a GAP between the two: TA0 is a free-running timer that keeps
 * pulsing its hardware trigger every 1 ms regardless of ADC12's state, and
 * current_Service() runs in the MAIN LOOP, so anything else that main loop
 * does first (console/beam/report work, all of it) delays the re-arm. A
 * trigger pulse landing in that gap - ENC cleared, not yet re-set - is exactly
 * the erratum's failure mode, and it only has to happen once in a drive to
 * wedge the pipeline for the rest of it.
 *
 * Fix: re-arm ENC INSIDE the ISR, immediately after clearing it, so there is
 * no scheduling gap for a stray trigger to land in at all. That means the ISR
 * can no longer safely keep writing into the buffer current_Service() is
 * about to read - hence the ping-pong: the ISR fills one half while
 * current_Service() reads the other, swapping which is which every completed
 * batch (8 ms), which is far longer than current_Service() ever needs. */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    if (s_mode == CURRENT_MODE_CONTINUOUS) {
        s_raw1[s_writeBuf][s_sampleIdx] = ADC12MEM0;
        s_raw2[s_writeBuf][s_sampleIdx] = ADC12MEM1;
        s_vmRaw                        = ADC12MEM2;   /* VM sampled alongside IPROPI this pass */

        /* decorrelate the ADC trigger phase against the 100-count PWM period */
        {
            uint16_t next = (uint16_t)(TA0CCR1 + ADC_PHASE_STEP);
            if (next >= TA0CCR0) { next -= TA0CCR0; }
            TA0CCR1 = next;
        }

        if (++s_sampleIdx >= ADC_AVG_SAMPLES) {
            s_sampleIdx = 0;
            ADC12CTL0 &= ~ADC12ENC;      /* toggle off... */
            s_readyBuf  = s_writeBuf;    /* this half is done - safe for current_Service() */
            s_writeBuf  = (uint8_t)(1u - s_writeBuf);   /* ISR moves to the OTHER half now */
            ADC12CTL0  |= ADC12ENC;      /* ...and immediately back on - no gap, per the erratum workaround */
            s_samplesReady = 1;
        }
    } else {
        s_vmRaw = ADC12MEM0;
        s_vmReady = 1;
        ADC12CTL0 &= ~ADC12ENC;
    }
}
