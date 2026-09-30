/*******************************************************************************
 * gate_fsm.c - see gate_fsm.h and project_plan.md Rev 3 Sec 6.
 *
 * Forward drive (DIR_OPEN) opens the gate, reverse drive (DIR_CLOSE) closes it.
 * Arm-Retracted limit (IN_LIMIT_OPEN) = OPEN, Arm-Extended limit
 * (IN_LIMIT_CLOSED) = CLOSED (confirmed).
 ******************************************************************************/
#include <stddef.h>
#include "gate_fsm.h"
#include "gate_config.h"
#include "board.h"
#include "drv8262.h"
#include "current.h"
#include "inputs.h"
#include "console.h"
#include "nvconfig.h"
#include "beam.h"

/* Not exposed for runtime tuning - see project_plan.md Rev 3 Sec 8. */
#define NFAULT_RETRY_MAX      3u
#define NFAULT_RETRY_GAP_MS   50u

/* Origin-limit-release safety: when a normal open/close starts from an engaged
 * limit, that limit must disengage within this long or we STOP with a serial
 * fault - catches reversed motor leads, a dead/jammed motor, or a stuck switch.
 * Must comfortably exceed the soft-start ramp (NVCFG_DEFAULT_RAMP_TIME_MS) so a
 * heavy gate that is still ramping up hasn't tripped it before it gets moving. */
#define ORIGIN_LIMIT_RELEASE_MS  2500u

/* While the motor is driving, print a rolling progress line this often so the
 * operator can see roughly what the gate draws in motion (~2 Hz). */
#define DRIVE_CURRENT_REPORT_MS  DRIVE_PROGRESS_MS

typedef enum { SS_RAMPING, SS_PREBRAKE, SS_DEADTIME, SS_BACKOFF } softstop_phase_t;

static gate_state_t     s_state = ST_INIT;
static gate_dir_t       s_lastDir = DIR_CLOSE;
static lockout_reason_t s_lockoutReason = LOCKOUT_NONE;
static uint8_t          s_stallCount = 0;
static uint8_t          s_nfaultRetryCount = 0;
static gate_dir_t       s_homeDir = DIR_CLOSE;
static gate_dir_t       s_reverseTargetDir = DIR_OPEN;

/* ST_SOFT_STOP bookkeeping */
static softstop_phase_t s_ssPhase = SS_RAMPING;
static gate_state_t     s_softStopNext = ST_STOPPED_MIDTRAVEL;
static bool             s_softStopHasPendingDir = false;
static gate_dir_t       s_softStopPendingDir = DIR_OPEN;
static uint16_t         s_softStopBackoffMs = 0;
static gate_dir_t       s_softStopBackoffDir = DIR_CLOSE;

/* down-counters, in milliseconds */
static uint16_t s_tBlank = 0;
static uint16_t s_tOcGrace = 0;   /* OC_GRACE_TIME: while nonzero, BOTH the soft and hard OC trips are suppressed */
static uint32_t s_tTravel = 0;
static uint16_t s_tReverse = 0;
static uint16_t s_tBrake = 0;
static uint16_t s_tDeadtime = 0;
static uint16_t s_tBackoff = 0;
static uint16_t s_tRetryGap = 0;
static uint32_t s_lowBattConfirmMs = 0;
static uint32_t s_lowBattRecoverMs = 0;
static uint32_t s_lockoutClearMs = 0;

/* Origin-limit-release safety (see ORIGIN_LIMIT_RELEASE_MS). Nonzero = armed and
 * counting; s_originLimit is the limit that must go inactive before it hits 0. */
static uint16_t   s_originLimitClearMs = 0;
static input_id_t s_originLimit = IN_LIMIT_CLOSED;

/* Drive-current reporting (see DRIVE_CURRENT_REPORT_MS). Accumulates sensed
 * current each driving tick and prints the window average when s_tCurrentReport
 * counts down to 0. Reset by startDriveCurrentReport() on every motor start. */
static uint16_t s_tCurrentReport = 0;
static uint32_t s_curReportSumMa = 0;
static uint16_t s_curReportSamples = 0;
static uint32_t s_moveElapsedMs = 0;   /* since the current motor start */

/* Origin of whatever most recently changed the gate's situation - a command
 * (from the bridge, the RF remote or a wired button) or a firmware-initiated
 * transition. Reported as src= on EVT MOVE_START / EVT STATE and surfaced in
 * Home Assistant as "Last command source". */
static cmd_src_t s_lastSrc = CMDSRC_BOOT;

/* Report bookkeeping - see gate_fsm_ReportService(). */
static uint32_t s_lastEventSig    = 0xFFFFFFFFul;   /* impossible -> first pass always emits */
static uint32_t s_lastHeartbeatMs = 0ul;

static void startDriveCurrentReport(void)
{
    s_tCurrentReport   = DRIVE_CURRENT_REPORT_MS;
    s_curReportSumMa   = 0;
    s_curReportSamples = 0;
    s_moveElapsedMs    = 0;
}

static const char *cmdSrcStr(cmd_src_t s)
{
    switch (s) {
        case CMDSRC_BOOT:              return "boot";
        case CMDSRC_HA:                return "ha";
        case CMDSRC_REMOTE:            return "remote";
        case CMDSRC_WIRED_BUTTON:      return "wired_button";
        case CMDSRC_AUTO_REVERSE:      return "auto_reverse";
        case CMDSRC_OBSTRUCTION_STOP:  return "obstruction_stop";
        case CMDSRC_FAULT:             return "fault";
        default:                       return "none";
    }
}

static void reportDriveCurrent(uint16_t dt)
{
    s_curReportSumMa += current_IsenseMilliAmps();
    s_curReportSamples++;
    s_moveElapsedMs += dt;
    if (s_tCurrentReport > dt) {
        s_tCurrentReport = (uint16_t)(s_tCurrentReport - dt);
        return;
    }
    dbg_Printf("Drive current: ~%u mA avg (t=%lu ms, VM %u mV)\r\n",
               (unsigned)(s_curReportSamples
                          ? (uint16_t)(s_curReportSumMa / s_curReportSamples) : 0u),
               (unsigned long)s_moveElapsedMs,
               (unsigned)current_VmMilliVolts());
    s_tCurrentReport   = DRIVE_CURRENT_REPORT_MS;
    s_curReportSumMa   = 0;
    s_curReportSamples = 0;
}

/* Soft end-of-travel: once elapsed drive time passes s_softEndAtMs, wind the
 * commanded duty down to g_cfg.softEndDutyPct for the rest of the move so the
 * gate eases into the limit. s_softEndAtMs == 0 disables it for this move. */
static uint32_t s_softEndAtMs = 0;
static bool     s_softEndDone = false;

static void softEndCheck(uint32_t elapsedMs)
{
    if (s_softEndDone || s_softEndAtMs == 0u || elapsedMs < s_softEndAtMs) { return; }
    s_softEndDone = true;
    dbg_Printf("Gate: soft end-of-travel at %lu ms - duty -> %u%%\r\n",
               (unsigned long)elapsedMs, (unsigned)g_cfg.softEndDutyPct);
    drv8262_SetDuty((uint8_t)g_cfg.softEndDutyPct);
}

/* ------------------------------------------------------------ predicates -- */
static bool ocSoftTrip(void) { return s_tOcGrace == 0u && s_tBlank == 0u && current_IsenseMilliAmps() > g_cfg.ocTripMa; }
static bool ocHardTrip(void) { return s_tOcGrace == 0u && current_IsenseMilliAmps() > g_cfg.ocHardLimitMa; }
static bool ocTrip(void)     { return ocSoftTrip() || ocHardTrip(); }
static bool dualLimit(void)  { return input_Level(IN_LIMIT_OPEN) && input_Level(IN_LIMIT_CLOSED); }
static bool nFaultActive(void) { return !drv8262_IsAsleep() && input_Level(IN_NFAULT); }
static bool beamBroken(void) { return input_Level(IN_PHOTOBEAM); }

/* ------------------------------------------------- human-readable labels -- */
/* cmd_t carries no origin, so TOGGLE covers both the button and the remote. */
static const char *cmdSourceStr(cmd_t c)
{
    switch (c) {
        case CMD_OPEN:   return "OPEN button";
        case CMD_CLOSE:  return "CLOSE button";
        case CMD_TOGGLE: return "TOGGLE button/remote";
        case CMD_STOP:   return "STOP command";
        case CMD_ACK:    return "ACK";
        default:          return "command";
    }
}

/* Kept short so "  reason: <text>\r\n" stays inside DBG_PRINTF_BUF_LEN. */
static const char *lockoutReasonStr(lockout_reason_t r)
{
    switch (r) {
        case LOCKOUT_DUAL_LIMIT:     return "both OPEN and CLOSED limit switches read active at once";
        case LOCKOUT_NFAULT_PERSIST: return "DRV8262 nFAULT stayed asserted after 3 clear attempts";
        case LOCKOUT_REPEAT_STALL:   return "too many consecutive stalls - power-cycle to clear";
        default:                      return "unspecified";
    }
}

/* Human-readable gate position/activity for the STATUS command. */
static const char *gateStateStr(gate_state_t st)
{
    switch (st) {
        case ST_INIT:              return "init";
        case ST_IDLE_OPEN:         return "OPEN (at limit)";
        case ST_IDLE_CLOSED:       return "CLOSED (at limit)";
        case ST_OPENING:           return "OPENING";
        case ST_CLOSING:           return "CLOSING";
        case ST_SOFT_STOP:         return "stopping";
        case ST_STOPPED_MIDTRAVEL: return "STOPPED mid-travel";
        case ST_OBSTACLE_REVERSE:  return "reversing off an obstacle";
        case ST_HOMING:            return "HOMING (seeking a limit)";
        case ST_UNKNOWN:           return "UNKNOWN (send a command to home)";
        case ST_STALL_FAULT:       return "STALL FAULT";
        case ST_DRIVER_FAULT:      return "DRIVER FAULT (DRV8262 nFAULT)";
        case ST_LOCKOUT:           return "LOCKED OUT";
        case ST_LOW_BATTERY:       return "LOW BATTERY";
        default:                    return "?";
    }
}

/* Origin-limit-release safety. Call once per driving tick with the tick dt.
 * Returns true (caller should stop) if the limit we started from hasn't
 * disengaged within ORIGIN_LIMIT_RELEASE_MS; disarms itself once it releases. */
static bool originLimitStuck(uint16_t dt)
{
    if (s_originLimitClearMs == 0u) { return false; }          /* not armed */
    if (!input_Level(s_originLimit)) { s_originLimitClearMs = 0u; return false; }  /* released - all good */
    s_originLimitClearMs = (s_originLimitClearMs > dt) ? (uint16_t)(s_originLimitClearMs - dt) : 0u;
    if (s_originLimitClearMs != 0u) { return false; }
    dbg_Printf("FAULT: %s limit still engaged %u ms after starting to drive away - motor not moving, leads reversed, or stuck switch\r\n",
               (s_originLimit == IN_LIMIT_CLOSED) ? "CLOSED" : "OPEN",
               (unsigned)ORIGIN_LIMIT_RELEASE_MS);
    return true;
}

/* --------------------------------------------------------- entry helpers -- */
static void enterMoving(gate_state_t st, gate_dir_t dir, uint16_t dutyPct, uint32_t timeoutMs)
{
    /* Every path into here was reached by popping a command off the inputs FIFO
     * (bridge/HA, RF remote or a wired button), so its origin is still the last
     * one inputs.c handed out. */
    cmd_src_t src = cmd_LastSource();
    if (src != CMDSRC_NONE) { s_lastSrc = src; }

    s_state = st;
    board_SetTickRate(TICK_ACTIVE);
    current_SetMode(CURRENT_MODE_CONTINUOUS);
    if (drv8262_IsAsleep()) { drv8262_Sleep(false); }
    drv8262_SetDir(dir);
#if TEST_MODE
    dutyPct = 100u;   /* bench: full duty so a stiff/unloaded motor breaks free */
#endif
    /* A normal open/close launches at RAMP_START_DUTY so the motor breaks free
     * right away, then ramps to target over RAMP_TIME. Homing stays a gentle
     * ramp up from 0 (it is deliberately a slow limit-seeking creep). */
    if ((st == ST_OPENING || st == ST_CLOSING) && g_cfg.rampStartDutyPct > 0u) {
        drv8262_StartRamp((uint8_t)g_cfg.rampStartDutyPct, (uint8_t)dutyPct);
    } else {
        drv8262_SetDuty((uint8_t)dutyPct);
    }
    metrics_OnMotorStart();
    startDriveCurrentReport();
    capture_Start(st == ST_OPENING ? "OPENING" :
                  st == ST_CLOSING ? "CLOSING" : "HOMING");
    s_tBlank = g_cfg.ocBlankMs;
    s_tOcGrace = g_cfg.ocGraceMs;
    s_tTravel = timeoutMs;
    s_lastDir = dir;
    led_SetFaultPattern(FAULT_LED_OFF);

    /* Soft end-of-travel arming: only for a normal open/close, and only if the
     * matching SOFT_END_* time is set (nonzero). Homing never softens. */
    s_softEndDone = false;
    s_softEndAtMs = (st == ST_OPENING) ? g_cfg.softEndOpenMs
                  : (st == ST_CLOSING) ? g_cfg.softEndCloseMs
                  : 0u;

    /* Announced in the SAME control-loop pass that energised the motor, so the
     * bridge sees a remote-started move as fast as an HA-started one. */
    dbg_Printf("EVT MOVE_START dir=%s src=%s st=%d t=%lu\r\n",
               (dir == DIR_OPEN) ? "OPEN" : "CLOSE", cmdSrcStr(s_lastSrc),
               (int)st, (unsigned long)board_UptimeMs());

    /* Arm the origin-limit-release safety only for a normal open/close that
     * starts with the limit behind us still engaged (not homing, not a
     * mid-travel reversal). */
    s_originLimitClearMs = 0u;
    if (st == ST_OPENING && input_Level(IN_LIMIT_CLOSED)) {
        s_originLimit = IN_LIMIT_CLOSED;
        s_originLimitClearMs = ORIGIN_LIMIT_RELEASE_MS;
    } else if (st == ST_CLOSING && input_Level(IN_LIMIT_OPEN)) {
        s_originLimit = IN_LIMIT_OPEN;
        s_originLimitClearMs = ORIGIN_LIMIT_RELEASE_MS;
    }
}

static void enterSoftStop(gate_state_t next)
{
    s_state = ST_SOFT_STOP;
    s_ssPhase = SS_RAMPING;
    s_softStopNext = next;
    s_softStopHasPendingDir = false;
    s_softStopBackoffMs = 0;
    drv8262_SetDuty(0);
}

static void enterSoftStopWithBackoff(gate_state_t next, uint16_t backoffMs, gate_dir_t backoffDir)
{
    enterSoftStop(next);
    s_softStopBackoffMs = backoffMs;
    s_softStopBackoffDir = backoffDir;
}

static void enterObstacleReverse(gate_dir_t dir)
{
    s_lastSrc = CMDSRC_AUTO_REVERSE;
    s_state = ST_OBSTACLE_REVERSE;
    s_reverseTargetDir = dir;
    drv8262_SetDir(dir);
    drv8262_SetDuty((uint8_t)g_cfg.reverseDutyPct);
    startDriveCurrentReport();
    s_tReverse = g_cfg.obstacleReverseMs;
    s_tBlank = (uint16_t)(g_cfg.ocBlankMs / 2u);   /* short re-blank so it doesn't instantly re-trip */
    s_tOcGrace = 0u;   /* no OC grace during an active obstacle reversal - we want the hard trip live here */
    dbg_Printf("EVT MOVE_START dir=%s src=auto_reverse st=%d t=%lu\r\n",
               (dir == DIR_OPEN) ? "OPEN" : "CLOSE", (int)ST_OBSTACLE_REVERSE,
               (unsigned long)board_UptimeMs());
}

static void enterIdleSettled(gate_state_t st)
{
    s_state = st;
    drv8262_Coast();
    if (g_cfg.idleSleepEnable) { drv8262_Sleep(true); }
    board_SetTickRate(TICK_IDLE);
    current_SetMode(CURRENT_MODE_IDLE);
    led_SetFaultPattern(FAULT_LED_OFF);
    s_tBrake = 0;
}

static void enterIdleFromSoftStop(gate_state_t st)
{
    s_state = st;
    drv8262_Brake();
    s_tBrake = g_cfg.brakeTimeMs;
    led_SetFaultPattern(FAULT_LED_OFF);
    s_stallCount = 0;   /* a full limit-to-limit travel clears prior stall history */
}

/* Limit reached while the motor is still carrying drive current: wind the duty
 * down over BRAKE_RAMP_MS first (SS_PREBRAKE), THEN short-brake. Slamming
 * straight into drv8262_Brake() from full duty dumps the winding energy as a
 * large current spike (visible on IPROPI). */
static void enterBrakeRamp(gate_state_t idleSt)
{
    s_state = ST_SOFT_STOP;
    s_ssPhase = SS_PREBRAKE;
    s_softStopNext = idleSt;
    drv8262_BeginStop(BRAKE_RAMP_MS);
}

static void enterStoppedMidtravel(void)
{
    s_state = ST_STOPPED_MIDTRAVEL;
    drv8262_Coast();
    board_SetTickRate(TICK_IDLE);
    current_SetMode(CURRENT_MODE_IDLE);
    if (g_cfg.idleSleepEnable) { drv8262_Sleep(true); }
    led_SetFaultPattern(FAULT_LED_OFF);
}

static void enterUnknown(void)
{
    s_state = ST_UNKNOWN;
    drv8262_Coast();
    board_SetTickRate(TICK_IDLE);
    current_SetMode(CURRENT_MODE_IDLE);
    if (g_cfg.idleSleepEnable) { drv8262_Sleep(true); }
    led_SetFaultPattern(FAULT_LED_OFF);
    dbg_Print("Gate: position unknown - send a command to home\r\n");
}

static void enterLockout(lockout_reason_t reason)
{
    s_lastSrc = CMDSRC_FAULT;
    s_state = ST_LOCKOUT;
    s_lockoutReason = reason;
    s_lockoutClearMs = 0;
    drv8262_Coast();
    drv8262_Sleep(true);
    board_SetTickRate(TICK_IDLE);
    current_SetMode(CURRENT_MODE_IDLE);
    led_SetFaultPattern(FAULT_LED_FAST_BLINK);
    dbg_Printf("LOCKOUT: %s\r\n", lockoutReasonStr(reason));
}

static void enterStallFault(void)
{
    s_lastSrc = CMDSRC_FAULT;
    s_state = ST_STALL_FAULT;
    drv8262_Coast();
    drv8262_Sleep(true);
    board_SetTickRate(TICK_IDLE);
    current_SetMode(CURRENT_MODE_IDLE);
    led_SetFaultPattern(FAULT_LED_SOLID);
    s_stallCount++;
    dbg_Printf("Gate: stall - MAX_DRIVE_DURATION exceeded, motor disabled (count=%u)\r\n",
               (unsigned)s_stallCount);
    if (s_stallCount >= (uint8_t)g_cfg.stallLockoutCount) {
        enterLockout(LOCKOUT_REPEAT_STALL);
        dbg_Printf("LOCKOUT: %u consecutive stalls - power cycle required\r\n", (unsigned)s_stallCount);
    }
}

static void enterDriverFault(void)
{
    s_lastSrc = CMDSRC_FAULT;
    s_state = ST_DRIVER_FAULT;
    drv8262_Coast();
    board_SetTickRate(TICK_IDLE);
    current_SetMode(CURRENT_MODE_IDLE);
    led_SetFaultPattern(FAULT_LED_FAST_BLINK);
    s_nfaultRetryCount = 0;
    s_tRetryGap = 0;
    dbg_Print("DRV8262 nFAULT asserted\r\n");
}

static void enterLowBattery(void)
{
    s_lastSrc = CMDSRC_FAULT;
    s_state = ST_LOW_BATTERY;
    drv8262_Coast();
    drv8262_Sleep(true);
    board_SetTickRate(TICK_IDLE);
    current_SetMode(CURRENT_MODE_IDLE);
    led_SetFaultPattern(FAULT_LED_SLOW_BLINK);
    s_lowBattRecoverMs = 0;
    dbg_Printf("Battery low (%u mV) - new moves blocked\r\n", (unsigned)current_VmMilliVolts());
}

/* Re-evaluate the limit switches and land in the correct resting state - used
 * at boot, after a lockout/low-battery clears, and after nFAULT clears. */
static void resolveRestState(void)
{
    bool limOpen = input_Level(IN_LIMIT_OPEN);
    bool limClosed = input_Level(IN_LIMIT_CLOSED);
    if (limOpen && limClosed) { enterLockout(LOCKOUT_DUAL_LIMIT); }
    else if (limClosed)       { enterIdleSettled(ST_IDLE_CLOSED); }
    else if (limOpen)         { enterIdleSettled(ST_IDLE_OPEN); }
    else                      { enterUnknown(); }
}

/* Only ever gates the START of a new move - a move already in progress is not
 * aborted by this check (project_plan.md Rev 3 Sec 6.1). Returns true if it
 * just entered ST_LOW_BATTERY (caller should stop processing this tick). */
static bool checkLowBattery(void)
{
    uint16_t vm = current_VmMilliVolts();
    uint16_t dt = board_GetTickMs();
    if (vm > 0u && vm < g_cfg.vmLowCutoffMv) {
        s_lowBattConfirmMs += dt;
        if (s_lowBattConfirmMs >= g_cfg.vmLowConfirmMs) {
            enterLowBattery();
            return true;
        }
    } else {
        s_lowBattConfirmMs = 0;
    }
    return false;
}

/* The heartbeat LED (D6) is driven from board.c's Timer_A2 tick ISR now, not
 * from here, so it keeps blinking even if this super-loop / FSM stalls and in
 * the 50 ms idle tick regime - see TimerA2_ISR in board.c. */

/* ------------------------------------------------------- per-state ticks -- */
static void tickIdleAtLimit(gate_state_t selfSt)
{
    cmd_t cmd;

    if (nFaultActive()) { enterDriverFault(); return; }

    if (s_tBrake > 0u) {
        uint16_t dt = board_GetTickMs();
        s_tBrake = (s_tBrake > dt) ? (uint16_t)(s_tBrake - dt) : 0u;
        if (s_tBrake == 0u) {
            drv8262_Coast();
            if (g_cfg.idleSleepEnable) { drv8262_Sleep(true); }
            board_SetTickRate(TICK_IDLE);
            current_SetMode(CURRENT_MODE_IDLE);
        }
        return;
    }

    if (checkLowBattery()) { return; }

    if (selfSt == ST_IDLE_CLOSED && !input_Level(IN_LIMIT_CLOSED)) { enterUnknown(); return; }
    if (selfSt == ST_IDLE_OPEN   && !input_Level(IN_LIMIT_OPEN))   { enterUnknown(); return; }

    cmd = cmd_Get();
    if (cmd == CMD_NONE) { return; }

    if (selfSt == ST_IDLE_CLOSED) {
        if (cmd == CMD_OPEN || cmd == CMD_TOGGLE) {
            dbg_Print("Gate: opening\r\n");
            enterMoving(ST_OPENING, DIR_OPEN, g_cfg.runDutyPct, (uint32_t)g_cfg.maxDriveDurationS * 1000ul);
        }
    } else {
        if (cmd == CMD_CLOSE || cmd == CMD_TOGGLE) {
            if (beamBroken()) {
                dbg_Print("Gate: close rejected - photo beam broken\r\n");
            } else {
                dbg_Print("Gate: closing\r\n");
                enterMoving(ST_CLOSING, DIR_CLOSE, g_cfg.runDutyPct, (uint32_t)g_cfg.maxDriveDurationS * 1000ul);
            }
        }
    }
}

static void tickOpening(void)
{
    uint16_t dt = board_GetTickMs();
    drv8262_Service();
    metrics_Tick(!drv8262_AtTarget(), drv8262_AtTarget());   /* rampUpActive, accumSteady */
    reportDriveCurrent(dt);
    capture_Tick(dt);
    if (s_tBlank > 0u) { s_tBlank = (s_tBlank > dt) ? (uint16_t)(s_tBlank - dt) : 0u; }
    if (s_tOcGrace > 0u) { s_tOcGrace = (s_tOcGrace > dt) ? (uint16_t)(s_tOcGrace - dt) : 0u; }
    s_tTravel = (s_tTravel > dt) ? (s_tTravel - dt) : 0u;
    softEndCheck((uint32_t)g_cfg.maxDriveDurationS * 1000ul - s_tTravel);

    if (nFaultActive()) { enterDriverFault(); return; }

    if (input_Level(IN_LIMIT_OPEN)) {
        uint16_t peakMa, ssAvgMa, ssMaxMa;
        uint32_t travelMs = (uint32_t)g_cfg.maxDriveDurationS * 1000ul - s_tTravel;
        metrics_Snapshot(&peakMa, &ssAvgMa, &ssMaxMa);
        dbg_BeginBlock();
        dbg_Printf("Gate: reached OPEN limit - travel time %lu ms, avg %u mA, max %u mA\r\n",
                   (unsigned long)travelMs, (unsigned)ssAvgMa, (unsigned)ssMaxMa);
        dbg_Printf("Peak ramp-up current: %u mA\r\n", (unsigned)peakMa);
        dbg_EndBlock();
        enterSoftStop(ST_IDLE_OPEN);
        return;
    }

    if (ocTrip()) {
        s_lastSrc = CMDSRC_OBSTRUCTION_STOP;
        dbg_Printf("OBSTACLE detected while OPENING (over-current%s, %u mA) - halted\r\n",
                   ocHardTrip() ? " HARD" : "", (unsigned)current_IsenseMilliAmps());
        if (g_cfg.openingBackoffMs > 0u) {
            enterSoftStopWithBackoff(ST_STOPPED_MIDTRAVEL, g_cfg.openingBackoffMs, DIR_CLOSE);
        } else {
            enterSoftStop(ST_STOPPED_MIDTRAVEL);
        }
        return;
    }

    if (originLimitStuck(dt)) { enterStoppedMidtravel(); return; }

    if (s_tTravel == 0u) { enterStallFault(); return; }

    {
        cmd_t cmd = cmd_Get();
        if (cmd != CMD_NONE) {
            dbg_Printf("Gate: %s while opening - immediate stop\r\n", cmdSourceStr(cmd));
            enterStoppedMidtravel();
        }
    }
}

static void tickClosing(void)
{
    uint16_t dt = board_GetTickMs();
    drv8262_Service();
    metrics_Tick(!drv8262_AtTarget(), drv8262_AtTarget());   /* rampUpActive, accumSteady */
    reportDriveCurrent(dt);
    capture_Tick(dt);
    if (s_tBlank > 0u) { s_tBlank = (s_tBlank > dt) ? (uint16_t)(s_tBlank - dt) : 0u; }
    if (s_tOcGrace > 0u) { s_tOcGrace = (s_tOcGrace > dt) ? (uint16_t)(s_tOcGrace - dt) : 0u; }
    s_tTravel = (s_tTravel > dt) ? (s_tTravel - dt) : 0u;
    softEndCheck((uint32_t)g_cfg.maxDriveDurationS * 1000ul - s_tTravel);

    if (nFaultActive()) { enterDriverFault(); return; }

    if (input_Level(IN_LIMIT_CLOSED)) {
        uint16_t peakMa, ssAvgMa, ssMaxMa;
        uint32_t travelMs = (uint32_t)g_cfg.maxDriveDurationS * 1000ul - s_tTravel;
        metrics_Snapshot(&peakMa, &ssAvgMa, &ssMaxMa);
        dbg_BeginBlock();
        dbg_Printf("Gate: reached CLOSED limit - travel time %lu ms, avg %u mA, max %u mA\r\n",
                   (unsigned long)travelMs, (unsigned)ssAvgMa, (unsigned)ssMaxMa);
        dbg_Printf("Peak ramp-up current: %u mA\r\n", (unsigned)peakMa);
        dbg_EndBlock();
        enterSoftStop(ST_IDLE_CLOSED);
        return;
    }

    if (ocTrip()) {
        dbg_Printf("OBSTACLE detected while CLOSING (over-current%s, %u mA) - reversing toward OPEN\r\n",
                   ocHardTrip() ? " HARD" : "", (unsigned)current_IsenseMilliAmps());
        enterObstacleReverse(DIR_OPEN);
        return;
    }
    if (beamBroken()) {
        dbg_Print("OBSTACLE detected while CLOSING (photo beam) - reversing toward OPEN\r\n");
        enterObstacleReverse(DIR_OPEN);
        return;
    }
    if (originLimitStuck(dt)) { enterStoppedMidtravel(); return; }

    if (s_tTravel == 0u) { enterStallFault(); return; }

    {
        cmd_t cmd = cmd_Get();
        if (cmd != CMD_NONE) {
            dbg_Printf("Gate: %s while closing - immediate stop\r\n", cmdSourceStr(cmd));
            enterStoppedMidtravel();
        }
    }
}

static void tickSoftStop(void)
{
    uint16_t dt = board_GetTickMs();

    /* Every other "motor is driving" tick handler checks this - SS_BACKOFF
     * actively drives the motor too, and even SS_RAMPING/SS_DEADTIME
     * shouldn't be able to fall through into a fresh enterMoving() (the
     * deadtime->reversal path) without ever having reacted to a fault. */
    if (nFaultActive()) { enterDriverFault(); return; }

    if (s_ssPhase == SS_PREBRAKE) {
        drv8262_Service();              /* fast wind-down of duty toward 0 */
        metrics_Tick(false, false);     /* stopping - keep it out of the averages */
        if (drv8262_DutyIsZero()) {
            enterIdleFromSoftStop(s_softStopNext);   /* now short-brake from ~0, no spike */
        }
        return;
    }

    if (s_ssPhase == SS_RAMPING) {
        cmd_t cmd;
        drv8262_Service();
        metrics_Tick(false, false);

        cmd = cmd_Get();
        if (cmd == CMD_OPEN)  { s_softStopHasPendingDir = true; s_softStopPendingDir = DIR_OPEN; }
        if (cmd == CMD_CLOSE) { s_softStopHasPendingDir = true; s_softStopPendingDir = DIR_CLOSE; }

        if (input_Level(IN_LIMIT_OPEN))   { enterBrakeRamp(ST_IDLE_OPEN); return; }
        if (input_Level(IN_LIMIT_CLOSED)) { enterBrakeRamp(ST_IDLE_CLOSED); return; }

        if (!drv8262_DutyIsZero()) { return; }

        if (s_softStopHasPendingDir) {
            drv8262_Coast();
            s_tDeadtime = g_cfg.deadtimeMs;
            s_ssPhase = SS_DEADTIME;
        } else if (s_softStopBackoffMs > 0u) {
            drv8262_SetDir(s_softStopBackoffDir);
            drv8262_SetDuty((uint8_t)g_cfg.homingDutyPct);
            s_tBackoff = s_softStopBackoffMs;
            s_ssPhase = SS_BACKOFF;
        } else if (s_softStopNext == ST_IDLE_OPEN || s_softStopNext == ST_IDLE_CLOSED) {
            enterIdleFromSoftStop(s_softStopNext);
        } else {
            enterStoppedMidtravel();
        }
        return;
    }

    if (s_ssPhase == SS_DEADTIME) {
        s_tDeadtime = (s_tDeadtime > dt) ? (uint16_t)(s_tDeadtime - dt) : 0u;
        if (s_tDeadtime == 0u) {
            dbg_Printf("Gate: %s\r\n", (s_softStopPendingDir == DIR_OPEN) ? "opening" : "closing");
            enterMoving((s_softStopPendingDir == DIR_OPEN) ? ST_OPENING : ST_CLOSING,
                        s_softStopPendingDir, g_cfg.runDutyPct,
                        (uint32_t)g_cfg.maxDriveDurationS * 1000ul);
        }
        return;
    }

    /* SS_BACKOFF */
    drv8262_Service();
    s_tBackoff = (s_tBackoff > dt) ? (uint16_t)(s_tBackoff - dt) : 0u;
    if (s_tBackoff == 0u || ocHardTrip()) {
        enterStoppedMidtravel();
    }
}

static void tickStoppedMidtravel(void)
{
    cmd_t cmd;

    if (nFaultActive()) { enterDriverFault(); return; }
    if (checkLowBattery()) { return; }
    if (input_Level(IN_LIMIT_OPEN))   { enterIdleSettled(ST_IDLE_OPEN); return; }
    if (input_Level(IN_LIMIT_CLOSED)) { enterIdleSettled(ST_IDLE_CLOSED); return; }

    cmd = cmd_Get();
    if (cmd == CMD_NONE) { return; }

    if (cmd == CMD_OPEN || (cmd == CMD_TOGGLE && s_lastDir == DIR_CLOSE)) {
        dbg_Print("Gate: opening\r\n");
        enterMoving(ST_OPENING, DIR_OPEN, g_cfg.runDutyPct, (uint32_t)g_cfg.maxDriveDurationS * 1000ul);
    } else if (cmd == CMD_CLOSE || (cmd == CMD_TOGGLE && s_lastDir == DIR_OPEN)) {
        if (beamBroken()) {
            dbg_Print("Gate: close rejected - photo beam broken\r\n");
        } else {
            dbg_Print("Gate: closing\r\n");
            enterMoving(ST_CLOSING, DIR_CLOSE, g_cfg.runDutyPct, (uint32_t)g_cfg.maxDriveDurationS * 1000ul);
        }
    }
}

static void tickObstacleReverse(void)
{
    uint16_t dt = board_GetTickMs();
    bool atTarget;
    drv8262_Service();
    metrics_Tick(!drv8262_AtTarget(), drv8262_AtTarget());   /* rampUpActive, accumSteady */
    reportDriveCurrent(dt);
    capture_Tick(dt);
    if (s_tBlank > 0u) { s_tBlank = (s_tBlank > dt) ? (uint16_t)(s_tBlank - dt) : 0u; }
    if (s_tOcGrace > 0u) { s_tOcGrace = (s_tOcGrace > dt) ? (uint16_t)(s_tOcGrace - dt) : 0u; }
    if (s_tReverse > 0u) { s_tReverse = (s_tReverse > dt) ? (uint16_t)(s_tReverse - dt) : 0u; }

    if (nFaultActive()) { enterDriverFault(); return; }

    atTarget = (s_reverseTargetDir == DIR_OPEN) ? input_Level(IN_LIMIT_OPEN) : input_Level(IN_LIMIT_CLOSED);
    if (atTarget) {
        dbg_Print("Gate: reversal reached a limit\r\n");
        enterSoftStop((s_reverseTargetDir == DIR_OPEN) ? ST_IDLE_OPEN : ST_IDLE_CLOSED);
        return;
    }
#if OBSTACLE_REVERSE_OC_STALL
    if (ocTrip()) {
        dbg_Print("Gate: obstacle on both sides - stall\r\n");
        enterStallFault();
        return;
    }
#else
    /* The motor's restart inrush during the reverse can trip ocSoftTrip() even
     * with the halved re-blank; ignore it here so the gate actually backs off
     * the obstacle instead of locking out. A sustained HARD over-current still
     * stops the move - but only to mid-travel, with no stall count / lockout. */
    if (ocHardTrip()) {
        dbg_Print("Gate: hard over-current during reversal - stopped mid-travel\r\n");
        /* Label ONLY - purely diagnostic, does not change WHETHER the gate
         * stops or keeps reversing (that decision is unchanged, above/below).
         * s_lastSrc still reads "auto_reverse" from enterObstacleReverse() at
         * this point, which is technically true (this IS why the reversal
         * attempt started) but reads misleadingly like "it is still reversing"
         * to a human/automation watching cmd_src, when the real outcome here
         * is a safety stop. obstruction_stop is the more honest label for what
         * a viewer now sees: motion stopped because of an obstruction/OC. */
        s_lastSrc = CMDSRC_OBSTRUCTION_STOP;
        enterSoftStop(ST_STOPPED_MIDTRAVEL);
        return;
    }
#endif
    if (s_tReverse == 0u) {
        dbg_Print("Gate: reversal complete, stopped mid-travel - awaiting command\r\n");
        enterSoftStop(ST_STOPPED_MIDTRAVEL);
        return;
    }
    {
        cmd_t c = cmd_Get();
        if (c != CMD_NONE) {
            dbg_Printf("%s ignored - backing off an obstacle, try again shortly\r\n",
                       cmdSourceStr(c));
        }
    }
}

static void tickHoming(void)
{
    uint16_t dt = board_GetTickMs();
    bool reachedHome, reachedOther;
    drv8262_Service();
    metrics_Tick(!drv8262_AtTarget(), drv8262_AtTarget());   /* rampUpActive, accumSteady */
    reportDriveCurrent(dt);
    capture_Tick(dt);
    if (s_tBlank > 0u) { s_tBlank = (s_tBlank > dt) ? (uint16_t)(s_tBlank - dt) : 0u; }
    if (s_tOcGrace > 0u) { s_tOcGrace = (s_tOcGrace > dt) ? (uint16_t)(s_tOcGrace - dt) : 0u; }
    s_tTravel = (s_tTravel > dt) ? (s_tTravel - dt) : 0u;

    if (nFaultActive()) { enterDriverFault(); return; }

    reachedHome  = (s_homeDir == DIR_OPEN) ? input_Level(IN_LIMIT_OPEN)   : input_Level(IN_LIMIT_CLOSED);
    reachedOther = (s_homeDir == DIR_OPEN) ? input_Level(IN_LIMIT_CLOSED) : input_Level(IN_LIMIT_OPEN);

    if (reachedHome) {
        dbg_Print("Gate: homing complete\r\n");
        enterSoftStop((s_homeDir == DIR_OPEN) ? ST_IDLE_OPEN : ST_IDLE_CLOSED);
        return;
    }
    if (reachedOther) {
        dbg_Print("Gate: homing found the opposite limit - position now known\r\n");
        enterSoftStop((s_homeDir == DIR_OPEN) ? ST_IDLE_CLOSED : ST_IDLE_OPEN);
        return;
    }
    if (ocTrip()) {
        if (s_homeDir == DIR_OPEN) {
            s_lastSrc = CMDSRC_OBSTRUCTION_STOP;
            dbg_Print("OBSTACLE detected while homing toward OPEN - halted\r\n");
            if (g_cfg.openingBackoffMs > 0u) {
                enterSoftStopWithBackoff(ST_STOPPED_MIDTRAVEL, g_cfg.openingBackoffMs, DIR_CLOSE);
            } else {
                enterSoftStop(ST_STOPPED_MIDTRAVEL);
            }
        } else {
            dbg_Print("OBSTACLE detected while homing toward CLOSED - reversing toward OPEN\r\n");
            enterObstacleReverse(DIR_OPEN);
        }
        return;
    }
    if (s_tTravel == 0u) { enterStallFault(); return; }
    {
        cmd_t c = cmd_Get();
        if (c != CMD_NONE) {
            dbg_Printf("Gate: homing aborted by %s - immediate stop\r\n", cmdSourceStr(c));
            enterStoppedMidtravel();
        }
    }
}

static void tickUnknown(void)
{
    cmd_t cmd;
    gate_dir_t home;

    if (nFaultActive()) { enterDriverFault(); return; }
    if (checkLowBattery()) { return; }
    if (input_Level(IN_LIMIT_OPEN))   { enterIdleSettled(ST_IDLE_OPEN); return; }
    if (input_Level(IN_LIMIT_CLOSED)) { enterIdleSettled(ST_IDLE_CLOSED); return; }

    cmd = cmd_Get();
    if (cmd == CMD_OPEN)        { home = DIR_OPEN; }
    else if (cmd == CMD_CLOSE)  { home = DIR_CLOSE; }
    else if (cmd == CMD_TOGGLE) { home = DIR_CLOSE; }   /* default homing reference */
    else                        { return; }

    dbg_Printf("Gate: homing toward %s\r\n", (home == DIR_OPEN) ? "OPEN" : "CLOSED");
    s_homeDir = home;
    enterMoving(ST_HOMING, home, g_cfg.homingDutyPct, (uint32_t)g_cfg.homingTimeoutS * 1000ul);
}

static void tickStallFault(void)
{
    cmd_t cmd = cmd_Get();
    if (cmd == CMD_NONE) { return; }
    if (nFaultActive()) { enterDriverFault(); return; }

    dbg_Print("Gate: retrying after stall (homing)\r\n");
    s_homeDir = (s_lastDir == DIR_OPEN) ? DIR_CLOSE : DIR_OPEN;   /* opposite of the direction that stalled */
    enterMoving(ST_HOMING, s_homeDir, g_cfg.homingDutyPct, (uint32_t)g_cfg.homingTimeoutS * 1000ul);
}

static void tickDriverFault(void)
{
    uint16_t dt = board_GetTickMs();

    if (!input_Level(IN_NFAULT)) {
        dbg_Print("DRV8262 nFAULT cleared\r\n");
        enterStoppedMidtravel();
        return;
    }
    if (s_tRetryGap > 0u) {
        s_tRetryGap = (s_tRetryGap > dt) ? (uint16_t)(s_tRetryGap - dt) : 0u;
        return;
    }
    if (s_nfaultRetryCount >= NFAULT_RETRY_MAX) {
        enterLockout(LOCKOUT_NFAULT_PERSIST);
        dbg_Print("LOCKOUT: persistent DRV8262 fault\r\n");
        return;
    }
    s_nfaultRetryCount++;
    dbg_Printf("DRV8262 nFAULT recovery attempt %u\r\n", (unsigned)s_nfaultRetryCount);
    drv8262_ClearFault();
    s_tRetryGap = NFAULT_RETRY_GAP_MS;
}

static void tickLockout(void)
{
    uint16_t dt = board_GetTickMs();
    cmd_t cmd = cmd_Get();
    bool conditionGone = false;

    if (s_lockoutReason == LOCKOUT_DUAL_LIMIT)        { conditionGone = !dualLimit(); }
    else if (s_lockoutReason == LOCKOUT_NFAULT_PERSIST) { conditionGone = !input_Level(IN_NFAULT); }
    /* LOCKOUT_REPEAT_STALL: conditionGone stays false - power cycle only */

    if (conditionGone) { s_lockoutClearMs += dt; } else { s_lockoutClearMs = 0; }

    if (cmd == CMD_NONE) { return; }

    dbg_Printf("%s ignored - controller LOCKED OUT\r\n", cmdSourceStr(cmd));
    dbg_Printf("  reason: %s\r\n", lockoutReasonStr(s_lockoutReason));

    if (s_lockoutReason == LOCKOUT_REPEAT_STALL) {
        dbg_Print("  clear: power-cycle the controller\r\n");
        return;
    }
    if (s_lockoutClearMs >= g_cfg.lockoutClearStableMs) {
        dbg_Print("LOCKOUT: fault gone and stable - cleared\r\n");
        s_lockoutClearMs = 0;
        s_lockoutReason = LOCKOUT_NONE;
        resolveRestState();   /* this press only clears - it does not also start a move */
    } else if (conditionGone) {
        dbg_Printf("  clear: fault gone - hold %u ms more to auto-clear\r\n",
                   (unsigned)(g_cfg.lockoutClearStableMs - s_lockoutClearMs));
    } else {
        dbg_Print("  clear: fault condition still present\r\n");
    }
}

static void tickLowBattery(void)
{
    uint16_t dt = board_GetTickMs();
    uint16_t vm = current_VmMilliVolts();

    {
        cmd_t c = cmd_Get();
        if (c != CMD_NONE) {
            dbg_Printf("%s ignored - low battery (VM %u mV < %u mV)\r\n",
                       cmdSourceStr(c), (unsigned)vm, (unsigned)g_cfg.vmLowRecoverMv);
        }
    }

    if (vm >= g_cfg.vmLowRecoverMv) {
        s_lowBattRecoverMs += dt;
        if (s_lowBattRecoverMs >= g_cfg.lockoutClearStableMs) {
            s_lowBattRecoverMs = 0;
            s_lowBattConfirmMs = 0;
            dbg_Printf("Battery recovered (%u mV) - resuming normal operation\r\n", (unsigned)vm);
            resolveRestState();
        }
    } else {
        s_lowBattRecoverMs = 0;
    }
}

/* ------------------------------------------------------------- public API - */
void gate_fsm_Init(bool limOpen, bool limClosed)
{
    s_stallCount = 0;
    s_lockoutReason = LOCKOUT_NONE;
    if (limOpen && limClosed)  { enterLockout(LOCKOUT_DUAL_LIMIT); }
    else if (limClosed)        { s_lastDir = DIR_CLOSE; enterIdleSettled(ST_IDLE_CLOSED); }
    else if (limOpen)          { s_lastDir = DIR_OPEN;  enterIdleSettled(ST_IDLE_OPEN); }
    else                       { enterUnknown(); }
}

void gate_fsm_Tick(void)
{
    /* When a driving state ends (any exit: limit, stop, fault, lockout), queue
     * the per-move current/VM capture for its paced CSV dump. Detected here so
     * every transition is caught, including the dual-limit early-return below. */
    static gate_state_t s_capPrevState = ST_INIT;
    {
        bool wasDriving = (s_capPrevState == ST_OPENING || s_capPrevState == ST_CLOSING ||
                           s_capPrevState == ST_HOMING  || s_capPrevState == ST_OBSTACLE_REVERSE);
        bool nowDriving = (s_state == ST_OPENING || s_state == ST_CLOSING ||
                           s_state == ST_HOMING  || s_state == ST_OBSTACLE_REVERSE);
        if (wasDriving && !nowDriving) { capture_Finish(); }
        s_capPrevState = s_state;
    }

    current_TickIdle();
    led_Service();

    if (s_state != ST_LOCKOUT && dualLimit()) {
        enterLockout(LOCKOUT_DUAL_LIMIT);
        return;
    }

    switch (s_state) {
        case ST_INIT:               resolveRestState(); break;
        case ST_IDLE_OPEN:          tickIdleAtLimit(ST_IDLE_OPEN); break;
        case ST_IDLE_CLOSED:        tickIdleAtLimit(ST_IDLE_CLOSED); break;
        case ST_OPENING:            tickOpening(); break;
        case ST_CLOSING:            tickClosing(); break;
        case ST_SOFT_STOP:          tickSoftStop(); break;
        case ST_STOPPED_MIDTRAVEL:  tickStoppedMidtravel(); break;
        case ST_OBSTACLE_REVERSE:   tickObstacleReverse(); break;
        case ST_HOMING:              tickHoming(); break;
        case ST_UNKNOWN:            tickUnknown(); break;
        case ST_STALL_FAULT:        tickStallFault(); break;
        case ST_DRIVER_FAULT:       tickDriverFault(); break;
        case ST_LOCKOUT:            tickLockout(); break;
        case ST_LOW_BATTERY:        tickLowBattery(); break;
        default:                     break;
    }
}

gate_state_t gate_fsm_State(void)
{
    return s_state;
}

/* Compact fingerprint of the DISCRETE status fields only - state, the four
 * digital inputs, last direction, sleep, lockout reason, stall count. The live
 * analog readings (sensed current, VM) are deliberately excluded so a poller
 * using STATUS_IF_CHANGED isn't woken by every ADC wobble. */
static uint32_t gateStatusSignature(void)
{
    uint32_t sig = (uint32_t)((uint8_t)s_state & 0x1Fu);
    if (input_Level(IN_LIMIT_OPEN))   { sig |= 1UL << 5; }
    if (input_Level(IN_LIMIT_CLOSED)) { sig |= 1UL << 6; }
    if (input_Level(IN_PHOTOBEAM))    { sig |= 1UL << 7; }
    if (input_Level(IN_NFAULT))       { sig |= 1UL << 8; }
    if (s_lastDir == DIR_OPEN)        { sig |= 1UL << 9; }
    if (drv8262_IsAsleep())           { sig |= 1UL << 10; }
    sig |= (uint32_t)((uint8_t)s_lockoutReason & 0x0Fu) << 11;
    sig |= (uint32_t)s_stallCount << 16;
    return sig;
}

static uint32_t s_lastStatusSig = 0xFFFFFFFFul;   /* impossible value -> first call always prints */

/* The same idea, minus the photo beam: beam edges have their own dedicated,
 * higher-priority event path (beam.c, EVT BEAM) and would otherwise drag a
 * whole state line onto the wire for every wave of a hand. The command source
 * IS included so Home Assistant always learns "who started this" in the same
 * frame as the opening/closing it caused. */
static uint32_t gateEventSignature(void)
{
    uint32_t sig = (uint32_t)((uint8_t)s_state & 0x1Fu);
    if (input_Level(IN_LIMIT_OPEN))   { sig |= 1UL << 5; }
    if (input_Level(IN_LIMIT_CLOSED)) { sig |= 1UL << 6; }
    if (input_Level(IN_NFAULT))       { sig |= 1UL << 7; }
    if (s_lastDir == DIR_OPEN)        { sig |= 1UL << 8; }
    if (drv8262_IsAsleep())           { sig |= 1UL << 9; }
    sig |= (uint32_t)((uint8_t)s_lockoutReason & 0x03u) << 10;
    sig |= (uint32_t)((uint8_t)s_lastSrc & 0x0Fu) << 12;
    sig |= (uint32_t)s_stallCount << 16;
    return sig;
}

/* One compact, machine-parseable line carrying every field the bridge needs to
 * publish a complete gate state. Deliberately ONE line and ~130 bytes: at
 * 115200 baud it is on the wire in ~11 ms, versus ~50 ms for the full STATUS
 * block, and the TX path is interrupt-driven so neither blocks the control
 * loop. Field order is fixed; parsers key on the "<name>=" tokens. */
static void printEventState(uint32_t now)
{
    dbg_Printf("EVT STATE st=%d lo=%u lc=%u bm=%u nf=%u slp=%u sc=%u dir=%s "
               "ma=%u mv=%u lk=%u dl=%u src=%s t=%lu\r\n",
               (int)s_state,
               (unsigned)input_Level(IN_LIMIT_OPEN),
               (unsigned)input_Level(IN_LIMIT_CLOSED),
               (unsigned)input_Level(IN_PHOTOBEAM),
               (unsigned)input_Level(IN_NFAULT),
               (unsigned)drv8262_IsAsleep(),
               (unsigned)s_stallCount,
               (s_lastDir == DIR_OPEN) ? "OPEN" : "CLOSE",
               (unsigned)current_IsenseMilliAmps(),
               (unsigned)current_VmMilliVolts(),
               (unsigned)s_lockoutReason,
               (unsigned)(dualLimit() ? 1u : 0u),
               cmdSrcStr(s_lastSrc),
               (unsigned long)now);
}

void gate_fsm_ReportService(void)
{
    uint32_t now = board_UptimeMs();
    uint32_t sig = gateEventSignature();

    if (sig != s_lastEventSig) {
        s_lastEventSig = sig;
        printEventState(now);
        return;        /* one line is all this change needs - keep the line quiet */
    }

    /* Slow full-status heartbeat. This is only a backstop against a bridge or
     * Home Assistant restart losing the retained state; the real recovery path
     * is the bridge asking for STATUS on boot and on every MQTT reconnect. */
    if ((uint32_t)(now - s_lastHeartbeatMs) >= STATUS_HEARTBEAT_MS) {
        gate_fsm_PrintStatus();
    }
}

/* STATUS_IF_CHANGED: print the full STATUS block only when a discrete field has
 * changed since the last STATUS / STATUS_IF_CHANGED. Returns true if it printed.
 * When nothing changed it emits nothing at all (no body, no ";") so a fast
 * poller doesn't flood the console. */
bool gate_fsm_PrintStatusIfChanged(void)
{
    uint32_t sig = gateStatusSignature();
    if (sig == s_lastStatusSig) { return false; }
    gate_fsm_PrintStatus();               /* also refreshes s_lastStatusSig */
    return true;
}

void gate_fsm_PrintStatus(void)
{
    s_lastStatusSig = gateStatusSignature();   /* re-baseline: a plain STATUS resyncs STATUS_IF_CHANGED */
    s_lastEventSig  = gateEventSignature();    /* ...and the pushed-event baseline */
    s_lastHeartbeatMs = board_UptimeMs();      /* a full block just went out       */
    dbg_BeginBlock();                 /* one multi-line message, one ";" at the end */
    dbg_Printf("Gate: %s\r\n", gateStateStr(s_state));
    dbg_Printf("  OPEN   limit (Arm-Retracted, P7.5) = %u   <- 1 here means gate is at the OPEN end\r\n",
               (unsigned)input_Level(IN_LIMIT_OPEN));
    dbg_Printf("  CLOSED limit (Arm-Extended,  P7.6) = %u   <- 1 here means gate is at the CLOSED end\r\n",
               (unsigned)input_Level(IN_LIMIT_CLOSED));
    dbg_Printf("  (limit inputs: internal pull-up, read as active-%s)\r\n",
               LIMIT_SWITCH_ACTIVE_HIGH ? "HIGH (pin HIGH = at the limit)" : "LOW (pin LOW = at the limit)");
    dbg_Printf("  photo beam broken = %u   nFAULT = %u\r\n",
               (unsigned)input_Level(IN_PHOTOBEAM), (unsigned)input_Level(IN_NFAULT));
    dbg_Printf("State=%d  lastDir=%s\r\n", (int)s_state, (s_lastDir == DIR_OPEN) ? "OPEN" : "CLOSE");
    dbg_Printf("Current: %u mA (%u counts)  VM: %u mV\r\n",
               (unsigned)current_IsenseMilliAmps(), (unsigned)current_IsenseCounts(),
               (unsigned)current_VmMilliVolts());
    dbg_Printf("Stall count: %u  Driver asleep: %u\r\n",
               (unsigned)s_stallCount, (unsigned)drv8262_IsAsleep());
    dbg_Printf("Uptime: %lu ms  src=%s  beam10s=%u  lastbreak=%u ms\r\n",
               (unsigned long)board_UptimeMs(), cmdSrcStr(s_lastSrc),
               (unsigned)beam_BreaksLast10s(), (unsigned)beam_LastBreakMs());
    if (s_state == ST_LOCKOUT) {
        dbg_Printf("LOCKED OUT: %s\r\n", lockoutReasonStr(s_lockoutReason));
    } else if (dualLimit()) {
        dbg_Print("Both limit switches read ACTIVE - will lock out\r\n");
    }
    dbg_EndBlock();
}
