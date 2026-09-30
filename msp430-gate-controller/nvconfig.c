/*******************************************************************************
 * nvconfig.c - see nvconfig.h. Storage: one 128-byte MSP430 Information Memory
 * segment (INFOD, 0x1800-0x187F per lnk_msp430f5338.cmd). g_cfg is the ordinary
 * RAM struct every module reads; the flash-resident blob is only ever touched
 * through flash_EraseSegment()/flash_WriteWords() (a raw store to flash does
 * NOT program it correctly - it must go through the FCTL1/FCTL3 sequence).
 *
 * Note: flashing new firmware from CCS reprograms this segment back to its
 * compiled-in (invalid-magic) default, so saved settings survive normal resets
 * and power cycles, but not a firmware reflash - that's the intended scope.
 ******************************************************************************/
#include <msp430.h>
#include <string.h>
#include "nvconfig.h"
#include "gate_config.h"
#include "board.h"

nvconfig_values_t g_cfg;

typedef struct {
    uint16_t magic;
    uint16_t version;
    nvconfig_values_t values;
    uint16_t crc16;
} nvconfig_blob_t;

#pragma DATA_SECTION(s_flashBlob, ".infoD")
#pragma RETAIN(s_flashBlob)
static const nvconfig_blob_t s_flashBlob = { 0, 0, { 0 }, 0 };

typedef struct {
    const char *name;
    uint16_t   *shadow;
    uint16_t    min;
    uint16_t    max;
    uint16_t    deflt;
    const char *units;
    const char *desc;      /* one-line explanation, printed by LIST_DETAILS */
} nv_param_t;

static const nv_param_t s_params[] = {
    { "MAX_DRIVE_DURATION",      &g_cfg.maxDriveDurationS,     3,   60,   NVCFG_DEFAULT_MAX_DRIVE_DURATION_S,    "s",
      "Max seconds for one open or close drive - if no limit is reached by then the motor is stopped and a stall fault is raised." },
    { "HOMING_TIMEOUT",          &g_cfg.homingTimeoutS,        5,   120,  NVCFG_DEFAULT_HOMING_TIMEOUT_S,        "s",
      "Max seconds for a homing move (run when position is unknown) - no limit found in time raises a stall fault." },
    { "RUN_DUTY",                &g_cfg.runDutyPct,            10,  100,  NVCFG_DEFAULT_RUN_DUTY_PCT,            "%",
      "PWM duty percent for a normal open or close move once it has ramped up to speed." },
    { "HOMING_DUTY",             &g_cfg.homingDutyPct,         10,  100,  NVCFG_DEFAULT_HOMING_DUTY_PCT,         "%",
      "PWM duty percent while homing - driving toward a limit switch to re-learn the gate position." },
    { "REVERSE_DUTY",            &g_cfg.reverseDutyPct,        10,  100,  NVCFG_DEFAULT_REVERSE_DUTY_PCT,        "%",
      "PWM duty percent used to drive back toward OPEN after an obstacle is hit while closing." },
    { "RAMP_TIME",                &g_cfg.rampTimeMs,           100, 5000, NVCFG_DEFAULT_RAMP_TIME_MS,            "ms",
      "Milliseconds to ramp duty RAMP_START_DUTY->target at the start of an open/close and target->0 at a normal stop (soft start / soft stop)." },
    { "RAMP_START_DUTY",          &g_cfg.rampStartDutyPct,     0,   100,  NVCFG_DEFAULT_RAMP_START_DUTY_PCT,     "%",
      "Duty an open/close LAUNCHES at, then ramps up to RUN_DUTY over RAMP_TIME. Set near where the motor breaks free so it starts moving at once instead of after a slow ramp from 0. 0 = ramp from a standstill. Homing and obstacle-reverse ignore this." },
    { "OBSTACLE_REVERSE_TIME",   &g_cfg.obstacleReverseMs,     100, 3000, NVCFG_DEFAULT_OBSTACLE_REVERSE_MS,     "ms",
      "Milliseconds to drive in reverse after an obstacle is hit while closing, before stopping mid-travel." },
    { "OPENING_BACKOFF",         &g_cfg.openingBackoffMs,      0,   2000, NVCFG_DEFAULT_OPENING_BACKOFF_MS,      "ms",
      "Milliseconds of gentle reverse after an over-current while opening, to unload the jam before stopping (0 = none)." },
    { "BRAKE_TIME",               &g_cfg.brakeTimeMs,          0,   5000, NVCFG_DEFAULT_BRAKE_TIME_MS,           "ms",
      "Milliseconds the motor is short-braked (windings shorted) after reaching a limit, before the driver coasts and sleeps." },
    { "OC_TRIP_CURRENT",         &g_cfg.ocTripMa,              500, 9400, NVCFG_DEFAULT_OC_TRIP_MA,              "mA",
      "Soft obstacle threshold: sensed motor current above this, after the startup blanking window, stops or reverses the gate. Kept below OC_HARD_LIMIT." },
    { "OC_HARD_LIMIT",           &g_cfg.ocHardLimitMa,         500, 9400, NVCFG_DEFAULT_OC_HARD_LIMIT_MA,        "mA",
      "Hard obstacle threshold: sensed current above this reacts immediately, even during startup blanking (jam or partial short). Kept above OC_TRIP_CURRENT." },
#if VREF_DAC_ENABLE
    { "VREF_CURRENT_REGULATION", &g_cfg.vrefCurrentRegulationMa, 500, 9400, NVCFG_DEFAULT_VREF_CURRENT_REGULATION_MA, "mA",
      "DRV8262 hardware current-regulation point (I_TRIP). Real hardware control: drives VREF1 via MSP430 DAC12_1 over P6.7 (EVM jumpered J25+J6) - see drv8262_SetCurrentLimitMa(). The DAC output clamps at AVCC (~3.3 V) above roughly 9.4 A, matching the ADC current-sense ceiling, so values near/above the top of this range behave the same as 9400." },
#else
    { "VREF_CURRENT_REGULATION", &g_cfg.vrefCurrentRegulationMa, 500, 9400, NVCFG_DEFAULT_VREF_CURRENT_REGULATION_MA, "mA",
      "DRV8262 hardware current-regulation point (I_TRIP) - hardware control is currently DISABLED (VREF_DAC_ENABLE=0 in gate_config.h, pending bench verification). VREF1 is the original fixed ~3.3 V GPIO reference (~9.4 A with R81||R82 = 1650 ohm); this value is stored/range-checked but does NOT move the hardware trip, same as the old CURRENT_LIMIT. See drv8262_SetCurrentLimitMa()." },
#endif
    { "STALL_LOCKOUT_COUNT",     &g_cfg.stallLockoutCount,     1,   20,   NVCFG_DEFAULT_STALL_LOCKOUT_COUNT,     "",
      "Number of consecutive stall faults that escalates to a full lockout requiring a power cycle." },
    { "LOCKOUT_CLEAR_STABLE_TIME", &g_cfg.lockoutClearStableMs, 0,  10000,NVCFG_DEFAULT_LOCKOUT_CLEAR_STABLE_MS, "ms",
      "Milliseconds the lockout cause (e.g. both limits active) must stay clear before a button press is allowed to clear the lockout." },
    { "IDLE_SLEEP",              &g_cfg.idleSleepEnable,       0,   1,    NVCFG_DEFAULT_IDLE_SLEEP_ENABLE,       "",
      "1 = sleep the DRV8262 (outputs off, low power) whenever the gate is at rest. 0 = keep it powered." },
    { "OC_BLANK_TIME",           &g_cfg.ocBlankMs,             100, 20000,NVCFG_DEFAULT_OC_BLANK_MS,             "ms",
      "Startup blanking window in ms where OC_TRIP_CURRENT (the soft trip) is ignored to allow motor inrush - OC_HARD_LIMIT still applies unless OC_GRACE_TIME also covers it." },
    { "OC_GRACE_TIME",           &g_cfg.ocGraceMs,             0,   20000,NVCFG_DEFAULT_OC_GRACE_MS,             "ms",
      "Window from the start of every move where BOTH OC_TRIP_CURRENT and OC_HARD_LIMIT are ignored, to ride through a heavy startup surge. 0 = disabled. Trades startup obstacle detection for reliability - the stall timeout and stuck-limit checks still stop a jammed gate." },
    { "VM_LOW_CUTOFF",           &g_cfg.vmLowCutoffMv,         8000,14000,NVCFG_DEFAULT_VM_LOW_CUTOFF_MV,        "mV",
      "Motor supply mV below this (held for VM_LOW_CONFIRM_TIME) blocks new moves and enters low-battery. Kept below VM_LOW_RECOVER." },
    { "VM_LOW_RECOVER",          &g_cfg.vmLowRecoverMv,        8000,14000,NVCFG_DEFAULT_VM_LOW_RECOVER_MV,       "mV",
      "Motor supply mV that VM must climb back above (and hold) to leave low-battery and run again. Kept above VM_LOW_CUTOFF." },
    { "VM_LOW_CONFIRM_TIME",     &g_cfg.vmLowConfirmMs,        0,   5000, NVCFG_DEFAULT_VM_LOW_CONFIRM_MS,       "ms",
      "Milliseconds VM must stay under VM_LOW_CUTOFF before low-battery is entered (ignores a brief sag)." },
    { "DEADTIME",                &g_cfg.deadtimeMs,            0,   1000, NVCFG_DEFAULT_DEADTIME_MS,             "ms",
      "Milliseconds with both motor legs off when reversing direction mid-move, so the H-bridge fully turns off before driving the other way." },
    { "SOFT_END_DUTY",           &g_cfg.softEndDutyPct,        10,  100,  NVCFG_DEFAULT_SOFT_END_DUTY_PCT,       "%",
      "Duty a normal open/close ramps down to after SOFT_END_OPEN/CLOSE ms, so the gate eases into the end-of-travel limit instead of hitting it at full drive." },
    { "SOFT_END_OPEN",           &g_cfg.softEndOpenMs,         0,   60000,NVCFG_DEFAULT_SOFT_END_OPEN_MS,        "ms",
      "Elapsed ms into an OPEN drive after which duty drops to SOFT_END_DUTY (0 = disabled). Set it a bit under your typical open travel time from the capture logs." },
    { "SOFT_END_CLOSE",          &g_cfg.softEndCloseMs,        0,   60000,NVCFG_DEFAULT_SOFT_END_CLOSE_MS,       "ms",
      "Elapsed ms into a CLOSE drive after which duty drops to SOFT_END_DUTY (0 = disabled). Set it a bit under your typical close travel time from the capture logs." },
};
#define NUM_PARAMS (sizeof(s_params) / sizeof(s_params[0]))

/* ------------------------------------------------------------- flash driver */
/* FCTL3's LOCK bit only protects main flash. Info Memory (segments B/C/D,
 * which includes .infoD used here) has its own LOCKINFO bit in FCTL4, which
 * defaults SET (locked) out of reset - unlocking only FCTL3 leaves LOCKINFO
 * set, so every erase/write below would fault (ACCVIFG) and silently do
 * nothing without also clearing/restoring FCTL4.LOCKINFO here. */
static void flash_EraseSegment(void *segStart)
{
    __disable_interrupt();
    FCTL3 = FWKEY;                     /* unlock main flash              */
    FCTL4 = FWKEY;                     /* unlock Info Memory (LOCKINFO)  */
    FCTL1 = FWKEY | ERASE;
    *(uint8_t *)segStart = 0;          /* dummy write triggers the segment erase */
    while (FCTL3 & BUSY) { }
    FCTL1 = FWKEY;                     /* clear ERASE */
    FCTL3 = FWKEY | LOCK;
    FCTL4 = FWKEY | LOCKINFO;
    __enable_interrupt();
}

static void flash_WriteWords(uint16_t *dst, const uint16_t *src, uint16_t count)
{
    uint16_t i;
    __disable_interrupt();
    FCTL3 = FWKEY;
    FCTL4 = FWKEY;
    FCTL1 = FWKEY | WRT;
    for (i = 0; i < count; i++) {
        dst[i] = src[i];
        while (FCTL3 & BUSY) { }
    }
    FCTL1 = FWKEY;
    FCTL3 = FWKEY | LOCK;
    FCTL4 = FWKEY | LOCKINFO;
    __enable_interrupt();
}

/* ---------------------------------------------------------------- CRC16 --- */
static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, b;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void nvconfig_ResetDefaults(void)
{
    uint16_t i;
    for (i = 0; i < NUM_PARAMS; i++) {
        *(s_params[i].shadow) = s_params[i].deflt;
    }
}

void nvconfig_Save(void)
{
    nvconfig_blob_t local;
    local.magic   = NVCONFIG_MAGIC;
    local.version = NVCONFIG_VERSION;
    local.values  = g_cfg;
    local.crc16   = crc16((const uint8_t *)&local, (uint16_t)(sizeof(local) - sizeof(local.crc16)));

    wdt_Kick();                             /* flash erase/write blocks a few ms - kick before/after */
    flash_EraseSegment((void *)&s_flashBlob);
    flash_WriteWords((uint16_t *)&s_flashBlob, (const uint16_t *)&local,
                      (uint16_t)(sizeof(local) / sizeof(uint16_t)));
    wdt_Kick();
}

void nvconfig_Init(void)
{
    nvconfig_blob_t local = s_flashBlob;   /* ordinary read - fine without the flash sequence */
    uint16_t crc = crc16((const uint8_t *)&local, (uint16_t)(sizeof(local) - sizeof(local.crc16)));

    if (local.magic == NVCONFIG_MAGIC && local.version == NVCONFIG_VERSION && crc == local.crc16) {
        g_cfg = local.values;
    } else {
        nvconfig_ResetDefaults();
        nvconfig_Save();                    /* commit sane defaults on first boot / blank flash */
    }
}

static const nv_param_t *find_param(const char *name)
{
    uint16_t i;
    for (i = 0; i < NUM_PARAMS; i++) {
        if (strcmp(s_params[i].name, name) == 0) {
            return &s_params[i];
        }
    }
    return 0;
}

nv_result_t nvconfig_SetByName(const char *name, int32_t value)
{
    const nv_param_t *p = find_param(name);
    if (!p) {
        return NV_ERR_UNKNOWN_NAME;
    }
    if (value < (int32_t)p->min || value > (int32_t)p->max) {
        return NV_ERR_RANGE;
    }
    /* Two-tier over-current thresholds must stay ordered (Rev 3 Sec 8/10). */
    if (strcmp(name, "OC_TRIP_CURRENT") == 0 && (uint16_t)value >= g_cfg.ocHardLimitMa) {
        return NV_ERR_ORDERING;
    }
    if (strcmp(name, "OC_HARD_LIMIT") == 0 && (uint16_t)value <= g_cfg.ocTripMa) {
        return NV_ERR_ORDERING;
    }
    if (strcmp(name, "VM_LOW_RECOVER") == 0 && (uint16_t)value <= g_cfg.vmLowCutoffMv) {
        return NV_ERR_ORDERING;
    }
    if (strcmp(name, "VM_LOW_CUTOFF") == 0 && (uint16_t)value >= g_cfg.vmLowRecoverMv) {
        return NV_ERR_ORDERING;
    }
    *(p->shadow) = (uint16_t)value;
    return NV_OK;
}

bool nvconfig_GetByName(const char *name, uint16_t *out)
{
    const nv_param_t *p = find_param(name);
    if (!p) {
        return false;
    }
    *out = *(p->shadow);
    return true;
}

void nvconfig_ForEach(void (*cb)(const char *name, uint16_t value, uint16_t deflt, const char *units))
{
    uint16_t i;
    for (i = 0; i < NUM_PARAMS; i++) {
        cb(s_params[i].name, *(s_params[i].shadow), s_params[i].deflt, s_params[i].units);
    }
}

void nvconfig_ForEachDetail(void (*cb)(const char *name, const char *units, const char *desc))
{
    uint16_t i;
    for (i = 0; i < NUM_PARAMS; i++) {
        cb(s_params[i].name, s_params[i].units, s_params[i].desc);
    }
}
