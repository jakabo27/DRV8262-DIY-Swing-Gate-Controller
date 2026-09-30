/*******************************************************************************
 * nvconfig.h - runtime-tunable configuration, persisted in flash (INFOD).
 * Every other module reads g_cfg instead of a #define for anything that can be
 * changed from the UART console (project_plan.md Rev 3 Sec 8/9).
 ******************************************************************************/
#ifndef NVCONFIG_H
#define NVCONFIG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t maxDriveDurationS;     /* MAX_DRIVE_DURATION,       s   */
    uint16_t homingTimeoutS;        /* HOMING_TIMEOUT,           s   */
    uint16_t runDutyPct;            /* RUN_DUTY,                 %   */
    uint16_t homingDutyPct;         /* HOMING_DUTY,              %   */
    uint16_t reverseDutyPct;        /* REVERSE_DUTY,             %   */
    uint16_t rampTimeMs;            /* RAMP_TIME,                ms  */
    uint16_t obstacleReverseMs;     /* OBSTACLE_REVERSE_TIME,    ms  */
    uint16_t openingBackoffMs;      /* OPENING_BACKOFF,          ms  */
    uint16_t brakeTimeMs;           /* BRAKE_TIME,               ms  */
    uint16_t ocTripMa;              /* OC_TRIP_CURRENT,          mA  */
    uint16_t ocHardLimitMa;         /* OC_HARD_LIMIT,            mA  */
    uint16_t vrefCurrentRegulationMa; /* VREF_CURRENT_REGULATION (hw DAC12_1), mA */
    uint16_t stallLockoutCount;     /* STALL_LOCKOUT_COUNT           */
    uint16_t lockoutClearStableMs;  /* LOCKOUT_CLEAR_STABLE_TIME,ms  */
    uint16_t idleSleepEnable;       /* IDLE_SLEEP,               0/1 */
    uint16_t ocBlankMs;             /* OC_BLANK_TIME,            ms  */
    uint16_t vmLowCutoffMv;         /* VM_LOW_CUTOFF,            mV  */
    uint16_t vmLowRecoverMv;        /* VM_LOW_RECOVER,           mV  */
    uint16_t vmLowConfirmMs;        /* VM_LOW_CONFIRM_TIME,      ms  */
    uint16_t deadtimeMs;            /* DEADTIME,                 ms  */
    uint16_t softEndDutyPct;        /* SOFT_END_DUTY,            %   */
    uint16_t softEndOpenMs;         /* SOFT_END_OPEN,    ms (0=off)  */
    uint16_t softEndCloseMs;        /* SOFT_END_CLOSE,   ms (0=off)  */
    uint16_t rampStartDutyPct;     /* RAMP_START_DUTY,          %   */
    uint16_t ocGraceMs;            /* OC_GRACE_TIME,           ms  */
} nvconfig_values_t;

extern nvconfig_values_t g_cfg;    /* live RAM shadow - everyone reads this   */

void nvconfig_Init(void);          /* load from flash, or seed+save defaults  */
void nvconfig_Save(void);          /* erase+rewrite the flash segment         */
void nvconfig_ResetDefaults(void); /* reset g_cfg only (flash unchanged until Save) */

typedef enum {
    NV_OK = 0,
    NV_ERR_UNKNOWN_NAME,
    NV_ERR_RANGE,
    NV_ERR_ORDERING
} nv_result_t;

nv_result_t nvconfig_SetByName(const char *name, int32_t value);
bool        nvconfig_GetByName(const char *name, uint16_t *out);

/* Invoked once per parameter, in table order, for the LIST command. */
void nvconfig_ForEach(void (*cb)(const char *name, uint16_t value, uint16_t deflt, const char *units));

/* Invoked once per parameter, in table order, for the LIST_DETAILS command. */
void nvconfig_ForEachDetail(void (*cb)(const char *name, const char *units, const char *desc));

#endif /* NVCONFIG_H */
