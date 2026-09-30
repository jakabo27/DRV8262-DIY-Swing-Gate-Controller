/*******************************************************************************
 * drv8262.h - DRV8262 bring-up, PWM + soft ramp, sleep/wake, fault-clear.
 * Forward drive (DIR_OPEN, IN1 as the active PWM leg) opens the gate, reverse
 * drive (DIR_CLOSE, IN2 active) closes it - confirm on the bench; if backwards,
 * swap the two motor leads at the DRV8262 OUT terminals (project_plan.md
 * Rev 2 Sec 7.1 / Sec 12).
 ******************************************************************************/
#ifndef DRV8262_H
#define DRV8262_H

#include <stdint.h>
#include <stdbool.h>
#include "gate_types.h"

void drv8262_Init(void);

void drv8262_SetDir(gate_dir_t dir);
void drv8262_SetDuty(uint8_t pct);        /* 0-100, ramped target                */
void drv8262_StartRamp(uint8_t startPct, uint8_t targetPct); /* launch at startPct, slew to targetPct over RAMP_TIME */
void drv8262_Coast(void);                 /* both legs low, ramp reset           */
void drv8262_Brake(void);                 /* both legs high, ramp reset          */
void drv8262_Service(void);               /* call every ACTIVE (1 ms) tick only  */
bool drv8262_DutyIsZero(void);
void drv8262_SetRampTimeMs(uint16_t ms);  /* recompute the Q8 ramp step          */
void drv8262_BeginStop(uint16_t ms);      /* fast wind-down of duty to 0, then Brake() */
bool drv8262_AtTarget(void);              /* ramp has reached the commanded duty */

void drv8262_SetCurrentLimitMa(uint16_t ma); /* DRV8262 hardware I_TRIP backstop */
void drv8262_ClearFault(void);             /* brief nSLEEP pulse, ~30us + 1.5ms wake */
bool drv8262_ReadNFaultRaw(void);

void drv8262_Sleep(bool asleep);           /* battery-saving idle; halts PWM too */
bool drv8262_IsAsleep(void);

#endif /* DRV8262_H */
