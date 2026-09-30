/*******************************************************************************
 * current.h - the single ADC12 owner: continuous IPROPI averaging while the
 * motor is driven, on-demand single-shot VM reads while idle. Every channel
 * uses ADC12SREF_0 (AVCC/AVSS) - no external VeREF+ wiring required, and it
 * puts the ADC's full 4095-count range at ~AVCC, matching the DRV8262
 * hardware current-limit ceiling (project_plan.md Rev 3 Sec 7.2/7.3).
 ******************************************************************************/
#ifndef CURRENT_H
#define CURRENT_H

#include <stdint.h>
#include "gate_types.h"

void current_Init(void);
void current_SetMode(current_mode_t mode);
void current_Service(void);     /* call every super-loop pass - drains ADC results */
void current_TickIdle(void);    /* call once per tick - no-op unless in IDLE mode  */
void current_RequestVm(void);   /* idle mode: kick one on-demand VM conversion     */

uint16_t current_IsenseCounts(void);
uint16_t current_IsenseMilliAmps(void);
uint16_t current_VmMilliVolts(void);   /* live in BOTH modes: VM is interleaved
                                          into the IPROPI sequence while driving  */

void metrics_OnMotorStart(void);
void metrics_Tick(bool rampUpActive, bool accumSteady);
void metrics_Snapshot(uint16_t *peakRampMa, uint16_t *ssAvgMa, uint16_t *ssMaxMa);

/* ---- Per-move data capture (see gate_config.h CAPTURE_*) ----------------- *
 * gate_fsm drives these: capture_Start() at each motor start, capture_Tick()
 * every driving super-loop pass, capture_Finish() when the move ends (queues
 * the CSV dump). capture_DumpService() is pumped from the main loop and emits
 * the dump paced against the UART ring. */
void capture_Start(const char *label);
void capture_Tick(uint16_t dt);
void capture_Finish(void);
void capture_DumpService(void);

#endif /* CURRENT_H */
