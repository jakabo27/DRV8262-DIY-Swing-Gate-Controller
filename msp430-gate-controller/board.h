/*******************************************************************************
 * board.h - BSP: clock, GPIO map, LEDs, dynamic Timer_A2 tick, watchdog.
 * Owns every pin direction/function on the board. DRV8262 control-pin *levels*
 * are driven by drv8262.c through the drvpin_* helpers below so drv8262.c never
 * touches a port register directly.
 ******************************************************************************/
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "gate_types.h"

/* volatile state touched by ISRs, read by main.c's super-loop */
extern volatile uint8_t  g_tickFlag;   /* set once per Timer_A2 period, cleared by main.c */
extern volatile uint32_t g_tickCount;  /* free-running, wraps; ticks are NOT constant duration */

void board_Init(void);

/* Reprogram Timer_A2 for the 1 ms (TICK_ACTIVE) or 50 ms (TICK_IDLE) period.
 * Safe to call every tick if desired (a no-op if already at that rate). */
void board_SetTickRate(tick_rate_t rate);
uint16_t board_GetTickMs(void);        /* 1 or 10, matching the current rate    */

/* Free-running milliseconds since boot. Accumulated in the Timer_A2 ISR from
 * the CURRENT tick duration, so it stays correct across tick-rate switches
 * (g_tickCount cannot be used for this - its ticks are not constant duration).
 * Wraps after ~49.7 days; every consumer uses unsigned differences, which are
 * wrap-safe.
 *
 * DELIBERATELY NOT interrupt-protected, even though the 32-bit load is not
 * atomic on this 16-bit core - see board.c for why. Called from beam_Tick()
 * on every tick during a drive (up to 1 kHz), so it sits directly beside the
 * current-sense ADC pipeline that over-current protection depends on; any
 * added interrupt-disabled window here is a latency risk to that pipeline
 * that is not worth taking for a logging/reporting timestamp. Worst case on
 * an unlucky torn read is one wrong `t=` value that self-corrects on the very
 * next call - never a safety-relevant consequence. */
uint32_t board_UptimeMs(void);

/* The single place LPM0+GIE is issued - call once per super-loop pass. On MSP430,
 * an ISR only wakes the CPU out of LPMx if it calls __bic_SR_register_on_exit();
 * only TimerA2_ISR does that here, so this only ever wakes on the next Timer_A2
 * tick. The ADC12/UART ISRs still run and service their peripherals (buffering
 * RX bytes, latching conversions) while asleep, but don't wake the super-loop
 * early - it picks their results up on the next tick (up to 50 ms later while
 * TICK_IDLE, which is fine for a UART console and a ~1 Hz VM sample). */
void board_SleepUntilTick(void);

void wdt_Kick(void);

/* LEDs */
void led_Set(led_id_t id, bool on);
void led_Toggle(led_id_t id);
void led_SetFaultPattern(fault_led_pattern_t pattern); /* LED_FAULT only */
void led_Service(void);                                /* advance blink patterns; call once/tick */

/* DRV8262 control-pin helpers (levels only - direction/function set in board_Init) */
void drvpin_SetSleep(bool assertLow);   /* true = nSLEEP low (asleep/reset)      */
void drvpin_SetMode1(bool high);
void drvpin_SetMode2(bool high);
void drvpin_SetDecay0(bool high);
void drvpin_SetToffCtrl(bool high);
bool drvpin_ReadNFault(void);           /* true = fault line low (asserted)      */
bool drvpin_ReadNHome(void);

/* Raw (undebounced) electrical level of a digital input - true = pin HIGH.
 * inputs.c applies the active-high/active-low polarity per signal. */
bool pin_RawRead(input_id_t id);

#endif /* BOARD_H */
