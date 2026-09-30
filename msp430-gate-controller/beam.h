/*******************************************************************************
 * beam.h - photo-beam edge reporting and beam-activity classification.
 *
 * This module OBSERVES the debounced photo-beam input and reports on it. It
 * never drives the state machine: obstacle reversal, the "close rejected -
 * photo beam broken" check and every other safety decision still read
 * input_Level(IN_PHOTOBEAM) directly in gate_fsm.c, exactly as before. Nothing
 * here can change when or whether the gate reverses.
 *
 * What it produces on the console (one ';'-framed message each, immediately):
 *
 *   EVT BEAM b=<0|1> t=<uptime_ms> dur=<ms> n10=<breaks in last 10 s> [chat=1]
 *       One line per debounced edge. b=1 is blocked, b=0 is clear; dur is the
 *       length of the break that just ended (0 on a blocked edge). There is no
 *       latch and no minimum hold - a 200 ms break is reported as 200 ms.
 *
 *   EVT BEAMACT ev=<vehicle_pass|short_break|long_block> pat=<axle_pair|body|none>
 *               n=<breaks> tot=<ms> gs=<gate_state_t> t=<uptime_ms>
 *               brk=<d1,d2,...> gap=<g1,g2,...>
 *       One line per classification window. A window opens on the first blocked
 *       edge and closes once the beam has been clear for BEAM_AXLE_GAP_MAX_MS.
 *       gs is the gate state when the window OPENED (a single beam cannot tell
 *       which way a car is travelling, so Home Assistant needs that context).
 ******************************************************************************/
#ifndef BEAM_H
#define BEAM_H

#include <stdint.h>
#include <stdbool.h>

void beam_Init(void);

/* Call once per tick, immediately after inputs_Poll() and BEFORE the FSM tick,
 * so a beam edge is always the first thing queued for transmission that pass. */
void beam_Tick(void);

bool     beam_Blocked(void);          /* debounced "beam broken" level         */
uint16_t beam_BreaksLast10s(void);    /* rolling count, for the HA attribute   */
uint16_t beam_LastBreakMs(void);      /* duration of the most recent break     */
uint32_t beam_LastEdgeMs(void);       /* uptime ms at the most recent edge     */

#endif /* BEAM_H */
