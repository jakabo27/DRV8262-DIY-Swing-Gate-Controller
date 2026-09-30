/*******************************************************************************
 * gate_fsm.h - the gate state machine. Pure logic on top of board/drv8262/
 * current/inputs/console - see project_plan.md Rev 3 Sec 6 for the full
 * transition table this implements.
 ******************************************************************************/
#ifndef GATE_FSM_H
#define GATE_FSM_H

#include <stdbool.h>
#include "gate_types.h"

void gate_fsm_Init(bool limOpen, bool limClosed);
void gate_fsm_Tick(void);          /* call once per tick, either regime */
gate_state_t gate_fsm_State(void);
void gate_fsm_PrintStatus(void);   /* for the console's STATUS command  */

/* Call once per tick, immediately after gate_fsm_Tick(). Pushes one compact
 * "EVT STATE ..." line the moment any discrete field changes (so a move that
 * started from the RF remote reaches the bridge in the same control-loop pass
 * that energised the motor), and a full STATUS block every
 * STATUS_HEARTBEAT_MS as a backstop. Emits nothing otherwise. */
void gate_fsm_ReportService(void);
bool gate_fsm_PrintStatusIfChanged(void); /* STATUS_IF_CHANGED: prints only on a discrete-field change; true if it printed */

#endif /* GATE_FSM_H */
