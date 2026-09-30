/*******************************************************************************
 * inputs.h - debounced polling of every digital input (P6/P7/P8 have no
 * pin-change interrupt on this device, so everything is polled once per tick)
 * plus the resulting operator-command FIFO.
 ******************************************************************************/
#ifndef INPUTS_H
#define INPUTS_H

#include <stdint.h>
#include <stdbool.h>
#include "gate_types.h"

void inputs_Init(void);
void inputs_Poll(void);              /* call once per tick, either regime */

bool input_Level(input_id_t id);     /* debounced logical level (active = true) */
bool input_RisingEdge(input_id_t id);  /* one-shot, consumed on read */
bool input_FallingEdge(input_id_t id); /* one-shot, consumed on read */
bool input_AnyButtonPressed(void);

cmd_t cmd_Get(void);                 /* pop the next queued operator command    */

/* Origin of the command most recently returned by cmd_Get(). CMDSRC_NONE until
 * the first command is popped. Valid until the next cmd_Get() call. */
cmd_src_t cmd_LastSource(void);

/* Queue a command from outside the button matrix. inputs_InjectCommand() keeps
 * its old signature (console/bridge origin) so existing callers are unchanged;
 * inputs_InjectCommandSrc() is the explicit form. */
void  inputs_InjectCommand(cmd_t cmd);
void  inputs_InjectCommandSrc(cmd_t cmd, cmd_src_t src);

#endif /* INPUTS_H */
