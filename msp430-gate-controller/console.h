/*******************************************************************************
 * console.h - human-readable debug output + the SET_, LIST, SAVE command line.
 *
 * HARD RULES (project_plan.md Rev 3 Sec 9.4 - checked by code review, not just
 * convention):
 *   1. dbg_Printf formats into ONE static scratch buffer - never a per-call
 *      stack allocation.
 *   2. No ISR anywhere in this project may call dbg_Print/dbg_Printf/console_*.
 *      ISRs only push raw bytes/counts and set flags.
 *   3. The RX line buffer has a hard cap (CONSOLE_RX_LINE_MAX); an overlong
 *      line gets "ERR LINE_TOO_LONG" and is discarded, never silently acted on.
 ******************************************************************************/
#ifndef CONSOLE_H
#define CONSOLE_H

void console_Init(void);
void dbg_Print(const char *s);           /* one message; appends the ";" marker  */
void dbg_Printf(const char *fmt, ...);   /* %d %u %ld %lu %s %% only - no floats  */
void dbg_PrintRaw(const char *s);        /* fragment: no [epoch], no ";" marker   */
void dbg_PrintfRaw(const char *fmt, ...);/* fragment: no [epoch], no ";" marker   */
void dbg_BeginBlock(void);               /* start a multi-line message ...        */
void dbg_EndBlock(void);                 /* ... which ends with one ";" marker    */
const char *fw_VersionString(void);      /* "GateController_DRV8262 <date> <time>"*/
void console_Service(void);              /* main-loop only; call once per tick    */

#endif /* CONSOLE_H */
