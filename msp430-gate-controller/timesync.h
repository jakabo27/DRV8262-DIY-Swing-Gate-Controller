/*******************************************************************************
 * timesync.h - wall-clock time tracking synced from an external time source
 * (the ESP32, once its EZFET jumpers on J10 are pulled and it drives the
 * console UART directly) over the same UART/console line as uart.c/console.c.
 * The MSP430 has no RTC/battery-backed clock of its own: timesync_TryHandleLine()
 * consumes "TIME:<unix_epoch>\n" packets and timesync_Tick() advances the
 * estimate between syncs using the same per-tick mechanism as the rest of the
 * app (board.c's Timer_A2), so it keeps counting even if the ESP32 goes quiet.
 ******************************************************************************/
#ifndef TIMESYNC_H
#define TIMESYNC_H

#include <stdint.h>
#include <stdbool.h>

void timesync_Init(void);

/* Call once per tick (main loop) with the elapsed ms for that tick
 * (board_GetTickMs()) to advance the time estimate between syncs. */
void timesync_Tick(uint16_t elapsedMs);

/* Feed one already NUL-terminated console line (no \r\n, as produced by
 * console.c's line assembler). Returns true if the line was a "TIME:<unix_epoch>"
 * sync packet (consumed - caller must not also pass it to command dispatch),
 * false if it's an ordinary console line the caller should handle as before. */
bool timesync_TryHandleLine(const char *line);

bool timesync_IsSynced(void);      /* true once at least one valid sync has landed */
uint32_t timesync_Now(void);       /* current estimated Unix epoch seconds, 0 if never synced */

#endif /* TIMESYNC_H */
