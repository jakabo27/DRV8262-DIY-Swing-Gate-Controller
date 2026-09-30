/*******************************************************************************
 * timesync.c - see timesync.h. Pure time bookkeeping - no UART/console access
 * of its own; console.c owns the wire format and feeds lines in.
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include "timesync.h"

static bool     s_synced          = false;
static uint32_t s_epochAtSync     = 0;
static uint32_t s_secondsSinceSync = 0;
static uint16_t s_msAccumulator   = 0;

void timesync_Init(void)
{
    s_synced           = false;
    s_epochAtSync      = 0;
    s_secondsSinceSync = 0;
    s_msAccumulator    = 0;
}

void timesync_Tick(uint16_t elapsedMs)
{
    if (!s_synced) {
        return;
    }
    s_msAccumulator = (uint16_t)(s_msAccumulator + elapsedMs);
    while (s_msAccumulator >= 1000u) {
        s_msAccumulator = (uint16_t)(s_msAccumulator - 1000u);
        s_secondsSinceSync++;
    }
}

bool timesync_TryHandleLine(const char *line)
{
    static const char PREFIX[] = "TIME:";
    const size_t prefixLen = sizeof(PREFIX) - 1u;
    char *endp;
    unsigned long v;

    if (strncmp(line, PREFIX, prefixLen) != 0) {
        return false;
    }
    v = strtoul(line + prefixLen, &endp, 10);
    if (endp == line + prefixLen) {
        return true;    /* "TIME:" with no digits - consumed, but ignored */
    }
    s_epochAtSync      = (uint32_t)v;
    s_secondsSinceSync = 0;
    s_msAccumulator    = 0;
    s_synced           = true;
    return true;
}

bool timesync_IsSynced(void)
{
    return s_synced;
}

uint32_t timesync_Now(void)
{
    return s_synced ? (s_epochAtSync + s_secondsSinceSync) : 0u;
}
