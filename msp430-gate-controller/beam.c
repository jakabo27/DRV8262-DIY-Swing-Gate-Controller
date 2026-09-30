/*******************************************************************************
 * beam.c - see beam.h.
 *
 * Everything here is pure observation on top of inputs.c's debounced
 * IN_PHOTOBEAM level. No safety decision is taken or influenced in this file.
 ******************************************************************************/
#include <stddef.h>
#include "beam.h"
#include "board.h"
#include "inputs.h"
#include "console.h"
#include "gate_config.h"
#include "gate_fsm.h"

/* ---- live edge state ----------------------------------------------------- */
static bool     s_blocked      = false;
static uint32_t s_breakStartMs = 0;
static uint16_t s_lastBreakMs  = 0;
static uint32_t s_lastEdgeMs   = 0;
static bool     s_longBlockSent = false;   /* the early long_block already went out */

/* ---- classification window ----------------------------------------------- */
static bool         s_winOpen     = false;
static uint32_t     s_winStartMs  = 0;
static uint32_t     s_lastClearMs = 0;
static gate_state_t s_winState    = ST_INIT;
static uint16_t     s_breakMs[BEAM_MAX_BREAKS];
static uint16_t     s_gapMs[BEAM_MAX_BREAKS];
static uint8_t      s_nBreakRec   = 0;    /* durations actually stored (capped) */
static uint8_t      s_nGapRec     = 0;
static uint16_t     s_nBreakTotal = 0;    /* true break count for this window    */

/* ---- rolling counters ----------------------------------------------------- *
 * s_blkTs holds the uptime of the last BEAM_RATE_RING blocked edges; 0 means
 * "slot never used" (uptime 0 is the first millisecond after reset, so the only
 * cost of that sentinel is one possible miscount in that first millisecond). */
static uint32_t s_blkTs[BEAM_RATE_RING];
static uint8_t  s_blkIdx = 0;

/* One-second edge-rate window feeding the chatter guard. */
static uint32_t s_rateWinStartMs = 0;
static uint16_t s_rateWinEdges   = 0;
static bool     s_chatter        = false;
static uint32_t s_lastChatPrintMs = 0;

/* --------------------------------------------------------------- helpers -- */

/* "142,131" into dst, or "-" when the list is empty. Never overruns dst. */
static void fmtList(char *dst, uint8_t cap, const uint16_t *v, uint8_t n)
{
    uint8_t i;
    uint8_t o = 0;

    if (cap < 2u) { if (cap) { dst[0] = '\0'; } return; }
    if (n == 0u) { dst[0] = '-'; dst[1] = '\0'; return; }

    for (i = 0u; i < n; i++) {
        uint16_t x = v[i];
        char     tmp[6];
        int8_t   k = 0;
        if (i != 0u && o < (uint8_t)(cap - 1u)) { dst[o++] = ','; }
        if (x == 0u) { tmp[k++] = '0'; }
        while (x > 0u && k < 5) { tmp[k++] = (char)('0' + (x % 10u)); x /= 10u; }
        while (k > 0 && o < (uint8_t)(cap - 1u)) { dst[o++] = tmp[--k]; }
    }
    dst[o] = '\0';
}

static void noteBlockedEdge(uint32_t now)
{
    s_blkTs[s_blkIdx] = now;
    s_blkIdx = (uint8_t)((s_blkIdx + 1u) % BEAM_RATE_RING);
}

/* Edge-rate bookkeeping. Returns true if the beam is currently "chattering"
 * (more than BEAM_CHATTER_EDGES_PER_S edges inside one 1 s window) - sun glare,
 * rain or a bug on the lens. While chattering we stop printing a line per edge
 * so a flapping sensor cannot fill the UART ring; the debounced level the
 * safety logic reads is completely unaffected. */
static bool noteEdgeRate(uint32_t now)
{
    uint32_t elapsed = (uint32_t)(now - s_rateWinStartMs);
    if (elapsed >= 1000ul) {
        /* The window that just closed only means anything if this edge actually
         * follows it. More than one window's worth of silence means the beam has
         * been quiet, so whatever the stale counter says it is not chattering -
         * without this the flag stayed latched onto the next edge, however much
         * later it came. */
        s_chatter        = (elapsed < 2000ul) && (s_rateWinEdges > BEAM_CHATTER_EDGES_PER_S);
        s_rateWinStartMs = now;
        s_rateWinEdges   = 0u;
    }
    s_rateWinEdges++;
    if (s_rateWinEdges > BEAM_CHATTER_EDGES_PER_S) { s_chatter = true; }
    return s_chatter;
}

static void emitEdge(uint32_t now, bool blocked, uint16_t durMs, bool chatter)
{
    if (chatter) {
        /* At most one line per BEAM_CHATTER_MIN_GAP_MS while chattering. */
        if ((uint32_t)(now - s_lastChatPrintMs) < BEAM_CHATTER_MIN_GAP_MS) { return; }
        s_lastChatPrintMs = now;
        dbg_Printf("EVT BEAM b=%u t=%lu dur=%u n10=%u chat=1\r\n",
                   (unsigned)(blocked ? 1u : 0u), (unsigned long)now,
                   (unsigned)durMs, (unsigned)beam_BreaksLast10s());
        return;
    }
    dbg_Printf("EVT BEAM b=%u t=%lu dur=%u n10=%u\r\n",
               (unsigned)(blocked ? 1u : 0u), (unsigned long)now,
               (unsigned)durMs, (unsigned)beam_BreaksLast10s());
}

static void emitActivity(const char *ev, const char *pat, uint16_t nBreaks,
                         uint32_t totalMs, uint32_t now)
{
    char brk[56];
    char gap[56];

    fmtList(brk, (uint8_t)sizeof(brk), s_breakMs, s_nBreakRec);
    fmtList(gap, (uint8_t)sizeof(gap), s_gapMs,   s_nGapRec);

    dbg_Printf("EVT BEAMACT ev=%s pat=%s n=%u tot=%lu gs=%d t=%lu brk=%s gap=%s\r\n",
               ev, pat, (unsigned)nBreaks, (unsigned long)totalMs,
               (int)s_winState, (unsigned long)now, brk, gap);
}

static void windowReset(void)
{
    s_winOpen     = false;
    s_nBreakRec   = 0u;
    s_nGapRec     = 0u;
    s_nBreakTotal = 0u;
}

/* Pattern A ("axle pair", low beam, sees only the tyres): two or more short
 * breaks separated by a wheelbase-sized gap. Pattern B ("body", high beam):
 * one long break. Anything else is a person/animal (short_break). */
static void classifyAndPublish(uint32_t now)
{
    uint8_t     i;
    uint8_t     axleBreaks = 0u;
    uint8_t     axleGaps   = 0u;
    const char *ev;
    const char *pat;
    uint32_t    totalMs = (uint32_t)(s_lastClearMs - s_winStartMs);

    for (i = 0u; i < s_nBreakRec; i++) {
        if (s_breakMs[i] >= BEAM_AXLE_BREAK_MIN_MS && s_breakMs[i] <= BEAM_AXLE_BREAK_MAX_MS) {
            axleBreaks++;
        }
    }
    for (i = 0u; i < s_nGapRec; i++) {
        if (s_gapMs[i] >= BEAM_AXLE_GAP_MIN_MS && s_gapMs[i] <= BEAM_AXLE_GAP_MAX_MS) {
            axleGaps++;
        }
    }

#if BEAM_AXLE_PATTERN_ENABLE
    if (s_nBreakTotal >= 2u && axleBreaks >= 2u && axleGaps >= 1u) {
        ev = "vehicle_pass"; pat = "axle_pair";       /* a trailer just adds breaks */
    } else
#else
    (void)axleBreaks; (void)axleGaps;   /* computed above regardless; unused with axle pattern off */
#endif
    if (s_nBreakTotal == 1u && s_nBreakRec == 1u &&
               s_breakMs[0] >= BEAM_BODY_BREAK_MIN_MS &&
               s_breakMs[0] <= BEAM_BODY_BREAK_MAX_MS) {
        ev = "vehicle_pass"; pat = "body";
    } else {
        ev = "short_break";  pat = "none";
    }
    emitActivity(ev, pat, s_nBreakTotal, totalMs, now);
    windowReset();
}

/* ------------------------------------------------------------ public API -- */
void beam_Init(void)
{
    uint8_t i;
    for (i = 0u; i < BEAM_RATE_RING; i++) { s_blkTs[i] = 0ul; }
    s_blkIdx        = 0u;
    s_blocked       = input_Level(IN_PHOTOBEAM);
    s_breakStartMs  = board_UptimeMs();
    s_lastBreakMs   = 0u;
    s_lastEdgeMs    = 0ul;
    s_longBlockSent = false;
    s_rateWinStartMs = board_UptimeMs();
    s_rateWinEdges  = 0u;
    s_chatter       = false;
    s_lastChatPrintMs = 0ul;
    s_lastClearMs   = board_UptimeMs();
    s_winState      = ST_INIT;
    windowReset();
}

void beam_Tick(void)
{
    uint32_t now     = board_UptimeMs();
    bool     blocked = input_Level(IN_PHOTOBEAM);

    if (blocked != s_blocked) {
        bool chatter = noteEdgeRate(now);
        s_blocked    = blocked;
        s_lastEdgeMs = now;

        if (blocked) {
            s_breakStartMs  = now;
            s_longBlockSent = false;
            noteBlockedEdge(now);
            emitEdge(now, true, 0u, chatter);

            if (!s_winOpen) {
                s_winOpen    = true;
                s_winStartMs = now;
                s_winState   = gate_fsm_State();
            } else if (s_nGapRec < BEAM_MAX_BREAKS) {
                uint32_t g = (uint32_t)(now - s_lastClearMs);
                s_gapMs[s_nGapRec++] = (g > 65535ul) ? 65535u : (uint16_t)g;
            }
        } else {
            uint32_t d = (uint32_t)(now - s_breakStartMs);
            s_lastBreakMs = (d > 65535ul) ? 65535u : (uint16_t)d;
            s_lastClearMs = now;
            emitEdge(now, false, s_lastBreakMs, chatter);

            s_nBreakTotal++;
            if (s_nBreakRec < BEAM_MAX_BREAKS) { s_breakMs[s_nBreakRec++] = s_lastBreakMs; }

            if (s_longBlockSent) {
                /* The early long_block already told Home Assistant something is
                 * standing in the beam; this is the promised follow-up carrying
                 * the total duration. No further classification for this window. */
                emitActivity("long_block", "none", s_nBreakTotal, d, now);
                windowReset();
                s_longBlockSent = false;
            }
#if !BEAM_AXLE_PATTERN_ENABLE
            else if (s_winOpen) {
                /* Axle-pair is disabled on this install (see gate_config.h) so
                 * a second break could never turn this into anything other than
                 * what it already is - waiting BEAM_AXLE_GAP_MAX_MS "just in
                 * case" only adds latency for nothing. Classify and publish the
                 * instant this break clears instead. */
                classifyAndPublish(now);
            }
#endif
        }
        return;   /* one edge per tick is all the 10 ms sampling can produce */
    }

    /* Still blocked and it has gone on too long: publish long_block NOW rather
     * than waiting for it to clear, so HA learns something is parked in the
     * beam while it is still happening. */
    if (blocked && !s_longBlockSent &&
        (uint32_t)(now - s_breakStartMs) >= BEAM_BODY_BREAK_MAX_MS) {
        s_longBlockSent = true;
        if (!s_winOpen) { s_winOpen = true; s_winStartMs = s_breakStartMs; s_winState = gate_fsm_State(); }
        /* n=1 explicitly: this break has not ended yet, so it is not counted in
         * s_nBreakTotal - and must not be pre-counted there either, or the
         * follow-up event on the clear edge would report the same break twice. */
        emitActivity("long_block", "none", 1u, (uint32_t)(now - s_breakStartMs), now);
        return;
    }

    /* Window closes once the beam has been clear for a full inter-axle gap. */
    if (s_winOpen && !blocked && (uint32_t)(now - s_lastClearMs) >= BEAM_AXLE_GAP_MAX_MS) {
        classifyAndPublish(now);
    }
}

bool beam_Blocked(void) { return s_blocked; }

uint16_t beam_BreaksLast10s(void)
{
    uint32_t now = board_UptimeMs();
    uint16_t n   = 0u;
    uint8_t  i;
    for (i = 0u; i < BEAM_RATE_RING; i++) {
        if (s_blkTs[i] != 0ul && (uint32_t)(now - s_blkTs[i]) <= BEAM_RATE_WINDOW_MS) { n++; }
    }
    return n;
}

uint16_t beam_LastBreakMs(void) { return s_lastBreakMs; }
uint32_t beam_LastEdgeMs(void)  { return s_lastEdgeMs; }
