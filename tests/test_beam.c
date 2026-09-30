/* Host test harness for beam.c - the photo-beam edge reporter and
 * vehicle-pass classifier. Compiles the REAL beam.c (no copy of its logic) with
 * stubs for board/inputs/console/gate_fsm, then drives it with the beam
 * waveforms from the firmware change request's acceptance tests.
 *
 * Runs the FULL suite twice via run_tests.ps1: once with beam.c's compiled-in
 * default (BEAM_AXLE_PATTERN_ENABLE=0, this install's actual behavior - the
 * beam is body-height, confirmed from real captures), and once built with
 * -DBEAM_AXLE_PATTERN_ENABLE=1 to keep the axle-pair code path covered for any
 * install where the beam sits low enough to see only tyres. Every acceptance
 * check below branches on #if BEAM_AXLE_PATTERN_ENABLE for the parts of the
 * result that legitimately differ (grouping, timing) between the two modes.
 *
 * Build (default / this install's mode):
 *   cl /nologo /W3 /I.. test_beam.c ..\beam.c
 * Build (axle-pair mode, for coverage of that path):
 *   cl /nologo /W3 /I.. /DBEAM_AXLE_PATTERN_ENABLE=1 test_beam.c ..\beam.c
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "gate_config.h"
#include "gate_types.h"
#include "beam.h"

/* ---- simulated environment ------------------------------------------------ */
static uint32_t g_nowMs   = 0;
static bool     g_beamRaw = false;      /* true = beam broken (debounced level) */
static gate_state_t g_state = ST_IDLE_OPEN;

uint32_t board_UptimeMs(void)          { return g_nowMs; }
bool     input_Level(input_id_t id)    { return (id == IN_PHOTOBEAM) ? g_beamRaw : false; }
gate_state_t gate_fsm_State(void)      { return g_state; }

/* ---- captured console output ---------------------------------------------- */
#define MAX_LINES 256
static char g_lines[MAX_LINES][256];
static int  g_nLines = 0;

void dbg_Printf(const char *fmt, ...)
{
    va_list ap;
    if (g_nLines >= MAX_LINES) { printf("FAIL: line capture overflow\n"); exit(1); }
    va_start(ap, fmt);
    vsnprintf(g_lines[g_nLines], sizeof(g_lines[0]), fmt, ap);
    va_end(ap);
    /* strip the trailing CRLF the firmware appends */
    {
        size_t n = strlen(g_lines[g_nLines]);
        while (n > 0 && (g_lines[g_nLines][n-1] == '\r' || g_lines[g_nLines][n-1] == '\n')) {
            g_lines[g_nLines][--n] = '\0';
        }
    }
    g_nLines++;
}
void dbg_Print(const char *s) { (void)s; }

/* ---- test driver ----------------------------------------------------------- */
static int g_fail = 0;

#define TICK_MS 10u      /* the firmware's idle tick after this change */

static void resetCapture(void) { g_nLines = 0; }

/* Advance the simulated clock in TICK_MS steps, calling beam_Tick() each step,
 * exactly as the super-loop does. */
static void advance(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms; i += TICK_MS) {
        g_nowMs += TICK_MS;
        beam_Tick();
    }
}

static void beamSet(bool blocked) { g_beamRaw = blocked; }

static void dump(const char *label)
{
    int i;
    printf("  [%s] %d line(s)\n", label, g_nLines);
    for (i = 0; i < g_nLines; i++) { printf("      %s\n", g_lines[i]); }
}

static int countLines(const char *needle)
{
    int i, n = 0;
    for (i = 0; i < g_nLines; i++) { if (strstr(g_lines[i], needle)) n++; }
    return n;
}
static const char *findLine(const char *needle)
{
    int i;
    for (i = 0; i < g_nLines; i++) { if (strstr(g_lines[i], needle)) return g_lines[i]; }
    return NULL;
}
/* Nth (0-based) line containing `needle`, in emission order. */
static const char *findLineNth(const char *needle, int n)
{
    int i;
    for (i = 0; i < g_nLines; i++) {
        if (strstr(g_lines[i], needle)) { if (n == 0) return g_lines[i]; n--; }
    }
    return NULL;
}

static void check(int cond, const char *what)
{
    printf("    %-62s %s\n", what, cond ? "PASS" : "**FAIL**");
    if (!cond) g_fail++;
}

/* Extract the integer following `key` in `line`. -1 if absent. */
static long fieldOf(const char *line, const char *key)
{
    const char *p = line ? strstr(line, key) : NULL;
    if (!p) return -1;
    return strtol(p + strlen(key), NULL, 10);
}

/* ============================================================================ */
int main(void)
{
#if BEAM_AXLE_PATTERN_ENABLE
    printf("beam.c host tests - AXLE-PAIR MODE (tick = %u ms)\n", (unsigned)TICK_MS);
#else
    printf("beam.c host tests - DEFAULT MODE, axle-pair OFF (tick = %u ms)\n", (unsigned)TICK_MS);
#endif
    beam_Init();
    advance(5000);                       /* settle; nothing should be emitted */
    resetCapture();

    /* ---- Acceptance test 3: a ~200 ms hand wave --------------------------- */
    printf("\n[T3] single 200 ms break -> two edges, dur ~200 ms, then short_break\n");
    beamSet(true);  advance(200);
    beamSet(false); advance(100);
    {
        const char *blocked = findLine("EVT BEAM b=1");
        const char *clear   = findLine("EVT BEAM b=0");
        long dur = fieldOf(clear, "dur=");
        check(blocked != NULL, "blocked edge reported");
        check(clear != NULL,   "clear edge reported");
        check(dur >= 150 && dur <= 250, "break duration within +/-50 ms of 200 ms");
        printf("      measured dur=%ld ms\n", dur);
#if BEAM_AXLE_PATTERN_ENABLE
        check(countLines("EVT BEAMACT") == 0, "no activity event yet (window still open, axle mode)");
        advance(BEAM_AXLE_GAP_MAX_MS + 100);
        check(countLines("EVT BEAMACT") == 1, "activity event fired after the gap timeout");
#else
        check(countLines("EVT BEAMACT") == 1,
              "activity event fired IMMEDIATELY on clear (axle pattern off - nothing to wait for)");
#endif
        {
            const char *act = findLine("EVT BEAMACT");
            check(act != NULL, "activity event present");
            check(act && strstr(act, "ev=short_break"), "classified short_break (person/dog)");
            check(act && strstr(act, "n=1"), "reported exactly one break");
        }
        dump("T3");
    }
    resetCapture();

    /* ---- Acceptance test 4: two waves ~1 s apart -------------------------- */
    printf("\n[T4] two 200 ms breaks 1000 ms apart\n");
    beamSet(true);  advance(200);
    beamSet(false); advance(1000);
    beamSet(true);  advance(200);
    beamSet(false); advance(BEAM_AXLE_GAP_MAX_MS + 100);
    check(countLines("EVT BEAM b=1") == 2, "two separate blocked edges reported");
    check(countLines("EVT BEAM b=0") == 2, "two separate clear edges reported");
#if BEAM_AXLE_PATTERN_ENABLE
    {
        const char *act = findLine("EVT BEAMACT");
        check(countLines("EVT BEAMACT") == 1, "exactly one activity event (grouped)");
        check(act && strstr(act, "ev=vehicle_pass"), "classified vehicle_pass");
        check(act && strstr(act, "pat=axle_pair"),   "pattern axle_pair");
        check(act && strstr(act, "n=2"),             "two breaks counted");
    }
#else
    {
        const char *a1 = findLineNth("EVT BEAMACT", 0);
        const char *a2 = findLineNth("EVT BEAMACT", 1);
        check(countLines("EVT BEAMACT") == 2,
              "TWO separate activity events (axle pattern off - not grouped)");
        check(a1 && strstr(a1, "ev=short_break") && strstr(a1, "n=1"), "first break -> its own short_break(n=1)");
        check(a2 && strstr(a2, "ev=short_break") && strstr(a2, "n=1"), "second break -> its own short_break(n=1)");
        check(a1 && a2 && (fieldOf(a2, "t=") - fieldOf(a1, "t=")) < 1500,
              "second event arrives close behind its own break, not 3.5 s later");
    }
#endif
    dump("T4");
    resetCapture();

    /* ---- Acceptance test 5a: a real car, tyres only (low beam) ------------ */
    printf("\n[T5a] axle pair: 140 ms / 1180 ms gap / 130 ms\n");
    beamSet(true);  advance(140);
    beamSet(false); advance(1180);
    beamSet(true);  advance(130);
    beamSet(false); advance(BEAM_AXLE_GAP_MAX_MS + 100);
#if BEAM_AXLE_PATTERN_ENABLE
    {
        const char *act = findLine("EVT BEAMACT");
        check(act && strstr(act, "ev=vehicle_pass"), "classified vehicle_pass");
        check(act && strstr(act, "pat=axle_pair"),   "pattern axle_pair");
        check(act && strstr(act, "brk="),            "break list present");
        check(act && strstr(act, "gap="),            "gap list present");
    }
#else
    {
        check(countLines("EVT BEAMACT") == 2, "two SEPARATE short_break events, not one axle_pair");
        check(countLines("ev=vehicle_pass") == 0,
              "NOT misclassified as a vehicle (this is exactly the false-positive this mode prevents)");
    }
#endif
    dump("T5a");
    resetCapture();

    /* ---- Acceptance test 5b: a real car, body (high beam) ----------------- */
    printf("\n[T5b] body: one 2000 ms break - result must be identical in both modes\n");
    beamSet(true);  advance(2000);
    beamSet(false);
#if BEAM_AXLE_PATTERN_ENABLE
    advance(BEAM_AXLE_GAP_MAX_MS + 100);
#else
    advance(50);   /* just long enough for the immediate-classify tick to run */
#endif
    {
        const char *act = findLine("EVT BEAMACT");
        check(act && strstr(act, "ev=vehicle_pass"), "classified vehicle_pass");
        check(act && strstr(act, "pat=body"),        "pattern body");
        check(act && strstr(act, "n=1"),             "one break");
#if !BEAM_AXLE_PATTERN_ENABLE
        check(act && fieldOf(act, "t=") <= 12100,
              "published within ~100 ms of the clear edge (t=12000), not 3.5 s later");
        printf("      published at t=%ld ms (break cleared at t=12000 ms)\n", act ? fieldOf(act, "t=") : -1);
#endif
    }
    dump("T5b");
    resetCapture();

    /* ---- car + trailer: 4 axles ------------------------------------------ */
    printf("\n[T5c] car + trailer: four 140 ms breaks, 1 s gaps\n");
    {
        int i;
        for (i = 0; i < 4; i++) {
            beamSet(true);  advance(140);
            beamSet(false); advance(i < 3 ? 1000 : (BEAM_AXLE_GAP_MAX_MS + 100));
        }
#if BEAM_AXLE_PATTERN_ENABLE
        {
            const char *act = findLine("EVT BEAMACT");
            check(act && strstr(act, "ev=vehicle_pass"), "classified vehicle_pass");
            check(act && strstr(act, "n=4"),             "four breaks counted");
        }
#else
        check(countLines("EVT BEAMACT") == 4, "four SEPARATE short_break events, not one grouped vehicle_pass");
        check(countLines("ev=vehicle_pass") == 0, "never misclassified as a vehicle");
#endif
        dump("T5c");
    }
    resetCapture();

    /* ---- Acceptance test 6: long block - unaffected by axle setting ------- */
    printf("\n[T6] 10 s continuous block -> long_block at 6 s, follow-up on clear (both modes identical)\n");
    beamSet(true);
    advance(BEAM_BODY_BREAK_MAX_MS - 500);
    check(countLines("EVT BEAMACT") == 0, "nothing published before the 6 s threshold");
    advance(1000);
    {
        const char *act = findLine("ev=long_block");
        check(act != NULL, "long_block published as soon as it reached 6 s");
        check(act && fieldOf(act, "tot=") >= (long)BEAM_BODY_BREAK_MAX_MS,
              "early long_block carries >= 6000 ms so far");
    }
    advance(4000);
    beamSet(false); advance(100);
    {
        check(countLines("ev=long_block") == 2, "follow-up long_block on clear");
        check(findLine("ev=long_block") && strstr(findLine("ev=long_block"), " n=1 "),
              "early long_block reports one break, not two");
        const char *last = NULL;
        int i;
        for (i = 0; i < g_nLines; i++) if (strstr(g_lines[i], "ev=long_block")) last = g_lines[i];
        check(last && fieldOf(last, "tot=") >= 9500 && fieldOf(last, "tot=") <= 10500,
              "follow-up total duration ~10 s");
        printf("      follow-up tot=%ld ms\n", fieldOf(last, "tot="));
        dump("T6");
    }
    advance(BEAM_AXLE_GAP_MAX_MS + 100);
    check(countLines("EVT BEAMACT") == 2, "window closed - no extra classification event");
    resetCapture();

    /* ---- below the axle minimum: a 20 ms glitch -------------------------- */
    printf("\n[T7] 20 ms glitch -> reported, but not a vehicle\n");
    beamSet(true);  advance(20);
    beamSet(false);
#if BEAM_AXLE_PATTERN_ENABLE
    advance(BEAM_AXLE_GAP_MAX_MS + 100);
#else
    advance(50);
#endif
    {
        const char *act = findLine("EVT BEAMACT");
        check(act && strstr(act, "ev=short_break"), "classified short_break");
        dump("T7");
    }
    resetCapture();

    /* ---- breaks_last_10s rolling counter (unrelated to axle setting) ------ */
    printf("\n[T8] breaks_last_10s counter\n");
    advance(11000);                      /* let earlier tests' breaks age out */
    {
        int i;
        for (i = 0; i < 3; i++) {
            beamSet(true);  advance(100);
            beamSet(false); advance(400);
        }
        check(beam_BreaksLast10s() == 3, "three breaks counted inside the 10 s window");
        advance(11000);
        check(beam_BreaksLast10s() == 0, "counter decays to zero after 10 s");
    }
    resetCapture();

    /* ---- chatter guard (unrelated to axle setting) ------------------------ */
    printf("\n[T9] chatter guard: 40 edges in ~1 s\n");
    {
        int i;
        for (i = 0; i < 20; i++) {
            beamSet(true);  advance(20);
            beamSet(false); advance(30);
        }
        int edges = countLines("EVT BEAM b=");
        check(edges < 40, "per-edge lines suppressed while chattering");
        check(countLines("chat=1") > 0, "chat=1 flag raised");
        printf("      %d edge line(s) emitted for 40 physical edges\n", edges);
        advance(BEAM_AXLE_GAP_MAX_MS + 2000);
    }
    resetCapture();

    /* ---- gate_state context is the state at window OPEN ------------------- */
    printf("\n[T10] gs= carries the gate state from when the window opened\n");
    g_state = ST_CLOSING;
    beamSet(true);  advance(200);
    beamSet(false);
    g_state = ST_IDLE_OPEN;                     /* gate finishes moving mid-window */
#if BEAM_AXLE_PATTERN_ENABLE
    advance(BEAM_AXLE_GAP_MAX_MS + 100);
#else
    advance(50);
#endif
    {
        const char *act = findLine("EVT BEAMACT");
        check(act && fieldOf(act, "gs=") == (long)ST_CLOSING,
              "gs= is the state at window open, not at publish");
        dump("T10");
    }

    printf("\n%s (%d failure(s))\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
