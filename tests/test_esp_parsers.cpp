/* Host test for the ESP32 bridge's message parsers.
 *
 * esp_parsers.inc is sliced verbatim out of src/main.cpp by gen_esp_test.py, so
 * these tests exercise the shipping code, not a copy of it. Everything
 * ESP-IDF-specific below is a stub that records what would have been published.
 *
 * Build:
 *   python gen_esp_test.py
 *   cl /nologo /W3 /EHsc test_esp_parsers.cpp
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define CRLF "\r\n"

/* ---------------------------- ESP-IDF stubs -------------------------------- */
typedef void *SemaphoreHandle_t;
typedef void *TaskHandle_t;
typedef void *esp_mqtt_client_handle_t;
typedef int   nvs_handle_t;
#define portMAX_DELAY 0xFFFFFFFFu
#define NVS_READONLY  0
#define NVS_READWRITE 1
#define ESP_OK        0

static int64_t g_fake_us = 0;
static int64_t esp_timer_get_time(void) { return g_fake_us; }

static void xSemaphoreTake(SemaphoreHandle_t, unsigned) {}
static void xSemaphoreGive(SemaphoreHandle_t) {}
static int  g_notifies = 0;
static void xTaskNotifyGive(TaskHandle_t) { g_notifies++; }
static int  nvs_open(const char *, int, nvs_handle_t *) { return 1; }
static int  nvs_get_u32(nvs_handle_t, const char *, uint32_t *) { return 1; }
static int  nvs_set_u32(nvs_handle_t, const char *, uint32_t) { return 0; }
static int  nvs_commit(nvs_handle_t) { return 0; }
static void nvs_close(nvs_handle_t) {}
static void ws_broadcast(const char *, size_t) {}

/* Recorded MQTT publishes. */
struct Pub { char topic[128]; char payload[512]; int qos; int retain; };
#define MAX_PUB 64
static Pub g_pub[MAX_PUB];
static int g_nPub = 0;

static int esp_mqtt_client_publish(esp_mqtt_client_handle_t, const char *topic,
                                   const char *data, int len, int qos, int retain) {
    (void)len;
    if (g_nPub >= MAX_PUB) { printf("FAIL: publish overflow\n"); exit(1); }
    snprintf(g_pub[g_nPub].topic,   sizeof(g_pub[0].topic),   "%s", topic);
    snprintf(g_pub[g_nPub].payload, sizeof(g_pub[0].payload), "%s", data ? data : "");
    g_pub[g_nPub].qos = qos;
    g_pub[g_nPub].retain = retain;
    g_nPub++;
    return 0;
}

/* Topic names must match main.cpp's. */
#define MQTT_BASE   "msp430gate"
#define BEAM_T      MQTT_BASE "/beam"
#define BEAM_ATTR_T MQTT_BASE "/beam/attr"
#define BEAMACT_T   MQTT_BASE "/beamact"
#define BEAMDIAG_T  MQTT_BASE "/diag/beam_edges"
#define BEAM_MIN_PUB_GAP_MS 100

/* ------------------------- the code under test ------------------------------ */
#include "esp_parsers.inc"

/* --------------------------------- harness ---------------------------------- */
static int g_fail = 0;

static void check(int cond, const char *what) {
    printf("    %-64s %s\n", what, cond ? "PASS" : "**FAIL**");
    if (!cond) g_fail++;
}
static void checkStr(const char *got, const char *want, const char *what) {
    int ok = (got && want && strcmp(got, want) == 0);
    printf("    %-64s %s\n", what, ok ? "PASS" : "**FAIL**");
    if (!ok) { printf("        got  \"%s\"\n        want \"%s\"\n", got ? got : "(null)", want); g_fail++; }
}
static void checkInt(long got, long want, const char *what) {
    int ok = (got == want);
    printf("    %-64s %s\n", what, ok ? "PASS" : "**FAIL**");
    if (!ok) { printf("        got %ld, want %ld\n", got, want); g_fail++; }
}
static void resetPubs(void) { g_nPub = 0; g_notifies = 0; }
static const Pub *findPub(const char *topic) {
    for (int i = 0; i < g_nPub; i++) if (!strcmp(g_pub[i].topic, topic)) return &g_pub[i];
    return NULL;
}
static void dumpPubs(void) {
    for (int i = 0; i < g_nPub; i++)
        printf("      -> %-28s retain=%d  %s\n", g_pub[i].topic, g_pub[i].retain, g_pub[i].payload);
}

int main(void) {
    s_mqtt      = (esp_mqtt_client_handle_t)1;   /* pretend the broker is up */
    s_mqtt_task = (TaskHandle_t)1;
    printf("ESP32 bridge parser tests\n");

    /* ---- EVT STATE: a remote-started open ------------------------------- */
    printf("\n[E1] EVT STATE - remote-started OPENING\n");
    resetPubs();
    gate_rx_message("[1758400000] EVT STATE st=3 lo=0 lc=0 bm=0 nf=0 slp=0 sc=0 "
                    "dir=OPEN ma=1840 mv=12960 lk=0 dl=0 src=remote t=987654\r\n;\r\n");
    checkInt(g_gate_status.state, 3, "state = 3 (ST_OPENING)");
    checkInt(g_gate_status.open_limit, 0, "open_limit = 0");
    checkInt(g_gate_status.closed_limit, 0, "closed_limit = 0");
    checkInt(g_gate_status.beam_broken, 0, "beam_broken = 0");
    checkInt(g_gate_status.nfault, 0, "nfault = 0");
    checkInt(g_gate_status.driver_asleep, 0, "driver_asleep = 0");
    checkInt(g_gate_status.stall_count, 0, "stall_count = 0");
    checkInt(g_gate_status.current_ma, 1840, "current_ma = 1840");
    checkInt(g_gate_status.vm_mv, 12960, "vm_mv = 12960");
    checkStr(g_gate_status.last_dir, "OPEN", "last_dir = OPEN");
    checkStr(g_gate_status.cmd_src, "remote", "cmd_src = remote");
    checkStr(g_gate_status.lockout, "", "lockout empty");
    check(g_notifies > 0, "publisher task was notified");
    checkStr(gate_state_name(g_gate_status.state), "ST_OPENING", "state name");
    checkStr(gate_cover_state(g_gate_status.state), "opening", "HA cover state");
    check(gate_is_moving(g_gate_status.state), "reported as moving");

    /* ---- EVT STATE: lockout + dual limit -------------------------------- */
    printf("\n[E2] EVT STATE - lockout reasons\n");
    resetPubs();
    gate_rx_message("EVT STATE st=12 lo=1 lc=1 bm=0 nf=0 slp=1 sc=2 dir=CLOSE "
                    "ma=0 mv=12800 lk=1 dl=1 src=fault t=1000\r\n;\r\n");
    checkStr(g_gate_status.lockout, "both OPEN and CLOSED limit switches read active at once",
             "LOCKOUT_DUAL_LIMIT text");
    checkInt(g_gate_status.stall_count, 2, "stall_count = 2");
    checkStr(g_gate_status.cmd_src, "fault", "cmd_src = fault");

    resetPubs();
    gate_rx_message("EVT STATE st=2 lo=0 lc=1 bm=0 nf=0 slp=1 sc=0 dir=CLOSE "
                    "ma=0 mv=12800 lk=0 dl=1 src=ha t=2000\r\n;\r\n");
    checkStr(g_gate_status.lockout, "both limit switches active - will lock out",
             "early dual-limit warning when not yet locked out");

    resetPubs();
    gate_rx_message("EVT STATE st=12 lo=0 lc=0 bm=0 nf=0 slp=1 sc=5 dir=CLOSE "
                    "ma=0 mv=12800 lk=3 dl=0 src=fault t=3000\r\n;\r\n");
    checkStr(g_gate_status.lockout, "too many consecutive stalls - power-cycle to clear",
             "LOCKOUT_REPEAT_STALL text");

    /* ---- EVT BEAM edges -------------------------------------------------- */
    printf("\n[E3] EVT BEAM - blocked then clear\n");
    resetPubs();
    g_fake_us = 1000000;
    gate_rx_message("[1758400001] EVT BEAM b=1 t=123456 dur=0 n10=1\r\n;\r\n");
    {
        const Pub *st = findPub(BEAM_T);
        const Pub *at = findPub(BEAM_ATTR_T);
        const Pub *dg = findPub(BEAMDIAG_T);
        check(st != NULL, "binary-sensor state published");
        checkStr(st ? st->payload : NULL, "ON", "payload ON");
        checkInt(st ? st->retain : -1, 1, "state topic retained");
        check(at != NULL, "attributes published");
        check(at && strstr(at->payload, "\"breaks_last_10s\":1"), "breaks_last_10s attribute");
        check(at && strstr(at->payload, "\"last_edge_uptime_ms\":123456"), "last_edge_uptime_ms attribute");
        check(at && strstr(at->payload, "\"chatter\":false"), "chatter false");
        check(dg != NULL, "raw diagnostic edge published");
        check(dg && strstr(dg->payload, "\"edge\":\"blocked\""), "diag edge = blocked");
        checkInt(dg ? dg->retain : -1, 0, "diag topic NOT retained");
        checkInt(g_gate_status.beam_broken, 1, "legacy beam sensor follows the edge");
        dumpPubs();
    }

    resetPubs();
    g_fake_us = 2000000;
    gate_rx_message("EVT BEAM b=0 t=123656 dur=200 n10=1\r\n;\r\n");
    {
        const Pub *st = findPub(BEAM_T);
        const Pub *at = findPub(BEAM_ATTR_T);
        checkStr(st ? st->payload : NULL, "OFF", "payload OFF");
        check(at && strstr(at->payload, "\"last_break_ms\":200"), "last_break_ms = 200");
        checkInt(g_gate_status.beam_broken, 0, "legacy beam sensor cleared");
        dumpPubs();
    }

    /* ---- chatter rate limit --------------------------------------------- */
    printf("\n[E4] EVT BEAM chat=1 - state publishes rate limited, diag never is\n");
    resetPubs();
    g_fake_us = 3000000;
    gate_rx_message("EVT BEAM b=1 t=200000 dur=0 n10=12 chat=1\r\n;\r\n");
    check(findPub(BEAM_T) != NULL, "first chattering edge still publishes");
    check(findPub(BEAM_ATTR_T) && strstr(findPub(BEAM_ATTR_T)->payload, "\"chatter\":true"),
          "chatter attribute true");
    resetPubs();
    g_fake_us = 3000000 + 20000;          /* +20 ms, inside the 100 ms guard */
    gate_rx_message("EVT BEAM b=0 t=200020 dur=20 n10=13 chat=1\r\n;\r\n");
    check(findPub(BEAM_T) == NULL, "second edge inside 100 ms is NOT published");
    check(findPub(BEAMDIAG_T) != NULL, "...but the raw diagnostic edge still is");
    resetPubs();
    g_fake_us = 3000000 + 150000;         /* +150 ms, guard expired */
    gate_rx_message("EVT BEAM b=1 t=200150 dur=0 n10=14 chat=1\r\n;\r\n");
    check(findPub(BEAM_T) != NULL, "publishing resumes after the guard window");

    /* ---- EVT BEAMACT ------------------------------------------------------ */
    printf("\n[E5] EVT BEAMACT - vehicle_pass payload\n");
    resetPubs();
    gate_rx_message("[1758400002] EVT BEAMACT ev=vehicle_pass pat=axle_pair n=2 tot=1453 "
                    "gs=1 t=123456789 brk=142,131 gap=1180\r\n;\r\n");
    {
        const Pub *p = findPub(BEAMACT_T);
        check(p != NULL, "event entity payload published");
        checkInt(p ? p->retain : -1, 0, "event topic NOT retained (no replay on HA restart)");
        check(p && strstr(p->payload, "\"event_type\":\"vehicle_pass\""), "event_type");
        check(p && strstr(p->payload, "\"pattern\":\"axle_pair\""), "pattern");
        check(p && strstr(p->payload, "\"breaks\":2"), "breaks");
        check(p && strstr(p->payload, "\"break_ms\":[142,131]"), "break_ms array");
        check(p && strstr(p->payload, "\"gap_ms\":[1180]"), "gap_ms array");
        check(p && strstr(p->payload, "\"total_ms\":1453"), "total_ms");
        check(p && strstr(p->payload, "\"gate_state\":\"ST_IDLE_OPEN\""), "gate_state resolved to a name");
        check(p && strstr(p->payload, "\"uptime_ms\":123456789"), "uptime_ms survives >2^31");
        dumpPubs();
    }

    printf("\n[E6] EVT BEAMACT - empty lists become empty JSON arrays\n");
    resetPubs();
    gate_rx_message("EVT BEAMACT ev=long_block pat=none n=1 tot=6000 gs=4 t=50 brk=- gap=-\r\n;\r\n");
    {
        const Pub *p = findPub(BEAMACT_T);
        check(p && strstr(p->payload, "\"break_ms\":[]"), "break_ms = []");
        check(p && strstr(p->payload, "\"gap_ms\":[]"), "gap_ms = []");
        check(p && strstr(p->payload, "\"event_type\":\"long_block\""), "event_type long_block");
        dumpPubs();
    }

    /* ---- the full STATUS block still parses ------------------------------ */
    printf("\n[E7] full STATUS block (heartbeat / status request) still parses\n");
    // Leave the beam reported CLEAR first, so the block below is a real change.
    g_fake_us = 10000000;
    gate_rx_message("EVT BEAM b=0 t=300000 dur=50 n10=0" CRLF ";" CRLF);
    resetPubs();
    gate_rx_message(
        "[1758400003] Gate: OPEN (at limit)\r\n"
        "  OPEN   limit (Arm-Retracted, P7.5) = 1   <- 1 here means gate is at the OPEN end\r\n"
        "  CLOSED limit (Arm-Extended,  P7.6) = 0   <- 1 here means gate is at the CLOSED end\r\n"
        "  (limit inputs: internal pull-up, read as active-LOW (pin LOW = at the limit))\r\n"
        "  photo beam broken = 1   nFAULT = 0\r\n"
        "State=1  lastDir=OPEN\r\n"
        "Current: 0 mA (0 counts)  VM: 12930 mV\r\n"
        "Stall count: 0  Driver asleep: 1\r\n"
        "Uptime: 987654321 ms  src=wired_button  beam10s=3  lastbreak=142 ms\r\n"
        ";\r\n");
    checkInt(g_gate_status.state, 1, "state = 1 (ST_IDLE_OPEN)");
    checkInt(g_gate_status.open_limit, 1, "open_limit = 1");
    checkInt(g_gate_status.closed_limit, 0, "closed_limit = 0");
    checkInt(g_gate_status.beam_broken, 1, "beam_broken = 1");
    checkInt(g_gate_status.vm_mv, 12930, "vm_mv = 12930");
    checkInt(g_gate_status.driver_asleep, 1, "driver_asleep = 1");
    checkStr(g_gate_status.cmd_src, "wired_button", "cmd_src parsed from the new Uptime line");
    check(findPub(BEAM_T) != NULL, "beam binary sensor resynced from the full status");
    checkStr(findPub(BEAM_T) ? findPub(BEAM_T)->payload : NULL, "ON", "...as ON");
    dumpPubs();

    // ...and a status that AGREES with the current beam level must not republish.
    resetPubs();
    gate_rx_message(
        "State=1  lastDir=OPEN" CRLF
        "  photo beam broken = 1   nFAULT = 0" CRLF
        "Current: 0 mA (0 counts)  VM: 12930 mV" CRLF
        ";" CRLF);
    check(findPub(BEAM_T) == NULL, "no redundant beam publish when the level is unchanged");

    /* ---- regression: existing async lines are untouched ------------------ */
    printf("\n[E8] existing non-EVT messages still take their old paths\n");
    resetPubs();
    gate_rx_message("[1758400004] Gate: reached OPEN limit - travel time 10410 ms, avg 1160 mA, max 3200 mA\r\n"
                    "Peak ramp-up current: 4100 mA\r\n;\r\n");
    checkStr(s_lastmove.dir, "OPEN", "arrival message -> lastmove.dir");
    checkInt(s_lastmove.travel_ms, 10410, "lastmove.travel_ms");
    checkInt(s_lastmove.avg_ma, 1160, "lastmove.avg_ma");
    checkInt(s_lastmove.max_ma, 3200, "lastmove.max_ma");
    checkInt(s_lastmove.peak_ma, 4100, "lastmove.peak_ma");
    check(s_lastmove_dirty, "lastmove marked dirty for publishing");

    resetPubs();
    gate_rx_message("VERSION GateController_DRV8262 Sep 28 2026 12:00:00\r\n;\r\n");
    checkStr(s_msp_version, "GateController_DRV8262 Sep 28 2026 12:00:00", "VERSION line captured");
    check(s_disc_dirty, "version change forces a discovery re-publish");

    /* ---- a MOVE_START line must not be mistaken for anything else -------- */
    printf("\n[E9] EVT MOVE_START only nudges the publisher\n");
    resetPubs();
    int before = g_gate_status.state;
    gate_rx_message("EVT MOVE_START dir=CLOSE src=ha st=4 t=555\r\n;\r\n");
    checkInt(g_gate_status.state, before, "state untouched (the EVT STATE line carries it)");
    check(g_notifies > 0, "publisher notified");
    check(g_nPub == 0, "nothing published from MOVE_START alone");

    printf("\n%s (%d failure(s))\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
