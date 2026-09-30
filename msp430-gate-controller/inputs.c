/*******************************************************************************
 * inputs.c - see inputs.h. Debounce is timed in real milliseconds (via
 * board_GetTickMs()) rather than a fixed sample count, so it stays correct
 * whether the tick is currently 1 ms (active) or 50 ms (idle).
 ******************************************************************************/
#include <stddef.h>
#include "inputs.h"
#include "board.h"
#include "console.h"
#include "gate_config.h"

typedef struct {
    bool     debounced;
    uint16_t accumMs;
} dbnc_t;

static dbnc_t s_state[IN_COUNT];
static bool   s_rising[IN_COUNT];
static bool   s_falling[IN_COUNT];

#define CMD_FIFO_LEN 4
static cmd_t     s_cmdFifo[CMD_FIFO_LEN];
static cmd_src_t s_srcFifo[CMD_FIFO_LEN];
static uint8_t   s_cmdHead = 0, s_cmdTail = 0;
static cmd_src_t s_lastSrc = CMDSRC_NONE;   /* origin of the last popped command */

static bool rawActive(input_id_t id)
{
    bool raw = pin_RawRead(id);

    /* Photo-beam "broken" polarity - PHOTOBEAM_ACTIVE_HIGH in gate_config.h
     * (the P8.6 internal pull direction is a separate knob, PHOTOBEAM_PULL_UP,
     * handled in board.c gpio_Init). This flag only picks which raw level
     * counts as "beam broken". */
    if (id == IN_PHOTOBEAM) {
#if PHOTOBEAM_ACTIVE_HIGH
        return raw != false;   /* pin HIGH = beam broken */
#else
        return raw == false;   /* pin LOW  = beam broken */
#endif
    }

    /* Travel-limit logical polarity is a build option (LIMIT_SWITCH_ACTIVE_HIGH
     * in gate_config.h), independent of TEST_MODE. The pin itself is always
     * internal-pull-UP (opto open-drain output - see board.c gpio_Init); this
     * flag only picks which level counts as "at the limit". */
    if (id == IN_LIMIT_OPEN || id == IN_LIMIT_CLOSED) {
#if LIMIT_SWITCH_ACTIVE_HIGH
        return raw != false;   /* pin HIGH  = at the limit (opto not conducting) */
#else
        return raw == false;   /* pin LOW   = at the limit (opto conducting)     */
#endif
    }

    /* REMOTE toggle (P7.4) logical polarity - REMOTE_ACTIVE_HIGH in
     * gate_config.h. P7.4 is always internal pull-UP (open-drain opto output);
     * this flag only picks which raw level counts as "pressed". */
    if (id == IN_REMOTE) {
#if REMOTE_ACTIVE_HIGH
        return raw != false;   /* pin HIGH = pressed */
#else
        return raw == false;   /* pin LOW  = pressed */
#endif
    }

    return raw == false;   /* buttons, nFAULT, nHOME: active-LOW */
}

static void pushCmd(cmd_t c, cmd_src_t src)
{
    uint8_t next = (uint8_t)((s_cmdHead + 1u) % CMD_FIFO_LEN);
    if (next != s_cmdTail) {
        s_cmdFifo[s_cmdHead] = c;
        s_srcFifo[s_cmdHead] = src;
        s_cmdHead = next;
    }
}

cmd_t cmd_Get(void)
{
    cmd_t c;
    if (s_cmdTail == s_cmdHead) { return CMD_NONE; }
    c = s_cmdFifo[s_cmdTail];
    s_lastSrc = s_srcFifo[s_cmdTail];
    s_cmdTail = (uint8_t)((s_cmdTail + 1u) % CMD_FIFO_LEN);
    return c;
}

cmd_src_t cmd_LastSource(void)
{
    return s_lastSrc;
}

/* The console/bridge link is the only caller of the old one-argument form, so
 * it keeps meaning "arrived over the ESP32 link" - Home Assistant, MQTT and the
 * web UI's Gate Controls all reach the firmware as the same serial command. */
void inputs_InjectCommand(cmd_t cmd)
{
    pushCmd(cmd, CMDSRC_HA);
}

void inputs_InjectCommandSrc(cmd_t cmd, cmd_src_t src)
{
    pushCmd(cmd, src);
}

void inputs_Init(void)
{
    uint8_t i;
    for (i = 0; i < IN_COUNT; i++) {
        s_state[i].debounced = rawActive((input_id_t)i);
        s_state[i].accumMs = 0;
        s_rising[i] = false;
        s_falling[i] = false;
    }
    s_cmdHead = 0;
    s_cmdTail = 0;
    s_lastSrc = CMDSRC_NONE;
}

void inputs_Poll(void)
{
    uint16_t dt = board_GetTickMs();
    uint8_t i;

    for (i = 0; i < IN_COUNT; i++) {
        bool raw = rawActive((input_id_t)i);
        /* The photo beam gets its own, shorter debounce so a real tyre break
         * (down to BEAM_AXLE_BREAK_MIN_MS) survives - see PHOTOBEAM_DEBOUNCE_MS
         * in gate_config.h for the safety note that goes with it. */
        uint16_t debounceMs = (i == (uint8_t)IN_NFAULT)    ? NFAULT_DEBOUNCE_MS
                            : (i == (uint8_t)IN_PHOTOBEAM) ? PHOTOBEAM_DEBOUNCE_MS
                                                           : INPUT_DEBOUNCE_MS;

        if (raw == s_state[i].debounced) {
            s_state[i].accumMs = 0;
        } else {
            s_state[i].accumMs = (uint16_t)(s_state[i].accumMs + dt);
            if (s_state[i].accumMs >= debounceMs) {
                s_state[i].debounced = raw;
                s_state[i].accumMs = 0;
                if (raw) { s_rising[i] = true; } else { s_falling[i] = true; }
            }
        }
    }

    /* input_RisingEdge() consumes (clears) the flag on read - a raw s_rising[]
     * read here would never clear it, so the very first press of any of these
     * would re-push its command on every tick forever. Both sides are read
     * unconditionally (not via ||) so a same-tick edge on the other one is
     * never left uncleared by short-circuit evaluation. */
    {
        bool toggleEdge = input_RisingEdge(IN_BTN_TOGGLE);
        bool remoteEdge  = input_RisingEdge(IN_REMOTE);
        /* The origin now rides along with the command (cmd_src_t) and is what
         * EVT MOVE_START src=... reports, so these per-press lines are just
         * development chatter and sit behind GATE_DEBUG. */
#if GATE_DEBUG
        if (toggleEdge) { dbg_Print("INPUT: TOGGLE button pressed\r\n"); }
        if (remoteEdge) { dbg_Print("INPUT: REMOTE toggle asserted\r\n"); }
#endif
        if (remoteEdge)      { pushCmd(CMD_TOGGLE, CMDSRC_REMOTE); }
        else if (toggleEdge) { pushCmd(CMD_TOGGLE, CMDSRC_WIRED_BUTTON); }
    }
    if (input_RisingEdge(IN_BTN_OPEN))  { pushCmd(CMD_OPEN,  CMDSRC_WIRED_BUTTON); }
    if (input_RisingEdge(IN_BTN_CLOSE)) { pushCmd(CMD_CLOSE, CMDSRC_WIRED_BUTTON); }

    /* D8 (beam OK, P1.1) and D9 (closed-limit, P1.2) retired: those pins are now
     * the ROM-BSL UART lines shared with the ESP32 programmer - see board.c. */
    led_Set(LED_LIMIT_OPEN,    s_state[IN_LIMIT_OPEN].debounced);
    led_Set(LED_BUTTON,        input_AnyButtonPressed());
}

bool input_Level(input_id_t id)
{
    return s_state[id].debounced;
}

bool input_RisingEdge(input_id_t id)
{
    bool e = s_rising[id];
    s_rising[id] = false;
    return e;
}

bool input_FallingEdge(input_id_t id)
{
    bool e = s_falling[id];
    s_falling[id] = false;
    return e;
}

bool input_AnyButtonPressed(void)
{
    return s_state[IN_BTN_TOGGLE].debounced || s_state[IN_BTN_OPEN].debounced ||
           s_state[IN_BTN_CLOSE].debounced  || s_state[IN_REMOTE].debounced;
}
