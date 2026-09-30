/*******************************************************************************
 * console.c - see console.h.
 ******************************************************************************/
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "console.h"
#include "uart.h"
#include "timesync.h"
#include "nvconfig.h"
#include "gate_config.h"
#include "gate_fsm.h"
#include "inputs.h"
#include "drv8262.h"

static char    s_line[CONSOLE_RX_LINE_MAX];
static uint8_t s_lineLen = 0;
static bool    s_overflow = false;

void console_Init(void)
{
    s_lineLen = 0;
    s_overflow = false;
}

/* ------------------------------------------------------- tiny formatter --- */
static void appendChar(char **p, const char *end, char c)
{
    if (*p < end) { **p = c; (*p)++; }
}

static void appendStr(char **p, const char *end, const char *s)
{
    while (*s && *p < end) { appendChar(p, end, *s++); }
}

static void appendUint32(char **p, const char *end, uint32_t v)
{
    char tmp[10];
    int8_t n = 0;
    if (v == 0) { appendChar(p, end, '0'); return; }
    while (v > 0 && n < 10) { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n > 0) { appendChar(p, end, tmp[--n]); }
}

static void appendInt32(char **p, const char *end, int32_t v)
{
    uint32_t uv;
    if (v < 0) { appendChar(p, end, '-'); uv = (uint32_t)(-v); } else { uv = (uint32_t)v; }
    appendUint32(p, end, uv);
}

/* Prepends "[<epoch>] " once the ESP32 has sent a TIME: sync (timesync.c);
 * silent no-op before the first sync, so early boot messages are unaffected. */
static void printTimestampPrefix(void)
{
    char buf[14];
    char *p = buf;
    const char *end = buf + sizeof(buf) - 1u;
    const char *c;

    if (!timesync_IsSynced()) {
        return;
    }
    appendChar(&p, end, '[');
    appendUint32(&p, end, timesync_Now());
    appendStr(&p, end, "] ");
    *p = '\0';
    for (c = buf; *c; c++) {
        (void)uart_PutByte((uint8_t)*c);
    }
}

/* ---- serial framing --------------------------------------------------------
 * Every logical message the firmware emits is followed by an end-of-message
 * marker: a line containing only ";". Single-line messages (dbg_Print /
 * dbg_Printf) emit it automatically. Multi-line messages (STATUS, LIST,
 * LIST_DETAILS, the capture dump) suppress the per-call marker with
 * dbg_BeginBlock()/dbg_EndBlock() and emit exactly one ";" line at the end.
 * NOTE for the serial parser: NO message body ever contains a ";".
 * See SERIAL_PROTOCOL.md. */
static uint8_t s_blockDepth = 0;

/* Raw byte push - no [epoch] prefix, no end-of-message marker. The primitive
 * used to build machine-parsed output (the capture CSV) a fragment at a time. */
void dbg_PrintRaw(const char *s)
{
    while (*s) {
        (void)uart_PutByte((uint8_t)*s++);
    }
}

void dbg_BeginBlock(void)
{
    s_blockDepth++;
}

void dbg_EndBlock(void)
{
    if (s_blockDepth > 0u) { s_blockDepth--; }
    if (s_blockDepth == 0u) { dbg_PrintRaw(";\r\n"); }
}

void dbg_Print(const char *s)
{
    printTimestampPrefix();
    dbg_PrintRaw(s);
    if (s_blockDepth == 0u) { dbg_PrintRaw(";\r\n"); }   /* end-of-message marker */
}

/* ONE static scratch buffer (console.h rule 1) - never on the stack. Shared by
 * dbg_Printf / dbg_PrintfRaw; neither is reentrant and neither runs in an ISR. */
static char s_fmtBuf[DBG_PRINTF_BUF_LEN];

static void dbg_vformat(const char *fmt, va_list ap)
{
    char *p = s_fmtBuf;
    const char *end = s_fmtBuf + sizeof(s_fmtBuf) - 1u;

    while (*fmt && p < end) {
        if (*fmt != '%') {
            appendChar(&p, end, *fmt++);
            continue;
        }
        fmt++;
        if (*fmt == 'l') {
            fmt++;
            if (*fmt == 'd')      { appendInt32(&p, end, (int32_t)va_arg(ap, long)); fmt++; }
            else if (*fmt == 'u') { appendUint32(&p, end, (uint32_t)va_arg(ap, unsigned long)); fmt++; }
        } else if (*fmt == 'd') { appendInt32(&p, end, (int32_t)va_arg(ap, int)); fmt++; }
        else if (*fmt == 'u')   { appendUint32(&p, end, (uint32_t)va_arg(ap, unsigned int)); fmt++; }
        else if (*fmt == 's')   { appendStr(&p, end, va_arg(ap, const char *)); fmt++; }
        else if (*fmt == '%')   { appendChar(&p, end, '%'); fmt++; }
        else                    { appendChar(&p, end, '%'); }
    }
    *p = '\0';
}

void dbg_Printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    dbg_vformat(fmt, ap);
    va_end(ap);
    dbg_Print(s_fmtBuf);
}

void dbg_PrintfRaw(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    dbg_vformat(fmt, ap);
    va_end(ap);
    dbg_PrintRaw(s_fmtBuf);
}

/* ------------------------------------------------------ command dispatch -- */
static bool parseInt(const char *s, int32_t *out)
{
    char *endp;
    long v;
    if (!s || *s == '\0') { return false; }
    v = strtol(s, &endp, 10);
    if (endp == s || *endp != '\0') { return false; }
    *out = (int32_t)v;
    return true;
}

static void list_Cb(const char *name, uint16_t value, uint16_t deflt, const char *units)
{
    dbg_Printf("  %s = %u %s (default %u)\r\n", name, (unsigned int)value, units, (unsigned int)deflt);
}

static void detail_Cb(const char *name, const char *units, const char *desc)
{
    if (units && units[0] != '\0') {
        dbg_Printf("%s (%s)\r\n    %s\r\n", name, units, desc);
    } else {
        dbg_Printf("%s\r\n    %s\r\n", name, desc);
    }
}

/* ---- Software BSL entry (SLAU319 sec 2.2.2) -------------------------------
 * The hardware TEST/RST invoke sequence never latched on this board's F5338,
 * but the on-chip BSL code at 0x1000 is intact (verified over JTAG). Jump to it
 * directly: disable interrupts, stop the watchdog, hand TA0 back (the F5xx
 * TA-UART BSL bit-bangs its UART on TA0 / P1.1 / P1.2), then call 0x1000.
 *
 * After this call the application is gone. The BSL owns P1.1/P1.2 at 9600 and
 * waits for the host (the ESP32, which is already wired to those pins). If the
 * BSL image is bad it will fault and the forced WDT password violation below
 * resets the board back into the app. SYSBSLC stays at its 0x8003 POR value:
 * SYSBSLOFF=0 so the BSL is mapped and callable; SYSBSLPE only gates *writing*
 * the BSL region, not calling into it. */
static void console_EnterBsl(void)
{
    dbg_Print("BSL: jumping to bootloader at 0x1000 - app UART goes dark, "
              "BSL is now on P1.1/P1.2 @ 9600 8N1\r\n");

    /* Flush that line out of the ring AND the UCA1 shift register before we kill
     * interrupts, or it is lost mid-byte. */
    while (!uart_TxIdle()) { }
    while (UCA1STAT & UCBUSY) { }
    __delay_cycles(200000);              /* ~10 ms guard @ 20 MHz */

    __disable_interrupt();
    WDTCTL = WDTPW | WDTHOLD;            /* stop the 250 ms ARST watchdog */
    TA0CTL = 0;                          /* BSL TA-UART drives TA0 / P1.1 / P1.2 */

    ((void (*)(void))0x1000)();          /* enter BSL */

    /* Only reached if the BSL rejects entry / returns - force a PUC. */
    WDTCTL = 0;
    for (;;) { }
}

/* ==== Wireless firmware self-update over the app UART ===================== *
 * The BSL on this F5338 is dead (corrupt and unerasable), so a new image is
 * received over the normal UCA1 console link and burned into main flash by the
 * routine below. Everything from here down runs from RAM (.TI.ramfunc, placed
 * by lnk_msp430f5338.cmd, copied to RAM by the BINIT table at boot) so it can
 * erase and rewrite ALL of main flash (0x8000-0xFFFF) - including the code that
 * called it - and never fetches an instruction from flash while flash is being
 * programmed. It ONLY ever writes 0x8000-0xFFFF: the BSL area (0x1000-0x17FF)
 * and the flash-backed config in INFOD are left untouched, so tuning survives.
 *
 * Wire protocol (ESP32 -> MSP430), little-endian, after the "FWUPDATE" command:
 *   MSP430 emits one 0xCC when the RAM updater is ready.
 *   data frame : 0xA5, addrLo, addrHi, len(1..128), data[len], crcLo, crcHi
 *                reply 0x06 = written+verified, 0x15 = bad CRC / verify fail /
 *                out of range / non-ascending  -> ESP32 resends the frame
 *   commit     : 0x5B  -> reply 0x06, then software BOR into the new image
 *   Frames are ascending and never span a 512-byte segment; a segment is erased
 *   the first time a write lands in it. CRC is CRC-16/CCITT-FALSE (poly 0x1021,
 *   init 0xFFFF, no reflection) over addrLo..data. */

#define FWU_READY    0xCCu
#define FWU_SOF      0xA5u
#define FWU_COMMIT   0x5Bu
#define FWU_ACK      0x06u
#define FWU_NAK      0x15u
#define FWU_MAX_PAY  128u
#define FWU_SEG_MASK 0xFE00u             /* 512-byte main-flash segment mask */
#define FWU_LO       0x8000u             /* lowest address the updater will touch */

/* .bss (RAM) - kept off the small (160 B) stack */
static uint8_t fwu_frame[3u + FWU_MAX_PAY + 2u];

#pragma CODE_SECTION(fwu_die, ".TI.ramfunc")
static void fwu_die(uint8_t code)
{
    uint16_t g;
    /* best-effort single byte so the host learns which wait timed out, then BOR */
    for (g = 0u; (UCA1STAT & UCBUSY) && g != 0xFFFFu; g++) { }
    UCA1TXBUF = code;
    for (g = 0u; (UCA1STAT & UCBUSY) && g != 0xFFFFu; g++) { }
    PMMCTL0 = PMMPW | PMMSWBOR;          /* full software brown-out reset */
    for (;;) { }
}

#pragma CODE_SECTION(fwu_reset, ".TI.ramfunc")
static void fwu_reset(void)
{
    PMMCTL0 = PMMPW | PMMSWBOR;
    for (;;) { }
}

#pragma CODE_SECTION(fwu_getc, ".TI.ramfunc")
static uint8_t fwu_getc(void)
{
    uint16_t lo = 0u, hi = 0u;
    while (!(UCA1IFG & UCRXIFG)) {
        if (++lo == 0u) {               /* wraps ~ every 26 ms @ 20 MHz */
            if (++hi > 300u) {          /* host silent ~7 s -> give up */
                fwu_reset();            /* silent BOR: host is gone, no point announcing */
            }
        }
    }
    return UCA1RXBUF;                    /* read clears UCRXIFG */
}

#pragma CODE_SECTION(fwu_putc, ".TI.ramfunc")
static void fwu_putc(uint8_t b)
{
    uint16_t g;
    for (g = 0u; (UCA1STAT & UCBUSY); g++) { if (g == 0xFFFFu) { fwu_die(0xE2u); } }
    UCA1TXBUF = b;                       /* UART idle here -> TXBUF free; write clears UCTXIFG */
    for (g = 0u; !(UCA1IFG & UCTXIFG); g++) { if (g == 0xFFFFu) { fwu_die(0xE2u); } }
    for (g = 0u; (UCA1STAT & UCBUSY);  g++) { if (g == 0xFFFFu) { fwu_die(0xE2u); } }
}

#pragma CODE_SECTION(fwu_crc16, ".TI.ramfunc")
static uint16_t fwu_crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  j;
    for (i = 0u; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (j = 0u; j < 8u; j++) {
            if (crc & 0x8000u) { crc = (uint16_t)((crc << 1) ^ 0x1021u); }
            else               { crc = (uint16_t)(crc << 1); }
        }
    }
    return crc;
}

#pragma CODE_SECTION(fwu_wait_flash, ".TI.ramfunc")
static void fwu_wait_flash(void)
{
    /* A segment erase is ~23-32 ms (up to ~50 ms worst case) and holds BUSY the
     * whole time - so the timeout has to be generous. ~30 wraps * ~23 ms ≈ 0.7 s
     * before we conclude the flash controller is genuinely wedged. */
    uint16_t lo = 0u, hi = 0u;
    while (FCTL3 & BUSY) {
        if (++lo == 0u) {
            if (++hi > 30u) { fwu_die(0xE1u); }
        }
    }
}

#pragma CODE_SECTION(fwu_erase_seg, ".TI.ramfunc")
static void fwu_erase_seg(uint16_t seg)
{
    fwu_wait_flash();
    FCTL3 = FWKEY;                       /* clear LOCK (LOCKA left as-is) */
    FCTL1 = FWKEY | ERASE;               /* segment erase */
    *(volatile uint8_t *)seg = 0u;       /* dummy write starts the erase */
    fwu_wait_flash();
    FCTL1 = FWKEY;                       /* clear ERASE */
    FCTL3 = FWKEY | LOCK;
}

#pragma CODE_SECTION(fwupdate_Run, ".TI.ramfunc")
static void fwupdate_Run(void)
{
    uint16_t addr, i, want, got, seg;
    uint16_t curSeg = 0xFFFFu;           /* nothing erased yet */
    uint8_t  len, sof, ok;

    __disable_interrupt();
    UCA1IE = 0u;                         /* poll, don't interrupt */
    (void)UCA1RXBUF;                     /* drop a straggler */
    UCA1IFG = (uint8_t)UCTXIFG;          /* clear RX flag, force TX-ready */

    fwu_putc(0xC2u);                     /* DIAG: this RAM copy is executing real code */
    fwu_putc(FWU_READY);

    for (;;) {
        sof = fwu_getc();

        if (sof == FWU_COMMIT) {
            fwu_putc(FWU_ACK);
            fwu_reset();                 /* -> boots the freshly written image */
        }
        if (sof != FWU_SOF) {
            continue;                    /* noise / resync */
        }

        fwu_frame[0] = fwu_getc();       /* addr lo */
        fwu_frame[1] = fwu_getc();       /* addr hi */
        len          = fwu_getc();
        fwu_frame[2] = len;
        if ((len == 0u) || (len > FWU_MAX_PAY)) {
            fwu_putc(FWU_NAK);
            continue;
        }
        for (i = 0u; i < len; i++) {
            fwu_frame[3u + i] = fwu_getc();
        }
        got  = (uint16_t)fwu_getc();
        got |= (uint16_t)((uint16_t)fwu_getc() << 8);

        want = fwu_crc16(fwu_frame, (uint16_t)(3u + len));
        if (got != want) {
            fwu_putc(FWU_NAK);
            continue;
        }

        addr = (uint16_t)fwu_frame[0] | (uint16_t)((uint16_t)fwu_frame[1] << 8);

        /* stay inside main flash; reject a frame that would wrap past 0xFFFF */
        if ((addr < FWU_LO) ||
            ((uint16_t)(addr + (uint16_t)(len - 1u)) < addr)) {
            fwu_putc(FWU_NAK);
            continue;
        }

        seg = addr & FWU_SEG_MASK;
        if (seg != curSeg) {
            if ((curSeg != 0xFFFFu) && (seg < curSeg)) {   /* not ascending */
                fwu_putc(FWU_NAK);
                continue;
            }
            fwu_erase_seg(seg);
            curSeg = seg;
        }

        fwu_wait_flash();
        FCTL3 = FWKEY;
        FCTL1 = FWKEY | WRT;
        for (i = 0u; i < len; i++) {
            *(volatile uint8_t *)(addr + i) = fwu_frame[3u + i];
            fwu_wait_flash();
        }
        FCTL1 = FWKEY;
        FCTL3 = FWKEY | LOCK;

        ok = 1u;
        for (i = 0u; i < len; i++) {
            if (*(volatile uint8_t *)(addr + i) != fwu_frame[3u + i]) {
                ok = 0u;
                break;
            }
        }
        fwu_putc(ok ? FWU_ACK : FWU_NAK);
    }
}

const char *fw_VersionString(void)
{
    /* __DATE__/__TIME__ are this file's compile timestamp - close enough to
     * identify the image; the boot banner prints the same string. */
    return "GateController_DRV8262 " __DATE__ " " __TIME__ FW_BUILD_ID;
}

/* Multi-line so HELP works without a full printf. One dbg_Print call = one
 * message (no ";" inside - it is the message terminator, see the framing note
 * at the top of this file). */
static const char s_helpText[] =
    "Commands (case-insensitive, end each line with CR, LF or ';'):\r\n"
    "  STATUS           state, limits, sensed current, VM rail, stall count\r\n"
    "  STATUS_IF_CHANGED  same block, but only if a discrete field changed (else silent)\r\n"
    "    (STATUS is also the bridge status request - the firmware pushes EVT lines on its own)\r\n"
    "  LIST             every tunable param - current value, default, units\r\n"
    "  LIST_DETAILS     each param with a one-line explanation of what it does\r\n"
    "  GET_<NAME>       print one param value (names from LIST)\r\n"
    "  SET_<NAME> <n>   set a param in RAM (range-checked, not saved yet)\r\n"
    "  SAVE             persist current params to flash (survives power cycle)\r\n"
    "  DEFAULTS         restore factory params in RAM (follow with SAVE)\r\n"
    "  VERSION          print firmware build date/time and build id\r\n"
    "  OPEN             start an OPEN move (same as the OPEN button)\r\n"
    "  CLOSE            start a CLOSE move (same as the CLOSE button)\r\n"
    "  TOGGLE           start/reverse a move (same as the TOGGLE button / remote)\r\n"
    "  STOP             stop motion / abort the move in progress\r\n"
    "  BSL              jump to the bootloader (0x1000) for a wireless reflash\r\n"
    "  FWUPDATE         receive a new image over this UART and self-flash it\r\n"
    "  HELP or ?        this text\r\n"
    "Moves can also be started with the OPEN / CLOSE / TOGGLE buttons or the remote.\r\n";

static void console_Dispatch(char *line)
{
    char *sp = strchr(line, ' ');
    char *arg = 0;
    char *c;
    if (sp) {
        *sp = '\0';
        arg = sp + 1;
    }

    /* Command keywords (and SET_/GET_ param names, which are all upper-case in
     * nvconfig.c) are matched case-insensitively - upper-case the token in
     * place. The value in 'arg' is left exactly as typed. */
    for (c = line; *c != '\0'; c++) {
        if (*c >= 'a' && *c <= 'z') { *c = (char)(*c - 'a' + 'A'); }
    }

    if (strncmp(line, "SET_", 4) == 0) {
        int32_t val;
        const char *paramName = line + 4;
        nv_result_t result;
        if (!arg || !parseInt(arg, &val)) {
            dbg_Print("ERR BAD_VALUE\r\n");
            return;
        }
        result = nvconfig_SetByName(paramName, val);
        if (result == NV_OK) {
            /* g_cfg is only a RAM shadow - a handful of params also drive live
             * hardware state that must be re-applied immediately, or the change
             * has no effect until the next power cycle (and only then if it was
             * also SAVEd to flash). */
            if (strcmp(paramName, "VREF_CURRENT_REGULATION") == 0) {
                drv8262_SetCurrentLimitMa(g_cfg.vrefCurrentRegulationMa);
            } else if (strcmp(paramName, "RAMP_TIME") == 0) {
                drv8262_SetRampTimeMs(g_cfg.rampTimeMs);
            }
        }
        switch (result) {
            case NV_OK:               dbg_Print("OK\r\n"); break;
            case NV_ERR_UNKNOWN_NAME: dbg_Print("ERR UNKNOWN_PARAM\r\n"); break;
            case NV_ERR_RANGE:        dbg_Print("ERR OUT_OF_RANGE\r\n"); break;
            case NV_ERR_ORDERING:     dbg_Print("ERR ORDERING (thresholds must stay separated)\r\n"); break;
            default:                  dbg_Print("ERR\r\n"); break;
        }
    } else if (strncmp(line, "GET_", 4) == 0) {
        uint16_t val;
        if (nvconfig_GetByName(line + 4, &val)) {
            dbg_Printf("%u\r\n", (unsigned int)val);
        } else {
            dbg_Print("ERR UNKNOWN_PARAM\r\n");
        }
    } else if (strcmp(line, "LIST") == 0) {
        dbg_BeginBlock();
        nvconfig_ForEach(list_Cb);
        dbg_EndBlock();
    } else if (strcmp(line, "LIST_DETAILS") == 0) {
        dbg_BeginBlock();
        nvconfig_ForEachDetail(detail_Cb);
        dbg_EndBlock();
    } else if (strcmp(line, "SAVE") == 0) {
        nvconfig_Save();
        dbg_Print("Saved.\r\n");
    } else if (strcmp(line, "DEFAULTS") == 0) {
        nvconfig_ResetDefaults();
        dbg_Print("Reset to defaults in RAM - send SAVE to persist.\r\n");
    } else if (strcmp(line, "STATUS") == 0) {
        gate_fsm_PrintStatus();
    } else if (strcmp(line, "STATUS_IF_CHANGED") == 0) {
        /* Prints the same block as STATUS, but only if a discrete status field
         * changed since the last STATUS/STATUS_IF_CHANGED. No change -> no output
         * at all (not even ";"), so a fast poller doesn't spam the console. */
        (void)gate_fsm_PrintStatusIfChanged();
    } else if (strcmp(line, "VERSION") == 0) {
        dbg_Printf("VERSION %s\r\n", fw_VersionString());
    } else if (strcmp(line, "OPEN") == 0) {
        inputs_InjectCommand(CMD_OPEN);
        dbg_Print("OK\r\n");
    } else if (strcmp(line, "CLOSE") == 0) {
        inputs_InjectCommand(CMD_CLOSE);
        dbg_Print("OK\r\n");
    } else if (strcmp(line, "TOGGLE") == 0) {
        inputs_InjectCommand(CMD_TOGGLE);
        dbg_Print("OK\r\n");
    } else if (strcmp(line, "STOP") == 0) {
        inputs_InjectCommand(CMD_STOP);
        dbg_Print("OK\r\n");
    } else if (strcmp(line, "BSL") == 0) {
        console_EnterBsl();              /* does not return */
    } else if (strcmp(line, "FWUPDATE") == 0) {
        drv8262_Coast();                 /* de-energize the motor before the app stops */
        dbg_Print("FWUPDATE: RAM updater armed - send the image now, app halting.\r\n");
        while (!uart_TxIdle()) { }       /* let that line fully leave UCA1 */
        while (UCA1STAT & UCBUSY) { }
        __delay_cycles(200000);          /* ~10 ms for the ESP32 to switch to raw framing */
        __disable_interrupt();
        WDTCTL = WDTPW | WDTHOLD;
        {
            /* Belt-and-suspenders: copy the RAM routine ourselves (BINIT should
             * already have done this at boot, but confirm it so a call into
             * garbage RAM is impossible). Runs from flash, before any erase. */
            extern char __fwu_load, __fwu_run, __fwu_size;
            char *src = &__fwu_load, *dst = &__fwu_run;
            uint16_t n = (uint16_t)(uintptr_t)&__fwu_size, k;
            for (k = 0u; k < n; k++) { dst[k] = src[k]; }

            /* DIAG marker 0xC1: reached here with app-UART TX alive. Bounded. */
            uint16_t g;
            for (g = 0u; (UCA1STAT & UCBUSY) && g != 0xFFFFu; g++) { }
            UCA1TXBUF = 0xC1u;
            for (g = 0u; (UCA1STAT & UCBUSY) && g != 0xFFFFu; g++) { }
        }
        fwupdate_Run();                  /* never returns (BOR on commit / timeout) */
        for (;;) { }
    } else if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
        dbg_Print(s_helpText);
    } else {
        dbg_Print("ERR UNKNOWN_COMMAND (try HELP)\r\n");
    }
}

void console_Service(void)
{
    uint8_t b;
    while (uart_GetByte(&b)) {
        /* ';' is accepted as a line terminator too, so the ESP32 can frame its
         * outbound commands the same way the firmware frames replies. */
        if (b == '\r' || b == '\n' || b == ';') {
            if (s_overflow) {
                dbg_Print("ERR LINE_TOO_LONG\r\n");
            } else if (s_lineLen > 0) {
                s_line[s_lineLen] = '\0';
                if (!timesync_TryHandleLine(s_line)) {
                    console_Dispatch(s_line);
                }
            }
            s_lineLen = 0;
            s_overflow = false;
        } else if (s_lineLen < (CONSOLE_RX_LINE_MAX - 1u)) {
            s_line[s_lineLen++] = (char)b;
        } else {
            s_overflow = true;   /* cap hit - discard the rest until the terminator */
        }
    }
}
