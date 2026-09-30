"""Contract test across the two firmwares.

The MSP430 prints the EVT lines; the ESP32 parses them. Nothing in either build
would catch a key being renamed on one side only - the bridge would simply stop
seeing that field and Home Assistant would quietly show a stale or default
value. This test reads BOTH sources and checks that every key the bridge looks
for is actually emitted, and vice versa.
"""
import io
import re
import sys

import os
_HERE = os.path.dirname(os.path.abspath(__file__))
MSP = os.path.join(_HERE, "..", "msp430-gate-controller")
ESP = os.path.join(_HERE, "..", "esp32-wifi-bridge", "src", "main.cpp")

fsm = io.open(os.path.join(MSP, "gate_fsm.c"), encoding="utf-8").read()
beam = io.open(os.path.join(MSP, "beam.c"), encoding="utf-8").read()
esp = io.open(ESP, encoding="utf-8").read()

fails = []
def check(cond, what):
    print("    %-70s %s" % (what, "PASS" if cond else "**FAIL**"))
    if not cond:
        fails.append(what)


def fmt_of(src, marker):
    """The concatenated string literal of the dbg_Printf call containing `marker`."""
    i = src.index(marker)
    j = src.index(");", i)
    chunk = src[i - 200:j]
    lits = re.findall(r'"((?:[^"\\]|\\.)*)"', chunk[chunk.index(marker) - 60:])
    return "".join(lits)


print("MSP430 -> ESP32 wire-format contract")

# ---- EVT STATE ------------------------------------------------------------
print("\n[C1] EVT STATE")
state_fmt = fmt_of(fsm, "EVT STATE st=")
print("      firmware emits: " + state_fmt.replace("\\r\\n", ""))
# every " <key>=" the bridge's handle_evt_state() looks for
# Every key the bridge actually asks a parse helper for, in handle_evt_state().
KEYCALL = re.compile(r'(?:int_after|u32_after|token_after)\(\s*p\s*,\s*"\s?([A-Za-z0-9_]+)="')
h = esp[esp.index("static void handle_evt_state("):esp.index("// Called by uart_rx_task")]
for k in sorted(set(KEYCALL.findall(h))):
    check(k + "=" in state_fmt, "bridge key '%s=' is emitted by the firmware" % k)

# ---- EVT BEAM -------------------------------------------------------------
print("\n[C2] EVT BEAM")
beam_fmt = fmt_of(beam, "EVT BEAM b=")
print("      firmware emits: " + beam_fmt.replace("\\r\\n", ""))
hb = esp[esp.index("static void handle_beam_edge("):esp.index("// \"EVT BEAMACT")]
for k in sorted(set(KEYCALL.findall(hb))):
    check(k + "=" in beam_fmt, "bridge key '%s=' is emitted by the firmware" % k)
check("chat=1" in beam_fmt and 'strstr(p, "chat=1")' in hb,
      "chatter flag spelled 'chat=1' on both sides")

# ---- EVT BEAMACT ----------------------------------------------------------
print("\n[C3] EVT BEAMACT")
act_fmt = fmt_of(beam, "EVT BEAMACT ev=")
print("      firmware emits: " + act_fmt.replace("\\r\\n", ""))
ha = esp[esp.index("static void handle_beam_activity("):esp.index("// \"EVT STATE st=")]
for k in sorted(set(KEYCALL.findall(ha))):
    check(k + "=" in act_fmt, "bridge key '%s=' is emitted by the firmware" % k)

# HA event entity: the event_type values the firmware can emit must all be
# declared in the discovery config, or HA drops the event.
emitted = set(re.findall(r'emitActivity\("([a-z_]+)"', beam)) | \
          set(re.findall(r'ev = "([a-z_]+)"', beam))
_et = esp.index("event_types")
declared = set(re.findall(r'\\"([a-z_]+)\\"', esp[_et:_et + 140]))
print("      firmware event types: %s" % sorted(emitted))
print("      HA discovery declares: %s" % sorted(declared))
check(emitted <= declared, "every emitted event_type is declared in HA discovery")

# ---- EVT MOVE_START -------------------------------------------------------
print("\n[C4] EVT MOVE_START")
check("EVT MOVE_START dir=" in fsm, "firmware emits EVT MOVE_START")
check('strstr(msg, "EVT MOVE_START ")' in esp, "bridge recognises EVT MOVE_START")

# ---- command-source vocabulary -------------------------------------------
print("\n[C5] command sources")
srcs = set(re.findall(r'case CMDSRC_[A-Z_]+:\s+return "([a-z_]+)"', fsm))
print("      firmware sources: %s" % sorted(srcs))
required = {"ha", "remote", "wired_button", "auto_reverse", "obstruction_stop", "fault", "boot"}
check(required <= srcs, "every source named in the change request exists")
# they must all survive token_after(..., tok_word), i.e. [a-zA-Z_] only
check(all(re.fullmatch(r"[a-zA-Z_]+", x) for x in srcs),
      "all source strings are parseable by the bridge's tok_word()")

# ---- STATUS block additions ----------------------------------------------
print("\n[C6] STATUS block")
check('"Uptime: %lu ms  src=%s' in fsm, "STATUS carries the Uptime/src/beam line")
check('strstr(msg, "Uptime:")' in esp, "bridge uses 'Uptime:' to detect pushing firmware")
check('token_after(b, "src=", o->cmd_src' in esp, "bridge reads src= out of the STATUS block")

# ---- no ';' inside any event body ----------------------------------------
print("\n[C7] framing")
for name, f in (("EVT STATE", state_fmt), ("EVT BEAM", beam_fmt), ("EVT BEAMACT", act_fmt)):
    body = f.replace("\\r\\n", "")
    check(";" not in body, "%s body contains no ';' (it is the message terminator)" % name)

# ---- the beam line must not be mistaken for the activity line -------------
print("\n[C8] prefix disambiguation")
check(esp.index('strstr(msg, "EVT BEAMACT ")') < esp.index('strstr(msg, "EVT BEAM ")'),
      "BEAMACT is tested before BEAM")
check(act_fmt.startswith("EVT BEAMACT ") and beam_fmt.startswith("EVT BEAM b="),
      "the two prefixes cannot alias")

# ---- regression guard: idle current must not latch the last driving value ---
# Found live on the bench 2026-09-28: s_isenseCounts is only ever written by the
# CONTINUOUS-mode ADC ISR/current_Service() and was never cleared going idle, so
# it latched whatever the last driving batch held and the pushed EVT STATE /
# STATUS kept reporting real motor current (observed: a steady 2680 mA) with the
# driver asleep. Guard the fix at the source level since current.c touches raw
# MSP430 registers and cannot be compiled on the host like beam.c can.
print("\n[C9] idle current regression guard")
cur = io.open(os.path.join(MSP, "current.c"), encoding="utf-8").read()
setmode = cur[cur.index("void current_SetMode("):cur.index("void current_Init(")]
idle_branch = setmode[setmode.index("if (mode == CURRENT_MODE_IDLE) {"):]
check("s_isenseCounts = 0;" in idle_branch,
      "current_SetMode(CURRENT_MODE_IDLE) zeroes s_isenseCounts")
check(idle_branch.index("s_isenseCounts = 0;") < idle_branch.index("current_RequestVm();"),
      "isense is cleared before the VM re-sample kick (both must land the same tick)")

# ---- regression guard: ADC/TA0 stall-recovery watchdog must stay wired in ---
# Root-caused live on the bench 2026-09-28: partway into essentially every
# drive, Timer_A0 (the ADC trigger) stops counting - confirmed with a register
# dump (ADC12ENC stayed set, TA0R itself stopped advancing, s_sampleIdx stuck
# mid-batch). Closing the ADC12ENC re-arm race by moving it into the ISR did
# NOT stop it recurring, so the exact silicon mechanism is still not 100%
# pinned down - what IS shipped and bench-verified is a watchdog in
# current_Service() that detects an overdue batch and force-restarts the
# whole TA0+ADC12 pipeline, keeping over-current/obstacle protection from
# ever going blind for more than one short, bounded window. Guard its
# presence at the source level (current.c touches raw registers, can't be
# compiled on the host like beam.c).
print("\n[C10] ADC/TA0 stall-recovery watchdog regression guard")
svc = cur[cur.index("void current_Service("):]
svc = svc[:svc.index("\nvoid ", 10)] if "\nvoid " in svc[10:] else svc
check("armContinuous()" in cur, "armContinuous() exists (the shared re-init the watchdog calls)")
check("ADC_STALL_RECOVERY_MS" in svc,
      "current_Service() checks the stall watchdog threshold")
check("armContinuous();" in svc,
      "a stalled pipeline gets force-restarted, not just logged")
gcfg = io.open(os.path.join(MSP, "gate_config.h"), encoding="utf-8").read()
check("#define ADC_STALL_RECOVERY_MS" in gcfg, "ADC_STALL_RECOVERY_MS is defined")
# In-ISR re-arm: both the clear and the set of ADC12ENC must appear inside
# ADC12_ISR, with no re-arm left for the main loop (current_Service()) to do -
# that main-loop gap was the original race.
isr = cur[cur.index("__interrupt void ADC12_ISR"):]
isr = isr[:isr.index("\n}\n") + 3]
check(isr.count("ADC12CTL0") >= 2 and "&= ~ADC12ENC" in isr and "|= ADC12ENC" in isr,
      "ADC12ENC is toggled off AND back on inside the ISR itself (no main-loop gap)")

print("\n%s (%d failure(s))" % ("TESTS FAILED" if fails else "ALL TESTS PASSED", len(fails)))
sys.exit(1 if fails else 0)
