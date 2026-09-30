/*******************************************************************************
 * gate_config.h - COMPILE-TIME configuration: hardware/architecture facts that
 * never change at runtime (pin timing, ADC scaling, buffer sizes).
 *
 * Runtime-tunable behavior (durations, duty cycles, thresholds) lives in RAM
 * in g_cfg (nvconfig.h) and is persisted to flash; the NVCFG_DEFAULT_* values
 * below are only the factory-default seed used the first time the board boots
 * (or after a DEFAULTS command) - see project_plan.md Rev 3 Sec 8/9.
 ******************************************************************************/
#ifndef GATE_CONFIG_H
#define GATE_CONFIG_H

/* ---- Bench TEST MODE ----------------------------------------------------------
 * 1 = bench rig. The devices you don't have wired - the two LIMIT switches, the
 *     REMOTE input, and the photo beam - are reconfigured to an internal
 *     pull-DOWN and read ACTIVE-HIGH, so an unconnected pin reads "inactive"
 *     (no dual-limit lockout) and you assert one by poking the pin to 3V3.
 *     The OPEN/CLOSE/TOGGLE buttons KEEP their internal pull-UPs and stay
 *     active-LOW, so real momentary switches work normally. Moves are also
 *     forced to 100% duty (gate_fsm.c enterMoving). nFAULT/nHOME are real
 *     DRV8262 signals and are never inverted. "*** TEST MODE ***" prints at boot.
 * 0 = production: field wiring, all operator inputs active-LOW with pull-UPs.
 * MUST be 0 for a real gate install. Override with -DTEST_MODE=0 if preferred. */
#ifndef TEST_MODE
#define TEST_MODE 0
#endif

/* Optional build id folded into the VERSION string / boot banner so a running
 * image can be identified after a FWUPDATE. Override from the build, e.g.
 *   -DFW_BUILD_ID='" g1a2b3c4"'      (keep the leading space)
 * Must be a string literal - it is concatenated, not printf'd. */
#ifndef FW_BUILD_ID
#define FW_BUILD_ID ""
#endif

/* ---- Travel-limit input polarity --------------------------------------------
 * IN_LIMIT_OPEN (P7.5, Arm-Retracted) and IN_LIMIT_CLOSED (P7.6, Arm-Extended)
 * are ALWAYS configured with the MSP430 internal pull-UP: the limit hardware is
 * switch -> optocoupler -> MSP430 and the opto output transistor is effectively
 * open-drain (it can only pull the pin LOW), so the pull-up is what lets the
 * pin read HIGH at all. This flag only chooses the LOGICAL reading - which
 * level means "at the limit":
 *   0 = active-LOW : pin LOW  = at the limit  (opto conducting / contact to GND)
 *   1 = active-HIGH: pin HIGH = at the limit  (opto NOT conducting at the limit)
 * Which is correct depends on whether the switch's NC or NO contact drives the
 * opto LED - set it once that's known on the real gate. Buttons and REMOTE keep
 * their own polarity. Override from the build with -DLIMIT_SWITCH_ACTIVE_HIGH=0. */
#ifndef LIMIT_SWITCH_ACTIVE_HIGH
#define LIMIT_SWITCH_ACTIVE_HIGH 0
#endif

/* ---- REMOTE toggle input (P7.4, acts exactly like the TOGGLE button) --------
 * P7.4 is ALWAYS configured with the MSP430 internal pull-UP (board.c
 * gpio_Init), in every build / mode - the receiver feeds an optocoupler whose
 * output is open-drain (can only sink), so the pull-UP is what lets the pin
 * read HIGH at all. This flag ONLY chooses the LOGICAL reading (inputs.c
 * rawActive) - which level means "pressed":
 *   1 = pin HIGH = pressed   (DEFAULT - the opto stage inverts the receiver
 *       output on this install, so a press releases the pin to the pull-UP)
 *   0 = pin LOW  = pressed
 * Override for a single build with -DREMOTE_ACTIVE_HIGH=0 or =1. */
#ifndef REMOTE_ACTIVE_HIGH
#define REMOTE_ACTIVE_HIGH 1
#endif

/* ---- Photo-beam input (P8.6) - two independent knobs --------------------- *
 *
 * PHOTOBEAM_PULL_UP - MSP430 internal pull on P8.6 (board.c gpio_Init):
 *   1 = internal pull-UP   (DEFAULT)
 *   0 = internal pull-DOWN
 * The install here is: open-drain photo-eye -> external 10k pull-up to 12V ->
 * optocoupler -> opto output (also open-drain) -> P8.6. An open-drain opto can
 * only pull the pin LOW, so it needs the internal pull-UP to read HIGH at all.
 * Use pull-DOWN only if the sensor/opto actively drives the pin HIGH.
 *
 * PHOTOBEAM_ACTIVE_HIGH - which LOGICAL level means "beam broken" (obstruction),
 * inputs.c rawActive():
 *   0 = active-LOW : pin LOW  = beam broken   (DEFAULT - matches this install:
 *                    opto conducts / pin sinks LOW when the beam is broken)
 *   1 = active-HIGH: pin HIGH = beam broken
 *
 * FAIL-SAFE NOTE: with pull-UP + active-LOW (the defaults) a disconnected or
 * unpowered sensor floats HIGH and reads "beam clear" - the gate will still
 * close on a dead sensor. For cut-cable-blocks-closing, either set
 * PHOTOBEAM_PULL_UP=0 (if the live signal can still drive HIGH against the
 * internal pull-down) or fit an external ~4.7k pull-DOWN at the P8.6 connector.
 *
 * Override either for a single build with -DPHOTOBEAM_PULL_UP=0/1 and
 * -DPHOTOBEAM_ACTIVE_HIGH=0/1. */
#ifndef PHOTOBEAM_PULL_UP
#define PHOTOBEAM_PULL_UP 1
#endif
#ifndef PHOTOBEAM_ACTIVE_HIGH
#define PHOTOBEAM_ACTIVE_HIGH 0
#endif

/* ---- Obstacle-on-reverse over-current handling (gate_fsm.c tickObstacleReverse)
 * After the gate hits an obstacle while closing it drives back toward OPEN for
 * OBSTACLE_REVERSE_TIME ms. The motor's own restart inrush during that reverse
 * can look like a fresh over-current; the old behaviour escalated that to a
 * stall fault, and STALL_LOCKOUT_COUNT of them latched a LOCKOUT needing a
 * power cycle.
 *   0 = the soft over-current trip is IGNORED during the reverse so the gate
 *       actually backs off; only a sustained HARD over-current stops the move,
 *       and only to mid-travel - never a stall count, never a lockout (DEFAULT).
 *   1 = old behaviour: any over-current during the reverse raises a stall fault
 *       and can lock out.
 * The nFAULT (driver fault) and hard-limit paths are unaffected either way. */
#ifndef OBSTACLE_REVERSE_OC_STALL
#define OBSTACLE_REVERSE_OC_STALL 0
#endif

/* ---- Heartbeat LED D6 / P1.0 (board.c TimerA2_ISR) --------------------------
 * Blink half-period in ms: D6 toggles every HEARTBEAT_TOGGLE_MS, so the full
 * on/off cycle is 2x this. 500 -> ~1 Hz. Give each build a distinctive value
 * (e.g. 125 / 250 / 500 / 1000) and you can tell at a glance from D6's speed
 * whether an ESP32/BSL reflash actually loaded the new image. Range 1..65535. */
#ifndef HEARTBEAT_TOGGLE_MS
#define HEARTBEAT_TOGGLE_MS      200u
#endif

/* ---- Timer_A2 tick periods (board.c, board_SetTickRate) -------------------
 * The idle tick was 50 ms. It is now 10 ms so that (a) a photo-beam edge is
 * seen within 10 ms instead of up to 50 ms, (b) a break is TIMED to +/-10 ms
 * instead of +/-50 ms, and (c) a console command lands on the FSM within
 * 10 ms. Everything downstream of the tick is expressed in real milliseconds
 * (debounce, all FSM down-counters, timesync), so nothing else needed
 * retuning except IDLE_VM_SAMPLE_TICKS (a tick count) just below and the
 * fault-LED blink, which board.c now accumulates in ms.
 * Cost: the CPU leaves LPM0 100x/s instead of 20x/s. Each wake is tens of
 * microseconds, so this is <1% duty on top of an LPM0 baseline that already
 * keeps SMCLK running at 20 MHz. */
#define TICK_ACTIVE_MS           1u      /* while moving/ramping/braking        */
#define TICK_IDLE_MS             10u     /* while at rest (was 50)              */
#define TICK_ACTIVE_TACCR0       2499u   /* SMCLK/8 = 2.5 MHz -> 2500 cnt = 1 ms */
#define TICK_IDLE_TACCR0         3124u   /* SMCLK/64 = 312.5 kHz -> 3125 = 10 ms */
#define TICK_IDLE_EXDIV          TAIDEX_7 /* combined with ID_3 -> /8 * /8 = /64 */
#define TICK_ACTIVE_EXDIV        TAIDEX_0 /* /1 extra -> /8 total               */

/* ---- PWM (drv8262.c) ------------------------------------------------------- */
#define PWM_PERIOD_COUNTS        100u    /* SMCLK/8/100 = 25 kHz carrier         */
#define PWM_PERIOD_TBCCR0        (PWM_PERIOD_COUNTS - 1u)
#define DRV_WAKE_DELAY_CYCLES    30000u  /* ~1.5 ms @ 20 MHz nSLEEP wake time    */
#define DRV_FAULT_PULSE_CYCLES   600u    /* ~30 us nSLEEP low pulse to clear a latched fault */
#define BRAKE_RAMP_MS            100u    /* wind drive duty to 0 over this long before short-braking
                                           at a limit - braking straight from full duty dumps the
                                           winding energy as a big IPROPI current spike */

/* ---- ADC / current sense (current.c) --------------------------------------- */
#define ADC_TRIGGER_TACCR0       2499u   /* SMCLK/8 -> 1 kHz ADC trigger (was 25k)*/
#define ADC_AVG_SAMPLES          8u      /* 8 @ 1 kHz = 8 ms avg (was 48 @ 25kHz) */
#define ADC_PHASE_STEP           37u     /* decorrelates trigger phase vs the 100-count PWM period */
#define ADC_FULLSCALE            4095u
/* Physical parts are 3.3k on EACH IPROPI pin (R81/R82). In single-H-bridge mode
 * the J19 jumper ties IPROPI1+IPROPI2 together, so the combined mirror current
 * (I_motor * ISENSE_UA_PER_A) flows through R81 || R82 = 1650 ohm. Using 3300
 * here made the firmware read HALF the real current. J18/J20 must stay OUT (a
 * fitted jumper shorts that pin's sense resistor = current regulation disabled).
 * VERIFY with a clamp meter vs the capture CSV; tweak MA_PER_COUNT_Q8 if the
 * ratio is off. */
#define R_IPROPI_OHMS            1650u   /* R81 || R82, single-bridge (J19 fitted) */
#define ISENSE_UA_PER_A          212u    /* DRV8262 IPROPI mirror gain, uA per A  */
#define IDLE_VM_SAMPLE_TICKS    100u     /* @ 10 ms idle tick -> ~1 Hz VM sampling*/

/* ADC/TA0 stall-recovery watchdog (current.c current_Service) - added
 * 2026-09-28 after root-causing a live current-sense freeze on the bench (see
 * the ADC12_ISR header comment in current.c for the full story). A batch
 * normally completes every ADC_AVG_SAMPLES=8 ms; this is a generous multiple
 * of that so ordinary jitter never falsely triggers a reset, while still
 * recovering fast enough that over-current protection is never blind for
 * long. */
#define ADC_STALL_RECOVERY_MS    30ul

/* ---- Per-move data capture (current.c) ----------------------------------- *
 * During every open/close/home/reverse the driver records {mA, mV} into a RAM
 * ring at CAPTURE_INTERVAL_MS spacing, then dumps it as CSV over the console
 * when the move ends (paced against the UART ring so no bytes are dropped).
 * CAPTURE_SAMPLES * CAPTURE_INTERVAL_MS is the covered window; keep the product
 * < 65535 (the t_ms column is printed as a 16-bit value). 1024 * 25 = 25.6 s,
 * 4 KB of RAM. */
#define CAPTURE_SAMPLES          1024u
#define CAPTURE_INTERVAL_MS      25u
#define CAPTURE_DUMP_GROUP       10u     /* records per CSV line in the dump, joined by '~',
                                           to keep the dump inside a short terminal history */

/* ADC12SREF_0 (VR+=AVCC, VR-=AVSS) is used for every channel - no external
 * VeREF+ wiring needed, and it puts the ADC's full 4095-count range at ~AVCC
 * (~3.3 V), matching the DRV8262 hardware current-limit ceiling (Sec 7.2). */

/* mA per raw ADC count, Q8 fixed point (avoids float in the hot path):
 *   mA = counts * MA_PER_COUNT_Q8 / 256
 * mA = counts * ADC_VREF_MV * 1000 / (ADC_FULLSCALE * R_IPROPI_OHMS * ISENSE_UA_PER_A * 1e-6)
 * At ADC_VREF_MV ~= AVCC ~= 3300 mV, R_IPROPI_OHMS=1650: ~2.304 mA/count -> Q8 ~= 590.
 * Full-scale (4095 counts) is now ~9.4 A, matching the DRV8262 hardware trip. */
#define ADC_VREF_MV              3300u
#define MA_PER_COUNT_Q8          590u

/* ---- VREF1 current-regulation DAC enable/disable (board.c, drv8262.c) ------
 * 1 = drive VREF1 from DAC12_1/P6.7 (runtime-adjustable via
 *     VREF_CURRENT_REGULATION - see the constants/comment just below).
 * 0 = DISABLED (default, for now) - reverts to the original fixed ~3.3 V GPIO
 *     drive (P6.6 AND P6.7 both driven high in board.c), byte-for-byte the
 *     pre-DAC behavior. VREF_CURRENT_REGULATION is still stored/range-checked
 *     by nvconfig but inert, exactly like the old CURRENT_LIMIT was. Flip to 1
 *     (and rebuild) once the DAC path has been bench-verified not to disturb
 *     anything. Override with -DVREF_DAC_ENABLE=1. */
#ifndef VREF_DAC_ENABLE
#define VREF_DAC_ENABLE 0
#endif

/* ---- VREF1 current-regulation DAC (drv8262.c) ------------------------------
 * With the EVM jumpered J25 + J6 (this build), DRV8262 pin 33 (VREF1/VREF)
 * reaches MSP430 P6.7 / DAC12_1 (see PinoutGuide.md and drv8262.c file header).
 * Scheme: internal REF module at DAC_REF_MV (REFVSEL_0 = 1.5 V), DAC12_1 gain
 * = 3x (DAC12IR cleared):
 *   V_VREF = 3 * DAC_REF_MV * code/4095, clamped at AVCC once code exceeds
 *   ~3003 (nominal 3.3V AVCC) - matches the ADC current-sense ceiling (Sec
 *   above), since both ride the same AVCC rail through the same
 *   R_IPROPI_OHMS/ISENSE_UA_PER_A.
 * mA -> DAC12_1DAT code, Q16 fixed point (drv8262_SetCurrentLimitMa):
 *   code = mA * DAC_CODE_PER_MA_Q16 / 65536
 * Derivation: code = I_TRIP(mA) * R_IPROPI_OHMS * ISENSE_UA_PER_A * 1e-6 /
 *             (1000 * 3 * DAC_REF_MV/1000) * 4095
 *           = mA * 1650 * 212e-6 / 4.5 * 4095 ~= mA * 0.31831
 * DAC_CODE_PER_MA_Q16 = round(0.31831 * 65536) = 20861.
 * Currently gated off by VREF_DAC_ENABLE (see just above) pending bench
 * verification - these constants are unused while it's 0. */
#define DAC_REF_MV               1500u   /* internal REF module, REFVSEL_0 */
#define DAC_CODE_PER_MA_Q16      20861u  /* round(0.31831 * 65536) - see above */

/* VSEN_VM resistor divider (board schematic: R98 = 97.6k from VM to the sense
 * node, R104 = 4.99k from the sense node to GND, C71 = 100pF filter to GND):
 *   V_pin = VM * R104 / (R98 + R104)  ->  VM = V_pin * (R98 + R104) / R104
 * Folded together with the ADC_VREF_MV/ADC_FULLSCALE pin-voltage conversion
 * into one mV-per-count Q8 constant (avoids compounding rounding error from
 * two separate integer divides):
 *   mV/count = ADC_VREF_MV/ADC_FULLSCALE * (R98+R104)/R104
 *            = 3300/4095 * 102590/4990 ~= 16.5686  ->  Q8 ~= 4242 */
#define VM_DIVIDER_R98_OHMS      97600u
#define VM_DIVIDER_R104_OHMS      4990u
#define VM_MV_PER_COUNT_Q8        4242u

/* ---- Debounce (inputs.c) --------------------------------------------------- */
#define INPUT_DEBOUNCE_MS        30u
#define NFAULT_DEBOUNCE_MS       16u
/* Photo-beam "broken" polarity moved to PHOTOBEAM_ACTIVE_HIGH, up with the
 * other input-polarity build options.
 *
 * PHOTOBEAM_DEBOUNCE_MS - the beam gets its OWN, shorter debounce so a real
 * vehicle-tyre break (as short as AXLE_BREAK_MIN_MS = 30 ms) is not swallowed
 * by the 30 ms general-purpose input debounce. 10 ms is still ample for the
 * opto/photo-eye electrical settling this install needs.
 * SAFETY NOTE: input_Level(IN_PHOTOBEAM) is the single source of truth for
 * BOTH the reversal logic and the new reporting, so beam-triggered reversal
 * now reacts ~20 ms SOONER than before. That is strictly in the safe
 * direction, and it is the only behavioural change to the safety path. Set
 * this back to 30u (or -DPHOTOBEAM_DEBOUNCE_MS=30) to restore the exact old
 * reversal timing - at the cost of missing the shortest tyre breaks. */
#ifndef PHOTOBEAM_DEBOUNCE_MS
#define PHOTOBEAM_DEBOUNCE_MS    10u
#endif

/* ---- Cold-boot pull-up settle (board.c board_Init) ---------------------- *
 * The P7 limit-switch inputs have NO external pull resistor - only the MSP430
 * internal pull-UP (PinoutGuide Sec 5.1, and always pull-UP now - see
 * LIMIT_SWITCH_ACTIVE_HIGH). On a true power-on (ESP32, no debugger) the pins
 * charge from 0 V through that pull-up, so with LIMIT_SWITCH_ACTIVE_HIGH == 0
 * (LOW = at the limit) they briefly read "both limits active" -> a false
 * LOCKOUT_DUAL_LIMIT (D12 fast-blinking until an operator clears it). Wait this
 * long after gpio_Init() enables the pulls, before the first input sample, so
 * the pin + wiring capacitance has charged. Harmless boot latency with
 * LIMIT_SWITCH_ACTIVE_HIGH == 1 (a still-low pin reads "not at the limit").
 * Never seen under the debugger (board already powered through flash+load).
 * ~10 ms @ 20 MHz; raise it for long limit-switch cabling on an active-LOW build. */
#define BOOT_PULL_SETTLE_CYCLES  200000u

/* ---- UART / console (uart.c, console.c) ------------------------------------ */
#define UART_BAUD                115200u
#define UART_UCA1BR0             10u
#define UART_UCA1BR1             0u
#define UART_UCA1MCTL            0xE1u   /* UCBRF_14 | UCBRS_0 | UCOS16, ~0.22% error */
#define UART_TX_RING_LEN         4096u   /* must be a power of 2 - LIST_DETAILS prints ~20
                                             params with a sentence each, ~2.8 kB in one
                                             burst; 1024 dropped most of it (plain LIST is
                                             ~900 bytes and still fits fine) */
#define UART_RX_RING_LEN         64u     /* must be a power of 2                  */
#define CONSOLE_RX_LINE_MAX      40u     /* hard cap - overflow -> ERR LINE_TOO_LONG */
#define DBG_PRINTF_BUF_LEN       256u    /* ONE static scratch buffer, no stack alloc -
                                           holds the longest LIST_DETAILS line */

/* ---- Flash-backed configuration (nvconfig.c) ------------------------------- */
#define NVCONFIG_MAGIC           0xC0DEu
#define NVCONFIG_VERSION         4u      /* v2: added SOFT_END_* params; v3: added
                                           RAMP_START_DUTY + OC_GRACE_TIME; v4:
                                           CURRENT_LIMIT renamed to
                                           VREF_CURRENT_REGULATION and made a real
                                           DAC12_1-driven VREF1 (range tightened
                                           500-9400) - a saved v3 blob re-seeds to
                                           defaults */

/* ---- Factory defaults for runtime-tunable parameters (nvconfig.c) ---------- */
#define NVCFG_DEFAULT_MAX_DRIVE_DURATION_S    16u
#define NVCFG_DEFAULT_HOMING_TIMEOUT_S        45u
#define NVCFG_DEFAULT_RUN_DUTY_PCT           100u   /* normal open/close drives full-on (static high, no PWM) */
#define NVCFG_DEFAULT_HOMING_DUTY_PCT         85u   /* homing creeps slower to find the limit gently */
#define NVCFG_DEFAULT_REVERSE_DUTY_PCT        70u
#define NVCFG_DEFAULT_RAMP_TIME_MS          2500u   /* soft-start: RAMP_START_DUTY -> full duty over ~2.5 s to keep inrush below OC_TRIP */
#define NVCFG_DEFAULT_RAMP_START_DUTY_PCT     75u   /* open/close LAUNCH at this duty instead of from a standstill, so the motor
                                                      breaks free right away, then ramps to RUN_DUTY over RAMP_TIME. 0 = old
                                                      behaviour (ramp up from 0). Homing and obstacle-reverse are unaffected. */
#define NVCFG_DEFAULT_OBSTACLE_REVERSE_MS     750u
#define NVCFG_DEFAULT_OPENING_BACKOFF_MS      250u
#define NVCFG_DEFAULT_BRAKE_TIME_MS          1000u
/* OC thresholds below are in the CORRECTED (R_IPROPI=1650) scale, i.e. ~2x the
 * old numbers for the same physical amps. Re-tune OC_TRIP from a capture CSV:
 * set it ~20% above the real closing peak once the ramp has settled. The soft
 * threshold sits well above the observed startup breakaway now; OC_BLANK_TIME
 * blanks it for the first part of the ramp, and OC_GRACE_TIME can suppress the
 * hard trip too for a heavy start (see below). */
#define NVCFG_DEFAULT_OC_TRIP_MA             9000u
#define NVCFG_DEFAULT_OC_HARD_LIMIT_MA       9400u
#define NVCFG_DEFAULT_VREF_CURRENT_REGULATION_MA 9400u   /* DRV8262 hardware I_TRIP,
                                           DAC12_1-driven via P6.7/VREF1 (J25+J6) -
                                           ~= the AVCC-clamped ceiling with
                                           R81||R82 = 1650 ohm; see gate_config.h
                                           DAC_CODE_PER_MA_Q16 and drv8262.c */
#define NVCFG_DEFAULT_STALL_LOCKOUT_COUNT       5u
#define NVCFG_DEFAULT_LOCKOUT_CLEAR_STABLE_MS 2000u
#define NVCFG_DEFAULT_IDLE_SLEEP_ENABLE          1u
#define NVCFG_DEFAULT_OC_BLANK_MS            1500u   /* soft-trip blank from move start - now long enough to cover the launch breakaway */
#define NVCFG_DEFAULT_OC_GRACE_MS               0u   /* extra window from move start where BOTH soft AND hard OC are ignored
                                                       (0 = off). Set to a few thousand ms to ride through a stiff startup;
                                                       less safe, but the stall timeout and stuck-limit checks still apply. */
#define NVCFG_DEFAULT_VM_LOW_CUTOFF_MV       10800u
#define NVCFG_DEFAULT_VM_LOW_RECOVER_MV      11200u
#define NVCFG_DEFAULT_VM_LOW_CONFIRM_MS        500u
#define NVCFG_DEFAULT_DEADTIME_MS              150u
/* Soft end-of-travel: after SOFT_END_OPEN/CLOSE ms into a normal drive, duty
 * ramps down to SOFT_END_DUTY for the rest of the travel so the gate eases into
 * the limit instead of slamming it. 0 ms = disabled. Defaults below are tuned
 * from bench measurements of ~10.4 s (open) / ~10.7 s (close) full travel,
 * leaving only a ~0.6-0.9 s eased-in cushion before each limit (deliberately
 * short) - re-tune from the capture logs if the real travel time on your gate
 * differs. */
#define NVCFG_DEFAULT_SOFT_END_DUTY_PCT        60u
#define NVCFG_DEFAULT_SOFT_END_OPEN_MS       9800u
#define NVCFG_DEFAULT_SOFT_END_CLOSE_MS      9800u

/* ===========================================================================
 * Event-driven reporting (gate_fsm.c, beam.c) - added 2026-09-28
 * ---------------------------------------------------------------------------
 * The controller used to say nothing on its own; the ESP32 bridge asked it for
 * a STATUS block every 8 s at rest. That poll interval WAS the latency Home
 * Assistant saw on a remote-started move (2.8-5.8 s, i.e. a random point in an
 * 8 s window) and it was also why a short beam break either vanished or looked
 * like one full 8.1 s poll period.
 *
 * The controller now PUSHES:
 *   - one compact "EVT STATE ..." line the moment any discrete field changes,
 *   - one "EVT BEAM ..." line on every debounced photo-beam edge,
 *   - one "EVT MOVE_START ..." line in the same control-loop pass that
 *     energises the motor,
 *   - one "EVT BEAMACT ..." line per classified beam-activity window,
 *   - a full STATUS block every STATUS_HEARTBEAT_MS as a backstop, and on
 *     demand (the plain STATUS command is the bridge's status request).
 * Every one of those is a normal ';'-terminated console message, so the
 * existing serial framing and the web console are unchanged.
 * ========================================================================= */

/* Full-STATUS heartbeat while nothing is happening. This is only a backstop
 * against a bridge/HA restart losing the retained state - the real recovery
 * path is the bridge sending STATUS on boot and on every MQTT reconnect. */
#ifndef STATUS_HEARTBEAT_MS
#define STATUS_HEARTBEAT_MS      300000ul   /* 5 minutes */
#endif

/* Progress line cadence while the motor is driving (~2 Hz). */
#define DRIVE_PROGRESS_MS        500u

/* ---- Photo-beam activity classification (beam.c) ---------------------------
 * The beam's mounting height is not known, so both patterns are supported:
 *   Pattern A "axle pair" - a low beam sees only the tyres: two or more short
 *     breaks separated by a wheelbase-sized gap.
 *   Pattern B "body" - a high beam sees the car body: one long break.
 * A classification window opens on the first blocked edge and closes once the
 * beam has been clear for BEAM_AXLE_GAP_MAX_MS; one event is published then.
 *
 * Reference numbers: a 2.8 m wheelbase at 1-4 m/s gives gaps of ~0.6-2.5 s and
 * tyre breaks of ~60-250 ms; a ~4.8 m body at driveway speed takes ~1.2-4 s.
 *
 * These are compile-time #defines rather than nvconfig params on purpose: the
 * nvconfig blob is version-stamped and a version bump RE-SEEDS every saved
 * parameter back to its factory default, which would throw away the gate's
 * tuned OC/ramp/soft-end values. Re-tune these from the bridge's
 * msp430gate/diag/beam_edges topic and rebuild. */
#define BEAM_AXLE_BREAK_MIN_MS     30u
#define BEAM_AXLE_BREAK_MAX_MS   1000u
#define BEAM_AXLE_GAP_MIN_MS      300u
#define BEAM_AXLE_GAP_MAX_MS     3500u
#define BEAM_BODY_BREAK_MIN_MS    700u
#define BEAM_BODY_BREAK_MAX_MS   6000u

/* BEAM_AXLE_PATTERN_ENABLE - whether the axle-pair ("tyre") pattern is even
 * considered. This install's beam is mounted at body height (confirmed from
 * real captures: every genuine car produces one continuous ~1-2 s break, not a
 * tyre pair), so a rapid double-tap - a person testing the beam, a dog, a
 * strong gust catching a leaf - can spuriously satisfy the axle-pair gap
 * window (300-3500 ms) and get reported as a false vehicle_pass. Disabled by
 * default here for that reason.
 *   1 = both patterns considered (axle_pair AND body) - appropriate for a
 *       LOW beam that genuinely only sees tyres.
 *   0 = axle_pair can never fire; only a single continuous body-height break
 *       can classify as vehicle_pass. As a direct benefit, a classification
 *       window no longer needs to wait BEAM_AXLE_GAP_MAX_MS after a break
 *       clears "just in case a second axle follows" - beam_Tick() closes and
 *       publishes the instant that single break clears, so a real car's
 *       vehicle_pass now arrives with ~zero delay instead of ~3.5 s later.
 *       Trade-off: a genuine multi-break sequence (e.g. a trailer's second
 *       bump) becomes a separate event instead of one grouped one.
 * Override with -DBEAM_AXLE_PATTERN_ENABLE=1 if the beam is ever remounted
 * low enough to see only tyres. */
#ifndef BEAM_AXLE_PATTERN_ENABLE
#define BEAM_AXLE_PATTERN_ENABLE 0
#endif

/* Most breaks/gaps recorded in one classification window (a car + trailer is
 * 3-4 breaks; the count reported is the true count, this only bounds how many
 * individual durations are listed in the event payload). */
#define BEAM_MAX_BREAKS             8u

/* Rolling "breaks in the last 10 s" counter reported as an attribute. */
#define BEAM_RATE_WINDOW_MS     10000ul
#define BEAM_RATE_RING            16u

/* Chatter guard (sun glare / rain / a bug on the lens). If the beam produces
 * more than BEAM_CHATTER_EDGES_PER_S edges in one second, per-edge lines stop
 * and one line is emitted at most every BEAM_CHATTER_MIN_GAP_MS with chat=1,
 * so a flapping sensor cannot fill the UART ring. The SAFETY path is
 * unaffected - it reads the debounced level directly, not these lines. */
#define BEAM_CHATTER_EDGES_PER_S   10u
#define BEAM_CHATTER_MIN_GAP_MS   100u

/* ---- Capture-dump UART headroom (current.c capture_DumpService) -----------
 * The end-of-move CSV dump used to fill the 4 KB TX ring to within 220 bytes
 * of full, so a beam edge landing mid-dump could sit ~350 ms behind it at
 * 115200 baud. The dump now never queues more than this many bytes at a time,
 * bounding that worst case to ~45 ms. The dump itself is UART-rate-limited
 * either way, so it takes no longer overall. */
#define CAPTURE_DUMP_MAX_QUEUED   512u

/* ---- Verbose/debug console output ------------------------------------------
 * 0 (default) = only event lines, move progress, faults and replies.
 * 1 = also the per-press "INPUT: ..." lines and other development chatter.
 * The command source is carried by EVT MOVE_START / EVT STATE now, so the
 * INPUT: lines are no longer needed to tell a remote press from a button. */
#ifndef GATE_DEBUG
#define GATE_DEBUG 0
#endif

#endif /* GATE_CONFIG_H */
