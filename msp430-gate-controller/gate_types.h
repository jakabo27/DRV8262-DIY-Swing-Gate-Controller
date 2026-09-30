/*******************************************************************************
 * gate_types.h - shared enums/types used across the GateController modules.
 * No storage, no register access - safe to include everywhere.
 ******************************************************************************/
#ifndef GATE_TYPES_H
#define GATE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Gate travel direction. Forward drive = OPEN, reverse drive = CLOSE.
 * (Confirmed on the bench - see drv8262.h if this ever needs to be flipped.) */
typedef enum {
    DIR_OPEN = 0,
    DIR_CLOSE = 1
} gate_dir_t;

/* Top-level gate state machine states - project_plan.md Rev 3 Sec 6. */
typedef enum {
    ST_INIT = 0,
    ST_IDLE_OPEN,
    ST_IDLE_CLOSED,
    ST_OPENING,
    ST_CLOSING,
    ST_SOFT_STOP,
    ST_STOPPED_MIDTRAVEL,
    ST_OBSTACLE_REVERSE,
    ST_HOMING,
    ST_UNKNOWN,
    ST_STALL_FAULT,
    ST_DRIVER_FAULT,
    ST_LOCKOUT,
    ST_LOW_BATTERY
} gate_state_t;

/* Why a lockout was entered - drives whether/how a button can clear it. */
typedef enum {
    LOCKOUT_NONE = 0,
    LOCKOUT_DUAL_LIMIT,
    LOCKOUT_NFAULT_PERSIST,
    LOCKOUT_REPEAT_STALL
} lockout_reason_t;

/* Why the motor stopped - used only for the human-readable debug line. */
typedef enum {
    STOP_ARRIVED = 0,
    STOP_COMMAND,
    STOP_OBSTACLE_OC,
    STOP_OBSTACLE_BEAM,
    STOP_OBSTACLE_HARD_OC,
    STOP_TIMEOUT,
    STOP_REVERSE_DONE,
    STOP_HOME_DONE
} stop_reason_t;

/* Operator/remote commands - produced by inputs.c from debounced button edges,
 * consumed one at a time by gate_fsm.c. */
typedef enum {
    CMD_NONE = 0,
    CMD_TOGGLE,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP,
    CMD_ACK
} cmd_t;

/* Where a command / state change came from. cmd_t itself carries no origin, so
 * inputs.c stores one of these alongside every queued command and gate_fsm.c
 * reports it (EVT MOVE_START src=..., EVT STATE src=...). Home Assistant
 * surfaces it as "Last command source". Keep the strings in
 * gate_fsm.c:cmdSrcStr() in step with this enum. */
typedef enum {
    CMDSRC_NONE = 0,
    CMDSRC_BOOT,              /* power-on / reset resolve                       */
    CMDSRC_HA,                /* console command over the ESP32 bridge link     */
    CMDSRC_REMOTE,            /* RF remote receiver, P7.4                       */
    CMDSRC_WIRED_BUTTON,      /* OPEN / CLOSE / TOGGLE buttons                  */
    CMDSRC_AUTO_REVERSE,      /* firmware-initiated obstacle reversal           */
    CMDSRC_OBSTRUCTION_STOP,  /* over-current / beam stop while opening         */
    CMDSRC_FAULT              /* stall, nFAULT, lockout, low battery            */
} cmd_src_t;

/* Debounced digital input channels. */
typedef enum {
    IN_LIMIT_OPEN = 0,   /* P7.5, Arm-Retracted  = OPEN  */
    IN_LIMIT_CLOSED,     /* P7.6, Arm-Extended   = CLOSED*/
    IN_BTN_TOGGLE,       /* P7.7 */
    IN_BTN_OPEN,         /* P6.5 */
    IN_BTN_CLOSE,        /* P6.0 */
    IN_REMOTE,           /* P7.4 - acts like TOGGLE */
    IN_PHOTOBEAM,        /* P8.6, pull per PHOTOBEAM_PULL_UP; "broken" level per PHOTOBEAM_ACTIVE_HIGH (defaults: pull-UP, LOW=broken) */
    IN_NFAULT,           /* P2.7, active-low */
    IN_NHOME,            /* P1.4 */
    IN_COUNT
} input_id_t;

/* Status LEDs. */
typedef enum {
    LED_HEARTBEAT = 0,  /* D6 */
    LED_BEAM_OK,        /* RETIRED - P1.1 is now BSL TX, D8/R113 depopulated; led_Set() is a no-op */
    LED_LIMIT_CLOSED,   /* RETIRED - P1.2 is now BSL RX, D9/R115 depopulated; led_Set() is a no-op */
    LED_LIMIT_OPEN,     /* D10 */
    LED_BUTTON,         /* D11 */
    LED_FAULT,          /* D12 */
    LED_COUNT
} led_id_t;

/* D12 (LED_FAULT) blink patterns - lets stall/lockout/low-battery be told apart
 * at a glance (project_plan.md Rev 3 Sec 6.1). */
typedef enum {
    FAULT_LED_OFF = 0,
    FAULT_LED_SOLID,       /* ST_STALL_FAULT */
    FAULT_LED_FAST_BLINK,  /* ST_LOCKOUT */
    FAULT_LED_SLOW_BLINK   /* ST_LOW_BATTERY */
} fault_led_pattern_t;

/* Timer_A2 tick profile - project_plan.md Rev 3 Sec 7.1. */
typedef enum {
    TICK_ACTIVE = 0,   /* 1 ms, while moving/ramping/braking */
    TICK_IDLE = 1      /* 50 ms, while at rest                */
} tick_rate_t;

/* Current-sense ADC servicing mode - project_plan.md Rev 3 Sec 7.2/4.5. */
typedef enum {
    CURRENT_MODE_CONTINUOUS = 0,  /* 1 kHz IPROPI averaging - motor is driven   */
    CURRENT_MODE_IDLE = 1         /* on-demand single-shot VM reads only        */
} current_mode_t;

#endif /* GATE_TYPES_H */
