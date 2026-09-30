/*******************************************************************************
 * main.c - GateController_DRV8262
 *
 * See the header comment at the top of gate_fsm.c for the full behavior
 * specification (normal cycle, UL 325 obstacle handling, stall/lockout/
 * low-battery safety states, soft start/stop, debounce/inrush constants).
 * See project_plan.md (Rev 3) for the full design rationale.
 *
 * This file is intentionally thin: initialize every module in dependency
 * order, then run a non-blocking super-loop. The Timer_A2 tick dynamically
 * switches between 1 ms (while the motor is moving/ramping/braking) and
 * 10 ms (while at rest) - see board_SetTickRate() - and the CPU sleeps in
 * LPM0 between ticks either way, so the console stays reachable at any time.
 *
 * Pass order inside the tick matters and is not arbitrary:
 *   inputs_Poll -> beam_Tick -> console_Service -> gate_fsm_Tick ->
 *   gate_fsm_ReportService -> capture_DumpService -> timesync_Tick
 * Beam edges are queued for transmission before anything else can occupy the
 * UART ring, and console_Service runs before the FSM so a command received
 * this tick is executed this tick rather than one tick later.
 *==============================================================================
 * FIELD WIRING GUIDE - new connections only (bench reference)
 *==============================================================================
 * The existing DRV8262<->MSP430 control wiring (IN1/IN2, nSLEEP, nFAULT,
 * MODE1/MODE2, TOFF, DECAY0, VREF1, IPROPI1/2, VM) is unchanged from the EVM
 * and is NOT repeated here - see PinoutGuide.md for that. Below is only the
 * new buttons/switches/sensor being wired onto the EVM's test points and
 * header jumpers for gate control.
 *
 * BOARD MOD REQUIRED FIRST: depopulate R5, R8, R13 and R16. They tie test
 * points A1-A4 to the analog phase-sense signals (VSEN_AOUT1/AOUT2/BOUT1/
 * BOUT2); left in place, the four digital inputs below read a fought
 * mid-rail voltage instead of a clean high/low.

 * BUTTON LAYOUT
 * [OPEN] [CLOSE] [TOGGLE]
 *
 *   Signal                          Test point / jumper           MSP430   Port
 *   ---------------------------------------------------------------------------
 *   TOGGLE button                   "A4"                           pin 8    P7.7
 *   REMOTE input (acts like TOGGLE) "A1"                           pin 5    P7.4
 *   OPEN button                     TP13                           pin 2    P6.5
 *   CLOSE button                    bottom header, 4th jumper      pin 97   P6.0
 *                                    from the bottom (J4, net on
 *                                    DRV8262 pin 35)
 *   Arm-Extended limit = CLOSED     "A3"                           pin 7    P7.6
 *   Arm-Retracted limit = OPEN      "A2"                           pin 6    P7.5
 *   Photo-beam sensor               top header, 2nd jumper from    pin 66   P8.6
 *                                    the bottom (J8, net on
 *                                    DRV8262 pin 31)
 *
 *   Every signal above wires straight to its MSP430 pin with NO external
 *   resistor needed - the firmware enables that pin's internal pull-up
 *   (board.c, gpio_Init). The buttons and limit switches are active-LOW
 *   (read low when pressed / switch closed). The photo beam (P8.6) has two
 *   build knobs in gate_config.h: PHOTOBEAM_PULL_UP (internal pull dir,
 *   default UP for the open-drain sensor/opto) and PHOTOBEAM_ACTIVE_HIGH
 *   (which level = "broken", default LOW). See the fail-safe note there.
 *
 * LED QUICK REFERENCE (already on the EVM - nothing to wire)
 *   D6  Green   P1.0  heartbeat - blinks about once a second whenever alive
 *   D8  --      P1.1  RETIRED. P1.1 is the ROM-BSL TX line to the ESP32
 *                      programmer - firmware leaves it Hi-Z. R113 depopulated.
 *   D9  --      P1.2  RETIRED. P1.2 is the ROM-BSL RX line from the ESP32
 *                      programmer - firmware leaves it Hi-Z. R115 depopulated.
 *   D10 Yellow  P1.3  OPEN limit switch state (Arm-Retracted)
 *   D11 Yellow  P1.5  lit while any button or the remote is pressed
 *   D12 Yellow  P1.6  fault: solid = stall, fast blink = lockout, slow
 *                      blink = low battery
 *   D7  (Super Red, driven directly by the DRV8262's own nFAULT output -
 *        not firmware controlled) lights on a hardware driver fault.

 ---------------------
 * ESP32 Pin    | Variable   | Target Connection
 * -------------+------------+----------------------------------------------------------
 * GPIO 6 (D4)  | PIN_RST    | MSP430 RST / NMI Pin
 * GPIO 7 (D5)  | PIN_TEST   | MSP430 TEST / TCK Pin
 * GPIO 21 (D6) | PIN_TX     | MSP430 BSL RX (P1.2 on F5529 / FR5xx series) - the "DRV_ENABLE" D9 LED on my EVM
 * GPIO 20 (D7) | PIN_RX     | MSP430 BSL TX (P1.1 on F5529 / FR5xx series) - the "GUI_STATUS" D8 LED on my EVM
 * NEW:  D10  --> MSP430 MCU_UARTTXD (Normal UART)
 * NEW: D9 --> MSP430 MCU_UARTRXD (Normal UART)
 *==============================================================================*/
#include <msp430.h>
#include "board.h"
#include "gate_config.h"
#include "uart.h"
#include "console.h"
#include "nvconfig.h"
#include "drv8262.h"
#include "current.h"
#include "inputs.h"
#include "gate_fsm.h"
#include "beam.h"
#include "timesync.h"

int main(void)
{
    board_Init();
    uart_Init();
    console_Init();
    nvconfig_Init();     /* load persisted config (or seed+save defaults)     */
    wdt_Kick();          /* first-boot path erases+writes a flash segment - keep the 250 ms WDT happy */
    drv8262_Init();      /* DRV8262 bring-up; starts asleep                   */
    current_Init();      /* ADC12 owner; starts in CONTINUOUS mode            */
    inputs_Init();
    inputs_Poll();        /* one immediate sample so gate_fsm_Init sees real levels */
    timesync_Init();      /* wall-clock estimate; stays unsynced until the ESP32 sends TIME: */

    gate_fsm_Init(input_Level(IN_LIMIT_OPEN), input_Level(IN_LIMIT_CLOSED));
    beam_Init();          /* photo-beam edge reporting + vehicle-pass classifier */

    dbg_Print("\r\nGateController_DRV8262 ready.\r\n");
    dbg_Printf("VERSION %s\r\n", fw_VersionString());   /* confirms which image booted (e.g. after FWUPDATE) */
#if TEST_MODE
    dbg_Print("*** TEST MODE: limits/remote/beam pull-down + active-HIGH, buttons pull-up + active-LOW, 100% duty ***\r\n");
#endif

    __bis_SR_register(GIE);

    for (;;) {
        wdt_Kick();
        current_Service();          /* drains ADC results whenever they arrive */
        if (g_tickFlag) {
            g_tickFlag = 0;
            inputs_Poll();
            beam_Tick();            /* photo-beam edges go out FIRST - top priority */
            console_Service();      /* ...then commands land in the FIFO BEFORE the FSM
                                       reads it, so an incoming OPEN/CLOSE/STOP is acted
                                       on in this same pass instead of the next one */
            gate_fsm_Tick();        /* drives drv8262_Service()/current_TickIdle() as needed */
            gate_fsm_ReportService(); /* EVT STATE on change + slow full-STATUS heartbeat */
            capture_DumpService(); /* paced CSV dump of the last move, if one is queued */
            timesync_Tick(board_GetTickMs());
        }
        board_SleepUntilTick();     /* LPM0 + GIE; wakes on the next tick or any other IRQ */
    }
}
