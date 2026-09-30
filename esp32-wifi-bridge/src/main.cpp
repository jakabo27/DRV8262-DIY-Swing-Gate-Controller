/*
 * =====================================================================================
 * PROJECT: MSP430 Wireless Flasher, Web Console & Time Sync Gateway
 * ENVIRONMENT: ESP32 / ESP-IDF / C++
 * FILE: src/main.cpp
 * =====================================================================================
 *
 * OVERVIEW:
 * ---------
 * This firmware transforms an ESP32 into a wireless Over-The-Air (OTA) programmer and 
 * WebSerial diagnostic bridge for Texas Instruments MSP430 microcontrollers 
 * (MSP430F5xx/6xx family using standard UART BSL).
 *
 * KEY FEATURES & ARCHITECTURE:
 * ----------------------------
 * 1. Automated BSL Firmware Flashing:
 *    - Parses TI-TXT format files directly from Code Composer Studio (CCS).
 *    - Automatically handles MSP430 BSL entry sequence using RST and TEST pins.
 *    - enter_bsl()/reset_msp430() automatically switch UART framing (9600 8E1 for
 *      BSL, 115200 8N1 for the console) and swap the shared UART_PORT peripheral
 *      between PIN_TX/PIN_RX and the always-on application pins PIN_APP_TX/
 *      PIN_APP_RX, since PIN_TX/PIN_RX double as the EVM's DRV_ENABLE/GUI_STATUS
 *      LEDs while the MSP430 application is running (P1.1/P1.2 are not a real UART
 *      outside of an actual BSL session on this device - no port-mapping on Port 1).
 *    - Mass-erases flash, unlocks BSL (32x 0xFF default password - only matches
 *      once erased), then streams write blocks in chunks, checking the target's
 *      real per-command status response (not just the low-level packet ACK)
 *      after every step.
 *
 * 2. Real-Time Web Serial Console & Log Downloader:
 *    - Bidirectional WebSocket streaming between the Web UI and the MSP430's
 *      normal application UART (MCU_UARTTXD/RXD on P8.2/P8.3), wired to the ESP32
 *      on the dedicated always-on pair PIN_APP_RX/PIN_APP_TX (D10/D3). Whatever
 *      you type in the web terminal goes straight to the running gate controller
 *      firmware ("help", status queries, variable tweaks, ...) and everything it
 *      prints back is streamed to every connected browser as the gate moves.
 *    - Client-side 500-line DOM buffer management to prevent browser memory leaks.
 *    - One-click `.txt` file export of captured serial logs (`msp430_serial_log.txt`).
 *    - Optional per-line client-side timestamps, an auto-scroll lock toggle, and
 *      a configurable line ending (None/CR/LF/CRLF) for outgoing typed messages.
 *    - This link is separate from the BSL flashing pins (PIN_TX/PIN_RX = the EVM's
 *      DRV_ENABLE/GUI_STATUS LEDs, only a live UART during a BSL session). The one
 *      UART peripheral is time-shared: the app console is unavailable only while a
 *      BSL session is held open on the flashing pins (enter_bsl()..reset_msp430()),
 *      since the gate firmware is halted then anyway.
 *
 * 3. NTP Time Synchronization Handshake:
 *    - Syncs ESP32 system clock with NTP (`pool.ntp.org`) upon Wi-Fi connection.
 *    - Transmits timestamp packet over UART every 30 minutes, on boot, and on
 *      demand via the "Force Time Sync" hardware action (do_time_sync()).
 *    - Protocol Format: "TIME:<10_DIGIT_UNIX_TIMESTAMP>\n" (e.g., "TIME:1725474469\n").
 *    - Sent on the same always-on application UART as the console (section 2), so
 *      it reaches the running firmware except while a BSL session is held open.
 *
 * 4. Dual-Mode Network Resilience:
 *    - Primary: Attempts connection to target Wi-Fi Access Point (WIFI_STA_SSID).
 *    - Fallback: Automatically spins up Standalone AP ("MSP430_Flasher" @ 192.168.4.1)
 *      if primary Wi-Fi is unreachable after 30 seconds. The fallback is NOT
 *      permanent - the ESP32 stays in APSTA mode and keeps retrying WIFI_STA_SSID
 *      forever in the background, re-joining automatically (and dropping back to
 *      just the fallback AP if it later loses WIFI_STA_SSID again).
 *
 * 5. Flashing Reliability & Diagnostics:
 *    - Live progress broadcast ("PROGRESS:<percent>:<label>\n") during erase/
 *      unlock/write/verify, rendered as a progress bar + step label in the UI.
 *    - Per-block verification via BSL_CRC_CHECK (0x16): compares the target's
 *      own CRC over each just-written range against calculate_bsl_crc() run
 *      locally over the same source bytes - no full readback required.
 *    - "Read Device Info" (CMD:INFO) queries BSL_VERSION (0x19) and dumps the
 *      raw TLV device descriptor (0x18 read @ 0x1A04). Decoding the device ID
 *      byte into a part number needs SLAU208's Table 60 and isn't attempted
 *      here - the raw bytes plus this project's known linker memory map are
 *      reported instead of guessing.
 *    - ESP32 self-telemetry (Wi-Fi RSSI/IP/uptime/free heap) broadcast every 5s
 *      as "TELEMETRY:<rssi>:<ip>:<uptime_s>:<heap_bytes>\n" for the status footer.
 *    - Client-side confirmation prompts before CMD:BSL/CMD:RST.
 *
 * HARDWARE PIN MAPPING:
 * ---------------------
 * ESP32 Pin     | Variable    | Target Connection
 * --------------+-------------+--------------------------------------------------------
 * GPIO 6  (D4)  | PIN_RST     | MSP430 RST / NMI / SBWTDIO   Pin 96
 * GPIO 7  (D5)  | PIN_TEST    | MSP430 TEST / SBWTCK         Pin 91
 * GPIO 21 (D6)  | PIN_TX      | MSP430 BSL RX = P1.2 / pin 36 (nominal; auto-swaps with D7)
 * GPIO 20 (D7)  | PIN_RX      | MSP430 BSL TX = P1.1 / pin 35 (nominal; auto-swaps with D6)
 * GPIO 5  (D3)  | PIN_APP_TX  | MSP430 MCU_UARTRXD - normal application UART (P8.3, pin 61)
 * GPIO 10 (D10) | PIN_APP_RX  | MSP430 MCU_UARTTXD - normal application UART (P8.2, pin 60)
 *
 * PIN_TX/PIN_RX carry a UART only during a BSL flashing session (LED lines the
 * rest of the time). PIN_APP_TX/PIN_APP_RX are a dedicated, always-connected pair
 * to the MSP430's running application UART - the web Serial Console and the NTP
 * time-sync packets both use this link. There is one UART peripheral (UART_PORT);
 * enter_bsl()/reset_msp430() swap it between the two pin pairs and between
 * 115200 8N1 and 9600 8E1 framing.
 *
 * UART CONFIGURATION STATES (one peripheral, UART_PORT, time-shared):
 * ------------------------------------------------------------------
 * - App Console Mode (default) : 115200 8N1, routed to PIN_APP_TX/PIN_APP_RX
 * - BSL Flashing Mode          : 9600 8E1, routed to PIN_TX/PIN_RX (BSL session only)
 *
 * WEB SERVER ENDPOINTS:
 * ---------------------
 * - GET  /        : Embedded Web UI (HTML/CSS/JS)
 * - GET  /ws      : WebSocket for serial terminal & hardware commands (CMD:BSL, CMD:RST)
 * - POST /upload  : Handles HTTP POST payload of TI-TXT firmware file
 * =====================================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/temperature_sensor.h"
#include "esp_http_server.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_mac.h"
#include "freertos/semphr.h"
#include "mqtt_client.h"

#define PIN_RST   GPIO_NUM_6  // D4 -> MSP430 RST / NMI          (F5338 pin 96)
#define PIN_TEST  GPIO_NUM_7  // D5 -> MSP430 TEST / SBWTCK      (F5338 pin 91)

// BSL 9600-8E1 data pair. NOMINAL orientation: the ESP32's UART TX must land on
// the MSP430's BSL RX (P1.2 / pin 36) and the ESP32's UART RX on the MSP430's
// BSL TX (P1.1 / pin 35). If the two dupont wires on D6/D7 are reversed you do
// NOT have to re-pin the harness - bsl_orient_data_pins() probes the BSL on the
// nominal pins and, if it gets no answer, automatically retries with this pair
// swapped in software (uart_set_pin) and logs which orientation worked.
#define PIN_TX    GPIO_NUM_21 // D6 -> MSP430 BSL RX (P1.2 / pin 36)  [nominal]
#define PIN_RX    GPIO_NUM_20 // D7 -> MSP430 BSL TX (P1.1 / pin 35)  [nominal]

// Always-on link to the MSP430's normal application UART (P8.2/P8.3). Unlike
// PIN_TX/PIN_RX these are real dedicated wires that stay connected while the gate
// controller firmware runs, so the web console and time sync talk to it here.
#define PIN_APP_TX GPIO_NUM_5  // D3  -> MSP430 MCU_UARTRXD (P8.3, pin 61) - moved off D9 (GPIO9 is a boot strapping pin)
#define PIN_APP_RX GPIO_NUM_10 // D10 -> MSP430 MCU_UARTTXD (P8.2, pin 60)

#define UART_PORT     UART_NUM_1 // sole UART to the MSP430 - time-shared between the
                                 // app console (PIN_APP_*) and BSL flashing (PIN_TX/RX)
#define APP_UART_BAUD 115200     // MSP430 normal application UART baud (8N1)
#define BUF_SIZE  2048

// Wi-Fi, OTA and MQTT credentials live in src/secrets.h, which is gitignored.
// Copy src/secrets.example.h to src/secrets.h and fill it in before building.
// It defines WIFI_STA_SSID / WIFI_STA_PASS, WIFI_AP_SSID / WIFI_AP_PASS,
// OTA_TOKEN, MQTT_URI / MQTT_USERNAME / MQTT_PASSWORD and BRIDGE_CONFIG_URL.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Missing src/secrets.h - copy src/secrets.example.h to src/secrets.h and fill in your Wi-Fi/MQTT settings."
#endif

// ---- Home Assistant / MQTT (see docs/HOMEASSISTANT_MQTT_PLAN.md) -----------
#define MQTT_BASE           "msp430gate"       // <base>/state, <base>/cmd, <base>/availability
#define MQTT_DISC_PREFIX    "homeassistant"    // HA MQTT-discovery prefix
// The gate firmware PUSHES every change now (EVT STATE / EVT BEAM / EVT
// BEAMACT / EVT MOVE_START, plus a full STATUS block every 5 min and on
// demand), so this bridge no longer polls it. What is left:
#define MQTT_TASK_TICK_MS   1000               // housekeeping wake; a pushed line wakes it instantly
#define MQTT_HEARTBEAT_MS   30000              // re-publish state at least this often
#define MQTT_SANITY_POLL_MS 660000             // nothing heard from the gate this long -> ask for one
                                               // STATUS (the firmware's own heartbeat is 5 min)
#define MQTT_LEGACY_POLL_MS 2000               // fallback cadence while the gate firmware is the
                                               // older, non-pushing build (see s_gate_pushes)

// Photo-beam topics. The binary sensor gets its own state topic so an edge is
// one tiny retained publish instead of the whole gate-state JSON.
#define BEAM_T      MQTT_BASE "/beam"          // "ON" / "OFF", retained
#define BEAM_ATTR_T MQTT_BASE "/beam/attr"     // JSON attributes, retained
#define BEAMACT_T   MQTT_BASE "/beamact"       // HA event entity payload, NOT retained
#define BEAMDIAG_T  MQTT_BASE "/diag/beam_edges"  // raw per-edge diagnostics, not an entity
#define BEAM_MIN_PUB_GAP_MS 100                // chatter guard (sun glare / rain / a bug on the lens)
// HISTORY - why there is no poll cadence here any more (2026-09-28).
// This bridge used to ask the MSP430 for STATUS every 8 s at rest and every 1 s
// while moving. That poll interval WAS the latency Home Assistant saw: a move
// started from the RF remote surfaced at a random point inside the 8 s window
// (measured 2.8-5.8 s), and a photo-beam break shorter than the interval either
// vanished or was stretched to one full ~8.1 s period. A 4 s CMD_SETTLE_MS
// window of fast+full polling papered over the HA-command case by also beating
// a race against the firmware's tick order (gate_fsm_Tick ran before
// console_Service, so a queued command was applied one tick late).
// Both are fixed at the source: the firmware now runs console_Service BEFORE
// the FSM (so a command is executed in the tick it arrives) and PUSHES
// EVT STATE / EVT BEAM / EVT BEAMACT / EVT MOVE_START the moment anything
// happens. This task is now purely a publisher - see mqtt_gate_task.

#define WIFI_CONNECTED_BIT BIT0

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

static httpd_handle_t server_handle = NULL;
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "MSP430_FLASHER";
static uint32_t s_last_sync_epoch = 0; // 0 = never synced
static volatile bool s_bsl_active = false;
static volatile bool s_telemetry_enabled = true;
// Serialises the app-UART *writers* (web console, time sync, MQTT commands, the
// STATUS poller). uart_rx_task is the sole reader, so it takes no lock.
static SemaphoreHandle_t s_uart_mux = NULL;
static temperature_sensor_handle_t s_tsens = NULL;   // ESP32-C3 internal die-temp sensor
// Cached by telemetry_task (the sole hardware reader) every 5 s; read elsewhere.
static volatile float s_die_temp_c  = 0.0f;
static volatile bool  s_die_temp_ok = false;

// Byte-level BSL tracing to the web console. Left on - the per-write-block spam
// is suppressed inside send_bsl_cmd_raw() (only failures + non-0x10 commands log).
static volatile bool s_bsl_verbose = true;
// Runtime orientation of the BSL data pair, chosen by bsl_orient_data_pins().
// false = nominal (TX->PIN_TX/D6, RX->PIN_RX/D7); true = the two swapped.
static bool s_bsl_pins_swapped = false;
// BSL UART parity. The MSP430 5xx/6xx ROM BSL is 9600 8E1; kept as a knob only so
// the bring-up matrix can also rule out 8N1. Restored to true after probing.
static bool s_bsl_parity_even = true;

// Forward Declarations
void ws_broadcast(const char *data, size_t len);
void set_uart_mode(bool bsl_mode);

// All non-BSL writes to the shared app UART go through here so the STATUS poller,
// the web console, MQTT commands and time sync never interleave a request. Safe
// to call before s_uart_mux is created (early boot).
static int app_uart_write(const void *buf, size_t len) {
    if (s_uart_mux) xSemaphoreTake(s_uart_mux, portMAX_DELAY);
    int w = uart_write_bytes(UART_PORT, (const char *)buf, len);
    if (s_uart_mux) xSemaphoreGive(s_uart_mux);
    return w;
}

// printf-style line to every web console, prefixed so it's easy to grep in the
// downloaded log. No-op when tracing is disabled.
static void bsl_dbg(const char *fmt, ...) {
    if (!s_bsl_verbose) return;
    char buf[224];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) ws_broadcast(buf, strlen(buf));
}

// Broadcasts a "PROGRESS:<percent>:<label>\n" sentinel line the web UI parses
// into a progress bar + step label, instead of dumping it as raw console text.
static void report_progress(uint8_t percent, const char *label) {
    char msg[96];
    snprintf(msg, sizeof(msg), "PROGRESS:%u:%s\n", (unsigned)percent, label);
    ws_broadcast(msg, strlen(msg));
}

// Routes the single UART_PORT peripheral between the two physical link pairs.
//
//   attach == true  : point UART_PORT at PIN_TX/PIN_RX for a BSL session. Those
//                     two pins double as the EVM's DRV_ENABLE/GUI_STATUS LEDs
//                     while the MSP430 application runs, so the UART is only ever
//                     driven onto them for the duration of a BSL session.
//   attach == false : normal state - park the LED pins as floating inputs (so the
//                     ESP32 doesn't fight the MSP430 firmware's LED drive on
//                     P1.1/P1.2) and point UART_PORT at PIN_APP_TX/PIN_APP_RX,
//                     the dedicated always-on wire pair to the MSP430's running
//                     application UART (P8.2/P8.3).
// Point UART_PORT at the BSL flashing pins in whichever orientation
// s_bsl_pins_swapped currently selects. Safe to call repeatedly.
static void bsl_apply_data_pins(void) {
    gpio_num_t tx = s_bsl_pins_swapped ? PIN_RX : PIN_TX;
    gpio_num_t rx = s_bsl_pins_swapped ? PIN_TX : PIN_RX;
    uart_set_pin(UART_PORT, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void bsl_uart_attach(bool attach) {
    if (attach) {
        bsl_apply_data_pins();
    } else {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << PIN_TX) | (1ULL << PIN_RX);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        uart_set_pin(UART_PORT, PIN_APP_TX, PIN_APP_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
}

// PIN_RST / PIN_TEST double as the MSP430's SBW pins (91/96). Drive them push-pull
// only while we own a BSL session; the rest of the time leave them Hi-Z so an
// MSP-FET / eZ-FET can use JTAG/SBW without the ESP32 fighting it (R29 47k pulls
// RST high, the internal pull-down holds TEST low -> MSP430 runs normally).
static void bsl_pins_drive(void) {
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_INPUT_OUTPUT;   // output + readback for bsl_probe_pad()
    io.pin_bit_mask = (1ULL << PIN_RST) | (1ULL << PIN_TEST);
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io);
}
static void bsl_pins_release(void) {
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_INPUT;          // Hi-Z - hand RST/TEST back to JTAG / the target's own pulls
    io.pin_bit_mask = (1ULL << PIN_RST) | (1ULL << PIN_TEST);
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io);
}

// gpio_get_level() on an INPUT_OUTPUT pin returns the real pad voltage. Drive a
// level, read it back: a mismatch means something else is holding the line.
static int bsl_probe_pad(gpio_num_t pin, int want) {
    int got = gpio_get_level(pin);
    if (got != want) {
        bsl_dbg("[BSL dbg]: *** CONTENTION on GPIO%d: drove %d, pad reads %d - another driver is "
                "holding this line; BSL entry cannot latch.\n", (int)pin, want, got);
    }
    return got;
}

// MSP430 5xx/6xx BSL entry (SLAU319). test_pulses full low->high->low pulses on
// TEST while RST is held low, then TEST left high for the RST release, then TEST
// low. The exact TEST-clock count varies between published host implementations
// (2 vs 3); bsl_bringup() sweeps 1..4 so we don't have to guess. edge_us spaces
// the edges; rst_low_ms is the reset hold. Prints the RST/TEST pad readback.
static void enter_bsl_seq(uint32_t edge_us, uint32_t rst_low_ms, int test_pulses) {
    s_bsl_active = true;
    set_uart_mode(true);
    bsl_uart_attach(true);
    bsl_pins_drive();

    gpio_set_level(PIN_RST, 0);
    gpio_set_level(PIN_TEST, 0);
    vTaskDelay(pdMS_TO_TICKS(rst_low_ms));
    int rl = bsl_probe_pad(PIN_RST, 0);
    int tl = bsl_probe_pad(PIN_TEST, 0);

    for (int i = 0; i < test_pulses; i++) {
        gpio_set_level(PIN_TEST, 1);
        esp_rom_delay_us(edge_us);
        gpio_set_level(PIN_TEST, 0);
        esp_rom_delay_us(edge_us);
    }
    gpio_set_level(PIN_TEST, 1);          // leave TEST high across the RST release
    esp_rom_delay_us(edge_us);
    int th = bsl_probe_pad(PIN_TEST, 1);
    gpio_set_level(PIN_RST, 1);
    esp_rom_delay_us(edge_us);
    int rh = bsl_probe_pad(PIN_RST, 1);
    gpio_set_level(PIN_TEST, 0);
    vTaskDelay(pdMS_TO_TICKS(120));

    bsl_dbg("[BSL dbg]: entry pulsed (%d TEST pulse%s, edge=%uus, rst_low=%ums). "
            "Pad readback RST:L=%d,H=%d TEST:L=%d,H=%d (want L=0 H=1)\n",
            test_pulses, test_pulses == 1 ? "" : "s", (unsigned)edge_us, (unsigned)rst_low_ms,
            rl, rh, tl, th);
}

// Hardware BSL Invocation. Resets the orientation/parity knobs to the MSP430
// 5xx/6xx defaults; the flash / device-info paths call bsl_bringup() right after
// to sweep TEST-pulse count + link framing.
void enter_bsl(void) {
    s_bsl_pins_swapped = false;
    s_bsl_parity_even  = true;
    enter_bsl_seq(400, 25, 2);
}

// Hard Reset Sequence for MSP430. Parks PIN_TX/PIN_RX and PIN_RST/PIN_TEST back
// as Hi-Z inputs (so the running app - and any JTAG/SBW tool - owns those lines),
// reverts UART_PORT to app-console framing and re-routes it to PIN_APP_TX/RX.
void reset_msp430(void) {
    bsl_pins_drive();
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    bsl_uart_attach(false);
    set_uart_mode(false);
    bsl_pins_release();
    s_bsl_active = false;
}

// Switch UART_PORT framing between BSL (9600 8E1) and the app console (APP_UART_BAUD 8N1).
// enter_bsl()/reset_msp430() call this alongside bsl_uart_attach(), which re-routes
// the same peripheral to the matching pin pair.
void set_uart_mode(bool bsl_mode) {
    uart_config_t uart_config = {};
    if (bsl_mode) {
        uart_config.baud_rate = 9600;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity    = s_bsl_parity_even ? UART_PARITY_EVEN : UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
    } else {
        uart_config.baud_rate = APP_UART_BAUD;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity    = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
    }
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    uart_param_config(UART_PORT, &uart_config);
    uart_flush(UART_PORT);
}

// MSP430 5xx/6xx BSL CRC-16. NOT standard CRC-CCITT or CRC-16/ARC - this exact
// non-reflected update function is what TI's BSL core protocol requires
// (verified against the reference host implementation in python-msp430-tools,
// msp430/bsl5/uart.py). Seed 0xFFFF; covers the command byte + payload only,
// never the 0x80 header or length field.
uint16_t calculate_bsl_crc(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        uint8_t x = (uint8_t)((crc >> 8) ^ data[i]);
        x = (uint8_t)(x ^ (x >> 4));
        crc = (uint16_t)((crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ x);
    }
    return crc;
}

// Translates MSP430 5xx/6xx BSL core status response codes into human-readable text
const char* bsl_status_to_string(uint8_t status) {
    switch (status) {
        case 0x00: return "SUCCESS";
        case 0x01: return "Flash write check failed";
        case 0x02: return "Flash fail bit set";
        case 0x03: return "Voltage change during program";
        case 0x04: return "BSL locked (password required)";
        case 0x05: return "BSL password error (incorrect password)";
        case 0x06: return "Byte write forbidden";
        case 0x07: return "Unknown command";
        case 0x08: return "Packet length exceeds buffer";
        default:   return "Unknown BSL error";
    }
}

// Low-level "packet framing accepted" ACK (0x00). A non-zero byte here is a
// framing-level NAK (bad header/CRC/size), separate from whether the command
// itself succeeded.
static bool read_packet_ack(uint32_t timeout_ms, uint8_t *out_ack = NULL) {
    uint8_t ack = 0xFF;
    int len = uart_read_bytes(UART_PORT, &ack, 1, pdMS_TO_TICKS(timeout_ms));
    if (out_ack) *out_ack = (len > 0) ? ack : 0xFF;
    return (len > 0 && ack == 0x00);
}

// Sends a framed BSL command and, on success, copies the response body (the
// bytes between the [0x80][len] header and the trailing CRC) into out_body -
// out_body should be at least 260 bytes. Returns false on any framing/ACK/CRC
// failure. Interpreting the body is caller-specific: most commands use
// body[0] == 0x3B (BSL_MESSAGE) and body[1] == 0x00 (BSL_SUCCESS, see send_bsl_cmd()),
// while data replies like CRC_CHECK start with body[0] == 0x3A (BSL_DATA_REPLY).
static bool send_bsl_cmd_raw(uint8_t cmd, const uint8_t *payload, uint16_t payload_len,
                              uint8_t *out_body, uint16_t *out_len, uint32_t timeout_ms) {
    uint8_t packet[260];
    uint16_t core_len = 1 + payload_len; // Command + Payload

    packet[0] = 0x80;                   // BSL Core Header
    packet[1] = core_len & 0xFF;        // Length LSB
    packet[2] = (core_len >> 8) & 0xFF; // Length MSB
    packet[3] = cmd;                    // Core Command

    if (payload_len > 0 && payload != NULL) {
        memcpy(&packet[4], payload, payload_len);
    }

    uint16_t crc = calculate_bsl_crc(&packet[3], core_len);
    packet[4 + payload_len] = crc & 0xFF;        // CRC LSB
    packet[5 + payload_len] = (crc >> 8) & 0xFF; // CRC MSB

    // Per-write-block (0x10) traffic is not traced on the happy path - it would
    // be hundreds of lines during a real flash. Everything else, and every
    // failure, is logged byte-for-byte.
    bool trace = s_bsl_verbose && (cmd != 0x10);
    int total = 6 + payload_len;

    uart_flush_input(UART_PORT);
    int wrote = uart_write_bytes(UART_PORT, (const char*)packet, total);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
    if (trace) {
        bsl_dbg("[BSL dbg]: -> cmd 0x%02X  (%u payload byte%s, wrote %d/%d, crc 0x%04X)\n",
                cmd, payload_len, payload_len == 1 ? "" : "s", wrote, total, crc);
    }

    uint8_t ack_byte = 0xFF;
    if (!read_packet_ack(timeout_ms, &ack_byte)) {
        if (ack_byte == 0xFF) {
            bsl_dbg("[BSL dbg]: <- cmd 0x%02X: NO reply byte within %u ms. The BSL is not "
                    "responding at all - target not in BSL mode (check TEST wire D5->pin 91), "
                    "wrong/broken data pins, missing common GND, or target unpowered.\n",
                    cmd, (unsigned)timeout_ms);
        } else {
            const char *why = ack_byte == 0x51 ? "header wrong"
                            : ack_byte == 0x52 ? "checksum wrong"
                            : ack_byte == 0x53 ? "packet size 0"
                            : ack_byte == 0x54 ? "packet too big"
                            : ack_byte == 0x56 ? "baud-rate error" : "unknown";
            bsl_dbg("[BSL NAK]: <- cmd 0x%02X: framing NAK 0x%02X (%s) - link is alive but the "
                    "packet was rejected.\n", cmd, ack_byte, why);
        }
        return false;
    }

    uint8_t head[3];
    int hn = uart_read_bytes(UART_PORT, head, 3, pdMS_TO_TICKS(timeout_ms));
    if (hn != 3) {
        bsl_dbg("[BSL dbg]: <- cmd 0x%02X: got ACK 0x00 but response header was short (%d/3 bytes)\n", cmd, hn);
        return false;
    }
    if (head[0] != 0x80) {
        bsl_dbg("[BSL dbg]: <- cmd 0x%02X: bad response header %02X %02X %02X (expected 80 ..). "
                "Looks like line noise / wrong baud / half-duplex echo.\n", cmd, head[0], head[1], head[2]);
        return false;
    }
    uint16_t len = (uint16_t)head[1] | ((uint16_t)head[2] << 8);
    if (len == 0 || len > 260) {
        bsl_dbg("[BSL dbg]: <- cmd 0x%02X: implausible response length %u\n", cmd, len);
        return false;
    }
    uint8_t body[260];
    int bn = uart_read_bytes(UART_PORT, body, len, pdMS_TO_TICKS(timeout_ms));
    if (bn != len) {
        bsl_dbg("[BSL dbg]: <- cmd 0x%02X: response body short (%d/%u bytes)\n", cmd, bn, len);
        return false;
    }
    uint8_t crc_bytes[2];
    if (uart_read_bytes(UART_PORT, crc_bytes, 2, pdMS_TO_TICKS(timeout_ms)) != 2) {
        bsl_dbg("[BSL dbg]: <- cmd 0x%02X: response CRC bytes missing\n", cmd);
        return false;
    }
    uint16_t crc_rx = (uint16_t)crc_bytes[0] | ((uint16_t)crc_bytes[1] << 8);
    uint16_t crc_calc = calculate_bsl_crc(body, len);
    if (crc_calc != crc_rx) {
        bsl_dbg("[BSL dbg]: <- cmd 0x%02X: response CRC mismatch rx=0x%04X calc=0x%04X (framing OK "
                "but data corrupted)\n", cmd, crc_rx, crc_calc);
        return false;
    }

    if (trace) {
        char hex[3 * 24 + 4];
        char *p = hex;
        uint16_t show = len < 24 ? len : 24;
        for (uint16_t k = 0; k < show; k++) p += snprintf(p, 4, "%02X ", body[k]);
        bsl_dbg("[BSL dbg]: <- cmd 0x%02X: OK, %u-byte body: %s%s\n",
                cmd, len, hex, len > show ? "..." : "");
    }

    if (out_body && out_len) {
        memcpy(out_body, body, len);
        *out_len = len;
    }
    return true;
}

// Fire a harmless BSL_VERSION (0x19) query - answered even by a locked/unerased
// BSL - and return true if a well-formed reply came back. Used to sense whether
// the target is actually in BSL and which way round the data pair is wired.
static bool bsl_probe(void) {
    uint8_t body[64];
    uint16_t blen = 0;
    return send_bsl_cmd_raw(0x19, NULL, 0, body, &blen, 200) && blen >= 1;
}

// Probe every plausible link permutation without re-pulsing the entry sequence
// (the MSP430 stays in BSL once it's in): {8E1, 8N1} x {nominal, swapped pair}.
// Returns true and leaves the working combo selected as soon as one answers.
static bool bsl_probe_matrix(void) {
    const bool parity_opts[2] = { true, false };  // 8E1 first (the real default)
    const bool swap_opts[2]   = { false, true };
    for (int p = 0; p < 2; p++) {
        s_bsl_parity_even = parity_opts[p];
        set_uart_mode(true);
        for (int s = 0; s < 2; s++) {
            s_bsl_pins_swapped = swap_opts[s];
            bsl_apply_data_pins();
            uart_flush_input(UART_PORT);
            if (bsl_probe()) {
                bsl_dbg("[BSL]: *** BSL ANSWERED *** framing=%s, data pair=%s. Locked in for this session.\n",
                        s_bsl_parity_even ? "8E1" : "8N1",
                        s_bsl_pins_swapped ? "SWAPPED (swap D6/D7 to make permanent)" : "nominal");
                return true;
            }
        }
    }
    s_bsl_parity_even = true;
    s_bsl_pins_swapped = false;
    set_uart_mode(true);
    bsl_apply_data_pins();
    return false;
}

// ---- Software BSL entry (no RST/TEST pulse) -------------------------------
// The hardware TEST/RST invoke sequence never latched on this board's F5338,
// but the on-chip BSL at 0x1000 is intact. Instead of toggling pins, ask the
// running gate firmware - over the always-on app UART (D3/D10, P8.2/P8.3) - to
// disable interrupts and jump to 0x1000 itself (it handles the "BSL\r\n"
// console command, see GateController_DRV8262/console.c). Then hand our UART to
// the BSL data pair (P1.1/P1.2) and probe. RST and TEST are never touched, so
// this works with the eZ-FET still connected. Returns true if the BSL answered.
static bool bsl_enter_via_software(void) {
    // 1. be on the app-UART pins / app framing, and send the token
    s_bsl_active = false;
    bsl_uart_attach(false);                 // route UART_PORT -> PIN_APP_TX/RX
    set_uart_mode(false);                   // 115200 8N1
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(UART_PORT);
    const char *tok = "BSL\r\n";
    uart_write_bytes(UART_PORT, tok, strlen(tok));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
    bsl_dbg("[SW-BSL]: sent 'BSL' on the app UART - waiting for the gate firmware to jump to 0x1000...\n");

    // 2. let the MSP430 drain its farewell line, __disable_interrupt(), enter BSL
    vTaskDelay(pdMS_TO_TICKS(300));

    // 3. hand the UART to the BSL data pair and probe - NO pin sequence
    s_bsl_active = true;
    s_bsl_pins_swapped = false;
    s_bsl_parity_even  = false;             // F5xx TA-UART BSL default: 9600 8N1
    set_uart_mode(true);
    bsl_uart_attach(true);
    uart_flush_input(UART_PORT);

    if (bsl_probe_matrix()) {
        bsl_dbg("[SW-BSL]: *** BSL ANSWERED via software entry *** - no RST/TEST pulse needed.\n");
        return true;
    }
    bsl_dbg("[SW-BSL]: no answer. Either the gate firmware never got/handled the 'BSL' command on "
            "the app UART (send 'help' - is BSL listed?), or the on-chip BSL isn't executing even "
            "when called directly.\n");
    return false;
}

// ---- Deterministic single-shot BSL entry test (diagnostic) ----------------
// With BSL_CLEAN_TEST defined, bsl_bringup() does exactly ONE canonical 5xx
// entry (RST low 250 ms, two 1 ms TEST pulses, RST released while TEST high,
// TEST low) followed by ONE 0x19 probe at 9600 8N1 on the nominal data pair -
// no pulse-count sweep, no framing/orientation matrix. Run it with the Saleae
// probes on the MSP430 pins themselves (U1 35/36/91/96) so the capture shows
// whether the sequence reaches the die and whether pin 35 (BSL TX) ever answers.
// Comment this line out to restore the full sweep.
#define BSL_CLEAN_TEST

// Full BSL bring-up. Sweeps the TEST-pulse count (1..4 - published host tools
// disagree on 2 vs 3) and, for each, the link framing/orientation matrix
// ({8E1,8N1} x {nominal,swapped}). Then one slow-edge retry. Leaves the working
// combo selected, or reports that the ROM BSL itself is not responding.
static void bsl_bringup(void) {
#ifdef BSL_CLEAN_TEST
    s_bsl_pins_swapped = false;
    s_bsl_parity_even  = false;              // 9600 8N1
    s_bsl_active = true;
    set_uart_mode(true);
    bsl_uart_attach(true);
    bsl_pins_drive();

    gpio_set_level(PIN_RST, 0);
    gpio_set_level(PIN_TEST, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    int rl = bsl_probe_pad(PIN_RST, 0);
    int tl = bsl_probe_pad(PIN_TEST, 0);

    gpio_set_level(PIN_TEST, 1); esp_rom_delay_us(1000);
    gpio_set_level(PIN_TEST, 0); esp_rom_delay_us(1000);   // full TEST pulse 1
    gpio_set_level(PIN_TEST, 1); esp_rom_delay_us(1000);
    gpio_set_level(PIN_TEST, 0); esp_rom_delay_us(1000);   // full TEST pulse 2
    gpio_set_level(PIN_TEST, 1); esp_rom_delay_us(1000);   // hold TEST high...
    int th = bsl_probe_pad(PIN_TEST, 1);
    gpio_set_level(PIN_RST, 1);  esp_rom_delay_us(1000);   // ...release RST while TEST high
    int rh = bsl_probe_pad(PIN_RST, 1);
    gpio_set_level(PIN_TEST, 0);                           // TEST low after RST high
    vTaskDelay(pdMS_TO_TICKS(100));

    bsl_dbg("[BSL clean]: entry done - 2 full TEST pulses, 1 ms edges, rst_low=250 ms. "
            "Pad readback RST:L=%d,H=%d TEST:L=%d,H=%d (want L=0 H=1)\n", rl, rh, tl, th);

    bsl_apply_data_pins();                                 // nominal: TX=D6/P1.2, RX=D7/P1.1
    uart_flush_input(UART_PORT);
    bool ok = bsl_probe();                                 // one 0x19, traced by send_bsl_cmd_raw()
    if (ok) {
        bsl_dbg("[BSL clean]: *** BSL ANSWERED (9600 8N1, nominal pins) *** - entry works; "
                "re-enable the sweep and let it lock in framing.\n");
    } else {
        bsl_dbg("[BSL clean]: no 0x19 reply (9600 8N1, nominal). Check the Saleae on U1 pins "
                "35/36/91/96: did pin 96 go low ~250 ms, pin 91 pulse twice, pin 96 rise while "
                "pin 91 high - and did anything appear on pin 35? If the sequence is at the die "
                "and pin 35 stays silent, prove the BSL with TI's own tool (UniFlash BSL / BSL "
                "Scripter) before touching this code again.\n");
    }
    return;
#else
    for (int tp = 1; tp <= 4; tp++) {
        enter_bsl_seq(400, 30, tp);
        if (bsl_probe_matrix()) {
            bsl_dbg("[BSL]: >>> entered with %d TEST pulse%s. Set enter_bsl() to match.\n",
                    tp, tp == 1 ? "" : "s");
            return;
        }
    }
    bsl_dbg("[BSL]: silent for 1..4 TEST pulses x {8E1,8N1} x {nominal,swap} - one more try, slow edges...\n");
    enter_bsl_seq(3000, 200, 2);
    if (bsl_probe_matrix()) return;

    bsl_dbg("[BSL]: STILL no answer. RST/TEST/data drive is confirmed clean on the scope "
            "(perfect entry waveform, pad readback OK, no contention) and the MSP430 reboots "
            "into its app every time - so this is not wiring and not the entry sequence. The "
            "ROM BSL is not executing. Over JTAG/SBW: (1) read 0x17FC/0x17FE - both 0xAAAA => "
            "BSL DISABLED by signature; (2) dump 0x1000-0x17FF - all 0xFFFF => BSL ERASED "
            "(a full-chip / 'include BSL' JTAG erase does this). Fix over JTAG, then retry.\n");
#endif // BSL_CLEAN_TEST
}

// Send Framed MSP430 5xx BSL Command - true if the target reports success.
// Standard MSP430 5xx BSL responses return BSL_MESSAGE (0x3B) with status 0x00 (BSL_SUCCESS).
// If an error is returned (e.g. 0x04 locked, 0x05 bad password), it logs the specific error name.
bool send_bsl_cmd(uint8_t cmd, const uint8_t *payload, uint16_t payload_len, uint32_t ack_timeout_ms = 200) {
    uint8_t body[260];
    uint16_t len = 0;
    if (!send_bsl_cmd_raw(cmd, payload, payload_len, body, &len, ack_timeout_ms)) {
        return false;
    }
    if (len >= 2 && body[0] == 0x3B) {
        if (body[1] == 0x00) {
            return true;
        }
        char err_msg[128];
        snprintf(err_msg, sizeof(err_msg), "[BSL Error]: Command 0x%02X rejected with status 0x%02X (%s)\n",
                 cmd, body[1], bsl_status_to_string(body[1]));
        ws_broadcast(err_msg, strlen(err_msg));
        return false;
    }
    if (len == 1 && body[0] == 0x00) {
        return true;
    }
    return false;
}

// Erase entire MSP430 flash before reprogramming (BSL_MASS_ERASE = 0x15).
// Required before writing new firmware: flash bits can only go 1->0 without an
// erase, so any address where the old image had a 0 that the new image needs
// as a 1 would silently fail to program without this step. Mass erase can take
// noticeably longer than a normal command to complete, hence the longer timeout.
bool bsl_mass_erase(void) {
    return send_bsl_cmd(0x15, NULL, 0, 3000);
}

// Unlock MSP430 Flash using default 32x 0xFF Password (matches once erased -
// the password lives at 0xFFE0-0xFFFF, which mass erase resets to all-0xFF).
bool bsl_unlock(void) {
    uint8_t pass[32];
    memset(pass, 0xFF, 32);
    return send_bsl_cmd(0x11, pass, 32); // 0x11 = RX Password (0x10 is RX Data Block - easy to mix up)
}

// Write a block of data to MSP430 Flash Memory
bool bsl_write_block(uint32_t addr, const uint8_t *data, uint16_t len) {
    uint8_t payload[250];
    payload[0] = addr & 0xFF;
    payload[1] = (addr >> 8) & 0xFF;
    payload[2] = (addr >> 16) & 0xFF;
    memcpy(&payload[3], data, len);

    return send_bsl_cmd(0x10, payload, len + 3); // 0x10 = RX Data Block
}

// BSL_CRC_CHECK (0x16): asks the target to compute the SAME CRC-16 primitive
// used for packet framing over [addr, addr+len) in flash, so a just-written
// block can be confirmed by comparing against calculate_bsl_crc() run locally
// over the same source bytes - no full readback of flash required.
// MSP430 5xx BSL returns BSL_DATA_REPLY (0x3A) + 2-byte CRC (LSB, MSB).
bool bsl_verify_block(uint32_t addr, const uint8_t *data, uint16_t len) {
    uint8_t payload[5];
    payload[0] = addr & 0xFF;
    payload[1] = (addr >> 8) & 0xFF;
    payload[2] = (addr >> 16) & 0xFF;
    payload[3] = len & 0xFF;
    payload[4] = (len >> 8) & 0xFF;

    uint8_t body[32];
    uint16_t body_len = 0;
    if (!send_bsl_cmd_raw(0x16, payload, sizeof(payload), body, &body_len, 500) || body_len < 3 || body[0] != 0x3A) {
        return false;
    }
    uint16_t device_crc = (uint16_t)body[1] | ((uint16_t)body[2] << 8);
    return device_crc == calculate_bsl_crc(data, len);
}

// Writes one block then immediately verifies it, reporting flashing progress
// (parsed as a percentage of the source file consumed so far) to the web UI.
static bool write_and_verify_block(uint32_t addr, const uint8_t *data, uint16_t len,
                                    size_t parsed, size_t total) {
    if (!bsl_write_block(addr, data, len)) {
        const char *err = "[BSL Error]: Write failed!\n";
        ws_broadcast(err, strlen(err));
        return false;
    }
    if (!bsl_verify_block(addr, data, len)) {
        const char *err = "[BSL Error]: Verification mismatch - written data does not match the source file!\n";
        ws_broadcast(err, strlen(err));
        return false;
    }
    char label[48];
    snprintf(label, sizeof(label), "Wrote & verified 0x%06lX", (unsigned long)addr);
    uint8_t percent = (uint8_t)(15 + (total ? (parsed * 75) / total : 0));
    if (percent > 90) percent = 90;
    report_progress(percent, label);
    return true;
}

// TX_BSL_VERSION (0x19) + TX_DATA_BLOCK (0x18) over the fixed TLV device-
// descriptor region. Reports raw bytes rather than decoding a part number:
// the descriptor's device-ID field must be cross-referenced against Table 60
// in TI's SLAU208 (MSP430x5xx/6xx Family User's Guide) or the device-specific
// datasheet, which varies per silicon revision and isn't guessed at here.
void bsl_read_device_info(void) {
    const char *start_msg = "[System]: Invoking BSL mode to query MSP430 Device Info...\n";
    ws_broadcast(start_msg, strlen(start_msg));

    bsl_bringup();

    char msg[256];
    uint8_t body[260];
    uint16_t body_len = 0;

    if (send_bsl_cmd_raw(0x19, NULL, 0, body, &body_len, 500) && body_len >= 5 && body[0] == 0x3A) {
        snprintf(msg, sizeof(msg),
            "[Device Info]: BSL version response: %02X %02X %02X %02X (Vendor: 0x%02X, CI: 0x%02X, API: 0x%02X, PI: 0x%02X)\n",
            body[1], body[2], body[3], body[4], body[1], body[2], body[3], body[4]);
        ws_broadcast(msg, strlen(msg));
    } else {
        const char *err = "[Device Info]: BSL version query timed out (no ACK/reply). Target MSP430 may be disconnected, unpowered, or pins (RST/TEST/RX/TX) not wired.\n";
        ws_broadcast(err, strlen(err));
    }

    uint8_t desc_payload[5] = { 0x04, 0x1A, 0x00, 16, 0 }; // read 16 bytes @ 0x1A04 (TLV descriptor)
    if (send_bsl_cmd_raw(0x18, desc_payload, sizeof(desc_payload), body, &body_len, 500) && body_len > 1 && body[0] == 0x3A) {
        char hex[3 * 32 + 1] = {0};
        char *p = hex;
        for (uint16_t k = 1; k < body_len; k++) { // body[0] is 0x3A (BSL_DATA_REPLY)
            p += snprintf(p, 4, "%02X ", body[k]);
        }
        snprintf(msg, sizeof(msg), "[Device Info]: TLV descriptor @0x1A04: %s\n", hex);
        ws_broadcast(msg, strlen(msg));
    } else {
        const char *err = "[Device Info]: TLV descriptor read timed out (no ACK/reply).\n";
        ws_broadcast(err, strlen(err));
    }

    const char *known = "[Device Info]: Configured project target (GateController_DRV8262): "
                         "MSP430F5338 - Flash 0x8000-0xFFFF + 0x10000-0x47FF7 (~256 KB), RAM 0x1C00-0x63FF (18 KB)\n";
    ws_broadcast(known, strlen(known));

    reset_msp430();
    const char *done_msg = "[System]: MSP430 target rebooted back to normal mode.\n";
    ws_broadcast(done_msg, strlen(done_msg));
}

// Parse TI-TXT Firmware File and Program MSP430
bool flash_ti_txt(const char *txt_data, size_t length) {
    // Software entry first (the RST/TEST hardware invoke never latched on this
    // board). Only if the running app can't hand us to the BSL do we fall back
    // to pulsing the pins.
    if (!bsl_enter_via_software()) {
        const char *fb = "[BSL]: software entry got no answer - falling back to the RST/TEST pin sequence...\n";
        ws_broadcast(fb, strlen(fb));
        bsl_bringup();
    }
    report_progress(2, "Erasing Flash...");

    const char *log0 = "[BSL]: Erasing MSP430 Flash...\n";
    ws_broadcast(log0, strlen(log0));

    if (!bsl_mass_erase()) {
        const char *err = "[BSL Error]: Mass erase failed!\n";
        ws_broadcast(err, strlen(err));
        reset_msp430();
        report_progress(0, "Failed");
        return false;
    }

    report_progress(10, "Unlocking BSL...");
    const char *log1 = "[BSL]: Unlocking MSP430 Flash...\n";
    ws_broadcast(log1, strlen(log1));

    if (!bsl_unlock()) {
        const char *err = "[BSL Error]: Password unlock failed!\n";
        ws_broadcast(err, strlen(err));
        reset_msp430();
        report_progress(0, "Failed");
        return false;
    }

    report_progress(15, "Flashing...");
    const char *log2 = "[BSL]: Password accepted. Flashing TI-TXT image...\n";
    ws_broadcast(log2, strlen(log2));

    uint32_t current_addr = 0;
    uint8_t block_buf[128];
    uint16_t block_len = 0;

    size_t i = 0;
    while (i < length) {
        while (i < length && isspace((unsigned char)txt_data[i])) i++;
        if (i >= length) break;

        if (txt_data[i] == '@') {
            if (block_len > 0) {
                if (!write_and_verify_block(current_addr, block_buf, block_len, i, length)) {
                    reset_msp430();
                    report_progress(0, "Failed");
                    return false;
                }
                current_addr += block_len;
                block_len = 0;
            }
            i++;
            char addr_str[9] = {0};
            int a_idx = 0;
            while (i < length && isxdigit((unsigned char)txt_data[i]) && a_idx < 8) {
                addr_str[a_idx++] = txt_data[i++];
            }
            current_addr = strtoul(addr_str, NULL, 16);
        } else if (txt_data[i] == 'q' || txt_data[i] == 'Q') {
            break;
        } else if (isxdigit((unsigned char)txt_data[i])) {
            char hex_str[3] = {txt_data[i], txt_data[i+1], '\0'};
            i += 2;
            block_buf[block_len++] = (uint8_t)strtoul(hex_str, NULL, 16);

            if (block_len >= 128) {
                if (!write_and_verify_block(current_addr, block_buf, block_len, i, length)) {
                    reset_msp430();
                    report_progress(0, "Failed");
                    return false;
                }
                current_addr += block_len;
                block_len = 0;
            }
        } else {
            i++;
        }
    }

    if (block_len > 0) {
        if (!write_and_verify_block(current_addr, block_buf, block_len, length, length)) {
            reset_msp430();
            report_progress(0, "Failed");
            return false;
        }
    }

    report_progress(95, "Resetting MSP430...");
    reset_msp430();
    report_progress(100, "Complete!");
    return true;
}

// ===== Wireless firmware self-update over the app UART ===================== //
// The BSL on this F5338 is dead, so the new image is streamed to the running
// gate firmware over the always-on app UART (D3/D10) and the MSP430 burns it
// into its own main flash from a RAM-resident routine (see the FWUPDATE command
// in GateController_DRV8262/console.c). The full TI-TXT is already buffered in
// ESP32 RAM by upload_post_handler before this runs, so a flaky Wi-Fi link
// during the burn cannot corrupt anything - only the HTTP upload itself needs
// Wi-Fi, and that completes first.
#define FWU_READY   0xCC
#define FWU_SOF     0xA5
#define FWU_COMMIT  0x5B
#define FWU_ACK     0x06
#define FWU_NAK     0x15
#define FWU_MAX_PAY 128

static void fwu_ws(const char *s) { ws_broadcast(s, strlen(s)); }

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection) - byte-for-byte
// identical to fwu_crc16() in the gate firmware.
static uint16_t fwu_crc16(const uint8_t *p, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// One framed write + wait for the gate updater's ACK, with resend on NAK/timeout.
static bool fwu_send_frame(uint16_t addr, const uint8_t *data, uint8_t len) {
    uint8_t f[1 + 3 + FWU_MAX_PAY + 2];
    f[0] = FWU_SOF;
    f[1] = (uint8_t)(addr & 0xFF);
    f[2] = (uint8_t)(addr >> 8);
    f[3] = len;
    for (uint8_t k = 0; k < len; k++) f[4 + k] = data[k];
    uint16_t crc = fwu_crc16(&f[1], (size_t)(3 + len));   // over addrLo,addrHi,len,data
    f[4 + len]     = (uint8_t)(crc & 0xFF);
    f[4 + len + 1] = (uint8_t)(crc >> 8);
    int total = 1 + 3 + len + 2;

    for (int attempt = 0; attempt < 6; attempt++) {
        uart_flush_input(UART_PORT);
        uart_write_bytes(UART_PORT, (const char *)f, total);
        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(300));
        uint8_t ack = 0;
        int r = uart_read_bytes(UART_PORT, &ack, 1, pdMS_TO_TICKS(1500));
        if (r == 1 && ack == FWU_ACK) return true;
        if (r == 1 && ack != FWU_NAK) {
            char m[112];
            const char *why = ack == 0xE1 ? " (gate BOR: flash-erase wait timed out - slow/failing flash)"
                            : ack == 0xE2 ? " (gate BOR: UART wait timed out)"
                            : "";
            snprintf(m, sizeof(m), "[FWU]: frame @0x%04X got 0x%02X, not ACK%s\n",
                     (unsigned)addr, ack, why);
            fwu_ws(m);
        }
        // NAK, unexpected byte, or timeout -> resend
    }
    return false;
}

static void fwu_restore_relay(void) {
    set_uart_mode(false);
    bsl_uart_attach(false);
    s_bsl_active = false;
}

// Stream an already-buffered TI-TXT image to the gate firmware's RAM updater.
bool flash_via_app_uart(const char *txt, size_t length) {
    // Take the app UART, pause the console relay, let any in-flight relay read
    // (<=50 ms in uart_rx_task) drain, then flush.
    s_bsl_active = true;
    bsl_uart_attach(false);
    set_uart_mode(false);
    vTaskDelay(pdMS_TO_TICKS(60));
    uart_flush_input(UART_PORT);

    report_progress(2, "Arming gate updater...");
    const char *cmd = "FWUPDATE\r\n";
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));

    // Wait for the 0xCC ready marker, ignoring the app's ASCII ack text. Capture
    // whatever the gate does send so a failure is diagnosable in one shot.
    bool ready = false;
    uint8_t seen[128];
    int seen_n = 0;
    int64_t t0 = esp_timer_get_time();
    while ((esp_timer_get_time() - t0) < 3000000) {   // 3 s
        uint8_t b = 0;
        if (uart_read_bytes(UART_PORT, &b, 1, pdMS_TO_TICKS(200)) == 1) {
            if (seen_n < (int)sizeof(seen)) seen[seen_n++] = b;
            if (b == FWU_READY) { ready = true; break; }
        }
    }
    if (!ready) {
        char dump[3 * 96 + 64];
        int p = snprintf(dump, sizeof(dump),
                         "[FWU Error]: no 0x%02X ready marker. Gate sent %d byte(s): ", FWU_READY, seen_n);
        for (int k = 0; k < seen_n && p < (int)sizeof(dump) - 4; k++)
            p += snprintf(dump + p, sizeof(dump) - p, "%02X ", seen[k]);
        snprintf(dump + p, sizeof(dump) - p, "\n");
        fwu_ws(dump);
        fwu_ws("[FWU Error]: 0 bytes -> gate never saw FWUPDATE / not on app UART. ASCII text ending "
               "with a reboot banner -> the RAM updater crashed. Nothing -> re-check the FWUPDATE "
               "command is in the running firmware ('help').\n");
        fwu_restore_relay();
        report_progress(0, "Failed");
        return false;
    }
    fwu_ws("[FWU]: gate RAM updater ready - streaming image (main flash 0x8000-0xFFFF only)...\n");

    // Parse TI-TXT -> ascending, <=128-byte frames that never cross a 128-byte
    // boundary (=> never cross a 512-byte flash segment), main flash only.
    uint32_t cur_addr = 0;
    uint8_t  buf[FWU_MAX_PAY];
    uint16_t buf_len  = 0;
    uint32_t buf_base = 0;
    int      last_pct = -1;
    bool     ok = true;
    size_t   i = 0;

    while (ok && i < length) {
        while (i < length && isspace((unsigned char)txt[i])) i++;
        if (i >= length) break;

        if (txt[i] == '@') {
            if (buf_len > 0) { ok = fwu_send_frame((uint16_t)buf_base, buf, (uint8_t)buf_len); buf_len = 0; }
            i++;
            char as[9] = {0};
            int a = 0;
            while (i < length && isxdigit((unsigned char)txt[i]) && a < 8) as[a++] = txt[i++];
            cur_addr = strtoul(as, NULL, 16);
        } else if (txt[i] == 'q' || txt[i] == 'Q') {
            break;
        } else if (isxdigit((unsigned char)txt[i]) && (i + 1) < length && isxdigit((unsigned char)txt[i + 1])) {
            char hs[3] = { txt[i], txt[i + 1], 0 };
            i += 2;
            uint8_t byte = (uint8_t)strtoul(hs, NULL, 16);

            if (cur_addr >= 0x8000 && cur_addr <= 0xFFFF) {
                if (buf_len > 0 &&
                    (cur_addr != buf_base + buf_len ||
                     (cur_addr & 0x7F) == 0 ||
                     buf_len >= FWU_MAX_PAY)) {
                    ok = fwu_send_frame((uint16_t)buf_base, buf, (uint8_t)buf_len);
                    buf_len = 0;
                }
                if (buf_len == 0) buf_base = cur_addr;
                buf[buf_len++] = byte;
            } else if (buf_len > 0) {
                ok = fwu_send_frame((uint16_t)buf_base, buf, (uint8_t)buf_len);
                buf_len = 0;
            }
            cur_addr++;

            int pct = (int)(5 + (i * 85) / (length ? length : 1));
            if (pct > 90) pct = 90;
            if (pct != last_pct) { report_progress((uint8_t)pct, "Streaming to gate..."); last_pct = pct; }
        } else {
            i++;
        }
    }
    if (ok && buf_len > 0) ok = fwu_send_frame((uint16_t)buf_base, buf, (uint8_t)buf_len);

    if (!ok) {
        fwu_ws("[FWU Error]: a frame kept failing - transfer aborted. The gate reverts to the "
               "previous firmware in ~5 s (nothing was committed).\n");
        fwu_restore_relay();
        report_progress(0, "Failed");
        return false;
    }

    // Commit: MSP430 ACKs then software-BORs into the new image.
    report_progress(95, "Committing...");
    uint8_t c = FWU_COMMIT;
    uart_flush_input(UART_PORT);
    uart_write_bytes(UART_PORT, (const char *)&c, 1);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
    uint8_t ack = 0;
    bool committed = (uart_read_bytes(UART_PORT, &ack, 1, pdMS_TO_TICKS(2000)) == 1 && ack == FWU_ACK);

    fwu_restore_relay();   // the gate is rebooting; resume the normal relay

    if (committed) {
        report_progress(100, "Complete!");
        fwu_ws("[FWU]: image committed - MSP430 rebooting into the new firmware.\n");
    } else {
        report_progress(100, "Done (commit unconfirmed)");
        fwu_ws("[FWU Warn]: every frame verified but no ACK for the commit byte. If the gate came "
               "back on the new version it worked; if not, retry.\n");
    }
    return true;
}

// The UART transfer blocks for several seconds. Run it on its own task so the
// web server task stays free to actually push the PROGRESS WebSocket frames
// while it runs (otherwise the bar only updates once, at the very end).
static volatile bool s_fwu_busy = false;
struct fwu_job_t { char *buf; size_t len; };

static void fwu_worker_task(void *arg) {
    fwu_job_t *job = (fwu_job_t *)arg;
    bool ok = flash_via_app_uart(job->buf, job->len);
    free(job->buf);
    free(job);
    ws_broadcast(ok ? "[System]: MSP430 firmware update finished.\n"
                    : "[System Error]: MSP430 firmware update failed.\n",
                 strlen(ok ? "[System]: MSP430 firmware update finished.\n"
                           : "[System Error]: MSP430 firmware update failed.\n"));
    s_fwu_busy = false;
    vTaskDelete(NULL);
}

// Builds and sends one "TIME:<epoch>\n" packet, shared by the 30-minute
// scheduled sync (time_sync_task) and the manual "Force Time Sync" hardware
// action (CMD:SYNC in ws_handler). Goes out UART_PORT, which in normal operation
// is routed to the MSP430's application UART (PIN_APP_TX/PIN_APP_RX), so the
// timestamp reaches the running firmware - unless a BSL session currently owns
// UART_PORT on the flashing pins (see bsl_uart_attach()), in which case it is
// skipped rather than injected as noise into the BSL byte stream.
void do_time_sync(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (s_bsl_active) {
        const char *err = "[Time Sync]: Skipped - a BSL session currently owns the UART.\n";
        ws_broadcast(err, strlen(err));
        return;
    }

    if (timeinfo.tm_year <= (2020 - 1900)) {
        const char *err = "[Time Sync]: Skipped - ESP32 clock isn't set yet (NTP hasn't synced).\n";
        ws_broadcast(err, strlen(err));
        return;
    }

    char time_buf[32];
    snprintf(time_buf, sizeof(time_buf), "TIME:%ld\n", (long)now);
    app_uart_write(time_buf, strlen(time_buf));

    // localtime_r() above is UTC (no TZ is set anywhere in this file) - call
    // setenv("TZ", ...) + tzset() before this runs if you want local time.
    char human_time[32];
    strftime(human_time, sizeof(human_time), "%b %d %Y %I:%M:%S %p", &timeinfo);
    s_last_sync_epoch = (uint32_t)now;

    char console_msg[96];
    snprintf(console_msg, sizeof(console_msg), "[Time Sync]: Sent timestamp to MSP430 (%s UTC)\n", human_time);
    ws_broadcast(console_msg, strlen(console_msg));

    char synced_msg[64];
    snprintf(synced_msg, sizeof(synced_msg), "SYNCED:%s UTC\n", human_time);
    ws_broadcast(synced_msg, strlen(synced_msg));
}

// SNTP Time Sync Task (Sends periodic timestamp to MSP430)
void time_sync_task(void *pvParameters) {
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    while (1) {
        do_time_sync();
        // Wait 30 minutes between syncs
        vTaskDelay(pdMS_TO_TICKS(30 * 60 * 1000));
    }
}

// Broadcasts "TELEMETRY:<rssi>:<qual>:<ip>:<uptime_s>:<heap_bytes>:<net_mode>\n"
// every 5s for the web UI's status footer. <net_mode> is a colon-free human
// string ("STA <ssid>" / "Fallback AP - retrying <ssid>"). Self-contained
// - doesn't touch the MSP430 link at all.
void telemetry_task(void *pvParameters) {
    while (1) {
        // Refresh the die-temp cache every loop (sole hardware reader) - keep
        // doing it even when the web telemetry stream is paused, so MQTT still
        // has a fresh value.
        {
            float tc;
            s_die_temp_ok = (s_tsens && temperature_sensor_get_celsius(s_tsens, &tc) == ESP_OK);
            if (s_die_temp_ok) s_die_temp_c = tc;
        }

        if (!s_telemetry_enabled) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int rssi = 0;
        char net_mode[48];
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            rssi = ap_info.rssi;
            snprintf(net_mode, sizeof(net_mode), "STA %s", (const char *)ap_info.ssid);
        } else {
            wifi_mode_t wmode = WIFI_MODE_NULL;
            esp_wifi_get_mode(&wmode);
            if (wmode == WIFI_MODE_AP || wmode == WIFI_MODE_APSTA) {
                snprintf(net_mode, sizeof(net_mode), "Fallback AP - retrying %s", WIFI_STA_SSID);
            } else {
                snprintf(net_mode, sizeof(net_mode), "connecting to %s", WIFI_STA_SSID);
            }
        }

        const char *qual = "N/A";
        if (rssi != 0) {
            if (rssi >= -50) qual = "Very Strong";
            else if (rssi >= -65) qual = "Strong";
            else if (rssi >= -75) qual = "Medium";
            else if (rssi >= -85) qual = "Weak";
            else qual = "Very Weak";
        } else {
            qual = "AP Mode";
        }

        esp_netif_ip_info_t ip_info = {};
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (!netif || esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
            netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
            if (netif) {
                esp_netif_get_ip_info(netif, &ip_info);
            }
        }
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u",
                 (unsigned)(ip_info.ip.addr & 0xFF), (unsigned)((ip_info.ip.addr >> 8) & 0xFF),
                 (unsigned)((ip_info.ip.addr >> 16) & 0xFF), (unsigned)((ip_info.ip.addr >> 24) & 0xFF));

        uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        uint32_t heap = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT);
        char temp_str[12] = "-";
        if (s_die_temp_ok) snprintf(temp_str, sizeof(temp_str), "%.1f", (double)s_die_temp_c);

        // TELEMETRY:<rssi>:<qual>:<ip>:<uptime>:<heap>:<temp_c>:<net_mode>  (net_mode last = colon-free "rest")
        char msg[176];
        snprintf(msg, sizeof(msg), "TELEMETRY:%d:%s:%s:%lu:%lu:%s:%s\n",
                 rssi, qual, ip_str, (unsigned long)uptime_s, (unsigned long)heap, temp_str, net_mode);
        ws_broadcast(msg, strlen(msg));

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ======================= Home Assistant / MQTT bridge ====================== //
// One esp-mqtt client. mqtt_gate_task periodically writes "STATUS;" to the gate
// (the MSP430 firmware terminates each reply with ';'); the shared uart_rx_task
// parses the reply into g_gate_status and nudges this task, which publishes a
// retained JSON state plus HA MQTT-discovery configs and handles <base>/cmd
// (OPEN/CLOSE/TOGGLE/STOP). Everything is skipped while a BSL/flash/OTA session
// owns the UART. See docs/HOMEASSISTANT_MQTT_PLAN.md.

typedef struct {
    int  state;          // gate_state_t index; may be out of range if firmware grows the enum
    int  open_limit;     // 0 / 1 / -1 (unknown)
    int  closed_limit;
    int  beam_broken;
    int  nfault;
    int  current_ma;
    int  vm_mv;
    int  stall_count;
    int  driver_asleep;
    char last_dir[8];    // "OPEN" / "CLOSE" / ""
    char lockout[80];    // reason text; "" when not locked out
    char cmd_src[20];    // "ha" / "remote" / "wired_button" / "auto_reverse" / ...
} gate_status_t;

static const char *const GATE_STATE_NAMES[] = {
    "ST_INIT","ST_IDLE_OPEN","ST_IDLE_CLOSED","ST_OPENING","ST_CLOSING",
    "ST_SOFT_STOP","ST_STOPPED_MIDTRAVEL","ST_OBSTACLE_REVERSE","ST_HOMING",
    "ST_UNKNOWN","ST_STALL_FAULT","ST_DRIVER_FAULT","ST_LOCKOUT","ST_LOW_BATTERY"
};
#define GATE_STATE_COUNT ((int)(sizeof(GATE_STATE_NAMES) / sizeof(GATE_STATE_NAMES[0])))

// state=-1 and limits=-1 read as "unknown" until the first STATUS reply.
static gate_status_t            g_gate_status = { -1, -1, -1, -1, -1, 0, 0, 0, 0, "", "", "" };
static SemaphoreHandle_t        s_status_mux  = NULL;   // guards g_gate_status
static TaskHandle_t             s_mqtt_task   = NULL;
static esp_mqtt_client_handle_t s_mqtt        = NULL;
static volatile bool            s_mqtt_up     = false;
static volatile bool            s_disc_dirty  = false;  // set when discovery needs re-publishing (e.g. MSP430 version learned)
static char                     s_dev_id[24]  = MQTT_BASE;   // "msp430gate_<mac6>"
static char                     s_msp_version[64] = "";      // last "VERSION ..." line seen from the gate firmware

// Parsed from the two-line arrival message: "Gate: reached <OPEN|CLOSED> limit -
// travel time <n> ms, avg <n> mA, max <n> mA" + "Peak ramp-up current: <n> mA".
typedef struct {
    char     dir[8];       // "OPEN" / "CLOSED"
    int      travel_ms;
    int      avg_ma;
    int      max_ma;
    int      peak_ma;
    uint32_t cycles;       // running count of completed moves (persisted in NVS)
    bool     valid;
} lastmove_t;
static lastmove_t     s_lastmove = { "", 0, 0, 0, 0, 0, false };
static volatile bool  s_lastmove_dirty = false;
static uint32_t       s_cycle_count = 0;

static void cycles_load(void) {
    nvs_handle_t h;
    if (nvs_open("gatestats", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, "cycles", &s_cycle_count);
        nvs_close(h);
    }
}
static void cycles_save(void) {
    nvs_handle_t h;
    if (nvs_open("gatestats", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "cycles", s_cycle_count);
        nvs_commit(h);
        nvs_close(h);
    }
}

static const char *gate_state_name(int s) {
    return (s >= 0 && s < GATE_STATE_COUNT) ? GATE_STATE_NAMES[s] : "UNKNOWN";
}
// HA cover entities accept only open/closed/opening/closing/stopped - everything
// that is not a clean powered move or a settled limit reports "stopped".
static const char *gate_cover_state(int s) {
    switch (s) {
        case 1:  return "open";
        case 2:  return "closed";
        case 3:  return "opening";
        case 4:  return "closing";
        default: return "stopped";
    }
}
static bool gate_is_moving(int s) { return s == 3 || s == 4 || s == 5 || s == 7 || s == 8; }

// Digit right after the next '=' following `anchor` (skips spaces). -1 if none.
static int digit_after_eq(const char *anchor) {
    if (!anchor) return -1;
    const char *e = strchr(anchor, '=');
    if (!e) return -1;
    for (e++; *e == ' '; e++) { }
    return (*e >= '0' && *e <= '9') ? (*e - '0') : -1;
}
// Signed int after `key`, skipping spaces / '=' / ':'. false if not found.
static bool int_after(const char *buf, const char *key, int *out) {
    const char *p = strstr(buf, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == '=' || *p == ':') p++;
    if (*p != '-' && (*p < '0' || *p > '9')) return false;
    *out = atoi(p);
    return true;
}

// Unsigned 32-bit after `key` (MSP430 uptime ms overflows a signed int after
// ~24 days, so atoi() is not good enough for the t= fields).
static bool u32_after(const char *buf, const char *key, uint32_t *out) {
    const char *p = strstr(buf, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == '=' || *p == ':') p++;
    if (*p < '0' || *p > '9') return false;
    *out = (uint32_t)strtoul(p, NULL, 10);
    return true;
}

// Copy the token after `key` up to the first space / CR / LF / ';'. `allow`
// is a predicate on each character - anything it rejects ends the token, so a
// malformed line can never inject arbitrary text into a JSON payload.
static void token_after(const char *buf, const char *key, char *out, int outsz,
                        bool (*allow)(char)) {
    out[0] = 0;
    const char *p = strstr(buf, key);
    if (!p) return;
    p += strlen(key);
    int i = 0;
    while (i < outsz - 1 && p[i] && allow(p[i])) { out[i] = p[i]; i++; }
    out[i] = 0;
}
static bool tok_word(char c)  { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; }
static bool tok_list(char c)  { return (c >= '0' && c <= '9') || c == ','; }

// Parse a STATUS reply (may carry "[<epoch>] " line prefixes and a trailing ';').
// True only if it really is a status block.
static bool parse_gate_status(const char *b, gate_status_t *o) {
    int st, vm, v;
    if (!int_after(b, "State=", &st)) return false;
    if (!int_after(b, "VM:", &vm))    return false;

    memset(o, 0, sizeof(*o));
    o->state        = st;
    o->vm_mv        = vm;
    o->open_limit   = digit_after_eq(strstr(b, "P7.5"));
    o->closed_limit = digit_after_eq(strstr(b, "P7.6"));
    o->beam_broken  = digit_after_eq(strstr(b, "photo beam broken"));
    o->nfault       = digit_after_eq(strstr(b, "nFAULT"));
    o->current_ma    = int_after(b, "Current:", &v)      ? v : 0;
    o->stall_count   = int_after(b, "Stall count:", &v)  ? v : 0;
    o->driver_asleep = int_after(b, "Driver asleep:", &v) ? v : 0;

    const char *d = strstr(b, "lastDir=");
    if (d) {
        d += 8;
        int i = 0;
        while (i < 7 && ((d[i] >= 'A' && d[i] <= 'Z') || (d[i] >= 'a' && d[i] <= 'z'))) { o->last_dir[i] = d[i]; i++; }
        o->last_dir[i] = 0;
    }
    // "Uptime: <n> ms  src=<source>  beam10s=<n>  lastbreak=<n> ms" (firmware >= 2026-09-28)
    token_after(b, "src=", o->cmd_src, (int)sizeof(o->cmd_src), tok_word);

    const char *lo = strstr(b, "LOCKED OUT:");
    if (lo) {
        lo += 11;
        while (*lo == ' ') lo++;
        int i = 0;
        while (i < (int)sizeof(o->lockout) - 1 && lo[i] && lo[i] != '\r' && lo[i] != '\n' && lo[i] != ';') { o->lockout[i] = lo[i]; i++; }
        while (i > 0 && o->lockout[i - 1] == ' ') i--;
        o->lockout[i] = 0;
    } else if (strstr(b, "Both limit switches read ACTIVE")) {
        // Not locked out yet, but both limit inputs read active - surface it early
        // (STATUS prints this conditional line just before it would lock out).
        strcpy(o->lockout, "both limit switches active - will lock out");
    }
    return true;
}

static bool gate_status_equal(const gate_status_t *a, const gate_status_t *c) {
    return a->state == c->state && a->open_limit == c->open_limit && a->closed_limit == c->closed_limit &&
           a->beam_broken == c->beam_broken && a->nfault == c->nfault && a->current_ma == c->current_ma &&
           a->vm_mv == c->vm_mv && a->stall_count == c->stall_count && a->driver_asleep == c->driver_asleep &&
           strcmp(a->last_dir, c->last_dir) == 0 && strcmp(a->lockout, c->lockout) == 0 &&
           strcmp(a->cmd_src, c->cmd_src) == 0;
}

// ===================== pushed-event handling (EVT lines) ==================== //
// The gate firmware emits these the moment something happens, so the bridge
// publishes straight from the parse - no batching, no poll to wait for. These
// run in uart_rx_task context; esp_mqtt_client_publish is thread-safe and the
// payloads here are small, which is why the beam path publishes inline rather
// than handing off to mqtt_gate_task (that hop is what a beam edge cannot
// afford). Gate-state changes DO hand off, because their JSON is much larger.

static volatile int      s_beam_blocked      = -1;   // -1 unknown / 0 clear / 1 blocked
static volatile int      s_beam_last_break   = 0;    // ms, most recent completed break
static volatile uint32_t s_beam_last_edge_ms = 0;    // MSP430 uptime ms at the most recent edge
static volatile int      s_beam_breaks10     = 0;    // rolling count, last 10 s
static volatile bool     s_beam_chatter      = false;
static int64_t           s_beam_last_pub_us  = 0;
static volatile int64_t  s_last_gate_msg_us  = 0;    // any recognised message from the gate
static volatile bool     s_status_req_pending = true; // ask for a full STATUS (boot / MQTT reconnect)
static volatile int64_t  s_status_req_at_us   = 0;    // ...but not before this time (0 = now)

// Does the attached gate firmware push events, or does it still need polling?
// Decided from its own STATUS reply: the pushing build prints an "Uptime: ...
// src=... beam10s=..." line, the older one does not. Starts false so that a
// bridge updated ahead of the MSP430 (or an MSP430 reflash that rolled back)
// keeps Home Assistant live at MQTT_LEGACY_POLL_MS instead of going silent.
// Set true by any EVT line as well, since only the new build emits those.
static volatile bool     s_gate_pushes = false;

// Publish the beam binary sensor + its JSON attributes (both retained, so an HA
// restart picks the last values straight off the broker).
static void mqtt_publish_beam(void) {
    if (!s_mqtt || s_beam_blocked < 0) return;
    esp_mqtt_client_publish(s_mqtt, BEAM_T, s_beam_blocked ? "ON" : "OFF", 0, 1, 1);
    char a[160];
    int n = snprintf(a, sizeof(a),
        "{\"last_break_ms\":%d,\"last_edge_uptime_ms\":%lu,\"breaks_last_10s\":%d,\"chatter\":%s}",
        s_beam_last_break, (unsigned long)s_beam_last_edge_ms, s_beam_breaks10,
        s_beam_chatter ? "true" : "false");
    if (n > 0 && n < (int)sizeof(a)) esp_mqtt_client_publish(s_mqtt, BEAM_ATTR_T, a, 0, 1, 1);
}

// Every message that carries a complete gate state (EVT STATE and the full
// STATUS block) also carries the beam level. Use it to seed the binary sensor
// before the first edge ever arrives, and to resync it afterwards - otherwise a
// bridge that reboots while the beam is quiet would have nothing to publish.
// Edges are never routed through here, so the chatter guard is not bypassed.
static void beam_sync_from_state(int bm) {
    if (bm < 0 || bm == s_beam_blocked) return;
    s_beam_blocked = bm ? 1 : 0;
    mqtt_publish_beam();
}

// "EVT BEAM b=<0|1> t=<uptime_ms> dur=<ms> n10=<n> [chat=1]"
static void handle_beam_edge(const char *p) {
    int      b = 0, dur = 0, n10 = 0;
    uint32_t t = 0;
    int_after(p, " b=", &b);
    int_after(p, " dur=", &dur);
    int_after(p, " n10=", &n10);
    u32_after(p, " t=", &t);
    bool chatter = (strstr(p, "chat=1") != NULL);

    s_beam_last_edge_ms = t;
    if (dur > 0) s_beam_last_break = dur;
    s_beam_breaks10 = n10;
    s_beam_chatter  = chatter;

    // Raw diagnostic edge stream - every edge, never retained, not an entity.
    // This is what the beam thresholds get tuned from with real cars.
    if (s_mqtt) {
        char d[128];
        int n = snprintf(d, sizeof(d), "{\"edge\":\"%s\",\"uptime_ms\":%lu,\"dur_ms\":%d}",
                         b ? "blocked" : "clear", (unsigned long)t, dur);
        if (n > 0 && n < (int)sizeof(d)) esp_mqtt_client_publish(s_mqtt, BEAMDIAG_T, d, 0, 0, 0);
    }

    // Chatter guard: while the beam is flapping, publish the CURRENT state at
    // most every BEAM_MIN_PUB_GAP_MS instead of one publish per edge. A normal
    // edge is never delayed - the guard only arms when the firmware itself has
    // already flagged chat=1.
    int64_t now = esp_timer_get_time();
    if (chatter && (now - s_beam_last_pub_us) < (int64_t)BEAM_MIN_PUB_GAP_MS * 1000) return;
    s_beam_last_pub_us = now;

    s_beam_blocked = b ? 1 : 0;
    mqtt_publish_beam();

    // Keep the legacy sensor.<device>_gate_photo_beam (Clear/Blocked, fed from
    // the gate-state JSON) backed by the same real-time value.
    if (s_status_mux) xSemaphoreTake(s_status_mux, portMAX_DELAY);
    g_gate_status.beam_broken = s_beam_blocked;
    if (s_status_mux) xSemaphoreGive(s_status_mux);
    if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);
}

// "EVT BEAMACT ev=<type> pat=<pattern> n=<n> tot=<ms> gs=<state> t=<ms> brk=<list> gap=<list>"
static void handle_beam_activity(const char *p) {
    if (!s_mqtt) return;
    char ev[20], pat[16], brk[64], gap[64];
    int  n = 0, gs = -1;
    uint32_t tot = 0, t = 0;
    token_after(p, "ev=",  ev,  (int)sizeof(ev),  tok_word);
    token_after(p, "pat=", pat, (int)sizeof(pat), tok_word);
    token_after(p, "brk=", brk, (int)sizeof(brk), tok_list);
    token_after(p, "gap=", gap, (int)sizeof(gap), tok_list);
    int_after(p, " n=", &n);
    int_after(p, " gs=", &gs);
    u32_after(p, " tot=", &tot);
    u32_after(p, " t=", &t);
    if (!ev[0]) return;

    char js[384];
    int w = snprintf(js, sizeof(js),
        "{\"event_type\":\"%s\",\"pattern\":\"%s\",\"breaks\":%d,"
        "\"break_ms\":[%s],\"gap_ms\":[%s],\"total_ms\":%lu,"
        "\"gate_state\":\"%s\",\"uptime_ms\":%lu}",
        ev, pat[0] ? pat : "none", n, brk, gap,
        (unsigned long)tot, gate_state_name(gs), (unsigned long)t);
    // Event entities must NOT be retained, or every HA restart replays the last
    // car as a fresh trigger.
    if (w > 0 && w < (int)sizeof(js)) esp_mqtt_client_publish(s_mqtt, BEAMACT_T, js, 0, 1, 0);
}

// "EVT STATE st=.. lo=.. lc=.. bm=.. nf=.. slp=.. sc=.. dir=.. ma=.. mv=.. lk=.. dl=.. src=.. t=.."
// Everything mqtt_publish_state() needs, in one ~130-byte line.
static void handle_evt_state(const char *p) {
    gate_status_t o;
    int v;
    memset(&o, 0, sizeof(o));
    if (!int_after(p, " st=", &o.state)) return;
    o.open_limit    = int_after(p, " lo=",  &v) ? v : -1;
    o.closed_limit  = int_after(p, " lc=",  &v) ? v : -1;
    o.beam_broken   = int_after(p, " bm=",  &v) ? v : -1;
    o.nfault        = int_after(p, " nf=",  &v) ? v : -1;
    o.driver_asleep = int_after(p, " slp=", &v) ? v : 0;
    o.stall_count   = int_after(p, " sc=",  &v) ? v : 0;
    o.current_ma    = int_after(p, " ma=",  &v) ? v : 0;
    o.vm_mv         = int_after(p, " mv=",  &v) ? v : 0;
    token_after(p, "dir=", o.last_dir, (int)sizeof(o.last_dir), tok_word);
    token_after(p, "src=", o.cmd_src,  (int)sizeof(o.cmd_src),  tok_word);

    // lk = lockout_reason_t, only meaningful in ST_LOCKOUT (12); dl = both limit
    // inputs active right now. Text kept identical to the STATUS block's, so the
    // two paths cannot disagree about what HA displays.
    int lk = int_after(p, " lk=", &v) ? v : 0;
    int dl = int_after(p, " dl=", &v) ? v : 0;
    if (o.state == 12 && lk == 1)      strcpy(o.lockout, "both OPEN and CLOSED limit switches read active at once");
    else if (o.state == 12 && lk == 2) strcpy(o.lockout, "DRV8262 nFAULT stayed asserted after 3 clear attempts");
    else if (o.state == 12 && lk == 3) strcpy(o.lockout, "too many consecutive stalls - power-cycle to clear");
    else if (dl)                       strcpy(o.lockout, "both limit switches active - will lock out");

    if (s_status_mux) xSemaphoreTake(s_status_mux, portMAX_DELAY);
    g_gate_status = o;
    if (s_status_mux) xSemaphoreGive(s_status_mux);
    beam_sync_from_state(o.beam_broken);
    if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);
}

// Called by uart_rx_task for every ';'- (or content-) delimited message. The raw
// bytes were already broadcast to the web console; here we sniff for a pushed
// EVT line, a STATUS block, or an async fault/arrival line.
static void gate_rx_message(const char *msg) {
    gate_status_t tmp;

    // ---- pushed events first: cheapest to detect and the most latency-critical.
    // BEAMACT is tested before BEAM so "EVT BEAMACT" can never be mistaken for
    // an edge line (the trailing space in "EVT BEAM " already prevents it, but
    // the order makes that independent of the exact spelling).
    {
        const char *p;
        if (strstr(msg, "EVT ") != NULL) s_gate_pushes = true;
        if ((p = strstr(msg, "EVT BEAMACT ")) != NULL) { s_last_gate_msg_us = esp_timer_get_time(); handle_beam_activity(p); return; }
        if ((p = strstr(msg, "EVT BEAM "))    != NULL) { s_last_gate_msg_us = esp_timer_get_time(); handle_beam_edge(p);     return; }
        if ((p = strstr(msg, "EVT STATE "))   != NULL) { s_last_gate_msg_us = esp_timer_get_time(); handle_evt_state(p);     return; }
        if ((p = strstr(msg, "EVT MOVE_START ")) != NULL) {
            s_last_gate_msg_us = esp_timer_get_time();
            // The EVT STATE line for the same control-loop pass is right behind
            // this one and carries src= too, so there is nothing to publish here
            // - just make sure the publisher is awake for it.
            if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);
            return;
        }
    }

    if (parse_gate_status(msg, &tmp)) {
        s_last_gate_msg_us = esp_timer_get_time();
        s_gate_pushes = (strstr(msg, "Uptime:") != NULL);
        if (s_status_mux) xSemaphoreTake(s_status_mux, portMAX_DELAY);
        g_gate_status = tmp;
        if (s_status_mux) xSemaphoreGive(s_status_mux);
        beam_sync_from_state(tmp.beam_broken);
        return;
    }

    // "VERSION GateController_DRV8262 <date> <time>" - from the VERSION command
    // and from the boot banner. Store it; force a discovery re-publish so HA's
    // device sw_version updates.
    const char *v = strstr(msg, "VERSION GateController");
    if (v) {
        v += 8;                                   // past "VERSION "
        char buf[sizeof(s_msp_version)];
        int i = 0;
        while (i < (int)sizeof(buf) - 1 && v[i] && v[i] != '\r' && v[i] != '\n' && v[i] != ';') { buf[i] = v[i]; i++; }
        while (i > 0 && buf[i - 1] == ' ') i--;
        buf[i] = 0;
        if (strcmp(buf, s_msp_version) != 0) {
            strcpy(s_msp_version, buf);
            s_disc_dirty = true;
            if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);
        }
        return;
    }

    if (strstr(msg, "GateController_DRV8262 ready")) {
        const char *m = "[MSP430]: gate controller booted\n";
        ws_broadcast(m, strlen(m));
        // The gate's first pushed EVT STATE goes out one 10 ms tick after boot,
        // before its ADC has a VM reading, so it reports mv=0. Ask for a full
        // STATUS once things have settled rather than leaving that 0 retained
        // until the next state change.
        s_status_req_pending = true;
        s_status_req_at_us   = esp_timer_get_time() + (int64_t)1500 * 1000;
        if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);
        return;
    }

    // Arrival message (one ';'-block, two lines):
    //   "Gate: reached OPEN limit - travel time 8200 ms, avg 1160 mA, max 3200 mA"
    //   "Peak ramp-up current: 4100 mA"
    const char *ar = strstr(msg, "reached ");
    if (ar && strstr(ar, "limit - travel time")) {
        lastmove_t lm;
        memset(&lm, 0, sizeof(lm));
        strcpy(lm.dir, strstr(ar, "reached CLOSED") ? "CLOSED" : "OPEN");
        int x;
        if (int_after(ar, "travel time", &x)) lm.travel_ms = x;
        if (int_after(ar, "avg", &x))         lm.avg_ma    = x;
        if (int_after(ar, "max", &x))         lm.max_ma    = x;
        const char *pk = strstr(msg, "ramp-up current:");
        if (pk && int_after(pk, "current:", &x)) lm.peak_ma = x;
        s_cycle_count++;
        cycles_save();
        lm.cycles = s_cycle_count;
        lm.valid  = true;
        if (s_status_mux) xSemaphoreTake(s_status_mux, portMAX_DELAY);
        s_lastmove = lm;
        if (s_status_mux) xSemaphoreGive(s_status_mux);
        s_lastmove_dirty = true;
        if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);
        return;
    }

    if (s_mqtt_task &&
        (strstr(msg, "OBSTACLE") || strstr(msg, "LOCKED OUT") || strstr(msg, "LOCKOUT") ||
         strstr(msg, "stall") || strstr(msg, "homing") || strstr(msg, "Battery") ||
         strstr(msg, "nFAULT") || strstr(msg, "FAULT:"))) {
        xTaskNotifyGive(s_mqtt_task);
    }
}

static void json_escape(const char *in, char *out, int outsz) {
    int o = 0;
    for (const char *p = in; *p && o < outsz - 2; p++) {
        if (*p == '"' || *p == '\\') { out[o++] = '\\'; out[o++] = *p; }
        else if ((unsigned char)*p >= 0x20) { out[o++] = *p; }
    }
    out[o] = 0;
}

static void mqtt_publish_state(const gate_status_t *s) {
    if (!s_mqtt) return;
    char lo[168];
    json_escape(s->lockout, lo, sizeof(lo));
    char js[700];
    int n = snprintf(js, sizeof(js),
        "{\"state\":%d,\"state_name\":\"%s\",\"cover_state\":\"%s\",\"moving\":%s,\"low_batt\":%s,"
        "\"open_limit\":%s,\"closed_limit\":%s,\"beam_broken\":%s,\"nfault\":%d,"
        "\"current_ma\":%d,\"vm_mv\":%d,\"stall_count\":%d,\"driver_asleep\":%s,"
        "\"last_dir\":\"%s\",\"lockout\":\"%s\",\"cmd_src\":\"%s\"}",
        s->state, gate_state_name(s->state), gate_cover_state(s->state),
        gate_is_moving(s->state) ? "true" : "false",
        s->state == 13 ? "true" : "false",       // 13 = ST_LOW_BATTERY
        s->open_limit == 1 ? "true" : "false",
        s->closed_limit == 1 ? "true" : "false",
        s->beam_broken == 1 ? "true" : "false",
        s->nfault,
        s->current_ma, s->vm_mv, s->stall_count,
        s->driver_asleep == 1 ? "true" : "false",
        s->last_dir, lo, s->cmd_src);
    if (n > 0 && n < (int)sizeof(js)) esp_mqtt_client_publish(s_mqtt, MQTT_BASE "/state", js, 0, 1, 1);
}

// Change-gated diagnostics state. RSSI/heap jitter by a few units on almost
// every sample and the on-chip temp wobbles ~0.1-1 C constantly, so publishing
// msp430gate/esp (retained) unconditionally every 30 s was turning into a
// near-continuous stream of near-identical rows in HA's history. Round each
// field to a step that actually means something, and only publish the whole
// document (it is one JSON blob on one topic - there is no per-field publish)
// when a rounded value actually moved.
static int      s_esp_last_rssi_r      = 0;
static uint32_t s_esp_last_heap_r      = 0;
static int      s_esp_last_temp_r      = INT32_MIN;  // impossible -> first sample "changed"
static int64_t  s_esp_last_temp_pub_us = 0;

static int round_to_step(int v, int step) {
    return (int)lroundf((float)v / (float)step) * step;   // lroundf: correct for negative v (RSSI) too
}

// force=true bypasses every change gate and publishes unconditionally - used
// on the very first publish and right after a (re)discovery, so newly-created
// HA entities get an initial value instead of sitting unavailable until
// something happens to change by chance.
static void mqtt_publish_esp(bool force) {
    if (!s_mqtt) return;
    int rssi = 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    uint32_t up   = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t heap = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT);

    int      rssi_r         = round_to_step(rssi, 5);           // -65, -70, -75, ...
    uint32_t heap_r         = ((heap + 50u) / 100u) * 100u;      // nearest 100 B
    bool     temp_available = s_die_temp_ok;
    int      temp_r         = temp_available ? (int)lroundf(s_die_temp_c) : 0;   // nearest 1 C

    int64_t now = esp_timer_get_time();
    bool rssi_changed = (rssi_r != s_esp_last_rssi_r);
    bool heap_changed = (heap_r != s_esp_last_heap_r);
    // Temperature additionally needs >= 60 s since its last change-driven
    // publish, even if it keeps changing faster than that - "at most once a
    // minute, and only when it's different" is the spec, not "every minute".
    bool temp_trigger = temp_available && (temp_r != s_esp_last_temp_r) &&
                        (now - s_esp_last_temp_pub_us >= (int64_t)60 * 1000000);

    // uptime_s rides along whenever something else triggers a publish, but is
    // deliberately NOT itself a trigger - it changes every single sample, so
    // gating on it would defeat the point. msp430gate/availability (LWT/birth)
    // is already the "is the bridge alive" signal; this just stops being a
    // second, noisier heartbeat for the same fact.
    if (!force && !rssi_changed && !heap_changed && !temp_trigger) return;

    char tb[24] = "";
    if (temp_available) snprintf(tb, sizeof(tb), ",\"temp_c\":%d", temp_r);
    char js[160];
    int n = snprintf(js, sizeof(js), "{\"rssi\":%d,\"uptime_s\":%lu,\"free_heap\":%lu%s}",
                     rssi_r, (unsigned long)up, (unsigned long)heap_r, tb);
    if (n > 0 && n < (int)sizeof(js)) esp_mqtt_client_publish(s_mqtt, MQTT_BASE "/esp", js, 0, 1, 1);

    s_esp_last_rssi_r = rssi_r;
    s_esp_last_heap_r = heap_r;
    if (temp_available) { s_esp_last_temp_r = temp_r; s_esp_last_temp_pub_us = now; }
}

static void mqtt_publish_lastmove(void) {
    if (!s_mqtt) return;
    lastmove_t lm;
    if (s_status_mux) xSemaphoreTake(s_status_mux, portMAX_DELAY);
    lm = s_lastmove;
    if (s_status_mux) xSemaphoreGive(s_status_mux);
    if (!lm.valid) return;

    char ts[48] = "";
    time_t now; time(&now);
    struct tm tmv; gmtime_r(&now, &tmv);
    if (tmv.tm_year > (2020 - 1900)) {
        strftime(ts, sizeof(ts), ",\"ts\":\"%Y-%m-%dT%H:%M:%S+00:00\"", &tmv);
    }
    char js[256];
    int n = snprintf(js, sizeof(js),
        "{\"dir\":\"%s\",\"travel_ms\":%d,\"avg_ma\":%d,\"max_ma\":%d,\"peak_ma\":%d,\"cycles\":%lu%s}",
        lm.dir, lm.travel_ms, lm.avg_ma, lm.max_ma,
        lm.peak_ma, (unsigned long)lm.cycles, ts);
    if (n > 0 && n < (int)sizeof(js)) esp_mqtt_client_publish(s_mqtt, MQTT_BASE "/lastmove", js, 0, 1, 1);
}

// One retained HA discovery config. `body` = entity-specific JSON fields, no
// surrounding braces. A small delay after each keeps the burst off the wire.
static void mqtt_pub_discovery(const char *comp, const char *key, const char *name, const char *body) {
    if (!s_mqtt) return;
    char topic[128];
    int tn = snprintf(topic, sizeof(topic), MQTT_DISC_PREFIX "/%s/%s/%s/config", comp, s_dev_id, key);

    char sw[96] = "";
    if (s_msp_version[0]) {
        char ve[80];
        json_escape(s_msp_version, ve, sizeof(ve));
        snprintf(sw, sizeof(sw), ",\"sw\":\"%s\"", ve);
    }
    char js[1024];
    int n = snprintf(js, sizeof(js),
        "{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"object_id\":\"%s_%s\","
        "\"avty_t\":\"" MQTT_BASE "/availability\","
        "\"dev\":{\"ids\":[\"%s\"],\"name\":\"MSP430 Gate Controller\","
        "\"mf\":\"DIY\",\"mdl\":\"MSP430F5338 + DRV8262 / ESP32-C3 flasher\","
        "\"cu\":\"" BRIDGE_CONFIG_URL "\"%s},%s}",
        name, s_dev_id, key, s_dev_id, key, s_dev_id, sw, body);
    if (tn > 0 && tn < (int)sizeof(topic) && n > 0 && n < (int)sizeof(js)) {
        esp_mqtt_client_publish(s_mqtt, topic, js, 0, 1, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(15));
}

#define STATE_T MQTT_BASE "/state"

static void mqtt_publish_discovery(void) {
    // One-time: the photo beam moved from binary_sensor to sensor - clear the
    // stale retained config so HA drops the old entity. No-op once it's gone.
    {
        char t[128];
        snprintf(t, sizeof(t), MQTT_DISC_PREFIX "/binary_sensor/%s/beam/config", s_dev_id);
        esp_mqtt_client_publish(s_mqtt, t, "", 0, 1, 1);
    }

    // --- from msp430gate/state --------------------------------------------
    // cover_state is computed on the ESP32 to exactly HA's default state words.
    // Every cover_state string this firmware can ever emit (see
    // gate_cover_state()) is declared explicitly here, by its FULL config key
    // name (not the abbreviated form, to remove any risk of a misremembered
    // abbreviation) - so HA's interpretation of e.g. "stopped" can never
    // silently depend on whichever default a given HA version happens to use.
    mqtt_pub_discovery("cover", "gate", "Gate",
        "\"cmd_t\":\"" MQTT_BASE "/cmd\",\"pl_open\":\"OPEN\",\"pl_cls\":\"CLOSE\",\"pl_stop\":\"STOP\","
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ value_json.cover_state }}\",\"dev_cla\":\"gate\","
        "\"state_open\":\"open\",\"state_opening\":\"opening\",\"state_closed\":\"closed\","
        "\"state_closing\":\"closing\",\"state_stopped\":\"stopped\"");
    mqtt_pub_discovery("sensor", "state", "Gate state",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ value_json.state_name }}\",\"ic\":\"mdi:gate\"");
    mqtt_pub_discovery("binary_sensor", "moving", "Gate moving",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ 'ON' if value_json.moving else 'OFF' }}\",\"dev_cla\":\"moving\"");
    mqtt_pub_discovery("binary_sensor", "low_batt", "Gate low battery",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ 'ON' if value_json.low_batt else 'OFF' }}\",\"dev_cla\":\"battery\"");
    mqtt_pub_discovery("binary_sensor", "open_limit", "Gate open limit",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ 'ON' if value_json.open_limit else 'OFF' }}\"");
    mqtt_pub_discovery("binary_sensor", "closed_limit", "Gate closed limit",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ 'ON' if value_json.closed_limit else 'OFF' }}\"");
    mqtt_pub_discovery("sensor", "beam", "Gate photo beam",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ 'Blocked' if value_json.beam_broken else 'Clear' }}\",\"ic\":\"mdi:motion-sensor\"");
    mqtt_pub_discovery("binary_sensor", "lockout", "Gate lockout",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ 'ON' if value_json.lockout|length else 'OFF' }}\","
        "\"json_attr_t\":\"" STATE_T "\",\"dev_cla\":\"problem\"");
    mqtt_pub_discovery("sensor", "current", "Gate motor current",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ value_json.current_ma }}\",\"unit_of_meas\":\"mA\",\"stat_cla\":\"measurement\"");
    mqtt_pub_discovery("sensor", "vm", "Gate supply voltage",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ (value_json.vm_mv / 1000) | round(2) }}\","
        "\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"stat_cla\":\"measurement\"");
    mqtt_pub_discovery("sensor", "stalls", "Gate stall count",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ value_json.stall_count }}\",\"ic\":\"mdi:alert-octagon\"");
    mqtt_pub_discovery("sensor", "cmd_src", "Last command source",
        "\"stat_t\":\"" STATE_T "\",\"val_tpl\":\"{{ value_json.cmd_src | default('') }}\",\"ic\":\"mdi:gesture-tap\"");

    // --- photo beam (own topics, pushed on every edge) --------------------
    // The old sensor "beam" (Clear/Blocked, from the state JSON) stays for now -
    // HA switches over to this binary sensor, then "beam" can be dropped.
    mqtt_pub_discovery("binary_sensor", "beam_blocked", "Gate photo beam blocked",
        "\"stat_t\":\"" BEAM_T "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
        "\"json_attr_t\":\"" BEAM_ATTR_T "\",\"dev_cla\":\"occupancy\"");
    mqtt_pub_discovery("event", "beam_activity", "Gate beam activity",
        "\"stat_t\":\"" BEAMACT_T "\","
        "\"event_types\":[\"vehicle_pass\",\"short_break\",\"long_block\"],"
        "\"ic\":\"mdi:car-side\"");

    // --- from msp430gate/lastmove (per-move health + cycle count) ---------
    mqtt_pub_discovery("sensor", "lm_travel", "Last move travel time",
        "\"stat_t\":\"" MQTT_BASE "/lastmove\",\"val_tpl\":\"{{ value_json.travel_ms }}\",\"unit_of_meas\":\"ms\",\"ic\":\"mdi:timer-outline\"");
    mqtt_pub_discovery("sensor", "lm_avg", "Last move avg current",
        "\"stat_t\":\"" MQTT_BASE "/lastmove\",\"val_tpl\":\"{{ value_json.avg_ma }}\",\"unit_of_meas\":\"mA\"");
    mqtt_pub_discovery("sensor", "lm_max", "Last move max current",
        "\"stat_t\":\"" MQTT_BASE "/lastmove\",\"val_tpl\":\"{{ value_json.max_ma }}\",\"unit_of_meas\":\"mA\"");
    mqtt_pub_discovery("sensor", "lm_peak", "Last move peak inrush",
        "\"stat_t\":\"" MQTT_BASE "/lastmove\",\"val_tpl\":\"{{ value_json.peak_ma }}\",\"unit_of_meas\":\"mA\"");
    mqtt_pub_discovery("sensor", "lm_dir", "Last move direction",
        "\"stat_t\":\"" MQTT_BASE "/lastmove\",\"val_tpl\":\"{{ value_json.dir }}\",\"ic\":\"mdi:swap-horizontal\"");
    mqtt_pub_discovery("sensor", "lm_ts", "Last move completed",
        "\"stat_t\":\"" MQTT_BASE "/lastmove\",\"val_tpl\":\"{{ value_json.ts | default('') }}\",\"dev_cla\":\"timestamp\"");
    mqtt_pub_discovery("sensor", "cycles", "Gate cycle count",
        "\"stat_t\":\"" MQTT_BASE "/lastmove\",\"val_tpl\":\"{{ value_json.cycles }}\",\"stat_cla\":\"total_increasing\",\"ic\":\"mdi:counter\"");

    // --- ESP32 bridge diagnostics (msp430gate/esp) -----------------------
    mqtt_pub_discovery("sensor", "esp_rssi", "Bridge Wi-Fi RSSI",
        "\"stat_t\":\"" MQTT_BASE "/esp\",\"val_tpl\":\"{{ value_json.rssi }}\",\"unit_of_meas\":\"dBm\","
        "\"dev_cla\":\"signal_strength\",\"stat_cla\":\"measurement\",\"entity_category\":\"diagnostic\"");
    mqtt_pub_discovery("sensor", "esp_uptime", "Bridge uptime",
        "\"stat_t\":\"" MQTT_BASE "/esp\",\"val_tpl\":\"{{ value_json.uptime_s }}\",\"unit_of_meas\":\"s\","
        "\"dev_cla\":\"duration\",\"entity_category\":\"diagnostic\"");
    mqtt_pub_discovery("sensor", "esp_heap", "Bridge free heap",
        "\"stat_t\":\"" MQTT_BASE "/esp\",\"val_tpl\":\"{{ value_json.free_heap }}\",\"unit_of_meas\":\"B\","
        "\"stat_cla\":\"measurement\",\"entity_category\":\"diagnostic\"");
    mqtt_pub_discovery("sensor", "esp_temp", "Bridge chip temperature",
        "\"stat_t\":\"" MQTT_BASE "/esp\",\"val_tpl\":\"{{ value_json.temp_c | default('') }}\",\"unit_of_meas\":\"\\u00b0C\","
        "\"dev_cla\":\"temperature\",\"stat_cla\":\"measurement\",\"entity_category\":\"diagnostic\"");

    // --- buttons (publish to msp430gate/cmd) -----------------------------
    mqtt_pub_discovery("button", "clear_fault", "Gate clear fault / lockout",
        "\"cmd_t\":\"" MQTT_BASE "/cmd\",\"payload_press\":\"STOP\",\"ic\":\"mdi:lock-reset\"");
    mqtt_pub_discovery("button", "btn_open", "Gate open (button)",
        "\"cmd_t\":\"" MQTT_BASE "/cmd\",\"payload_press\":\"OPEN\",\"ic\":\"mdi:gate-open\"");
    mqtt_pub_discovery("button", "btn_close", "Gate close (button)",
        "\"cmd_t\":\"" MQTT_BASE "/cmd\",\"payload_press\":\"CLOSE\",\"ic\":\"mdi:gate\"");
    mqtt_pub_discovery("button", "btn_stop", "Gate stop (button)",
        "\"cmd_t\":\"" MQTT_BASE "/cmd\",\"payload_press\":\"STOP\",\"ic\":\"mdi:stop\"");
}
#undef STATE_T

static bool is_move_cmd(const char *s) {
    return !strcmp(s, "OPEN") || !strcmp(s, "CLOSE") || !strcmp(s, "TOGGLE") || !strcmp(s, "STOP");
}

// Was: arm a 4 s window of fast full-STATUS polling to beat the race between a
// relayed command and the MSP430's own tick. There is no race to beat any more
// - the firmware executes a queued command in the same tick it arrives and
// pushes EVT STATE from that same pass - so this only nudges the publisher.
static void arm_cmd_settle(void) {
    if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);
}

static void mqtt_handle_cmd(const char *data, int len) {
    char c[16];
    if (len < 0) len = 0;
    int n = len < (int)sizeof(c) - 1 ? len : (int)sizeof(c) - 1;
    for (int i = 0; i < n; i++) c[i] = (char)toupper((unsigned char)data[i]);
    c[n] = 0;
    while (n > 0 && (c[n - 1] == '\r' || c[n - 1] == '\n' || c[n - 1] == ' ')) c[--n] = 0;

    if (!is_move_cmd(c)) {
        char m[80];
        snprintf(m, sizeof(m), "[MQTT]: ignored unknown cmd '%s'\n", c);
        ws_broadcast(m, strlen(m));
        return;
    }
    if (s_bsl_active || s_fwu_busy) {
        char m[96];
        snprintf(m, sizeof(m), "[MQTT]: '%s' ignored - a flash/BSL session owns the UART\n", c);
        ws_broadcast(m, strlen(m));
        return;
    }
    char line[16];
    int ln = snprintf(line, sizeof(line), "%s\r\n", c);
    app_uart_write(line, ln);
    char m[64];
    snprintf(m, sizeof(m), "[MQTT]: -> gate command %s\n", c);
    ws_broadcast(m, strlen(m));
    arm_cmd_settle();   // wake the publisher so it is ready for the firmware's EVT line
}

static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED: {
            s_mqtt_up = true;
            const char *cm = "[MQTT]: broker connected\n";
            ws_broadcast(cm, strlen(cm));
            esp_mqtt_client_publish(s_mqtt, MQTT_BASE "/availability", "online", 0, 1, 1);
            esp_mqtt_client_subscribe(s_mqtt, MQTT_BASE "/cmd", 1);
            s_status_req_pending = true;   // ask the gate for one full STATUS now, so a
            s_status_req_at_us   = 0;      // reconnect resyncs HA without waiting for the
                                           // firmware's 5-minute heartbeat
            if (s_mqtt_task) xTaskNotifyGive(s_mqtt_task);   // task (re)publishes discovery + state
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt_up = false;
            break;
        case MQTT_EVENT_DATA:
            if (e->topic && e->topic_len == (int)strlen(MQTT_BASE "/cmd") &&
                strncmp(e->topic, MQTT_BASE "/cmd", e->topic_len) == 0) {
                mqtt_handle_cmd(e->data, e->data_len);
            }
            break;
        default:
            break;
    }
}

static void mqtt_gate_task(void *pv) {
    // Broker is only reachable once the station has a real IP (not in fallback-AP mode).
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_dev_id, sizeof(s_dev_id), MQTT_BASE "_%02x%02x%02x", mac[3], mac[4], mac[5]);

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri                   = MQTT_URI;
    cfg.credentials.username                 = MQTT_USERNAME;
    cfg.credentials.authentication.password  = MQTT_PASSWORD;
    cfg.session.keepalive                    = 60;     // spotty Wi-Fi: fewer false "offline" blips
    cfg.session.last_will.topic              = MQTT_BASE "/availability";
    cfg.session.last_will.msg                = "offline";
    cfg.session.last_will.msg_len            = 7;
    cfg.session.last_will.qos                = 1;
    cfg.session.last_will.retain             = 1;
    cfg.buffer.size                          = 2048;   // discovery payloads are large
    cfg.task.stack_size                      = 8192;   // our event handler does ws_broadcast + UART writes
    cfg.network.disable_auto_reconnect      = false;   // esp-mqtt keeps retrying the broker on its own

    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt);

    gate_status_t published;
    memset(&published, 0x7F, sizeof(published));   // != any real status -> first pass always publishes
    published.last_dir[0] = published.lockout[0] = published.cmd_src[0] = 0;
    int64_t last_pub_us = 0;
    int64_t last_disc_us = 0;
    int64_t last_esp_us = 0;
    int64_t last_status_req_us = 0;
    bool disc_sent = false;
    bool force_pub = false;

    for (;;) {
        int64_t now = esp_timer_get_time();

        // MSP430 version just learned/changed (once per gate reflash) - re-send
        // discovery now so HA's device sw_version updates; bypasses the rate limit
        // because s_disc_dirty only flips on an actual version-string change.
        if (s_disc_dirty) { s_disc_dirty = false; disc_sent = false; last_disc_us = 0; }

        if (!s_mqtt_up) {
            disc_sent = false;
        } else if (!disc_sent && (last_disc_us == 0 || now - last_disc_us > (int64_t)20 * 1000000)) {
            // Retained discovery, re-sent after a (re)connect but rate-limited so
            // a flapping Wi-Fi link can't turn it into a publish storm.
            mqtt_publish_discovery();
            disc_sent        = true;
            last_disc_us     = now;
            force_pub        = true;          // push a fresh state right after (re)discovery
            s_lastmove_dirty = true;          // ...and the retained lastmove/esp topics
            last_esp_us      = 0;
            mqtt_publish_beam();              // ...and the beam sensor's own retained topics
        }

        if (s_mqtt_up && (last_esp_us == 0 || now - last_esp_us > (int64_t)30 * 1000000)) {
            // Checked every 30 s; only actually PUBLISHES when something rounded
            // changed (see mqtt_publish_esp). force on the first check and right
            // after rediscovery (last_esp_us reset to 0 there).
            mqtt_publish_esp(last_esp_us == 0);
            last_esp_us = now;
        }
        if (s_mqtt_up && s_lastmove_dirty) {
            s_lastmove_dirty = false;
            mqtt_publish_lastmove();          // a move just completed (or after rediscovery)
        }

        bool busy = s_bsl_active || s_fwu_busy;

        // ---- status request ------------------------------------------------
        // The gate firmware pushes every change on its own, so this is NOT a
        // poll. It fires exactly twice: once at boot and once per MQTT
        // (re)connect - which is what makes HA correct again within seconds of a
        // bridge restart instead of waiting for the firmware's 5-minute
        // heartbeat. The second clause is a pure stale-link backstop: nothing at
        // all heard from the gate for MQTT_SANITY_POLL_MS (11 min, i.e. two
        // missed firmware heartbeats) means the link is suspect, so ask once.
        if (!busy && !s_bsl_active) {
            int64_t gap_ms = s_gate_pushes ? (int64_t)MQTT_SANITY_POLL_MS : (int64_t)MQTT_LEGACY_POLL_MS;
            bool stale = (s_last_gate_msg_us == 0)
                             ? (now > gap_ms * 1000)
                             : ((now - s_last_gate_msg_us) > gap_ms * 1000);
            if (stale) stale = (now - last_status_req_us) > gap_ms * 1000;
            if ((s_status_req_pending && now >= s_status_req_at_us) || stale) {
                s_status_req_pending = false;
                last_status_req_us   = now;
                app_uart_write("STATUS\r\n", 8);   // ';' is the MSP430's *reply* terminator, not an input one
            }
        }

        // ---- publish the gate state ----------------------------------------
        // Whoever parsed a pushed line already updated g_gate_status and woke
        // this task, so this runs within a task switch of the line landing -
        // well inside the 20 ms batching budget.
        if (s_mqtt_up) {
            gate_status_t cur;
            if (s_status_mux) xSemaphoreTake(s_status_mux, portMAX_DELAY);
            cur = g_gate_status;
            if (s_status_mux) xSemaphoreGive(s_status_mux);

            now = esp_timer_get_time();
            if (cur.state >= 0 &&
                (force_pub || !gate_status_equal(&cur, &published) ||
                 (now - last_pub_us) > (int64_t)MQTT_HEARTBEAT_MS * 1000)) {
                mqtt_publish_state(&cur);
                published   = cur;
                last_pub_us = now;
                force_pub   = false;
            }
        }

        // Nothing to poll: sleep until a pushed line wakes us, or the next
        // housekeeping tick (esp diagnostics / state heartbeat / stale check).
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MQTT_TASK_TICK_MS));
    }
}

// Embedded Web Interface HTML
const char index_html[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MSP430 Wireless Flasher & Console</title>
  <style>
    :root {
      --bg: #161616;
      --panel: #1e1e1e;
      --inset: #121212;
      --field: #242424;
      --border: #333333;
      --border-strong: #454545;
      --text: #eaeaea;
      --text-dim: #9b9b9b;
      --text-mute: #6a6a6a;
      --accent: #cc0000;
      --accent-hover: #e21b1b;
      --ok: #3fb950;
      --ok-bg: #12261a;
      --warn: #d6a01f;
      --warn-bg: #2a2413;
      --err: #f0523f;
      --err-bg: #2a1512;
      --font: system-ui, -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      --mono: ui-monospace, "Cascadia Code", "SF Mono", Menlo, Consolas, monospace;
      --radius: 6px;
    }
    html { scrollbar-gutter: stable; overflow-anchor: none; }
    body { font: 14px/1.5 var(--font); background: var(--bg); color: var(--text); margin: 0; padding: 20px; overflow-anchor: none; }
    #statusBoxes, #statusBanner, #paramRows, #capMeta, #capFw, .telemetry { overflow-anchor: none; }
    .container { max-width: 960px; margin: 0 auto; }
    h2 { margin: 0 0 20px; padding-bottom: 12px; font-size: 15px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.09em; color: var(--text); border-bottom: 1px solid var(--border); }
    h2 .sub { float: right; font: 12px/1.55 var(--mono); font-weight: 400; letter-spacing: 0; text-transform: none; color: var(--text-mute); }
    h3 { margin: 0 0 14px; padding-bottom: 8px; font-size: 12px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.08em; color: var(--text-dim); border-bottom: 1px solid var(--border); }
    .card { background: var(--panel); border: 1px solid var(--border); border-radius: var(--radius); padding: 18px; margin-bottom: 16px; }
    code { font-family: var(--mono); font-size: 0.92em; color: var(--text); }
    .btn { background: #2b2b2b; color: var(--text); border: 1px solid var(--border-strong); padding: 9px 15px; font-weight: 600; font-size: 13px; border-radius: 4px; cursor: pointer; margin-right: 8px; font-family: inherit; }
    .btn:hover { background: #353535; }
    .btn-secondary { background: transparent; color: var(--text-dim); border: 1px solid var(--border); }
    .btn-secondary:hover { background: var(--field); color: var(--text); }
    .btn-danger { background: var(--accent); color: #fff; border: 1px solid var(--accent); }
    .btn-danger:hover { background: var(--accent-hover); border-color: var(--accent-hover); }
    .btn-xs { padding: 4px 10px; font-size: 12px; }
    :focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
    #console { width: 100%; height: 340px; background: var(--inset); color: var(--text); font-family: var(--mono); font-size: 13px; padding: 10px; border-radius: 4px; border: 1px solid var(--border); overflow-y: scroll; box-sizing: border-box; white-space: pre-wrap; word-wrap: break-word; }
    .input-group { display: flex; gap: 10px; margin-top: 10px; }
    input[type="text"] { flex: 1; background: var(--field); border: 1px solid var(--border-strong); color: var(--text); padding: 10px; border-radius: 4px; font-family: var(--mono); }
    input[type="password"] { background: var(--field); border: 1px solid var(--border-strong); color: var(--text); padding: 10px; border-radius: 4px; font-family: var(--mono); }
    input[type="file"] { background: var(--field); padding: 8px; border-radius: 4px; color: var(--text); border: 1px solid var(--border); }
    input[type="text"]:focus, input[type="password"]:focus, input[type="number"]:focus, select:focus { border-color: var(--accent); outline: none; }
    .status { margin-top: 10px; font-size: 14px; color: var(--warn); }
    .instructions { font-size: 14px; color: var(--text-dim); line-height: 1.5; }
    .instructions ul { padding-left: 20px; margin: 8px 0; }
    table.pinmap { width: 100%; border-collapse: collapse; font-size: 14px; margin-top: 8px; }
    table.pinmap th, table.pinmap td { text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--border); }
    table.pinmap th { color: var(--text-dim); text-transform: uppercase; font-size: 11px; letter-spacing: 0.05em; }
    .progress-outer { width: 100%; background: var(--field); border-radius: 4px; height: 18px; margin-top: 12px; overflow: hidden; }
    .progress-inner { height: 100%; width: 0%; background: var(--ok); transition: width 0.2s ease; }
    select { background: var(--field); border: 1px solid var(--border-strong); color: var(--text); padding: 10px; border-radius: 4px; font-family: var(--mono); }
    .checkbox-row { display: flex; gap: 20px; margin-top: 10px; font-size: 13px; color: var(--text-dim); align-items: center; }
    .telemetry { display: flex; gap: 28px; flex-wrap: wrap; font-size: 14px; color: var(--text-dim); }
    .telemetry b { color: var(--text); font-variant-numeric: tabular-nums; }
    .cardhead { display: flex; justify-content: space-between; align-items: center; }
    .cardhead h3 { margin: 0; padding: 0; border: 0; }
    .btn-row { margin-bottom: 8px; }
    details.card > summary { list-style: none; cursor: pointer; color: var(--text); font-weight: 600; font-size: 12px; text-transform: uppercase; letter-spacing: 0.08em; display: flex; align-items: center; gap: 8px; user-select: none; }
    details.card > summary::-webkit-details-marker { display: none; }
    details.card > summary::before { content: "\25B6"; font-size: 0.75em; color: var(--text-mute); transition: transform 0.15s ease; }
    details.card[open] > summary::before { transform: rotate(90deg); }
    details.card[open] > summary { margin-bottom: 12px; padding-bottom: 8px; border-bottom: 1px solid var(--border); }
    .statgrid { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 10px; margin-top: 12px; min-height: 126px; align-content: start; }
    .stat { background: var(--inset); border: 1px solid var(--border); border-radius: 4px; padding: 10px 12px; min-height: 58px; }
    .stat .k { font-size: 11px; color: var(--text-dim); text-transform: uppercase; letter-spacing: 0.04em; }
    .stat .v { font-size: 17px; font-weight: bold; color: var(--text); margin-top: 4px; font-family: var(--mono); font-variant-numeric: tabular-nums; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .stat.good .v { color: var(--ok); }
    .stat.warn .v { color: var(--warn); }
    .stat.bad  .v { color: var(--err); }
    .banner { margin-top: 12px; padding: 10px 12px; border-radius: 4px; font-size: 14px; }
    .banner.bad  { background: var(--err-bg); color: var(--err); border: 1px solid var(--err); }
    .banner.good { background: var(--ok-bg); color: var(--ok); border: 1px solid var(--ok); }
    .banner.warn { background: var(--warn-bg); color: var(--warn); border: 1px solid var(--warn); }
    .tablewrap { overflow-x: auto; overflow-y: hidden; }
    table.params { width: 100%; border-collapse: collapse; font-size: 14px; margin-top: 8px; }
    table.params th, table.params td { text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--border); white-space: nowrap; }
    table.params th { color: var(--text-dim); text-transform: uppercase; font-size: 11px; letter-spacing: 0.05em; }
    table.params th.num, table.params td.num { text-align: right; font-family: var(--mono); }
    table.params td.num { font-variant-numeric: tabular-nums; }
    table.params td.warn { color: var(--warn); }
    table.params input[type="number"] { width: 88px; background: var(--field); border: 1px solid var(--border-strong); color: var(--text); padding: 6px; border-radius: 4px; font-family: var(--mono); }
    table.params .btn { padding: 6px 12px; margin: 0; font-size: 13px; }
    .param-name { cursor: pointer; border-bottom: 1px dotted var(--text-mute); }
    .help-hint { color: var(--text-mute); font-size: 11px; }
    table.params tr.phelp td { white-space: pre-line; background: var(--bg); }
    .dim { color: var(--text-mute); }
    #console div { min-height: 1.15em; }
    #console .c-err { color: var(--err); }
    #console .c-ok  { color: var(--text-dim); }
    #console .c-sys { color: var(--text-mute); }
    table.params tr.grouphdr td { background: var(--bg); color: var(--text-dim); font-weight: bold; cursor: pointer; user-select: none; text-transform: uppercase; font-size: 11px; letter-spacing: 0.05em; }
    table.params tr.grouphdr td:hover { color: var(--text); }
    table.params .btn-mini { background: transparent; color: var(--text-dim); border: 1px solid var(--border); border-radius: 4px; padding: 5px 8px; margin-right: 6px; cursor: pointer; font-size: 12px; }
    table.params .btn-mini:hover { background: var(--field); color: var(--text); }
    .inline-chk { display: inline-flex; align-items: center; gap: 6px; font-size: 13px; color: var(--text-dim); cursor: pointer; margin-left: 8px; }
    #capChart { width: 100%; max-width: 880px; height: 280px; background: var(--inset); border: 1px solid var(--border); border-radius: 4px; margin-top: 10px; display: block; }
    .legend { font-size: 12px; color: var(--text-dim); margin-top: 6px; }
    .legend .sw { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin: 0 4px 0 12px; vertical-align: middle; }
  </style>
</head>
<body>
  <div class="container">
    <h2>MSP430 Wireless Flasher &amp; Serial Console<span class="sub">MSP430F5338 &middot; DRV8262</span></h2>

    <div class="card">
      <h3>Gate Controls</h3>
      <p class="instructions" style="margin-top:0;">Sends console commands to the running gate controller over the app UART (D3/D10). Responses stream into the Serial Console below.</p>
      <div class="btn-row">
        <button class="btn" onclick="sendGate('OPEN')">Open Gate</button>
        <button class="btn" onclick="sendGate('CLOSE')">Close Gate</button>
        <button class="btn btn-secondary" onclick="sendGate('TOGGLE')">Toggle Gate</button>
        <button class="btn btn-danger" onclick="sendGate('STOP')">Stop</button>
        <button class="btn btn-danger" onclick="clearFault()">Clear Fault / Lockout</button>
      </div>
      <label class="inline-chk" style="margin-left:0;"><input type="checkbox" id="confirmMoves" checked> Confirm before the gate moves</label>
    </div>

    <div class="card">
      <h3>Gate Status</h3>
      <div class="btn-row">
        <button class="btn" onclick="refreshStatus()">Refresh status</button>
        <label class="inline-chk"><input type="checkbox" id="statusAuto" onchange="setStatusAuto(this.checked)"> Auto-refresh (10s)</label>
        <button class="btn btn-secondary" onclick="readMspVersion()">Read gate firmware version</button>
        <span id="statusStamp" class="dim" style="font-size:13px; margin-left:8px;">not read yet</span>
      </div>
      <div class="dim" style="font-size:13px; margin-bottom:4px;">Gate firmware: <b id="mspVer" style="color:var(--text);">unknown</b></div>
      <div id="statusBanner"></div>
      <div id="statusBoxes" class="statgrid"></div>
    </div>

    <div class="card">
      <h3>Last Move &mdash; Motor Current &amp; Supply Voltage</h3>
      <p class="instructions" style="margin-top:0;">The gate firmware records current + VM every 25&nbsp;ms during a drive and dumps the trace when the move ends (<code>CAP_BEGIN&hellip;CAP_END</code>). Newest move shown; the record lines are kept out of the console.</p>
      <div id="moveOutcome" class="banner" style="margin-top:0; margin-bottom:8px;">No move yet.</div>
      <div id="capMeta" class="dim" style="font-size:13px;">No capture yet &mdash; runs automatically after each gate move.</div>
      <canvas id="capChart"></canvas>
      <div class="legend"><span class="sw" style="background:#cc0000;"></span>motor current (mA)<span class="sw" style="background:#7fa8c9;"></span>supply VM (mV)</div>
      <div id="capFw" class="dim" style="font-size:13px; margin-top:6px;"></div>
    </div>

    <div class="card">
      <h3>Gate Settings</h3>
      <p class="instructions" style="margin-top:0;">Live values from <code>LIST</code>. <b>Set</b> writes one parameter to RAM only (the firmware range-checks it); use <b>Save all to flash</b> to make changes survive a power cycle. Hover a parameter name for what it does; <b>&#8634;</b> puts the default in the box. Yellow = currently differs from default; click a group heading to collapse it.</p>
      <div id="unsavedBanner" hidden></div>
      <div class="tablewrap">
        <table class="params">
          <thead><tr><th>Parameter</th><th class="num">Current</th><th class="num">Default</th><th>Unit</th><th>New value</th><th></th></tr></thead>
          <tbody id="paramRows"><tr><td colspan="6" class="dim">Click &ldquo;Reload values&rdquo; to read parameters from the gate.</td></tr></tbody>
        </table>
      </div>
      <div class="btn-row" style="margin-top:12px;">
        <button class="btn btn-secondary" onclick="refreshParams()">Reload values (LIST)</button>
        <button class="btn" onclick="saveToFlash()">Save all to flash (SAVE)</button>
        <button class="btn btn-danger" onclick="restoreDefaults()">Restore defaults (DEFAULTS)</button>
      </div>
      <div id="paramsMsg" class="status"></div>
    </div>

    <div class="card">
      <h3>Serial Console &mdash; Live MSP430 Application UART (115200 8N1)</h3>
      <p class="instructions" style="margin-top:0;">Wired to the running gate controller on D3/D10. Type <code>help</code> for its command list; gate motion and status prints stream in here live. Unavailable only while a BSL session is open.</p>
      <div id="console"></div>
      <div class="input-group">
        <input type="text" id="sendInput" placeholder="Type a command for the gate controller (e.g. help)..." onkeypress="handleKeyPress(event)">
        <select id="lineEnding" title="Line ending appended to sent messages">
          <option value="none">No line ending</option>
          <option value="cr">CR</option>
          <option value="lf">LF</option>
          <option value="crlf" selected>CRLF</option>
        </select>
        <button class="btn" onclick="sendSerial()">Send</button>
        <button class="btn btn-secondary" onclick="downloadConsole()">Download Log (.txt)</button>
        <button class="btn btn-secondary" onclick="clearConsole()">Clear Terminal</button>
      </div>
      <div class="checkbox-row">
        <label><input type="checkbox" id="autoScrollChk" checked onchange="autoScroll = this.checked"> Auto-scroll</label>
        <label><input type="checkbox" id="tsChk" onchange="showTimestamps = this.checked"> Timestamp console lines</label>
        <button class="btn btn-secondary btn-xs" onclick="sendGate('HELP')">Print command list (HELP)</button>
      </div>
    </div>

    <div class="card">
      <h3>TI-TXT Firmware Upload</h3>
      <p class="instructions" style="margin-top:0;">Streams the image to the running gate firmware over the app UART (D3/D10); the MSP430 self-writes main flash (0x8000-0xFFFF) from RAM. No BSL. The gate firmware must have the <code>FWUPDATE</code> command (JTAG-flash it once to get there).</p>
      <input type="file" id="fileInput" accept=".txt,.titxt">
      <button class="btn" onclick="uploadFirmware()">Upload &amp; Flash</button>
      <div class="progress-outer"><div id="progressBar" class="progress-inner"></div></div>
      <div id="uploadStatus" class="status">Ready</div>
    </div>

    <div class="card">
      <h3>ESP32 Firmware (OTA)</h3>
      <p class="instructions" style="margin-top:0;">Wireless self-update of <b>this flasher</b>. Pick a PlatformIO <code>firmware.bin</code> (<code>.pio/build/&hellip;/firmware.bin</code>), enter the OTA token, Flash. The ESP32 writes its spare OTA slot then reboots into it &mdash; this page drops and reconnects. Same thing as <code>pio run -e ota -t upload</code>. Needs the two-slot partition table (flash once over USB first).</p>
      <input type="file" id="otaFile" accept=".bin">
      <input type="password" id="otaToken" placeholder="OTA token" style="max-width:170px; flex:none;">
      <button class="btn btn-danger" onclick="uploadEspOta()">Flash ESP32</button>
      <div class="progress-outer"><div id="otaBar" class="progress-inner"></div></div>
      <div id="otaStatus" class="status">Ready</div>
    </div>

    <div class="card">
      <h3>Hardware Actions</h3>
      <button class="btn btn-danger" onclick="sendCmd('CMD:BSL', 'Invoke BSL mode now? This halts running gate controller firmware.')">Invoke BSL Mode</button>
      <button class="btn btn-danger" onclick="sendCmd('CMD:SWBSL', 'Software BSL entry: ask the running gate firmware to jump to the bootloader (no RST/TEST pulse). Halts the app.')">SW BSL Entry</button>
      <button class="btn btn-danger" onclick="sendCmd('CMD:RST', 'Reboot the MSP430 target now? (Hardware RST pulse - no flash erase)')">Reboot MSP430</button>
      <button class="btn btn-secondary" onclick="sendCmd('CMD:INFO')">Read Device Info</button>
      <button class="btn btn-secondary" onclick="sendCmd('CMD:PINTEST', 'Bit-bang each BSL line low/high for a multimeter check? Briefly reboots the gate.')">Pin Wiggle Test</button>
      <button class="btn btn-secondary" onclick="sendCmd('CMD:SYNC')">Force Time Sync</button>
      <div class="status">Last Synced: <span id="lastSynced">never</span></div>
    </div>

    <div class="card">
      <div class="cardhead" style="margin-bottom:14px;">
        <h3>ESP32 Status</h3>
        <label class="inline-chk" style="margin-left:0;">
          <input type="checkbox" id="telToggle" checked onchange="toggleTelemetry(this.checked)"> Enable Live Telemetry (5s)
        </label>
      </div>
      <div class="telemetry">
        <div>Network: <b id="telNet">-</b></div>
        <div>Wi-Fi Signal: <b id="telRssi">-</b></div>
        <div>IP Address: <b id="telIp">-</b></div>
        <div>Uptime: <b id="telUptime">-</b></div>
        <div>Free Heap: <b id="telHeap">-</b></div>
        <div>Chip Temp: <b id="telTemp">-</b></div>
      </div>
      <div class="btn-row" style="margin-top:14px;">
        <button class="btn btn-danger" onclick="sendCmd('CMD:ESPRST', 'Reboot the ESP32 now? The web UI drops and reconnects in a few seconds. The gate controller is not affected.')">Reboot ESP32</button>
      </div>
    </div>

    <details class="card">
      <summary>Quick Instructions</summary>
      <div class="instructions">
        <ul>
          <li><b>Flashing Firmware:</b> Select your CCS <code>.txt</code> or <code>.titxt</code> file and click <b>Upload & Flash</b>. The ESP32 handles BSL entry, flash erase, programming, and target reboot automatically.</li>
          <li><b>Reboot MSP430:</b> Pulses the hardware RST line for a clean power-on reset without erasing any memory.</li>
          <li><b>Serial Console:</b> Talks to the MSP430's normal application UART (D3/D10), so it works while the gate controller firmware is running &mdash; send <code>help</code>, read status, adjust variables, and watch the gate report its motion live. Goes quiet only during a BSL session.</li>
          <li><b>Time Sync:</b> Every 30 minutes (and on boot / on demand) the ESP32 transmits a <code>TIME:&lt;epoch&gt;\n</code> string on that same application UART to sync the MSP430 clock.</li>
        </ul>
      </div>
    </details>

    <details class="card">
      <summary>Pin Connections</summary>
      <div class="instructions">
        <table class="pinmap">
          <tr><th>ESP32 Pin</th><th>MSP430 Connection</th><th>Notes</th></tr>
          <tr><td>D4 (GPIO6)</td><td>RST / NMI</td><td>BSL invoke sequence + hardware reboot</td></tr>
          <tr><td>D5 (GPIO7)</td><td>TEST / TCK</td><td>BSL invoke sequence</td></tr>
          <tr><td>D6 (GPIO21)</td><td>BSL RX &mdash; MSP430 P1.2 (pin 36)</td><td>ESP32 UART TX (nominal). Auto-swaps with D7 if the BSL is silent. Only driven during a BSL session.</td></tr>
          <tr><td>D7 (GPIO20)</td><td>BSL TX &mdash; MSP430 P1.1 (pin 35)</td><td>ESP32 UART RX (nominal). Only listened to during a BSL session.</td></tr>
          <tr><td>D3 (GPIO5)</td><td>App UART RX (P8.3, pin 61)</td><td>ESP32 &rarr; MSP430 - always-on link to the running gate controller</td></tr>
          <tr><td>D10 (GPIO10)</td><td>App UART TX (P8.2, pin 60)</td><td>MSP430 &rarr; ESP32 - live gate status &amp; command output</td></tr>
          <tr><td>3.3V</td><td>3.3V</td><td>Shared supply rail</td></tr>
          <tr><td>GND</td><td>GND</td><td>Shared ground</td></tr>
        </table>
      </div>
    </details>
  </div>

  <script>
    var socket = null;
    var consoleDiv = document.getElementById('console');
    var MAX_LINES = 500;
    var autoScroll = true;
    var showTimestamps = false;
    var reconnectTimer = null;

    function connectWs() {
      if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
        return;
      }
      socket = new WebSocket('ws://' + window.location.host + '/ws');

      socket.onopen = function() {
        appendConsole("[System]: Connected to WebSerial\n");
        if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
        setTimeout(function() { refreshParams(); refreshStatus(); }, 700);
      };

      socket.onclose = function() {
        appendConsole("\n[System]: WebSocket disconnected. Reconnecting...\n");
        if (!reconnectTimer) {
          reconnectTimer = setInterval(connectWs, 2000);
        }
      };

      socket.onerror = function() {
        try { socket.close(); } catch(e) {}
      };

      socket.onmessage = function(event) {
        var data = event.data;
        if (data.indexOf("PROGRESS:") === 0) {
          var rest = data.slice(9).replace(/\n$/, '');
          var sep = rest.indexOf(":");
          var percent = parseInt(rest.slice(0, sep), 10);
          var label = rest.slice(sep + 1);
          document.getElementById('progressBar').style.width = percent + '%';
          var status = document.getElementById('uploadStatus');
          if (status) status.innerText = label + ' (' + percent + '%)';
          return;
        }
        if (data.indexOf("SYNCED:") === 0) {
          document.getElementById('lastSynced').textContent = data.slice(7).replace(/\n$/, '');
          return;
        }
        if (data.indexOf("TELEMETRY:") === 0) {
          // <rssi>:<qual>:<ip>:<uptime>:<heap>:<temp_c>:<net_mode>
          var parts = data.slice(10).replace(/\n$/, '').split(':');
          var rssi = parts[0];
          var qual = parts[1];
          var ip = parts[2];
          var uptime = parts[3];
          var heap = parts[4];
          var temp = parts[5];
          var net = parts.slice(6).join(':');   // colon-free from the firmware, but be safe
          if (rssi !== '0') {
            document.getElementById('telRssi').textContent = rssi + ' dBm (' + qual + ')';
          } else {
            document.getElementById('telRssi').textContent = 'AP Mode (' + qual + ')';
          }
          document.getElementById('telNet').textContent = net || '-';
          document.getElementById('telIp').textContent = ip;
          document.getElementById('telUptime').textContent = uptime + ' s';
          document.getElementById('telHeap').textContent = heap + ' bytes';
          document.getElementById('telTemp').textContent = (temp && temp !== '-') ? temp + ' °C' : '-';
          return;
        }
        feedCapture(data);
        appendConsole(data);
      };
    }
    connectWs();

    function withTimestamp(text) {
      if (!showTimestamps) return text;
      var t = new Date();
      var pad = function(n) { return String(n).padStart(2, '0'); };
      return '[' + pad(t.getHours()) + ':' + pad(t.getMinutes()) + ':' + pad(t.getSeconds()) + '] ' + text;
    }

    // The MSP430 prefixes every line with "[<unix_epoch_seconds>] " once it has
    // a TIME: sync (raw digits - not readable at a glance, and now that
    // STATUS_IF_CHANGED/etc print a lot more often it's on nearly every line).
    // Replace just that prefix with the local wall-clock time (no date - this
    // console is a live feed, not a log you'd need to date). Leaves the rest of
    // the line untouched, and leaves non-prefixed lines (System/UI/etc) alone.
    function humanizeTimestamp(ln) {
      var m = /^\s*\[(\d+)\]\s*/.exec(ln);
      if (!m) return ln;
      var epochSec = parseInt(m[1], 10);
      if (!epochSec) return ln;
      var d = new Date(epochSec * 1000);
      var pad = function(n) { return String(n).padStart(2, '0'); };
      return '[' + d.getHours() + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()) + '] ' +
             ln.slice(m[0].length);
    }

    // Console is now one <div> per line so fault/OK/system lines can be coloured.
    // Data arrives in arbitrary chunks, so a partial trailing line is held in
    // conPending and shown in a live node until its newline arrives.
    var conPending = '';
    var conLive = null;

    function consoleLineClass(ln) {
      if (/(\bERR\b|ERROR|\bFAIL(ED)?\b|LOCKED OUT|LOCKOUT|OBSTACLE|stall fault|DRIVER_FAULT|mismatch|\bNAK\b|rejected|\baborted?\b)/i.test(ln)) return 'c-err';
      if (/(^|[\s\]])(OK|SUCCESS|Saved\.|Complete!?|verified|ANSWERED)([\s.!]|$)/i.test(ln)) return 'c-ok';
      if (/^\s*(\[\d+\]\s*)?\[(System|UI|Serial TX|PinTest|BSL[^\]]*|SW-BSL|FWU[^\]]*|Device Info|Time Sync)\]/i.test(ln)) return 'c-sys';
      return '';
    }

    function makeConsoleLine(ln) {
      ln = ln.replace(/;\s*$/, '');   // strip the MSP430 end-of-message marker
      var d = document.createElement('div');
      var cls = consoleLineClass(ln);   // classify on the raw line (still has "[<epoch>]" if present)
      if (cls) d.className = cls;
      d.textContent = withTimestamp(humanizeTimestamp(ln));
      return d;
    }

    function appendConsole(text) {
      conPending += text;
      var parts = conPending.split('\n');
      conPending = parts.pop();
      if (conLive) { consoleDiv.removeChild(conLive); conLive = null; }
      for (var i = 0; i < parts.length; i++) {
        var raw = parts[i].replace(/\r$/, '');
        if (raw === ';') continue;                 // bare end-of-message marker
        if (feedMoveCapture(raw)) continue;        // CAP_BEGIN..CAP_END -> chart, kept out of the console
        scanForVersion(raw);
        scanForArrival(raw);
        scanForOutcome(raw);
        consoleDiv.appendChild(makeConsoleLine(raw));
      }
      if (conPending) { conLive = makeConsoleLine(conPending); consoleDiv.appendChild(conLive); }
      while (consoleDiv.childElementCount > MAX_LINES) consoleDiv.removeChild(consoleDiv.firstChild);
      if (autoScroll) consoleDiv.scrollTop = consoleDiv.scrollHeight;
    }

    // ---- MSP430 firmware version --------------------------------------------
    function scanForVersion(ln) {
      var m = /VERSION (GateController\S* .+?)\s*;?\s*$/.exec(ln.replace(/^\s*\[\d+\]\s*/, ''));
      if (m) document.getElementById('mspVer').textContent = m[1].trim();
    }

    // Firmware's own end-of-move numbers, shown under the capture chart.
    // "Gate: reached OPEN limit - travel time 8200 ms, avg 1160 mA, max 3200 mA"
    var lastArrival = null;
    function scanForArrival(ln) {
      var s = ln.replace(/^\s*\[\d+\]\s*/, '');
      var m = /reached (OPEN|CLOSED) limit - travel time (\d+) ms, avg (\d+) mA, max (\d+) mA/.exec(s);
      if (m) {
        lastArrival = { dir: m[1], travel: +m[2], avg: +m[3], max: +m[4], peak: null };
        renderArrival();
        return;
      }
      var p = /Peak ramp-up current:\s*(\d+)\s*mA/.exec(s);
      if (p && lastArrival) { lastArrival.peak = +p[1]; renderArrival(); }
    }
    function renderArrival() {
      var a = lastArrival, el = document.getElementById('capFw');
      if (!a) return;
      el.textContent = 'Gate reported: ' + a.dir + ' in ' + (a.travel / 1000).toFixed(2) + ' s · avg ' +
        a.avg + ' mA · max ' + a.max + ' mA' + (a.peak != null ? ' · peak inrush ' + a.peak + ' mA' : '');
    }

    // ---- Move outcome badge (above the chart) ------------------------------
    // Turns the "why did it stop" async event line the gate firmware already
    // prints into a one-line badge, so you don't have to go find it in the
    // console. Order matters only where two patterns could both match the same
    // line - they can't here (each starts with a distinct literal prefix).
    var OUTCOME_PATTERNS = [
      { re: /^Gate: reached (OPEN|CLOSED) limit/,                       cls: 'good', make: function(m) { return 'SUCCESS – reached ' + m[1]; } },
      { re: /^Gate: homing complete/,                                   cls: 'good', make: function()  { return 'SUCCESS – homing complete'; } },
      { re: /^Gate: homing found the opposite limit/,                   cls: 'good', make: function()  { return 'SUCCESS – homing found the opposite limit'; } },
      { re: /^Gate: reversal reached a limit/,                          cls: 'good', make: function()  { return 'SUCCESS – reversed back to a limit'; } },
      { re: /^OBSTACLE detected while (OPENING|CLOSING) \(over-current( HARD)?, (\d+) mA\)/,
                                                                         cls: 'bad',  make: function(m) { return 'OBSTACLE' + (m[2] ? ' (HARD)' : '') + ' – ' + m[3] + ' mA while ' + m[1].toLowerCase(); } },
      { re: /^OBSTACLE detected while CLOSING \(photo beam\)/,          cls: 'bad',  make: function()  { return 'OBSTACLE – photo beam broken'; } },
      { re: /^OBSTACLE detected while homing toward (OPEN|CLOSED)/,     cls: 'bad',  make: function(m) { return 'OBSTACLE – while homing toward ' + m[1]; } },
      { re: /^Gate: hard over-current during reversal/,                 cls: 'bad',  make: function()  { return 'STOPPED – hard over-current during reversal'; } },
      { re: /^Gate: obstacle on both sides - stall/,                    cls: 'bad',  make: function()  { return 'STALLED – obstacle on both sides'; } },
      { re: /^Gate: stall - MAX_DRIVE_DURATION exceeded.*count=(\d+)/,  cls: 'bad',  make: function(m) { return 'STALLED – timeout, no limit reached (count ' + m[1] + ')'; } },
      { re: /^FAULT: (OPEN|CLOSED) limit still engaged/,                cls: 'bad',  make: function(m) { return 'FAULT – ' + m[1] + ' limit still engaged (motor not moving?)'; } },
      { re: /^DRV8262 nFAULT asserted/,                                 cls: 'bad',  make: function()  { return 'DRIVER FAULT – DRV8262 nFAULT'; } },
      { re: /^LOCKOUT: fault gone and stable - cleared/,                cls: 'good', make: function()  { return 'LOCKOUT cleared'; } },
      { re: /^LOCKOUT: (\d+) consecutive stalls/,                       cls: 'bad',  make: function(m) { return 'LOCKOUT – ' + m[1] + ' consecutive stalls'; } },
      { re: /^LOCKOUT: (.+)/,                                           cls: 'bad',  make: function(m) { return 'LOCKOUT – ' + m[1]; } },
      { re: /^Gate: homing aborted by (.+?) - immediate stop/,          cls: 'warn', make: function(m) { return 'STOPPED – homing aborted (' + m[1] + ')'; } },
      { re: /^Gate: reversal complete, stopped mid-travel/,             cls: 'warn', make: function()  { return 'STOPPED – backed off obstacle, mid-travel'; } },
      { re: /^Gate: (.+?) while (opening|closing) - immediate stop/,    cls: 'warn', make: function(m) { return 'STOPPED – ' + m[1] + ' while ' + m[2]; } }
    ];
    function scanForOutcome(ln) {
      var s = ln.replace(/^\s*\[\d+\]\s*/, '');
      for (var i = 0; i < OUTCOME_PATTERNS.length; i++) {
        var m = OUTCOME_PATTERNS[i].re.exec(s);
        if (m) { renderMoveOutcome({ cls: OUTCOME_PATTERNS[i].cls, text: OUTCOME_PATTERNS[i].make(m) }); return; }
      }
    }
    function renderMoveOutcome(o) {
      var el = document.getElementById('moveOutcome');
      if (!el) return;
      el.className = 'banner' + (o.cls ? ' ' + o.cls : '');
      el.textContent = o.text;
    }
    function readMspVersion() { requestCapture('VER', 'DATA:VERSION\r\n', { idleMs: 300, maxMs: 1500 }); }
    function parseVerReply(buf) {
      var ln = lastReplyLine(buf, /VERSION GateController/);
      if (ln) {
        var m = /VERSION (.+?)\s*;?\s*$/.exec(ln);
        setParamsMsg('Gate firmware: ' + (m ? m[1].trim() : ln));
        if (m) document.getElementById('mspVer').textContent = m[1].trim();
      } else {
        setParamsMsg('VERSION: no reply - is the gate firmware new enough? (see console)');
      }
    }

    // ---- Per-move current / VM capture chart ------------------------------
    var moveCap = { on: false, label: '', ivl: 25, rows: [], last: null, deadline: 0 };

    function finishMoveCapture() {
      moveCap.on = false;
      moveCap.last = { label: moveCap.label, ivl: moveCap.ivl, rows: moveCap.rows.slice() };
      if (moveCap.rows.length) renderMoveChart();
      consoleNote('[capture]: ' + moveCap.rows.length + ' samples -> chart updated');
    }

    function feedMoveCapture(ln) {
      if (ln.indexOf('CAP_BEGIN') === 0) {
        var m = /CAP_BEGIN (\S+) samples=(\d+) interval_ms=(\d+)/.exec(ln);
        moveCap.on = true;
        moveCap.label = m ? m[1] : '?';
        moveCap.ivl = m ? parseInt(m[3], 10) : 25;
        moveCap.expected = m ? parseInt(m[2], 10) : 0;
        moveCap.rows = [];
        moveCap.deadline = Date.now() + 8000;   // don't stay in capture mode forever if CAP_END is lost
        consoleNote('[capture]: ' + moveCap.label + ' move - collecting ' + (moveCap.expected || '?') + ' samples...');
        return true;
      }
      if (!moveCap.on) return false;
      if (Date.now() > moveCap.deadline) { finishMoveCapture(); return false; }
      if (ln.indexOf('CAP_END') === 0) { finishMoveCapture(); return true; }
      if (/^\d/.test(ln)) {
        var recs = ln.split('~');
        for (var i = 0; i < recs.length; i++) {
          var p = recs[i].split(',');
          if (p.length === 3) moveCap.rows.push([+p[0], +p[1], +p[2]]);
        }
        return true;
      }
      return false;   // an async event line landed inside the block - let it through
    }

    function renderMoveChart() {
      var c = moveCap.last;
      var cv = document.getElementById('capChart');
      var meta = document.getElementById('capMeta');
      if (!c || !c.rows.length) { meta.textContent = 'Last capture had no samples.'; return; }

      var rows = c.rows;
      var tMax = rows[rows.length - 1][0] || 1;
      var maMax = 1, maSum = 0, vLo = 1e9, vHi = 0;
      for (var i = 0; i < rows.length; i++) {
        if (rows[i][1] > maMax) maMax = rows[i][1];
        maSum += rows[i][1];
        if (rows[i][2] < vLo) vLo = rows[i][2];
        if (rows[i][2] > vHi) vHi = rows[i][2];
      }
      var maAxis = Math.ceil(maMax / 500) * 500 || 500;
      var vPad = Math.max(200, Math.round((vHi - vLo) * 0.15));
      var vAxisLo = vLo - vPad, vAxisHi = vHi + vPad;
      if (vAxisHi <= vAxisLo) vAxisHi = vAxisLo + 1000;

      meta.textContent = c.label + '  ·  ' + rows.length + ' samples @ ' + c.ivl + ' ms  ·  ' +
        (tMax / 1000).toFixed(2) + ' s  ·  peak ' + maMax + ' mA, avg ' + Math.round(maSum / rows.length) +
        ' mA  ·  VM ' + vLo + '–' + vHi + ' mV';

      // crisp canvas
      var dpr = window.devicePixelRatio || 1;
      var W = cv.clientWidth || 860, H = cv.clientHeight || 280;
      cv.width = W * dpr; cv.height = H * dpr;
      var g = cv.getContext('2d');
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      g.clearRect(0, 0, W, H);
      var L = 52, R = 56, T = 12, B = 26;
      var pw = W - L - R, ph = H - T - B;

      g.strokeStyle = '#333333'; g.fillStyle = '#9b9b9b'; g.font = '11px ui-monospace, Consolas, monospace'; g.lineWidth = 1;
      for (var k = 0; k <= 4; k++) {
        var y = T + ph * k / 4;
        g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
        g.textAlign = 'right'; g.fillStyle = '#cc0000';
        g.fillText(Math.round(maAxis * (1 - k / 4)), L - 6, y + 3);
        g.textAlign = 'left'; g.fillStyle = '#7fa8c9';
        g.fillText(Math.round(vAxisLo + (vAxisHi - vAxisLo) * (1 - k / 4)), L + pw + 6, y + 3);
      }
      g.fillStyle = '#9b9b9b'; g.textAlign = 'center';
      for (var t = 0; t <= tMax + 1; t += Math.max(250, Math.round(tMax / 6 / 250) * 250)) {
        var x = L + pw * (t / tMax);
        g.fillText((t / 1000).toFixed(1) + 's', x, H - 8);
      }

      function plot(idx, axLo, axHi, color) {
        g.strokeStyle = color; g.lineWidth = 1.6; g.beginPath();
        for (var i = 0; i < rows.length; i++) {
          var x = L + pw * (rows[i][0] / tMax);
          var yv = (rows[i][idx] - axLo) / (axHi - axLo);
          var y = T + ph * (1 - Math.max(0, Math.min(1, yv)));
          if (i === 0) g.moveTo(x, y); else g.lineTo(x, y);
        }
        g.stroke();
      }
      plot(2, vAxisLo, vAxisHi, '#7fa8c9');   // VM behind
      plot(1, 0, maAxis, '#cc0000');          // current in front
    }

    function consoleText() {
      var out = [];
      for (var i = 0; i < consoleDiv.children.length; i++) out.push(consoleDiv.children[i].textContent);
      return out.join('\n');
    }

    // Append a single synthetic line WITHOUT touching conPending (safe to call
    // from inside appendConsole's own loop).
    function consoleNote(text) {
      var atEnd = consoleDiv.scrollHeight - consoleDiv.scrollTop - consoleDiv.clientHeight < 6;
      consoleDiv.appendChild(makeConsoleLine(text));
      while (consoleDiv.childElementCount > MAX_LINES) consoleDiv.removeChild(consoleDiv.firstChild);
      if (autoScroll && atEnd) consoleDiv.scrollTop = consoleDiv.scrollHeight;
    }

    function clearConsole() { consoleDiv.innerHTML = ''; conPending = ''; conLive = null; }

    function downloadConsole() {
      var blob = new Blob([consoleText()], { type: "text/plain;charset=utf-8" });
      var link = document.createElement("a");
      link.href = URL.createObjectURL(blob);
      link.download = "msp430_serial_log.txt";
      link.click();
      URL.revokeObjectURL(link.href);
    }

    function sendCmd(cmd, confirmMsg) {
      if (confirmMsg && !confirm(confirmMsg)) return;
      if (socket && socket.readyState === WebSocket.OPEN) socket.send(cmd);
    }

    // Gate control buttons: send a plain console command (CRLF-terminated) to the
    // running gate controller over the app UART, same path as typing in the
    // Serial Console. Motion commands optionally confirm, then kick an immediate
    // status read plus a short burst of auto-refreshes to watch the move.
    function sendGate(cmd) {
      if (!(socket && socket.readyState === WebSocket.OPEN)) {
        appendConsole("[System]: Not connected - command '" + cmd + "' not sent.\n");
        return;
      }
      var isMove = /^(OPEN|CLOSE|TOGGLE)$/.test(cmd);
      var cm = document.getElementById('confirmMoves');
      if (isMove && cm && cm.checked && !confirm('Move the gate now: ' + cmd + '?')) return;
      socket.send("DATA:" + cmd + "\r\n");
      if (isMove) { renderMoveOutcome({ cls: '', text: 'MOVING…' }); }
      if (isMove || cmd === 'STOP') {
        refreshStatus();
        pulseStatusAuto(isMove ? 22000 : 8000);
      }
    }

    // STOP doubles as the fault/lockout clear-attempt in the gate FSM
    // (tickLockout: any command clears once the fault condition is gone and has
    // been stable for LOCKOUT_CLEAR_STABLE_TIME) - and STOP does not also start a
    // move, so it is the safe one to use for this.
    function clearFault() {
      if (!(socket && socket.readyState === WebSocket.OPEN)) {
        appendConsole("[System]: Not connected - clear not sent.\n");
        return;
      }
      appendConsole("[UI]: Sending STOP to attempt a fault / lockout clear (no motion). Clears only once the underlying fault is gone and stable.\n");
      socket.send("DATA:STOP\r\n");
      refreshStatus();
      pulseStatusAuto(8000);
    }

    // ---- Auto-refresh of Gate Status -----------------------------------
    var statusAutoTimer = null;
    var statusAutoBurst = false;
    var statusAutoBurstT = null;

    // on: start/stop the timer. intervalMs defaults to 10 s. full=true polls a
    // real STATUS (live current/VM, always replies); otherwise STATUS_IF_CHANGED,
    // which the MSP430 answers only on a discrete-state change - so the steady
    // auto-refresh no longer spams the Serial Console at rest.
    function setStatusAuto(on, intervalMs, full) {
      if (statusAutoTimer) { clearInterval(statusAutoTimer); statusAutoTimer = null; }
      if (on) {
        var iv = intervalMs || 10000;
        var fn = full ? refreshStatus : pollStatus;
        statusAutoTimer = setInterval(function() {
          if (socket && socket.readyState === WebSocket.OPEN) fn();
        }, iv);
      }
    }
    // Poll hard for `ms` after a move command (full STATUS at 2 s so the live
    // current trace updates), unless the user already has the box on.
    function pulseStatusAuto(ms) {
      var cb = document.getElementById('statusAuto');
      if (cb && cb.checked) return;
      statusAutoBurst = true;
      setStatusAuto(true, 2000, true);
      if (statusAutoBurstT) clearTimeout(statusAutoBurstT);
      statusAutoBurstT = setTimeout(function() {
        if (statusAutoBurst && !(cb && cb.checked)) setStatusAuto(false);
        statusAutoBurst = false;
      }, ms);
    }

    function saveToFlash() {
      requestCapture('CMD', 'DATA:SAVE\r\n', { name: 'SAVE', idleMs: 400, maxMs: 3000 });
    }

    function restoreDefaults() {
      if (!confirm("Restore ALL parameters to factory defaults in RAM? Then use \"Save all to flash\" to make it permanent.")) return;
      requestCapture('CMD', 'DATA:DEFAULTS\r\n', { name: 'DEFAULTS', idleMs: 400, maxMs: 3000 });
    }

    // ---- Unsaved-parameter tracking ----------------------------------------
    // LIST reports the RAM value, so a SET_ you forgot to SAVE looks normal and
    // then silently reverts on the next power cycle. Track what changed this
    // session and nag until SAVE.
    var unsavedParams = {};
    var defaultsPending = false;

    function updateUnsavedBanner() {
      var el = document.getElementById('unsavedBanner');
      if (!el) return;
      var names = Object.keys(unsavedParams);
      if (defaultsPending) {
        el.className = 'banner warn';
        el.textContent = '⚠ Factory defaults are loaded in RAM - click "Save all to flash" to keep them (otherwise a power cycle restores the last saved values).';
        el.hidden = false;
      } else if (names.length) {
        el.className = 'banner warn';
        el.textContent = '⚠ ' + names.length + ' unsaved change' + (names.length > 1 ? 's' : '') +
          ' in RAM (' + names.join(', ') + ') - click "Save all to flash" or ' +
          (names.length > 1 ? 'they revert' : 'it reverts') + ' on the next power cycle.';
        el.hidden = false;
      } else {
        el.hidden = true;
      }
    }

    // ---- Serial-response capture -----------------------------------------
    // STATUS / LIST / SET replies come back as ordinary console text with no
    // end-of-output marker (the gate firmware has no prompt), and every line is
    // CRLF-terminated and - once time-synced - prefixed with "[<epoch>] ". So:
    // queue a request, collect what streams in, and finish once the stream has
    // been quiet for idleMs (or maxMs elapses as a backstop).
    var cap = { active: false, kind: null, name: null, buf: '', gotAny: false,
                lastRx: 0, started: 0, idleMs: 400, maxMs: 4000, poll: null };
    var capQueue = [];

    function requestCapture(kind, sendStr, opts) {
      opts = opts || {};
      if (!(socket && socket.readyState === WebSocket.OPEN)) {
        appendConsole("[System]: Not connected - " + kind + " request not sent.\n");
        return;
      }
      capQueue.push({ kind: kind, send: sendStr, name: opts.name || null,
                      idleMs: opts.idleMs || 400, maxMs: opts.maxMs || 4000 });
      if (!cap.active) runNextCapture();
    }

    function runNextCapture() {
      if (!capQueue.length) return;
      var job = capQueue.shift();
      cap.active = true; cap.kind = job.kind; cap.name = job.name;
      cap.buf = ''; cap.gotAny = false;
      cap.started = Date.now(); cap.lastRx = Date.now();
      cap.idleMs = job.idleMs; cap.maxMs = job.maxMs;
      try { socket.send(job.send); } catch (e) {}
      if (cap.poll) clearInterval(cap.poll);
      cap.poll = setInterval(capTick, 80);
    }

    function capTick() {
      var now = Date.now();
      // A per-move capture dump (CAP_BEGIN..CAP_END;) can interleave with a reply.
      // Strip whole CAP blocks so their trailing ';' can't end the capture early
      // and their record lines can't confuse the parsers.
      var buf = cap.buf.replace(/CAP_BEGIN[\s\S]*?CAP_END\r?\n;?\r?\n?/g, '');
      // The MSP430 ends each reply with ';' - finish as soon as we see one.
      // Idle-gap and max-time are fallbacks (older firmware / lost bytes).
      // If another reply (e.g. a STATUS poll) interleaves, its ';' must not end
      // this capture before our own reply's content has landed.
      var contentRe = cap.kind === 'LIST'   ? /[A-Z][A-Z0-9_]*\s*=\s*-?\d+\s*[%A-Za-z]*\s*\(default\s+-?\d+\)/
                    : cap.kind === 'STATUS' ? /State\s*=\s*\d+/i
                    : null;
      var haveContent = !contentRe || contentRe.test(buf);
      var done = (cap.gotAny && haveContent && buf.indexOf(';') !== -1) ||
                 (cap.gotAny && haveContent && now - cap.lastRx >= cap.idleMs) ||
                 (now - cap.started >= cap.maxMs);
      if (!done) return;
      clearInterval(cap.poll); cap.poll = null;
      cap.active = false;
      var kind = cap.kind, name = cap.name;
      if (kind === 'STATUS') parseStatus(buf);
      else if (kind === 'LIST') parseList(buf);
      else if (kind === 'SET') parseSetReply(buf, name);
      else if (kind === 'CMD') parseCmdReply(buf, name);
      else if (kind === 'VER') parseVerReply(buf);
      runNextCapture();
    }

    function feedCapture(text) {
      if (!cap.active) return;
      cap.buf += text; cap.gotAny = true; cap.lastRx = Date.now();
    }

    function esc(s) {
      return String(s).replace(/&/g, '&amp;').replace(/"/g, '&quot;')
                      .replace(/</g, '&lt;').replace(/>/g, '&gt;');
    }
    function firstMatch(re, s) { var m = re.exec(s); return m ? m[1] : null; }

    // Last line matching re, with any "[<epoch>] " firmware prefix stripped.
    function lastReplyLine(buf, re) {
      var lines = String(buf).split(/\r?\n/);
      for (var i = lines.length - 1; i >= 0; i--) {
        var ln = lines[i].replace(/^\s*\[\d+\]\s*/, '').trim();
        if (ln && re.test(ln)) return ln;
      }
      return null;
    }

    function setParamsMsg(t) { var e = document.getElementById('paramsMsg'); if (e) e.textContent = t; }

    function parseSetReply(buf, name) {
      var ln = lastReplyLine(buf, /^(OK|ERR)\b/);
      if (ln && /^OK/i.test(ln)) unsavedParams[name] = true;
      setParamsMsg('SET_' + name + '  →  ' + (ln || '(no reply captured - see console)'));
      updateUnsavedBanner();
      refreshParams();
    }

    function parseCmdReply(buf, name) {
      var ln = lastReplyLine(buf, /\S/);
      if (name === 'SAVE' && ln && /saved/i.test(ln)) { unsavedParams = {}; defaultsPending = false; }
      if (name === 'DEFAULTS' && ln && /default/i.test(ln)) { unsavedParams = {}; defaultsPending = true; }
      setParamsMsg(name + '  →  ' + (ln || '(no reply captured - see console)'));
      updateUnsavedBanner();
      refreshParams();
    }

    // ---- STATUS -----------------------------------------------------------
    // gate_state_t from gate_types.h - index == the number printed as "State=".
    var GATE_STATES = ['ST_INIT','ST_IDLE_OPEN','ST_IDLE_CLOSED','ST_OPENING',
      'ST_CLOSING','ST_SOFT_STOP','ST_STOPPED_MIDTRAVEL','ST_OBSTACLE_REVERSE',
      'ST_HOMING','ST_UNKNOWN','ST_STALL_FAULT','ST_DRIVER_FAULT','ST_LOCKOUT',
      'ST_LOW_BATTERY'];
    function stateClass(n) {
      if (n === 1 || n === 2) return 'good';   // idle at a limit
      if (n >= 10) return 'bad';               // stall / driver fault / lockout / low battery
      return 'warn';                           // init / moving / homing / unknown
    }

    var lastParams = {};   // name -> numeric current value, from the last LIST

    function refreshStatus() { requestCapture('STATUS', 'DATA:STATUS\r\n', { idleMs: 400, maxMs: 4000 }); }
    // Quiet poll: the MSP430 stays silent unless a discrete field changed, so use
    // short timeouts and let parseStatus() ignore an empty (unchanged) reply.
    function pollStatus() { requestCapture('STATUS', 'DATA:STATUS_IF_CHANGED\r\n', { idleMs: 250, maxMs: 900 }); }

    var _lastStatusHtml = null, _lastBannerHtml = null;

    function parseStatus(buf) {
      // STATUS_IF_CHANGED replies with nothing when the discrete gate state is
      // unchanged. Don't touch the UI (and don't show a parse error) for that.
      if (!/State\s*=\s*\d+/i.test(buf)) {
        var body = String(buf).replace(/CAP_BEGIN[\s\S]*?CAP_END/g, '')
                              .replace(/^\s*\[\d+\]\s*/gm, '')
                              // Pushed event lines (EVT STATE / EVT BEAM / ...) can land
                              // inside a capture window. On their own they are not a
                              // failed STATUS reply - treat them as empty, or a beam edge
                              // during the 10 s auto-refresh would pop a parse-error banner.
                              .replace(/EVT\s+[A-Z_]+[^\r\n]*/g, '')
                              .replace(/[\s;]+/g, '');
        if (body === '') { return; }
      }
      var gate    = firstMatch(/Gate:\s*([^\r\n]+)/, buf);
      var openLim = firstMatch(/OPEN\s+limit[^\r\n]*?=\s*(\d+)/i, buf);
      var closeLim= firstMatch(/CLOSED\s+limit[^\r\n]*?=\s*(\d+)/i, buf);
      var beam    = firstMatch(/photo beam broken\s*=\s*(\d+)/i, buf);
      var nfault  = firstMatch(/nFAULT\s*=\s*(\d+)/i, buf);
      var state   = firstMatch(/State\s*=\s*(\d+)/i, buf);
      var lastDir = firstMatch(/lastDir\s*=\s*([A-Za-z_]+)/i, buf);
      var curMa   = firstMatch(/Current:\s*(-?\d+)\s*mA/i, buf);
      var vm      = firstMatch(/VM:\s*(-?\d+)\s*mV/i, buf);
      var stall   = firstMatch(/Stall count:\s*(\d+)/i, buf);
      var asleep  = firstMatch(/Driver asleep:\s*(\d+)/i, buf);
      // The "LOCKED OUT:" reason line - "Gate: LOCKED OUT" has no colon after
      // OUT, so this only matches the dedicated explanation line.
      var note    = firstMatch(/(LOCKED OUT:[^\r\n]+)/i, buf);
      // STATUS prints this line when both limits read active but it hasn't locked out yet.
      var dualWarn = /Both limit switches read ACTIVE/i.test(buf);

      var boxes = document.getElementById('statusBoxes');
      var banner = document.getElementById('statusBanner');

      if (gate === null && state === null) {
        var eb = '<div class="banner bad">Could not parse the STATUS reply - see the Serial Console.</div>';
        if (eb !== _lastBannerHtml) { banner.innerHTML = eb; _lastBannerHtml = eb; }
        return;
      }

      var sn = state === null ? null : parseInt(state, 10);
      var gcls = sn === null ? 'warn' : stateClass(sn);
      var stateLabel = (sn === null) ? '-' : (state + (GATE_STATES[sn] ? ' · ' + GATE_STATES[sn] : ''));

      // VM thresholds from the live params if we have them, else a static guess.
      var vmCls = '';
      if (vm !== null) {
        var v = parseInt(vm, 10);
        if (lastParams.VM_LOW_CUTOFF != null) {
          if (v < lastParams.VM_LOW_CUTOFF) vmCls = 'bad';
          else if (lastParams.VM_LOW_RECOVER != null && v < lastParams.VM_LOW_RECOVER) vmCls = 'warn';
        } else if (v < 11000) { vmCls = 'warn'; }
      }

      var html = '';
      html += mkBox('Gate', gate || '-', gcls);
      html += mkBox('State', stateLabel, gcls);
      html += mkBox('Last direction', lastDir || '-', '');
      html += mkBox('OPEN limit', openLim === null ? '-' : (openLim === '1' ? 'at OPEN end' : 'no'), openLim === '1' ? 'good' : '');
      html += mkBox('CLOSED limit', closeLim === null ? '-' : (closeLim === '1' ? 'at CLOSED end' : 'no'), closeLim === '1' ? 'good' : '');
      html += mkBox('Photo beam', beam === null ? '-' : (beam === '1' ? 'Blocked' : 'Clear'), beam === '1' ? 'bad' : 'good');
      html += mkBoxTitle('nFAULT (P2.7)', nfault === null ? '-' : nfault,
                         nfault === '0' && asleep !== '1' ? 'warn' : '',
                         'DRV8262 nFAULT is active-low: 0 = fault asserted, 1 = OK. Can also read 0 while the driver is asleep.');
      html += mkBox('Motor current', curMa === null ? '-' : curMa + ' mA', '');
      html += mkBox('VM rail', vm === null ? '-' : vm + ' mV', vmCls);
      html += mkBox('Stall count', stall === null ? '-' : stall, stall !== null && parseInt(stall, 10) > 0 ? 'warn' : 'good');
      html += mkBox('Driver', asleep === null ? '-' : (asleep === '1' ? 'asleep' : 'awake'), '');

      if (html !== _lastStatusHtml) { boxes.innerHTML = html; _lastStatusHtml = html; }
      var bh = note ? '<div class="banner bad">' + esc(note) + '</div>'
             : dualWarn ? '<div class="banner warn">Both limit switches read ACTIVE - the controller will lock out. Check the limit-switch wiring.</div>'
             : '';
      if (bh !== _lastBannerHtml) { banner.innerHTML = bh; _lastBannerHtml = bh; }
      document.getElementById('statusStamp').textContent = 'read ' + new Date().toLocaleTimeString();
    }

    function mkBox(k, v, cls) {
      var sv = (v == null ? '-' : String(v));
      return '<div class="stat ' + (cls || '') + '"><div class="k">' + esc(k) +
             '</div><div class="v" title="' + esc(sv) + '">' + esc(sv) + '</div></div>';
    }
    function mkBoxTitle(k, v, cls, title) {
      return '<div class="stat ' + (cls || '') + '" title="' + esc(title) + '"><div class="k">' + esc(k) +
             '</div><div class="v">' + esc(v == null ? '-' : v) + '</div></div>';
    }

    // ---- LIST (tunable parameters) --------------------------------------
    // One-line explanations lifted from the firmware's LIST_DETAILS output.
    var PARAM_HELP = {
      MAX_DRIVE_DURATION: "Max seconds for one open or close drive; if no limit is reached by then the motor is stopped and a stall fault is raised.",
      HOMING_TIMEOUT: "Max seconds for a homing move (run when position is unknown); no limit found in time raises a stall fault.",
      RUN_DUTY: "PWM duty percent for a normal open or close move once it has ramped up to speed.",
      HOMING_DUTY: "PWM duty percent while homing - driving toward a limit switch to re-learn the gate position.",
      REVERSE_DUTY: "PWM duty percent used to drive back toward OPEN after an obstacle is hit while closing.",
      RAMP_START_DUTY: "Duty an open/close LAUNCHES at (instead of from a standstill), then ramps up to RUN_DUTY over RAMP_TIME. Set near where the motor actually breaks free so it starts moving at once instead of after a slow climb from 0%. 0 = old behaviour (ramp from 0). Homing and obstacle-reverse ignore this.",
      RAMP_TIME: "Milliseconds to ramp duty RAMP_START_DUTY->target at the start of an open/close, and target->0 at a normal stop (soft start / soft stop).",
      OBSTACLE_REVERSE_TIME: "Milliseconds to drive in reverse after an obstacle is hit while closing, before stopping mid-travel.",
      OPENING_BACKOFF: "Milliseconds of gentle reverse after an over-current while opening, to unload the jam before stopping (0 = none).",
      BRAKE_TIME: "Milliseconds the motor is short-braked (windings shorted) after reaching a limit, before the driver coasts and sleeps.",
      OC_TRIP_CURRENT: "SOFT obstacle threshold (level 1 of 3). Sensed motor current above this, after OC_BLANK_TIME/OC_GRACE_TIME expire, stops or reverses the gate - the everyday 'something's in the way' detector. Kept below OC_HARD_LIMIT.",
      OC_HARD_LIMIT: "HARD obstacle threshold (level 2 of 3). Sensed current above this reacts immediately, even during OC_BLANK_TIME (a real jam/short) - only OC_GRACE_TIME can also silence this one. Kept above OC_TRIP_CURRENT.",
      VREF_CURRENT_REGULATION: "Hardware current-regulation reference (level 3 of 3). Hardware control is currently DISABLED in firmware (VREF_DAC_ENABLE=0, pending bench verification) - VREF1 is the original fixed ~3.3 V GPIO reference, so this value is stored/range-checked but does NOT move the hardware trip right now. When enabled it drives VREF1 via the MSP430's DAC12_1 (P6.7, EVM jumpered J25+J6) - see drv8262_SetCurrentLimitMa(). The DAC output clamps at AVCC (~3.3 V) above roughly 9.4 A (R81||R82 = 1650 ohm), matching the ADC current-sense ceiling, so values near/above the top of the range behave the same as 9400 once it's enabled.",
      STALL_LOCKOUT_COUNT: "Number of consecutive stall faults that escalates to a full lockout requiring a power cycle.",
      LOCKOUT_CLEAR_STABLE_TIME: "Milliseconds the lockout cause (e.g. both limits active) must stay clear before a button press is allowed to clear the lockout.",
      IDLE_SLEEP: "1 = sleep the DRV8262 (outputs off, low power) whenever the gate is at rest; 0 = keep it powered.",
      OC_BLANK_TIME: "Startup window (ms) where OC_TRIP_CURRENT (the SOFT threshold) is ignored, to ride out normal motor inrush. OC_HARD_LIMIT still applies during this window unless OC_GRACE_TIME also covers it.",
      OC_GRACE_TIME: "Startup window (ms) where BOTH OC_TRIP_CURRENT and OC_HARD_LIMIT are ignored - the 'ignore overcurrent entirely for a few seconds' knob. 0 = disabled (default, safest). Non-zero trades startup obstacle detection for reliability - the stall timeout and stuck-limit checks still stop a genuinely jammed gate.",
      VM_LOW_CUTOFF: "Motor supply mV below this (held for VM_LOW_CONFIRM_TIME) blocks new moves and enters low-battery. Kept below VM_LOW_RECOVER.",
      VM_LOW_RECOVER: "Motor supply mV that VM must climb back above (and hold) to leave low-battery and run again. Kept above VM_LOW_CUTOFF.",
      VM_LOW_CONFIRM_TIME: "Milliseconds VM must stay under VM_LOW_CUTOFF before low-battery is entered (ignores a brief sag).",
      DEADTIME: "Milliseconds with both motor legs off when reversing direction mid-move, so the H-bridge fully turns off before driving the other way.",
      SOFT_END_DUTY: "Duty a normal open/close ramps down to after SOFT_END_OPEN/CLOSE ms, so the gate eases into the end-of-travel limit instead of hitting it at full drive.",
      SOFT_END_OPEN: "Elapsed ms into an OPEN drive after which duty drops to SOFT_END_DUTY (0 = disabled). Set it a bit under your typical open travel time from the capture chart.",
      SOFT_END_CLOSE: "Elapsed ms into a CLOSE drive after which duty drops to SOFT_END_DUTY (0 = disabled). Set it a bit under your typical close travel time from the capture chart."
    };

    // Range / units per parameter, from s_params[] in the gate firmware's
    // nvconfig.c - used for the number-input min/max, the tooltip, and a
    // client-side check before a SET_ is sent.
    var PARAMS_META = {
      MAX_DRIVE_DURATION:        { min: 3,    max: 60,    unit: 's'  },
      HOMING_TIMEOUT:            { min: 5,    max: 120,   unit: 's'  },
      RUN_DUTY:                  { min: 10,   max: 100,   unit: '%'  },
      HOMING_DUTY:               { min: 10,   max: 100,   unit: '%'  },
      REVERSE_DUTY:              { min: 10,   max: 100,   unit: '%'  },
      RAMP_START_DUTY:           { min: 0,    max: 100,   unit: '%'  },
      RAMP_TIME:                 { min: 100,  max: 5000,  unit: 'ms' },
      OBSTACLE_REVERSE_TIME:     { min: 100,  max: 3000,  unit: 'ms' },
      OPENING_BACKOFF:           { min: 0,    max: 2000,  unit: 'ms' },
      BRAKE_TIME:                { min: 0,    max: 5000,  unit: 'ms' },
      OC_TRIP_CURRENT:           { min: 500,  max: 9400,  unit: 'mA' },
      OC_HARD_LIMIT:             { min: 500,  max: 9400,  unit: 'mA' },
      VREF_CURRENT_REGULATION:   { min: 500,  max: 9400,  unit: 'mA' },
      STALL_LOCKOUT_COUNT:       { min: 1,    max: 20,    unit: ''   },
      LOCKOUT_CLEAR_STABLE_TIME: { min: 0,    max: 10000, unit: 'ms' },
      IDLE_SLEEP:                { min: 0,    max: 1,     unit: ''   },
      OC_BLANK_TIME:             { min: 100,  max: 20000, unit: 'ms' },
      OC_GRACE_TIME:             { min: 0,    max: 20000, unit: 'ms' },
      VM_LOW_CUTOFF:             { min: 8000, max: 14000, unit: 'mV' },
      VM_LOW_RECOVER:            { min: 8000, max: 14000, unit: 'mV' },
      VM_LOW_CONFIRM_TIME:       { min: 0,    max: 5000,  unit: 'ms' },
      DEADTIME:                  { min: 0,    max: 1000,  unit: 'ms' },
      SOFT_END_DUTY:             { min: 10,   max: 100,   unit: '%'  },
      SOFT_END_OPEN:             { min: 0,    max: 60000, unit: 'ms' },
      SOFT_END_CLOSE:            { min: 0,    max: 60000, unit: 'ms' }
    };
    // Cross-parameter ordering the firmware enforces (returns ERR ORDERING).
    var ORDER_NOTE = {
      OC_TRIP_CURRENT: 'Must stay below OC_HARD_LIMIT.',
      OC_HARD_LIMIT:   'Must stay above OC_TRIP_CURRENT.',
      VM_LOW_CUTOFF:   'Must stay below VM_LOW_RECOVER.',
      VM_LOW_RECOVER:  'Must stay above VM_LOW_CUTOFF.'
    };

    // Collapsible groupings for the params table. A param not listed here lands
    // in an "Other" group so nothing ever disappears.
    var PARAM_GROUPS = [
      { title: 'Travel & timing',         names: ['MAX_DRIVE_DURATION','HOMING_TIMEOUT','RAMP_TIME','BRAKE_TIME','DEADTIME'] },
      { title: 'Speed (PWM duty)',        names: ['RUN_DUTY','RAMP_START_DUTY','HOMING_DUTY','REVERSE_DUTY'] },
      { title: 'Soft end-of-travel',      names: ['SOFT_END_DUTY','SOFT_END_OPEN','SOFT_END_CLOSE'] },
      { title: 'Obstacle & over-current', names: ['OC_TRIP_CURRENT','OC_HARD_LIMIT','OC_BLANK_TIME','OC_GRACE_TIME','OBSTACLE_REVERSE_TIME','OPENING_BACKOFF','VREF_CURRENT_REGULATION'],
        note: 'Five related knobs, easy to mix up: OC_TRIP_CURRENT and OC_HARD_LIMIT are the two actual SOFTWARE trip levels (soft vs. hard - tap either name above for detail). OC_BLANK_TIME and OC_GRACE_TIME are startup windows that temporarily silence those levels (blank = soft only, grace = both). VREF_CURRENT_REGULATION is a separate HARDWARE trip (DAC-driven VREF1) meant to backstop all of them - currently DISABLED in firmware (fixed ~3.3 V instead, pending bench verification), so it is stored/range-checked but inert for now. Once enabled, keep it at or above OC_HARD_LIMIT or the hardware trip would fire before the software ever gets a chance to react.' },
      { title: 'Stall & lockout',         names: ['STALL_LOCKOUT_COUNT','LOCKOUT_CLEAR_STABLE_TIME','IDLE_SLEEP'] },
      { title: 'Battery / motor supply',  names: ['VM_LOW_CUTOFF','VM_LOW_RECOVER','VM_LOW_CONFIRM_TIME'] }
    ];
    var groupCollapsed = {};   // title -> true; persists across LIST refreshes
    var paramHelpOpen  = {};   // param NAME -> true; persists across LIST refreshes, like groupCollapsed

    function toggleGroup(idx) {
      var t = PARAM_GROUPS[idx].title;
      groupCollapsed[t] = !groupCollapsed[t];
      applyGroupCollapse();
    }
    function applyGroupCollapse() {
      for (var i = 0; i < PARAM_GROUPS.length; i++) {
        var t = PARAM_GROUPS[i].title, col = !!groupCollapsed[t];
        var hdr = document.getElementById('grphdr_' + i);
        if (hdr) hdr.cells[0].textContent = (col ? '▸ ' : '▾ ') + t;
        var rows = document.querySelectorAll('tr.prow:not(.phelp)[data-grp="' + i + '"]');
        for (var j = 0; j < rows.length; j++) rows[j].hidden = col;
        // A param's explanation row stays governed by paramHelpOpen, but never
        // shows while its own group is collapsed (nothing to point it at).
        var helps = document.querySelectorAll('tr.phelp[data-grp="' + i + '"]');
        for (var h = 0; h < helps.length; h++) {
          var nm = helps[h].getAttribute('data-name');
          helps[h].hidden = col || !paramHelpOpen[nm];
        }
      }
    }

    // Tap/click a parameter name to show a plain-English explanation inline -
    // the table also has a hover "title" tooltip, but that's unreachable on a
    // touchscreen, which is most of how this page gets used.
    function toggleParamHelp(name) {
      paramHelpOpen[name] = !paramHelpOpen[name];
      var row = document.getElementById('help_' + name);
      if (row) row.hidden = !paramHelpOpen[name];
      var hint = document.getElementById('hint_' + name);
      if (hint) hint.textContent = paramHelpOpen[name] ? '▴ hide' : 'ℹ what is this?';
    }

    function refreshParams() { requestCapture('LIST', 'DATA:LIST\r\n', { idleMs: 450, maxMs: 4000 }); }

    function parseList(buf) {
      // Not anchored to ^ - each line carries a "[<epoch>] " prefix once synced.
      // "NAME = <int> [unit] (default <int>)".
      var re = /([A-Z][A-Z0-9_]*)\s*=\s*(-?\d+)\s*([%A-Za-z]*)\s*\(default\s+(-?\d+)\)/g;
      var rows = [], m;
      while ((m = re.exec(buf)) !== null) {
        rows.push({ name: m[1], val: m[2], unit: m[3] || '', def: m[4] });
      }
      if (!rows.length) {
        setParamsMsg('Could not parse the LIST reply - see the Serial Console.');
        return;
      }
      lastParams = {};
      for (var i = 0; i < rows.length; i++) lastParams[rows[i].name] = parseInt(rows[i].val, 10);
      renderParams(rows);
      setParamsMsg(rows.length + ' parameters read ' + new Date().toLocaleTimeString());
    }

    function paramRowHtml(r, gi, collapsed) {
      var meta = PARAMS_META[r.name] || {};
      var changed = r.val !== r.def;
      var desc = PARAM_HELP[r.name] || 'No description available.';
      var tip = desc;
      if (meta.min != null) tip += '\n\nDocumented range ' + meta.min + '–' + meta.max +
        (meta.unit ? ' ' + meta.unit : '') + ' (advisory - the firmware enforces the real limits)';
      if (ORDER_NOTE[r.name]) tip += '\n' + ORDER_NOTE[r.name];
      var open = !!paramHelpOpen[r.name];
      return '<tr class="prow" data-grp="' + gi + '"' + (collapsed ? ' hidden' : '') + '>' +
        '<td><span class="param-name" title="' + esc(tip) + '" onclick="toggleParamHelp(\'' + esc(r.name) + '\')">' +
          esc(r.name) + '</span> <span class="help-hint" id="hint_' + esc(r.name) + '">' +
          (open ? '▴ hide' : 'ℹ what is this?') + '</span></td>' +
        '<td class="num' + (changed ? ' warn' : '') + '">' + esc(r.val) + '</td>' +
        '<td class="num dim">' + esc(r.def) + '</td>' +
        '<td>' + esc(r.unit || meta.unit || '') + '</td>' +
        '<td><input type="number" id="p_' + esc(r.name) + '" placeholder="' + esc(r.val) + '"></td>' +
        '<td><button class="btn-mini" title="Put the default (' + esc(r.def) + ') in the box" ' +
             'onclick="fillDefault(\'' + esc(r.name) + '\',\'' + esc(r.def) + '\')">↺</button>' +
            '<button class="btn" onclick="setParam(\'' + esc(r.name) + '\')">Set</button></td>' +
        '</tr>' +
        '<tr class="prow phelp" data-grp="' + gi + '" data-name="' + esc(r.name) + '" id="help_' + esc(r.name) + '"' +
          (collapsed || !open ? ' hidden' : '') + '><td colspan="6" class="dim">' + esc(tip) + '</td></tr>';
    }

    var _lastParamsHtml = null;

    function renderParams(rows) {
      var byName = {}, used = {}, html = '', i;
      for (i = 0; i < rows.length; i++) byName[rows[i].name] = rows[i];

      for (var g = 0; g < PARAM_GROUPS.length; g++) {
        var grp = PARAM_GROUPS[g], present = [];
        for (var n = 0; n < grp.names.length; n++) if (byName[grp.names[n]]) present.push(byName[grp.names[n]]);
        if (!present.length) continue;
        var col = !!groupCollapsed[grp.title];
        html += '<tr class="grouphdr" id="grphdr_' + g + '" onclick="toggleGroup(' + g + ')">' +
                '<td colspan="6">' + (col ? '▸ ' : '▾ ') + esc(grp.title) + '</td></tr>';
        if (grp.note) {
          html += '<tr class="prow" data-grp="' + g + '"' + (col ? ' hidden' : '') +
                  '><td colspan="6" class="dim" style="white-space:pre-line; font-size:13px;">' + esc(grp.note) + '</td></tr>';
        }
        for (var k = 0; k < present.length; k++) { html += paramRowHtml(present[k], g, col); used[present[k].name] = 1; }
      }
      var other = [];
      for (i = 0; i < rows.length; i++) if (!used[rows[i].name]) other.push(rows[i]);
      if (other.length) {
        var gi = PARAM_GROUPS.length;
        html += '<tr class="grouphdr"><td colspan="6">▾ Other</td></tr>';
        for (var o = 0; o < other.length; o++) html += paramRowHtml(other[o], gi, false);
      }
      if (html !== _lastParamsHtml) { document.getElementById('paramRows').innerHTML = html; _lastParamsHtml = html; }
    }

    function fillDefault(name, def) {
      var el = document.getElementById('p_' + name);
      if (el) { el.value = def; el.focus(); }
    }

    function setParam(name) {
      var el = document.getElementById('p_' + name);
      if (!el) return;
      var raw = el.value.trim();
      if (raw === '') return;
      if (!/^-?\d+$/.test(raw)) { setParamsMsg('SET_' + name + ': enter a whole number.'); return; }
      var v = parseInt(raw, 10);
      var meta = PARAMS_META[name];
      var warn = '';
      if (meta && (v < meta.min || v > meta.max)) {
        // Advisory only. The firmware is the authority on the real range and
        // range-checks every SET_ itself; some limits have been raised there
        // beyond what this page has hard-coded. Send it and let the gate decide.
        warn = '  (note: outside this page’s documented ' +
               meta.min + '–' + meta.max + (meta.unit ? ' ' + meta.unit : '') + ' - the gate will accept or ERR)';
      }
      el.value = '';
      if (warn) setParamsMsg('SET_' + name + ' ' + v + warn);
      // Capture OK / ERR OUT_OF_RANGE / ERR ORDERING / ... then re-LIST.
      requestCapture('SET', 'DATA:SET_' + name + ' ' + v + '\r\n', { name: name, idleMs: 350, maxMs: 2000 });
    }

    function toggleTelemetry(enabled) {
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(enabled ? "CMD:TELEM:ON" : "CMD:TELEM:OFF");
      }
      if (!enabled) {
        document.getElementById('telRssi').textContent = 'Paused';
        document.getElementById('telUptime').textContent = 'Paused';
        document.getElementById('telHeap').textContent = 'Paused';
        document.getElementById('telTemp').textContent = 'Paused';
      }
    }

    function getLineEnding() {
      var v = document.getElementById('lineEnding').value;
      if (v === 'cr') return '\r';
      if (v === 'lf') return '\n';
      if (v === 'crlf') return '\r\n';
      return '';
    }

    function sendSerial() {
      var input = document.getElementById('sendInput');
      if (input.value && socket && socket.readyState === WebSocket.OPEN) {
        var str = input.value + getLineEnding();
        socket.send("DATA:" + str);
        input.value = '';
      }
    }
    function handleKeyPress(e) { if (e.key === 'Enter') sendSerial(); }

    function uploadFirmware() {
      var file = document.getElementById('fileInput').files[0];
      var status = document.getElementById('uploadStatus');

      if (!file) { alert("Select a TI-TXT firmware file first."); return; }

      document.getElementById('progressBar').style.width = '0%';
      status.innerText = "Uploading image to ESP32...";
      var xhr = new XMLHttpRequest();
      xhr.open("POST", "/upload", true);
      xhr.upload.onprogress = function(e) {
        if (e.lengthComputable && e.loaded < e.total) {
          var p = Math.round(e.loaded / e.total * 100);
          document.getElementById('progressBar').style.width = p + '%';
          status.innerText = "Uploading image to ESP32... (" + p + "%)";
        }
      };
      xhr.onload = function() {
        if (xhr.status === 200) {
          // Burn runs on the ESP32 now; live progress + final result arrive
          // as PROGRESS: WebSocket messages.
          status.innerText = "Image received - streaming to MSP430...";
        } else {
          document.getElementById('progressBar').style.width = '0%';
          status.innerText = "Upload rejected (" + xhr.status + ") - see console.";
        }
      };
      xhr.onerror = function() { status.innerText = "Upload failed (network)."; };
      xhr.send(file);
    }

    function uploadEspOta() {
      var f = document.getElementById('otaFile').files[0];
      var token = document.getElementById('otaToken').value.trim();
      var st = document.getElementById('otaStatus');
      var bar = document.getElementById('otaBar');
      if (!f) { alert("Pick a firmware.bin first."); return; }
      if (!token) { alert("Enter the OTA token."); return; }
      if (!confirm("Flash THIS ESP32 with " + f.name + " (" + Math.round(f.size / 1024) +
                   " KB)? It will reboot into the new image.")) return;

      bar.style.width = '0%';
      st.innerText = "Uploading to ESP32...";
      var xhr = new XMLHttpRequest();
      xhr.open("POST", "/ota?token=" + encodeURIComponent(token), true);
      xhr.upload.onprogress = function(e) {
        if (e.lengthComputable) {
          var p = Math.round(e.loaded / e.total * 100);
          bar.style.width = p + '%';
          st.innerText = "Uploading to ESP32... (" + p + "%)";
        }
      };
      xhr.onload = function() {
        if (xhr.status === 200) {
          bar.style.width = '100%';
          st.innerText = "Image accepted - ESP32 rebooting. Page will reconnect shortly.";
        } else {
          bar.style.width = '0%';
          st.innerText = "OTA rejected (" + xhr.status + "): " + (xhr.responseText || "see console").trim();
        }
      };
      xhr.onerror = function() {
        // The reboot right after a 200 can also trip this - treat as probably-OK.
        st.innerText = "Connection closed - if the image was accepted the ESP32 is rebooting.";
      };
      xhr.send(f);
    }
  </script>
</body>
</html>
)rawliteral";

static void ws_async_free_cb(esp_err_t err, int socket, void *arg) {
    if (arg) {
        free(arg);
    }
}

void ws_broadcast(const char *data, size_t len) {
    if (!server_handle || !data || len == 0) return;
    size_t max_clients = 8;
    int fds[8];
    size_t client_count = max_clients;

    if (httpd_get_client_list(server_handle, &client_count, fds) == ESP_OK) {
        for (size_t i = 0; i < client_count; i++) {
            if (httpd_ws_get_fd_info(server_handle, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                uint8_t *copy = (uint8_t*)malloc(len + 1);
                if (copy) {
                    memcpy(copy, data, len);
                    copy[len] = '\0';
                    httpd_ws_frame_t ws_pkt;
                    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
                    ws_pkt.payload = copy;
                    ws_pkt.len = len;
                    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
                    esp_err_t err = httpd_ws_send_data_async(server_handle, fds[i], &ws_pkt, ws_async_free_cb, copy);
                    if (err != ESP_OK) {
                        free(copy);
                    }
                }
            }
        }
    }
}

// Streams the MSP430's normal application UART (UART_PORT routed to PIN_APP_RX)
// to every connected web console - gate status lines, "help" output, and any
// other prints from the running gate controller firmware.
// Pauses while s_bsl_active: during a BSL session UART_PORT is switched over to
// the 9600 8E1 flashing pins and is read directly by the BSL command helpers.
void uart_rx_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    static char acc[1200];        // message accumulator for the STATUS parser
    int acc_len = 0;
    while (1) {
        if (s_bsl_active) {
            acc_len = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            data[len] = '\0';
            ws_broadcast((char*)data, len);

            // Assemble whole messages for gate_rx_message(). The MSP430 firmware
            // terminates each reply with ';'. Fallbacks: a content-complete
            // STATUS block (older firmware, no ';'), and a hard cap so console
            // chatter can't grow this without bound.
            for (int i = 0; i < len; i++) {
                char ch = (char)data[i];
                if (acc_len < (int)sizeof(acc) - 1) acc[acc_len++] = ch;
                if (ch == ';') {
                    acc[acc_len] = '\0';
                    gate_rx_message(acc);
                    acc_len = 0;
                }
            }
            acc[acc_len] = '\0';
            // A per-move capture dump (CAP_BEGIN..CAP_END, up to ~16 KB) is relayed
            // raw to the web console but never parsed here - drop it from the
            // accumulator so it doesn't thrash it (keep it if a STATUS reply
            // happens to be interleaved).
            if (acc_len > 200 && strstr(acc, "CAP_BEGIN") && !strstr(acc, "CAP_END") && !strstr(acc, "State=")) {
                acc_len = 0; acc[0] = '\0';
            } else if (acc_len >= (int)sizeof(acc) - 1) {
                gate_rx_message(acc);
                int keep = 300;
                memmove(acc, acc + acc_len - keep, keep);
                acc_len = keep;
                acc[acc_len] = '\0';
            } else if (strstr(acc, "State=") && strstr(acc, "Driver asleep:")) {
                gate_rx_message(acc);
                acc_len = 0;
            }
        }
    }
}

// Bit-bang one programming line LOW (3 s) then HIGH (3 s) so a multimeter on the
// *MSP430* end of the jumper can confirm the wire is really connected to the pin
// it's supposed to be. Used by CMD:PINTEST.
static void pintest_pulse(const char *label, gpio_num_t pin) {
    char m[176];
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(pin, 0);
    snprintf(m, sizeof(m), "[PinTest]: %s = LOW (expect ~0 V at the MSP430 pin) ...\n", label);
    ws_broadcast(m, strlen(m));
    vTaskDelay(pdMS_TO_TICKS(3000));
    gpio_set_level(pin, 1);
    snprintf(m, sizeof(m), "[PinTest]: %s = HIGH (expect ~3.3 V at the MSP430 pin) ...\n", label);
    ws_broadcast(m, strlen(m));
    vTaskDelay(pdMS_TO_TICKS(3000));
    gpio_set_level(pin, 0);
}

static void cmd_worker_task(void *pvParameters) {
    char *cmd = (char*)pvParameters;
    if (!cmd) {
        vTaskDelete(NULL);
        return;
    }

    if (strcmp(cmd, "CMD:PINTEST") == 0) {
        if (s_bsl_active) {
            const char *busy = "[PinTest]: Busy - a BSL session is active. Try again after it finishes.\n";
            ws_broadcast(busy, strlen(busy));
        } else {
            s_bsl_active = true; // pause the app-UART relay while we bit-bang the pins
            const char *intro =
                "[PinTest]: Each line goes LOW for 3 s then HIGH for 3 s. Probe the MSP430 pin "
                "(far end of the jumper) and confirm it follows:\n"
                "  D5->TEST pin 91,  D6->P1.2 pin 36,  D7->P1.1 pin 35,  D4->RST pin 96 (reboots the gate).\n";
            ws_broadcast(intro, strlen(intro));
            pintest_pulse("D5 / TEST  (MSP430 pin 91)", PIN_TEST);
            pintest_pulse("D6 / BSL data (MSP430 P1.2, pin 36)", PIN_TX);
            pintest_pulse("D7 / BSL data (MSP430 P1.1, pin 35)", PIN_RX);
            pintest_pulse("D4 / RST   (MSP430 pin 96)", PIN_RST);
            reset_msp430(); // restores pin directions + UART routing, clears s_bsl_active
            const char *done = "[PinTest]: Done - lines restored, MSP430 rebooted to normal mode.\n";
            ws_broadcast(done, strlen(done));
        }
    } else if (strcmp(cmd, "CMD:BSL") == 0) {
        enter_bsl();
        const char *resp = "[System]: Invoked Hardware BSL Entry Sequence (Target is now in BSL mode)\n";
        ws_broadcast(resp, strlen(resp));
    } else if (strcmp(cmd, "CMD:SWBSL") == 0) {
        // Software entry: no RST/TEST pulse. Ask the gate firmware to jump to
        // 0x1000 itself, then probe the BSL on P1.1/P1.2.
        const char *m = "[System]: Software BSL entry - asking the running gate firmware to jump to 0x1000...\n";
        ws_broadcast(m, strlen(m));
        if (bsl_enter_via_software()) {
            uint8_t body[64]; uint16_t bl = 0;
            if (send_bsl_cmd_raw(0x19, NULL, 0, body, &bl, 500) && bl >= 5 && body[0] == 0x3A) {
                char msg[160];
                snprintf(msg, sizeof(msg),
                    "[SW-BSL]: BSL_VERSION reply: %02X %02X %02X %02X (Vendor 0x%02X, CI 0x%02X, API 0x%02X, PI 0x%02X)\n",
                    body[1], body[2], body[3], body[4], body[1], body[2], body[3], body[4]);
                ws_broadcast(msg, strlen(msg));
            }
        }
        reset_msp430();
        const char *d = "[System]: MSP430 rebooted back to the application.\n";
        ws_broadcast(d, strlen(d));
    } else if (strcmp(cmd, "CMD:RST") == 0) {
        reset_msp430();
        const char *resp = "[System]: Hardware RST pulsed - MSP430 target rebooted\n";
        ws_broadcast(resp, strlen(resp));
    } else if (strcmp(cmd, "CMD:INFO") == 0) {
        bsl_read_device_info();
    } else if (strcmp(cmd, "CMD:SYNC") == 0) {
        do_time_sync();
    } else if (strcmp(cmd, "CMD:ESPRST") == 0) {
        const char *resp = "[System]: Rebooting ESP32 now - reconnect in a few seconds...\n";
        ws_broadcast(resp, strlen(resp));
        vTaskDelay(pdMS_TO_TICKS(400));   // let the line flush to every client
        esp_restart();                    // does not return
    }

    free(cmd);
    vTaskDelete(NULL);
}

esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) return ESP_OK;

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // First call with max_len = 0 to get actual frame length
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    if (ws_pkt.len > 0) {
        uint8_t *buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (!buf) {
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(buf);
            return ret;
        }

        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
            char *msg = (char*)buf;
            if (strncmp(msg, "CMD:", 4) == 0) {
                if (strcmp(msg, "CMD:TELEM:ON") == 0) {
                    s_telemetry_enabled = true;
                    const char *resp = "[System]: Live telemetry enabled\n";
                    ws_broadcast(resp, strlen(resp));
                } else if (strcmp(msg, "CMD:TELEM:OFF") == 0) {
                    s_telemetry_enabled = false;
                    const char *resp = "[System]: Live telemetry paused\n";
                    ws_broadcast(resp, strlen(resp));
                } else {
                    char *cmd_copy = strdup(msg);
                    if (cmd_copy) {
                        xTaskCreate(cmd_worker_task, "cmd_worker", 4096, cmd_copy, 5, NULL);
                    }
                }
            } else if (strncmp(msg, "DATA:", 5) == 0) {
                char *payload = msg + 5;
                if (s_bsl_active) {
                    const char *busy = "[Serial TX]: Ignored - a BSL session is active, the MSP430 application UART is offline.\n";
                    ws_broadcast(busy, strlen(busy));
                } else {
                    int written = app_uart_write(payload, strlen(payload));
                    char log_msg[384];
                    snprintf(log_msg, sizeof(log_msg), "[Serial TX]: Sent %d bytes to MSP430 app UART: %s\n", written, payload);
                    ws_broadcast(log_msg, strlen(log_msg));

                    // A web-UI-triggered move/stop is relayed byte-for-byte like
                    // any other command; the nudge just wakes the publisher so it
                    // is already awake when the firmware's EVT line lands.
                    char up[16];
                    size_t un = strlen(payload);
                    if (un >= sizeof(up)) un = sizeof(up) - 1;
                    for (size_t i = 0; i < un; i++) up[i] = (char)toupper((unsigned char)payload[i]);
                    up[un] = 0;
                    while (un > 0 && (up[un - 1] == '\r' || up[un - 1] == '\n' || up[un - 1] == ' ')) up[--un] = 0;
                    if (is_move_cmd(up)) arm_cmd_settle();
                }
            }
        }
        free(buf);
    }
    return ESP_OK;
}

esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t upload_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 256 * 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File too large");
        return ESP_FAIL;
    }

    char *file_buf = (char*)malloc(total_len + 1);
    if (!file_buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    int cur_len = 0;
    while (cur_len < total_len) {
        int received = httpd_req_recv(req, file_buf + cur_len, total_len - cur_len);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
            free(file_buf);
            return ESP_FAIL;
        }
        cur_len += received;
    }
    file_buf[total_len] = '\0';

    if (s_fwu_busy) {
        free(file_buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "An update is already in progress");
        return ESP_FAIL;
    }

    // Whole image is now in RAM (Wi-Fi no longer needed for the burn). Hand off
    // to a worker task so this server task can keep pushing PROGRESS frames.
    fwu_job_t *job = (fwu_job_t *)malloc(sizeof(fwu_job_t));
    if (!job) {
        free(file_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }
    job->buf = file_buf;
    job->len = total_len;
    s_fwu_busy = true;
    if (xTaskCreate(fwu_worker_task, "fwu_worker", 8192, job, 5, NULL) != pdPASS) {
        s_fwu_busy = false;
        free(file_buf);
        free(job);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not start update task");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK");   // accepted - progress + final result stream over WebSocket
    return ESP_OK;
}

// Wireless ESP32 self-update. POST the raw firmware.bin body to
// /ota?token=<OTA_TOKEN>; it is streamed straight into the spare OTA slot and the
// device reboots into it. Used both by the web UI's "Flash ESP32" button and by
// `pio run -e ota -t upload` (a curl POST - see platformio.ini). Requires the
// two-slot partition table (partitions_ota.csv) - flash that once over USB first.
esp_err_t ota_post_handler(httpd_req_t *req) {
    char query[96];
    char token[48] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "token", token, sizeof(token)) != ESP_OK ||
        strcmp(token, OTA_TOKEN) != 0) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Bad or missing ?token=");
        return ESP_FAIL;
    }

    int total = req->content_len;
    if (total <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    const esp_partition_t *tgt = esp_ota_get_next_update_partition(NULL);
    if (!tgt) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
            "No spare OTA partition - flash partitions_ota.csv over USB first");
        return ESP_FAIL;
    }
    if ((size_t)total > tgt->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image larger than the OTA slot");
        return ESP_FAIL;
    }

    char start_msg[112];
    snprintf(start_msg, sizeof(start_msg),
             "[OTA]: receiving %d bytes into '%s' (0x%06X)...\n",
             total, tgt->label, (unsigned)tgt->address);
    ws_broadcast(start_msg, strlen(start_msg));

    esp_ota_handle_t ota = 0;
    esp_err_t err = esp_ota_begin(tgt, (size_t)total, &ota);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_FAIL;
    }

    char *buf = (char *)malloc(4096);
    if (!buf) {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    int got = 0, last_decile = -1, stalls = 0;
    while (got < total) {
        int want = total - got; if (want > 4096) want = 4096;
        int r = httpd_req_recv(req, buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            // Spotty Wi-Fi: retry a few times, but don't wedge the (single-task)
            // web server forever on a client that vanished mid-upload.
            if (++stalls > 12) {   // ~12 * 5 s recv timeout
                free(buf); esp_ota_abort(ota);
                const char *am = "[OTA]: aborted - upload stalled (client gone / Wi-Fi dropped)\n";
                ws_broadcast(am, strlen(am));
                return ESP_FAIL;
            }
            continue;
        }
        stalls = 0;
        if (r <= 0) { free(buf); esp_ota_abort(ota); return ESP_FAIL; }
        if (esp_ota_write(ota, buf, r) != ESP_OK) {
            free(buf); esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "flash write failed");
            return ESP_FAIL;
        }
        got += r;
        int decile = (int)((int64_t)got * 10 / total);
        if (decile != last_decile) {
            last_decile = decile;
            char m[48];
            snprintf(m, sizeof(m), "[OTA]: %d%%\n", decile * 10);
            ws_broadcast(m, strlen(m));
        }
    }
    free(buf);

    err = esp_ota_end(ota);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
            err == ESP_ERR_OTA_VALIDATE_FAILED ? "image failed validation" : esp_err_to_name(err));
        return ESP_FAIL;
    }
    err = esp_ota_set_boot_partition(tgt);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK - ESP32 rebooting into the new image\n");
    { const char *m = "[OTA]: image OK, rebooting into it now...\n"; ws_broadcast(m, strlen(m)); }
    vTaskDelay(pdMS_TO_TICKS(600));   // let the HTTP response + WS line flush
    esp_restart();
    return ESP_OK;   // not reached
}

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.stack_size = 8192;   // esp_ota_write runs in this task

    httpd_uri_t uri_get = {};
    uri_get.uri      = "/";
    uri_get.method   = HTTP_GET;
    uri_get.handler  = root_get_handler;

    httpd_uri_t uri_ws = {};
    uri_ws.uri          = "/ws";
    uri_ws.method       = HTTP_GET;
    uri_ws.handler      = ws_handler;
    uri_ws.is_websocket = true;

    httpd_uri_t uri_upload = {};
    uri_upload.uri      = "/upload";
    uri_upload.method   = HTTP_POST;
    uri_upload.handler  = upload_post_handler;

    httpd_uri_t uri_ota = {};
    uri_ota.uri      = "/ota";
    uri_ota.method   = HTTP_POST;
    uri_ota.handler  = ota_post_handler;

    if (httpd_start(&server_handle, &config) == ESP_OK) {
        httpd_register_uri_handler(server_handle, &uri_get);
        httpd_register_uri_handler(server_handle, &uri_ws);
        httpd_register_uri_handler(server_handle, &uri_upload);
        httpd_register_uri_handler(server_handle, &uri_ota);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Retry the primary AP indefinitely. This runs both before the initial
        // connect succeeds and every time the link is later lost, and it stays
        // registered after the AP fallback (see wifi_init_sta_with_fallback),
        // so WIFI_STA_SSID is retried forever - the fallback AP is never a dead
        // end. esp_wifi_connect() blocks internally on a scan (~1-2 s) when the
        // AP is absent, which naturally throttles this loop.
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *) event_data;
        ESP_LOGW(TAG, "STA disconnected (reason %d) - retrying '%s'...",
                 d ? d->reason : -1, WIFI_STA_SSID);
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected to Wi-Fi! Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta_with_fallback(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t sta_config = {};
    strcpy((char*)sta_config.sta.ssid, WIFI_STA_SSID);
    strcpy((char*)sta_config.sta.password, WIFI_STA_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    // This bridge is mains-powered (no battery to protect), and gate-state /
    // beam-edge latency matters a lot more here than radio power draw. Modem
    // sleep (the ESP-IDF default) delays TX/RX until the next DTIM beacon,
    // which is the most likely explanation for state updates and beam edges
    // occasionally arriving in Home Assistant bunched together instead of
    // individually. Disable it outright.
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "Attempting connection to '%s' (30-second timeout)...", WIFI_STA_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(30000)
    );

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "Could not reach '%s' within 30 s. Bringing up fallback AP '%s'; "
                      "the station stays active and keeps retrying '%s' in the background.",
                 WIFI_STA_SSID, WIFI_AP_SSID, WIFI_STA_SSID);

        // Deliberately do NOT unregister the handlers or esp_wifi_stop() here.
        // The WIFI_EVENT_STA_DISCONNECTED -> esp_wifi_connect() loop in
        // wifi_event_handler must keep running so the ESP32 re-joins WIFI_STA_SSID
        // automatically whenever it comes back (e.g. router rebooting after a
        // power cut). (void) the now-unused unregister handles.
        (void) instance_any_id;
        (void) instance_got_ip;

        esp_netif_create_default_wifi_ap();

        wifi_config_t ap_config = {};
        strcpy((char*)ap_config.ap.ssid, WIFI_AP_SSID);
        strcpy((char*)ap_config.ap.password, WIFI_AP_PASS);
        ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
        ap_config.ap.channel = 1;
        ap_config.ap.max_connection = 4;
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

        // APSTA, not AP: the single radio serves the fallback AP while the
        // station half keeps hunting for WIFI_STA_SSID. Switching mode here does
        // not stop the STA or its in-flight reconnect attempts. Single-radio
        // caveat: once the STA finally associates, the SoftAP hops to that AP's
        // channel and any clients on 192.168.4.1 briefly drop and must rejoin.
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

        ESP_LOGI(TAG, "Fallback AP running: SSID '%s', IP 192.168.4.1 (still retrying '%s')",
                 WIFI_AP_SSID, WIFI_STA_SSID);
    }
}

void init_peripherals(void) {
    // Leave RST/TEST Hi-Z at boot - the MSP430's own pulls (R29 47k up on RST,
    // internal pull-down on TEST) run the app, and a JTAG/SBW tool can use pins
    // 91/96 without the ESP32 fighting it. enter_bsl_seq() claims them push-pull
    // only for the duration of a BSL session; reset_msp430() hands them back.
    bsl_pins_release();

    set_uart_mode(false); // app console framing (APP_UART_BAUD 8N1)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    bsl_uart_attach(false); // park PIN_TX/PIN_RX, route UART_PORT to PIN_APP_TX/PIN_APP_RX

    s_uart_mux   = xSemaphoreCreateMutex();
    s_status_mux = xSemaphoreCreateMutex();

    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&tcfg, &s_tsens) == ESP_OK) {
        temperature_sensor_enable(s_tsens);
    } else {
        s_tsens = NULL;
    }
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    cycles_load();   // restore the gate cycle counter (NVS namespace "gatestats")
    init_peripherals();
    wifi_init_sta_with_fallback();
    start_webserver();

    xTaskCreate(uart_rx_task, "uart_rx_task", 8192, NULL, 10, NULL);   // 2KB rx buffer + STATUS parse frames
                                                                      // + inline beam MQTT publishes
    xTaskCreate(time_sync_task, "time_sync_task", 3072, NULL, 5, NULL);
    xTaskCreate(telemetry_task, "telemetry_task", 3072, NULL, 4, NULL);
    xTaskCreate(mqtt_gate_task, "mqtt_gate", 8192, NULL, 4, &s_mqtt_task);   // JSON build + discovery
}