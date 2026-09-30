// secrets.example.h - template for the bridge's credentials.
//
// Copy this file to secrets.h (same folder) and fill in your own values.
// secrets.h is gitignored, so real credentials never get committed.
#pragma once

// Home Wi-Fi network the bridge joins.
#define WIFI_STA_SSID     "your-wifi-ssid"
#define WIFI_STA_PASS     "your-wifi-password"

// Fallback access point. If WIFI_STA_SSID can't be reached within 30s the
// bridge also starts its own AP at 192.168.4.1, and keeps retrying the home
// network in the background. WPA2 needs a password of at least 8 characters.
#define WIFI_AP_SSID      "MSP430_Flasher"
#define WIFI_AP_PASS      "change-me-too"

// Shared secret for the ESP32's own wireless update: POST /ota?token=<OTA_TOKEN>.
// For `pio run -e ota -t upload`, set the GATE_OTA_TOKEN environment variable
// to the same value (platformio.ini reads it from there).
#define OTA_TOKEN         "change-me"

// MQTT broker used for the Home Assistant integration.
#define MQTT_URI          "mqtt://192.168.1.10:1883"
#define MQTT_USERNAME     "gate-bridge"
#define MQTT_PASSWORD     "change-me"

// The bridge's own web UI. Home Assistant shows it as the "Visit" link on the
// device page.
#define BRIDGE_CONFIG_URL "http://192.168.1.50/"
