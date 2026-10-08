/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "phone_gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "NimBLEDevice.h"
#include "esp_log.h"
#include "esp_timer.h"

// Nordic UART Service (NUS) — transparent serial port over BLE.
#define BLE_DEVICE_NAME     "Heltec-V4"
#define NUS_SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID         "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // phone -> device
#define NUS_TX_UUID         "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // device -> phone (notify)

#define RX_LINE_MAX         160     // assembled RX line buffer

static const char *TAG = "phone_gps";

static NimBLECharacteristic *s_tx_char = NULL;
static volatile bool s_connected = false;

// Written from the BLE host task, read from the display task. gps_data_t is
// small and the races here are harmless (a slightly stale fix on screen).
static gps_data_t s_fix = {0.0, 0.0, false};
static int64_t s_fix_time_us = 0;

// RX line assembly (tolerant to fragmented writes; line ends on \n or \r).
static char s_rx_buf[RX_LINE_MAX];
static size_t s_rx_len = 0;

// ---------------------------------------------------------------------------
// Parsing: "GPS:lat,lng,alt,acc" or {"lat":..,"lng":..,"alt":..,"acc":..}
// ---------------------------------------------------------------------------

static bool parse_csv(const char *str, double *lat, double *lng)
{
    float la = 0.0f, ln = 0.0f, alt = 0.0f, acc = 0.0f;
    if (sscanf(str, "GPS:%f,%f,%f,%f", &la, &ln, &alt, &acc) >= 2) {
        *lat = la;
        *lng = ln;
        return true;
    }
    return false;
}

static bool json_get_number(const char *json, const char *key, double *out)
{
    char pattern[16];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(json, pattern);
    if (p == NULL) {
        return false;
    }
    *out = strtod(p + strlen(pattern), NULL);
    return true;
}

static bool parse_json(const char *str, double *lat, double *lng)
{
    double la = 0.0, ln = 0.0;
    if (!json_get_number(str, "lat", &la) || !json_get_number(str, "lng", &ln)) {
        return false;
    }
    *lat = la;
    *lng = ln;
    return true;
}

static void handle_line(const char *line)
{
    double lat = 0.0, lng = 0.0;
    bool ok = false;

    if (strncmp(line, "GPS:", 4) == 0) {
        ok = parse_csv(line, &lat, &lng);
    } else if (line[0] == '{') {
        ok = parse_json(line, &lat, &lng);
    }

    if (!ok || lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0) {
        ESP_LOGW(TAG, "unparsed RX data: %s", line);
        return;
    }

    s_fix.latitude = lat;
    s_fix.longitude = lng;
    s_fix.valid = true;
    s_fix_time_us = esp_timer_get_time();
    ESP_LOGI(TAG, "phone fix: lat=%.6f lng=%.6f", lat, lng);

    if (s_tx_char != NULL && s_connected) {
        s_tx_char->setValue("OK\n");
        s_tx_char->notify();
    }
}

// ---------------------------------------------------------------------------
// BLE callbacks
// ---------------------------------------------------------------------------

class NusRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
    {
        (void)connInfo;
        std::string value = pChar->getValue();
        ESP_LOGI(TAG, "RX %d bytes: %.*s", (int)value.size(), (int)value.size(), value.c_str());

        for (char c : value) {
            if (c == '\n' || c == '\r') {
                flush_line();
            } else if (s_rx_len < sizeof(s_rx_buf) - 1) {
                s_rx_buf[s_rx_len++] = c;
            }
        }
        // A write without a trailing newline is a complete message on its own.
        flush_line();
    }

    static void flush_line(void)
    {
        if (s_rx_len > 0) {
            s_rx_buf[s_rx_len] = '\0';
            handle_line(s_rx_buf);
            s_rx_len = 0;
        }
    }
};

class NusServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
    {
        (void)pServer;
        (void)connInfo;
        s_connected = true;
        ESP_LOGI(TAG, ">>> Phone Connected!");
    }

    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
    {
        (void)pServer;
        (void)connInfo;
        s_connected = false;
        ESP_LOGI(TAG, ">>> Phone Disconnected! (reason %d)", reason);
        // Re-advertise so the phone can reconnect without a board reset.
        NimBLEDevice::startAdvertising();
    }
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool phone_gps_init(void)
{
    NimBLEDevice::init(BLE_DEVICE_NAME);

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new NusServerCallbacks());

    NimBLEService *nus = server->createService(NimBLEUUID(NUS_SERVICE_UUID));

    NimBLECharacteristic *rx = nus->createCharacteristic(
        NimBLEUUID(NUS_RX_UUID),
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new NusRxCallbacks());

    // NOTIFY property makes the stack add the CCCD (0x2902) descriptor.
    s_tx_char = nus->createCharacteristic(NimBLEUUID(NUS_TX_UUID),
                                          NIMBLE_PROPERTY::NOTIFY);

    nus->start();

    NimBLEDevice::startAdvertising();
    ESP_LOGI(TAG, "BLE advertising started, name \"%s\"", BLE_DEVICE_NAME);
    return true;
}

bool phone_gps_connected(void)
{
    return s_connected;
}

gps_data_t phone_gps_read(void)
{
    return s_fix;
}

uint32_t phone_gps_age_ms(void)
{
    if (!s_fix.valid) {
        return UINT32_MAX;
    }
    return (uint32_t)((esp_timer_get_time() - s_fix_time_us) / 1000);
}
