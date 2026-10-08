/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "gps.h"

#include <string.h>
#include <stdlib.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define GPS_UART_NUM        UART_NUM_1
#define GPS_RX_PIN          GPIO_NUM_38
#define GPS_TX_PIN          GPIO_NUM_39
#define GPS_BAUD_RATE       9600
#define GPS_BUF_SIZE        1024
#define GPS_LINE_MAX        256

static const char *TAG = "gps";

static char s_line_buf[GPS_LINE_MAX];
static int s_line_idx = 0;
static gps_data_t s_last_data = {0.0, 0.0, false};

static bool gps_parse_gga(const char *sentence, gps_data_t *out)
{
    // $GxGGA,hhmmss.ss,llll.ll,a,ddddd.dd,a,q,nn,p.p,h.h,M,g.g,M,x.x,xxxx*hh
    const char *p = strchr(sentence, ',');
    if (p == NULL) {
        return false;
    }
    p++;

    // Skip UTC time.
    p = strchr(p, ',');
    if (p == NULL) {
        return false;
    }
    p++;

    char lat_str[16];
    int i = 0;
    while (*p != ',' && *p != '\0' && i < (int)sizeof(lat_str) - 1) {
        lat_str[i++] = *p++;
    }
    lat_str[i] = '\0';
    if (*p != ',') {
        return false;
    }
    p++;

    char lat_dir = *p;
    p += 2;

    char lon_str[16];
    i = 0;
    while (*p != ',' && *p != '\0' && i < (int)sizeof(lon_str) - 1) {
        lon_str[i++] = *p++;
    }
    lon_str[i] = '\0';
    if (*p != ',') {
        return false;
    }
    p++;

    char lon_dir = *p;
    p += 2;

    int fix = atoi(p);
    if (fix == 0) {
        out->valid = false;
        return true;
    }

    double lat_raw = atof(lat_str);
    double lon_raw = atof(lon_str);

    int lat_deg = (int)(lat_raw / 100.0);
    double lat_min = lat_raw - (double)lat_deg * 100.0;
    out->latitude = lat_deg + lat_min / 60.0;
    if (lat_dir == 'S') {
        out->latitude = -out->latitude;
    }

    int lon_deg = (int)(lon_raw / 100.0);
    double lon_min = lon_raw - (double)lon_deg * 100.0;
    out->longitude = lon_deg + lon_min / 60.0;
    if (lon_dir == 'W') {
        out->longitude = -out->longitude;
    }

    out->valid = true;
    return true;
}

bool gps_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t err = uart_param_config(GPS_UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return false;
    }
    err = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return false;
    }
    err = uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "GPS UART initialized on RX=%d TX=%d @ %d baud", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);
    return true;
}

gps_data_t gps_read(void)
{
    uint8_t ch;
    while (uart_read_bytes(GPS_UART_NUM, &ch, 1, 0) > 0) {
        if (ch == '\n') {
            s_line_buf[s_line_idx] = '\0';
            if (strncmp(s_line_buf, "$GPGGA", 6) == 0 || strncmp(s_line_buf, "$GNGGA", 6) == 0) {
                gps_parse_gga(s_line_buf, &s_last_data);
            }
            s_line_idx = 0;
        } else if (s_line_idx < GPS_LINE_MAX - 1) {
            s_line_buf[s_line_idx++] = (char)ch;
        } else {
            // Line too long, reset.
            s_line_idx = 0;
        }
    }
    return s_last_data;
}
