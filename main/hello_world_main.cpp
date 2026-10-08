/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <RadioLib.h>
#include "Esp32S3Hal.h"
#include "display.h"
#include "icm20948.h"
#include "gps.h"
#include "phone_gps.h"

static const char *TAG = "HELTEC_V4";

#define HEART_WIDTH  32
#define HEART_HEIGHT 32
static const uint8_t heart32x32_bits[] = {
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x06,
    0x00, 0x00, 0x00, 0x06,
    0x00, 0x00, 0x00, 0x06,
    0x00, 0x00, 0x80, 0x19,
    0x00, 0x00, 0x80, 0x19,
    0x00, 0x00, 0x60, 0x18,
    0x00, 0x00, 0x60, 0x18,
    0x00, 0x00, 0x60, 0x18,
    0x00, 0x00, 0x18, 0x06,
    0x00, 0x00, 0x18, 0x06,
    0x00, 0x00, 0x8E, 0x01,
    0x00, 0x00, 0x86, 0x01,
    0x00, 0x00, 0x86, 0x01,
    0x38, 0x80, 0x61, 0x00,
    0x38, 0x80, 0x61, 0x00,
    0x6C, 0xE0, 0x18, 0x00,
    0xEE, 0x60, 0x18, 0x00,
    0xEE, 0x60, 0x18, 0x00,
    0x8E, 0x19, 0x06, 0x00,
    0x8E, 0x19, 0x06, 0x00,
    0x38, 0x8E, 0x01, 0x00,
    0x38, 0x86, 0x01, 0x00,
    0x38, 0x86, 0x01, 0x00,
    0xE0, 0x60, 0x00, 0x00,
    0xE0, 0x60, 0x00, 0x00,
    0x80, 0x39, 0x00, 0x00,
    0x80, 0x19, 0x00, 0x00,
    0x80, 0x1B, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};

// Heltec V4 (ESP32-S3 + SX1262 + GC1109 FEM) pin mapping
#define LORA_SCK    GPIO_NUM_9
#define LORA_MISO   GPIO_NUM_11
#define LORA_MOSI   GPIO_NUM_10
#define LORA_NSS    GPIO_NUM_8
#define LORA_DIO1   GPIO_NUM_14
#define LORA_RST    GPIO_NUM_12
#define LORA_BUSY   GPIO_NUM_13
#define FEM_EN      GPIO_NUM_2

// RadioLib HAL and module instances
Esp32S3Hal hal(SPI2_HOST, LORA_SCK, LORA_MISO, LORA_MOSI);
Module radioModule(&hal, LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
SX1262 radio(&radioModule);

static bool s_display_ok = false;
static bool s_imu_ok = false;
static bool s_gps_ok = false;
static bool s_ble_ok = false;

static void sensor_display_task(void *arg)
{
    (void)arg;

    while (true) {
        float heading = s_imu_ok ? icm20948_get_heading() : -1.0f;

        // Coordinates: phone over BLE when connected and fresh, else GPS module.
        gps_data_t gps = {0.0, 0.0, false};
        bool from_phone = false;
        if (s_ble_ok && phone_gps_connected() && phone_gps_age_ms() < 10000) {
            gps = phone_gps_read();
            from_phone = true;
        } else if (s_gps_ok) {
            gps = gps_read();
        }

        if (s_display_ok) {
            display_show_sensors(heading, gps.latitude, gps.longitude, gps.valid, from_phone);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== BOOT: app_main start ===");

    // Initialize the onboard OLED and draw a greeting.
    // The display module handles Vext (GPIO36/GPIO40) and I2C setup internally.
    s_display_ok = display_init();
    ESP_LOGI(TAG, "=== BOOT: display_init done, ok=%d ===", s_display_ok);

    if (!s_display_ok) {
        ESP_LOGE(TAG, "OLED display init failed on both Vext pins");
        ESP_LOGE(TAG, "Check: board model / OLED SDA,SCL pins / pull-ups / display hardware");
    } else {
        display_clear();
        display_print(15, 20, "Привет, Саня!");
        display_draw_xbm(48, 26, HEART_WIDTH, HEART_HEIGHT, heart32x32_bits);
        ESP_LOGI(TAG, "OLED greeting drawn");
    }

    // Start the BLE NUS server so a phone can push its coordinates.
    s_ble_ok = phone_gps_init();
    ESP_LOGI(TAG, "=== BOOT: phone_gps_init done, ok=%d ===", s_ble_ok);

    // Initialize the ICM20948 IMU (magnetometer for the compass).
    s_imu_ok = icm20948_init();
    ESP_LOGI(TAG, "=== BOOT: icm20948_init done, ok=%d ===", s_imu_ok);

    // Initialize the ATGM336H GPS.
    s_gps_ok = gps_init();
    ESP_LOGI(TAG, "=== BOOT: gps_init done, ok=%d ===", s_gps_ok);

    // Start the periodic sensor + display update task.
    xTaskCreate(sensor_display_task, "sensor_display", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "=== BOOT: sensor_display task created ===");

    // Enable the GC1109 front-end amplifier
    gpio_reset_pin(FEM_EN);
    gpio_set_direction(FEM_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(FEM_EN, 1);
    ESP_LOGI(TAG, "FEM enabled on GPIO %d", FEM_EN);

    ESP_LOGI(TAG, "Initializing SX1262 ...");

    // LoRa parameters: 868 MHz, 125 kHz BW, SF9, CR 4/5, private sync word, 14 dBm, preamble 8
    int state = radio.begin(868.0, 125.0, 9, 5,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                            14, 8, 1.6, false);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio init failed, code %d", state);
        return;
    }
    ESP_LOGI(TAG, "Radio init OK");

    uint32_t counter = 0;
    char txBuf[64];

    while (true) {
        snprintf(txBuf, sizeof(txBuf), "Hello Heltec V4! #%lu", (unsigned long)counter++);
        ESP_LOGI(TAG, "Transmitting: %s", txBuf);

        state = radio.transmit((uint8_t*)txBuf, strlen(txBuf));
        if (state == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "TX OK");
        } else {
            ESP_LOGE(TAG, "TX failed, code %d", state);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
