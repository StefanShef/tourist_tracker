/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "display.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "u8g2.h"
#include "esp32_hw_i2c.h"

// Heltec V4 onboard OLED SSD1306 pin mapping
#define OLED_SDA            GPIO_NUM_17
#define OLED_SCL            GPIO_NUM_18
#define OLED_RST            GPIO_NUM_21
#define OLED_I2C_PORT       I2C_NUM_0
#define OLED_ADDR_7BIT      0x3C
#define OLED_ADDR_8BIT      (OLED_ADDR_7BIT << 1)

// Vext switch: active LOW. GPIO36 on most V4 boards, GPIO40 on V4-R8 revision.
#define VEXT_CTRL           GPIO_NUM_36
#define VEXT_CTRL_FALLBACK  GPIO_NUM_40

// Display rotation: change to U8G2_R2 if the image appears upside down.
#define DISPLAY_ROTATION    U8G2_R0

static const char *TAG = "display";
static u8g2_t u8g2;
static u8g2_esp32_i2c_ctx_t i2c_ctx;
static SemaphoreHandle_t display_mutex = NULL;

static void display_power_on(gpio_num_t vext_pin)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << vext_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    // Active LOW on Heltec V4.
    gpio_set_level(vext_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void display_reset(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << OLED_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    gpio_set_level(OLED_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(OLED_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void display_i2c_cleanup(void)
{
    if (i2c_ctx.bus_handle != NULL) {
        i2c_del_master_bus((i2c_master_bus_handle_t)i2c_ctx.bus_handle);
    }
    memset(&i2c_ctx, 0, sizeof(i2c_ctx));
}

static bool display_post_init(void)
{
    display_mutex = xSemaphoreCreateMutex();
    if (display_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create display mutex");
        return false;
    }

    // Cyrillic-aware font; also covers basic Latin.
    u8g2_SetFont(&u8g2, u8g2_font_8x13_t_cyrillic);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);

    return true;
}

static bool display_init_with_vext(gpio_num_t vext_pin)
{
    display_power_on(vext_pin);
    display_reset();

    i2c_ctx.cfg = (u8g2_esp32_i2c_config_t){
        .i2c_port = OLED_I2C_PORT,
        .sda_pin = OLED_SDA,
        .scl_pin = OLED_SCL,
        .clk_hz = 400000,
        .dev_addr_7bit = OLED_ADDR_7BIT,
        .timeout_ms = 1000,
        .reset_pin = U8G2_ESP32_PIN_UNUSED,
    };

    esp_err_t err = u8g2_esp32_i2c_set_default_context(&i2c_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "u8g2_esp32_i2c_set_default_context failed: %s", esp_err_to_name(err));
        return false;
    }

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, DISPLAY_ROTATION,
                                          u8x8_byte_esp32_hw_i2c,
                                          u8x8_gpio_and_delay_esp32_i2c);
    u8x8_SetI2CAddress(&u8g2.u8x8, OLED_ADDR_8BIT);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    return i2c_ctx.initialized == 1;
}

bool display_init(void)
{
    if (display_init_with_vext(VEXT_CTRL)) {
        return display_post_init();
    }

    ESP_LOGW(TAG, "OLED init failed with Vext GPIO %d, trying GPIO %d",
             VEXT_CTRL, VEXT_CTRL_FALLBACK);
    display_i2c_cleanup();

    if (display_init_with_vext(VEXT_CTRL_FALLBACK)) {
        return display_post_init();
    }

    return false;
}

void display_clear(void)
{
    if (display_mutex == NULL) {
        return;
    }

    xSemaphoreTake(display_mutex, portMAX_DELAY);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(display_mutex);
}

void display_print(int x, int y, const char *str)
{
    if (display_mutex == NULL) {
        return;
    }

    xSemaphoreTake(display_mutex, portMAX_DELAY);
    u8g2_DrawUTF8(&u8g2, x, y, str);
    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(display_mutex);
}

void display_draw_xbm(int x, int y, int width, int height, const uint8_t *xbm)
{
    if (display_mutex == NULL || xbm == NULL) {
        return;
    }

    xSemaphoreTake(display_mutex, portMAX_DELAY);
    u8g2_DrawXBM(&u8g2, x, y, width, height, xbm);
    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(display_mutex);
}

void display_show_sensors(float heading_deg, double latitude, double longitude,
                          bool gps_valid, bool from_phone)
{
    if (display_mutex == NULL) {
        return;
    }

    xSemaphoreTake(display_mutex, portMAX_DELAY);
    u8g2_ClearBuffer(&u8g2);

    // Compass on the left half of the display.
    const int cx = 32;
    const int cy = 32;
    const int r = 28;

    u8g2_DrawCircle(&u8g2, cx, cy, r, U8G2_DRAW_ALL);

    if (heading_deg >= 0.0f) {
        // Arrow points North: rotate by -heading so the arrow always shows where North is.
        float rad = (360.0f - heading_deg) * (float)M_PI / 180.0f;
        int tip_x = cx + (int)((r - 4) * sinf(rad));
        int tip_y = cy - (int)((r - 4) * cosf(rad));
        u8g2_DrawLine(&u8g2, cx, cy, tip_x, tip_y);

        // Arrowhead.
        int head_len = 6;
        int head_angle1_x = tip_x - (int)(head_len * sinf(rad + 0.6f));
        int head_angle1_y = tip_y + (int)(head_len * cosf(rad + 0.6f));
        int head_angle2_x = tip_x - (int)(head_len * sinf(rad - 0.6f));
        int head_angle2_y = tip_y + (int)(head_len * cosf(rad - 0.6f));
        u8g2_DrawLine(&u8g2, tip_x, tip_y, head_angle1_x, head_angle1_y);
        u8g2_DrawLine(&u8g2, tip_x, tip_y, head_angle2_x, head_angle2_y);

        // Cardinal mark.
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
        u8g2_DrawStr(&u8g2, cx - 2, cy - r + 8, "N");
    } else {
        u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
        u8g2_DrawStr(&u8g2, cx - 18, cy + 4, "No IMU");
    }

    // Coordinate source tag in the top-right corner.
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    u8g2_DrawStr(&u8g2, 100, 8, from_phone ? "BLE" : "GPS");

    // GPS coordinates on the right half.
    char buf[32];
    u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
    if (gps_valid) {
        snprintf(buf, sizeof(buf), "Lat:%.5f", latitude);
        u8g2_DrawStr(&u8g2, 68, 24, buf);
        snprintf(buf, sizeof(buf), "Lon:%.5f", longitude);
        u8g2_DrawStr(&u8g2, 68, 44, buf);
    } else {
        u8g2_DrawStr(&u8g2, 68, 32, "No fix");
    }

    u8g2_SendBuffer(&u8g2);
    xSemaphoreGive(display_mutex);
}
