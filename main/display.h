/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SSD1306 OLED via u8g2.
 *
 * Powers the display through the Heltec V4 Vext switch (GPIO36 by default,
 * falling back to GPIO40 on the V4-R8 revision), resets it, and starts the
 * I2C/u8g2 driver.
 *
 * @return true on success, false on failure.
 */
bool display_init(void);

/**
 * @brief Clear the display and send the empty frame to the screen.
 */
void display_clear(void);

/**
 * @brief Print a UTF-8 string on the display and refresh.
 *
 * @param x Horizontal position in pixels.
 * @param y Baseline vertical position in pixels.
 * @param str Null-terminated UTF-8 string.
 */
void display_print(int x, int y, const char *str);

/**
 * @brief Draw an XBM bitmap on the display and refresh.
 *
 * @param x Horizontal position in pixels.
 * @param y Vertical position in pixels.
 * @param width Bitmap width in pixels.
 * @param height Bitmap height in pixels.
 * @param xbm Bitmap data in XBM format.
 */
void display_draw_xbm(int x, int y, int width, int height, const uint8_t *xbm);

#ifdef __cplusplus
}
#endif
