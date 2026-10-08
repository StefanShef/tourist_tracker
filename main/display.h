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

/**
 * @brief Show compass heading and GPS coordinates on the display.
 *
 * Draws a compass rose on the left half (arrow points North) and GPS latitude /
 * longitude on the right half. Everything is composed into one frame and sent
 * to the display in a single transfer.
 *
 * @param heading_deg Compass heading in degrees (0-360). Negative value means
 *                    no valid magnetometer reading.
 * @param latitude GPS latitude in decimal degrees.
 * @param longitude GPS longitude in decimal degrees.
 * @param gps_valid true if the coordinates are valid.
 */
/**
 * @brief Draw the compass + coordinates sensor view.
 *
 * @param heading_deg Compass heading (0-360) or negative to show "No IMU".
 * @param latitude    Coordinate latitude.
 * @param longitude   Coordinate longitude.
 * @param gps_valid   true if the coordinates are usable.
 * @param from_phone  true if coordinates came from the BLE phone link
 *                    (tag "BLE" is drawn), false for the GPS module ("GPS").
 */
void display_show_sensors(float heading_deg, double latitude, double longitude,
                          bool gps_valid, bool from_phone);

#ifdef __cplusplus
}
#endif
