/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the ICM20948 IMU and its embedded AK09916 magnetometer.
 *
 * Uses I2C1 on GPIO15 (SCL) / GPIO16 (SDA). Puts the ICM20948 into I2C bypass
 * mode so the AK09916 is directly accessible at address 0x0C.
 *
 * @return true on success, false on failure.
 */
bool icm20948_init(void);

/**
 * @brief Read the latest magnetometer sample.
 *
 * @param x Raw X-axis value.
 * @param y Raw Y-axis value.
 * @param z Raw Z-axis value.
 * @return true if new data was read, false otherwise.
 */
bool icm20948_read_mag(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Read the latest accelerometer sample from the ICM20948 itself.
 *
 * @param x Raw X-axis value.
 * @param y Raw Y-axis value.
 * @param z Raw Z-axis value.
 * @return true on success, false otherwise.
 */
bool icm20948_read_accel(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Magnetometer calibration: hard-iron offset + soft-iron scale.
 */
typedef struct {
    float offset[3];    //!< hard-iron bias per axis, raw LSB
    float scale[3];     //!< soft-iron gain per axis (1.0 = no correction)
} icm20948_mag_cal_t;

/**
 * @brief Start a field calibration run (about 25 s).
 *
 * While it is active, rotate the board through all orientations
 * (figure-eight / sphere). Feeding get_heading() collects samples; the run
 * finishes automatically and the resulting coefficients are logged.
 */
void icm20948_cal_start(void);

/** @return true while a calibration run is collecting samples. */
bool icm20948_cal_active(void);

/**
 * @brief Stop collecting and bake the recorded min/max extremes into the
 *        active calibration. Safe to call when no run is active.
 *
 * @return true if a valid calibration was computed.
 */
bool icm20948_cal_finish(void);

/**
 * @brief Compute a tilt-compensated compass heading.
 *
 * Pitch/roll come from the accelerometer; the magnetometer is corrected with
 * the current calibration (identity until a calibration run finishes).
 *
 * @return Heading in degrees (0-360, North = 0), or -1.0f if no data.
 */
float icm20948_get_heading(void);
