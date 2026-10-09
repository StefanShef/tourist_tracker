/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the ICM20948 IMU, the AK09916 magnetometer and the DMP.
 *
 * Uses I2C1 on GPIO15 (SCL) / GPIO16 (SDA), driven through the vendored
 * SparkFun ICM-20948 C core. The DMP is configured for 9-axis orientation
 * (Quat9). Starts a ~25 s magnetometer calibration run — rotate the board
 * through all orientations after boot.
 *
 * @return true on success, false on failure.
 */
bool icm20948_init(void);

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
 * @brief Compute the compass heading.
 *
 * Primary source: the DMP 9-axis orientation quaternion (accel + gyro +
 * magnetometer fusion). Fallback while the fusion warms up: tilt-compensated
 * accel + compass packets with the current hard/soft-iron calibration.
 *
 * @return Heading in degrees (0-360, North = 0), or -1.0f if no data.
 */
float icm20948_get_heading(void);
