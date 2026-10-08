/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "gps.h"    // gps_data_t

/**
 * @brief Start the BLE server emulating the Nordic UART Service (NUS).
 *
 * Advertises as "Heltec-V4" and accepts connections from any phone app
 * compatible with the NUS profile (nRF Connect, Serial Bluetooth Terminal).
 *
 * @return true on success, false on failure.
 */
bool phone_gps_init(void);

/**
 * @brief Check whether a phone is currently connected.
 */
bool phone_gps_connected(void);

/**
 * @brief Return the latest coordinates received from the phone.
 *
 * @return Latest fix; valid=false if nothing was received yet.
 */
gps_data_t phone_gps_read(void);

/**
 * @brief Age of the last received fix.
 *
 * @return Milliseconds since the last fix, or UINT32_MAX if none.
 */
uint32_t phone_gps_age_ms(void);
