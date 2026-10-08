/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double latitude;
    double longitude;
    bool valid;
} gps_data_t;

/**
 * @brief Initialize the ATGM336H GPS UART interface.
 *
 * Uses UART1: RX on GPIO38 (module TX), TX on GPIO39 (module RX), 9600 baud.
 *
 * @return true on success, false on failure.
 */
bool gps_init(void);

/**
 * @brief Poll the GPS UART and return the latest parsed position.
 *
 * Reads available bytes, accumulates complete NMEA lines, and updates the
 * cached position when a valid GGA sentence is received.
 *
 * @return The latest GPS data; valid=false if no fix has been seen yet.
 */
gps_data_t gps_read(void);
