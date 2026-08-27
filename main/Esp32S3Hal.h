/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef ESP32S3_HAL_H
#define ESP32S3_HAL_H

// include RadioLib
#include <RadioLib.h>

// ESP-IDF headers
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// Arduino-style macros used by RadioLib
#define LOW                         (0x0)
#define HIGH                        (0x1)
#define INPUT                       (0x01)
#define OUTPUT                      (0x03)
#define RISING                      (0x01)
#define FALLING                     (0x02)

/*!
  \class Esp32S3Hal
  \brief Hardware abstraction layer for RadioLib on ESP32-S3 using ESP-IDF SPI driver.
*/
class Esp32S3Hal : public RadioLibHal {
  public:
    Esp32S3Hal(spi_host_device_t host, gpio_num_t sck, gpio_num_t miso, gpio_num_t mosi);

    void init() override;
    void term() override;

    void pinMode(uint32_t pin, uint32_t mode) override;
    void digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override;
    void detachInterrupt(uint32_t interruptNum) override;

    void delay(RadioLibTime_t ms) override;
    void delayMicroseconds(RadioLibTime_t us) override;
    RadioLibTime_t millis() override;
    RadioLibTime_t micros() override;
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override;

    void spiBegin() override;
    void spiBeginTransaction() override;
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
    void spiEndTransaction() override;
    void spiEnd() override;

  private:
    spi_host_device_t spiHost;
    gpio_num_t spiSCK;
    gpio_num_t spiMISO;
    gpio_num_t spiMOSI;
    spi_device_handle_t spiHandle;
    static bool isrServiceInstalled;
};

#endif
