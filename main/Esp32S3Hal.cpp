/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "Esp32S3Hal.h"
#include <string.h>

// Callback table for GPIO ISR dispatch.
static void (*isrCallbacks[GPIO_NUM_MAX])(void) = { nullptr };

bool Esp32S3Hal::isrServiceInstalled = false;

Esp32S3Hal::Esp32S3Hal(spi_host_device_t host, gpio_num_t sck, gpio_num_t miso, gpio_num_t mosi)
  : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
    spiHost(host), spiSCK(sck), spiMISO(miso), spiMOSI(mosi), spiHandle(nullptr) {
}

void Esp32S3Hal::init() {
    spiBegin();
}

void Esp32S3Hal::term() {
    spiEnd();
}

void Esp32S3Hal::pinMode(uint32_t pin, uint32_t mode) {
    if (pin == RADIOLIB_NC) {
        return;
    }

    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = (gpio_mode_t)mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
}

void Esp32S3Hal::digitalWrite(uint32_t pin, uint32_t value) {
    if (pin == RADIOLIB_NC) {
        return;
    }

    gpio_set_level((gpio_num_t)pin, value);
}

uint32_t Esp32S3Hal::digitalRead(uint32_t pin) {
    if (pin == RADIOLIB_NC) {
        return 0;
    }

    return gpio_get_level((gpio_num_t)pin);
}

static void IRAM_ATTR radiolib_isr_handler(void* arg) {
    uint32_t pin = (uint32_t)arg;
    if (pin < GPIO_NUM_MAX && isrCallbacks[pin] != nullptr) {
        isrCallbacks[pin]();
    }
}

void Esp32S3Hal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) {
    if (interruptNum == RADIOLIB_NC) {
        return;
    }

    if (!isrServiceInstalled) {
        gpio_install_isr_service(0);
        isrServiceInstalled = true;
    }

    isrCallbacks[interruptNum] = interruptCb;
    gpio_set_intr_type((gpio_num_t)interruptNum,
                       (mode == GpioInterruptRising) ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add((gpio_num_t)interruptNum, radiolib_isr_handler, (void*)interruptNum);
}

void Esp32S3Hal::detachInterrupt(uint32_t interruptNum) {
    if (interruptNum == RADIOLIB_NC) {
        return;
    }

    gpio_isr_handler_remove((gpio_num_t)interruptNum);
    gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    isrCallbacks[interruptNum] = nullptr;
}

void Esp32S3Hal::delay(RadioLibTime_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void Esp32S3Hal::delayMicroseconds(RadioLibTime_t us) {
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (uint64_t)us) {
        // busy wait
    }
}

RadioLibTime_t Esp32S3Hal::millis() {
    return (RadioLibTime_t)(esp_timer_get_time() / 1000ULL);
}

RadioLibTime_t Esp32S3Hal::micros() {
    return (RadioLibTime_t)esp_timer_get_time();
}

long Esp32S3Hal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) {
    if (pin == RADIOLIB_NC) {
        return 0;
    }

    this->pinMode(pin, INPUT);
    RadioLibTime_t start = this->micros();

    while (this->digitalRead(pin) == state) {
        if ((this->micros() - start) > timeout) {
            return 0;
        }
    }

    RadioLibTime_t pulseStart = this->micros();
    while (this->digitalRead(pin) != state) {
        if ((this->micros() - start) > timeout) {
            return 0;
        }
    }

    return (long)(this->micros() - pulseStart);
}

void Esp32S3Hal::spiBegin() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = spiMOSI,
        .miso_io_num = spiMISO,
        .sclk_io_num = spiSCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 256,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(spiHost, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 2000000,
        .input_delay_ns = 0,
        .spics_io_num = -1,  // NSS is controlled manually by RadioLib
        .flags = 0,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(spiHost, &devcfg, &spiHandle));
}

void Esp32S3Hal::spiBeginTransaction() {
    // Nothing to do: CS is handled manually.
}

void Esp32S3Hal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
    if (len == 0) {
        return;
    }

    spi_transaction_t trans = {};
    trans.length = len * 8;
    trans.tx_buffer = out;
    trans.rx_buffer = in;
    ESP_ERROR_CHECK(spi_device_transmit(spiHandle, &trans));
}

void Esp32S3Hal::spiEndTransaction() {
    // Nothing to do.
}

void Esp32S3Hal::spiEnd() {
    if (spiHandle != nullptr) {
        spi_bus_remove_device(spiHandle);
        spiHandle = nullptr;
    }
    spi_bus_free(spiHost);
}
