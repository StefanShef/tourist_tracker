# AGENTS.md

This file contains project-specific context for AI coding agents. The project uses English for all source comments and documentation.

## Project Overview

This is an **ESP-IDF** firmware project named `hello_world`, adapted from the ESP-IDF hello_world example to drive the **Heltec V4** board (ESP32-S3 + SX1262 LoRa transceiver + GC1109 RF front-end + onboard SSD1306 128x64 I2C OLED).

On boot the firmware:

1. Powers the OLED via the Vext switch (tries GPIO 36, falls back to GPIO 40 for the V4-R8 revision), initializes the SSD1306, and draws the text "I love Lenusia!" plus a 32x32 heart bitmap.
2. Enables the GC1109 front-end amplifier (`FEM_EN` on GPIO 2).
3. Initializes the SX1262 through RadioLib using a custom ESP-IDF hardware abstraction layer (`Esp32S3Hal`).
4. Transmits a `Hello Heltec V4! #N` LoRa packet every 5 seconds (868 MHz, 125 kHz BW, SF9, CR 4/5, private sync word, 14 dBm, preamble 8).

Key facts:

- **Build system:** CMake via ESP-IDF's `idf.py` wrapper.
- **ESP-IDF version:** 5.3.0 (locked in `dependencies.lock`).
- **Language:** C/C++ (`main` is a C++ component because RadioLib is a C++ library).
- **Target:** ESP32-S3 (Xtensa), `CONFIG_IDF_TARGET="esp32s3"`.
- **Framework:** FreeRTOS (provided by ESP-IDF).

### ESP-IDF 5.3 compatibility note

`main/idf_component.yml` pins `nixy4/u8g2: '0.1.4'`. That component uses `i2c_master_bus_config_t.flags.allow_pd`, which was introduced in ESP-IDF 5.4. On the project's ESP-IDF 5.3 the member is missing, so `CMakeLists.txt` applies an automated patch that strips `.flags.allow_pd = 0,` from `managed_components/nixy4__u8g2/src/port/esp32_hw_i2c.c` during `idf.py reconfigure`.

## Repository Layout

```
.
├── CMakeLists.txt              # Top-level project CMake file (project(hello_world), also patches u8g2 for IDF 5.3)
├── main/                       # Main application component
│   ├── CMakeLists.txt          # Registers C++ sources and component dependencies
│   ├── display.h               # OLED display abstraction layer
│   ├── display.cpp             # u8g2-based SSD1306 driver for Heltec V4
│   ├── Esp32S3Hal.h            # RadioLib HAL header for ESP32-S3
│   ├── Esp32S3Hal.cpp          # RadioLib HAL implementation (ESP-IDF SPI/GPIO/timer)
│   ├── hello_world_main.cpp    # Application entry point (app_main)
│   └── idf_component.yml       # IDF Component Manager manifest
├── pytest_hello_world.py       # Automated pytest cases (stale: expects old "Hello world!" output)
├── sdkconfig                   # Generated project configuration for ESP32-S3
├── sdkconfig.old               # Previous sdkconfig backup
├── sdkconfig.ci                # CI-specific sdkconfig override (currently empty)
├── .clangd                     # clangd editor configuration (points at build/, Xtensa toolchain flags)
├── README.md                   # Original Espressif hello_world example README (largely outdated)
└── managed_components/         # Downloaded by the IDF Component Manager
    ├── jgromes__radiolib/      # RadioLib 7.7.1
    └── nixy4__u8g2/            # u8g2 0.1.4 (used by main)
```

`README.md` is the stock Espressif hello_world example README and describes the original C example; trust this file and the actual sources over it.

## Build and Development Commands

All build operations go through ESP-IDF's `idf.py`. Source the ESP-IDF environment first (`get_idf` or `. $IDF_PATH/export.sh`).

```bash
# Configure for ESP32-S3 (one-time unless sdkconfig changes)
idf.py set-target esp32s3

# Re-fetch component dependencies after editing main/idf_component.yml
idf.py reconfigure

# Build
idf.py build
```

Build output goes to `build/`, which contains a generated Ninja build and `compile_commands.json` (used by `.clangd`).

### Flash and Monitor

```bash
# Replace PORT with the actual serial port, e.g., /dev/ttyUSB0
idf.py -p PORT flash
idf.py -p PORT monitor

# Combined build, flash, and monitor
idf.py -p PORT build flash monitor
```

### Menuconfig

```bash
idf.py menuconfig
```

## Code Organization

All functionality lives in the `main` component; there are no custom components.

- **`main/hello_world_main.cpp`**: Application entry point (`extern "C" void app_main(void)`). Contains the Heltec V4 pin mapping as macros, initializes the OLED via `display_init()`, draws the greeting, enables the GC1109 FEM, brings up RadioLib, and runs the 5-second transmit loop. Log tag: `HELTEC_V4` (e.g. `HELTEC_V4: Radio init OK`, `HELTEC_V4: TX OK`).
- **`main/display.h` / `main/display.cpp`**: Heltec V4 OLED abstraction. Powers the SSD1306 through Vext (GPIO 36, fallback GPIO 40), resets it, initializes u8g2 over ESP-IDF I2C, and exposes `display_init()`, `display_clear()`, `display_print()`, and `display_draw_xbm()`.
- **`main/Esp32S3Hal.h` / `main/Esp32S3Hal.cpp`**: Custom RadioLib `RadioLibHal` subclass for ESP32-S3 built on the ESP-IDF driver API (GPIO, `spi_master`, `esp_timer`, FreeRTOS delays, GPIO ISR dispatch). The SPI device runs in mode 0 at 2 MHz with `spics_io_num = -1` because NSS is toggled manually by RadioLib.
- **`main/CMakeLists.txt`**: `idf_component_register(SRCS "Esp32S3Hal.cpp" "display.cpp" "hello_world_main.cpp" ... REQUIRES jgromes__radiolib nixy4__u8g2 driver esp_timer)`.

### Heltec V4 pin mapping (from `hello_world_main.cpp`)

- LoRa (SX1262 on SPI2): SCK GPIO 9, MISO GPIO 11, MOSI GPIO 10, NSS GPIO 8, DIO1 GPIO 14, RST GPIO 12, BUSY GPIO 13.
- GC1109 front-end enable (`FEM_EN`): GPIO 2.
- OLED SSD1306 (I2C): SDA GPIO 17, SCL GPIO 18, RST GPIO 21.
- OLED power switch (Vext): GPIO 36 (GPIO 40 on V4-R8).

## Dependencies

Declared in `main/idf_component.yml` (resolved versions in `dependencies.lock`):

- `jgromes/radiolib: '*'` → 7.7.1 (Espressif component registry). Used by `main`.
- `nixy4/u8g2: '0.1.4'` → 0.1.4 (registry). Used by `main` through `display.cpp`/`display.h`.

The display driver is implemented locally in `main/display.cpp`/`main/display.h` on top of u8g2.

## Code Style Guidelines

- Write source comments and documentation in **English**.
- Source files use the standard Espressif SPDX header:

  ```c
  /*
   * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
   *
   * SPDX-License-Identifier: CC0-1.0
   */
  ```

- Follow the existing C/C++ formatting (K&R-style braces, 4-space indentation).
- Include `sdkconfig.h` when using Kconfig-generated macros such as `CONFIG_IDF_TARGET`.
- Use ESP-IDF-provided types and APIs (`gpio_set_level`, `spi_device_transmit`, `vTaskDelay`, `esp_log.h`, etc.).

## Testing Instructions

Tests live in `pytest_hello_world.py` and run with `pytest` plus the `pytest-embedded` plugins (`pytest-embedded`, `pytest-embedded-idf`, `pytest-embedded-qemu`):

```bash
pytest
```

**The tests are stale**: they still assert the original `Hello world!` serial output from the unmodified example. The firmware now logs `HELTEC_V4:` lines (`Radio init OK`, `TX OK`, OLED/FEM status) instead, so the assertions must be updated before the tests can pass. The QEMU test is additionally marked `@pytest.mark.esp32`, which does not match the esp32s3 target.

Test markers used: `supported_targets`, `preview_targets`, `generic`, `linux`, `host_test`, `esp32`, `qemu`.

## Configuration Notes

- **`sdkconfig`**: auto-generated for ESP32-S3. Notable values:
  - `CONFIG_IDF_TARGET="esp32s3"`
  - Flash: DIO mode, 80 MHz, 2 MB
  - Partition table: single app (`CONFIG_PARTITION_TABLE_SINGLE_APP=y`)
  - Compiler optimization: Debug (`CONFIG_OPTIMIZATION_LEVEL_DEBUG=y`)
  - FreeRTOS tick: 100 Hz; main task stack: 3584 bytes
- **`sdkconfig.ci`**: empty; add CI-specific Kconfig overrides here if needed.
- **`.clangd`**: points clangd at the `build/` compilation database and adjusts Xtensa toolchain flags (sysroot under `~/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20240530`); the sysroot path is machine-specific.

## Security Considerations

- Secure boot and flash encryption are **disabled** in the current `sdkconfig`.
- The firmware transmits periodic LoRa packets; do not put sensitive data into `transmit()` payloads before production use.
- To enable security features, re-run `idf.py menuconfig` under *Security features* and follow Espressif's secure-boot/flash-encryption guides.

## Useful References

- [ESP-IDF Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
- [ESP-IDF Get Started](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html)
- [IDF Component Manager](https://docs.espressif.com/projects/idf-component-manager/en/latest/)
- [RadioLib GitHub](https://github.com/jgromes/RadioLib)
- [RadioLib Non-Arduino ESP-IDF Example](https://github.com/jgromes/RadioLib/tree/master/examples/NonArduino/ESP-IDF)
