/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "icm20948.h"

#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// IMPORTANT: use only the new I2C driver (driver/i2c_master.h). The legacy
// driver (driver/i2c.h) cannot be linked together with it in ESP-IDF 5.x —
// the firmware aborts at startup with "CONFLICT! driver_ng ...".

// I2C bus configuration.
// Heltec V4 sensor header: SDA = GPIO16, SCL = GPIO15. If the wires got
// swapped on the breadboard, set IMU_I2C_PINS_SWAPPED to 1 to probe the
// other pin order.
#define IMU_I2C_PINS_SWAPPED    0
#define IMU_I2C_PORT            I2C_NUM_1
#if IMU_I2C_PINS_SWAPPED
#define IMU_I2C_SDA             GPIO_NUM_15
#define IMU_I2C_SCL             GPIO_NUM_16
#else
#define IMU_I2C_SDA             GPIO_NUM_16
#define IMU_I2C_SCL             GPIO_NUM_15
#endif
#define IMU_I2C_CLK_HZ          100000
#define IMU_I2C_TIMEOUT_MS      100

// ICM20948 (7-bit address depends on AD0 pin)
#define ICM20948_ADDR_AD0_LOW   0x68
#define ICM20948_ADDR_AD0_HIGH  0x69
#define ICM20948_REG_WHO_AM_I   0x00    // returns 0xEA
#define ICM20948_REG_PWR_MGMT_1 0x06
#define ICM20948_REG_INT_PIN_CFG 0x0F
#define ICM20948_REG_ACCEL_XOUT_H 0x2D   // bank 0: XH, XL, YH, YL, ZH, ZL

// AK09916 magnetometer (reachable on the main bus via ICM20948 bypass mode)
#define AK09916_ADDR            0x0C
#define AK09916_REG_WIA1        0x00    // company ID, returns 0x48
#define AK09916_REG_WIA2        0x01    // device ID, returns 0x09
#define AK09916_REG_ST1         0x10
#define AK09916_REG_HXL         0x11
#define AK09916_REG_ST2         0x18
#define AK09916_REG_CNTL2       0x31    // mode control (CNTL1 is 0x30!)
#define AK09916_REG_CNTL3       0x32

static const char *TAG = "icm20948";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev_icm = NULL;
static i2c_master_dev_handle_t s_dev_ak = NULL;

// --- Magnetometer field calibration (min/max collector) ---
#define MAG_CAL_DURATION_US     (25 * 1000000ULL)   // collection window
#define MAG_CAL_MIN_SPAN        100                 // LSB; rejects stale runs

static icm20948_mag_cal_t s_mag_cal = {{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};
static bool s_cal_active = false;
static bool s_cal_valid = false;
static int16_t s_cal_min[3] = {0, 0, 0};
static int16_t s_cal_max[3] = {0, 0, 0};
static uint64_t s_cal_start_us = 0;

static esp_err_t imu_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev, buf, sizeof(buf), IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(dev, &reg, 1, value, 1, IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, IMU_I2C_TIMEOUT_MS);
}

// Probe a 7-bit address by reading one register from it. Returns true if the
// device ACKed, and reports the register value.
static bool imu_probe_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    i2c_device_config_t probe_cfg = {};
    probe_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    probe_cfg.device_address = addr;
    probe_cfg.scl_speed_hz = IMU_I2C_CLK_HZ;
    probe_cfg.scl_wait_us = 0;
    probe_cfg.flags.disable_ack_check = 0;

    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(s_bus, &probe_cfg, &dev) != ESP_OK) {
        return false;
    }

    uint8_t value = 0;
    esp_err_t err = imu_read_reg(dev, reg, &value);
    i2c_master_bus_rm_device(dev);

    *val = value;
    return err == ESP_OK;
}

// Log every device that ACKs on the bus — handy to find wiring/address issues.
static void imu_scan_bus(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=GPIO%d, SCL=GPIO%d) ...", IMU_I2C_SDA, IMU_I2C_SCL);

    // A scan probes ~100 addresses and most will NACK — silence the i2c
    // driver error spam so the result stays readable.
    esp_log_level_t saved_level = esp_log_level_get("i2c.master");
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    int found = 0;
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        uint8_t val = 0;
        if (imu_probe_reg(addr, 0x00, &val)) {
            ESP_LOGI(TAG, "  ACK at 0x%02X (reg[0x00] = 0x%02X)", addr, val);
            found++;
        }
    }

    esp_log_level_set("i2c.master", saved_level);

    if (found == 0) {
        ESP_LOGW(TAG, "  no devices found — check SDA/SCL wiring, module power, pull-ups");
    }
}

bool icm20948_init(void)
{
    // Fields are assigned individually: C++ does not support nested
    // designated initializers (.flags.*).
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = IMU_I2C_PORT;
    bus_cfg.sda_io_num = IMU_I2C_SDA;
    bus_cfg.scl_io_num = IMU_I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 0;
    bus_cfg.trans_queue_depth = 0;
    bus_cfg.flags.enable_internal_pullup = 1;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return false;
    }

    imu_scan_bus();

    // --- Locate the ICM20948 (try both possible addresses) ---
    static const uint8_t icm_candidates[] = {ICM20948_ADDR_AD0_LOW, ICM20948_ADDR_AD0_HIGH};
    uint8_t icm_addr = 0;
    for (size_t i = 0; i < sizeof(icm_candidates); i++) {
        uint8_t id = 0;
        if (!imu_probe_reg(icm_candidates[i], ICM20948_REG_WHO_AM_I, &id)) {
            ESP_LOGW(TAG, "no ACK from 0x%02X", icm_candidates[i]);
            continue;
        }
        if (id != 0xEA) {
            ESP_LOGW(TAG, "device at 0x%02X answered, WHO_AM_I=0x%02X (expected 0xEA) — not an ICM20948?",
                     icm_candidates[i], id);
            continue;
        }
        icm_addr = icm_candidates[i];
        ESP_LOGI(TAG, "ICM20948 found at 0x%02X", icm_addr);
        break;
    }
    if (icm_addr == 0) {
        ESP_LOGE(TAG, "ICM20948 not found at 0x68/0x69 — check wiring, power, pull-ups");
        return false;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.scl_speed_hz = IMU_I2C_CLK_HZ;
    dev_cfg.scl_wait_us = 0;
    dev_cfg.flags.disable_ack_check = 0;

    dev_cfg.device_address = icm_addr;
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev_icm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device(ICM20948) failed: %s", esp_err_to_name(err));
        return false;
    }

    // Wake up and select best available clock source.
    imu_write_reg(s_dev_icm, ICM20948_REG_PWR_MGMT_1, 0x01);
    // Enable I2C bypass so the AK09916 appears directly on the main bus.
    imu_write_reg(s_dev_icm, ICM20948_REG_INT_PIN_CFG, 0x02);

    vTaskDelay(pdMS_TO_TICKS(10));

    // --- Locate the AK09916 through the bypass ---
    uint8_t wia2 = 0;
    if (!imu_probe_reg(AK09916_ADDR, AK09916_REG_WIA2, &wia2) || wia2 != 0x09) {
        ESP_LOGE(TAG, "AK09916 not reachable at 0x0C (WIA2=0x%02X, expected 0x09) — bypass failed?",
                 wia2);
        return false;
    }
    ESP_LOGI(TAG, "AK09916 found at 0x0C");

    dev_cfg.device_address = AK09916_ADDR;
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev_ak);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device(AK09916) failed: %s", esp_err_to_name(err));
        return false;
    }

    // Soft reset AK09916.
    imu_write_reg(s_dev_ak, AK09916_REG_CNTL3, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
    // Continuous measurement mode 2 (100 Hz).
    imu_write_reg(s_dev_ak, AK09916_REG_CNTL2, 0x08);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify the mode actually latched — catches register-map mistakes.
    uint8_t cntl2 = 0;
    imu_read_reg(s_dev_ak, AK09916_REG_CNTL2, &cntl2);
    if ((cntl2 & 0x1F) != 0x08) {
        ESP_LOGW(TAG, "AK09916 CNTL2 readback = 0x%02X (expected 0x08)", cntl2);
    }

    ESP_LOGI(TAG, "ICM20948 + AK09916 initialized (IMU at 0x%02X)", icm_addr);

    // Begin the first field calibration run right away — the user should
    // rotate the board through all orientations for ~25 s after boot.
    icm20948_cal_start();
    return true;
}

bool icm20948_read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    if (s_dev_ak == NULL) {
        return false;
    }

    static uint32_t fail_count = 0;
    uint8_t st1 = 0;
    if (imu_read_reg(s_dev_ak, AK09916_REG_ST1, &st1) != ESP_OK) {
        if (++fail_count % 50 == 1) {
            ESP_LOGW(TAG, "mag ST1 read failed (%lu times)", (unsigned long)fail_count);
        }
        return false;
    }
    if ((st1 & 0x01) == 0) {
        if (++fail_count % 50 == 1) {
            ESP_LOGW(TAG, "mag DRDY not set, ST1=0x%02X (%lu times)", st1, (unsigned long)fail_count);
        }
        return false;
    }

    uint8_t buf[6];
    if (imu_read_regs(s_dev_ak, AK09916_REG_HXL, buf, sizeof(buf)) != ESP_OK) {
        if (++fail_count % 50 == 1) {
            ESP_LOGW(TAG, "mag data read failed (%lu times)", (unsigned long)fail_count);
        }
        return false;
    }
    fail_count = 0;

    static bool s_first_mag = true;
    if (s_first_mag) {
        s_first_mag = false;
        ESP_LOGI(TAG, "first mag read: X=%d Y=%d Z=%d", (int)*x, (int)*y, (int)*z);
    }

    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);

    // ST2 must be read to unlock the data registers.
    uint8_t st2 = 0;
    imu_read_reg(s_dev_ak, AK09916_REG_ST2, &st2);

    return true;
}

bool icm20948_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    if (s_dev_icm == NULL) {
        return false;
    }

    uint8_t buf[6];
    if (imu_read_regs(s_dev_icm, ICM20948_REG_ACCEL_XOUT_H, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    *x = (int16_t)((buf[0] << 8) | buf[1]);
    *y = (int16_t)((buf[2] << 8) | buf[3]);
    *z = (int16_t)((buf[4] << 8) | buf[5]);
    return true;
}

// ---------------------------------------------------------------------------
// Magnetometer field calibration.
//
// Collector: while a run is active, every raw sample expands per-axis
// min/max. Finish computes hard-iron (offset = midpoint) and soft-iron
// (scale = R_avg / r_i) coefficients, exactly the classic min/max method.
// ---------------------------------------------------------------------------

void icm20948_cal_start(void)
{
    s_cal_min[0] = s_cal_min[1] = s_cal_min[2] = INT16_MAX;
    s_cal_max[0] = s_cal_max[1] = s_cal_max[2] = INT16_MIN;
    s_cal_start_us = esp_timer_get_time();
    s_cal_valid = false;
    s_cal_active = true;
    ESP_LOGI(TAG, "mag calibration started — rotate the board in all orientations for 25 s");
}

bool icm20948_cal_active(void)
{
    return s_cal_active;
}

bool icm20948_cal_finish(void)
{
    if (!s_cal_active) {
        return s_cal_valid;
    }
    s_cal_active = false;

    float radius[3];
    for (int i = 0; i < 3; i++) {
        if ((int32_t)s_cal_max[i] - (int32_t)s_cal_min[i] < MAG_CAL_MIN_SPAN) {
            ESP_LOGW(TAG, "mag calibration: axis %d span too small (%d) — keep old cal",
                     i, (int)(s_cal_max[i] - s_cal_min[i]));
            return s_cal_valid;
        }
        s_mag_cal.offset[i] = ((float)s_cal_max[i] + (float)s_cal_min[i]) * 0.5f;
        radius[i] = ((float)s_cal_max[i] - (float)s_cal_min[i]) * 0.5f;
    }

    float r_avg = (radius[0] + radius[1] + radius[2]) / 3.0f;
    for (int i = 0; i < 3; i++) {
        s_mag_cal.scale[i] = (radius[i] > 1.0f) ? (r_avg / radius[i]) : 1.0f;
    }
    s_cal_valid = true;

    ESP_LOGI(TAG, "mag calibration done: offset %.1f %.1f %.1f, scale %.3f %.3f %.3f",
             s_mag_cal.offset[0], s_mag_cal.offset[1], s_mag_cal.offset[2],
             s_mag_cal.scale[0], s_mag_cal.scale[1], s_mag_cal.scale[2]);
    return true;
}

// Fold one raw sample into the running min/max; auto-finish after the window.
static void mag_cal_collect(int16_t x, int16_t y, int16_t z)
{
    const int16_t v[3] = {x, y, z};
    for (int i = 0; i < 3; i++) {
        if (v[i] < s_cal_min[i]) {
            s_cal_min[i] = v[i];
        }
        if (v[i] > s_cal_max[i]) {
            s_cal_max[i] = v[i];
        }
    }
    if (esp_timer_get_time() - s_cal_start_us >= MAG_CAL_DURATION_US) {
        icm20948_cal_finish();
    }
}

// Correct raw LSB values with the active calibration.
static void mag_cal_apply(int16_t rx, int16_t ry, int16_t rz,
                          float *mx, float *my, float *mz)
{
    *mx = ((float)rx - s_mag_cal.offset[0]) * s_mag_cal.scale[0];
    *my = ((float)ry - s_mag_cal.offset[1]) * s_mag_cal.scale[1];
    *mz = ((float)rz - s_mag_cal.offset[2]) * s_mag_cal.scale[2];
}
float icm20948_get_heading(void)
{
    int16_t mx_raw = 0, my_raw = 0, mz_raw = 0;
    if (!icm20948_read_mag(&mx_raw, &my_raw, &mz_raw)) {
        return -1.0f;
    }

    if (s_cal_active) {
        mag_cal_collect(mx_raw, my_raw, mz_raw);
    }

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    mag_cal_apply(mx_raw, my_raw, mz_raw, &mx, &my, &mz);

    int16_t ax = 0, ay = 0, az = 0;
    if (!icm20948_read_accel(&ax, &ay, &az)) {
        return -1.0f;
    }
    float axf = (float)ax, ayf = (float)ay, azf = (float)az;

    // Tilt angles from the gravity vector.
    float roll = atan2f(ayf, azf);
    float pitch = atanf(-axf / sqrtf(ayf * ayf + azf * azf));

    // Tilt-compensated horizontal field components.
    float xh = mx * cosf(pitch) + mz * sinf(pitch);
    float yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    float heading = atan2f(-yh, xh) * 180.0f / (float)M_PI;
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}
