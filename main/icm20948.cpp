/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * ICM20948 driver built on the SparkFun ICM-20948 C core, vendored from
 * SparkFun_ICM-20948_ArduinoLibrary v1.3.2 (src/util, MIT-licensed). The
 * Arduino wrapper is not used: the portable C core is driven through the
 * ICM_20948_Serif_t transport implemented on the ESP-IDF i2c_master driver.
 *
 * Heading comes from the DMP 9-axis fusion (Quat9, DMP sensor
 * INV_ICM20948_SENSOR_ORIENTATION). While the DMP warms up the fallback is
 * tilt-compensated accel + uncalibrated compass packets from the same FIFO,
 * with the min/max hard-iron/soft-iron calibration collected below.
 *
 * The init sequence mirrors SparkFun's startupDefault(minimal) +
 * initializeDMP(): see the comments in the original library for the
 * InvenSense programming sequence details.
 */

#include "icm20948.h"

#include <math.h>
#include <string.h>

#include "ICM_20948_C.h"
#include "AK09916_REGISTERS.h"
#include "driver/i2c_master.h"
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
#define IMU_I2C_CLK_HZ          400000
#define IMU_I2C_TIMEOUT_MS      100

// The C core never transfers more than INV_MAX_SERIAL_WRITE / _READ (16)
// bytes in one serif call; one extra byte for the register address.
#define SERIF_BUF_MAX           (16 + 1)

// DMP sensor rate: interval = (DMP running rate 225 Hz / ODR) - 1.
// 8 -> ~25 Hz output, well above the 5 Hz display refresh.
#define QUAT9_ODR_INTERVAL      8

// Heading offset of the DMP yaw relative to the board mounting, in degrees.
// Adjust after comparing against a reference compass.
#define DMP_HEADING_OFFSET_DEG  0.0f

static const char *TAG = "icm20948";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static ICM_20948_Device_t s_imu;
static bool s_dmp_ready = false;

// ---------------------------------------------------------------------------
// Magnetometer field calibration (min/max collector, applied to the
// tilt-compensated fallback path).
// ---------------------------------------------------------------------------
#define MAG_CAL_DURATION_US     (25 * 1000000ULL)   // collection window
#define MAG_CAL_MIN_SPAN        100                 // LSB; rejects stale runs

static icm20948_mag_cal_t s_mag_cal = {{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};
static bool s_cal_active = false;
static bool s_cal_valid = false;
static int16_t s_cal_min[3] = {0, 0, 0};
static int16_t s_cal_max[3] = {0, 0, 0};
static uint64_t s_cal_start_us = 0;

// ---------------------------------------------------------------------------
// Serif: I2C transport for the SparkFun C core
// ---------------------------------------------------------------------------

static ICM_20948_Status_e serif_write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user)
{
    (void)user;
    if (len > 16) {
        return ICM_20948_Stat_ParamErr;
    }
    uint8_t buf[SERIF_BUF_MAX];
    buf[0] = regaddr;
    memcpy(&buf[1], pdata, len);
    return i2c_master_transmit(s_dev, buf, len + 1, IMU_I2C_TIMEOUT_MS) == ESP_OK
               ? ICM_20948_Stat_Ok
               : ICM_20948_Stat_Err;
}

static ICM_20948_Status_e serif_read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user)
{
    (void)user;
    if (len > 16) {
        return ICM_20948_Stat_ParamErr;
    }
    return i2c_master_transmit_receive(s_dev, &regaddr, 1, pdata, len, IMU_I2C_TIMEOUT_MS) == ESP_OK
               ? ICM_20948_Stat_Ok
               : ICM_20948_Stat_Err;
}

static const ICM_20948_Serif_t s_serif = {serif_write, serif_read, NULL};

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static bool check(ICM_20948_Status_e st, const char *what)
{
    if (st > ICM_20948_Stat_Ok) {
        ESP_LOGE(TAG, "%s failed: status %d", what, (int)st);
        return false;
    }
    return true;
}

// Minimal startupMagnetometer(): bring up the I2C master interface and make
// sure the AK09916 answers, but leave the 100 Hz continuous configuration to
// the DMP path (which uses Single Measurement mode instead).
static bool startup_magnetometer_minimal(void)
{
    ICM_20948_i2c_master_passthrough(&s_imu, false); // AUX pins not connected to SDA/SCL
    if (!check(ICM_20948_i2c_master_enable(&s_imu, true), "i2c_master_enable")) {
        return false;
    }

    // Soft-reset the AK09916 after the ICM reset (it may stop answering).
    uint8_t reset = 0x01; // AK09916 CNTL3
    ICM_20948_i2c_master_single_w(&s_imu, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, &reset);

    const int max_tries = 10;
    for (int tries = 1; tries <= max_tries; tries++) {
        uint8_t wia1 = 0, wia2 = 0;
        ICM_20948_Status_e s1 = ICM_20948_i2c_master_single_r(&s_imu, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA1, &wia1);
        ICM_20948_Status_e s2 = ICM_20948_i2c_master_single_r(&s_imu, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &wia2);
        if (s1 == ICM_20948_Stat_Ok && s2 == ICM_20948_Stat_Ok &&
            wia1 == (MAG_AK09916_WHO_AM_I >> 8) && wia2 == (MAG_AK09916_WHO_AM_I & 0xFF)) {
            return true;
        }
        ICM_20948_i2c_master_reset(&s_imu);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "AK09916 not answering on the aux bus (WIA != 0x4809)");
    return false;
}

// DMP configuration, following SparkFun's initializeDMP() step by step.
static bool configure_dmp(void)
{
    ICM_20948_Status_e st;

    // I2C_SLV0: read 10 bytes from the AK09916 starting at RSV2 (0x03) —
    // the InvenSense eMD "secret sauce" for DMP compass data (big endian).
    if (!check(ICM_20948_i2c_controller_configure_peripheral(&s_imu, 0, MAG_AK09916_I2C_ADDR,
                                                             AK09916_REG_RSV2, 10, true, true,
                                                             false, true, true, 0),
               "slv0 config")) {
        return false;
    }
    // I2C_SLV1: trigger a single magnetometer measurement every cycle.
    if (!check(ICM_20948_i2c_controller_configure_peripheral(&s_imu, 1, MAG_AK09916_I2C_ADDR,
                                                             AK09916_REG_CNTL2, 1, false, true,
                                                             false, false, false, AK09916_mode_single),
               "slv1 config")) {
        return false;
    }

    // I2C master ODR 1100/2^4 = 68.75 Hz (essential despite the datasheet).
    if (!check(ICM_20948_set_bank(&s_imu, 3), "set_bank 3")) {
        return false;
    }
    uint8_t mst_odr = 0x04;
    if (!check(ICM_20948_execute_w(&s_imu, AGB3_REG_I2C_MST_ODR_CONFIG, &mst_odr, 1), "mst odr")) {
        return false;
    }

    if (!check(ICM_20948_set_clock_source(&s_imu, ICM_20948_Clock_Auto), "clock source")) {
        return false;
    }
    if (!check(ICM_20948_set_bank(&s_imu, 0), "set_bank 0")) {
        return false;
    }
    uint8_t pwr_mgmt_2 = 0x40; // reserved bit 6, pressure sensor disabled
    if (!check(ICM_20948_execute_w(&s_imu, AGB0_REG_PWR_MGMT_2, &pwr_mgmt_2, 1), "pwr mgmt 2")) {
        return false;
    }

    // Only the I2C master in duty-cycled mode; accel/gyro run continuously.
    if (!check(ICM_20948_set_sample_mode(&s_imu, ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled),
               "sample mode")) {
        return false;
    }
    if (!check(ICM_20948_enable_FIFO(&s_imu, false), "fifo off")) {
        return false;
    }
    if (!check(ICM_20948_enable_DMP(&s_imu, false), "dmp off")) {
        return false;
    }

    ICM_20948_fss_t fss = {gpm4, dps2000, 0};
    if (!check(ICM_20948_set_full_scale(&s_imu,
                                        (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc |
                                                                        ICM_20948_Internal_Gyr),
                                        fss),
               "full scale")) {
        return false;
    }
    if (!check(ICM_20948_enable_dlpf(&s_imu, ICM_20948_Internal_Gyr, true), "gyro dlpf")) {
        return false;
    }

    // Nothing into the FIFO except DMP data.
    uint8_t zero = 0;
    ICM_20948_execute_w(&s_imu, AGB0_REG_FIFO_EN_1, &zero, 1);
    ICM_20948_execute_w(&s_imu, AGB0_REG_FIFO_EN_2, &zero, 1);

    // Data-ready interrupt off (we poll the FIFO).
    ICM_20948_INT_enable_t en;
    if (!check(ICM_20948_int_enable(&s_imu, NULL, &en), "int_enable read")) {
        return false;
    }
    en.RAW_DATA_0_RDY_EN = 0;
    if (!check(ICM_20948_int_enable(&s_imu, &en, &en), "int_enable write")) {
        return false;
    }

    if (!check(ICM_20948_reset_FIFO(&s_imu), "fifo reset")) {
        return false;
    }

    // 1.125 kHz/(1+19) = ~56 Hz accel/gyro ODR (InvenSense reference value).
    ICM_20948_smplrt_t smplrt = {19, 19};
    if (!check(ICM_20948_set_sample_rate(&s_imu,
                                         (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc |
                                                                         ICM_20948_Internal_Gyr),
                                         smplrt),
               "sample rate")) {
        return false;
    }

    // Load the DMP firmware and point the program counter at it.
    if (!check(ICM_20948_set_dmp_start_address(&s_imu, DMP_START_ADDRESS), "dmp addr")) {
        return false;
    }
    if (!check(ICM_20948_firmware_load(&s_imu), "firmware load")) {
        return false;
    }
    if (!check(ICM_20948_set_dmp_start_address(&s_imu, DMP_START_ADDRESS), "dmp addr")) {
        return false;
    }

    if (!check(ICM_20948_set_bank(&s_imu, 0), "set_bank 0")) {
        return false;
    }
    uint8_t hw_fix = 0x48;
    if (!check(ICM_20948_execute_w(&s_imu, AGB0_REG_HW_FIX_DISABLE, &hw_fix, 1), "hw fix")) {
        return false;
    }
    uint8_t fifo_prio = 0xE4;
    if (!check(ICM_20948_execute_w(&s_imu, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifo_prio, 1),
               "fifo prio")) {
        return false;
    }

    // Accel scaling for the DMP (internal 2^25 = 1g at 4g FSR).
    const unsigned char acc_scale[4] = {0x04, 0x00, 0x00, 0x00};
    const unsigned char acc_scale2[4] = {0x00, 0x04, 0x00, 0x00};
    if (!check(inv_icm20948_write_mems(&s_imu, ACC_SCALE, 4, acc_scale), "ACC_SCALE")) {
        return false;
    }
    if (!check(inv_icm20948_write_mems(&s_imu, ACC_SCALE2, 4, acc_scale2), "ACC_SCALE2")) {
        return false;
    }

    // Compass mount matrix and scale (1 uT = 2^30, from the eMD example).
    const unsigned char m_zero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char m_plus[4] = {0x09, 0x99, 0x99, 0x99};
    const unsigned char m_minus[4] = {0xF6, 0x66, 0x66, 0x67};
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_00, 4, m_plus);
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_01, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_02, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_10, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_11, 4, m_minus);
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_12, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_20, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, CPASS_MTX_21, 4, m_zero);
    st = inv_icm20948_write_mems(&s_imu, CPASS_MTX_22, 4, m_minus);
    if (!check(st, "CPASS_MTX")) {
        return false;
    }

    // B2S mount matrix.
    const unsigned char b_plus[4] = {0x40, 0x00, 0x00, 0x00};
    inv_icm20948_write_mems(&s_imu, B2S_MTX_00, 4, b_plus);
    inv_icm20948_write_mems(&s_imu, B2S_MTX_01, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, B2S_MTX_02, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, B2S_MTX_10, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, B2S_MTX_11, 4, b_plus);
    inv_icm20948_write_mems(&s_imu, B2S_MTX_12, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, B2S_MTX_20, 4, m_zero);
    inv_icm20948_write_mems(&s_imu, B2S_MTX_21, 4, m_zero);
    st = inv_icm20948_write_mems(&s_imu, B2S_MTX_22, 4, b_plus);
    if (!check(st, "B2S_MTX")) {
        return false;
    }

    // Gyro scaling factor (55 Hz ODR, 2000 dps).
    if (!check(inv_icm20948_set_gyro_sf(&s_imu, 19, 3), "gyro sf")) {
        return false;
    }

    const unsigned char gyro_fullscale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000 dps : 2^28
    if (!check(inv_icm20948_write_mems(&s_imu, GYRO_FULLSCALE, 4, gyro_fullscale), "GYRO_FULLSCALE")) {
        return false;
    }

    // Accel-only filter gains for the 56 Hz ODR.
    const unsigned char accel_only_gain[4] = {0x03, 0xA4, 0x92, 0x49};
    const unsigned char accel_alpha_var[4] = {0x34, 0x92, 0x49, 0x25};
    const unsigned char accel_a_var[4] = {0x0B, 0x6D, 0xB6, 0xDB};
    const unsigned char accel_cal_rate[2] = {0x00, 0x00};
    if (!check(inv_icm20948_write_mems(&s_imu, ACCEL_ONLY_GAIN, 4, accel_only_gain), "ACCEL_ONLY_GAIN")) {
        return false;
    }
    if (!check(inv_icm20948_write_mems(&s_imu, ACCEL_ALPHA_VAR, 4, accel_alpha_var), "ACCEL_ALPHA_VAR")) {
        return false;
    }
    if (!check(inv_icm20948_write_mems(&s_imu, ACCEL_A_VAR, 4, accel_a_var), "ACCEL_A_VAR")) {
        return false;
    }
    if (!check(inv_icm20948_write_mems(&s_imu, ACCEL_CAL_RATE, 2, accel_cal_rate), "ACCEL_CAL_RATE")) {
        return false;
    }

    // Compass time buffer: mag read at 68.75 Hz -> 69.
    const unsigned char compass_rate[2] = {0x00, 0x45};
    if (!check(inv_icm20948_write_mems(&s_imu, CPASS_TIME_BUFFER, 2, compass_rate), "CPASS_TIME_BUFFER")) {
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Magnetometer field calibration.
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

static void mag_cal_apply(int16_t rx, int16_t ry, int16_t rz,
                          float *mx, float *my, float *mz)
{
    *mx = ((float)rx - s_mag_cal.offset[0]) * s_mag_cal.scale[0];
    *my = ((float)ry - s_mag_cal.offset[1]) * s_mag_cal.scale[1];
    *mz = ((float)rz - s_mag_cal.offset[2]) * s_mag_cal.scale[2];
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool icm20948_init(void)
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = IMU_I2C_PORT;
    bus_cfg.sda_io_num = IMU_I2C_SDA;
    bus_cfg.scl_io_num = IMU_I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 0;
    bus_cfg.trans_queue_depth = 0;
    bus_cfg.flags.enable_internal_pullup = 1;

    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed");
        return false;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = ICM_20948_I2C_ADDR_AD0; // AD0 to GND -> 0x68
    dev_cfg.scl_speed_hz = IMU_I2C_CLK_HZ;
    dev_cfg.scl_wait_us = 0;
    dev_cfg.flags.disable_ack_check = 0;
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed");
        return false;
    }

    // Attach the SparkFun C core to our I2C transport.
    ICM_20948_init_struct(&s_imu);
    ICM_20948_link_serif(&s_imu, &s_serif);
#ifdef ICM_20948_USE_DMP
    // The wrapper does this in its constructor: tell the C core the DMP
    // firmware image is compiled in (icm20948_img.dmp3a.h).
    s_imu._dmp_firmware_available = true;
#endif

    // Minimal startup: reset, wake up, verify both chips.
    if (!check(ICM_20948_check_id(&s_imu), "check_id")) {
        return false;
    }
    if (!check(ICM_20948_sw_reset(&s_imu), "sw_reset")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!check(ICM_20948_sleep(&s_imu, false), "sleep off")) {
        return false;
    }
    if (!check(ICM_20948_low_power(&s_imu, false), "low power off")) {
        return false;
    }
    if (!startup_magnetometer_minimal()) {
        return false;
    }

    // Full DMP configuration (InvenSense programming sequence).
    if (!configure_dmp()) {
        return false;
    }

    // 9-axis orientation (Quat9 quaternion + heading accuracy), ~25 Hz.
    if (!check(inv_icm20948_enable_dmp_sensor(&s_imu, INV_ICM20948_SENSOR_ORIENTATION, 1),
               "enable orientation")) {
        return false;
    }
    if (!check(inv_icm20948_set_dmp_sensor_period(&s_imu, DMP_ODR_Reg_Quat9, QUAT9_ODR_INTERVAL),
               "quat9 period")) {
        return false;
    }
    // Raw accel + uncalibrated compass packets feed the tilt-compensated
    // fallback while the Quat9 fusion is still warming up. Same ODR, so the
    // FIFO does not overflow between 5 Hz display polls.
    if (!check(inv_icm20948_enable_dmp_sensor(&s_imu, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 1),
               "enable raw accel")) {
        return false;
    }
    if (!check(inv_icm20948_enable_dmp_sensor(&s_imu, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 1),
               "enable raw compass")) {
        return false;
    }
    if (!check(inv_icm20948_set_dmp_sensor_period(&s_imu, DMP_ODR_Reg_Accel, QUAT9_ODR_INTERVAL),
               "accel period")) {
        return false;
    }
    if (!check(inv_icm20948_set_dmp_sensor_period(&s_imu, DMP_ODR_Reg_Cpass, QUAT9_ODR_INTERVAL),
               "cpass period")) {
        return false;
    }
    if (!check(ICM_20948_enable_FIFO(&s_imu, true), "fifo on")) {
        return false;
    }
    if (!check(ICM_20948_enable_DMP(&s_imu, true), "dmp on")) {
        return false;
    }

    s_dmp_ready = true;
    ESP_LOGI(TAG, "ICM20948 + AK09916 + DMP initialized");

    // Begin the first field calibration run right away — the user should
    // rotate the board through all orientations for ~25 s after boot.
    icm20948_cal_start();
    return true;
}

// Yaw (heading around Z) from a Quat9 DMP packet. Quaternion elements are
// {q1=x, q2=y, q3=z} in q30 fixed point; q0 is reconstructed.
static bool heading_from_quat9_data(const icm_20948_DMP_data_t *dmp, float *heading_deg)
{
    const float q30 = 1073741824.0f; // 2^30
    float qx = (float)dmp->Quat9.Data.Q1 / q30;
    float qy = (float)dmp->Quat9.Data.Q2 / q30;
    float qz = (float)dmp->Quat9.Data.Q3 / q30;
    float sum = qx * qx + qy * qy + qz * qz;
    if (sum >= 1.0f) {
        return false;
    }
    float qw = sqrtf(1.0f - sum);

    // ZYX Euler yaw: heading = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2)).
    float heading = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    heading = heading * 180.0f / (float)M_PI + DMP_HEADING_OFFSET_DEG;
    while (heading < 0.0f) {
        heading += 360.0f;
    }
    while (heading >= 360.0f) {
        heading -= 360.0f;
    }
    *heading_deg = heading;
    return true;
}

// Fallback: tilt-compensated heading from the DMP raw accel + uncalibrated
// compass packets. Used while the Quat9 fusion is still warming up.
static bool heading_from_accel_compass(int16_t ax, int16_t ay, int16_t az,
                                       int16_t cx, int16_t cy, int16_t cz,
                                       float *heading_deg)
{
    if (s_cal_active) {
        mag_cal_collect(cx, cy, cz);
    }

    float mx, my, mz;
    mag_cal_apply(cx, cy, cz, &mx, &my, &mz);

    float axf = (float)ax, ayf = (float)ay, azf = (float)az;

    float roll = atan2f(ayf, azf);
    float pitch = atanf(-axf / sqrtf(ayf * ayf + azf * azf));

    float xh = mx * cosf(pitch) + mz * sinf(pitch);
    float yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    float heading = atan2f(-yh, xh) * 180.0f / (float)M_PI;
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    *heading_deg = heading;
    return true;
}

float icm20948_get_heading(void)
{
    if (!s_dmp_ready) {
        return -1.0f;
    }

    float quat_heading = 0.0f;
    float fallback_heading = 0.0f;
    bool got_quat = false;
    bool got_fallback = false;

    // Drain up to a few FIFO packets: the newest Quat9 wins, and the newest
    // accel+compass pair is kept as the fallback candidate.
    for (int i = 0; i < 20; i++) {
        icm_20948_DMP_data_t dmp;
        ICM_20948_Status_e st = inv_icm20948_read_dmp_data(&s_imu, &dmp);
        if (st == ICM_20948_Stat_FIFONoDataAvail) {
            break;
        }
        if (st != ICM_20948_Stat_Ok && st != ICM_20948_Stat_FIFOMoreDataAvail) {
            break;
        }

        if ((dmp.header & DMP_header_bitmap_Quat9) &&
            heading_from_quat9_data(&dmp, &quat_heading)) {
            got_quat = true;
        }
        if ((dmp.header & DMP_header_bitmap_Accel) &&
            (dmp.header & DMP_header_bitmap_Compass)) {
            heading_from_accel_compass(dmp.Raw_Accel.Data.X, dmp.Raw_Accel.Data.Y,
                                       dmp.Raw_Accel.Data.Z, dmp.Compass.Data.X,
                                       dmp.Compass.Data.Y, dmp.Compass.Data.Z,
                                       &fallback_heading);
            got_fallback = true;
        }

        if (st != ICM_20948_Stat_FIFOMoreDataAvail) {
            break;
        }
    }

    if (got_quat) {
        return quat_heading;
    }
    if (got_fallback) {
        static bool s_fallback_logged = false;
        if (!s_fallback_logged) {
            s_fallback_logged = true;
            ESP_LOGI(TAG, "DMP Quat9 not ready yet, using accel+compass fallback");
        }
        return fallback_heading;
    }
    return -1.0f;
}
