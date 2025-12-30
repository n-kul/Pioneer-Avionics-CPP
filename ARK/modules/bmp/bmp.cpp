/*
                                        ::                                                      
                                        ::                                                      
                                        ::                                                      
                                        ::                                                      
                                        ::                                                      
    ..    ..........    :.      ::      ::     .........  ..    ..........    ...      .        
    ::    ::            : .:.   ::     .::.       ::      ::    ::       :    :: :.    :        
    ::    ::   ..:::    :   .:. ::    ::::::      ::      ::    ::       :    ::   ::  :        
    ::    ::......::    :      :::    ::::::      ::      ::    ::.......:    ::     :::        
                                      ::::::                                                    
                                      :.::.:                                                    
                         .::::          ::          ::::.                                       
                       .::::::::.       ::       .:::::::::                                   
                       ::::::::::::....::::.....:::::::::::                                   
                        .:::::::::::::::::::::::::::::::::.        

                  © Copyright of Ignition Avionics
*/

/**************************************************************************************************
* File:        bmp.cpp
* Author:      Kunsh Jain
* Created On:  2025-12-22
* Brief:       BMP280 sensor driver implementation.
* Description: Implements initialization, calibration and raw reads for BMP280.
***************************************************************************************************
* HISTORY:
* +----- (NEW | MODify | ADD | DELete)
* |
* No#   |       when       who                  what
******+*********+**********+********************+**************************************************
* 000  NEW      2025-12-22   Kunsh Jain           Added header and Doxygen
* 001  MODify   2025-12-30   Nakul Niketan        Corrected BMP280 configuration and read timing
**************************************************************************************************/

#include "bmp280.h"
#include "pico/time.h"
#include <math.h>
#include <stdbool.h>

/* ===== Status Register Bits ===== */
#define BMP280_STATUS_MEASURING  (1 << 3)
#define BMP280_STATUS_IM_UPDATE (1 << 0)

/* ===== Internal Helper Functions ===== */

static inline uint16_t read_u16_le(const uint8_t *buf) {
    return (uint16_t)((buf[1] << 8) | buf[0]);
}

static inline int16_t read_s16_le(const uint8_t *buf) {
    return (int16_t)((buf[1] << 8) | buf[0]);
}

static inline bool is_valid_oversampling(bmp280_oversampling_t osrs) {
    return osrs <= BMP280_OSRS_X16;
}

static inline bool is_valid_filter(bmp280_filter_t filter) {
    return filter <= BMP280_FILTER_16;
}

static inline bool is_valid_mode(bmp280_mode_t mode) {
    return (mode == BMP280_SLEEP_MODE ||
            mode == BMP280_FORCED_MODE ||
            mode == BMP280_NORMAL_MODE);
}

/* ===== I2C Communication ===== */

static int i2c_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg,
                        uint8_t *data, uint16_t len) {
    int ret = i2c_write_blocking(i2c, addr, &reg, 1, true);
    if (ret < 0) return BMP280_ERR_COMM;

    ret = i2c_read_blocking(i2c, addr, data, len, false);
    if (ret < 0) return BMP280_ERR_COMM;

    return BMP280_SUCCESS;
}

static int i2c_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg,
                         const uint8_t *data, uint16_t len) {
    if (len > BMP280_MAX_I2C_BURST) {
        return BMP280_ERR_COMM;
    }

    uint8_t buf[BMP280_MAX_I2C_BURST + 1];
    buf[0] = reg;
    for (uint16_t i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }

    int ret = i2c_write_blocking(i2c, addr, buf, len + 1, false);
    if (ret < 0) return BMP280_ERR_COMM;

    return BMP280_SUCCESS;
}

/* ===== Measurement Wait Helper (FIXED) ===== */

static int bmp280_wait_measuring(bmp280_t *dev, uint32_t timeout_ms) {
    absolute_time_t start = get_absolute_time();
    uint8_t measuring;

    do {
        int ret = i2c_read_reg(dev->i2c, dev->address, 0xF3, &measuring, 1);
        if (ret != BMP280_SUCCESS) return ret;

        if (!(measuring & BMP280_STATUS_MEASURING)) {
            return BMP280_SUCCESS;
        }

        sleep_ms(1);
    } while (absolute_time_diff_us(start, get_absolute_time()) <
             (int64_t)timeout_ms * 1000);

    return BMP280_ERR_COMM;
}

/* ===== Core Driver Functions ===== */

int bmp280_init(bmp280_t *dev, i2c_inst_t *i2c, uint8_t address) {
    if (!dev || !i2c) return BMP280_ERR_NULL_PTR;

    dev->i2c = i2c;
    dev->address = address;

    uint8_t chip_id;
    int ret = i2c_read_reg(i2c, address, BMP280_REG_CHIP_ID, &chip_id, 1);
    if (ret != BMP280_SUCCESS) return ret;

    if (chip_id != BMP280_CHIP_ID && chip_id != BME280_CHIP_ID) {
        return BMP280_ERR_CHIP_ID;
    }

    uint8_t calib[BMP280_CALIBRATION_SIZE];
    ret = i2c_read_reg(i2c, address, BMP280_REG_DIG_T1,
                       calib, BMP280_CALIBRATION_SIZE);
    if (ret != BMP280_SUCCESS) return ret;

    dev->dig_T1 = read_u16_le(&calib[0]);
    dev->dig_T2 = read_s16_le(&calib[2]);
    dev->dig_T3 = read_s16_le(&calib[4]);
    dev->dig_P1 = read_u16_le(&calib[6]);
    dev->dig_P2 = read_s16_le(&calib[8]);
    dev->dig_P3 = read_s16_le(&calib[10]);
    dev->dig_P4 = read_s16_le(&calib[12]);
    dev->dig_P5 = read_s16_le(&calib[14]);
    dev->dig_P6 = read_s16_le(&calib[16]);
    dev->dig_P7 = read_s16_le(&calib[18]);
    dev->dig_P8 = read_s16_le(&calib[20]);
    dev->dig_P9 = read_s16_le(&calib[22]);

    ret = bmp280_reset(dev);
    if (ret != BMP280_SUCCESS) return ret;

    ret = bmp280_set_config(dev,
                            BMP280_OSRS_X2,
                            BMP280_OSRS_X16,
                            BMP280_FILTER_4,
                            BMP280_NORMAL_MODE);
    if (ret != BMP280_SUCCESS) return ret;

    ret = bmp280_set_standby(dev, BMP280_STANDBY_62_5MS);
    if (ret != BMP280_SUCCESS) return ret;

    return BMP280_SUCCESS;
}

/* ===== CONFIG FIXED (standby preserved) ===== */

int bmp280_set_config(bmp280_t *dev,
                      bmp280_oversampling_t temp_osrs,
                      bmp280_oversampling_t press_osrs,
                      bmp280_filter_t filter,
                      bmp280_mode_t mode) {
    if (!dev) return BMP280_ERR_NULL_PTR;

    if (!is_valid_oversampling(temp_osrs) ||
        !is_valid_oversampling(press_osrs) ||
        !is_valid_filter(filter) ||
        !is_valid_mode(mode)) {
        return BMP280_ERR_INVALID_CFG;
    }

    uint8_t ctrl = (temp_osrs << 5) | (press_osrs << 2) | mode;

    uint8_t config;
    int ret = i2c_read_reg(dev->i2c, dev->address,
                           BMP280_REG_CONFIG, &config, 1);
    if (ret != BMP280_SUCCESS) return ret;

    config = (config & 0xE3) | (filter << 2);

    ret = i2c_write_reg(dev->i2c, dev->address,
                        BMP280_REG_CTRL_MEAS, &ctrl, 1);
    if (ret != BMP280_SUCCESS) return ret;

    ret = i2c_write_reg(dev->i2c, dev->address,
                        BMP280_REG_CONFIG, &config, 1);
    return ret;
}

int bmp280_set_standby(bmp280_t *dev, bmp280_standby_t standby_time) {
    if (!dev) return BMP280_ERR_NULL_PTR;
    if (standby_time > BMP280_STANDBY_4000MS) return BMP280_ERR_INVALID_CFG;

    uint8_t config;
    int ret = i2c_read_reg(dev->i2c, dev->address,
                           BMP280_REG_CONFIG, &config, 1);
    if (ret != BMP280_SUCCESS) return ret;

    config = (config & 0x1F) | (standby_time << 5);

    return i2c_write_reg(dev->i2c, dev->address,
                         BMP280_REG_CONFIG, &config, 1);
}

int bmp280_read_temperature(bmp280_t *dev, float *temperature) {
    if (!dev || !temperature) return BMP280_ERR_NULL_PTR;

    bmp280_wait_measuring(dev, 50);

    uint8_t buf[3];
    int ret = i2c_read_reg(dev->i2c, dev->address,
                           BMP280_REG_TEMP_MSB, buf, 3);
    if (ret != BMP280_SUCCESS) return ret;

    int32_t adc_T = ((int32_t)buf[0] << 12) |
                    ((int32_t)buf[1] << 4) |
                    (buf[2] >> 4);

    int32_t var1 = ((((adc_T >> 3) -
                     ((int32_t)dev->dig_T1 << 1))) *
                     dev->dig_T2) >> 11;

    int32_t var2 = (((((adc_T >> 4) - dev->dig_T1) *
                     ((adc_T >> 4) - dev->dig_T1)) >> 12) *
                     dev->dig_T3) >> 14;

    dev->t_fine = var1 + var2;

    *temperature = ((dev->t_fine * 5 + 128) >> 8) / 100.0f;
    return BMP280_SUCCESS;
}

int bmp280_read_pressure(bmp280_t *dev, float *pressure) {
    if (!dev || !pressure) return BMP280_ERR_NULL_PTR;

    float temp;
    int ret = bmp280_read_temperature(dev, &temp);
    if (ret != BMP280_SUCCESS) return ret;

    bmp280_wait_measuring(dev, 50);

    uint8_t buf[3];
    ret = i2c_read_reg(dev->i2c, dev->address,
                       BMP280_REG_PRESS_MSB, buf, 3);
    if (ret != BMP280_SUCCESS) return ret;

    int32_t adc_P = ((int32_t)buf[0] << 12) |
                    ((int32_t)buf[1] << 4) |
                    (buf[2] >> 4);

    int64_t var1 = (int64_t)dev->t_fine - 128000;
    int64_t var2 = var1 * var1 * dev->dig_P6;
    var2 += (var1 * dev->dig_P5) << 17;
    var2 += ((int64_t)dev->dig_P4) << 35;

    var1 = ((var1 * var1 * dev->dig_P3) >> 8) +
           ((var1 * dev->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * dev->dig_P1 >> 33;

    if (var1 == 0) {
        *pressure = 0.0f;
        return BMP280_SUCCESS;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (dev->dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dev->dig_P7 << 4);
    *pressure = (float)p / 256.0f;

    return BMP280_SUCCESS;
}

int bmp280_read_altitude(bmp280_t *dev, float sea_level_hPa, float *altitude) {
    if (!dev || !altitude) return BMP280_ERR_NULL_PTR;

    float pressure_Pa;
    int ret = bmp280_read_pressure(dev, &pressure_Pa);
    if (ret != BMP280_SUCCESS) return ret;

    float pressure_hPa = pressure_Pa / 100.0f;
    *altitude = 44330.0f *
                (1.0f - powf(pressure_hPa / sea_level_hPa, 0.1903f));
    return BMP280_SUCCESS;
}

int bmp280_force_measurement(bmp280_t *dev) {
    if (!dev) return BMP280_ERR_NULL_PTR;

    uint8_t ctrl;
    int ret = i2c_read_reg(dev->i2c, dev->address,
                           BMP280_REG_CTRL_MEAS, &ctrl, 1);
    if (ret != BMP280_SUCCESS) return ret;

    ctrl = (ctrl & 0xFC) | BMP280_FORCED_MODE;
    return i2c_write_reg(dev->i2c, dev->address,
                         BMP280_REG_CTRL_MEAS, &ctrl, 1);
}

int bmp280_is_measuring(bmp280_t *dev, uint8_t *measuring) {
    if (!dev || !measuring) return BMP280_ERR_NULL_PTR;

    uint8_t status;
    int ret = i2c_read_reg(dev->i2c, dev->address, 0xF3, &status, 1);
    if (ret != BMP280_SUCCESS) return ret;

    *measuring = (status & BMP280_STATUS_MEASURING) ? 1 : 0;
    return BMP280_SUCCESS;
}

int bmp280_reset(bmp280_t *dev) {
    if (!dev) return BMP280_ERR_NULL_PTR;

    uint8_t cmd = BMP280_RESET_VALUE;
    int ret = i2c_write_reg(dev->i2c, dev->address,
                            BMP280_REG_RESET, &cmd, 1);
    if (ret != BMP280_SUCCESS) return ret;

    sleep_ms(BMP280_STARTUP_TIME_MS);
    return BMP280_SUCCESS;
}
