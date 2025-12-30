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
* File:        bmp.h
* Author:      Kunsh Jain
* Created On:  2025-12-22
* Brief:       BMP280 sensor driver interface.
* Description: Provides basic BMP280 initialization and read interfaces.
***************************************************************************************************
* HISTORY:
* +----- (NEW | MODify | ADD | DELete)
* |
* No#   |       when       who                  what
******+*********+**********+********************+**************************************************
* 000  NEW      2025-12-22   Kunsh Jain           Added header and Doxygen
* 001  MODify   2025-12-30   Nakul Niketan        Aligned header file for updated BMP280 driver 
**************************************************************************************************/

#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include "hardware/i2c.h"

/* ===== Error Codes ===== */
#define BMP280_SUCCESS           0    /* Operation successful */
#define BMP280_ERR_COMM         -1    /* I2C communication failure */
#define BMP280_ERR_CHIP_ID      -2    /* Wrong chip ID or no response */
#define BMP280_ERR_NULL_PTR     -3    /* NULL pointer passed */
#define BMP280_ERR_INVALID_CFG  -4    /* Invalid configuration parameter */

/* ===== I2C Addresses ===== */
#define BMP280_ADDR_0           0x76  /* SDO pin = GND */
#define BMP280_ADDR_1           0x77  /* SDO pin = VCC */

/* ===== BMP280 Registers ===== */
#define BMP280_REG_CHIP_ID      0xD0  /* Chip identification register */
#define BMP280_REG_RESET        0xE0  /* Software reset register */
#define BMP280_REG_STATUS       0xF3  /* Status register (measuring/updating) */
#define BMP280_REG_DIG_T1       0x88  /* Calibration data start address */
#define BMP280_REG_CTRL_MEAS    0xF4  /* Control register (oversampling, mode) */
#define BMP280_REG_CONFIG       0xF5  /* Configuration register (filter, standby) */
#define BMP280_REG_PRESS_MSB    0xF7  /* Pressure data MSB */
#define BMP280_REG_TEMP_MSB     0xFA  /* Temperature data MSB */

/* ===== Chip Identification ===== */
#define BMP280_CHIP_ID          0x58  /* BMP280 chip ID */
#define BME280_CHIP_ID          0x60  /* BME280 chip ID (humidity variant) */

/* ===== Commands ===== */
#define BMP280_RESET_VALUE      0xB6  /* Software reset command */

/* ===== Configuration Constants ===== */
#define BMP280_CALIBRATION_SIZE 24    /* Size of calibration data in bytes */
#define BMP280_STARTUP_TIME_MS  2     /* Time to wait after reset (ms) */
#define BMP280_MAX_I2C_BURST    32    /* Maximum I2C write length */

/* ===== Oversampling Options ===== */
typedef enum {
    BMP280_OSRS_SKIP = 0,  /* Measurement skipped (output set to 0x80000) */
    BMP280_OSRS_X1   = 1,  /* Oversampling x1 */
    BMP280_OSRS_X2   = 2,  /* Oversampling x2 */
    BMP280_OSRS_X4   = 3,  /* Oversampling x4 */
    BMP280_OSRS_X8   = 4,  /* Oversampling x8 */
    BMP280_OSRS_X16  = 5   /* Oversampling x16 (highest resolution) */
} bmp280_oversampling_t;

/* ===== IIR Filter Coefficient ===== */
typedef enum {
    BMP280_FILTER_OFF = 0,  /* Filter off */
    BMP280_FILTER_2   = 1,  /* Filter coefficient = 2 */
    BMP280_FILTER_4   = 2,  /* Filter coefficient = 4 */
    BMP280_FILTER_8   = 3,  /* Filter coefficient = 8 */
    BMP280_FILTER_16  = 4   /* Filter coefficient = 16 */
} bmp280_filter_t;

/* ===== Power Modes ===== */
typedef enum {
    BMP280_SLEEP_MODE  = 0,  /* Sleep mode - no measurements */
    BMP280_FORCED_MODE = 1,  /* Forced mode - single measurement then sleep */
    BMP280_NORMAL_MODE = 3   /* Normal mode - continuous measurement */
} bmp280_mode_t;

/* ===== Standby Time (Normal Mode) ===== */
typedef enum {
    BMP280_STANDBY_0_5MS   = 0,  /* 0.5 ms standby */
    BMP280_STANDBY_62_5MS  = 1,  /* 62.5 ms standby */
    BMP280_STANDBY_125MS   = 2,  /* 125 ms standby */
    BMP280_STANDBY_250MS   = 3,  /* 250 ms standby */
    BMP280_STANDBY_500MS   = 4,  /* 500 ms standby */
    BMP280_STANDBY_1000MS  = 5,  /* 1000 ms standby */
    BMP280_STANDBY_2000MS  = 6,  /* 2000 ms standby */
    BMP280_STANDBY_4000MS  = 7   /* 4000 ms standby */
} bmp280_standby_t;

/* ===== Device Structure ===== */
typedef struct {
    /* Factory calibration coefficients (read from sensor EEPROM) */
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Runtime data */
    int32_t      t_fine;   /* Fine temperature value (used for pressure compensation) */
    uint8_t      address;  /* I2C device address (0x76 or 0x77) */
    i2c_inst_t  *i2c;      /* Pointer to I2C instance (i2c0 or i2c1) */
} bmp280_t;

/* ===== Core API Functions ===== */

/**
 * @brief Initialize BMP280 sensor with rocketry-optimized configuration
 * 
 * This function initializes the sensor and automatically configures it for
 * rocketry applications with optimal settings for altitude tracking.
 * 
 * @param dev     Pointer to device structure (must not be NULL)
 * @param i2c     Pointer to I2C instance (i2c0 or i2c1)
 * @param address I2C device address (BMP280_ADDR_0 or BMP280_ADDR_1)
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR  : dev or i2c is NULL
 *         - BMP280_ERR_COMM      : I2C communication failure
 *         - BMP280_ERR_CHIP_ID   : Wrong chip ID (not BMP280/BME280)
 * 
 * @note Automatically applies rocketry configuration:
 *       - Temperature oversampling: x2 (sufficient accuracy, faster)
 *       - Pressure oversampling: x16 (maximum precision for altitude)
 *       - IIR filter: coefficient 4 (noise reduction without lag)
 *       - Power mode: NORMAL (continuous measurement)
 *       - Standby time: 62.5 ms (~16 Hz sampling rate)
 *       
 *       Configuration can be changed after initialization using
 *       bmp280_set_config() and bmp280_set_standby()
 */
int bmp280_init(bmp280_t *dev, i2c_inst_t *i2c, uint8_t address);

/**
 * @brief Configure sensor oversampling, filter, and power mode
 * 
 * @param dev        Pointer to device structure
 * @param temp_osrs  Temperature oversampling (BMP280_OSRS_SKIP to BMP280_OSRS_X16)
 * @param press_osrs Pressure oversampling (BMP280_OSRS_SKIP to BMP280_OSRS_X16)
 * @param filter     IIR filter coefficient (BMP280_FILTER_OFF to BMP280_FILTER_16)
 * @param mode       Power mode (BMP280_SLEEP_MODE/FORCED_MODE/NORMAL_MODE)
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR    : dev is NULL
 *         - BMP280_ERR_INVALID_CFG : Invalid parameter value
 *         - BMP280_ERR_COMM        : I2C communication failure
 * 
 * @note This function preserves the existing standby time setting.
 *       Changing mode from SLEEP to NORMAL will start continuous measurements.
 */
int bmp280_set_config(bmp280_t *dev,
                      bmp280_oversampling_t temp_osrs,
                      bmp280_oversampling_t press_osrs,
                      bmp280_filter_t filter,
                      bmp280_mode_t mode);

/**
 * @brief Set standby time between measurements in normal mode
 * 
 * @param dev          Pointer to device structure
 * @param standby_time Standby duration (BMP280_STANDBY_0_5MS to BMP280_STANDBY_4000MS)
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR    : dev is NULL
 *         - BMP280_ERR_INVALID_CFG : Invalid standby_time value
 *         - BMP280_ERR_COMM        : I2C communication failure
 * 
 * @note Standby time only applies in NORMAL mode. In FORCED mode, the sensor
 *       returns to sleep immediately after measurement.
 *       Shorter standby = faster sampling but higher power consumption.
 */
int bmp280_set_standby(bmp280_t *dev, bmp280_standby_t standby_time);

/**
 * @brief Read temperature from sensor
 * 
 * @param dev         Pointer to device structure
 * @param temperature Pointer to store temperature in degrees Celsius
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR : dev or temperature is NULL
 *         - BMP280_ERR_COMM     : I2C communication failure
 * 
 * @note This function may block up to 50ms waiting for measurement to complete.
 *       Temperature reading also updates the internal t_fine variable used
 *       for pressure compensation.
 */
int bmp280_read_temperature(bmp280_t *dev, float *temperature);

/**
 * @brief Read pressure from sensor
 * 
 * @param dev      Pointer to device structure
 * @param pressure Pointer to store pressure in Pascals (Pa)
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR : dev or pressure is NULL
 *         - BMP280_ERR_COMM     : I2C communication failure
 * 
 * @note This function internally reads temperature first (required for pressure
 *       compensation), then reads pressure. Total blocking time may be up to 50ms.
 *       To convert to hPa (hectopascals), divide result by 100.0
 */
int bmp280_read_pressure(bmp280_t *dev, float *pressure);

/**
 * @brief Calculate altitude from atmospheric pressure
 * 
 * Uses the barometric formula to calculate altitude based on current pressure
 * and sea level reference pressure.
 * 
 * @param dev           Pointer to device structure
 * @param sea_level_hPa Reference sea level pressure in hPa (default: 1013.25)
 * @param altitude      Pointer to store calculated altitude in meters
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR : dev or altitude is NULL
 *         - BMP280_ERR_COMM     : I2C communication failure
 * 
 * @note For accurate altitude:
 *       - Calibrate sea_level_hPa at ground level before flight
 *       - This function reads pressure internally (may block up to 50ms)
 *       - Formula assumes standard atmosphere (accurate to ~±10m)
 */
int bmp280_read_altitude(bmp280_t *dev, float sea_level_hPa, float *altitude);

/**
 * @brief Trigger single measurement in forced mode
 * 
 * Initiates a single measurement cycle. Sensor automatically returns to
 * sleep mode after measurement completes.
 * 
 * @param dev Pointer to device structure
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR : dev is NULL
 *         - BMP280_ERR_COMM     : I2C communication failure
 * 
 * @note Useful for low-power applications. After calling this function:
 *       1. Wait for measurement completion using bmp280_is_measuring()
 *       2. Read temperature/pressure values
 *       3. Sensor returns to sleep mode automatically
 */
int bmp280_force_measurement(bmp280_t *dev);

/**
 * @brief Check if sensor is currently performing a measurement
 * 
 * @param dev       Pointer to device structure
 * @param measuring Pointer to store measuring status:
 *                  - 1 = sensor is measuring/busy
 *                  - 0 = sensor is idle/ready
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR : dev or measuring is NULL
 *         - BMP280_ERR_COMM     : I2C communication failure
 * 
 * @note Useful when using forced mode to poll for measurement completion.
 */
int bmp280_is_measuring(bmp280_t *dev, uint8_t *measuring);

/**
 * @brief Perform software reset of sensor
 * 
 * Resets the sensor to power-on defaults. All registers return to default
 * values and calibration data must be re-read.
 * 
 * @param dev Pointer to device structure
 * 
 * @return BMP280_SUCCESS on success, error code otherwise:
 *         - BMP280_ERR_NULL_PTR : dev is NULL
 *         - BMP280_ERR_COMM     : I2C communication failure
 * 
 * @note This function blocks for 2ms to allow sensor to complete reset.
 *       After reset, sensor is in sleep mode with default settings.
 *       Called automatically by bmp280_init().
 */
int bmp280_reset(bmp280_t *dev);

#endif /* BMP280_H */
