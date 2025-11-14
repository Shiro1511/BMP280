/**
 ******************************************************************************
 * @file           : BMP280.h
 * @brief          : Header file for BMP280 pressure and temperature sensor driver.
 *                   Provides definitions, enums, data structures, and function
 *                   prototypes to interface with the BMP280 via I2C.
 ******************************************************************************
 * @attention
 *
 * The BMP280 is a combined digital pressure and temperature sensor.
 * This driver provides initialization, configuration, and data reading
 * functions for easy integration in STM32-based projects.
 *
 * Typical usage:
 * @code
 *   BMP280_HandleTypeDef hbmp280;
 *   BMP280_Init(&hbmp280, &hi2c1);
 *   float temp, pres;
 *   BMP280_Read_Temperature_And_Pressure(&hbmp280, &temp, &pres);
 * @endcode
 *
 ******************************************************************************
 */

#ifndef _BMP280_H_
#define _BMP280_H_

#include "stm32f1xx_hal.h"

/* -------------------------------------------------------------------------- */
/*                               BMP280 Address                               */
/* -------------------------------------------------------------------------- */

/** @brief BMP280 I2C address definitions */
#define BMP280_I2C_ADDR_WRITE 0xEC /*!< I2C write address */
#define BMP280_I2C_ADDR_READ 0xED  /*!< I2C read address  */

/* -------------------------------------------------------------------------- */
/*                               Register Map                                 */
/* -------------------------------------------------------------------------- */

#define BMP280_REG_ID 0xD0        /*!< Chip ID register */
#define BMP280_REG_RESET 0xE0     /*!< Soft reset register */
#define BMP280_REG_STATUS 0xF3    /*!< Status register */
#define BMP280_REG_CTRL_MEAS 0xF4 /*!< Measurement control register */
#define BMP280_REG_CONFIG 0xF5    /*!< Configuration register */
#define BMP280_REG_PRESS_MSB 0xF7 /*!< Pressure data MSB */
#define BMP280_REG_PRESS_LSB 0xF8
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_XLSB 0xFC

/* Calibration registers (0x88–0x9F) */
#define BMP280_DIG_T1_LSB 0x88
#define BMP280_DIG_T1_MSB 0x89
#define BMP280_DIG_T2_LSB 0x8A
#define BMP280_DIG_T2_MSB 0x8B
#define BMP280_DIG_T3_LSB 0x8C
#define BMP280_DIG_T3_MSB 0x8D
#define BMP280_DIG_P1_LSB 0x8E
#define BMP280_DIG_P1_MSB 0x8F
#define BMP280_DIG_P2_LSB 0x90
#define BMP280_DIG_P2_MSB 0x91
#define BMP280_DIG_P3_LSB 0x92
#define BMP280_DIG_P3_MSB 0x93
#define BMP280_DIG_P4_LSB 0x94
#define BMP280_DIG_P4_MSB 0x95
#define BMP280_DIG_P5_LSB 0x96
#define BMP280_DIG_P5_MSB 0x97
#define BMP280_DIG_P6_LSB 0x98
#define BMP280_DIG_P6_MSB 0x99
#define BMP280_DIG_P7_LSB 0x9A
#define BMP280_DIG_P7_MSB 0x9B
#define BMP280_DIG_P8_LSB 0x9C
#define BMP280_DIG_P8_MSB 0x9D
#define BMP280_DIG_P9_LSB 0x9E
#define BMP280_DIG_P9_MSB 0x9F
#define BMP280_RESERVED_REG 0xA0

#define BMP280_CALIB_DATA_SIZE 24  /*!< Number of calibration bytes */
#define BMP280_NUM_CALIB_PARAMS 12 /*!< Number of calibration parameters */

/* -------------------------------------------------------------------------- */
/*                               Enumerations                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief BMP280 power modes.
 */
typedef enum
{
    BMP280_MODE_SLEEP = 0x00,  /*!< Sleep mode */
    BMP280_MODE_FORCED = 0x01, /*!< Forced mode (one-shot measurement) */
    BMP280_MODE_NORMAL = 0x03  /*!< Continuous measurement */
} BMP280_Mode_t;

/**
 * @brief BMP280 oversampling settings.
 */
typedef enum
{
    BMP280_OS_SKIPPED = 0x00, /*!< Skip measurement */
    BMP280_OS_1X = 0x01,      /*!< Oversampling x1 */
    BMP280_OS_2X = 0x02,      /*!< Oversampling x2 */
    BMP280_OS_4X = 0x03,      /*!< Oversampling x4 */
    BMP280_OS_8X = 0x04,      /*!< Oversampling x8 */
    BMP280_OS_16X = 0x05      /*!< Oversampling x16 */
} BMP280_Oversampling_t;

/**
 * @brief BMP280 IIR filter settings.
 */
typedef enum
{
    BMP280_FILTER_OFF = 0x00, /*!< Filter off */
    BMP280_FILTER_2 = 0x01,   /*!< Coefficient 2 */
    BMP280_FILTER_4 = 0x02,   /*!< Coefficient 4 */
    BMP280_FILTER_8 = 0x03,   /*!< Coefficient 8 */
    BMP280_FILTER_16 = 0x04   /*!< Coefficient 16 */
} BMP280_Filter_t;

/**
 * @brief BMP280 standby time settings (in normal mode).
 */
typedef enum
{
    BMP280_STANDBY_0_5_MS = 0x00,
    BMP280_STANDBY_62_5_MS = 0x01,
    BMP280_STANDBY_125_MS = 0x02,
    BMP280_STANDBY_250_MS = 0x03,
    BMP280_STANDBY_500_MS = 0x04,
    BMP280_STANDBY_1000_MS = 0x05,
    BMP280_STANDBY_2000_MS = 0x06,
    BMP280_STANDBY_4000_MS = 0x07
} BMP280_Standby_Time_t;

/* -------------------------------------------------------------------------- */
/*                              Data Structures                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Structure for sensor output values.
 */
typedef struct
{
    float temperature; /*!< Temperature in °C */
    float pressure;    /*!< Pressure in Pa */
    float altitude;    /*!< Calculated altitude in meters */
} BMP280_Value_t;

/**
 * @brief  Structure containing calibration coefficients from the BMP280.
 */
typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine; /*!< Intermediate temperature value for compensation */
} BMP280_Calib_Data_t;

/**
 * @brief  BMP280 configuration structure.
 */
typedef struct
{
    BMP280_Mode_t mode;                 /*!< Operating mode */
    BMP280_Oversampling_t osrs_t;       /*!< Temperature oversampling */
    BMP280_Oversampling_t osrs_p;       /*!< Pressure oversampling */
    BMP280_Filter_t filter;             /*!< IIR filter coefficient */
    BMP280_Standby_Time_t standby_time; /*!< Standby time in normal mode */
} BMP280_Config_t;

/**
 * @brief  BMP280 device handle structure.
 * @note   Contains I2C handle, calibration data, configuration, and I2C address.
 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;        /*!< Pointer to I2C handle */
    BMP280_Calib_Data_t calib_data; /*!< Calibration coefficients */
    BMP280_Config_t config;         /*!< Sensor configuration */
    uint8_t i2c_addr;               /*!< I2C address of BMP280 */
} BMP280_HandleTypeDef;

/* -------------------------------------------------------------------------- */
/*                             Function Prototypes                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize the BMP280 sensor.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @param  hi2c: Pointer to I2C handle (e.g. &hi2c1).
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Init(BMP280_HandleTypeDef *bmp280x, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Reset the BMP280 device.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Reset(BMP280_HandleTypeDef *bmp280x);

/**
 * @brief  Configure BMP280 measurement settings.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @param  config: Pointer to configuration structure.
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Set_Config(BMP280_HandleTypeDef *bmp280x, BMP280_Config_t *config);

/**
 * @brief  Read current configuration from BMP280.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @param  config: Pointer to configuration structure to store values.
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Get_Config(BMP280_HandleTypeDef *bmp280x, BMP280_Config_t *config);

/**
 * @brief  Read compensated temperature and pressure values.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @param  temperature: Pointer to store temperature (°C).
 * @param  pressure: Pointer to store pressure (hPa).
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Read_Temperature_And_Pressure(BMP280_HandleTypeDef *bmp280x, float *temperature, float *pressure);

/**
 * @brief  Read temperature only.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @param  temperature: Pointer to store temperature (°C).
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Read_Temperature(BMP280_HandleTypeDef *bmp280x, float *temperature);

/**
 * @brief  Read pressure only.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @param  pressure: Pointer to store pressure (Pa).
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Read_Pressure(BMP280_HandleTypeDef *bmp280x, float *pressure);

/**
 * @brief  Read BMP280 device ID (should return 0x58).
 * @param  bmp280x: Pointer to BMP280 handle.
 * @param  chip_id: Pointer to variable for storing chip ID.
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Get_ID(BMP280_HandleTypeDef *bmp280x, uint8_t *chip_id);

/**
 * @brief  Check if BMP280 is currently performing a measurement.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Is_Measuring(BMP280_HandleTypeDef *bmp280x);

/**
 * @brief  Read calibration data from sensor’s NVM.
 * @param  bmp280x: Pointer to BMP280 handle.
 * @retval HAL status.
 */
HAL_StatusTypeDef BMP280_Read_Calibration_Data(BMP280_HandleTypeDef *bmp280x);

#endif /* _BMP280_H_ */
