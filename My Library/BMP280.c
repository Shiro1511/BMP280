#include "BMP280.h"
#include "math.h"

/* Private function to read register */
static HAL_StatusTypeDef BMP280_Read_Register(BMP280_HandleTypeDef *bmp280x, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(bmp280x->hi2c, BMP280_I2C_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/* Private function to write register */
static HAL_StatusTypeDef BMP280_Write_Register(BMP280_HandleTypeDef *bmp280x, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Write(bmp280x->hi2c, BMP280_I2C_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/* Set configuration */
HAL_StatusTypeDef BMP280_Set_Config(BMP280_HandleTypeDef *bmp280x, BMP280_Config_t *config)
{
    uint8_t ctrl_meas, config_reg;
    HAL_StatusTypeDef status;

    if (bmp280x == NULL || config == NULL)
    {
        return HAL_ERROR;
    }

    /* Prepare CTRL_MEAS register */
    ctrl_meas = (config->osrs_t << 5) | (config->osrs_p << 2) | config->mode;

    /* Prepare CONFIG register */
    config_reg = (config->standby_time << 5) | (config->filter << 2);

    /* Write registers */
    status = BMP280_Write_Register(bmp280x, BMP280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BMP280_Write_Register(bmp280x, BMP280_REG_CONFIG, &config_reg, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Update device configuration */
    bmp280x->config = *config;

    return HAL_OK;
}

/* Get chip ID */
HAL_StatusTypeDef BMP280_Get_ID(BMP280_HandleTypeDef *bmp280x, uint8_t *chip_id)
{
    if (bmp280x == NULL || chip_id == NULL)
    {
        return HAL_ERROR;
    }

    return BMP280_Read_Register(bmp280x, BMP280_REG_ID, chip_id, 1);
}

/* Read calibration data */
HAL_StatusTypeDef BMP280_Read_Calibration_Data(BMP280_HandleTypeDef *bmp280x)
{
    uint8_t calib_data[BMP280_CALIB_DATA_SIZE];
    HAL_StatusTypeDef status;

    status = BMP280_Read_Register(bmp280x, BMP280_DIG_T1_LSB, calib_data, BMP280_CALIB_DATA_SIZE);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Parse calibration data */
    bmp280x->calib_data.dig_T1 = (uint16_t)((calib_data[1] << 8) | calib_data[0]);
    bmp280x->calib_data.dig_T2 = (int16_t)((calib_data[3] << 8) | calib_data[2]);
    bmp280x->calib_data.dig_T3 = (int16_t)((calib_data[5] << 8) | calib_data[4]);
    bmp280x->calib_data.dig_P1 = (uint16_t)((calib_data[7] << 8) | calib_data[6]);
    bmp280x->calib_data.dig_P2 = (int16_t)((calib_data[9] << 8) | calib_data[8]);
    bmp280x->calib_data.dig_P3 = (int16_t)((calib_data[11] << 8) | calib_data[10]);
    bmp280x->calib_data.dig_P4 = (int16_t)((calib_data[13] << 8) | calib_data[12]);
    bmp280x->calib_data.dig_P5 = (int16_t)((calib_data[15] << 8) | calib_data[14]);
    bmp280x->calib_data.dig_P6 = (int16_t)((calib_data[17] << 8) | calib_data[16]);
    bmp280x->calib_data.dig_P7 = (int16_t)((calib_data[19] << 8) | calib_data[18]);
    bmp280x->calib_data.dig_P8 = (int16_t)((calib_data[21] << 8) | calib_data[20]);
    bmp280x->calib_data.dig_P9 = (int16_t)((calib_data[23] << 8) | calib_data[22]);

    bmp280x->calib_data.t_fine = 0;

    return HAL_OK;
}

/* Reset BMP280 */
HAL_StatusTypeDef BMP280_Reset(BMP280_HandleTypeDef *bmp280x)
{
    uint8_t reset_cmd = 0xB6; /* Reset command */

    return BMP280_Write_Register(bmp280x, BMP280_REG_RESET, &reset_cmd, 1);
}

/* Read raw temperature and pressure data */
static HAL_StatusTypeDef BMP280_Read_Raw_Data(BMP280_HandleTypeDef *bmp280x, int32_t *raw_temp, int32_t *raw_press)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = BMP280_Read_Register(bmp280x, BMP280_REG_PRESS_MSB, data, 6);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Pressure data (20 bits) */
    if (raw_press != NULL)
    {
        *raw_press = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));
    }

    /* Temperature data (20 bits) */
    if (raw_temp != NULL)
    {
        *raw_temp = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4));
    }

    return HAL_OK;
}

static float BMP280_Compensate_Temperature(BMP280_HandleTypeDef *bmp280x, int32_t raw_temp)
{
    int32_t var1, var2;

    var1 = ((((raw_temp >> 3) - ((int32_t)bmp280x->calib_data.dig_T1 << 1))) *
            ((int32_t)bmp280x->calib_data.dig_T2)) >>
           11;

    var2 = (((((raw_temp >> 4) - ((int32_t)bmp280x->calib_data.dig_T1)) *
              ((raw_temp >> 4) - ((int32_t)bmp280x->calib_data.dig_T1))) >>
             12) *
            ((int32_t)bmp280x->calib_data.dig_T3)) >>
           14;

    bmp280x->calib_data.t_fine = var1 + var2;
    return ((bmp280x->calib_data.t_fine * 5 + 128) >> 8) / 100.0f;
}

static float BMP280_Compensate_Pressure(BMP280_HandleTypeDef *bmp280x, int32_t raw_press)
{
    int64_t var1, var2, pressure;

    var1 = ((int64_t)bmp280x->calib_data.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280x->calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280x->calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280x->calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280x->calib_data.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp280x->calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280x->calib_data.dig_P1) >> 33;

    if (var1 == 0)
        return 0.0f;

    pressure = 1048576 - raw_press;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280x->calib_data.dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = (((int64_t)bmp280x->calib_data.dig_P8) * pressure) >> 19;

    pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)bmp280x->calib_data.dig_P7) << 4);

    return (float)pressure / 25600.0f; // Convert to hPa
}

HAL_StatusTypeDef BMP280_Read_Temperature_And_Pressure(BMP280_HandleTypeDef *bmp280x, float *temperature, float *pressure)
{
    int32_t raw_temp, raw_press;
    HAL_StatusTypeDef status;

    if (bmp280x == NULL || temperature == NULL || pressure == NULL)
        return HAL_ERROR;

    status = BMP280_Read_Raw_Data(bmp280x, &raw_temp, &raw_press);
    if (status != HAL_OK)
        return status;

    *temperature = BMP280_Compensate_Temperature(bmp280x, raw_temp);
    *pressure = BMP280_Compensate_Pressure(bmp280x, raw_press);

    return HAL_OK;
}

HAL_StatusTypeDef BMP280_Get_Config(BMP280_HandleTypeDef *bmp280x, BMP280_Config_t *config)
{
    uint8_t ctrl_meas, config_reg;
    HAL_StatusTypeDef status;

    if (bmp280x == NULL || config == NULL)
        return HAL_ERROR;

    status = BMP280_Read_Register(bmp280x, BMP280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (status != HAL_OK)
        return status;

    status = BMP280_Read_Register(bmp280x, BMP280_REG_CONFIG, &config_reg, 1);
    if (status != HAL_OK)
        return status;

    config->osrs_t = (ctrl_meas >> 5) & 0x07;
    config->osrs_p = (ctrl_meas >> 2) & 0x07;
    config->mode = ctrl_meas & 0x03;
    config->standby_time = (config_reg >> 5) & 0x07;
    config->filter = (config_reg >> 2) & 0x07;

    return HAL_OK;
}

/* Check if device is measuring */
HAL_StatusTypeDef BMP280_Is_Measuring(BMP280_HandleTypeDef *bmp280x)
{
    uint8_t status;
    if (BMP280_Read_Register(bmp280x, BMP280_REG_STATUS, &status, 1) != HAL_OK)
        return HAL_ERROR;

    return (status & 0x08) ? HAL_OK : HAL_ERROR;
}

/* Read temperature only */
HAL_StatusTypeDef BMP280_Read_Temperature(BMP280_HandleTypeDef *bmp280x, float *temperature)
{
    int32_t raw_temp;
    HAL_StatusTypeDef status;

    if (bmp280x == NULL || temperature == NULL)
        return HAL_ERROR;

    status = BMP280_Read_Raw_Data(bmp280x, &raw_temp, NULL);
    if (status != HAL_OK)
        return status;

    *temperature = BMP280_Compensate_Temperature(bmp280x, raw_temp);
    return HAL_OK;
}

/* Read pressure only */
HAL_StatusTypeDef BMP280_Read_Pressure(BMP280_HandleTypeDef *bmp280x, float *pressure)
{
    int32_t raw_temp, raw_press;
    HAL_StatusTypeDef status;

    if (bmp280x == NULL || pressure == NULL)
        return HAL_ERROR;

    status = BMP280_Read_Raw_Data(bmp280x, &raw_temp, &raw_press);
    if (status != HAL_OK)
        return status;

    /* Compensate temperature first to update t_fine */
    BMP280_Compensate_Temperature(bmp280x, raw_temp);
    *pressure = BMP280_Compensate_Pressure(bmp280x, raw_press);

    return HAL_OK;
}

/* Initialize BMP280 */
HAL_StatusTypeDef BMP280_Init(BMP280_HandleTypeDef *bmp280x, I2C_HandleTypeDef *hi2c)
{
    uint8_t chip_id;
    HAL_StatusTypeDef status;

    if (bmp280x == NULL || hi2c == NULL)
    {
        return HAL_ERROR;
    }

    bmp280x->hi2c = hi2c;

    /* Check chip ID */
    status = BMP280_Get_ID(bmp280x, &chip_id);
    if (status != HAL_OK)
    {
        return status;
    }

    if (chip_id != 0x58) /* BMP280 chip ID is 0x58 */
    {
        return HAL_ERROR;
    }

    /* Reset device */
    status = BMP280_Reset(bmp280x);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(10); /* Wait after reset */

    /* Read calibration data */
    status = BMP280_Read_Calibration_Data(bmp280x);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Set default configuration */
    bmp280x->config.mode = BMP280_MODE_NORMAL;
    bmp280x->config.osrs_t = BMP280_OS_2X;
    bmp280x->config.osrs_p = BMP280_OS_16X;
    bmp280x->config.filter = BMP280_FILTER_16;
    bmp280x->config.standby_time = BMP280_STANDBY_0_5_MS;

    /* Apply configuration */
    return BMP280_Set_Config(bmp280x, &bmp280x->config);
}
