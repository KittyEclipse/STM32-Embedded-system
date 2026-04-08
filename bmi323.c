/*
 * bmi323.c  —  BMI323 IMU driver over I2C (HAL)
 *
 * Protocol notes (BMI323 datasheet §6.2 I2C):
 *   Write : START | addr+W | reg | data_LSB | data_MSB | STOP
 *   Read  : START | addr+W | reg | RESTART | addr+R | data_LSB | data_MSB | STOP
 *           — NO dummy byte on I2C (dummy byte is SPI-only).
 *   All registers are 16-bit little-endian (LSB at lower address).
 *
 * Bugs fixed vs original SPI driver:
 *   1. Removed SPI dummy-byte logic (not present on I2C).
 *   2. ACC_CONF ODR was 0x0 (0.78 Hz!) → fixed to 0x7 (100 Hz).
 *   3. GYR_CONF ODR was 0x0 (0.78 Hz!) and avg bits were 0xB (reserved)
 *      → fixed to 0x7 ODR (100 Hz), avg 0x0 (no averaging).
 *   4. Struct member hspi/cs_port/cs_pin replaced with hi2c/addr8.
 */

#include "bmi323.h"

/* =========================================================================
   Register map
   ========================================================================= */
#define REG_CHIP_ID     0x00u
#define REG_ERR         0x01u
#define REG_STATUS      0x02u
#define REG_DATA_START  0x03u   /* burst: ACC_X ACC_Y ACC_Z GYR_X GYR_Y GYR_Z */
#define REG_ACC_CONF    0x20u
#define REG_GYR_CONF    0x21u
#define REG_CMD         0x7Eu   /* soft-reset command register               */

/* Expected chip_id (lower byte of REG_CHIP_ID word) */
#define CHIP_ID_VALUE   0x43u

/*
 * ACC_CONF  [15:12]=mode  [11:8]=odr  [6:4]=range  [2:0]=avg_num
 *   mode 0x4 = normal
 *   odr  0x7 = 100 Hz
 *   range 0x2 = ±8 g
 *   avg  0x0 = no averaging (fastest response)
 *
 * Value: 0x4720
 *   bits 15-12 = 0100  (normal)
 *   bits 11- 8 = 0111  (100 Hz)
 *   bits  6- 4 = 010   (±8 g)
 *   bits  2- 0 = 000   (no avg)
 */
#define ACC_CONF_VALUE  0x4720u

/*
 * GYR_CONF  [15:12]=mode  [11:8]=odr  [6:4]=range  [2:0]=avg_num
 *   mode 0x4 = normal
 *   odr  0x7 = 100 Hz
 *   range 0x4 = ±2000 dps
 *   avg  0x0 = no averaging
 *
 * Value: 0x4740
 *   bits 15-12 = 0100  (normal)
 *   bits 11- 8 = 0111  (100 Hz)
 *   bits  6- 4 = 100   (±2000 dps)
 *   bits  2- 0 = 000   (no avg)
 */
#define GYR_CONF_VALUE  0x4740u

/* Sensitivity constants (from BMI323 datasheet Table 5) */
#define ACC_LSB_PER_G_8G      4096.0f   /* ±8 g range   */
#define GYR_LSB_PER_DPS_2K    16.384f   /* ±2000 dps range */

/* I2C timeout */
#define I2C_TIMEOUT_MS        50u

/* =========================================================================
   Low-level I2C register read / write  (16-bit little-endian registers)
   ========================================================================= */

/**
 * Write one 16-bit register.
 * Wire format: reg_addr, LSB, MSB
 */
static HAL_StatusTypeDef reg_write16(BMI323_t *d, uint8_t reg, uint16_t val)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)(val & 0xFFu);          /* LSB */
    buf[2] = (uint8_t)((val >> 8u) & 0xFFu);  /* MSB */
    return HAL_I2C_Master_Transmit(d->hi2c, d->addr8,
                                   buf, 3, I2C_TIMEOUT_MS);
}

/**
 * Read one 16-bit register.
 * Wire format: write reg_addr, then read 2 bytes (LSB, MSB).
 */
static HAL_StatusTypeDef reg_read16(BMI323_t *d, uint8_t reg, uint16_t *out)
{
    uint8_t rx[2] = {0, 0};
    HAL_StatusTypeDef st;

    /* Point device to register */
    st = HAL_I2C_Master_Transmit(d->hi2c, d->addr8,
                                  &reg, 1, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    /* Read 2 bytes */
    st = HAL_I2C_Master_Receive(d->hi2c, d->addr8,
                                 rx, 2, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    *out = ((uint16_t)rx[1] << 8u) | (uint16_t)rx[0];  /* little-endian */
    return HAL_OK;
}

/**
 * Burst-read N consecutive 16-bit registers starting at start_reg.
 * Each register = 2 bytes little-endian, no dummy byte on I2C.
 */
static HAL_StatusTypeDef reg_read_burst(BMI323_t *d, uint8_t start_reg,
                                        int16_t *words, uint8_t word_count)
{
    uint8_t rx[12];  /* 6 words × 2 bytes = 12 bytes max */
    if (word_count > 6u) return HAL_ERROR;

    HAL_StatusTypeDef st;
    uint8_t byte_count = (uint8_t)(word_count * 2u);

    /* Set register pointer */
    st = HAL_I2C_Master_Transmit(d->hi2c, d->addr8,
                                  &start_reg, 1, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    /* Burst read all bytes */
    st = HAL_I2C_Master_Receive(d->hi2c, d->addr8,
                                 rx, byte_count, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    for (uint8_t i = 0; i < word_count; i++) {
        uint16_t raw = ((uint16_t)rx[2u*i + 1u] << 8u) | (uint16_t)rx[2u*i];
        words[i] = (int16_t)raw;
    }
    return HAL_OK;
}

/* =========================================================================
   Public API implementation
   ========================================================================= */

HAL_StatusTypeDef BMI323_Init(BMI323_t *dev,
                              I2C_HandleTypeDef *hi2c,
                              uint8_t addr_7bit)
{
    if (!dev || !hi2c) return HAL_ERROR;

    dev->hi2c  = hi2c;
    dev->addr8 = (uint8_t)(addr_7bit << 1u);  /* HAL expects 8-bit address */

    dev->acc_lsb_per_g   = ACC_LSB_PER_G_8G;
    dev->gyr_lsb_per_dps = GYR_LSB_PER_DPS_2K;

    for (int i = 0; i < 3; i++) {
        dev->acc_bias_g[i]    = 0.0f;
        dev->gyr_bias_dps[i]  = 0.0f;
        dev->vel_mps[i]       = 0.0f;
        dev->pos_m[i]         = 0.0f;
    }
    dev->last_ms = HAL_GetTick();

    /* Give device time to power up */
    HAL_Delay(10);

    /* Verify chip ID */
    uint16_t chip_word = 0;
    if (reg_read16(dev, REG_CHIP_ID, &chip_word) != HAL_OK) return HAL_ERROR;

    uint8_t chip_id = (uint8_t)(chip_word & 0xFFu);
    if (chip_id != CHIP_ID_VALUE) {
        /* Try once more — device may need a moment after power-on */
        HAL_Delay(5);
        if (reg_read16(dev, REG_CHIP_ID, &chip_word) != HAL_OK) return HAL_ERROR;
        chip_id = (uint8_t)(chip_word & 0xFFu);
        if (chip_id != CHIP_ID_VALUE) return HAL_ERROR;
    }

    /* Configure accelerometer: normal mode, 100 Hz, ±8 g, no averaging */
    if (reg_write16(dev, REG_ACC_CONF, ACC_CONF_VALUE) != HAL_OK) return HAL_ERROR;

    /* Configure gyroscope: normal mode, 100 Hz, ±2000 dps, no averaging */
    if (reg_write16(dev, REG_GYR_CONF, GYR_CONF_VALUE) != HAL_OK) return HAL_ERROR;

    /* Wait for sensors to leave suspend and produce valid data */
    HAL_Delay(50);

    return HAL_OK;
}

HAL_StatusTypeDef BMI323_ReadSample(BMI323_t *dev, BMI323_Sample_t *out)
{
    if (!dev || !out) return HAL_ERROR;

    /* Burst-read 6 words: ACC_X, ACC_Y, ACC_Z, GYR_X, GYR_Y, GYR_Z */
    int16_t w[6] = {0};
    if (reg_read_burst(dev, REG_DATA_START, w, 6u) != HAL_OK) return HAL_ERROR;

    out->acc_raw[0] = w[0];
    out->acc_raw[1] = w[1];
    out->acc_raw[2] = w[2];
    out->gyr_raw[0] = w[3];
    out->gyr_raw[1] = w[4];
    out->gyr_raw[2] = w[5];

    for (int i = 0; i < 3; i++) {
        out->acc_g[i]   = ((float)out->acc_raw[i] / dev->acc_lsb_per_g)
                          - dev->acc_bias_g[i];
        out->gyr_dps[i] = ((float)out->gyr_raw[i] / dev->gyr_lsb_per_dps)
                          - dev->gyr_bias_dps[i];
    }

    return HAL_OK;
}

HAL_StatusTypeDef BMI323_CalibrateBias(BMI323_t *dev,
                                       uint16_t  samples,
                                       uint16_t  sample_delay_ms)
{
    if (!dev || samples == 0u) return HAL_ERROR;

    /* Temporarily zero bias so raw values come through unmodified */
    for (int k = 0; k < 3; k++) {
        dev->acc_bias_g[k]   = 0.0f;
        dev->gyr_bias_dps[k] = 0.0f;
    }

    float acc_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyr_sum[3] = {0.0f, 0.0f, 0.0f};
    BMI323_Sample_t s;

    for (uint16_t i = 0; i < samples; i++) {
        if (BMI323_ReadSample(dev, &s) != HAL_OK) return HAL_ERROR;
        for (int k = 0; k < 3; k++) {
            acc_sum[k] += s.acc_g[k];
            gyr_sum[k] += s.gyr_dps[k];
        }
        HAL_Delay(sample_delay_ms);
    }

    float inv = 1.0f / (float)samples;
    for (int k = 0; k < 3; k++) {
        dev->acc_bias_g[k]   = acc_sum[k] * inv;
        dev->gyr_bias_dps[k] = gyr_sum[k] * inv;
    }

    /*
     * NOTE: If the robot is resting on a flat surface with Z pointing up,
     * acc_bias_g[2] will include ~1 g of gravity.  Without a full attitude
     * estimator (Madgwick/Mahony/EKF) we cannot separate gravity from motion
     * in arbitrary orientations.  The Jetson should handle gravity removal.
     */

    return HAL_OK;
}

void BMI323_ResetOdometry(BMI323_t *dev)
{
    if (!dev) return;
    for (int i = 0; i < 3; i++) {
        dev->vel_mps[i] = 0.0f;
        dev->pos_m[i]   = 0.0f;
    }
    dev->last_ms = HAL_GetTick();
}

HAL_StatusTypeDef BMI323_UpdateOdometry(BMI323_t *dev)
{
    if (!dev) return HAL_ERROR;

    BMI323_Sample_t s;
    if (BMI323_ReadSample(dev, &s) != HAL_OK) return HAL_ERROR;

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - dev->last_ms) * 0.001f;  /* ms → s */
    dev->last_ms = now;

    /* Skip if dt is nonsensical (first call, wrap, or stall) */
    if (dt <= 0.0f || dt > 0.2f) return HAL_OK;

    const float G0 = 9.80665f;
    for (int i = 0; i < 3; i++) {
        float acc_mps2     = s.acc_g[i] * G0;
        dev->vel_mps[i]   += acc_mps2 * dt;
        dev->pos_m[i]     += dev->vel_mps[i] * dt;
    }

    return HAL_OK;
}
