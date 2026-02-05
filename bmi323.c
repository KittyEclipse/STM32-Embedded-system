/*
 * bmi323.c
 *
 *  Created on: Feb 4, 2026
 *      Author: reinl
 */

#include "bmi323.h"

// Register addresses (8-bit addresses, 16-bit data words)
#define BMI323_REG_CHIP_ID     0x00
#define BMI323_REG_STATUS      0x01
#define BMI323_REG_SENS_STATUS 0x02
#define BMI323_REG_DATA_START  0x03   // burst 6 words: acc xyz + gyr xyz
#define BMI323_REG_ACC_CONF    0x20
#define BMI323_REG_GYR_CONF    0x21

// CHIP_ID expected value (LSB of reg 0x00 is chip_id) shown in datasheet.
#define BMI323_CHIP_ID_VALUE   0x43   // chip_id field reset value shown in register description :contentReference[oaicite:2]{index=2}

// Config examples from datasheet figures:
// Normal mode ACC: 0x4027 (8g, 50Hz), GYR: 0x404B (2kdps, 800Hz) :contentReference[oaicite:3]{index=3}
#define BMI323_ACC_CONF_NORMAL  0x4027
#define BMI323_GYR_CONF_NORMAL  0x404B

// Sensitivity values from datasheet tables:
// accel ±8g: 4096 LSB/g; gyro ±2000 dps: 16.384 LSB/(deg/s) :contentReference[oaicite:4]{index=4}
#define BMI323_ACC_LSB_PER_G_8G     4096.0f
#define BMI323_GYR_LSB_PER_DPS_2K   16.384f

// SPI protocol:
// - First byte: bit7 = RnW, bits[6:0] = address
// - On read: first returned byte is dummy, then data bytes (LSB then MSB for each 16-bit word) :contentReference[oaicite:5]{index=5}
static inline void cs_low(BMI323_t *d)  { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(BMI323_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef bmi323_read_bytes(BMI323_t *d, uint8_t addr, uint8_t *rx, uint16_t rx_len)
{
    // Send command byte, then clock out rx_len bytes.
    uint8_t cmd = 0x80u | (addr & 0x7Fu);

    cs_low(d);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, &cmd, 1, HAL_MAX_DELAY);
    if (st != HAL_OK) { cs_high(d); return st; }

    // clock out bytes (send zeros, read rx)
    for (uint16_t i = 0; i < rx_len; i++) {
        uint8_t tx = 0x00;
        st = HAL_SPI_TransmitReceive(d->hspi, &tx, &rx[i], 1, HAL_MAX_DELAY);
        if (st != HAL_OK) { cs_high(d); return st; }
    }
    cs_high(d);
    return HAL_OK;
}

static HAL_StatusTypeDef bmi323_write_bytes(BMI323_t *d, uint8_t addr, const uint8_t *tx, uint16_t tx_len)
{
    uint8_t cmd = (addr & 0x7Fu); // RnW=0

    cs_low(d);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, &cmd, 1, HAL_MAX_DELAY);
    if (st != HAL_OK) { cs_high(d); return st; }

    st = HAL_SPI_Transmit(d->hspi, (uint8_t*)tx, tx_len, HAL_MAX_DELAY);
    cs_high(d);
    return st;
}

static HAL_StatusTypeDef bmi323_read_u16(BMI323_t *d, uint8_t addr, uint16_t *out)
{
    // Need dummy byte + 2 data bytes (LSB then MSB) :contentReference[oaicite:6]{index=6}
    uint8_t rx[3] = {0};
    HAL_StatusTypeDef st = bmi323_read_bytes(d, addr, rx, 3);
    if (st != HAL_OK) return st;

    uint8_t lsb = rx[1];
    uint8_t msb = rx[2];
    *out = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return HAL_OK;
}

static HAL_StatusTypeDef bmi323_write_u16(BMI323_t *d, uint8_t addr, uint16_t val)
{
    // Device expects 2 bytes per register file; if only 1 byte sent, it discards it. :contentReference[oaicite:7]{index=7}
    uint8_t tx[2];
    tx[0] = (uint8_t)(val & 0xFF);       // LSB first
    tx[1] = (uint8_t)((val >> 8) & 0xFF);
    return bmi323_write_bytes(d, addr, tx, 2);
}

static HAL_StatusTypeDef bmi323_read_words(BMI323_t *d, uint8_t start_addr, uint16_t *words, uint16_t word_count)
{
    // For read: dummy byte then 2 bytes per word (LSB then MSB) :contentReference[oaicite:8]{index=8}
    uint16_t rx_len = 1 + (2 * word_count);
    uint8_t rx[1 + 2*12]; // enough for up to 12 words; we use 6
    if (rx_len > sizeof(rx)) return HAL_ERROR;

    HAL_StatusTypeDef st = bmi323_read_bytes(d, start_addr, rx, rx_len);
    if (st != HAL_OK) return st;

    for (uint16_t i = 0; i < word_count; i++) {
        uint8_t lsb = rx[1 + (2*i) + 0];
        uint8_t msb = rx[1 + (2*i) + 1];
        words[i] = ((uint16_t)msb << 8) | (uint16_t)lsb;
    }
    return HAL_OK;
}

HAL_StatusTypeDef BMI323_Init(BMI323_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (!dev || !hspi || !cs_port) return HAL_ERROR;

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    dev->acc_lsb_per_g = BMI323_ACC_LSB_PER_G_8G;
    dev->gyr_lsb_per_dps = BMI323_GYR_LSB_PER_DPS_2K;

    for (int i=0;i<3;i++){
        dev->acc_bias_g[i]=0; dev->gyr_bias_dps[i]=0;
        dev->vel_mps[i]=0; dev->pos_m[i]=0;
    }
    dev->last_ms = HAL_GetTick();

    cs_high(dev);
    HAL_Delay(5);

    // IMPORTANT: device defaults to I3C/I2C; one initial dummy read switches it to SPI, then read again. :contentReference[oaicite:9]{index=9}
    uint16_t chip;
    (void)bmi323_read_u16(dev, BMI323_REG_CHIP_ID, &chip); // dummy read
    HAL_Delay(1);
    if (bmi323_read_u16(dev, BMI323_REG_CHIP_ID, &chip) != HAL_OK) return HAL_ERROR;

    uint8_t chip_id = (uint8_t)(chip & 0xFF);
    if (chip_id != BMI323_CHIP_ID_VALUE) {
        return HAL_ERROR;
    }

    // Configure accel/gyro (values from datasheet example flow) :contentReference[oaicite:10]{index=10}
    if (bmi323_write_u16(dev, BMI323_REG_ACC_CONF, BMI323_ACC_CONF_NORMAL) != HAL_OK) return HAL_ERROR;
    if (bmi323_write_u16(dev, BMI323_REG_GYR_CONF, BMI323_GYR_CONF_NORMAL) != HAL_OK) return HAL_ERROR;

    HAL_Delay(50);
    return HAL_OK;
}

HAL_StatusTypeDef BMI323_ReadSample(BMI323_t *dev, BMI323_Sample_t *out)
{
    if (!dev || !out) return HAL_ERROR;

    uint16_t w[6];
    // Datasheet shows burst read of 6 words from 0x03 for accel+gyro sample set :contentReference[oaicite:11]{index=11}
    if (bmi323_read_words(dev, BMI323_REG_DATA_START, w, 6) != HAL_OK) return HAL_ERROR;

    // Interpret as signed 16-bit
    out->acc_raw[0] = (int16_t)w[0];
    out->acc_raw[1] = (int16_t)w[1];
    out->acc_raw[2] = (int16_t)w[2];
    out->gyr_raw[0] = (int16_t)w[3];
    out->gyr_raw[1] = (int16_t)w[4];
    out->gyr_raw[2] = (int16_t)w[5];

    // Convert using datasheet sensitivities (example: ±8g and ±2000 dps) :contentReference[oaicite:12]{index=12}
    for (int i=0;i<3;i++){
        out->acc_g[i] = ((float)out->acc_raw[i] / dev->acc_lsb_per_g) - dev->acc_bias_g[i];
        out->gyr_dps[i] = ((float)out->gyr_raw[i] / dev->gyr_lsb_per_dps) - dev->gyr_bias_dps[i];
    }

    return HAL_OK;
}

HAL_StatusTypeDef BMI323_CalibrateBias(BMI323_t *dev, uint16_t samples, uint16_t sample_delay_ms)
{
    if (!dev || samples == 0) return HAL_ERROR;

    float acc_sum[3] = {0}, gyr_sum[3] = {0};
    BMI323_Sample_t s;

    for (uint16_t i=0; i<samples; i++){
        if (BMI323_ReadSample(dev, &s) != HAL_OK) return HAL_ERROR;

        for (int k=0;k<3;k++){
            // Temporarily ignore existing bias while calibrating
            float acc_g = (float)s.acc_raw[k] / dev->acc_lsb_per_g;
            float gyr_dps = (float)s.gyr_raw[k] / dev->gyr_lsb_per_dps;
            acc_sum[k] += acc_g;
            gyr_sum[k] += gyr_dps;
        }
        HAL_Delay(sample_delay_ms);
    }

    for (int k=0;k<3;k++){
        dev->acc_bias_g[k] = acc_sum[k] / (float)samples;
        dev->gyr_bias_dps[k] = gyr_sum[k] / (float)samples;
    }

    // If the cat starts sitting still and "Z" axis points up,
    // you'd often want to subtract gravity here. Without attitude estimation,
    // we keep it simple and DO NOT try to remove gravity robustly.

    return HAL_OK;
}

void BMI323_ResetOdometry(BMI323_t *dev)
{
    if (!dev) return;
    for (int i=0;i<3;i++){
        dev->vel_mps[i] = 0;
        dev->pos_m[i] = 0;
    }
    dev->last_ms = HAL_GetTick();
}

HAL_StatusTypeDef BMI323_UpdateOdometry(BMI323_t *dev)
{
    if (!dev) return HAL_ERROR;

    BMI323_Sample_t s;
    if (BMI323_ReadSample(dev, &s) != HAL_OK) return HAL_ERROR;

    uint32_t now = HAL_GetTick();
    float dt = (now - dev->last_ms) / 1000.0f;
    dev->last_ms = now;
    if (dt <= 0.0f || dt > 0.2f) return HAL_OK; // skip weird gaps

    // Convert g -> m/s^2
    const float g0 = 9.80665f;
    float acc_mps2[3] = {
        s.acc_g[0] * g0,
        s.acc_g[1] * g0,
        s.acc_g[2] * g0,
    };

    // Basic integration (drifts!)
    for (int i=0;i<3;i++){
        dev->vel_mps[i] += acc_mps2[i] * dt;
        dev->pos_m[i] += dev->vel_mps[i] * dt;
    }

    return HAL_OK;
}



