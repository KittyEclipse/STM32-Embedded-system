#include "pca9685.h"
#include <math.h>

/* PCA9685 Registers */
#define PCA9685_MODE1        0x00
#define PCA9685_MODE2        0x01
#define PCA9685_PRESCALE     0xFE
#define LED0_ON_L            0x06

#define MODE1_SLEEP          0x10
#define MODE1_AI             0x20
#define MODE2_OUTDRV         0x04

/* Write 8-bit register */
static HAL_StatusTypeDef PCA9685_WriteReg(PCA9685_Handle_t *hpca, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hpca->hi2c,
                             hpca->address << 1,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1,
                             HAL_MAX_DELAY);
}

/* Read 8-bit register */
static HAL_StatusTypeDef PCA9685_ReadReg(PCA9685_Handle_t *hpca, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(hpca->hi2c,
                            hpca->address << 1,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            1,
                            HAL_MAX_DELAY);
}

/* Set PWM frequency */
HAL_StatusTypeDef PCA9685_SetPWMFreq(PCA9685_Handle_t *hpca, float freq)
{
    uint8_t prescale;
    float prescaleval = 25000000.0f; // 25MHz oscillator
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;

    prescale = (uint8_t)(floorf(prescaleval + 0.5f));

    uint8_t oldmode;
    if (PCA9685_ReadReg(hpca, PCA9685_MODE1, &oldmode) != HAL_OK) return HAL_ERROR;

    uint8_t sleep = (oldmode & 0x7F) | MODE1_SLEEP;
    if (PCA9685_WriteReg(hpca, PCA9685_MODE1, sleep) != HAL_OK) return HAL_ERROR;
    if (PCA9685_WriteReg(hpca, PCA9685_PRESCALE, prescale) != HAL_OK) return HAL_ERROR;
    if (PCA9685_WriteReg(hpca, PCA9685_MODE1, oldmode) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);
    if (PCA9685_WriteReg(hpca, PCA9685_MODE1, oldmode | MODE1_AI) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/* Set PWM value (on/off counts 0..4095) */
HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_Handle_t *hpca,
                                 uint8_t channel,
                                 uint16_t on,
                                 uint16_t off)
{
    uint8_t data[4];
    uint8_t reg = LED0_ON_L + 4 * channel;

    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0x0F;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0x0F;

    return HAL_I2C_Mem_Write(hpca->hi2c,
                             hpca->address << 1,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             4,
                             HAL_MAX_DELAY);
}

/* Convert angle (0-180) to PWM pulse (for servo) */
HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_Handle_t *hpca,
                                        uint8_t channel,
                                        float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    float pulse_min = 102.0f;   // ~0.5ms @ 50Hz (approx)
    float pulse_max = 512.0f;   // ~2.5ms @ 50Hz (approx)

    uint16_t pulse = (uint16_t)(pulse_min + ((angle / 180.0f) * (pulse_max - pulse_min)));

    return PCA9685_SetPWM(hpca, channel, 0, pulse);
}

/* Set servo pulse in microseconds (e.g. 500..2500). Assumes 50Hz (20ms) period */
HAL_StatusTypeDef PCA9685_SetServoPulseUs(PCA9685_Handle_t *hpca,
                                          uint8_t channel,
                                          uint16_t pulse_us)
{
    if (pulse_us < 0) pulse_us = 0;
    /* Using 50Hz standard period = 20,000 us */
    const uint32_t period_us = 20000U;
    if (pulse_us > period_us) pulse_us = period_us;

    /* counts = pulse_us / period_us * 4096 */
    uint32_t counts = ((uint32_t)pulse_us * 4096U + (period_us/2)) / period_us;
    if (counts > 4095U) counts = 4095U;

    return PCA9685_SetPWM(hpca, channel, 0, (uint16_t)counts);
}

/* Initialize PCA9685 with given I2C handle, address (7-bit or 8-bit), and frequency */
HAL_StatusTypeDef PCA9685_Init(PCA9685_Handle_t *hpca, I2C_HandleTypeDef *hi2c, uint8_t address, float freq)
{
    /* Normalize address: accept either 7-bit or 8-bit-shifted */
    hpca->hi2c = hi2c;
    if (address > 0x7F) hpca->address = address >> 1;
    else hpca->address = address;

    HAL_StatusTypeDef status;
    uint8_t mode1;

    if (PCA9685_ReadReg(hpca, PCA9685_MODE1, &mode1) != HAL_OK) return HAL_ERROR;

    /* Enter sleep to set prescaler */
    if (PCA9685_WriteReg(hpca, PCA9685_MODE1, (mode1 | MODE1_SLEEP)) != HAL_OK) return HAL_ERROR;

    /* Set PWM frequency (fallback to 50Hz if freq invalid) */
    if (freq <= 0.0f) freq = 50.0f;
    if (PCA9685_SetPWMFreq(hpca, freq) != HAL_OK) return HAL_ERROR;

    /* Wake up + enable auto increment */
    if (PCA9685_WriteReg(hpca, PCA9685_MODE1, MODE1_AI) != HAL_OK) return HAL_ERROR;

    /* Totem pole output driver */
    if (PCA9685_WriteReg(hpca, PCA9685_MODE2, MODE2_OUTDRV) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/* FULL OFF helper */
HAL_StatusTypeDef PCA9685_SetChannelFullOff(PCA9685_Handle_t *hpca,
                                            uint8_t channel,
                                            uint8_t full_off)
{
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t off_l = 0;
    uint8_t off_h = 0;

    if (full_off)
    {
        off_h = 0x10;  // FULL OFF bit (bit 4)
    }

    uint8_t data[2] = {off_l, off_h};

    return HAL_I2C_Mem_Write(hpca->hi2c,
                             hpca->address << 1,
                             reg + 2,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             2,
                             HAL_MAX_DELAY);
}
