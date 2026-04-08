#pragma once
/*
 * bmi323.h  —  BMI323 IMU driver over I2C (HAL)
 *
 * I2C address: 0x68 if SDO pin is LOW  (default)
 *              0x69 if SDO pin is HIGH
 *
 * Configured for:
 *   Accel : ±8 g,      100 Hz,  Normal mode
 *   Gyro  : ±2000 dps, 100 Hz,  Normal mode
 */

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* -------------------------------------------------------------------------
   Device struct
   ------------------------------------------------------------------------- */
typedef struct {
    I2C_HandleTypeDef *hi2c;   /* pointer to the HAL I2C handle (I2C1) */
    uint8_t            addr8;  /* 8-bit I2C address = 7-bit << 1        */

    /* Scale factors (set by Init, override if you change ACC/GYR conf)  */
    float acc_lsb_per_g;       /* 4096.0 for ±8 g                       */
    float gyr_lsb_per_dps;     /* 16.384 for ±2000 dps                  */

    /* Bias estimates (filled by CalibrateBias)                          */
    float acc_bias_g[3];
    float gyr_bias_dps[3];

    /* Dead-reckoning integration state                                  */
    float    vel_mps[3];
    float    pos_m[3];
    uint32_t last_ms;
} BMI323_t;

/* -------------------------------------------------------------------------
   Sample struct  (raw counts + converted SI values)
   ------------------------------------------------------------------------- */
typedef struct {
    int16_t acc_raw[3];   /* raw 16-bit accel counts  [X, Y, Z] */
    int16_t gyr_raw[3];   /* raw 16-bit gyro  counts  [X, Y, Z] */
    float   acc_g[3];     /* acceleration in g                   */
    float   gyr_dps[3];   /* angular rate   in deg/s             */
} BMI323_Sample_t;

/* -------------------------------------------------------------------------
   Public API
   ------------------------------------------------------------------------- */

/**
 * @brief  Initialise the BMI323 over I2C.
 * @param  dev      Pointer to BMI323_t struct (caller provides storage).
 * @param  hi2c     HAL I2C handle (e.g. &hi2c1).
 * @param  addr_7bit 7-bit I2C address: 0x68 (SDO low) or 0x69 (SDO high).
 * @retval HAL_OK on success, HAL_ERROR if chip not found or config failed.
 */
HAL_StatusTypeDef BMI323_Init(BMI323_t *dev,
                              I2C_HandleTypeDef *hi2c,
                              uint8_t addr_7bit);

/**
 * @brief  Read one accel+gyro sample.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef BMI323_ReadSample(BMI323_t *dev, BMI323_Sample_t *out);

/**
 * @brief  Estimate static bias by averaging N samples while the robot is still.
 *         Call once after Init, before moving.
 * @param  samples          Number of samples to average (e.g. 200).
 * @param  sample_delay_ms  Delay between samples (e.g. 5 ms → 200 samples ≈ 1 s).
 */
HAL_StatusTypeDef BMI323_CalibrateBias(BMI323_t *dev,
                                       uint16_t  samples,
                                       uint16_t  sample_delay_ms);

/** @brief Reset velocity and position integrators to zero. */
void BMI323_ResetOdometry(BMI323_t *dev);

/**
 * @brief  Read sensor and integrate accel → vel → pos.
 *         Call every control loop tick.
 * @note   Simple single-integration; will drift without attitude correction.
 */
HAL_StatusTypeDef BMI323_UpdateOdometry(BMI323_t *dev);

/** @brief Convenience: return pointer to current pos_m[3] array. */
static inline const float *BMI323_GetPositionM(const BMI323_t *dev)
{
    return dev->pos_m;
}
