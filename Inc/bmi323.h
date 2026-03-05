#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    // scale factors
    float acc_lsb_per_g;     // e.g. 4096 for ±8g
    float gyr_lsb_per_dps;   // e.g. 16.384 for ±2000 dps

    // bias (estimated at startup)
    float acc_bias_g[3];
    float gyr_bias_dps[3];

    // integration state
    float vel_mps[3];
    float pos_m[3];
    uint32_t last_ms;
} BMI323_t;

typedef struct {
    int16_t acc_raw[3];
    int16_t gyr_raw[3];
    float acc_g[3];
    float gyr_dps[3];
} BMI323_Sample_t;

// --- Public API ---
HAL_StatusTypeDef BMI323_Init(BMI323_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin);

HAL_StatusTypeDef BMI323_ReadSample(BMI323_t *dev, BMI323_Sample_t *out);

HAL_StatusTypeDef BMI323_CalibrateBias(BMI323_t *dev, uint16_t samples, uint16_t sample_delay_ms);

void BMI323_ResetOdometry(BMI323_t *dev);

// Call this periodically (e.g., every loop). It reads the sensor and updates pos/vel.
HAL_StatusTypeDef BMI323_UpdateOdometry(BMI323_t *dev);

// Optional helper to get current position pointer
static inline const float* BMI323_GetPositionM(const BMI323_t *dev) { return dev->pos_m; }
