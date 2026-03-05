#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32f4xx_hal.h"

/* Default I2C address (A0-A5 all grounded = 0x40) */
#define PCA9685_DEFAULT_ADDRESS  0x40

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address; /* 7-bit address (0x40 typical) */
} PCA9685_Handle_t;

/* Initialization: sets handle hi2c, address (accepts 7-bit or 8-bit-shifted addr),
   and sets the desired PWM frequency */
HAL_StatusTypeDef PCA9685_Init(PCA9685_Handle_t *hpca,
                               I2C_HandleTypeDef *hi2c,
                               uint8_t address,
                               float freq);

/* Frequency */
HAL_StatusTypeDef PCA9685_SetPWMFreq(PCA9685_Handle_t *hpca, float freq);

/* Raw PWM */
HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_Handle_t *hpca,
                                 uint8_t channel,
                                 uint16_t on,
                                 uint16_t off);

/* Servo helpers */
HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_Handle_t *hpca,
                                        uint8_t channel,
                                        float angle);

/* Set servo pulse length in microseconds (e.g., 500..2500) */
HAL_StatusTypeDef PCA9685_SetServoPulseUs(PCA9685_Handle_t *hpca,
                                          uint8_t channel,
                                          uint16_t pulse_us);

/* FULL OFF helper (fixes your previous linker error) */
HAL_StatusTypeDef PCA9685_SetChannelFullOff(PCA9685_Handle_t *hpca,
                                            uint8_t channel,
                                            uint8_t full_off);

#endif /* __PCA9685_H */
