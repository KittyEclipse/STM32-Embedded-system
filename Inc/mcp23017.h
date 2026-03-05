#ifndef MCP23017_H
#define MCP23017_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---------------- Public types ---------------- */

typedef enum {
  MCP_PORTA = 0,
  MCP_PORTB = 1
} MCP23017_Port_t;

typedef enum {
  MCP_PIN0 = 0, MCP_PIN1, MCP_PIN2, MCP_PIN3,
  MCP_PIN4,     MCP_PIN5, MCP_PIN6, MCP_PIN7
} MCP23017_Pin_t;

typedef enum {
  MCP_DIR_OUTPUT = 0,
  MCP_DIR_INPUT  = 1
} MCP23017_Dir_t;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t addr_7bit;      // 0x20..0x27
  uint32_t i2c_timeout_ms;

  // cached registers (optional, used for read-modify-write convenience)
  uint8_t iodir[2];       // IODIRA/B
  uint8_t gppu[2];        // GPPUA/B
  uint8_t gpio[2];        // GPIOA/B shadow
  uint8_t ipol[2];        // IPOLA/B
} MCP23017_Handle_t;

/* ---------------- API ---------------- */

HAL_StatusTypeDef MCP23017_Init(MCP23017_Handle_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t addr_7bit);

HAL_StatusTypeDef MCP23017_ResetDefaults(MCP23017_Handle_t *dev);

/* Pin direction */
HAL_StatusTypeDef MCP23017_PinMode(MCP23017_Handle_t *dev,
                                   MCP23017_Port_t port,
                                   MCP23017_Pin_t pin,
                                   MCP23017_Dir_t dir);

/* Pull-up enable (only meaningful for INPUT pins) */
HAL_StatusTypeDef MCP23017_SetPullup(MCP23017_Handle_t *dev,
                                     MCP23017_Port_t port,
                                     MCP23017_Pin_t pin,
                                     uint8_t enable);

/* Input polarity inversion */
HAL_StatusTypeDef MCP23017_SetPolarity(MCP23017_Handle_t *dev,
                                       MCP23017_Port_t port,
                                       MCP23017_Pin_t pin,
                                       uint8_t invert);

/* Read/write a single pin */
HAL_StatusTypeDef MCP23017_WritePin(MCP23017_Handle_t *dev,
                                    MCP23017_Port_t port,
                                    MCP23017_Pin_t pin,
                                    GPIO_PinState state);

HAL_StatusTypeDef MCP23017_ReadPin(MCP23017_Handle_t *dev,
                                   MCP23017_Port_t port,
                                   MCP23017_Pin_t pin,
                                   GPIO_PinState *state);

/* Read/write whole port (8 bits) */
HAL_StatusTypeDef MCP23017_WritePort(MCP23017_Handle_t *dev,
                                     MCP23017_Port_t port,
                                     uint8_t value);

HAL_StatusTypeDef MCP23017_ReadPort(MCP23017_Handle_t *dev,
                                    MCP23017_Port_t port,
                                    uint8_t *value);

/* Low-level register access (optional) */
HAL_StatusTypeDef MCP23017_WriteReg(MCP23017_Handle_t *dev, uint8_t reg, uint8_t value);
HAL_StatusTypeDef MCP23017_ReadReg (MCP23017_Handle_t *dev, uint8_t reg, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* MCP23017_H */
