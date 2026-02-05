#include "mcp23017.h"
#include <string.h>

/* ---------------- MCP23017 register map (BANK=0) ---------------- */
#define MCP_IODIRA   0x00
#define MCP_IODIRB   0x01
#define MCP_IPOLA    0x02
#define MCP_IPOLB    0x03
#define MCP_GPINTENA 0x04
#define MCP_GPINTENB 0x05
#define MCP_DEFVALA  0x06
#define MCP_DEFVALB  0x07
#define MCP_INTCONA  0x08
#define MCP_INTCONB  0x09
#define MCP_IOCON    0x0A  // also 0x0B
#define MCP_GPPUA    0x0C
#define MCP_GPPUB    0x0D
#define MCP_INTFA    0x0E
#define MCP_INTFB    0x0F
#define MCP_INTCAPA  0x10
#define MCP_INTCAPB  0x11
#define MCP_GPIOA    0x12
#define MCP_GPIOB    0x13
#define MCP_OLATA    0x14
#define MCP_OLATB    0x15

static inline uint16_t mcp_dev_addr8(const MCP23017_Handle_t *dev)
{
  return (uint16_t)(dev->addr_7bit << 1); // HAL expects 8-bit address
}

static HAL_StatusTypeDef mcp_write(MCP23017_Handle_t *dev, uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(dev->hi2c, mcp_dev_addr8(dev),
                           reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1, dev->i2c_timeout_ms);
}

static HAL_StatusTypeDef mcp_read(MCP23017_Handle_t *dev, uint8_t reg, uint8_t *val)
{
  return HAL_I2C_Mem_Read(dev->hi2c, mcp_dev_addr8(dev),
                          reg, I2C_MEMADD_SIZE_8BIT,
                          val, 1, dev->i2c_timeout_ms);
}

/* ---------------- Public low-level ---------------- */

HAL_StatusTypeDef MCP23017_WriteReg(MCP23017_Handle_t *dev, uint8_t reg, uint8_t value)
{
  if (!dev || !dev->hi2c) return HAL_ERROR;
  return mcp_write(dev, reg, value);
}

HAL_StatusTypeDef MCP23017_ReadReg(MCP23017_Handle_t *dev, uint8_t reg, uint8_t *value)
{
  if (!dev || !dev->hi2c || !value) return HAL_ERROR;
  return mcp_read(dev, reg, value);
}

/* ---------------- Init / defaults ---------------- */

HAL_StatusTypeDef MCP23017_ResetDefaults(MCP23017_Handle_t *dev)
{
  if (!dev || !dev->hi2c) return HAL_ERROR;

  // IOCON: BANK=0, SEQOP=0 (sequential enabled), HAEN=0 (SPI only), ODR=0, INTPOL=0
  // We write same value to both IOCON addresses in case silicon mirrors are used.
  HAL_StatusTypeDef st = HAL_OK;
  st |= mcp_write(dev, MCP_IOCON, 0x00);
  st |= mcp_write(dev, 0x0B,      0x00);

  // Default: all inputs
  st |= mcp_write(dev, MCP_IODIRA, 0xFF);
  st |= mcp_write(dev, MCP_IODIRB, 0xFF);

  // No pullups, no inversion
  st |= mcp_write(dev, MCP_GPPUA, 0x00);
  st |= mcp_write(dev, MCP_GPPUB, 0x00);
  st |= mcp_write(dev, MCP_IPOLA, 0x00);
  st |= mcp_write(dev, MCP_IPOLB, 0x00);

  // Clear outputs (OLAT)
  st |= mcp_write(dev, MCP_OLATA, 0x00);
  st |= mcp_write(dev, MCP_OLATB, 0x00);

  // Cache
  dev->iodir[MCP_PORTA] = 0xFF;
  dev->iodir[MCP_PORTB] = 0xFF;
  dev->gppu[MCP_PORTA]  = 0x00;
  dev->gppu[MCP_PORTB]  = 0x00;
  dev->ipol[MCP_PORTA]  = 0x00;
  dev->ipol[MCP_PORTB]  = 0x00;
  dev->gpio[MCP_PORTA]  = 0x00;
  dev->gpio[MCP_PORTB]  = 0x00;

  return (st == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MCP23017_Init(MCP23017_Handle_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t addr_7bit)
{
  if (!dev || !hi2c) return HAL_ERROR;

  memset(dev, 0, sizeof(*dev));
  dev->hi2c = hi2c;
  dev->addr_7bit = addr_7bit;
  dev->i2c_timeout_ms = 100;

  // Basic presence check: read IOCON (or IODIRA)
  uint8_t tmp = 0;
  if (mcp_read(dev, MCP_IOCON, &tmp) != HAL_OK)
  {
    // Some boards might not respond on IOCON if miswired;
    // try IODIRA as a secondary check.
    if (mcp_read(dev, MCP_IODIRA, &tmp) != HAL_OK)
      return HAL_ERROR;
  }

  return MCP23017_ResetDefaults(dev);
}

/* ---------------- Helpers ---------------- */

static inline uint8_t reg_iodir(MCP23017_Port_t port) { return (port == MCP_PORTA) ? MCP_IODIRA : MCP_IODIRB; }
static inline uint8_t reg_gppu (MCP23017_Port_t port) { return (port == MCP_PORTA) ? MCP_GPPUA  : MCP_GPPUB;  }
static inline uint8_t reg_ipol (MCP23017_Port_t port) { return (port == MCP_PORTA) ? MCP_IPOLA  : MCP_IPOLB;  }
static inline uint8_t reg_gpio (MCP23017_Port_t port) { return (port == MCP_PORTA) ? MCP_GPIOA  : MCP_GPIOB;  }
static inline uint8_t reg_olat (MCP23017_Port_t port) { return (port == MCP_PORTA) ? MCP_OLATA  : MCP_OLATB;  }

/* ---------------- Pin configuration ---------------- */

HAL_StatusTypeDef MCP23017_PinMode(MCP23017_Handle_t *dev,
                                   MCP23017_Port_t port,
                                   MCP23017_Pin_t pin,
                                   MCP23017_Dir_t dir)
{
  if (!dev || pin > MCP_PIN7) return HAL_ERROR;

  uint8_t mask = (uint8_t)(1u << pin);

  if (dir == MCP_DIR_INPUT) dev->iodir[port] |=  mask;
  else                     dev->iodir[port] &= (uint8_t)~mask;

  return mcp_write(dev, reg_iodir(port), dev->iodir[port]);
}

HAL_StatusTypeDef MCP23017_SetPullup(MCP23017_Handle_t *dev,
                                     MCP23017_Port_t port,
                                     MCP23017_Pin_t pin,
                                     uint8_t enable)
{
  if (!dev || pin > MCP_PIN7) return HAL_ERROR;

  uint8_t mask = (uint8_t)(1u << pin);
  if (enable) dev->gppu[port] |=  mask;
  else        dev->gppu[port] &= (uint8_t)~mask;

  return mcp_write(dev, reg_gppu(port), dev->gppu[port]);
}

HAL_StatusTypeDef MCP23017_SetPolarity(MCP23017_Handle_t *dev,
                                       MCP23017_Port_t port,
                                       MCP23017_Pin_t pin,
                                       uint8_t invert)
{
  if (!dev || pin > MCP_PIN7) return HAL_ERROR;

  uint8_t mask = (uint8_t)(1u << pin);
  if (invert) dev->ipol[port] |=  mask;
  else        dev->ipol[port] &= (uint8_t)~mask;

  return mcp_write(dev, reg_ipol(port), dev->ipol[port]);
}

/* ---------------- Read/write ---------------- */

HAL_StatusTypeDef MCP23017_WritePort(MCP23017_Handle_t *dev,
                                     MCP23017_Port_t port,
                                     uint8_t value)
{
  if (!dev) return HAL_ERROR;

  dev->gpio[port] = value;
  // Writing OLAT is generally preferred for outputs
  return mcp_write(dev, reg_olat(port), value);
}

HAL_StatusTypeDef MCP23017_ReadPort(MCP23017_Handle_t *dev,
                                    MCP23017_Port_t port,
                                    uint8_t *value)
{
  if (!dev || !value) return HAL_ERROR;

  HAL_StatusTypeDef st = mcp_read(dev, reg_gpio(port), value);
  if (st == HAL_OK) dev->gpio[port] = *value;
  return st;
}

HAL_StatusTypeDef MCP23017_WritePin(MCP23017_Handle_t *dev,
                                    MCP23017_Port_t port,
                                    MCP23017_Pin_t pin,
                                    GPIO_PinState state)
{
  if (!dev || pin > MCP_PIN7) return HAL_ERROR;

  uint8_t mask = (uint8_t)(1u << pin);

  if (state == GPIO_PIN_SET) dev->gpio[port] |=  mask;
  else                       dev->gpio[port] &= (uint8_t)~mask;

  // Write OLAT to affect outputs
  return mcp_write(dev, reg_olat(port), dev->gpio[port]);
}

HAL_StatusTypeDef MCP23017_ReadPin(MCP23017_Handle_t *dev,
                                   MCP23017_Port_t port,
                                   MCP23017_Pin_t pin,
                                   GPIO_PinState *state)
{
  if (!dev || !state || pin > MCP_PIN7) return HAL_ERROR;

  uint8_t v = 0;
  HAL_StatusTypeDef st = mcp_read(dev, reg_gpio(port), &v);
  if (st != HAL_OK) return st;

  dev->gpio[port] = v;
  *state = (v & (1u << pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  return HAL_OK;
}
