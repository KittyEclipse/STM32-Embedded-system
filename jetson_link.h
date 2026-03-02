/* jetson_link.h */
#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "bmi323.h"

// ---------- Protocol ----------
#define JL_SOF_1 0xAA
#define JL_SOF_2 0x55

// Message types STM32 -> Jetson
typedef enum {
  JL_MSG_HELLO      = 0x01,
  JL_MSG_IMU        = 0x10,
  JL_MSG_ODOM       = 0x11,
  JL_MSG_TOUCH      = 0x12,
  JL_MSG_SERVO      = 0x13,
  JL_MSG_TEXT       = 0x20,
} jl_msg_t;

// Message types Jetson -> STM32
typedef enum {
  JL_CMD_PING           = 0x80,
  JL_CMD_SET_STREAM     = 0x81, // payload: uint8_t mask, uint16_t period_ms
  JL_CMD_SET_SERVO_US   = 0x82, // payload: uint8_t ch, uint16_t pulse_us
  JL_CMD_SET_GAIT       = 0x83, // payload: float step_len, ride_h, lift_h, step_ms
  JL_CMD_REQUEST_ONESHOT= 0x84, // payload: uint8_t what
  JL_CMD_LOW_POWER      = 0x85, // payload: uint8_t level (0=warn,1=critical)
} jl_cmd_t;

typedef struct {
  UART_HandleTypeDef *huart;

  // streaming config
  uint8_t stream_mask;      // bit0 IMU, bit1 ODOM, bit2 TOUCH, bit3 SERVO
  uint16_t stream_period_ms;
  uint32_t last_stream_ms;

  // RX parser
  uint8_t rx_byte;
  uint8_t rx_buf[256];
  uint16_t rx_len;
  uint16_t rx_need;
  enum { RX_SOF1, RX_SOF2, RX_LEN, RX_TYPE, RX_PAYLOAD, RX_CRC1, RX_CRC2 } rx_state;
  uint8_t rx_type;
  uint16_t rx_crc;

} JetsonLink_t;

// Init + pump
void JL_Init(JetsonLink_t *jl, UART_HandleTypeDef *huart);
void JL_StartRx(JetsonLink_t *jl);          // call once after init
void JL_Poll(JetsonLink_t *jl);             // call in while(1)

// TX helpers
HAL_StatusTypeDef JL_SendHello(JetsonLink_t *jl);
HAL_StatusTypeDef JL_SendIMU(JetsonLink_t *jl, const BMI323_Sample_t *s);
HAL_StatusTypeDef JL_SendOdom(JetsonLink_t *jl, const float pos_m[3], const float vel_mps[3]);
HAL_StatusTypeDef JL_SendTouch(JetsonLink_t *jl, uint16_t touch_bits);
HAL_StatusTypeDef JL_SendServo(JetsonLink_t *jl, uint8_t ch, uint16_t us);
HAL_StatusTypeDef JL_SendText(JetsonLink_t *jl, const char *s);

// Must be called from stm32f4xx_it.c hook
void JL_OnUartRxCplt(JetsonLink_t *jl);

// Called when a full command is decoded
void JL_HandleCommand(JetsonLink_t *jl, uint8_t type, const uint8_t *payload, uint16_t len);

// Expose gait params if you want Jetson to tune them
typedef struct {
  float step_len;
  float ride_h;
  float lift_h;
  float step_ms;
} JL_GaitParams_t;

extern volatile JL_GaitParams_t g_jl_gait;

// Expose low power + servo request flags for main.c (do actions in main loop)
extern volatile uint8_t  g_jl_low_power_level;
extern volatile uint8_t  g_jl_low_power_event;

extern volatile uint8_t  g_jl_servo_event;
extern volatile uint8_t  g_jl_servo_ch;
extern volatile uint16_t g_jl_servo_us;
