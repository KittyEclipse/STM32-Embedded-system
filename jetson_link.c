/* jetson_link.c */
#include "jetson_link.h"
#include <string.h>

// ---- CRC16-CCITT (0x1021), init 0xFFFF ----
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

volatile JL_GaitParams_t g_jl_gait = {
  .step_len = 40.0f,
  .ride_h   = 85.0f,
  .lift_h   = 20.0f,
  .step_ms  = 1000.0f
};

volatile uint8_t  g_jl_low_power_level = 0;
volatile uint8_t  g_jl_low_power_event = 0;

volatile uint8_t  g_jl_servo_event = 0;
volatile uint8_t  g_jl_servo_ch = 0;
volatile uint16_t g_jl_servo_us = 1500;

static HAL_StatusTypeDef jl_send_frame(JetsonLink_t *jl, uint8_t type, const void *payload, uint8_t len)
{
  // frame: SOF1 SOF2 LEN TYPE PAYLOAD CRC_L CRC_H
  uint8_t buf[2 + 1 + 1 + 255 + 2];
  uint16_t idx = 0;
  buf[idx++] = JL_SOF_1;
  buf[idx++] = JL_SOF_2;
  buf[idx++] = len;
  buf[idx++] = type;
  if (len && payload) {
    memcpy(&buf[idx], payload, len);
    idx += len;
  }
  uint16_t crc = crc16_ccitt(&buf[2], (uint16_t)(1 + 1 + len)); // LEN+TYPE+PAYLOAD
  buf[idx++] = (uint8_t)(crc & 0xFF);
  buf[idx++] = (uint8_t)(crc >> 8);

  return HAL_UART_Transmit(jl->huart, buf, idx, HAL_MAX_DELAY);
}

void JL_Init(JetsonLink_t *jl, UART_HandleTypeDef *huart)
{
  memset(jl, 0, sizeof(*jl));
  jl->huart = huart;
  jl->stream_mask = 0;          // default off
  jl->stream_period_ms = 50;    // default 20 Hz
  jl->last_stream_ms = HAL_GetTick();

  jl->rx_state = RX_SOF1;
}

void JL_StartRx(JetsonLink_t *jl)
{
  // Receive 1 byte interrupt-driven
  HAL_UART_Receive_IT(jl->huart, &jl->rx_byte, 1);
}

void JL_OnUartRxCplt(JetsonLink_t *jl)
{
  uint8_t b = jl->rx_byte;

  switch (jl->rx_state) {
    case RX_SOF1:
      if (b == JL_SOF_1) jl->rx_state = RX_SOF2;
      break;

    case RX_SOF2:
      if (b == JL_SOF_2) jl->rx_state = RX_LEN;
      else jl->rx_state = RX_SOF1;
      break;

    case RX_LEN:
      jl->rx_len = b;
      if (jl->rx_len > 250) { jl->rx_state = RX_SOF1; break; }
      jl->rx_state = RX_TYPE;
      break;

    case RX_TYPE:
      jl->rx_type = b;
      jl->rx_need = jl->rx_len;
      jl->rx_crc = 0;
      if (jl->rx_need == 0) jl->rx_state = RX_CRC1;
      else {
        jl->rx_state = RX_PAYLOAD;
      }
      break;

    case RX_PAYLOAD: {
      uint16_t got = (uint16_t)(jl->rx_len - jl->rx_need);
      jl->rx_buf[got] = b;
      jl->rx_need--;
      if (jl->rx_need == 0) jl->rx_state = RX_CRC1;
    } break;

    case RX_CRC1:
      jl->rx_crc = b;
      jl->rx_state = RX_CRC2;
      break;

    case RX_CRC2: {
      jl->rx_crc |= ((uint16_t)b << 8);

      // verify CRC over LEN+TYPE+PAYLOAD
      uint8_t hdr[2]; // LEN, TYPE
      hdr[0] = (uint8_t)jl->rx_len;
      hdr[1] = (uint8_t)jl->rx_type;

      uint16_t crc;
      uint8_t tmp[2 + 250];
      tmp[0] = hdr[0];
      tmp[1] = hdr[1];
      if (jl->rx_len) {
        memcpy(&tmp[2], jl->rx_buf, jl->rx_len);
      }
      crc = crc16_ccitt(tmp, (uint16_t)(2 + jl->rx_len));

      if (crc == jl->rx_crc) {
        JL_HandleCommand(jl, jl->rx_type, jl->rx_buf, jl->rx_len);
      }

      jl->rx_state = RX_SOF1;
    } break;
  }

  // re-arm interrupt
  HAL_UART_Receive_IT(jl->huart, &jl->rx_byte, 1);
}

void JL_Poll(JetsonLink_t *jl)
{
  // streaming scheduler (send from your main loop)
  if (jl->stream_mask == 0) return;

  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - jl->last_stream_ms) < jl->stream_period_ms) return;
  jl->last_stream_ms = now;

  // Actual sending is done from main.c where you have IMU/touch/servo state.
  // (So JL_Poll can be empty or you can add callbacks.)
}

// ---------------- TX messages ----------------

HAL_StatusTypeDef JL_SendHello(JetsonLink_t *jl)
{
  struct __attribute__((packed)) {
    uint32_t ms;
    uint8_t  version;
  } p = { HAL_GetTick(), 1 };

  return jl_send_frame(jl, JL_MSG_HELLO, &p, sizeof(p));
}

HAL_StatusTypeDef JL_SendIMU(JetsonLink_t *jl, const BMI323_Sample_t *s)
{
  struct __attribute__((packed)) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t ms;
  } p = {
    s->acc_raw[0], s->acc_raw[1], s->acc_raw[2],
    s->gyr_raw[0], s->gyr_raw[1], s->gyr_raw[2],
    HAL_GetTick()
  };
  return jl_send_frame(jl, JL_MSG_IMU, &p, sizeof(p));
}

HAL_StatusTypeDef JL_SendOdom(JetsonLink_t *jl, const float pos_m[3], const float vel_mps[3])
{
  struct __attribute__((packed)) {
    float px, py, pz;
    float vx, vy, vz;
    uint32_t ms;
  } p = {
    pos_m[0], pos_m[1], pos_m[2],
    vel_mps[0], vel_mps[1], vel_mps[2],
    HAL_GetTick()
  };
  return jl_send_frame(jl, JL_MSG_ODOM, &p, sizeof(p));
}

HAL_StatusTypeDef JL_SendTouch(JetsonLink_t *jl, uint16_t touch_bits)
{
  struct __attribute__((packed)) {
    uint16_t bits;
    uint32_t ms;
  } p = { touch_bits, HAL_GetTick() };
  return jl_send_frame(jl, JL_MSG_TOUCH, &p, sizeof(p));
}

HAL_StatusTypeDef JL_SendServo(JetsonLink_t *jl, uint8_t ch, uint16_t us)
{
  struct __attribute__((packed)) {
    uint8_t ch;
    uint16_t us;
    uint32_t ms;
  } p = { ch, us, HAL_GetTick() };
  return jl_send_frame(jl, JL_MSG_SERVO, &p, sizeof(p));
}

HAL_StatusTypeDef JL_SendText(JetsonLink_t *jl, const char *s)
{
  uint8_t len = (uint8_t)strlen(s);
  if (len > 200) len = 200;
  return jl_send_frame(jl, JL_MSG_TEXT, s, len);
}

// ---------------- RX command handler ----------------
//
// You can edit this to call your servo functions, etc.
//
void JL_HandleCommand(JetsonLink_t *jl, uint8_t type, const uint8_t *payload, uint16_t len)
{
  switch ((jl_cmd_t)type) {

    case JL_CMD_PING:
      (void)JL_SendText(jl, "PONG\n");
      break;

    case JL_CMD_SET_STREAM: {
      if (len < 3) break;
      uint8_t mask = payload[0];
      uint16_t period = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
      if (period < 10) period = 10;
      jl->stream_mask = mask;
      jl->stream_period_ms = period;
      (void)JL_SendText(jl, "OK STREAM\n");
    } break;

    case JL_CMD_SET_SERVO_US: {
      if (len < 3) break;
      uint8_t ch = payload[0];
      uint16_t us = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
      g_jl_servo_ch = ch;
      g_jl_servo_us = us;
      g_jl_servo_event = 1;
      (void)JL_SendText(jl, "OK SERVO\n");
    } break;

    case JL_CMD_SET_GAIT: {
      if (len != 16) break;
      memcpy((void*)&g_jl_gait, payload, sizeof(g_jl_gait));
      (void)JL_SendText(jl, "OK GAIT\n");
    } break;

    case JL_CMD_REQUEST_ONESHOT: {
      (void)JL_SendText(jl, "OK REQ\n");
    } break;

    case JL_CMD_LOW_POWER: {
      uint8_t level = 0;
      if (len >= 1) level = payload[0];
      g_jl_low_power_level = level;
      g_jl_low_power_event = 1;

      if (level == 0) (void)JL_SendText(jl, "RECEIVED LOW POWER\n");
      else            (void)JL_SendText(jl, "RECEIVED LOW POWER\n");
    } break;

    default:
      // unknown command
      break;
  }
}
