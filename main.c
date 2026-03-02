/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407 Discovery + PCA9685 (I2C3) — 3-DOF leg walking verify
  *
  * Channel map (your wiring):
  *   CH0 = Hip forward/back
  *   CH1 = Knee
  *   CH2 = Extension
  *
  * Neutral angles (your measurements):
  *   CH0 hip neutral  = 100 deg
  *   CH1 knee neutral =   0 deg
  *   CH2 ext neutral  =  55 deg
  *
  * Notes:
  * - This file assumes CubeMX generated I2C3 init (MX_I2C3_Init) exists in this same file.
  * - If your PCA9685 address is not 0x40, change PCA9685_ADDR_7BIT.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pca9685.h"
#include <stdio.h>
#include <stdarg.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== PCA9685 I2C address (A0..A5 all low) =====
#define PCA9685_ADDR_7BIT          0x40

// ===== Channel mapping (per your note) =====
#define SERVO_HIP_CH               0
#define SERVO_KNEE_CH              2
#define SERVO_EXT_CH               1

// ===== Servo pulse ranges (safe starting defaults; tune per servo) =====
// If your servos don’t reach full range or buzz, adjust these.
#define HIP_MIN_US                 900.0f
#define HIP_MAX_US                 2200.0f
#define KNEE_MIN_US                900.0f
#define KNEE_MAX_US                2200.0f
#define EXT_MIN_US                 950.0f
#define EXT_MAX_US                 2050.0f

// ===== Neutral pose (your measured neutrals) =====
#define HIP_NEUTRAL_DEG            100.0f
#define KNEE_NEUTRAL_DEG             0.0f
#define EXT_NEUTRAL_DEG             55.0f

// ===== Invert directions if needed (0=no invert, 1=invert) =====
#define INV_HIP                    0
#define INV_KNEE                   0
#define INV_EXT                    0

// ===== Walking timing =====
#define CONTROL_DT_MS              15U     // faster updates -> snappier motion
#define STEP_PERIOD_MS             1100U   // slightly faster cycle
#define START_HOLD_MS              1200U   // hold neutral before gait starts
#define GAIT_RAMP_MS               1500U   // blend from neutral->gait to remove initial snap

// ===== Gait amplitudes (start small!) =====
#define HIP_SWING_AMPL_DEG         36.0f   // hip forward/back swing around neutral
#define KNEE_BEND_AMPL_DEG         85.0f   // knee bend during swing (if knee neutral is 0, reduce!)
#define EXT_LIFT_AMPL_DEG           0.0f   // extension during swing (set 0 to lock extension)

// ===== Boot self-test =====
#define SELFTEST_ON_BOOT           0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float smoothstep(float x)
{
  x = clampf(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

static float apply_inv(float deg, int inv)
{
  if (!inv) return deg;
  return 180.0f - deg;
}

static float map_angle_to_pulse_us(float angle_deg, float min_us, float max_us)
{
  angle_deg = clampf(angle_deg, 0.0f, 180.0f);
  return min_us + (angle_deg / 180.0f) * (max_us - min_us);
}

static void set_servo_deg(PCA9685_Handle_t *hpca, uint8_t ch,
                          float deg, float min_us, float max_us)
{
  float us = map_angle_to_pulse_us(deg, min_us, max_us);
  (void)PCA9685_SetServoPulseUs(hpca, ch, us);
}

/*
 * --- IMPORTANT FOR YOUR REQUEST ---
 * "Hold CH1 exactly where it physically is" at startup == do NOT output PWM on CH1.
 *
 * We do this by putting that PCA9685 channel into "FULL OFF" mode (datasheet).
 * The servo will stop receiving pulses and will NOT move to any commanded angle.
 *
 * Note: With no PWM signal, most hobby servos STOP actively holding torque and can
 * drift if the leg is loaded. This is expected behavior.
 */
#define PCA9685_REG_LED0_ON_L   0x06U

static HAL_StatusTypeDef PCA9685_SetChannelFullOff(PCA9685_Handle_t *hpca,
                                                   uint8_t channel,
                                                   uint8_t full_off)
{
  // LEDn regs: ON_L, ON_H, OFF_L, OFF_H
  // FULL OFF bit is bit4 of OFF_H
  uint8_t reg = (uint8_t)(PCA9685_REG_LED0_ON_L + 4U * channel);
  uint8_t buf[4];

  // We don't care about phase, so write zeros and set/clear full off bit.
  buf[0] = 0x00;                // ON_L
  buf[1] = 0x00;                // ON_H
  buf[2] = 0x00;                // OFF_L
  buf[3] = full_off ? 0x10 : 0x00; // OFF_H (bit4 = FULL OFF)

  return HAL_I2C_Mem_Write(hpca->hi2c, (uint16_t)(hpca->addr_7bit << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 200);
}

static void Leg_SetHipKnee(PCA9685_Handle_t *hpca, float hip_deg, float knee_deg)
{
  hip_deg  = apply_inv(hip_deg,  INV_HIP);
  knee_deg = apply_inv(knee_deg, INV_KNEE);

  set_servo_deg(hpca, SERVO_HIP_CH,  hip_deg,  HIP_MIN_US,  HIP_MAX_US);
  set_servo_deg(hpca, SERVO_KNEE_CH, knee_deg, KNEE_MIN_US, KNEE_MAX_US);
}

// Only use this when you WANT extension to be actively driven.
static void Leg_SetPoseAll(PCA9685_Handle_t *hpca, float hip_deg, float knee_deg, float ext_deg)
{
  hip_deg  = apply_inv(hip_deg,  INV_HIP);
  knee_deg = apply_inv(knee_deg, INV_KNEE);
  ext_deg  = apply_inv(ext_deg,  INV_EXT);

  set_servo_deg(hpca, SERVO_HIP_CH,  hip_deg,  HIP_MIN_US,  HIP_MAX_US);
  set_servo_deg(hpca, SERVO_KNEE_CH, knee_deg, KNEE_MIN_US, KNEE_MAX_US);
  set_servo_deg(hpca, SERVO_EXT_CH,  ext_deg,  EXT_MIN_US,  EXT_MAX_US);
}

// Kick action (NOT CALLED).
// Enables CH1 (extension), kicks out quickly, returns, then disables CH1 again.
static void Kick_Function(PCA9685_Handle_t *hpca)
{
  const float hip0  = HIP_NEUTRAL_DEG;
  const float knee0 = KNEE_NEUTRAL_DEG;
  const float ext0  = EXT_NEUTRAL_DEG;

  const float kick_out_deg = clampf(ext0 + 25.0f, 0.0f, 180.0f);
  const uint32_t out_ms    = 160;
  const uint32_t hold_ms   = 60;
  const uint32_t back_ms   = 220;

  // Enable CH1 output (stop FULL OFF)
  (void)PCA9685_SetChannelFullOff(hpca, SERVO_EXT_CH, 0);

  // Neutral then kick
  Leg_SetPoseAll(hpca, hip0, knee0, ext0);
  HAL_Delay(60);

  Leg_SetPoseAll(hpca, hip0, knee0, kick_out_deg);
  HAL_Delay(out_ms);

  HAL_Delay(hold_ms);

  Leg_SetPoseAll(hpca, hip0, knee0, ext0);
  HAL_Delay(back_ms);

  // Disable CH1 again so it "holds where it is" (no PWM)
  (void)PCA9685_SetChannelFullOff(hpca, SERVO_EXT_CH, 1);
}

// Walking task: call repeatedly from main while(1).
// Extension (CH1) is FULL OFF, so we never touch it here.
static void Walking_Task(PCA9685_Handle_t *hpca, uint32_t step_period_ms)
{
  static uint32_t t0 = 0;
  if (t0 == 0)
  {
    // Start gait at a phase where hip command is ~neutral to avoid an initial 'backward snap'
    const float start_phase = 0.30f; // = 0.5 * stance_end (stance_end=0.60)
    uint32_t now = HAL_GetTick();
    t0 = now - (uint32_t)(start_phase * (float)step_period_ms);
  }

  float phase = (float)((HAL_GetTick() - t0) % step_period_ms) / (float)step_period_ms;

  const float stance_end = 0.60f;

  float hip  = HIP_NEUTRAL_DEG;
  float knee = KNEE_NEUTRAL_DEG;

  if (phase < stance_end)
  {
    float u = smoothstep(phase / stance_end);
    hip  = HIP_NEUTRAL_DEG - HIP_SWING_AMPL_DEG * (2.0f * u - 1.0f);
    knee = KNEE_NEUTRAL_DEG + 7.0f;
  }
  else
  {
    float u = (phase - stance_end) / (1.0f - stance_end);
    float s = smoothstep(u);

    hip = HIP_NEUTRAL_DEG + HIP_SWING_AMPL_DEG * (2.0f * s - 1.0f);

    float lift = (u < 0.5f) ? smoothstep(u * 2.0f) : smoothstep((1.0f - u) * 2.0f);
    knee = KNEE_NEUTRAL_DEG + (KNEE_BEND_AMPL_DEG * lift);
  }

  // Blend-in ramp so the first step doesn't "snap" backward/forward
  static uint32_t gait_start_tick = 0;
  if (gait_start_tick == 0) gait_start_tick = HAL_GetTick();
  float a = (float)(HAL_GetTick() - gait_start_tick) / (float)GAIT_RAMP_MS;
  a = smoothstep(a);

  hip  = HIP_NEUTRAL_DEG  + a * (hip  - HIP_NEUTRAL_DEG);
  knee = KNEE_NEUTRAL_DEG + a * (knee - KNEE_NEUTRAL_DEG);

  hip  = clampf(hip,  0.0f, 180.0f);
  knee = clampf(knee, 0.0f, 180.0f);

  Leg_SetHipKnee(hpca, hip, knee);
}

extern UART_HandleTypeDef huart2;

static void UART_Sendf(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (len <= 0) return;
  if (len > (int)sizeof(buf)) len = (int)sizeof(buf);
  (void)HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)len, 200);
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
PCA9685_Handle_t pca;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  // Make sure the PCA9685 ACKs (wiring/pullups/address)
  if (HAL_I2C_IsDeviceReady(&hi2c3, (PCA9685_ADDR_7BIT << 1), 5, 200) != HAL_OK)
  {
    while (1) { } // stuck here = I2C issue
  }

  // Init PCA9685 at 50 Hz (standard servo rate)
  if (PCA9685_Init(&pca, &hi2c3, PCA9685_ADDR_7BIT, 50.0f) != HAL_OK)
  {
    while (1) { } // stuck here = PCA init failed
  }

  // FULL OFF at startup: do NOT output PWM on any channel.
// This prevents ANY servo movement at boot (servos stay wherever they physically are).
(void)PCA9685_SetChannelFullOff(&pca, SERVO_HIP_CH,  1);
(void)PCA9685_SetChannelFullOff(&pca, SERVO_KNEE_CH, 1);
(void)PCA9685_SetChannelFullOff(&pca, SERVO_EXT_CH,  1);

UART_Sendf("BOOT: outputs disabled (FULL OFF). Servos should NOT move.\r\n");
UART_Sendf("NEUTRAL_DEG H=%.1f K=%.1f E=%.1f\r\n", HIP_NEUTRAL_DEG, KNEE_NEUTRAL_DEG, EXT_NEUTRAL_DEG);

#if START_ON_UART2_COMMAND
UART_Sendf("Send 'g' over UART2 to START walking (CH0+CH2 enabled; CH1 stays off).\r\n");
uint8_t ch = 0;
do {
  (void)HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
} while (ch != (uint8_t)'g' && ch != (uint8_t)'G');
UART_Sendf("START.\r\n");
#endif

// Enable CH0 + CH2 outputs now
(void)PCA9685_SetChannelFullOff(&pca, SERVO_HIP_CH,  0);
(void)PCA9685_SetChannelFullOff(&pca, SERVO_KNEE_CH, 0);

// Command your defined neutral for hip+knee (extension remains FULL OFF)
Leg_SetHipKnee(&pca, HIP_NEUTRAL_DEG, KNEE_NEUTRAL_DEG);
HAL_Delay(START_HOLD_MS);

  while (1)
  {
    Walking_Task(&pca, STEP_PERIOD_MS);
    HAL_Delay(CONTROL_DT_MS);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while (1) { }
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while (1) { }
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    while (1) { }
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    while (1) { }
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
