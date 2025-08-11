/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I_O_Reg_RESET_Pin GPIO_PIN_2
#define I_O_Reg_RESET_GPIO_Port GPIOE
#define IMU_CS_GYRO_Pin GPIO_PIN_3
#define IMU_CS_GYRO_GPIO_Port GPIOA
#define IMU_ACC_Pin GPIO_PIN_4
#define IMU_ACC_GPIO_Port GPIOA
#define IMU_INT_ACC_Pin GPIO_PIN_0
#define IMU_INT_ACC_GPIO_Port GPIOB
#define IMU_INT_GYR_Pin GPIO_PIN_1
#define IMU_INT_GYR_GPIO_Port GPIOB
#define BMS_ALERT_Pin GPIO_PIN_15
#define BMS_ALERT_GPIO_Port GPIOE
#define BMS_I2C2_SCL_Pin GPIO_PIN_10
#define BMS_I2C2_SCL_GPIO_Port GPIOB
#define BMS_I2C2_SDA_Pin GPIO_PIN_11
#define BMS_I2C2_SDA_GPIO_Port GPIOB
#define SERVO_I2C3_SDA_Pin GPIO_PIN_9
#define SERVO_I2C3_SDA_GPIO_Port GPIOC
#define SERVO_I2C3_SCL_Pin GPIO_PIN_8
#define SERVO_I2C3_SCL_GPIO_Port GPIOA
#define I_O_I2C1_SCL_Pin GPIO_PIN_6
#define I_O_I2C1_SCL_GPIO_Port GPIOB
#define I_O_I2C1_SDA_Pin GPIO_PIN_7
#define I_O_I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
