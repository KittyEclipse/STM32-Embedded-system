/*
 * uart_jetson.c
 *
 *  Created on: Aug 5, 2025
 *      Author: reinl
 */

#include "uart_jetson.h"
#include <string.h>

// Initialize UART (already done in CubeMX, but can add custom config here)
void UART_Jetson_Init(UART_HandleTypeDef *huart) {
    // (MX_USART1_UART_Init() is called in main.c)
}

// Send data to Jetson Nano
void UART_Jetson_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
    HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY);
}

// Receive data from Jetson Nano (blocking mode)
void UART_Jetson_ReceiveData(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size) {
    HAL_UART_Receive(huart, buffer, size, HAL_MAX_DELAY);
}
