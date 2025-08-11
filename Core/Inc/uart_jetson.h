#ifndef UART_JETSON_H
#define UART_JETSON_H

#include "main.h"

// Function prototypes
void UART_Jetson_Init(UART_HandleTypeDef *huart);
void UART_Jetson_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
void UART_Jetson_ReceiveData(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

#endif
