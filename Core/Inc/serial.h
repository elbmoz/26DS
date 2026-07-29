#ifndef SERIAL_H
#define SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdarg.h>
#include <stdio.h>

void Serial_Init(UART_HandleTypeDef *huart);
void Serial_SendByte(uint8_t byte);
void Serial_SendString(const char *text);
void Serial_Printf(const char *format, ...);

void serial_Handler(UART_HandleTypeDef *huart);
void serial_ErrorHandler(UART_HandleTypeDef *huart);

void Vision_UART_StartCircle(void);
void Vision_UART_StartTask(uint8_t task);
void Vision_UART_StartStream(void);
void Vision_UART_StopStream(void);

#ifdef __cplusplus
}
#endif

#endif
