#ifndef __SCREEN_H
#define __SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Screen_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Screen_SendText(const char *component, const char *text);
HAL_StatusTypeDef Screen_SendQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3,
                                         uint8_t task4, uint8_t task5, uint8_t task6);
HAL_StatusTypeDef Screen_SendGyroAngle(float angle);
HAL_StatusTypeDef Screen_TestSend(void);

#ifdef __cplusplus
}
#endif

#endif
