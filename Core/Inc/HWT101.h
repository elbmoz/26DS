#ifndef HWT101_H
#define HWT101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern volatile double global_angle;
extern volatile uint8_t new_data_received;
extern volatile float angular_velocity_y;
extern volatile float angular_velocity_z;

void HWT101_Init(UART_HandleTypeDef *huart);
void HWT101_Clear(void);
void hwt_Handler(UART_HandleTypeDef *huart);
void HWT101_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
