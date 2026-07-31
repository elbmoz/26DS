#ifndef HWT101_H
#define HWT101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern volatile double global_angle;
extern volatile uint8_t new_data_received;
extern volatile float angle_x;
extern volatile float angle_y;
extern volatile float angle_z;
extern volatile float angular_velocity_x;
extern volatile float angular_velocity_y;
extern volatile float angular_velocity_z;
extern volatile uint32_t hwt_yaw_frame_count;
extern volatile uint32_t hwt_yaw_last_rx_ms;

void HWT101_Init(UART_HandleTypeDef *huart);
void HWT101_Clear(void);
void hwt_Handler(UART_HandleTypeDef *huart);
void HWT101_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
