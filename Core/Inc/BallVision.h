#ifndef BALL_VISION_H
#define BALL_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * USART6 line protocol for Question 2:
 *   -12.5\n    signed ball position relative to the rod center
 *   none\n     no valid detection
 */
extern volatile uint32_t ball_vision_parse_error_count;

void BallVision_Init(UART_HandleTypeDef *huart);
void BallVision_StartStream(void);
void BallVision_StopStream(void);
void BallVision_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void BallVision_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
