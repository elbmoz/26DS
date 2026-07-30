#ifndef BALL_VISION_H
#define BALL_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * USART6 line protocol for Question 2:
 *   B,-12,35\n  signed center error in reference pixels and velocity
 *               in reference pixels per second
 *   none\n       no valid detection
 *
 * The receiver is armed only by BallVision_StartStream(). The STM32 sends
 * "c2" to start MaixCAM output and "ok" when Question 2 stops.
 */
extern volatile uint32_t ball_vision_parse_error_count;
extern volatile uint8_t ball_vision_stream_active;

void BallVision_Init(UART_HandleTypeDef *huart);
void BallVision_StartStream(void);
void BallVision_StopStream(void);
void BallVision_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void BallVision_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
