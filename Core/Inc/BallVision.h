#ifndef BALL_VISION_H
#define BALL_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * USART6 line protocol for Questions 2, 4 and 5:
 *   B,-12,35\n  signed center error in reference pixels and velocity
 *               in reference pixels per second
 *   none\n       no valid detection
 *   PG/PS/PR/PT/PX runtime tuning commands; see BalanceTuning.h
 *
 * The receiver is armed only by BallVision_StartStream(). The STM32 sends
 * "c2" to start MaixCAM output and "ok" when a control task stops.
 *
 * After each balance-motor command attempt, STM32 sends one non-blocking
 * feedback line to MaixCAM:
 *   F,seq,mcu_ms,vision_frame,vision_age_ms,position_x10,velocity_x10,
 *     error_x10,p_x100,i_x100,d_x100,motor_command,motor_status\n
 * P/I/D use rod-angle units in Questions 2/4 and motor-speed units in
 * Question 5. motor_command is the signed speed request in Questions 2/5
 * and the absolute position target in Question 4. For Question 5,
 * position_x10 remains the measured camera error while error_x10 is the
 * delay-compensated predicted error used by the controller.
 *
 * Missing sequence numbers mean feedback was dropped because USART6 was
 * still transmitting. Motor control is never delayed to wait for logging.
 * After the first tuning command, feedback switches to the versioned F2 frame
 * containing both outer-loop and inner-loop state. PA is the STM32-side ACK.
 */
extern volatile uint32_t ball_vision_parse_error_count;
extern volatile uint8_t ball_vision_stream_active;
extern volatile uint32_t ball_vision_feedback_attempt_count;
extern volatile uint32_t ball_vision_feedback_sent_count;
extern volatile uint32_t ball_vision_feedback_drop_count;

typedef struct
{
    uint32_t vision_frame;
    uint32_t vision_age_ms;
    float position;
    float velocity;
    float control_error;
    float p_term;
    float i_term;
    float d_term;
    float target_rod_angle;
    float actual_rod_angle;
    float rod_rate;
    float angle_error;
    float desired_speed;
    int32_t motor_command;
    uint32_t position_age_ms;
    uint8_t position_valid;
    uint8_t protection_state;
    HAL_StatusTypeDef motor_status;
    uint8_t tuning_mode;
} BallVisionFeedbackV2;

typedef struct
{
    uint32_t sequence;
    uint8_t status;
    float outer_kp;
    float outer_kd;
    float angle_limit;
    float inner_kp;
    float inner_kd;
    float speed_limit;
    float slew;
    float deadband;
    float min_speed;
    float outer_ki;
    uint8_t mode;
    float test_target;
    uint32_t remaining_ms;
} BallVisionTuningAck;

void BallVision_Init(UART_HandleTypeDef *huart);
void BallVision_StartStream(void);
void BallVision_StopStream(void);
HAL_StatusTypeDef BallVision_SendFeedback(uint32_t vision_frame,
                                          uint32_t vision_age_ms,
                                          float position,
                                          float velocity,
                                          float control_error,
                                          float p_term,
                                          float i_term,
                                          float d_term,
                                          int32_t motor_command,
                                          HAL_StatusTypeDef motor_status);
HAL_StatusTypeDef BallVision_SendFeedbackV2(
    const BallVisionFeedbackV2 *feedback);
HAL_StatusTypeDef BallVision_SendTuningAck(
    const BallVisionTuningAck *ack);
void BallVision_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void BallVision_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void BallVision_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
