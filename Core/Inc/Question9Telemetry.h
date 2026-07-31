#ifndef QUESTION9_TELEMETRY_H
#define QUESTION9_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define QUESTION9_TELEMETRY_PERIOD_MS    200U

/*
 * USART6 outbound telemetry for Question 9:
 *
 * Q9,seq,mcu_ms,motor_position,angle_x_x10,angle_y_x10,angle_z_x10,
 *    imu_valid,position_valid,position_status,position_updates,
 *    move_direction,move_status\n
 *
 * Angles are relative to the stable Question 9 start orientation and use
 * 0.1 degree units. move_direction/move_status report the latest endpoint
 * move. HAL status values are 0=OK, 1=ERROR, 2=BUSY and 3=TIMEOUT.
 */
typedef struct
{
    int32_t motor_position;
    float angle_x_deg;
    float angle_y_deg;
    float angle_z_deg;
    uint8_t imu_valid;
    uint8_t position_valid;
    HAL_StatusTypeDef position_status;
    uint32_t position_updates;
    int8_t move_direction;
    HAL_StatusTypeDef move_status;
} Question9TelemetrySnapshot;

typedef struct
{
    volatile uint8_t active;
    volatile uint8_t tx_busy;
    volatile uint32_t attempt_count;
    volatile uint32_t sent_count;
    volatile uint32_t drop_count;
    volatile uint32_t last_attempt_ms;
    volatile HAL_StatusTypeDef last_status;
} Question9TelemetryState;

extern Question9TelemetryState question9_telemetry_state;

void Question9Telemetry_Init(UART_HandleTypeDef *huart);
void Question9Telemetry_Start(void);
void Question9Telemetry_Stop(void);
void Question9Telemetry_Update(const Question9TelemetrySnapshot *snapshot);
void Question9Telemetry_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void Question9Telemetry_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
