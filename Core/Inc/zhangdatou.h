#ifndef ZHANGDATOU_H
#define ZHANGDATOU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum
{
    MOTOR_DISABLE = 0x00,
    MOTOR_ENABLE = 0x01
} MotorState;

typedef enum
{
    DIRECTION_POSITIVE = 0x00,
    DIRECTION_NEGATIVE = 0x01
} MotorDirection;

typedef enum
{
    RELATIVE_POSITION = 0x00,
    ABSOLUTE_POSITION = 0x01
} PositionMode;

typedef enum
{
    SNF_DISABLE = 0x00,
    SNF_ENABLE = 0x01
} SNFMODE;

void Motor_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_Enable(uint8_t address,
                               MotorState state,
                               SNFMODE sync_mode);
HAL_StatusTypeDef Motor_SpeedControl(uint8_t address,
                                    MotorDirection direction,
                                    uint16_t slope,
                                    uint16_t speed,
                                    SNFMODE sync_mode);
HAL_StatusTypeDef Motor_PositionControl(uint8_t address,
                                       MotorDirection direction,
                                       uint16_t speed,
                                       uint8_t acceleration,
                                       uint32_t pulses,
                                       PositionMode position_mode,
                                       SNFMODE sync_mode);
HAL_StatusTypeDef Motor_Stop(uint8_t address, SNFMODE sync_mode);
HAL_StatusTypeDef Motor_SyncStart(void);

HAL_StatusTypeDef Motor_RequestSpeedUpdate(uint8_t address);
/*
 * Starts a non-blocking 0x36 request. Supports both the six-byte
 * address+int32+0x6B reply and the eight-byte
 * address+0x36+sign+magnitude+0x6B reply.
 */
HAL_StatusTypeDef Motor_RequestPositionUpdate(uint8_t address);
HAL_StatusTypeDef Motor_ReadSpeed(uint8_t address, int32_t *speed_rpm);
HAL_StatusTypeDef Motor_ReadPosition(uint8_t address,
                                     int32_t *position,
                                     float *angle);
HAL_StatusTypeDef Motor_ClearPosition(uint8_t address, uint8_t *state_code);

uint8_t Motor_IsComBusy(void);
HAL_StatusTypeDef Motor_GetLastComStatus(void);
void Motor_CancelRequest(void);
uint8_t Motor_GetLastRxFrame(uint8_t *buffer,
                             uint8_t buffer_length,
                             uint32_t *uart_error);
uint32_t Motor_GetSpeedTxCount(void);
HAL_StatusTypeDef Motor_GetLastSpeedTxStatus(void);
void Motor_GetLastSpeedTxCommand(uint8_t *command, uint8_t length);

void Motor_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Motor_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
