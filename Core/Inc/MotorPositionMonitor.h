#ifndef MOTOR_POSITION_MONITOR_H
#define MOTOR_POSITION_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define MOTOR_POSITION_MONITOR_PERIOD_MS        100U
#define MOTOR_POSITION_MONITOR_MIN_PERIOD_MS      5U
#define MOTOR_POSITION_MONITOR_TX_GUARD_MS         3U
#define MOTOR_POSITION_MONITOR_RX_BYTES          12U

typedef struct
{
    volatile int32_t position;
    volatile float angle_deg;
    volatile uint32_t request_count;
    volatile uint32_t update_count;
    volatile uint32_t last_update_ms;
    volatile uint32_t request_period_ms;
    volatile uint32_t uart_error;
    volatile HAL_StatusTypeDef last_status;
    volatile uint8_t rx_length;
    volatile uint8_t rx_bytes[MOTOR_POSITION_MONITOR_RX_BYTES];
    volatile uint8_t consecutive_failures;
    volatile uint8_t valid;
    volatile uint8_t active;
} MotorPositionMonitorState;

extern MotorPositionMonitorState motor_position_monitor_state;

void MotorPositionMonitor_Init(void);
void MotorPositionMonitor_Start(void);
void MotorPositionMonitor_StartWithPeriod(uint32_t request_period_ms);
void MotorPositionMonitor_SetPeriod(uint32_t request_period_ms);
void MotorPositionMonitor_Update(void);
void MotorPositionMonitor_Stop(void);
uint8_t MotorPositionMonitor_HasRecentSample(uint32_t maximum_age_ms);
uint8_t MotorPositionMonitor_IsFresh(uint32_t maximum_age_ms);

#ifdef __cplusplus
}
#endif

#endif
