#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "PID.h"
#include "stm32f4xx_hal.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float pid_deadband;
    float velocity_deadband;

    uint32_t control_period_ms;
    uint8_t motor_slope;
    int8_t motor_direction;

    float stable_error;
    float stable_velocity;
    uint16_t stable_cycles;

    /*
     * Vision sends the original 640-wide reference-pixel scale. The current
     * 450 px calibrated pipe axis spans about 24.7 cm.
     */
    float pixels_per_cm;
    float positive_5cm_target;
    float negative_5cm_target;
} BalanceControlConfig;

typedef struct
{
    volatile uint8_t enabled;
    volatile uint8_t vision_valid;
    volatile uint8_t stable;
    volatile float target_position;
    volatile float ball_position;
    volatile float ball_velocity;
    volatile float position_error;
    volatile float position_p_term;
    volatile float position_i_term;
    volatile float velocity_d_term;
    volatile float pid_output;
    volatile int32_t motor_command;
    volatile uint16_t stable_count;
    volatile uint32_t update_count;
    volatile HAL_StatusTypeDef last_motor_status;
} BalanceControlState;

extern BalanceControlConfig balance_control_config;
extern BalanceControlState balance_control_state;
extern PID_Cycle balance_position_pid;

void BalanceControl_Init(void);
void BalanceControl_Start(float target_position);
void BalanceControl_SetTarget(float target_position);
void BalanceControl_Update(void);
void BalanceControl_Stop(void);
uint8_t BalanceControl_IsStable(void);

#ifdef __cplusplus
}
#endif

#endif
