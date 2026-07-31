#ifndef TASK5_SPEED_CONTROL_H
#define TASK5_SPEED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * Question 5 isolated single-loop controller:
 *
 *   MaixCAM error/velocity -> fused state prediction -> PID
 *   predicted error direction -> static feedforward --------+-> speed
 *
 * It does not use motor-position feedback, linkage mappings, or any
 * BalanceControl/Task4PositionControl configuration or state.
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float velocity_feedforward_gain;
    float feedforward_limit;
    float static_feedforward_speed;
    float speed_limit;
    float error_deadband_px;
    float deadband_hysteresis_px;

    /* Low-latency observer and delay-compensating prediction. */
    float velocity_estimator_alpha;
    float camera_velocity_weight;
    float velocity_estimate_limit_px_s;
    float sensor_delay_ms;
    float prediction_time_limit_ms;
    float prediction_offset_limit_px;

    uint32_t control_period_ms;
    uint8_t motor_slope;
} Task5SpeedControlConfig;

typedef struct
{
    volatile uint8_t enabled;
    volatile uint8_t vision_valid;
    volatile uint8_t in_deadband;

    volatile float camera_error;
    volatile float ball_velocity;
    volatile float estimated_velocity;
    volatile float predicted_error;
    volatile float prediction_offset;
    volatile uint32_t vision_age_ms;
    volatile float p_term;
    volatile float i_term;
    volatile float d_term;
    volatile float velocity_feedforward_term;
    volatile float static_feedforward_term;
    volatile float feedforward_term;
    volatile float integral;
    volatile float output;
    volatile int32_t motor_command;

    volatile uint32_t update_count;
    volatile uint32_t measurement_update_count;
    volatile HAL_StatusTypeDef last_motor_status;
} Task5SpeedControlState;

extern Task5SpeedControlConfig task5_speed_control_config;
extern Task5SpeedControlState task5_speed_control_state;

void Task5SpeedControl_Init(void);
void Task5SpeedControl_Start(void);
void Task5SpeedControl_Update(void);
void Task5SpeedControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
