#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * Question 2 cascaded controller:
 *
 * ball position/velocity -> target rod angle -> motor speed
 *
 * All tunable values are collected here. The defaults preserve the structure
 * and approximate physical response of the proven reference project after
 * converting its millimetre input to the current reference-pixel scale.
 */
typedef struct
{
    /* Outer ball controller. Output unit is rod degrees. */
    float outer_kp_deg_per_px;
    float outer_ki_deg_per_px_s;
    float outer_kd_deg_per_px_s;
    float outer_integral_limit_px_s;
    float outer_angle_limit_deg;

    /* Near-target gain scheduling and level hold. */
    float hold_band_px;
    float fine_band_px;
    float fine_velocity_px_s;
    float soft_kp_scale;
    float soft_kd_scale;
    float soft_angle_limit_scale;
    float soft_ki_deg_per_px_s;
    float fine_fast_kp_scale;
    float fine_fast_ki_scale;
    float fine_fast_angle_limit_scale;
    float hold_integral_decay;
    float fine_hold_inner_kp_scale;

    /* High ball-speed damping. */
    float damping_velocity_px_s;
    float damping_kp_scale;
    float damping_kd_scale;
    float damping_angle_limit_scale;
    float freeze_integral_velocity_px_s;
    float freeze_kp_scale;
    float freeze_kd_scale;
    float freeze_angle_limit_scale;
    float freeze_integral_decay;

    /* Motor-position to rod-angle calibration. */
    float motor_zero_angle_deg;
    float rod_angle_per_motor_degree;
    float rod_angle_limit_deg;
    uint8_t capture_motor_zero_on_start;

    /* Inner rod-angle controller. Output unit follows the RS485 speed mode. */
    float angle_kp_speed_per_deg;
    float motor_speed_limit;
    float motor_speed_deadband;
    float motor_min_speed;
    float motor_slew_per_update;
    uint8_t motor_slope;
    int8_t tilt_direction;
    int8_t motor_direction;

    uint32_t control_period_ms;
    uint32_t motor_position_period_ms;
    uint32_t motor_position_timeout_ms;

    float stable_error_px;
    float stable_velocity_px_s;
    uint16_t stable_frames;

    float pixels_per_cm;
    float positive_5cm_target;
    float negative_5cm_target;
} BalanceControlConfig;

typedef struct
{
    volatile uint8_t enabled;
    volatile uint8_t vision_valid;
    volatile uint8_t motor_position_valid;
    volatile uint8_t leveling;
    volatile uint8_t stable;
    volatile uint8_t motor_zero_pending;

    volatile float target_position;
    volatile float ball_position;
    volatile float ball_velocity;
    volatile float position_error;
    volatile float position_p_term;
    volatile float position_i_term;
    volatile float velocity_d_term;
    volatile float outer_integral;
    volatile float outer_output_deg;
    volatile float target_rod_angle_deg;

    volatile int32_t motor_position;
    volatile float motor_angle_deg;
    volatile float rod_angle_deg;
    volatile float angle_error_deg;
    volatile float desired_motor_speed;
    volatile int32_t motor_command;

    volatile uint16_t stable_count;
    volatile uint32_t update_count;
    volatile uint32_t outer_update_count;
    volatile uint32_t motor_position_update_count;
    volatile HAL_StatusTypeDef last_motor_status;
} BalanceControlState;

extern BalanceControlConfig balance_control_config;
extern BalanceControlState balance_control_state;

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
