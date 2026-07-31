#ifndef TASK4_POSITION_CONTROL_H
#define TASK4_POSITION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * 任务 4：复用任务 2 的小球位置外环，但由驱动器位置模式直接追踪管道角。
 * 本模块拥有独立配置和运行状态，不读写 balance_control_config/state。
 */
typedef struct
{
    /* 小球位置/速度 -> 目标管道角。 */
    float outer_kp_deg_per_px;
    float outer_ki_deg_per_px_s;
    float outer_kd_deg_per_px_s;
    float outer_integral_limit_px_s;
    float outer_angle_limit_deg;

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

    float damping_velocity_px_s;
    float damping_kp_scale;
    float damping_kd_scale;
    float damping_angle_limit_scale;
    float freeze_integral_velocity_px_s;
    float freeze_kp_scale;
    float freeze_kd_scale;
    float freeze_angle_limit_scale;
    float freeze_integral_decay;

    /* a010e37 标定与 0xFD 位置命令单位换算。 */
    float rod_angle_per_position_count; /* -0.0020022658 °/count。 */
    float position_counts_per_command_pulse; /* 当前 3200 细分下为 20.48。 */
    float rod_angle_limit_deg;
    int8_t tilt_direction;

    /* 驱动器内部位置环只需速度上限和加速度档。 */
    uint16_t motor_move_speed;
    uint8_t motor_move_acceleration;

    uint32_t control_period_ms;
    uint32_t motor_position_startup_period_ms; /* Fast anchor acquisition. */
    uint32_t motor_position_period_ms; /* Low-rate 0x36 diagnostic polling. */

    float stable_error_px;
    float stable_velocity_px_s;
    uint16_t stable_frames;
} Task4PositionControlConfig;

typedef struct
{
    volatile uint8_t enabled;
    volatile uint8_t vision_valid;
    volatile uint8_t motor_position_valid; /* Horizontal anchor captured. */
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

    volatile int32_t motor_zero_position;
    volatile int32_t motor_zero_command_pulses;
    volatile int32_t motor_position;
    volatile float rod_angle_deg;
    volatile int32_t target_motor_position;
    volatile int32_t target_motor_pulses;
    volatile int32_t position_error_counts;
    volatile int32_t motor_command_pulses;

    volatile uint16_t stable_count;
    volatile uint32_t update_count;
    volatile uint32_t outer_update_count;
    volatile uint32_t motor_position_update_count;
    volatile HAL_StatusTypeDef last_motor_status;
} Task4PositionControlState;

extern Task4PositionControlConfig task4_position_control_config;
extern Task4PositionControlState task4_position_control_state;

void Task4PositionControl_Init(void);
void Task4PositionControl_Start(float target_position);
void Task4PositionControl_SetTarget(float target_position);
void Task4PositionControl_Update(void);
void Task4PositionControl_Stop(void);
uint8_t Task4PositionControl_IsStable(void);

#ifdef __cplusplus
}
#endif

#endif
