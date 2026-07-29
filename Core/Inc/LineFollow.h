#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef struct
{
    /* Motor speed unit follows the stepper protocol: 10 means about 1 RPM. */
    int32_t base_speed;
    uint8_t motor_slope;
    uint32_t control_period_ms;

    /* Discrete PD steering: kd acts on error change per control update. */
    float kp;
    float kd;
    int32_t max_correction;
    int32_t max_d_correction;

    /* Stability helpers around the P controller. */
    int32_t curve_slowdown;
    /* 0 disables software ramping and applies each correction immediately. */
    int32_t correction_slew_per_update;
    int32_t command_limit;

    /* Safe search command when none of the eight sensors sees the line. */
    int32_t lost_speed;
    int32_t lost_correction;

    /* Stop Question 1 after one unwrapped IMU yaw revolution. */
    uint8_t lap_stop_enabled;
    float lap_target_yaw_deg;
    float lap_max_yaw_step_deg;
    uint32_t lap_min_time_ms;
    uint8_t lap_confirm_frames;
} LineFollowConfig;

typedef struct
{
    volatile uint8_t enabled;
    volatile uint8_t sensor_bits;
    volatile uint8_t line_lost;
    volatile int32_t error;
    volatile int32_t error_delta;
    volatile int32_t d_correction;
    volatile int32_t correction;
    volatile int32_t left_command;
    volatile int32_t right_command;
    volatile uint8_t yaw_valid;
    volatile uint8_t lap_complete;
    volatile uint8_t lap_confirm_count;
    volatile float yaw_deg;
    volatile float yaw_delta_deg;
    volatile float accumulated_yaw_deg;
    volatile uint32_t yaw_frame_count;
    volatile uint32_t rejected_yaw_steps;
    volatile HAL_StatusTypeDef last_motor_status;
    volatile uint32_t update_count;
} LineFollowState;

/* Public globals make live parameter tuning and Watch inspection easy in Keil. */
extern LineFollowConfig line_follow_config;
extern LineFollowState line_follow_state;

void LineFollow_Init(void);
void LineFollow_Start(void);
void LineFollow_Update(void);
void LineFollow_Stop(void);
uint8_t LineFollow_IsLapComplete(void);

#ifdef __cplusplus
}
#endif

#endif
