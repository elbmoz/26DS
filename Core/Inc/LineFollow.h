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

    /* First version uses proportional steering only. */
    float kp;
    int32_t max_correction;

    /* Stability helpers around the P controller. */
    int32_t curve_slowdown;
    int32_t correction_slew_per_update;
    int32_t command_limit;

    /* Safe search command when none of the eight sensors sees the line. */
    int32_t lost_speed;
    int32_t lost_correction;
} LineFollowConfig;

typedef struct
{
    volatile uint8_t enabled;
    volatile uint8_t sensor_bits;
    volatile uint8_t line_lost;
    volatile int32_t error;
    volatile int32_t correction;
    volatile int32_t left_command;
    volatile int32_t right_command;
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

#ifdef __cplusplus
}
#endif

#endif
