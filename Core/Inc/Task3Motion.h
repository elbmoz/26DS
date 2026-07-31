#ifndef TASK3_MOTION_H
#define TASK3_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum
{
    TASK3_MOTION_DOWN = -1,
    TASK3_MOTION_STOP = 0,
    TASK3_MOTION_UP = 1
} Task3MotionDirection;

typedef struct
{
    volatile uint8_t active;
    volatile uint8_t completed;
    volatile uint8_t fault;
    volatile uint8_t step_index;
    volatile int8_t current_direction;
    volatile uint16_t speed_command;
    volatile uint32_t sequence_started_ms;
    volatile uint32_t step_started_ms;
    volatile HAL_StatusTypeDef last_status;
} Task3MotionState;

extern Task3MotionState task3_motion_state;

void Task3Motion_Init(void);
void Task3Motion_Start(void);
void Task3Motion_Update(void);
void Task3Motion_Stop(void);
uint8_t Task3Motion_GetStepCount(void);
uint8_t Task3Motion_IsComplete(void);
uint8_t Task3Motion_HasFault(void);

#ifdef __cplusplus
}
#endif

#endif
