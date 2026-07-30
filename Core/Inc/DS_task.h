#ifndef DS_TASK_H
#define DS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define DS_TASK_MAX_QUESTION          9U
#define DS_TASK_DISPLAY_PERIOD_MS     200U

typedef enum
{
    DS_TASK_MENU = 0,
    DS_TASK_RUNNING_Q1,
    DS_TASK_RUNNING_Q2,
    DS_TASK_FINISHED,
    DS_TASK_NOT_READY
} DS_TaskState;

typedef struct
{
    volatile DS_TaskState state;
    volatile uint8_t selected_question;
    volatile uint8_t oled_ready;
    volatile uint8_t oled_address;
    volatile uint32_t start_ms;
    volatile uint32_t elapsed_ms;
} DS_TaskContext;

extern DS_TaskContext ds_task;

void DS_Task_Init(void);
void DS_Task_Run(void);
void DS_Task_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
