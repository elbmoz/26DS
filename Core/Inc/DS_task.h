#ifndef DS_TASK_H
#define DS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define DS_TASK_MAX_QUESTION          9U
#define DS_TASK_DISPLAY_PERIOD_MS     200U
/* Non-negative endpoint offsets from the stable Question 9 start position. */
#define DS_TASK_Q9_UPPER_PULSES       210
#define DS_TASK_Q9_LOWER_PULSES       150
#define DS_TASK_Q9_MOVE_SPEED         50U
#define DS_TASK_Q9_MOVE_ACCELERATION  20U
#define DS_TASK_Q9_ENDPOINT_DWELL_MS  500U
#define DS_TASK_Q9_MIN_MOVE_MS        200U
#define DS_TASK_Q9_MOVE_TIMEOUT_MS    5000U
#define DS_TASK_Q9_POSITION_PERIOD_MS   20U
#define DS_TASK_Q9_POSITION_FRESH_MS   100U
#define DS_TASK_Q9_STABLE_DELTA       2U
#define DS_TASK_Q9_STABLE_UPDATES     15U

typedef enum
{
    DS_TASK_MENU = 0,
    DS_TASK_RUNNING_Q1,
    DS_TASK_RUNNING_Q2,
    DS_TASK_RUNNING_Q3,
    DS_TASK_RUNNING_Q4,
    DS_TASK_RUNNING_Q5,
    DS_TASK_RUNNING_Q9,
    DS_TASK_FINISHED,
    DS_TASK_NOT_READY
} DS_TaskState;

/* 任务 2 的 OLED 输出状态；暂停后不再产生周期性 I2C 写入。 */
typedef enum
{
    DS_TASK_Q2_OLED_PAUSED = 0,
    DS_TASK_Q2_OLED_UPDATING
} DS_TaskQuestion2OledState;

typedef struct
{
    volatile DS_TaskState state;
    volatile uint8_t selected_question;
    volatile uint8_t oled_ready;
    volatile uint8_t oled_address;
    volatile DS_TaskQuestion2OledState question2_oled_state;
    volatile DS_TaskQuestion2OledState question4_oled_state;
    volatile DS_TaskQuestion2OledState question5_oled_state;
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
