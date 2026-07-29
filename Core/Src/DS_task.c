#include "DS_task.h"

#include "LineFollow.h"
#include "OLED.h"
#include "button.h"

DS_TaskContext ds_task;

static uint32_t ds_task_last_display_ms;

static void DS_Task_ShowTime(uint32_t elapsed_ms)
{
    uint32_t seconds = (elapsed_ms / 1000U) % 10000U;
    uint32_t tenths = (elapsed_ms / 100U) % 10U;

    OLED_ShowNum(2U, 6U, seconds, 4U);
    OLED_ShowChar(2U, 10U, '.');
    OLED_ShowNum(2U, 11U, tenths, 1U);
    OLED_ShowChar(2U, 12U, 's');
}

static void DS_Task_ShowSensorBits(uint8_t sensor_bits)
{
    uint8_t index;

    /* Display order matches physical left-to-right order: IR1 ... IR8. */
    for (index = 0U; index < 8U; index++) {
        OLED_ShowChar(3U,
                      (uint8_t)(4U + index),
                      ((sensor_bits & (uint8_t)(1U << index)) != 0U) ?
                      '1' :
                      '0');
    }
}

static void DS_Task_ShowMenu(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "DS CAR READY");
    OLED_ShowString(2U, 1U, "QUESTION:");
    OLED_ShowNum(2U, 10U, ds_task.selected_question, 1U);
    OLED_ShowString(3U, 1U, "K1:SELECT");
    OLED_ShowString(4U, 1U, "K2:START");
}

static void DS_Task_ShowQuestion1(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q1 LINE FOLLOW");
    OLED_ShowString(2U, 1U, "TIME:0000.0s");
    OLED_ShowString(3U, 1U, "IR:00000000");
    OLED_ShowString(4U, 1U, "E:+0000 C:+000");
}

static void DS_Task_ShowFinished(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q1 STOPPED");
    OLED_ShowString(2U, 1U, "TIME:0000.0s");
    DS_Task_ShowTime(ds_task.elapsed_ms);
    OLED_ShowString(3U, 1U, "K2:MENU");
}

static void DS_Task_ShowNotReady(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "QUESTION");
    OLED_ShowNum(1U, 10U, ds_task.selected_question, 1U);
    OLED_ShowString(2U, 1U, "NOT READY");
    OLED_ShowString(4U, 1U, "K2:MENU");
}

static void DS_Task_StartQuestion1(void)
{
    ds_task.state = DS_TASK_RUNNING_Q1;
    ds_task.start_ms = HAL_GetTick();
    ds_task.elapsed_ms = 0U;
    ds_task_last_display_ms = ds_task.start_ms -
                              DS_TASK_DISPLAY_PERIOD_MS;

    DS_Task_ShowQuestion1();
    LineFollow_Start();
}

static void DS_Task_UpdateQuestion1Display(uint32_t now)
{
    if ((uint32_t)(now - ds_task_last_display_ms) <
        DS_TASK_DISPLAY_PERIOD_MS) {
        return;
    }

    ds_task_last_display_ms = now;
    DS_Task_ShowTime(ds_task.elapsed_ms);
    DS_Task_ShowSensorBits(line_follow_state.sensor_bits);
    OLED_ShowSignedNum(4U, 3U, line_follow_state.error, 4U);
    OLED_ShowSignedNum(4U, 11U, line_follow_state.correction, 3U);
}

void DS_Task_Init(void)
{
    HAL_StatusTypeDef oled_status;

    Button_Init();
    oled_status = OLED_Init();
    if (oled_status == HAL_OK) {
        /*
         * Visible power-on self-test: all pixels light briefly, then the menu
         * overwrites the cleared display RAM.
         */
        oled_status = OLED_TestAllPixels(200U);
    }
    LineFollow_Init();

    ds_task.state = DS_TASK_MENU;
    ds_task.selected_question = 0U;
    ds_task.oled_ready =
        (oled_status == HAL_OK && OLED_IsConnected() != 0U) ? 1U : 0U;
    ds_task.oled_address =
        (ds_task.oled_ready != 0U) ? OLED_GetAddress() : 0U;
    ds_task.start_ms = 0U;
    ds_task.elapsed_ms = 0U;
    ds_task_last_display_ms = 0U;
    DS_Task_ShowMenu();
}

void DS_Task_Run(void)
{
    uint8_t key1_clicked;
    uint8_t key2_clicked;
    uint32_t now;

    Button_Scan();
    key1_clicked = Button1_GetClick();
    key2_clicked = Button2_GetClick();
    now = HAL_GetTick();

    switch (ds_task.state) {
    case DS_TASK_MENU:
        if (key1_clicked != 0U) {
            ds_task.selected_question++;
            if (ds_task.selected_question > DS_TASK_MAX_QUESTION) {
                ds_task.selected_question = 1U;
            }
            OLED_ShowNum(2U, 10U, ds_task.selected_question, 1U);
        }

        if (key2_clicked != 0U && ds_task.selected_question != 0U) {
            if (ds_task.selected_question == 1U) {
                DS_Task_StartQuestion1();
            } else {
                ds_task.state = DS_TASK_NOT_READY;
                DS_Task_ShowNotReady();
            }
        }
        break;

    case DS_TASK_RUNNING_Q1:
        LineFollow_Update();
        ds_task.elapsed_ms = now - ds_task.start_ms;
        DS_Task_UpdateQuestion1Display(now);

        /* Temporary manual finish until a finish-line rule is specified. */
        if (key2_clicked != 0U) {
            LineFollow_Stop();
            ds_task.elapsed_ms = now - ds_task.start_ms;
            ds_task.state = DS_TASK_FINISHED;
            DS_Task_ShowFinished();
        }
        break;

    case DS_TASK_FINISHED:
    case DS_TASK_NOT_READY:
        if (key2_clicked != 0U) {
            ds_task.state = DS_TASK_MENU;
            DS_Task_ShowMenu();
        }
        break;

    default:
        DS_Task_Stop();
        ds_task.state = DS_TASK_MENU;
        DS_Task_ShowMenu();
        break;
    }
}

void DS_Task_Stop(void)
{
    LineFollow_Stop();
    if (ds_task.state == DS_TASK_RUNNING_Q1) {
        ds_task.elapsed_ms = HAL_GetTick() - ds_task.start_ms;
    }
    ds_task.state = DS_TASK_FINISHED;
}
