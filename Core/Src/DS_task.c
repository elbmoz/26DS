#include "DS_task.h"

#include "DS.h"
#include "LineFollow.h"
#include "OLED.h"
#include "button.h"

DS_TaskContext ds_task;

static uint32_t ds_task_last_display_ms;

static int32_t DS_Task_Clamp(int32_t value,
                             int32_t minimum,
                             int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void DS_Task_ShowSignedFixedOne(uint8_t line,
                                       uint8_t column,
                                       float value)
{
    int32_t scaled = (int32_t)(value * 10.0f);
    uint32_t magnitude;

    scaled = DS_Task_Clamp(scaled, -9999, 9999);
    if (scaled < 0) {
        OLED_ShowChar(line, column, '-');
        magnitude = (uint32_t)(-(scaled + 1)) + 1U;
    } else {
        OLED_ShowChar(line, column, '+');
        magnitude = (uint32_t)scaled;
    }

    OLED_ShowNum(line, (uint8_t)(column + 1U), magnitude / 10U, 3U);
    OLED_ShowChar(line, (uint8_t)(column + 4U), '.');
    OLED_ShowNum(line,
                 (uint8_t)(column + 5U),
                 magnitude % 10U,
                 1U);
}

static void DS_Task_UpdateImuValues(void)
{
    const DS_State *state = DS_GetState();
    int32_t gyro_z = DS_Task_Clamp((int32_t)state->gyro_z_dps,
                                   -999,
                                   999);

    DS_Task_ShowSignedFixedOne(4U, 3U, state->yaw_deg);
    OLED_ShowSignedNum(4U, 12U, gyro_z, 3U);
    OLED_ShowChar(4U, 16U, (state->yaw_valid != 0U) ? 'V' : 'X');
}

static void DS_Task_ShowImuLine(void)
{
    OLED_ShowString(4U, 1U, "Y:+000.0 Z:+000X");
    DS_Task_UpdateImuValues();
}

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
    OLED_ShowString(3U, 1U, "K1:SEL K2:START");
    DS_Task_ShowImuLine();
}

static void DS_Task_ShowQuestion1(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q1 LINE FOLLOW");
    OLED_ShowString(2U, 1U, "TIME:0000.0s");
    OLED_ShowString(3U, 1U, "IR:00000000");
    DS_Task_ShowImuLine();
}

static void DS_Task_ShowFinished(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q1 STOPPED");
    OLED_ShowString(2U, 1U, "TIME:0000.0s");
    DS_Task_ShowTime(ds_task.elapsed_ms);
    OLED_ShowString(3U, 1U, "K2:MENU");
    DS_Task_ShowImuLine();
}

static void DS_Task_ShowNotReady(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "QUESTION");
    OLED_ShowNum(1U, 10U, ds_task.selected_question, 1U);
    OLED_ShowString(2U, 1U, "NOT READY");
    OLED_ShowString(3U, 1U, "K2:MENU");
    DS_Task_ShowImuLine();
}

static void DS_Task_FinishQuestion1(uint32_t now)
{
    LineFollow_Stop();
    ds_task.elapsed_ms = now - ds_task.start_ms;
    ds_task.state = DS_TASK_FINISHED;
    DS_Task_ShowFinished();
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
    DS_Task_UpdateImuValues();
}

static void DS_Task_UpdatePassiveImuDisplay(uint32_t now)
{
    if (ds_task.state == DS_TASK_RUNNING_Q1 ||
        (uint32_t)(now - ds_task_last_display_ms) <
        DS_TASK_DISPLAY_PERIOD_MS) {
        return;
    }

    ds_task_last_display_ms = now;
    DS_Task_UpdateImuValues();
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
        oled_status = OLED_TestAllPixels(1000U);
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

        if (LineFollow_IsLapComplete() != 0U ||
            key2_clicked != 0U) {
            DS_Task_FinishQuestion1(now);
            break;
        }

        DS_Task_UpdateQuestion1Display(now);
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

    DS_Task_UpdatePassiveImuDisplay(now);
}

void DS_Task_Stop(void)
{
    LineFollow_Stop();
    if (ds_task.state == DS_TASK_RUNNING_Q1) {
        ds_task.elapsed_ms = HAL_GetTick() - ds_task.start_ms;
    }
    ds_task.state = DS_TASK_FINISHED;
}
