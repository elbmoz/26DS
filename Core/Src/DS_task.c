#include "DS_task.h"

#include "BalanceControl.h"
#include "BallVision.h"
#include "DS.h"
#include "LineFollow.h"
#include "MotorPositionMonitor.h"
#include "OLED.h"
#include "Question9Telemetry.h"
#include "button.h"

DS_TaskContext ds_task;

static uint32_t ds_task_last_display_ms;
static uint8_t ds_task_q9_diagnostic_page;
static int8_t ds_task_q9_next_move_direction;
static int8_t ds_task_q9_last_move_direction;
static uint32_t ds_task_q9_last_move_ms;
static HAL_StatusTypeDef ds_task_q9_move_status;
static uint8_t ds_task_q9_imu_zero_pending;
static float ds_task_q9_imu_zero_x;
static float ds_task_q9_imu_zero_y;
static float ds_task_q9_imu_zero_z;
static float ds_task_q9_angle_x;
static float ds_task_q9_angle_y;
static float ds_task_q9_angle_z;

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

static float DS_Task_NormalizeAngle(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void DS_Task_CaptureQuestion9ImuZero(void)
{
    const DS_State *state = DS_GetState();

    ds_task_q9_angle_x = 0.0f;
    ds_task_q9_angle_y = 0.0f;
    ds_task_q9_angle_z = 0.0f;

    if (state->yaw_valid == 0U) {
        ds_task_q9_imu_zero_pending = 1U;
        return;
    }

    ds_task_q9_imu_zero_x = state->roll_deg;
    ds_task_q9_imu_zero_y = state->pitch_deg;
    ds_task_q9_imu_zero_z = state->yaw_deg;
    ds_task_q9_imu_zero_pending = 0U;
}

static void DS_Task_UpdateQuestion9Angles(const DS_State *state)
{
    if (ds_task_q9_imu_zero_pending != 0U) {
        if (state->yaw_valid != 0U) {
            DS_Task_CaptureQuestion9ImuZero();
        }
        return;
    }

    ds_task_q9_angle_x = DS_Task_NormalizeAngle(
        state->roll_deg - ds_task_q9_imu_zero_x);
    ds_task_q9_angle_y = DS_Task_NormalizeAngle(
        state->pitch_deg - ds_task_q9_imu_zero_y);
    ds_task_q9_angle_z = DS_Task_NormalizeAngle(
        state->yaw_deg - ds_task_q9_imu_zero_z);
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

static void DS_Task_ShowTimeOnLine(uint8_t line, uint32_t elapsed_ms)
{
    uint32_t seconds = (elapsed_ms / 1000U) % 10000U;
    uint32_t tenths = (elapsed_ms / 100U) % 10U;

    OLED_ShowNum(line, 6U, seconds, 4U);
    OLED_ShowChar(line, 10U, '.');
    OLED_ShowNum(line, 11U, tenths, 1U);
    OLED_ShowChar(line, 12U, 's');
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

static void DS_Task_ShowQuestion2(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q2 CENTER RX:X R");
    OLED_ShowString(2U, 1U, "E:+000.0 V:+000");
    OLED_ShowString(3U, 1U, "OUT:+000 ST:000");
    OLED_ShowString(4U, 1U, "TIME:0000.0s");
}

static void DS_Task_ShowMotorPositionAngle(float angle_deg)
{
    int64_t scaled_angle;
    uint64_t magnitude;

    scaled_angle = (int64_t)(angle_deg * 10.0f);
    if (angle_deg < 0.0f) {
        OLED_ShowChar(3U, 3U, '-');
        magnitude = (uint64_t)(-(scaled_angle + 1LL)) + 1ULL;
    } else {
        OLED_ShowChar(3U, 3U, '+');
        magnitude = (uint64_t)scaled_angle;
    }

    OLED_ShowNum(3U, 4U, (uint32_t)(magnitude / 10ULL), 9U);
    OLED_ShowChar(3U, 13U, '.');
    OLED_ShowNum(3U, 14U, (uint32_t)(magnitude % 10ULL), 1U);
}

static const char *DS_Task_GetMotorPositionStatusText(void)
{
    switch (motor_position_monitor_state.last_status) {
    case HAL_OK:
        return "OK  ";
    case HAL_BUSY:
        return "WAIT";
    case HAL_TIMEOUT:
        return "TIME";
    default:
        return "ERR ";
    }
}

static void DS_Task_UpdateQuestion9Values(void)
{
    const DS_State *state = DS_GetState();
    char move_character;
    uint32_t update_count;

    OLED_ShowSignedNum(
        1U,
        6U,
        motor_position_monitor_state.position,
        10U);

    DS_Task_UpdateQuestion9Angles(state);
    DS_Task_ShowSignedFixedOne(2U, 2U, ds_task_q9_angle_x);
    DS_Task_ShowSignedFixedOne(2U, 10U, ds_task_q9_angle_y);
    DS_Task_ShowSignedFixedOne(3U, 2U, ds_task_q9_angle_z);
    OLED_ShowChar(3U, 13U, (state->yaw_valid != 0U) ? 'V' : 'X');

    OLED_ShowString(
        4U,
        4U,
        DS_Task_GetMotorPositionStatusText());

    update_count = motor_position_monitor_state.update_count;
    if (update_count > 99999U) {
        update_count = 99999U;
    }
    OLED_ShowNum(4U, 11U, update_count, 5U);

    if (ds_task_q9_move_status != HAL_OK &&
        ds_task_q9_move_status != HAL_BUSY) {
        move_character = '!';
    } else if (ds_task_q9_last_move_direction > 0) {
        move_character = '+';
    } else if (ds_task_q9_last_move_direction < 0) {
        move_character = '-';
    } else {
        move_character = '.';
    }
    OLED_ShowChar(4U, 16U, move_character);
}

static void DS_Task_UpdateQuestion9Telemetry(void)
{
    const DS_State *state = DS_GetState();
    Question9TelemetrySnapshot snapshot;

    DS_Task_UpdateQuestion9Angles(state);

    snapshot.motor_position =
        motor_position_monitor_state.position;
    snapshot.angle_x_deg = ds_task_q9_angle_x;
    snapshot.angle_y_deg = ds_task_q9_angle_y;
    snapshot.angle_z_deg = ds_task_q9_angle_z;
    snapshot.imu_valid = state->yaw_valid;
    snapshot.position_valid =
        motor_position_monitor_state.valid;
    snapshot.position_status =
        motor_position_monitor_state.last_status;
    snapshot.position_updates =
        motor_position_monitor_state.update_count;
    snapshot.move_direction =
        ds_task_q9_last_move_direction;
    snapshot.move_status = ds_task_q9_move_status;

    Question9Telemetry_Update(&snapshot);
}

static void DS_Task_ShowQuestion9(void)
{
    ds_task_q9_diagnostic_page = 0U;
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q9 P:+0000000000");
    OLED_ShowString(2U, 1U, "X+000.0 Y+000.0");
    OLED_ShowString(3U, 1U, "Z+000.0 IMU:X");
    OLED_ShowString(4U, 1U, "RX:WAIT N:00000");
    DS_Task_UpdateQuestion9Values();
}

static void DS_Task_UpdateQuestion9Diagnostic(void)
{
    uint8_t index;
    uint8_t line;
    uint8_t column;
    uint8_t rx_length = motor_position_monitor_state.rx_length;

    if (rx_length > 99U) {
        rx_length = 99U;
    }
    OLED_ShowString(
        1U,
        7U,
        DS_Task_GetMotorPositionStatusText());
    OLED_ShowNum(1U, 14U, rx_length, 2U);

    for (index = 0U; index < MOTOR_POSITION_MONITOR_RX_BYTES; index++) {
        line = (index < 6U) ? 2U : 3U;
        column = (uint8_t)(4U + (index % 6U) * 2U);
        if (index < motor_position_monitor_state.rx_length) {
            OLED_ShowHexNum(
                line,
                column,
                motor_position_monitor_state.rx_bytes[index],
                2U);
        } else {
            OLED_ShowString(line, column, "--");
        }
    }

    OLED_ShowHexNum(
        4U,
        4U,
        motor_position_monitor_state.uart_error,
        8U);
}

static void DS_Task_ShowQuestion9Diagnostic(void)
{
    ds_task_q9_diagnostic_page = 1U;
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q9 RX WAIT L:00");
    OLED_ShowString(2U, 1U, "B0:------------");
    OLED_ShowString(3U, 1U, "B6:------------");
    OLED_ShowString(4U, 1U, "UE:00000000");
    DS_Task_UpdateQuestion9Diagnostic();
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

static void DS_Task_ShowQuestion2Finished(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q2 STOPPED");
    OLED_ShowString(2U, 1U, "TIME:0000.0s");
    DS_Task_ShowTime(ds_task.elapsed_ms);
    OLED_ShowString(3U, 1U, "E:+000.0 V:+000");
    DS_Task_ShowSignedFixedOne(
        3U,
        3U,
        balance_control_state.ball_position);
    OLED_ShowSignedNum(
        3U,
        12U,
        DS_Task_Clamp(
            (int32_t)balance_control_state.ball_velocity,
            -999,
            999),
        3U);
    OLED_ShowString(4U, 1U, "K2:MENU");
}

static void DS_Task_ShowQuestion9Finished(void)
{
    OLED_Clear();
    OLED_ShowString(1U, 1U, "Q9 STOPPED X");
    OLED_ShowString(2U, 1U, "P:+0000000000");
    OLED_ShowString(3U, 1U, "A:+000000000.0");
    OLED_ShowString(4U, 1U, "K2:MENU RX:WAIT");

    OLED_ShowSignedNum(
        2U,
        3U,
        motor_position_monitor_state.position,
        10U);
    DS_Task_ShowMotorPositionAngle(
        motor_position_monitor_state.angle_deg);
    OLED_ShowString(
        4U,
        12U,
        DS_Task_GetMotorPositionStatusText());
    OLED_ShowChar(
        1U,
        12U,
        (motor_position_monitor_state.valid != 0U) ? 'V' : 'X');
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

static void DS_Task_FinishQuestion2(uint32_t now)
{
    BalanceControl_Stop();
    BallVision_StopStream();
    ds_task.elapsed_ms = now - ds_task.start_ms;
    ds_task.state = DS_TASK_FINISHED;
    DS_Task_ShowQuestion2Finished();
}

static void DS_Task_StartQuestion2(void)
{
    ds_task.state = DS_TASK_RUNNING_Q2;
    ds_task.start_ms = HAL_GetTick();
    ds_task.elapsed_ms = 0U;
    ds_task_last_display_ms = ds_task.start_ms -
                              DS_TASK_DISPLAY_PERIOD_MS;

    BalanceControl_Start(0.0f);
    BallVision_StartStream();
    DS_Task_ShowQuestion2();
}

static void DS_Task_FinishQuestion9(void)
{
    Question9Telemetry_Stop();
    MotorPositionMonitor_Stop();
    ds_task_q9_move_status = DS_BalanceStop();
    ds_task.elapsed_ms = HAL_GetTick() - ds_task.start_ms;
    ds_task.state = DS_TASK_FINISHED;
    DS_Task_ShowQuestion9Finished();
}

static void DS_Task_StartQuestion9(void)
{
    ds_task.state = DS_TASK_RUNNING_Q9;
    ds_task.start_ms = HAL_GetTick();
    ds_task.elapsed_ms = 0U;
    ds_task_last_display_ms = ds_task.start_ms -
                              DS_TASK_DISPLAY_PERIOD_MS;
    ds_task_q9_next_move_direction = 1;
    ds_task_q9_last_move_direction = 0;
    ds_task_q9_last_move_ms =
        ds_task.start_ms - DS_TASK_Q9_MOVE_PERIOD_MS;
    ds_task_q9_move_status = HAL_BUSY;
    DS_Task_CaptureQuestion9ImuZero();

    MotorPositionMonitor_Start();
    Question9Telemetry_Start();
    DS_Task_ShowQuestion9();
}

static void DS_Task_UpdateQuestion9Motion(uint32_t now)
{
    int32_t pulses;
    HAL_StatusTypeDef status;

    if ((uint32_t)(now - ds_task_q9_last_move_ms) <
        DS_TASK_Q9_MOVE_PERIOD_MS) {
        return;
    }

    pulses = (ds_task_q9_next_move_direction > 0) ?
             DS_TASK_Q9_MOVE_PULSES :
             -DS_TASK_Q9_MOVE_PULSES;
    status = DS_BalanceMoveRelative(
        pulses,
        DS_TASK_Q9_MOVE_SPEED,
        DS_TASK_Q9_MOVE_ACCELERATION);
    ds_task_q9_move_status = status;

    if (status == HAL_BUSY) {
        return;
    }

    ds_task_q9_last_move_ms = now;
    if (status == HAL_OK) {
        ds_task_q9_last_move_direction =
            ds_task_q9_next_move_direction;
        ds_task_q9_next_move_direction =
            (int8_t)-ds_task_q9_next_move_direction;
    }
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

static void DS_Task_UpdateQuestion2Display(uint32_t now)
{
    int32_t output;
    int32_t velocity;
    uint32_t stable_count;

    if ((uint32_t)(now - ds_task_last_display_ms) <
        DS_TASK_DISPLAY_PERIOD_MS) {
        return;
    }

    ds_task_last_display_ms = now;
    DS_Task_ShowSignedFixedOne(
        2U,
        3U,
        balance_control_state.ball_position);

    velocity = DS_Task_Clamp(
        (int32_t)balance_control_state.ball_velocity,
        -999,
        999);
    OLED_ShowSignedNum(2U, 12U, velocity, 3U);
    OLED_ShowChar(
        1U,
        14U,
        (balance_control_state.vision_valid != 0U) ? 'V' : 'X');

    output = DS_Task_Clamp(
        balance_control_state.motor_command,
        -999,
        999);
    OLED_ShowSignedNum(3U, 5U, output, 3U);
    stable_count = (balance_control_state.stable_count > 999U) ?
                   999U :
                   balance_control_state.stable_count;
    OLED_ShowNum(3U, 13U, stable_count, 3U);
    OLED_ShowChar(
        1U,
        16U,
        (BalanceControl_IsStable() != 0U) ? 'S' : 'R');
    DS_Task_ShowTimeOnLine(4U, ds_task.elapsed_ms);
}

static void DS_Task_UpdateQuestion9Display(uint32_t now)
{
    if ((uint32_t)(now - ds_task_last_display_ms) <
        DS_TASK_DISPLAY_PERIOD_MS) {
        return;
    }

    ds_task_last_display_ms = now;
    if (motor_position_monitor_state.valid == 0U &&
        motor_position_monitor_state.last_status != HAL_BUSY) {
        if (ds_task_q9_diagnostic_page == 0U) {
            DS_Task_ShowQuestion9Diagnostic();
        } else {
            DS_Task_UpdateQuestion9Diagnostic();
        }
    } else if (ds_task_q9_diagnostic_page != 0U) {
        DS_Task_ShowQuestion9();
    } else {
        DS_Task_UpdateQuestion9Values();
    }
}

static void DS_Task_UpdatePassiveImuDisplay(uint32_t now)
{
    if (ds_task.state == DS_TASK_RUNNING_Q1 ||
        ds_task.state == DS_TASK_RUNNING_Q2 ||
        ds_task.state == DS_TASK_RUNNING_Q9 ||
        (ds_task.state == DS_TASK_FINISHED &&
         (ds_task.selected_question == 2U ||
          ds_task.selected_question == 9U)) ||
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
    BalanceControl_Init();
    MotorPositionMonitor_Init();

    ds_task.state = DS_TASK_MENU;
    ds_task.selected_question = 0U;
    ds_task.oled_ready =
        (oled_status == HAL_OK && OLED_IsConnected() != 0U) ? 1U : 0U;
    ds_task.oled_address =
        (ds_task.oled_ready != 0U) ? OLED_GetAddress() : 0U;
    ds_task.start_ms = 0U;
    ds_task.elapsed_ms = 0U;
    ds_task_last_display_ms = 0U;
    ds_task_q9_diagnostic_page = 0U;
    ds_task_q9_next_move_direction = 1;
    ds_task_q9_last_move_direction = 0;
    ds_task_q9_last_move_ms = 0U;
    ds_task_q9_move_status = HAL_OK;
    ds_task_q9_imu_zero_pending = 1U;
    ds_task_q9_imu_zero_x = 0.0f;
    ds_task_q9_imu_zero_y = 0.0f;
    ds_task_q9_imu_zero_z = 0.0f;
    ds_task_q9_angle_x = 0.0f;
    ds_task_q9_angle_y = 0.0f;
    ds_task_q9_angle_z = 0.0f;
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
            } else if (ds_task.selected_question == 2U) {
                DS_Task_StartQuestion2();
            } else if (ds_task.selected_question == 9U) {
                DS_Task_StartQuestion9();
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

    case DS_TASK_RUNNING_Q2:
        ds_task.elapsed_ms = now - ds_task.start_ms;

        if (key2_clicked != 0U) {
            DS_Task_FinishQuestion2(now);
            break;
        }

        BalanceControl_Update();
        DS_Task_UpdateQuestion2Display(now);
        break;

    case DS_TASK_RUNNING_Q9:
        ds_task.elapsed_ms = now - ds_task.start_ms;

        if (key2_clicked != 0U) {
            DS_Task_FinishQuestion9();
            break;
        }

        MotorPositionMonitor_Update();
        DS_Task_UpdateQuestion9Motion(now);
        DS_Task_UpdateQuestion9Telemetry();
        DS_Task_UpdateQuestion9Display(now);
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
    if (ds_task.state == DS_TASK_RUNNING_Q2) {
        BalanceControl_Stop();
        BallVision_StopStream();
    } else if (ds_task.state == DS_TASK_RUNNING_Q1) {
        LineFollow_Stop();
    } else if (ds_task.state == DS_TASK_RUNNING_Q9) {
        Question9Telemetry_Stop();
        MotorPositionMonitor_Stop();
        ds_task_q9_move_status = DS_BalanceStop();
    }
    if (ds_task.state == DS_TASK_RUNNING_Q1 ||
        ds_task.state == DS_TASK_RUNNING_Q2 ||
        ds_task.state == DS_TASK_RUNNING_Q9) {
        ds_task.elapsed_ms = HAL_GetTick() - ds_task.start_ms;
    }
    ds_task.state = DS_TASK_FINISHED;
}
