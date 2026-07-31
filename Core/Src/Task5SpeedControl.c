#include "Task5SpeedControl.h"

#include "BallVision.h"
#include "DS.h"

Task5SpeedControlConfig task5_speed_control_config = {
    .kp = 0.008f,
    .ki = 0.0f,
    .kd = 0.00f,
    .integral_limit = 1000.0f,
    .velocity_feedforward_gain = 0.01f,
    .feedforward_limit = 20.0f,
    .speed_limit = 8.0f,
    .error_deadband_px = 20.0f,
    .control_period_ms = 20U,
    .motor_slope = 0U
};

Task5SpeedControlState task5_speed_control_state;

static uint32_t task5_last_update_ms;
static uint32_t task5_last_pid_ms;
static uint32_t task5_last_vision_frame;
static float task5_previous_error;
static uint8_t task5_has_previous_error;
static int32_t task5_last_sent_command;

static float Task5SpeedControl_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Task5SpeedControl_ClampFloat(float value,
                                          float minimum,
                                          float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t Task5SpeedControl_RoundToInt(float value)
{
    return (value < 0.0f) ?
           (int32_t)(value - 0.5f) :
           (int32_t)(value + 0.5f);
}

static void Task5SpeedControl_ResetPid(void)
{
    task5_speed_control_state.p_term = 0.0f;
    task5_speed_control_state.i_term = 0.0f;
    task5_speed_control_state.d_term = 0.0f;
    task5_speed_control_state.feedforward_term = 0.0f;
    task5_speed_control_state.integral = 0.0f;
    task5_speed_control_state.output = 0.0f;
    task5_previous_error = 0.0f;
    task5_has_previous_error = 0U;
    task5_last_pid_ms = 0U;
}

static void Task5SpeedControl_SendFeedback(uint32_t vision_frame,
                                           uint32_t vision_last_rx_ms,
                                           int32_t motor_command,
                                           HAL_StatusTypeDef motor_status)
{
    uint32_t now = HAL_GetTick();

    (void)BallVision_SendFeedback(
        vision_frame,
        (uint32_t)(now - vision_last_rx_ms),
        task5_speed_control_state.camera_error,
        task5_speed_control_state.ball_velocity,
        task5_speed_control_state.camera_error,
        task5_speed_control_state.p_term,
        task5_speed_control_state.i_term,
        task5_speed_control_state.d_term,
        motor_command,
        motor_status);
}

static HAL_StatusTypeDef Task5SpeedControl_StopMotor(void)
{
    HAL_StatusTypeDef status = DS_BalanceStop();

    if (status == HAL_OK) {
        task5_last_sent_command = 0;
    }
    task5_speed_control_state.motor_command = 0;
    task5_speed_control_state.last_motor_status = status;
    return status;
}

static float Task5SpeedControl_GetDt(uint32_t now)
{
    uint32_t elapsed_ms;

    if (task5_last_pid_ms == 0U) {
        elapsed_ms = task5_speed_control_config.control_period_ms;
    } else {
        elapsed_ms = (uint32_t)(now - task5_last_pid_ms);
    }
    task5_last_pid_ms = now;

    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    } else if (elapsed_ms > DS_BALL_VISION_TIMEOUT_MS) {
        elapsed_ms = DS_BALL_VISION_TIMEOUT_MS;
    }

    return (float)elapsed_ms * 0.001f;
}

static void Task5SpeedControl_ProcessFrame(uint32_t now,
                                           uint32_t vision_frame,
                                           uint32_t vision_last_rx_ms,
                                           float camera_error,
                                           float ball_velocity)
{
    float dt;
    float derivative = 0.0f;
    float feedforward_limit;
    float integral_limit;
    float speed_limit;
    float output;
    int32_t motor_command;
    HAL_StatusTypeDef motor_status = HAL_OK;
    uint8_t command_attempted = 0U;

    task5_speed_control_state.vision_valid = 1U;
    task5_speed_control_state.camera_error = camera_error;
    task5_speed_control_state.ball_velocity = ball_velocity;

    if (Task5SpeedControl_AbsFloat(camera_error) <=
        Task5SpeedControl_AbsFloat(
            task5_speed_control_config.error_deadband_px)) {
        task5_speed_control_state.in_deadband = 1U;
        Task5SpeedControl_ResetPid();
        motor_command = 0;

        if (task5_last_sent_command != 0) {
            motor_status = Task5SpeedControl_StopMotor();
            command_attempted = 1U;
        } else {
            task5_speed_control_state.motor_command = 0;
            task5_speed_control_state.last_motor_status = HAL_OK;
        }
    } else {
        task5_speed_control_state.in_deadband = 0U;
        dt = Task5SpeedControl_GetDt(now);

        if (Task5SpeedControl_AbsFloat(
                task5_speed_control_config.ki) > 0.0000001f) {
            integral_limit = Task5SpeedControl_AbsFloat(
                task5_speed_control_config.integral_limit);
            task5_speed_control_state.integral += camera_error * dt;
            task5_speed_control_state.integral =
                Task5SpeedControl_ClampFloat(
                    task5_speed_control_state.integral,
                    -integral_limit,
                    integral_limit);
        } else {
            task5_speed_control_state.integral = 0.0f;
        }

        if (task5_has_previous_error != 0U) {
            derivative = (camera_error - task5_previous_error) / dt;
        }
        task5_previous_error = camera_error;
        task5_has_previous_error = 1U;

        task5_speed_control_state.p_term =
            task5_speed_control_config.kp * camera_error;
        task5_speed_control_state.i_term =
            task5_speed_control_config.ki *
            task5_speed_control_state.integral;
        task5_speed_control_state.d_term =
            task5_speed_control_config.kd * derivative;
        feedforward_limit = Task5SpeedControl_AbsFloat(
            task5_speed_control_config.feedforward_limit);
        task5_speed_control_state.feedforward_term =
            Task5SpeedControl_ClampFloat(
                task5_speed_control_config.velocity_feedforward_gain *
                    ball_velocity,
                -feedforward_limit,
                feedforward_limit);

        speed_limit = Task5SpeedControl_AbsFloat(
            task5_speed_control_config.speed_limit);
        output = task5_speed_control_state.p_term +
                 task5_speed_control_state.i_term +
                 task5_speed_control_state.d_term +
                 task5_speed_control_state.feedforward_term;
        output = Task5SpeedControl_ClampFloat(output,
                                               -speed_limit,
                                               speed_limit);
        task5_speed_control_state.output = output;
        motor_command = Task5SpeedControl_RoundToInt(output);
        task5_speed_control_state.motor_command = motor_command;

        if (motor_command == 0) {
            if (task5_last_sent_command != 0) {
                motor_status = Task5SpeedControl_StopMotor();
                command_attempted = 1U;
            }
        } else {
            motor_status = DS_BalanceSetSpeed(
                motor_command,
                task5_speed_control_config.motor_slope);
            command_attempted = 1U;
            if (motor_status == HAL_OK) {
                task5_last_sent_command = motor_command;
            }
        }
        task5_speed_control_state.last_motor_status = motor_status;
    }

    if (command_attempted != 0U) {
        Task5SpeedControl_SendFeedback(
            vision_frame,
            vision_last_rx_ms,
            motor_command,
            motor_status);
    }
    task5_speed_control_state.update_count++;
}

void Task5SpeedControl_Init(void)
{
    task5_speed_control_state.enabled = 0U;
    task5_speed_control_state.vision_valid = 0U;
    task5_speed_control_state.in_deadband = 1U;
    task5_speed_control_state.camera_error = 0.0f;
    task5_speed_control_state.ball_velocity = 0.0f;
    task5_speed_control_state.motor_command = 0;
    task5_speed_control_state.update_count = 0U;
    task5_speed_control_state.last_motor_status = HAL_OK;
    Task5SpeedControl_ResetPid();

    task5_last_update_ms = HAL_GetTick();
    task5_last_vision_frame = 0xFFFFFFFFUL;
    task5_last_sent_command = 0;
}

void Task5SpeedControl_Start(void)
{
    (void)DS_BalanceStop();
    Task5SpeedControl_ResetPid();

    task5_speed_control_state.enabled = 1U;
    task5_speed_control_state.vision_valid = 0U;
    task5_speed_control_state.in_deadband = 1U;
    task5_speed_control_state.camera_error = 0.0f;
    task5_speed_control_state.ball_velocity = 0.0f;
    task5_speed_control_state.motor_command = 0;
    task5_speed_control_state.update_count = 0U;
    task5_speed_control_state.last_motor_status = HAL_BUSY;

    task5_last_update_ms =
        HAL_GetTick() - task5_speed_control_config.control_period_ms;
    task5_last_vision_frame = 0xFFFFFFFFUL;
    task5_last_sent_command = 0;
}

void Task5SpeedControl_Update(void)
{
    const DS_State *ds;
    uint32_t now;
    uint32_t primask;
    uint32_t vision_frame;
    uint32_t vision_last_rx_ms;
    float camera_error;
    float ball_velocity;
    uint8_t vision_valid;

    if (task5_speed_control_state.enabled == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - task5_last_update_ms) <
        task5_speed_control_config.control_period_ms) {
        return;
    }
    task5_last_update_ms = now;
    ds = DS_GetState();

    primask = __get_PRIMASK();
    __disable_irq();
    camera_error = ds->ball_position;
    ball_velocity = ds->ball_velocity;
    vision_frame = ds->ball_vision_frame_count;
    vision_last_rx_ms = ds->ball_vision_last_rx_ms;
    vision_valid = ds->ball_vision_valid;
    if (primask == 0U) {
        __enable_irq();
    }

    if (vision_valid != 0U &&
        (uint32_t)(now - vision_last_rx_ms) <=
            DS_BALL_VISION_TIMEOUT_MS) {
        if (vision_frame != task5_last_vision_frame) {
            task5_last_vision_frame = vision_frame;
            Task5SpeedControl_ProcessFrame(now,
                                           vision_frame,
                                           vision_last_rx_ms,
                                           camera_error,
                                           ball_velocity);
        }
        return;
    }

    task5_speed_control_state.vision_valid = 0U;
    task5_speed_control_state.in_deadband = 1U;
    Task5SpeedControl_ResetPid();
    if (task5_last_sent_command != 0) {
        HAL_StatusTypeDef motor_status = Task5SpeedControl_StopMotor();

        Task5SpeedControl_SendFeedback(
            vision_frame,
            vision_last_rx_ms,
            0,
            motor_status);
    } else {
        task5_speed_control_state.motor_command = 0;
    }
}

void Task5SpeedControl_Stop(void)
{
    const DS_State *ds = DS_GetState();
    uint32_t primask;
    uint32_t vision_frame;
    uint32_t vision_last_rx_ms;
    HAL_StatusTypeDef motor_status;

    primask = __get_PRIMASK();
    __disable_irq();
    vision_frame = ds->ball_vision_frame_count;
    vision_last_rx_ms = ds->ball_vision_last_rx_ms;
    if (primask == 0U) {
        __enable_irq();
    }

    motor_status = Task5SpeedControl_StopMotor();
    Task5SpeedControl_SendFeedback(
        vision_frame,
        vision_last_rx_ms,
        0,
        motor_status);

    task5_speed_control_state.enabled = 0U;
    task5_speed_control_state.vision_valid = 0U;
    task5_speed_control_state.in_deadband = 1U;
    Task5SpeedControl_ResetPid();
}
