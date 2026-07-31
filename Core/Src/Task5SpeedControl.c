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
    .static_feedforward_speed = 2.0f,
    .speed_limit = 8.0f,
    .error_deadband_px = 20.0f,
    .deadband_hysteresis_px = 5.0f,
    .velocity_estimator_alpha = 0.75f,
    .camera_velocity_weight = 0.35f,
    .velocity_estimate_limit_px_s = 1200.0f,
    .sensor_delay_ms = 60.0f,
    .prediction_time_limit_ms = 140.0f,
    .prediction_offset_limit_px = 60.0f,
    .control_period_ms = 10U,
    .motor_slope = 0U
};

Task5SpeedControlState task5_speed_control_state;

static uint32_t task5_last_update_ms;
static uint32_t task5_last_pid_ms;
static uint32_t task5_last_vision_frame;
static uint32_t task5_last_measurement_rx_ms;
static float task5_last_measurement_error;
static uint8_t task5_has_measurement;
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

static void Task5SpeedControl_ResetControl(void)
{
    task5_speed_control_state.p_term = 0.0f;
    task5_speed_control_state.i_term = 0.0f;
    task5_speed_control_state.d_term = 0.0f;
    task5_speed_control_state.velocity_feedforward_term = 0.0f;
    task5_speed_control_state.static_feedforward_term = 0.0f;
    task5_speed_control_state.feedforward_term = 0.0f;
    task5_speed_control_state.integral = 0.0f;
    task5_speed_control_state.output = 0.0f;
    task5_last_pid_ms = 0U;
}

static void Task5SpeedControl_ResetObserver(void)
{
    task5_speed_control_state.estimated_velocity = 0.0f;
    task5_speed_control_state.predicted_error = 0.0f;
    task5_speed_control_state.prediction_offset = 0.0f;
    task5_speed_control_state.vision_age_ms = 0U;
    task5_last_measurement_rx_ms = 0U;
    task5_last_measurement_error = 0.0f;
    task5_has_measurement = 0U;
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
        task5_speed_control_state.predicted_error,
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

static void Task5SpeedControl_UpdateObserver(uint32_t vision_last_rx_ms,
                                             float camera_error,
                                             float camera_velocity)
{
    float alpha = Task5SpeedControl_ClampFloat(
        task5_speed_control_config.velocity_estimator_alpha,
        0.0f,
        1.0f);
    float camera_weight = Task5SpeedControl_ClampFloat(
        task5_speed_control_config.camera_velocity_weight,
        0.0f,
        1.0f);
    float velocity_limit = Task5SpeedControl_AbsFloat(
        task5_speed_control_config.velocity_estimate_limit_px_s);
    float fused_velocity = camera_velocity;

    if (task5_has_measurement != 0U) {
        uint32_t elapsed_ms =
            (uint32_t)(vision_last_rx_ms - task5_last_measurement_rx_ms);

        if (elapsed_ms > 0U && elapsed_ms <= DS_BALL_VISION_TIMEOUT_MS) {
            float raw_velocity =
                (camera_error - task5_last_measurement_error) /
                ((float)elapsed_ms * 0.001f);

            if (velocity_limit > 0.0f) {
                raw_velocity = Task5SpeedControl_ClampFloat(
                    raw_velocity,
                    -velocity_limit,
                    velocity_limit);
            }
            fused_velocity =
                camera_weight * camera_velocity +
                (1.0f - camera_weight) * raw_velocity;
        }

        task5_speed_control_state.estimated_velocity +=
            alpha *
            (fused_velocity -
             task5_speed_control_state.estimated_velocity);
    } else {
        task5_speed_control_state.estimated_velocity = fused_velocity;
        task5_has_measurement = 1U;
    }

    if (velocity_limit > 0.0f) {
        task5_speed_control_state.estimated_velocity =
            Task5SpeedControl_ClampFloat(
                task5_speed_control_state.estimated_velocity,
                -velocity_limit,
                velocity_limit);
    }

    task5_last_measurement_rx_ms = vision_last_rx_ms;
    task5_last_measurement_error = camera_error;
    task5_speed_control_state.camera_error = camera_error;
    task5_speed_control_state.ball_velocity = camera_velocity;
    task5_speed_control_state.measurement_update_count++;
}

static void Task5SpeedControl_UpdatePrediction(uint32_t now,
                                               uint32_t vision_last_rx_ms)
{
    float prediction_ms;
    float prediction_time_limit = Task5SpeedControl_AbsFloat(
        task5_speed_control_config.prediction_time_limit_ms);
    float prediction_offset_limit = Task5SpeedControl_AbsFloat(
        task5_speed_control_config.prediction_offset_limit_px);
    float prediction_offset;

    task5_speed_control_state.vision_age_ms =
        (uint32_t)(now - vision_last_rx_ms);
    prediction_ms = Task5SpeedControl_AbsFloat(
        task5_speed_control_config.sensor_delay_ms) +
        (float)task5_speed_control_state.vision_age_ms;
    if (prediction_time_limit > 0.0f) {
        prediction_ms = Task5SpeedControl_ClampFloat(
            prediction_ms,
            0.0f,
            prediction_time_limit);
    }

    prediction_offset =
        task5_speed_control_state.estimated_velocity *
        prediction_ms * 0.001f;
    if (prediction_offset_limit > 0.0f) {
        prediction_offset = Task5SpeedControl_ClampFloat(
            prediction_offset,
            -prediction_offset_limit,
            prediction_offset_limit);
    }

    task5_speed_control_state.prediction_offset = prediction_offset;
    task5_speed_control_state.predicted_error =
        task5_speed_control_state.camera_error + prediction_offset;
}

static void Task5SpeedControl_RunController(uint32_t now,
                                            uint32_t vision_frame,
                                            uint32_t vision_last_rx_ms)
{
    float dt;
    float deadband;
    float deadband_hysteresis;
    float active_deadband;
    float feedforward_limit;
    float integral_limit;
    float speed_limit;
    float output;
    int32_t motor_command;
    HAL_StatusTypeDef motor_status = HAL_OK;
    uint8_t command_attempted = 0U;

    Task5SpeedControl_UpdatePrediction(now, vision_last_rx_ms);
    deadband = Task5SpeedControl_AbsFloat(
        task5_speed_control_config.error_deadband_px);
    deadband_hysteresis = Task5SpeedControl_AbsFloat(
        task5_speed_control_config.deadband_hysteresis_px);
    if (deadband_hysteresis > deadband) {
        deadband_hysteresis = deadband;
    }
    active_deadband =
        (task5_speed_control_state.in_deadband != 0U) ?
        deadband :
        deadband - deadband_hysteresis;

    if (Task5SpeedControl_AbsFloat(
            task5_speed_control_state.predicted_error) <=
        active_deadband) {
        task5_speed_control_state.in_deadband = 1U;
        Task5SpeedControl_ResetControl();
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
            task5_speed_control_state.integral +=
                task5_speed_control_state.predicted_error * dt;
            task5_speed_control_state.integral =
                Task5SpeedControl_ClampFloat(
                    task5_speed_control_state.integral,
                    -integral_limit,
                    integral_limit);
        } else {
            task5_speed_control_state.integral = 0.0f;
        }

        task5_speed_control_state.p_term =
            task5_speed_control_config.kp *
            task5_speed_control_state.predicted_error;
        task5_speed_control_state.i_term =
            task5_speed_control_config.ki *
            task5_speed_control_state.integral;
        task5_speed_control_state.d_term =
            task5_speed_control_config.kd *
            task5_speed_control_state.estimated_velocity;

        task5_speed_control_state.velocity_feedforward_term =
            task5_speed_control_config.velocity_feedforward_gain *
            task5_speed_control_state.estimated_velocity;
        task5_speed_control_state.static_feedforward_term =
            (task5_speed_control_state.predicted_error < 0.0f) ?
            -Task5SpeedControl_AbsFloat(
                task5_speed_control_config.static_feedforward_speed) :
            Task5SpeedControl_AbsFloat(
                task5_speed_control_config.static_feedforward_speed);
        feedforward_limit = Task5SpeedControl_AbsFloat(
            task5_speed_control_config.feedforward_limit);
        task5_speed_control_state.feedforward_term =
            Task5SpeedControl_ClampFloat(
                task5_speed_control_state.velocity_feedforward_term +
                task5_speed_control_state.static_feedforward_term,
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
        } else if (motor_command != task5_last_sent_command) {
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
    task5_speed_control_state.measurement_update_count = 0U;
    task5_speed_control_state.last_motor_status = HAL_OK;
    Task5SpeedControl_ResetControl();
    Task5SpeedControl_ResetObserver();

    task5_last_update_ms = HAL_GetTick();
    task5_last_vision_frame = 0xFFFFFFFFUL;
    task5_last_sent_command = 0;
}

void Task5SpeedControl_Start(void)
{
    (void)DS_BalanceStop();
    Task5SpeedControl_ResetControl();
    Task5SpeedControl_ResetObserver();

    task5_speed_control_state.enabled = 1U;
    task5_speed_control_state.vision_valid = 0U;
    task5_speed_control_state.in_deadband = 1U;
    task5_speed_control_state.camera_error = 0.0f;
    task5_speed_control_state.ball_velocity = 0.0f;
    task5_speed_control_state.motor_command = 0;
    task5_speed_control_state.update_count = 0U;
    task5_speed_control_state.measurement_update_count = 0U;
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
        task5_speed_control_state.vision_valid = 1U;
        if (vision_frame != task5_last_vision_frame) {
            task5_last_vision_frame = vision_frame;
            Task5SpeedControl_UpdateObserver(vision_last_rx_ms,
                                             camera_error,
                                             ball_velocity);
        }
        Task5SpeedControl_RunController(now,
                                        vision_frame,
                                        vision_last_rx_ms);
        return;
    }

    task5_speed_control_state.vision_valid = 0U;
    task5_speed_control_state.in_deadband = 1U;
    Task5SpeedControl_ResetControl();
    Task5SpeedControl_ResetObserver();
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
    Task5SpeedControl_ResetControl();
    Task5SpeedControl_ResetObserver();
}
