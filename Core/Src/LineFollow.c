#include "LineFollow.h"

#include "DS.h"

/*
 * Sensor array index follows IR1 through IR8. The current signs were selected
 * to match the tested chassis steering direction: IR1 is positive and IR8 is
 * negative. The midpoint between IR4 and IR5 is zero.
 */
static const int16_t line_sensor_weights[DS_IR_SENSOR_COUNT] = {
    22000, 13000, 1600, 500,
      -500,  -1600,  -13000, -22000
};

LineFollowConfig line_follow_config = {
    .base_speed = 100,
    .motor_slope = DS_MOTOR_DEFAULT_SLOPE,
    .control_period_ms = 20U,
    .kp = 0.05f,
    .kd = 0.00f,
    .max_correction = 300,
    .max_d_correction = 80,
    .curve_slowdown = 120,
    .correction_slew_per_update = 0,
    .command_limit = 1000,
    .lost_speed = 0,
    .lost_correction = 0,
    .lap_stop_enabled = 1U,
    .lap_target_yaw_deg = 350.0f,
    .lap_max_yaw_step_deg = 45.0f,
    .lap_min_time_ms = 3000U,
    .lap_confirm_frames = 3U
};

LineFollowState line_follow_state;

static int32_t line_follow_last_visible_error;
static int32_t line_follow_previous_error;
static int32_t line_follow_smoothed_correction;
static uint8_t line_follow_has_previous_error;
static uint32_t line_follow_last_update_ms;
static float line_follow_previous_yaw_deg;
static uint32_t line_follow_last_yaw_frame_count;
static uint8_t line_follow_has_yaw_reference;
static uint32_t line_follow_lap_start_ms;

static int32_t LineFollow_Clamp(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t LineFollow_Abs(int32_t value)
{
    if (value >= 0) {
        return value;
    }
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return -value;
}

static float LineFollow_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float LineFollow_NormalizeYawDelta(float delta_deg)
{
    while (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    }
    while (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

static int32_t LineFollow_ApplySlew(int32_t current,
                                    int32_t target,
                                    int32_t maximum_step)
{
    if (maximum_step <= 0) {
        return target;
    }

    if (target > current + maximum_step) {
        return current + maximum_step;
    }
    if (target < current - maximum_step) {
        return current - maximum_step;
    }
    return target;
}

static int32_t LineFollow_CalculateError(uint8_t sensor_bits,
                                         uint8_t *active_count)
{
    int32_t weighted_sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < DS_IR_SENSOR_COUNT; index++) {
        if ((sensor_bits & (uint8_t)(1U << index)) != 0U) {
            weighted_sum += line_sensor_weights[index];
            count++;
        }
    }

    *active_count = count;
    if (count == 0U) {
        return line_follow_last_visible_error;
    }

    return weighted_sum / (int32_t)count;
}

static void LineFollow_UpdateLapDetection(uint32_t now)
{
    const DS_State *ds = DS_GetState();
    float current_yaw;
    float yaw_delta;
    uint8_t required_confirm_frames;

    line_follow_state.yaw_valid = ds->yaw_valid;
    line_follow_state.yaw_deg = ds->yaw_deg;
    line_follow_state.yaw_frame_count = ds->yaw_frame_count;

    if (line_follow_config.lap_stop_enabled == 0U ||
        line_follow_state.lap_complete != 0U) {
        return;
    }

    if (ds->yaw_valid == 0U) {
        line_follow_has_yaw_reference = 0U;
        line_follow_state.yaw_delta_deg = 0.0f;
        line_follow_state.lap_confirm_count = 0U;
        line_follow_last_yaw_frame_count = ds->yaw_frame_count;
        return;
    }

    if (ds->yaw_frame_count == line_follow_last_yaw_frame_count) {
        return;
    }
    line_follow_last_yaw_frame_count = ds->yaw_frame_count;
    current_yaw = ds->yaw_deg;

    if (line_follow_has_yaw_reference == 0U) {
        line_follow_previous_yaw_deg = current_yaw;
        line_follow_has_yaw_reference = 1U;
        line_follow_state.yaw_delta_deg = 0.0f;
        return;
    }

    yaw_delta = LineFollow_NormalizeYawDelta(
        current_yaw - line_follow_previous_yaw_deg);
    line_follow_previous_yaw_deg = current_yaw;
    line_follow_state.yaw_delta_deg = yaw_delta;

    if (line_follow_config.lap_max_yaw_step_deg > 0.0f &&
        LineFollow_AbsFloat(yaw_delta) >
        line_follow_config.lap_max_yaw_step_deg) {
        line_follow_state.rejected_yaw_steps++;
        line_follow_state.lap_confirm_count = 0U;
        return;
    }

    line_follow_state.accumulated_yaw_deg += yaw_delta;

    if ((uint32_t)(now - line_follow_lap_start_ms) <
        line_follow_config.lap_min_time_ms ||
        LineFollow_AbsFloat(line_follow_state.accumulated_yaw_deg) <
        line_follow_config.lap_target_yaw_deg) {
        line_follow_state.lap_confirm_count = 0U;
        return;
    }

    required_confirm_frames = line_follow_config.lap_confirm_frames;
    if (required_confirm_frames == 0U) {
        required_confirm_frames = 1U;
    }

    if (line_follow_state.lap_confirm_count < UINT8_MAX) {
        line_follow_state.lap_confirm_count++;
    }
    if (line_follow_state.lap_confirm_count >= required_confirm_frames) {
        line_follow_state.lap_complete = 1U;
    }
}

void LineFollow_Init(void)
{
    line_follow_state.enabled = 0U;
    line_follow_state.sensor_bits = 0U;
    line_follow_state.line_lost = 1U;
    line_follow_state.error = 0;
    line_follow_state.error_delta = 0;
    line_follow_state.d_correction = 0;
    line_follow_state.correction = 0;
    line_follow_state.left_command = 0;
    line_follow_state.right_command = 0;
    line_follow_state.yaw_valid = 0U;
    line_follow_state.lap_complete = 0U;
    line_follow_state.lap_confirm_count = 0U;
    line_follow_state.yaw_deg = 0.0f;
    line_follow_state.yaw_delta_deg = 0.0f;
    line_follow_state.accumulated_yaw_deg = 0.0f;
    line_follow_state.yaw_frame_count = 0U;
    line_follow_state.rejected_yaw_steps = 0U;
    line_follow_state.last_motor_status = HAL_OK;
    line_follow_state.update_count = 0U;
    line_follow_last_visible_error = 0;
    line_follow_previous_error = 0;
    line_follow_smoothed_correction = 0;
    line_follow_has_previous_error = 0U;
    line_follow_last_update_ms = HAL_GetTick();
    line_follow_previous_yaw_deg = 0.0f;
    line_follow_last_yaw_frame_count = 0U;
    line_follow_has_yaw_reference = 0U;
    line_follow_lap_start_ms = 0U;
}

void LineFollow_Start(void)
{
    line_follow_state.enabled = 1U;
    line_follow_state.sensor_bits = DS_GetState()->ir_active_bits;
    line_follow_state.line_lost = 0U;
    line_follow_state.error = 0;
    line_follow_state.error_delta = 0;
    line_follow_state.d_correction = 0;
    line_follow_state.correction = 0;
    line_follow_state.left_command = 0;
    line_follow_state.right_command = 0;
    line_follow_state.yaw_valid = DS_GetState()->yaw_valid;
    line_follow_state.lap_complete = 0U;
    line_follow_state.lap_confirm_count = 0U;
    line_follow_state.yaw_deg = DS_GetState()->yaw_deg;
    line_follow_state.yaw_delta_deg = 0.0f;
    line_follow_state.accumulated_yaw_deg = 0.0f;
    line_follow_state.yaw_frame_count = DS_GetState()->yaw_frame_count;
    line_follow_state.rejected_yaw_steps = 0U;
    line_follow_state.last_motor_status = HAL_OK;
    line_follow_state.update_count = 0U;
    line_follow_last_visible_error = 0;
    line_follow_previous_error = 0;
    line_follow_smoothed_correction = 0;
    line_follow_has_previous_error = 0U;
    line_follow_previous_yaw_deg = DS_GetState()->yaw_deg;
    line_follow_last_yaw_frame_count = DS_GetState()->yaw_frame_count;
    line_follow_has_yaw_reference =
        (DS_GetState()->yaw_valid != 0U) ? 1U : 0U;
    line_follow_lap_start_ms = HAL_GetTick();
    line_follow_last_update_ms = HAL_GetTick() -
                                 line_follow_config.control_period_ms;
}

void LineFollow_Update(void)
{
    uint32_t now;
    uint8_t active_count;
    int32_t error;
    int32_t error_delta;
    int32_t d_correction;
    int32_t target_correction;
    int32_t running_speed;
    int32_t slowdown;
    int32_t left_command;
    int32_t right_command;

    if (line_follow_state.enabled == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - line_follow_last_update_ms) <
        line_follow_config.control_period_ms) {
        return;
    }
    line_follow_last_update_ms = now;

    line_follow_state.sensor_bits = DS_GetState()->ir_active_bits;
    error = LineFollow_CalculateError(line_follow_state.sensor_bits,
                                      &active_count);

    if (active_count == 0U) {
        line_follow_state.line_lost = 1U;
        error_delta = 0;
        d_correction = 0;
        line_follow_has_previous_error = 0U;
        if (line_follow_last_visible_error >= 0) {
            error = 3500;
            target_correction = line_follow_config.lost_correction;
        } else {
            error = -3500;
            target_correction = -line_follow_config.lost_correction;
        }
        running_speed = line_follow_config.lost_speed;
    } else {
        line_follow_state.line_lost = 0U;
        line_follow_last_visible_error = error;

        if (line_follow_has_previous_error != 0U) {
            error_delta = error - line_follow_previous_error;
            d_correction =
                (int32_t)(line_follow_config.kd * (float)error_delta);
            d_correction = LineFollow_Clamp(
                d_correction,
                -line_follow_config.max_d_correction,
                line_follow_config.max_d_correction);
        } else {
            error_delta = 0;
            d_correction = 0;
            line_follow_has_previous_error = 1U;
        }
        line_follow_previous_error = error;

        target_correction =
            (int32_t)(line_follow_config.kp * (float)error) +
            d_correction;
        target_correction = LineFollow_Clamp(
            target_correction,
            -line_follow_config.max_correction,
            line_follow_config.max_correction);

        slowdown = 0;
        if (line_follow_config.max_correction > 0) {
            slowdown = LineFollow_Abs(target_correction) *
                       line_follow_config.curve_slowdown /
                       line_follow_config.max_correction;
        }
        running_speed = line_follow_config.base_speed - slowdown;
        if (running_speed < 0) {
            running_speed = 0;
        }
    }

    line_follow_smoothed_correction = LineFollow_ApplySlew(
        line_follow_smoothed_correction,
        target_correction,
        line_follow_config.correction_slew_per_update);

    /*
     * Positive correction speeds up the left wheel and slows down the right
     * wheel. Sensor-weight signs above map the detected side to this steering.
     *
     * Extension point: add filtered steering or IMU yaw correction to the
     * correction term here without changing the task state machine.
     */
    left_command = running_speed + line_follow_smoothed_correction;
    right_command = running_speed - line_follow_smoothed_correction;
    left_command = LineFollow_Clamp(left_command,
                                    -line_follow_config.command_limit,
                                    line_follow_config.command_limit);
    right_command = LineFollow_Clamp(right_command,
                                     -line_follow_config.command_limit,
                                     line_follow_config.command_limit);

    line_follow_state.error = error;
    line_follow_state.error_delta = error_delta;
    line_follow_state.d_correction = d_correction;
    line_follow_state.correction = line_follow_smoothed_correction;
    line_follow_state.left_command = left_command;
    line_follow_state.right_command = right_command;
    line_follow_state.last_motor_status = DS_ChassisSetSpeed(
        left_command,
        right_command,
        line_follow_config.motor_slope);
    line_follow_state.update_count++;
    LineFollow_UpdateLapDetection(now);
}

void LineFollow_Stop(void)
{
    line_follow_state.enabled = 0U;
    line_follow_state.left_command = 0;
    line_follow_state.right_command = 0;
    line_follow_smoothed_correction = 0;
    DS_ChassisStop();
}

uint8_t LineFollow_IsLapComplete(void)
{
    return line_follow_state.lap_complete;
}
