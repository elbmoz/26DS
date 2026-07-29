#include "LineFollow.h"

#include "DS.h"

/*
 * Sensor order is left to right. The midpoint between IR4 and IR5 is zero.
 * A positive error means the line is to the right of the chassis center.
 */
static const int16_t line_sensor_weights[DS_IR_SENSOR_COUNT] = {
    -3500, -2500, -1500, -500,
      500,  1500,  2500, 3500
};

LineFollowConfig line_follow_config = {
    .base_speed = 100,
    .motor_slope = DS_MOTOR_DEFAULT_SLOPE,
    .control_period_ms = 20U,
    .kp = 0.08f,
    .max_correction = 300,
    .curve_slowdown = 120,
    .correction_slew_per_update = 20,
    .command_limit = 1000,
    .lost_speed = 200,
    .lost_correction = 260
};

LineFollowState line_follow_state;

static int32_t line_follow_last_visible_error;
static int32_t line_follow_smoothed_correction;
static uint32_t line_follow_last_update_ms;

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

void LineFollow_Init(void)
{
    line_follow_state.enabled = 0U;
    line_follow_state.sensor_bits = 0U;
    line_follow_state.line_lost = 1U;
    line_follow_state.error = 0;
    line_follow_state.correction = 0;
    line_follow_state.left_command = 0;
    line_follow_state.right_command = 0;
    line_follow_state.last_motor_status = HAL_OK;
    line_follow_state.update_count = 0U;
    line_follow_last_visible_error = 0;
    line_follow_smoothed_correction = 0;
    line_follow_last_update_ms = HAL_GetTick();
}

void LineFollow_Start(void)
{
    line_follow_state.enabled = 1U;
    line_follow_state.sensor_bits = DS_GetState()->ir_active_bits;
    line_follow_state.line_lost = 0U;
    line_follow_state.error = 0;
    line_follow_state.correction = 0;
    line_follow_state.left_command = 0;
    line_follow_state.right_command = 0;
    line_follow_state.last_motor_status = HAL_OK;
    line_follow_state.update_count = 0U;
    line_follow_last_visible_error = 0;
    line_follow_smoothed_correction = 0;
    line_follow_last_update_ms = HAL_GetTick() -
                                 line_follow_config.control_period_ms;
}

void LineFollow_Update(void)
{
    uint32_t now;
    uint8_t active_count;
    int32_t error;
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
        target_correction = (int32_t)(line_follow_config.kp * (float)error);
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
     * Positive error means the line is on the right, so speed up the left
     * wheel and slow down the right wheel.
     *
     * Extension point: add filtered/D steering or IMU yaw correction to the
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
    line_follow_state.correction = line_follow_smoothed_correction;
    line_follow_state.left_command = left_command;
    line_follow_state.right_command = right_command;
    line_follow_state.last_motor_status = DS_ChassisSetSpeed(
        left_command,
        right_command,
        line_follow_config.motor_slope);
    line_follow_state.update_count++;
}

void LineFollow_Stop(void)
{
    line_follow_state.enabled = 0U;
    line_follow_state.left_command = 0;
    line_follow_state.right_command = 0;
    line_follow_smoothed_correction = 0;
    DS_ChassisStop();
}
