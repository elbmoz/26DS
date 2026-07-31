#include "BalanceControl.h"

#include "BallVision.h"
#include "DS.h"
#include "MotorPositionMonitor.h"

/*
 * The proven reference controller uses millimetres and millimetres/second.
 * These defaults apply the current 18.2 px/cm calibration so that its
 * 0.11/0.06/0.34 outer gains become degree-per-pixel gains. Mechanical zero,
 * linkage ratio and the RS485 speed conversion still require bench
 * calibration before unrestricted testing.
 */
BalanceControlConfig balance_control_config = {
    .outer_kp_deg_per_px = 0.06044f,
    .outer_ki_deg_per_px_s = 0.03297f,
    .outer_kd_deg_per_px_s = 0.18681f,
    .outer_integral_limit_px_s = 109.2f,
    .outer_angle_limit_deg = 7.2f,

    .hold_band_px = 18.2f,
    .fine_band_px = 3.64f,
    .fine_velocity_px_s = 18.2f,
    .soft_kp_scale = 0.55f,
    .soft_kd_scale = 0.75f,
    .soft_angle_limit_scale = 0.65f,
    .soft_ki_deg_per_px_s = 0.10989f,
    .fine_fast_kp_scale = 0.25f,
    .fine_fast_ki_scale = 0.50f,
    .fine_fast_angle_limit_scale = 0.40f,
    .hold_integral_decay = 0.70f,
    .fine_hold_inner_kp_scale = 0.60f,

    .damping_velocity_px_s = 50.96f,
    .damping_kp_scale = 0.55f,
    .damping_kd_scale = 1.80f,
    .damping_angle_limit_scale = 0.70f,
    .freeze_integral_velocity_px_s = 81.90f,
    .freeze_kp_scale = 0.70f,
    .freeze_kd_scale = 1.40f,
    .freeze_angle_limit_scale = 0.75f,
    .freeze_integral_decay = 0.90f,

    .motor_zero_angle_deg = 0.0f,
    .rod_angle_per_motor_degree = 1.0f,
    .rod_angle_limit_deg = 10.0f,
    .capture_motor_zero_on_start = 1U,

    /*
     * With a direct linkage and protocol unit 10 ~= 1 RPM, 9.2 speed units
     * per degree gives approximately the reference inner-loop response.
     */
    .angle_kp_speed_per_deg = 9.2f,
    .motor_speed_limit = 180.0f,
    .motor_speed_deadband = 1.0f,
    .motor_min_speed = 4.0f,
    .motor_slew_per_update = 16.0f,
    .motor_slope = 0U,
    .tilt_direction = -1,
    .motor_direction = 1,

    .control_period_ms = 20U,
    .motor_position_period_ms = 20U,
    .motor_position_timeout_ms = 60U,

    .stable_error_px = 18.2f,
    .stable_velocity_px_s = 25.0f,
    .stable_frames = 25U,

    .pixels_per_cm = 18.2f,
    .positive_5cm_target = 91.0f,
    .negative_5cm_target = -91.0f
};

BalanceControlState balance_control_state;

static uint32_t balance_control_last_update_ms;
static uint32_t balance_control_last_outer_ms;
static uint32_t balance_control_last_vision_frame;
static uint8_t balance_control_had_valid_vision;
static uint8_t balance_control_fine_level_hold;
static float balance_control_slew_output;
static int32_t balance_control_last_sent_command;

static float BalanceControl_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float BalanceControl_ClampFloat(float value,
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

static int32_t BalanceControl_RoundToInt(float value)
{
    return (value >= 0.0f) ?
           (int32_t)(value + 0.5f) :
           (int32_t)(value - 0.5f);
}

static int8_t BalanceControl_NonzeroDirection(int8_t direction)
{
    return (direction < 0) ? -1 : 1;
}

static HAL_StatusTypeDef BalanceControl_StopMotorSafely(void)
{
    /*
     * A stop frame must not be transmitted while an asynchronous 0x36 reply
     * is still being collected on the same USART1 handle.
     */
    DS_BalanceCancelPositionRequest();
    return DS_BalanceStop();
}

static void BalanceControl_ResetOuter(void)
{
    balance_control_state.position_error = 0.0f;
    balance_control_state.position_p_term = 0.0f;
    balance_control_state.position_i_term = 0.0f;
    balance_control_state.velocity_d_term = 0.0f;
    balance_control_state.outer_integral = 0.0f;
    balance_control_state.outer_output_deg = 0.0f;
    balance_control_state.target_rod_angle_deg = 0.0f;
    balance_control_state.stable = 0U;
    balance_control_state.stable_count = 0U;
    balance_control_fine_level_hold = 0U;
    balance_control_last_outer_ms = 0U;
}

static void BalanceControl_ResetInner(void)
{
    balance_control_state.motor_position_valid = 0U;
    balance_control_state.leveling = 1U;
    balance_control_state.motor_position = 0;
    balance_control_state.motor_angle_deg = 0.0f;
    balance_control_state.rod_angle_deg = 0.0f;
    balance_control_state.angle_error_deg = 0.0f;
    balance_control_state.desired_motor_speed = 0.0f;
    balance_control_state.motor_command = 0;
    balance_control_slew_output = 0.0f;
    balance_control_last_sent_command = 0;
}

static void BalanceControl_SendMotorFeedback(uint32_t vision_frame,
                                             uint32_t vision_last_rx_ms,
                                             float ball_position,
                                             float ball_velocity,
                                             int32_t motor_command,
                                             HAL_StatusTypeDef motor_status)
{
    uint32_t now = HAL_GetTick();

    (void)BallVision_SendFeedback(
        vision_frame,
        (uint32_t)(now - vision_last_rx_ms),
        ball_position,
        ball_velocity,
        balance_control_state.position_error,
        balance_control_state.position_p_term,
        balance_control_state.position_i_term,
        balance_control_state.velocity_d_term,
        motor_command,
        motor_status);
}

static float BalanceControl_GetOuterDt(uint32_t now)
{
    float dt;
    uint32_t elapsed_ms;

    if (balance_control_last_outer_ms == 0U) {
        elapsed_ms = balance_control_config.control_period_ms;
    } else {
        elapsed_ms = (uint32_t)(now - balance_control_last_outer_ms);
    }
    balance_control_last_outer_ms = now;

    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    } else if (elapsed_ms > 200U) {
        elapsed_ms = 200U;
    }

    dt = (float)elapsed_ms * 0.001f;
    return dt;
}

static void BalanceControl_UpdateStableState(float error,
                                             float velocity)
{
    if (BalanceControl_AbsFloat(error) <=
            balance_control_config.stable_error_px &&
        BalanceControl_AbsFloat(velocity) <=
            balance_control_config.stable_velocity_px_s) {
        if (balance_control_state.stable_count < UINT16_MAX) {
            balance_control_state.stable_count++;
        }
    } else {
        balance_control_state.stable_count = 0U;
        balance_control_state.stable = 0U;
    }

    if (balance_control_state.stable_count >=
        balance_control_config.stable_frames) {
        balance_control_state.stable = 1U;
    }
}

static void BalanceControl_UpdateOuter(uint32_t now,
                                       uint32_t vision_frame,
                                       float ball_position,
                                       float ball_velocity)
{
    float error;
    float abs_error;
    float abs_velocity;
    float kp;
    float ki;
    float kd;
    float angle_limit;
    float dt;
    float output;
    int8_t tilt_direction;

    if (vision_frame == balance_control_last_vision_frame) {
        return;
    }
    balance_control_last_vision_frame = vision_frame;

    error = balance_control_state.target_position - ball_position;
    abs_error = BalanceControl_AbsFloat(error);
    abs_velocity = BalanceControl_AbsFloat(ball_velocity);
    balance_control_state.position_error = error;
    BalanceControl_UpdateStableState(error, ball_velocity);

    kp = balance_control_config.outer_kp_deg_per_px;
    ki = balance_control_config.outer_ki_deg_per_px_s;
    kd = balance_control_config.outer_kd_deg_per_px_s;
    angle_limit =
        BalanceControl_AbsFloat(balance_control_config.outer_angle_limit_deg);
    balance_control_fine_level_hold = 0U;

    if (abs_error <= balance_control_config.hold_band_px) {
        if (abs_error > balance_control_config.fine_band_px) {
            kp *= balance_control_config.soft_kp_scale;
            kd *= balance_control_config.soft_kd_scale;
            ki = balance_control_config.soft_ki_deg_per_px_s;
            angle_limit *=
                balance_control_config.soft_angle_limit_scale;
        } else if (abs_velocity <=
                   balance_control_config.fine_velocity_px_s) {
            balance_control_state.outer_integral *=
                balance_control_config.hold_integral_decay;
            balance_control_state.position_p_term = 0.0f;
            balance_control_state.position_i_term = 0.0f;
            balance_control_state.velocity_d_term = 0.0f;
            balance_control_state.outer_output_deg = 0.0f;
            balance_control_state.target_rod_angle_deg = 0.0f;
            balance_control_state.leveling = 1U;
            balance_control_fine_level_hold = 1U;
            balance_control_state.outer_update_count++;
            balance_control_last_outer_ms = now;
            return;
        } else {
            kp *= balance_control_config.fine_fast_kp_scale;
            kd *= 1.0f;
            ki = balance_control_config.soft_ki_deg_per_px_s *
                 balance_control_config.fine_fast_ki_scale;
            angle_limit *=
                balance_control_config.fine_fast_angle_limit_scale;
        }
    }

    if (abs_velocity >=
        balance_control_config.damping_velocity_px_s) {
        kp *= balance_control_config.damping_kp_scale;
        kd *= balance_control_config.damping_kd_scale;
        angle_limit *=
            balance_control_config.damping_angle_limit_scale;
    }

    if (abs_velocity >=
        balance_control_config.freeze_integral_velocity_px_s) {
        kp *= balance_control_config.freeze_kp_scale;
        kd *= balance_control_config.freeze_kd_scale;
        ki = 0.0f;
        balance_control_state.outer_integral *=
            balance_control_config.freeze_integral_decay;
        angle_limit *=
            balance_control_config.freeze_angle_limit_scale;
    }

    dt = BalanceControl_GetOuterDt(now);
    if (ki > 0.0f) {
        float integral_limit = BalanceControl_AbsFloat(
            balance_control_config.outer_integral_limit_px_s);

        balance_control_state.outer_integral += error * dt;
        balance_control_state.outer_integral =
            BalanceControl_ClampFloat(
                balance_control_state.outer_integral,
                -integral_limit,
                integral_limit);
    }

    balance_control_state.position_p_term = kp * error;
    balance_control_state.position_i_term =
        ki * balance_control_state.outer_integral;
    balance_control_state.velocity_d_term = -kd * ball_velocity;
    output = balance_control_state.position_p_term +
             balance_control_state.position_i_term +
             balance_control_state.velocity_d_term;
    output = BalanceControl_ClampFloat(output,
                                       -angle_limit,
                                       angle_limit);
    balance_control_state.outer_output_deg = output;

    tilt_direction = BalanceControl_NonzeroDirection(
        balance_control_config.tilt_direction);
    balance_control_state.target_rod_angle_deg =
        (float)tilt_direction * output;
    balance_control_state.leveling = 0U;
    balance_control_state.outer_update_count++;
}

static float BalanceControl_ApplySlew(float desired_speed)
{
    float slew = BalanceControl_AbsFloat(
        balance_control_config.motor_slew_per_update);
    float delta = desired_speed - balance_control_slew_output;

    if (slew <= 0.0f) {
        return desired_speed;
    }
    if (delta > slew) {
        delta = slew;
    } else if (delta < -slew) {
        delta = -slew;
    }

    return balance_control_slew_output + delta;
}

static int32_t BalanceControl_QuantizeMotorSpeed(float speed)
{
    float magnitude = BalanceControl_AbsFloat(speed);
    float deadband = BalanceControl_AbsFloat(
        balance_control_config.motor_speed_deadband);
    float minimum = BalanceControl_AbsFloat(
        balance_control_config.motor_min_speed);
    float limit = BalanceControl_AbsFloat(
        balance_control_config.motor_speed_limit);

    if (magnitude <= deadband) {
        return 0;
    }
    if (magnitude < minimum) {
        magnitude = minimum;
    }
    if (magnitude > limit) {
        magnitude = limit;
    }

    return (speed < 0.0f) ?
           -BalanceControl_RoundToInt(magnitude) :
           BalanceControl_RoundToInt(magnitude);
}

static void BalanceControl_StopForInvalidPosition(
    uint32_t vision_frame,
    uint32_t vision_last_rx_ms,
    float ball_position,
    float ball_velocity)
{
    HAL_StatusTypeDef status = HAL_TIMEOUT;

    balance_control_state.motor_position_valid = 0U;
    balance_control_state.leveling = 1U;
    balance_control_state.stable = 0U;
    balance_control_state.stable_count = 0U;
    balance_control_state.position_p_term = 0.0f;
    balance_control_state.position_i_term = 0.0f;
    balance_control_state.velocity_d_term = 0.0f;
    balance_control_state.outer_integral = 0.0f;
    balance_control_state.outer_output_deg = 0.0f;
    balance_control_state.target_rod_angle_deg = 0.0f;
    balance_control_state.angle_error_deg = 0.0f;
    balance_control_state.desired_motor_speed = 0.0f;
    balance_control_state.motor_command = 0;
    balance_control_slew_output = 0.0f;
    balance_control_fine_level_hold = 0U;
    balance_control_last_outer_ms = 0U;
    balance_control_last_vision_frame = 0xFFFFFFFFUL;

    if (balance_control_last_sent_command != 0) {
        status = BalanceControl_StopMotorSafely();
        if (status == HAL_OK) {
            balance_control_last_sent_command = 0;
        }
        BalanceControl_SendMotorFeedback(
            vision_frame,
            vision_last_rx_ms,
            ball_position,
            ball_velocity,
            0,
            status);
    }
    balance_control_state.last_motor_status = HAL_TIMEOUT;
}

static void BalanceControl_UpdateInner(uint32_t vision_frame,
                                       uint32_t vision_last_rx_ms,
                                       float ball_position,
                                       float ball_velocity)
{
    float rod_scale;
    float rod_limit;
    float target_angle;
    float angle_kp;
    float speed_limit;
    float desired_speed;
    float next_slew_output;
    int32_t motor_command;
    int8_t motor_direction;
    uint8_t command_attempted = 0U;
    HAL_StatusTypeDef motor_status = HAL_OK;

    if (MotorPositionMonitor_IsFresh(
            balance_control_config.motor_position_timeout_ms) == 0U) {
        BalanceControl_StopForInvalidPosition(
            vision_frame,
            vision_last_rx_ms,
            ball_position,
            ball_velocity);
        return;
    }

    rod_scale = balance_control_config.rod_angle_per_motor_degree;
    if (BalanceControl_AbsFloat(rod_scale) < 0.000001f) {
        BalanceControl_StopForInvalidPosition(
            vision_frame,
            vision_last_rx_ms,
            ball_position,
            ball_velocity);
        balance_control_state.last_motor_status = HAL_ERROR;
        return;
    }

    balance_control_state.motor_position_valid = 1U;
    balance_control_state.motor_position =
        motor_position_monitor_state.position;
    balance_control_state.motor_angle_deg =
        motor_position_monitor_state.angle_deg;
    balance_control_state.motor_position_update_count =
        motor_position_monitor_state.update_count;

    if (balance_control_state.motor_zero_pending != 0U) {
        balance_control_config.motor_zero_angle_deg =
            balance_control_state.motor_angle_deg;
        balance_control_state.motor_zero_pending = 0U;
        balance_control_slew_output = 0.0f;
        balance_control_last_sent_command = 0;
    }

    balance_control_state.rod_angle_deg =
        (balance_control_state.motor_angle_deg -
         balance_control_config.motor_zero_angle_deg) * rod_scale;

    target_angle = balance_control_state.target_rod_angle_deg;
    rod_limit = BalanceControl_AbsFloat(
        balance_control_config.rod_angle_limit_deg);
    if (rod_limit > 0.0f &&
        BalanceControl_AbsFloat(balance_control_state.rod_angle_deg) >
            rod_limit) {
        target_angle = 0.0f;
        balance_control_state.target_rod_angle_deg = 0.0f;
        balance_control_state.leveling = 1U;
        balance_control_state.outer_integral = 0.0f;
    }

    balance_control_state.angle_error_deg =
        target_angle - balance_control_state.rod_angle_deg;
    angle_kp = balance_control_config.angle_kp_speed_per_deg;
    if (balance_control_fine_level_hold != 0U) {
        angle_kp *= balance_control_config.fine_hold_inner_kp_scale;
    }

    motor_direction = BalanceControl_NonzeroDirection(
        balance_control_config.motor_direction);
    desired_speed = angle_kp *
                    balance_control_state.angle_error_deg *
                    (float)motor_direction;
    speed_limit = BalanceControl_AbsFloat(
        balance_control_config.motor_speed_limit);
    desired_speed = BalanceControl_ClampFloat(desired_speed,
                                               -speed_limit,
                                               speed_limit);
    balance_control_state.desired_motor_speed = desired_speed;

    next_slew_output = BalanceControl_ApplySlew(desired_speed);
    motor_command = BalanceControl_QuantizeMotorSpeed(next_slew_output);
    balance_control_state.motor_command = motor_command;

    if (motor_command == 0) {
        if (balance_control_last_sent_command != 0) {
            motor_status = BalanceControl_StopMotorSafely();
            command_attempted = 1U;
        }
    } else {
        motor_status = DS_BalanceSetSpeed(
            motor_command,
            balance_control_config.motor_slope);
        command_attempted = 1U;
    }

    if (motor_status == HAL_OK) {
        balance_control_slew_output = next_slew_output;
        balance_control_last_sent_command = motor_command;
    }
    balance_control_state.last_motor_status = motor_status;

    if (command_attempted != 0U) {
        BalanceControl_SendMotorFeedback(
            vision_frame,
            vision_last_rx_ms,
            ball_position,
            ball_velocity,
            motor_command,
            motor_status);
    }
}

void BalanceControl_Init(void)
{
    balance_control_state.enabled = 0U;
    balance_control_state.vision_valid = 0U;
    balance_control_state.motor_zero_pending = 0U;
    balance_control_state.target_position = 0.0f;
    balance_control_state.ball_position = 0.0f;
    balance_control_state.ball_velocity = 0.0f;
    balance_control_state.update_count = 0U;
    balance_control_state.outer_update_count = 0U;
    balance_control_state.motor_position_update_count = 0U;
    balance_control_state.last_motor_status = HAL_OK;
    BalanceControl_ResetOuter();
    BalanceControl_ResetInner();

    balance_control_last_update_ms = HAL_GetTick();
    balance_control_last_vision_frame = 0xFFFFFFFFUL;
    balance_control_had_valid_vision = 0U;
}

void BalanceControl_Start(float target_position)
{
    (void)DS_BalanceStop();
    BalanceControl_ResetOuter();
    BalanceControl_ResetInner();

    balance_control_state.enabled = 1U;
    balance_control_state.vision_valid = 0U;
    balance_control_state.target_position = target_position;
    balance_control_state.ball_position = 0.0f;
    balance_control_state.ball_velocity = 0.0f;
    balance_control_state.update_count = 0U;
    balance_control_state.outer_update_count = 0U;
    balance_control_state.motor_position_update_count = 0U;
    balance_control_state.motor_zero_pending =
        (balance_control_config.capture_motor_zero_on_start != 0U) ?
        1U :
        0U;
    balance_control_state.last_motor_status = HAL_BUSY;

    balance_control_last_update_ms =
        HAL_GetTick() - balance_control_config.control_period_ms;
    balance_control_last_vision_frame = 0xFFFFFFFFUL;
    balance_control_had_valid_vision = 0U;
    MotorPositionMonitor_StartWithPeriod(
        balance_control_config.motor_position_period_ms);
}

void BalanceControl_SetTarget(float target_position)
{
    balance_control_state.target_position = target_position;
    BalanceControl_ResetOuter();
    balance_control_last_vision_frame = 0xFFFFFFFFUL;
}

void BalanceControl_Update(void)
{
    const DS_State *ds;
    uint32_t now;
    uint32_t primask;
    uint32_t vision_frame;
    uint32_t vision_last_rx_ms;
    float ball_position;
    float ball_velocity;
    uint8_t vision_valid;

    if (balance_control_state.enabled == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - balance_control_last_update_ms) >=
        balance_control_config.control_period_ms) {
        balance_control_last_update_ms = now;
        ds = DS_GetState();

        primask = __get_PRIMASK();
        __disable_irq();
        ball_position = ds->ball_position;
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
            balance_control_state.vision_valid = 1U;
            balance_control_state.ball_position = ball_position;
            balance_control_state.ball_velocity = ball_velocity;

            if (balance_control_had_valid_vision == 0U) {
                balance_control_state.outer_integral = 0.0f;
                balance_control_last_outer_ms = 0U;
                balance_control_last_vision_frame = 0xFFFFFFFFUL;
            }
            balance_control_had_valid_vision = 1U;
            BalanceControl_UpdateOuter(now,
                                       vision_frame,
                                       ball_position,
                                       ball_velocity);
        } else {
            balance_control_state.vision_valid = 0U;
            if (balance_control_had_valid_vision != 0U) {
                BalanceControl_ResetOuter();
            }
            balance_control_had_valid_vision = 0U;
            balance_control_state.target_rod_angle_deg = 0.0f;
            balance_control_state.leveling = 1U;
        }

        BalanceControl_UpdateInner(vision_frame,
                                   vision_last_rx_ms,
                                   ball_position,
                                   ball_velocity);
        balance_control_state.update_count++;
    }

    /*
     * Run the readback state machine after the controller. This lets the
     * controller send its speed command before the next asynchronous 0x36
     * request owns USART1.
     */
    MotorPositionMonitor_Update();
}

void BalanceControl_Stop(void)
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

    MotorPositionMonitor_Stop();
    motor_status = DS_BalanceStop();
    BalanceControl_SendMotorFeedback(
        vision_frame,
        vision_last_rx_ms,
        balance_control_state.ball_position,
        balance_control_state.ball_velocity,
        0,
        motor_status);

    balance_control_state.enabled = 0U;
    balance_control_state.vision_valid = 0U;
    balance_control_state.motor_position_valid = 0U;
    balance_control_state.leveling = 0U;
    balance_control_state.motor_command = 0;
    balance_control_state.desired_motor_speed = 0.0f;
    balance_control_state.target_rod_angle_deg = 0.0f;
    balance_control_state.angle_error_deg = 0.0f;
    balance_control_state.stable = 0U;
    balance_control_state.stable_count = 0U;
    balance_control_state.motor_zero_pending = 0U;
    balance_control_state.last_motor_status = motor_status;

    balance_control_slew_output = 0.0f;
    balance_control_last_sent_command = 0;
    balance_control_had_valid_vision = 0U;
    balance_control_fine_level_hold = 0U;
}

uint8_t BalanceControl_IsStable(void)
{
    return balance_control_state.stable;
}
