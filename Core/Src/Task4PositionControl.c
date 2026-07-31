#include "Task4PositionControl.h"

#include "BallVision.h"
#include "DS.h"
#include "MotorPositionMonitor.h"

#include <limits.h>

/*
 * 任务 4 与任务 2 的外环初值保持一致，但不共享任何可写参数或状态。
 * 位置模式替代了任务 2 的“管角误差 -> 电机速度”软件内环，因此这里没有
 * angle_kp、速度死区、最小速度和 slew 等参数。
 */
Task4PositionControlConfig task4_position_control_config = {
    .outer_kp_deg_per_px = 0.028f,
    .outer_ki_deg_per_px_s = 0.0f,
    .outer_kd_deg_per_px_s = 1.05f,
    .outer_integral_limit_px_s = 109.2f,
    /* a010e37 公共拟合区的目标角上限。 */
    .outer_angle_limit_deg = 10.0f,

    .hold_band_px = 25.2f,
    .fine_band_px = 5.64f,
    .fine_velocity_px_s = 18.2f,
    .soft_kp_scale = 0.75f,
    .soft_kd_scale = 0.55f,
    .soft_angle_limit_scale = 0.65f,
    .soft_ki_deg_per_px_s = 0.0f,
    .fine_fast_kp_scale = 0.25f,
    .fine_fast_ki_scale = 0.50f,
    .fine_fast_angle_limit_scale = 0.40f,
    .hold_integral_decay = 0.70f,

    .damping_velocity_px_s = 50.96f,
    .damping_kp_scale = 0.55f,
    .damping_kd_scale = 1.80f,
    .damping_angle_limit_scale = 0.70f,
    .freeze_integral_velocity_px_s = 81.90f,
    .freeze_kp_scale = 0.70f,
    .freeze_kd_scale = 1.40f,
    .freeze_angle_limit_scale = 0.75f,
    .freeze_integral_decay = 0.90f,

    /*
     * a010e37 稳态拟合：
     *   rod_deg = -0.0020022658 * (position - horizontal_position)
     * Q9 的 360 个 0xFD 命令脉冲跨越约 7373 个 0x36 计数；当前驱动器
     * 3200 细分下理论值同样为 65536 / 3200 = 20.48 count/pulse。
     */
    .rod_angle_per_position_count = -0.0020022658f,
    .position_counts_per_command_pulse = 20.48f,
    .rod_angle_limit_deg = 6.5f,
    .tilt_direction = 1,

    .motor_move_speed = 10U,
    .motor_move_acceleration = 5U,

    .control_period_ms = 20U,
    .motor_position_startup_period_ms = 5U,
    /* 位置模式由驱动器闭环；0x36 仅低频刷新显示和诊断。 */
    .motor_position_period_ms = 200U,

    .stable_error_px = 19.2f,
    .stable_velocity_px_s = 25.0f,
    .stable_frames = 10U
};

Task4PositionControlState task4_position_control_state;

static uint32_t task4_position_control_last_update_ms;
static uint32_t task4_position_control_last_outer_ms;
static uint32_t task4_position_control_last_vision_frame;
static uint8_t task4_position_control_had_valid_vision;
static uint8_t task4_position_control_motor_started;

static float Task4PositionControl_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Task4PositionControl_ClampFloat(float value,
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

static int32_t Task4PositionControl_RoundToInt32(float value)
{
    if (value >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (value <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return (value >= 0.0f) ?
           (int32_t)(value + 0.5f) :
           (int32_t)(value - 0.5f);
}

static int8_t Task4PositionControl_NonzeroDirection(int8_t direction)
{
    return (direction < 0) ? -1 : 1;
}

static void Task4PositionControl_ResetOuter(void)
{
    task4_position_control_state.position_error = 0.0f;
    task4_position_control_state.position_p_term = 0.0f;
    task4_position_control_state.position_i_term = 0.0f;
    task4_position_control_state.velocity_d_term = 0.0f;
    task4_position_control_state.outer_integral = 0.0f;
    task4_position_control_state.outer_output_deg = 0.0f;
    task4_position_control_state.target_rod_angle_deg = 0.0f;
    task4_position_control_state.leveling = 1U;
    task4_position_control_state.stable = 0U;
    task4_position_control_state.stable_count = 0U;
    task4_position_control_last_outer_ms = 0U;
}

static void Task4PositionControl_ResetPositionState(void)
{
    task4_position_control_state.motor_position_valid = 0U;
    task4_position_control_state.motor_zero_position = 0;
    task4_position_control_state.motor_zero_command_pulses = 0;
    task4_position_control_state.motor_position = 0;
    task4_position_control_state.rod_angle_deg = 0.0f;
    task4_position_control_state.target_motor_position = 0;
    task4_position_control_state.target_motor_pulses = 0;
    task4_position_control_state.position_error_counts = 0;
    task4_position_control_state.motor_command_pulses = 0;
    task4_position_control_motor_started = 0U;
}

static HAL_StatusTypeDef Task4PositionControl_StopMotorSafely(void)
{
    DS_BalanceCancelPositionRequest();
    return DS_BalanceStop();
}

static void Task4PositionControl_SendFeedback(uint32_t vision_frame,
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
        task4_position_control_state.position_error,
        task4_position_control_state.position_p_term,
        task4_position_control_state.position_i_term,
        task4_position_control_state.velocity_d_term,
        motor_command,
        motor_status);
}

static float Task4PositionControl_GetOuterDt(uint32_t now)
{
    uint32_t elapsed_ms;

    if (task4_position_control_last_outer_ms == 0U) {
        elapsed_ms = task4_position_control_config.control_period_ms;
    } else {
        elapsed_ms = (uint32_t)(
            now - task4_position_control_last_outer_ms);
    }
    task4_position_control_last_outer_ms = now;

    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    } else if (elapsed_ms > 200U) {
        elapsed_ms = 200U;
    }

    return (float)elapsed_ms * 0.001f;
}

static void Task4PositionControl_UpdateStableState(float error,
                                                   float velocity)
{
    if (Task4PositionControl_AbsFloat(error) <=
            task4_position_control_config.stable_error_px &&
        Task4PositionControl_AbsFloat(velocity) <=
            task4_position_control_config.stable_velocity_px_s) {
        if (task4_position_control_state.stable_count < UINT16_MAX) {
            task4_position_control_state.stable_count++;
        }
    } else {
        task4_position_control_state.stable_count = 0U;
        task4_position_control_state.stable = 0U;
    }

    if (task4_position_control_state.stable_count >=
        task4_position_control_config.stable_frames) {
        task4_position_control_state.stable = 1U;
    }
}

static void Task4PositionControl_UpdateOuter(uint32_t now,
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
    float output;
    float dt;
    int8_t tilt_direction;

    if (vision_frame == task4_position_control_last_vision_frame) {
        return;
    }
    task4_position_control_last_vision_frame = vision_frame;

    error = task4_position_control_state.target_position - ball_position;
    abs_error = Task4PositionControl_AbsFloat(error);
    abs_velocity = Task4PositionControl_AbsFloat(ball_velocity);
    task4_position_control_state.position_error = error;
    Task4PositionControl_UpdateStableState(error, ball_velocity);

    kp = task4_position_control_config.outer_kp_deg_per_px;
    ki = task4_position_control_config.outer_ki_deg_per_px_s;
    kd = task4_position_control_config.outer_kd_deg_per_px_s;
    angle_limit = Task4PositionControl_AbsFloat(
        task4_position_control_config.outer_angle_limit_deg);

    if (abs_error <= task4_position_control_config.hold_band_px) {
        if (abs_error > task4_position_control_config.fine_band_px) {
            kp *= task4_position_control_config.soft_kp_scale;
            kd *= task4_position_control_config.soft_kd_scale;
            ki = task4_position_control_config.soft_ki_deg_per_px_s;
            angle_limit *=
                task4_position_control_config.soft_angle_limit_scale;
        } else if (abs_velocity <=
                   task4_position_control_config.fine_velocity_px_s) {
            task4_position_control_state.outer_integral *=
                task4_position_control_config.hold_integral_decay;
            task4_position_control_state.position_p_term = 0.0f;
            task4_position_control_state.position_i_term = 0.0f;
            task4_position_control_state.velocity_d_term = 0.0f;
            task4_position_control_state.outer_output_deg = 0.0f;
            task4_position_control_state.target_rod_angle_deg = 0.0f;
            task4_position_control_state.leveling = 1U;
            task4_position_control_state.outer_update_count++;
            task4_position_control_last_outer_ms = now;
            return;
        } else {
            kp *= task4_position_control_config.fine_fast_kp_scale;
            ki = task4_position_control_config.soft_ki_deg_per_px_s *
                 task4_position_control_config.fine_fast_ki_scale;
            angle_limit *= task4_position_control_config.
                fine_fast_angle_limit_scale;
        }
    }

    if (abs_velocity >=
        task4_position_control_config.damping_velocity_px_s) {
        kp *= task4_position_control_config.damping_kp_scale;
        kd *= task4_position_control_config.damping_kd_scale;
        angle_limit *=
            task4_position_control_config.damping_angle_limit_scale;
    }

    if (abs_velocity >=
        task4_position_control_config.freeze_integral_velocity_px_s) {
        kp *= task4_position_control_config.freeze_kp_scale;
        kd *= task4_position_control_config.freeze_kd_scale;
        ki = 0.0f;
        task4_position_control_state.outer_integral *=
            task4_position_control_config.freeze_integral_decay;
        angle_limit *=
            task4_position_control_config.freeze_angle_limit_scale;
    }

    dt = Task4PositionControl_GetOuterDt(now);
    if (ki > 0.0f) {
        float integral_limit = Task4PositionControl_AbsFloat(
            task4_position_control_config.outer_integral_limit_px_s);

        task4_position_control_state.outer_integral += error * dt;
        task4_position_control_state.outer_integral =
            Task4PositionControl_ClampFloat(
                task4_position_control_state.outer_integral,
                -integral_limit,
                integral_limit);
    }

    task4_position_control_state.position_p_term = kp * error;
    task4_position_control_state.position_i_term =
        ki * task4_position_control_state.outer_integral;
    task4_position_control_state.velocity_d_term = -kd * ball_velocity;
    output = task4_position_control_state.position_p_term +
             task4_position_control_state.position_i_term +
             task4_position_control_state.velocity_d_term;
    output = Task4PositionControl_ClampFloat(output,
                                             -angle_limit,
                                             angle_limit);
    task4_position_control_state.outer_output_deg = output;
    tilt_direction = Task4PositionControl_NonzeroDirection(
        task4_position_control_config.tilt_direction);
    task4_position_control_state.target_rod_angle_deg =
        (float)tilt_direction * output;
    task4_position_control_state.leveling = 0U;
    task4_position_control_state.outer_update_count++;
}

static void Task4PositionControl_StopForInvalidPosition(void)
{
    HAL_StatusTypeDef status = HAL_TIMEOUT;

    task4_position_control_state.motor_position_valid = 0U;
    task4_position_control_state.stable = 0U;
    task4_position_control_state.stable_count = 0U;
    task4_position_control_state.target_rod_angle_deg = 0.0f;
    task4_position_control_state.leveling = 1U;
    task4_position_control_state.outer_integral = 0.0f;
    task4_position_control_state.motor_command_pulses = 0;

    if (task4_position_control_motor_started != 0U) {
        status = Task4PositionControl_StopMotorSafely();
        if (status == HAL_OK) {
            task4_position_control_motor_started = 0U;
        }
    }
    task4_position_control_state.last_motor_status =
        (status == HAL_OK) ? HAL_TIMEOUT : status;
}

static void Task4PositionControl_UpdatePositionCommand(
    uint32_t vision_frame,
    uint32_t vision_last_rx_ms,
    float ball_position,
    float ball_velocity)
{
    float slope;
    float counts_per_pulse;
    float rod_limit;
    float target_rod_angle;
    float target_offset_counts;
    int32_t target_offset_pulses;
    int32_t target_motor_pulses;
    HAL_StatusTypeDef status;
    uint8_t reply_supported;

    reply_supported =
        (motor_position_monitor_state.rx_length == 6U ||
         motor_position_monitor_state.rx_length == 8U) ? 1U : 0U;

    slope = task4_position_control_config.rod_angle_per_position_count;
    counts_per_pulse =
        task4_position_control_config.position_counts_per_command_pulse;
    if (Task4PositionControl_AbsFloat(slope) < 0.0000001f ||
        Task4PositionControl_AbsFloat(counts_per_pulse) < 0.0001f) {
        Task4PositionControl_StopForInvalidPosition();
        task4_position_control_state.last_motor_status = HAL_ERROR;
        return;
    }

    if (task4_position_control_state.motor_zero_pending != 0U) {
        /*
         * Only the startup anchor is a hard dependency on 0x36. After it is
         * captured, the driver closes the position loop internally and low-
         * rate monitor failures must not interrupt the 0xFD command stream.
         */
        if (motor_position_monitor_state.valid == 0U ||
            motor_position_monitor_state.update_count == 0U ||
            reply_supported == 0U) {
            task4_position_control_state.motor_position_valid = 0U;
            task4_position_control_state.last_motor_status =
                motor_position_monitor_state.last_status;
            return;
        }

        task4_position_control_state.motor_position =
            motor_position_monitor_state.position;
        task4_position_control_state.motor_position_update_count =
            motor_position_monitor_state.update_count;
        task4_position_control_state.motor_zero_position =
            task4_position_control_state.motor_position;
        task4_position_control_state.motor_zero_command_pulses =
            Task4PositionControl_RoundToInt32(
                (float)task4_position_control_state.motor_zero_position /
                counts_per_pulse);
        task4_position_control_state.motor_zero_pending = 0U;
        MotorPositionMonitor_SetPeriod(
            task4_position_control_config.motor_position_period_ms);
    } else if (motor_position_monitor_state.valid != 0U &&
               reply_supported != 0U &&
               motor_position_monitor_state.update_count !=
                   task4_position_control_state.motor_position_update_count) {
        task4_position_control_state.motor_position =
            motor_position_monitor_state.position;
        task4_position_control_state.motor_position_update_count =
            motor_position_monitor_state.update_count;
    }

    /* P=V means the current Task 4 session has a valid horizontal anchor. */
    task4_position_control_state.motor_position_valid = 1U;

    task4_position_control_state.rod_angle_deg = slope *
        (float)(task4_position_control_state.motor_position -
                task4_position_control_state.motor_zero_position);

    target_rod_angle =
        task4_position_control_state.target_rod_angle_deg;
    rod_limit = Task4PositionControl_AbsFloat(
        task4_position_control_config.rod_angle_limit_deg);
    if (rod_limit > 0.0f &&
        Task4PositionControl_AbsFloat(
            task4_position_control_state.rod_angle_deg) > rod_limit) {
        target_rod_angle = 0.0f;
        task4_position_control_state.target_rod_angle_deg = 0.0f;
        task4_position_control_state.outer_integral = 0.0f;
        task4_position_control_state.leveling = 1U;
    }

    target_offset_counts = target_rod_angle / slope;
    task4_position_control_state.target_motor_position =
        Task4PositionControl_RoundToInt32(
            (float)task4_position_control_state.motor_zero_position +
            target_offset_counts);
    task4_position_control_state.position_error_counts =
        task4_position_control_state.target_motor_position -
        task4_position_control_state.motor_position;

    target_offset_pulses = Task4PositionControl_RoundToInt32(
        target_offset_counts / counts_per_pulse);
    target_motor_pulses = Task4PositionControl_RoundToInt32(
        (float)task4_position_control_state.motor_zero_command_pulses +
        (float)target_offset_pulses);
    task4_position_control_state.target_motor_pulses =
        target_motor_pulses;
    task4_position_control_state.motor_command_pulses =
        target_motor_pulses;

    /* Refresh the absolute position target every control period. */
    status = DS_BalanceMoveAbsolute(
        target_motor_pulses,
        task4_position_control_config.motor_move_speed,
        task4_position_control_config.motor_move_acceleration);
    task4_position_control_state.last_motor_status = status;
    if (status == HAL_OK) {
        task4_position_control_motor_started = 1U;
    }

    Task4PositionControl_SendFeedback(
        vision_frame,
        vision_last_rx_ms,
        ball_position,
        ball_velocity,
        target_motor_pulses,
        status);
}

void Task4PositionControl_Init(void)
{
    task4_position_control_state.enabled = 0U;
    task4_position_control_state.vision_valid = 0U;
    task4_position_control_state.motor_zero_pending = 0U;
    task4_position_control_state.target_position = 0.0f;
    task4_position_control_state.ball_position = 0.0f;
    task4_position_control_state.ball_velocity = 0.0f;
    task4_position_control_state.update_count = 0U;
    task4_position_control_state.outer_update_count = 0U;
    task4_position_control_state.motor_position_update_count = 0U;
    task4_position_control_state.last_motor_status = HAL_OK;
    Task4PositionControl_ResetOuter();
    Task4PositionControl_ResetPositionState();

    task4_position_control_last_update_ms = HAL_GetTick();
    task4_position_control_last_vision_frame = 0xFFFFFFFFUL;
    task4_position_control_had_valid_vision = 0U;
}

void Task4PositionControl_Start(float target_position)
{
    (void)DS_BalanceStop();
    Task4PositionControl_ResetOuter();
    Task4PositionControl_ResetPositionState();

    task4_position_control_state.enabled = 1U;
    task4_position_control_state.vision_valid = 0U;
    task4_position_control_state.motor_zero_pending = 1U;
    task4_position_control_state.target_position = target_position;
    task4_position_control_state.ball_position = 0.0f;
    task4_position_control_state.ball_velocity = 0.0f;
    task4_position_control_state.update_count = 0U;
    task4_position_control_state.outer_update_count = 0U;
    task4_position_control_state.motor_position_update_count = 0U;
    task4_position_control_state.last_motor_status = HAL_BUSY;

    task4_position_control_last_update_ms = HAL_GetTick() -
        task4_position_control_config.control_period_ms;
    task4_position_control_last_vision_frame = 0xFFFFFFFFUL;
    task4_position_control_had_valid_vision = 0U;
    MotorPositionMonitor_StartWithPeriod(
        task4_position_control_config.motor_position_startup_period_ms);
}

void Task4PositionControl_SetTarget(float target_position)
{
    task4_position_control_state.target_position = target_position;
    Task4PositionControl_ResetOuter();
    task4_position_control_last_vision_frame = 0xFFFFFFFFUL;
}

void Task4PositionControl_Update(void)
{
    const DS_State *ds;
    uint32_t now;
    uint32_t primask;
    uint32_t vision_frame;
    uint32_t vision_last_rx_ms;
    float ball_position;
    float ball_velocity;
    uint8_t vision_valid;

    if (task4_position_control_state.enabled == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - task4_position_control_last_update_ms) >=
        task4_position_control_config.control_period_ms) {
        task4_position_control_last_update_ms = now;
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
            task4_position_control_state.vision_valid = 1U;
            task4_position_control_state.ball_position = ball_position;
            task4_position_control_state.ball_velocity = ball_velocity;

            if (task4_position_control_had_valid_vision == 0U) {
                task4_position_control_state.outer_integral = 0.0f;
                task4_position_control_last_outer_ms = 0U;
                task4_position_control_last_vision_frame = 0xFFFFFFFFUL;
            }
            task4_position_control_had_valid_vision = 1U;
            Task4PositionControl_UpdateOuter(now,
                                             vision_frame,
                                             ball_position,
                                             ball_velocity);
        } else {
            task4_position_control_state.vision_valid = 0U;
            if (task4_position_control_had_valid_vision != 0U) {
                Task4PositionControl_ResetOuter();
            }
            task4_position_control_had_valid_vision = 0U;
            task4_position_control_state.target_rod_angle_deg = 0.0f;
            task4_position_control_state.leveling = 1U;
        }

        Task4PositionControl_UpdatePositionCommand(
            vision_frame,
            vision_last_rx_ms,
            ball_position,
            ball_velocity);
        task4_position_control_state.update_count++;
    }

    MotorPositionMonitor_Update();
}

void Task4PositionControl_Stop(void)
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
    Task4PositionControl_SendFeedback(
        vision_frame,
        vision_last_rx_ms,
        task4_position_control_state.ball_position,
        task4_position_control_state.ball_velocity,
        0,
        motor_status);

    task4_position_control_state.enabled = 0U;
    task4_position_control_state.vision_valid = 0U;
    task4_position_control_state.motor_position_valid = 0U;
    task4_position_control_state.leveling = 0U;
    task4_position_control_state.stable = 0U;
    task4_position_control_state.stable_count = 0U;
    task4_position_control_state.motor_zero_pending = 0U;
    task4_position_control_state.motor_command_pulses = 0;
    task4_position_control_state.last_motor_status = motor_status;

    task4_position_control_motor_started = 0U;
    task4_position_control_had_valid_vision = 0U;
}

uint8_t Task4PositionControl_IsStable(void)
{
    return task4_position_control_state.stable;
}
