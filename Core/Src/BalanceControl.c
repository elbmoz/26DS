#include "BalanceControl.h"

#include "BallVision.h"
#include "DS.h"
#include "MotorPositionMonitor.h"

/*
 * ==================== 任务 2 首轮调试入口 ====================
 *
 * 调试顺序必须是：
 *   1. 先调管道角度内环，让实际管道角稳定追踪目标管道角；
 *   2. 再调小球位置外环，让小球回到目标位置；
 *   3. 最后才恢复积分和调近中心/高速增益调度。
 *
 * Keil Watch 重点观察：
 *   balance_control_state.target_rod_angle_deg  外环给出的目标管道角
 *   balance_control_state.rod_angle_deg         拟合得到的实际管道角
 *   balance_control_state.angle_error_deg       内环角度误差
 *   balance_control_state.motor_command         最终电机速度命令
 *
 * 提交 a010e37 已完成电机位置到管道 X 角的比例和方向标定。
 * 这些拟合参数不要当 PID 参数随意修改。
 */
BalanceControlConfig balance_control_config = {
    /*
     * ---------- 第二阶段：小球位置外环 ----------
     *
     * 首轮只调 Kp、Kd，Ki 保持 0：
     * - 回中太慢：小幅增大 Kp；
     * - 冲过中心、低频往复：减小 Kp 或增大 Kd；
     * - 电机被速度噪声带着抖：减小 Kd；
     * - P/D 稳定后仍有固定静差，最后才逐步增加 Ki。
     */
    .outer_kp_deg_per_px = 0.02f,       /* 位置 P：误差每 1 px 产生的角度。 */
    /* 首轮为 0；P/D 调稳后可从参考候选 0.03297 开始小步增加。 */
    .outer_ki_deg_per_px_s = 0.0f,      /* 位置 I：用于消除固定静差。 */
    .outer_kd_deg_per_px_s = 0.05f,     /* 速度 D：使用 -Kd*球速抑制冲过中心。 */
    .outer_integral_limit_px_s = 109.2f, /* 限制积分累积，防止积分饱和。 */
    /*
     * 当前值允许 ±5.5°目标管道角；最终不得超过拟合得到的
     * ±6.0°公共安全范围。首次排查方向时应临时调低到约 1～2°。
     */
    .outer_angle_limit_deg = 5.5f,

    /* 近中心增益调度：内外环基础调通前先不要修改这一组。 */
    .hold_band_px = 18.2f,              /* 误差进入 ±18.2 px 后使用柔化参数。 */
    .fine_band_px = 3.64f,              /* 误差进入 ±3.64 px 后考虑回水平。 */
    .fine_velocity_px_s = 18.2f,        /* 精细区内低于此球速才强制回水平。 */
    .soft_kp_scale = 0.55f,             /* 柔化区把基础 Kp 乘 0.55。 */
    .soft_kd_scale = 0.75f,             /* 柔化区把基础 Kd 乘 0.75。 */
    .soft_angle_limit_scale = 0.65f,    /* 柔化区把目标角限幅乘 0.65。 */
    /* 首轮为 0；外环稳定后可从参考候选 0.10989 以下逐步增加。 */
    .soft_ki_deg_per_px_s = 0.0f,       /* 柔化区单独使用的 Ki，不是基础 Ki 倍率。 */
    .fine_fast_kp_scale = 0.25f,        /* 精细区但球速较快时的 Kp 倍率。 */
    .fine_fast_ki_scale = 0.50f,        /* 精细区高速时把 soft Ki 再乘 0.50。 */
    .fine_fast_angle_limit_scale = 0.40f, /* 精细区高速时的限幅倍率。 */
    .hold_integral_decay = 0.70f,       /* 回水平保持时每帧保留 70% 积分。 */
    .fine_hold_inner_kp_scale = 0.60f,  /* 回水平保持时把角度内环 Kp 乘 0.60。 */

    /* 高球速阻尼调度：基础外环调通前先保持默认，不作为首要调参项。 */
    .damping_velocity_px_s = 50.96f,    /* 达到该球速后开始减 P、增 D。 */
    .damping_kp_scale = 0.55f,          /* 高速阻尼区把当前 Kp 乘 0.55。 */
    .damping_kd_scale = 1.80f,          /* 高速阻尼区把当前 Kd 乘 1.80。 */
    .damping_angle_limit_scale = 0.70f, /* 高速阻尼区把当前限幅乘 0.70。 */
    .freeze_integral_velocity_px_s = 81.90f, /* 达到该球速后关闭本帧积分。 */
    .freeze_kp_scale = 0.70f,           /* 冻结区把已调度 Kp 再乘 0.70。 */
    .freeze_kd_scale = 1.40f,           /* 冻结区把已调度 Kd 再乘 1.40。 */
    .freeze_angle_limit_scale = 0.75f,  /* 冻结区把已调度限幅再乘 0.75。 */
    .freeze_integral_decay = 0.90f,     /* 冻结时每帧保留 90% 历史积分。 */

    /*
     * ---------- 已完成标定：不要当 PID 调 ----------
     *
     * a010e37 稳态拟合：
     *   管道X角 = -0.0020022658 * 电机原始位置 + 截距
     *
     * 六字节 0x36 回包在底层换算为：
     *   电机角 = 电机原始位置 * 360 / 65536
     *
     * 所以：
     *   管道角/电机角 = -0.36450137
     *
     * 数据里的 P=5025.92 只对应那次相对 IMU 零点，不能跨上电复用。
     * 每次进入任务 2 前必须先把管道物理调平，再捕获本次水平锚点。
     */
    .motor_zero_angle_deg = 0.0f,       /* 本次管道水平对应的电机角。 */
    .rod_angle_per_motor_degree = -0.36450137f, /* 实测带符号传动比例。 */
    .rod_angle_limit_deg = 6.5f,        /* 超过该实际管道角时强制回水平。 */
    .capture_motor_zero_on_start = 1U,  /* 1：启动后首个有效位置作为水平零点。 */

    /*
     * ---------- 第一阶段：管道角度内环 ----------
     *
     * 首轮主要调以下五项：
     * - angle_kp_speed_per_deg：追角力度。追得慢就增大，来回摆就减小；
     * - motor_speed_limit：最大速度。首轮限制为 30，确认安全后再放宽；
     * - motor_slew_per_update：每周期速度变化。越小越柔和；
     * - motor_min_speed：克服静摩擦的最小速度；
     * - motor_speed_deadband：目标附近的停车死区。
     */
    .angle_kp_speed_per_deg = 5.5f,     /* 角度误差每 1°产生的速度命令。 */
    .motor_speed_limit = 30.0f,         /* 最终速度命令绝对值不得超过 30。 */
    .motor_speed_deadband = 0.0f,       /* 连续速度落入该死区时命令为 0。 */
    .motor_min_speed = 0.0f,            /* 非零命令不足该值时向上补偿。 */
    .motor_slew_per_update = 2.0f,      /* 每 20 ms 最多改变 2 个速度命令单位。 */
    /*
     * F6 驱动器加减速档。0 表示直接启停/换向；非零时数值越大加减速越快。
     * 速度命令实际对应 1 RPM 还是 0.1 RPM，取决于驱动器 S_Vel_IS 设置。
     */
    .motor_slope = 0U,
    /*
     * 已完成的符号标定：
     * 正电机命令使 P 增大、管道 X 角减小，所以内环方向必须为 -1。
     * 正电机命令又会让球向画面左侧移动，所以外环倾斜方向必须为 +1。
     * 这两个方向和负的角度比例是配套的，不要只翻转其中一项。
     */
    .tilt_direction = 1,                /* 外环输出到目标管道角的符号。 */
    .motor_direction = -1,              /* 角度误差到电机命令的符号。 */

    .control_period_ms = 20U,           /* 任务 2 内外环计算周期：50 Hz。 */
    .motor_position_period_ms = 20U,    /* 0x36 电机位置查询周期：50 Hz。 */
    .motor_position_timeout_ms = 60U,   /* 连续 60 ms 没有新位置就停机。 */

    .stable_error_px = 18.2f,           /* 稳定时允许的位置误差：约 1 cm。 */
    .stable_velocity_px_s = 25.0f,      /* 稳定时允许的最大球速。 */
    .stable_frames = 25U,               /* 连续 25 个视觉帧满足条件才判稳定。 */

    .pixels_per_cm = 18.2f,             /* 当前视觉标定：每厘米 18.2 px。 */
    .positive_5cm_target = 91.0f,       /* 后续阶段预留的 +5 cm 目标。 */
    .negative_5cm_target = -91.0f       /* 后续阶段预留的 -5 cm 目标。 */
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
