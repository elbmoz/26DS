#include "BalanceControl.h"

#include "DS.h"

/*
 * Conservative starting values for the MaixCAM reference-pixel output.
 * The camera reports positive error/velocity toward image right. The known
 * mechanism moves the ball left for a positive/clockwise motor command, so
 * motor_direction is -1 because PID_Control uses target - feedback.
 */
BalanceControlConfig balance_control_config = {
    .kp = 0.55f,
    .ki = 0.001f,
    .kd = 0.08f,
    .integral_limit = 3000.0f,
    /*
     * Keep closed-loop motor output disabled while the bidirectional UART
     * link is brought up. Set a tested positive limit before PID trials.
     */
    .output_limit = 0.0f,
    .pid_deadband = 3.0f,
    .velocity_deadband = 12.0f,
    .control_period_ms = 20U,
    .motor_slope = 0U,
    .motor_direction = -1,
    .stable_error = 18.0f,
    .stable_velocity = 25.0f,
    .stable_cycles = 25U,
    .pixels_per_cm = 18.2f,
    .positive_5cm_target = 91.0f,
    .negative_5cm_target = -91.0f
};

BalanceControlState balance_control_state;
PID_Cycle balance_position_pid;

static uint32_t balance_control_last_update_ms;
static uint8_t balance_control_had_valid_vision;

static float BalanceControl_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int32_t BalanceControl_RoundToInt(float value)
{
    return (value >= 0.0f) ?
           (int32_t)(value + 0.5f) :
           (int32_t)(value - 0.5f);
}

static void BalanceControl_ApplyPIDConfig(void)
{
    balance_position_pid.p = balance_control_config.kp;
    balance_position_pid.i = balance_control_config.ki;

    /*
     * The MaixCAM already supplies a filtered velocity in px/s. Use that as
     * the D feedback below instead of differentiating quantized position
     * samples again inside the generic PID implementation.
     */
    balance_position_pid.d = 0.0f;
    balance_position_pid.out_max =
        balance_control_config.output_limit;
    balance_position_pid.i_max =
        balance_control_config.integral_limit;
    balance_position_pid.deadband =
        balance_control_config.pid_deadband;
    balance_position_pid.mode = PID_POSITION;
}

void BalanceControl_Init(void)
{
    BalanceControl_ApplyPIDConfig();
    balance_position_pid.enable = 0U;
    PID_Cycle_Reset(&balance_position_pid);

    balance_control_state.enabled = 0U;
    balance_control_state.vision_valid = 0U;
    balance_control_state.stable = 0U;
    balance_control_state.target_position = 0.0f;
    balance_control_state.ball_position = 0.0f;
    balance_control_state.ball_velocity = 0.0f;
    balance_control_state.position_error = 0.0f;
    balance_control_state.velocity_d_term = 0.0f;
    balance_control_state.pid_output = 0.0f;
    balance_control_state.motor_command = 0;
    balance_control_state.stable_count = 0U;
    balance_control_state.update_count = 0U;
    balance_control_state.last_motor_status = HAL_OK;
    balance_control_last_update_ms = HAL_GetTick();
    balance_control_had_valid_vision = 0U;
}

void BalanceControl_Start(float target_position)
{
    BalanceControl_ApplyPIDConfig();
    PID_Cycle_Reset(&balance_position_pid);
    PID_Cycle_SetEnable(&balance_position_pid, 1U);

    balance_control_state.enabled = 1U;
    balance_control_state.vision_valid = 0U;
    balance_control_state.stable = 0U;
    balance_control_state.target_position = target_position;
    balance_control_state.ball_position = 0.0f;
    balance_control_state.ball_velocity = 0.0f;
    balance_control_state.position_error = 0.0f;
    balance_control_state.velocity_d_term = 0.0f;
    balance_control_state.pid_output = 0.0f;
    balance_control_state.motor_command = 0;
    balance_control_state.stable_count = 0U;
    balance_control_state.update_count = 0U;
    balance_control_state.last_motor_status = HAL_OK;
    balance_control_had_valid_vision = 0U;
    balance_control_last_update_ms =
        HAL_GetTick() - balance_control_config.control_period_ms;
}

void BalanceControl_SetTarget(float target_position)
{
    balance_control_state.target_position = target_position;
    balance_control_state.stable = 0U;
    balance_control_state.stable_count = 0U;
    PID_Cycle_Reset(&balance_position_pid);
}

void BalanceControl_Update(void)
{
    const DS_State *ds;
    uint32_t now;
    uint32_t primask;
    float ball_position;
    float ball_velocity;
    float raw_error;
    float output;
    float velocity_d_term;
    int32_t motor_command;
    int32_t previous_motor_command;
    int8_t motor_direction;

    if (balance_control_state.enabled == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - balance_control_last_update_ms) <
        balance_control_config.control_period_ms) {
        return;
    }
    balance_control_last_update_ms = now;

    ds = DS_GetState();
    balance_control_state.vision_valid = DS_BallVisionIsFresh();

    if (balance_control_state.vision_valid == 0U) {
        if (balance_control_had_valid_vision != 0U ||
            balance_control_state.motor_command != 0) {
            (void)DS_BalanceStop();
        }
        balance_control_had_valid_vision = 0U;
        balance_control_state.stable = 0U;
        balance_control_state.stable_count = 0U;
        balance_control_state.velocity_d_term = 0.0f;
        balance_control_state.pid_output = 0.0f;
        balance_control_state.motor_command = 0;
        balance_control_state.last_motor_status = HAL_TIMEOUT;
        PID_Cycle_Reset(&balance_position_pid);
        return;
    }

    balance_control_had_valid_vision = 1U;

    /* Keep position and velocity from the same completed UART frame. */
    primask = __get_PRIMASK();
    __disable_irq();
    ball_position = ds->ball_position;
    ball_velocity = ds->ball_velocity;
    if (primask == 0U) {
        __enable_irq();
    }

    balance_control_state.ball_position = ball_position;
    balance_control_state.ball_velocity = ball_velocity;
    raw_error = balance_control_state.target_position -
                balance_control_state.ball_position;
    balance_control_state.position_error = raw_error;

    BalanceControl_ApplyPIDConfig();
    if (BalanceControl_AbsFloat(raw_error) <=
        balance_control_config.pid_deadband) {
        balance_position_pid.integral = 0.0f;
    }

    output = PID_Control(&balance_position_pid,
                         balance_control_state.target_position,
                         balance_control_state.ball_position);

    /*
     * For a fixed target, d(target - position)/dt = -ball_velocity.
     * Using the camera velocity also keeps Kd independent of UART frame
     * quantization and avoids differentiating the integer position twice.
     */
    velocity_d_term = -balance_control_config.kd *
                      balance_control_state.ball_velocity;
    output += velocity_d_term;
    output = LimitValue(output,
                        -balance_control_config.output_limit,
                        balance_control_config.output_limit);

    if (BalanceControl_AbsFloat(raw_error) <=
            balance_control_config.pid_deadband &&
        BalanceControl_AbsFloat(balance_control_state.ball_velocity) <=
            balance_control_config.velocity_deadband) {
        balance_position_pid.integral = 0.0f;
        velocity_d_term = 0.0f;
        output = 0.0f;
    }

    balance_control_state.velocity_d_term = velocity_d_term;
    motor_direction = balance_control_config.motor_direction;
    if (motor_direction == 0) {
        motor_direction = 1;
    }
    output *= (float)motor_direction;
    motor_command = BalanceControl_RoundToInt(output);
    previous_motor_command = balance_control_state.motor_command;

    balance_control_state.pid_output = output;
    balance_control_state.motor_command = motor_command;

    if (motor_command == 0) {
        if (previous_motor_command != 0) {
            balance_control_state.last_motor_status =
                DS_BalanceStop();
        } else {
            balance_control_state.last_motor_status = HAL_OK;
        }
    } else {
        balance_control_state.last_motor_status =
            DS_BalanceSetSpeed(motor_command,
                               balance_control_config.motor_slope);
    }

    if (BalanceControl_AbsFloat(raw_error) <=
            balance_control_config.stable_error &&
        BalanceControl_AbsFloat(balance_control_state.ball_velocity) <=
            balance_control_config.stable_velocity) {
        if (balance_control_state.stable_count < UINT16_MAX) {
            balance_control_state.stable_count++;
        }
    } else {
        balance_control_state.stable_count = 0U;
        balance_control_state.stable = 0U;
    }

    if (balance_control_state.stable_count >=
        balance_control_config.stable_cycles) {
        balance_control_state.stable = 1U;
    }

    balance_control_state.update_count++;
}

void BalanceControl_Stop(void)
{
    balance_control_state.enabled = 0U;
    balance_control_state.velocity_d_term = 0.0f;
    balance_control_state.pid_output = 0.0f;
    balance_control_state.motor_command = 0;
    balance_control_state.stable = 0U;
    balance_control_state.stable_count = 0U;
    balance_control_had_valid_vision = 0U;
    PID_Cycle_SetEnable(&balance_position_pid, 0U);
    PID_Cycle_Reset(&balance_position_pid);
    (void)DS_BalanceStop();
}

uint8_t BalanceControl_IsStable(void)
{
    return balance_control_state.stable;
}
