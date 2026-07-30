#include "BalanceControl.h"

#include "DS.h"

BalanceControlConfig balance_control_config = {
    .kp = 8.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 300.0f,
    .pid_deadband = 0.5f,
    .control_period_ms = 20U,
    .motor_slope = DS_MOTOR_DEFAULT_SLOPE,
    .motor_direction = 1,
    .stable_error = 1.0f,
    .stable_cycles = 25U,
    .positive_5cm_target = 5.0f,
    .negative_5cm_target = -5.0f
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
    balance_position_pid.d = balance_control_config.kd;
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
    balance_control_state.position_error = 0.0f;
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
    balance_control_state.position_error = 0.0f;
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
    float raw_error;
    float output;
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
            DS_BalanceStop();
        }
        balance_control_had_valid_vision = 0U;
        balance_control_state.stable = 0U;
        balance_control_state.stable_count = 0U;
        balance_control_state.pid_output = 0.0f;
        balance_control_state.motor_command = 0;
        balance_control_state.last_motor_status = HAL_TIMEOUT;
        PID_Cycle_Reset(&balance_position_pid);
        return;
    }

    balance_control_had_valid_vision = 1U;
    balance_control_state.ball_position = ds->ball_position;
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
        balance_control_config.stable_error) {
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
    balance_control_state.pid_output = 0.0f;
    balance_control_state.motor_command = 0;
    balance_control_state.stable = 0U;
    balance_control_state.stable_count = 0U;
    balance_control_had_valid_vision = 0U;
    PID_Cycle_SetEnable(&balance_position_pid, 0U);
    PID_Cycle_Reset(&balance_position_pid);
    DS_BalanceStop();
}

uint8_t BalanceControl_IsStable(void)
{
    return balance_control_state.stable;
}
