#include "BalanceTuning.h"

#include "BalanceControl.h"
#include "BallVision.h"

typedef enum
{
    BALANCE_TUNING_COMMAND_NONE = 0,
    BALANCE_TUNING_COMMAND_GET,
    BALANCE_TUNING_COMMAND_SET,
    BALANCE_TUNING_COMMAND_RESET,
    BALANCE_TUNING_COMMAND_TEST,
    BALANCE_TUNING_COMMAND_STOP,
    BALANCE_TUNING_COMMAND_REJECT
} BalanceTuningCommandType;

typedef struct
{
    float outer_kp;
    float outer_kd;
    float angle_limit;
    float inner_kp;
    float inner_kd;
    float speed_limit;
    float slew;
    float deadband;
    float min_speed;
} BalanceTuningParameters;

typedef struct
{
    uint8_t type;
    uint8_t status;
    uint8_t test_mode;
    uint32_t sequence;
    uint32_t mask;
    uint32_t duration_ms;
    float test_target;
    BalanceTuningParameters parameters;
} BalanceTuningCommand;

static BalanceTuningParameters balance_tuning_defaults;
static volatile BalanceTuningCommand balance_tuning_pending;
static volatile uint8_t balance_tuning_pending_valid;
static BallVisionTuningAck balance_tuning_ack;
static uint8_t balance_tuning_ack_pending;
static volatile uint8_t balance_tuning_feedback_v2;
static volatile uint8_t balance_tuning_mode;
static float balance_tuning_test_target;
static uint32_t balance_tuning_test_started_ms;
static uint32_t balance_tuning_test_duration_ms;

static void BalanceTuning_ReadCurrent(BalanceTuningParameters *parameters)
{
    parameters->outer_kp = balance_control_config.outer_kp_deg_per_px;
    parameters->outer_kd = balance_control_config.outer_kd_deg_per_px_s;
    parameters->angle_limit = balance_control_config.outer_angle_limit_deg;
    parameters->inner_kp = balance_control_config.angle_kp_speed_per_deg;
    parameters->inner_kd = balance_control_config.angle_kd_speed_per_deg_s;
    parameters->speed_limit = balance_control_config.motor_speed_limit;
    parameters->slew = balance_control_config.motor_slew_per_update;
    parameters->deadband = balance_control_config.motor_speed_deadband;
    parameters->min_speed = balance_control_config.motor_min_speed;
}

static void BalanceTuning_WriteCurrent(
    const BalanceTuningParameters *parameters)
{
    balance_control_config.outer_kp_deg_per_px = parameters->outer_kp;
    balance_control_config.outer_kd_deg_per_px_s = parameters->outer_kd;
    balance_control_config.outer_angle_limit_deg = parameters->angle_limit;
    balance_control_config.angle_kp_speed_per_deg = parameters->inner_kp;
    balance_control_config.angle_kd_speed_per_deg_s = parameters->inner_kd;
    balance_control_config.motor_speed_limit = parameters->speed_limit;
    balance_control_config.motor_slew_per_update = parameters->slew;
    balance_control_config.motor_speed_deadband = parameters->deadband;
    balance_control_config.motor_min_speed = parameters->min_speed;
}

static uint8_t BalanceTuning_ParametersValid(
    const BalanceTuningParameters *parameters)
{
    float rod_limit = balance_control_config.rod_angle_limit_deg;

    if (rod_limit < 0.0f) {
        rod_limit = -rod_limit;
    }
    return (parameters->outer_kp >= 0.0f &&
            parameters->outer_kp <= 1.0f &&
            parameters->outer_kd >= 0.0f &&
            parameters->outer_kd <= 1.0f &&
            parameters->angle_limit > 0.0f &&
            parameters->angle_limit <= rod_limit &&
            parameters->inner_kp >= 0.0f &&
            parameters->inner_kp <= 500.0f &&
            parameters->inner_kd >= 0.0f &&
            parameters->inner_kd <= 100.0f &&
            parameters->speed_limit >= 1.0f &&
            parameters->speed_limit <= 500.0f &&
            parameters->slew > 0.0f &&
            parameters->slew <= 500.0f &&
            parameters->deadband >= 0.0f &&
            parameters->deadband <= parameters->speed_limit &&
            parameters->min_speed >= 0.0f &&
            parameters->min_speed <= parameters->speed_limit) ? 1U : 0U;
}

static uint8_t BalanceTuning_ParseUnsigned(const char **text,
                                            uint32_t *value)
{
    const char *cursor = *text;
    uint32_t result = 0U;
    uint8_t has_digit = 0U;

    while (*cursor >= '0' && *cursor <= '9') {
        has_digit = 1U;
        result = result * 10U + (uint32_t)(*cursor - '0');
        cursor++;
    }
    if (has_digit == 0U) {
        return 0U;
    }
    *text = cursor;
    *value = result;
    return 1U;
}

static uint8_t BalanceTuning_ParseFloat(const char **text, float *value)
{
    const char *cursor = *text;
    float result = 0.0f;
    float fraction = 0.1f;
    float sign = 1.0f;
    uint8_t has_digit = 0U;

    if (*cursor == '-') {
        sign = -1.0f;
        cursor++;
    } else if (*cursor == '+') {
        cursor++;
    }
    while (*cursor >= '0' && *cursor <= '9') {
        has_digit = 1U;
        result = result * 10.0f + (float)(*cursor - '0');
        cursor++;
    }
    if (*cursor == '.') {
        cursor++;
        while (*cursor >= '0' && *cursor <= '9') {
            has_digit = 1U;
            result += (float)(*cursor - '0') * fraction;
            fraction *= 0.1f;
            cursor++;
        }
    }
    if (has_digit == 0U) {
        return 0U;
    }
    *text = cursor;
    *value = sign * result;
    return 1U;
}

static uint8_t BalanceTuning_ParseComma(const char **text)
{
    if (**text != ',') {
        return 0U;
    }
    (*text)++;
    return 1U;
}

static uint8_t BalanceTuning_ParseSetValues(
    const char **text,
    uint32_t mask,
    BalanceTuningParameters *parameters)
{
    float *values[9];
    uint8_t index;

    values[0] = &parameters->outer_kp;
    values[1] = &parameters->outer_kd;
    values[2] = &parameters->angle_limit;
    values[3] = &parameters->inner_kp;
    values[4] = &parameters->inner_kd;
    values[5] = &parameters->speed_limit;
    values[6] = &parameters->slew;
    values[7] = &parameters->deadband;
    values[8] = &parameters->min_speed;

    for (index = 0U; index < 9U; index++) {
        if ((mask & (1UL << index)) != 0U) {
            if (BalanceTuning_ParseComma(text) == 0U ||
                BalanceTuning_ParseFloat(text, values[index]) == 0U) {
                return 0U;
            }
        }
    }
    return (**text == '\0') ? 1U : 0U;
}

static void BalanceTuning_QueueAck(uint32_t sequence, uint8_t status)
{
    BalanceTuningParameters current;

    BalanceTuning_ReadCurrent(&current);
    balance_tuning_ack.sequence = sequence;
    balance_tuning_ack.status = status;
    balance_tuning_ack.outer_kp = current.outer_kp;
    balance_tuning_ack.outer_kd = current.outer_kd;
    balance_tuning_ack.angle_limit = current.angle_limit;
    balance_tuning_ack.inner_kp = current.inner_kp;
    balance_tuning_ack.inner_kd = current.inner_kd;
    balance_tuning_ack.speed_limit = current.speed_limit;
    balance_tuning_ack.slew = current.slew;
    balance_tuning_ack.deadband = current.deadband;
    balance_tuning_ack.min_speed = current.min_speed;
    balance_tuning_ack.mode = balance_tuning_mode;
    balance_tuning_ack.test_target = balance_tuning_test_target;
    balance_tuning_ack.remaining_ms = BalanceTuning_GetRemainingMs(
        HAL_GetTick());
    balance_tuning_ack_pending = 1U;
}

void BalanceTuning_Init(void)
{
    BalanceTuning_ReadCurrent(&balance_tuning_defaults);
    balance_tuning_pending_valid = 0U;
    balance_tuning_ack_pending = 0U;
    balance_tuning_feedback_v2 = 0U;
    balance_tuning_mode = BALANCE_TUNING_MODE_NORMAL;
    balance_tuning_test_target = 0.0f;
    balance_tuning_test_started_ms = 0U;
    balance_tuning_test_duration_ms = 0U;
}

void BalanceTuning_OnControllerStart(void)
{
    balance_tuning_pending_valid = 0U;
    balance_tuning_ack_pending = 0U;
    balance_tuning_mode = BALANCE_TUNING_MODE_NORMAL;
    balance_tuning_test_target = 0.0f;
    balance_tuning_test_started_ms = 0U;
    balance_tuning_test_duration_ms = 0U;
}

uint8_t BalanceTuning_ProcessLineFromISR(const char *line)
{
    BalanceTuningCommand command = {0};
    const char *text;

    if (line == NULL || line[0] != 'P' || balance_tuning_pending_valid != 0U) {
        return 0U;
    }
    text = &line[2];
    if (line[1] == 'G') {
        command.type = BALANCE_TUNING_COMMAND_GET;
    } else if (line[1] == 'S') {
        command.type = BALANCE_TUNING_COMMAND_SET;
    } else if (line[1] == 'R') {
        command.type = BALANCE_TUNING_COMMAND_RESET;
    } else if (line[1] == 'T') {
        command.type = BALANCE_TUNING_COMMAND_TEST;
    } else if (line[1] == 'X') {
        command.type = BALANCE_TUNING_COMMAND_STOP;
    } else {
        return 0U;
    }

    if (BalanceTuning_ParseComma(&text) == 0U ||
        BalanceTuning_ParseUnsigned(&text, &command.sequence) == 0U) {
        return 0U;
    }

    if (command.type == BALANCE_TUNING_COMMAND_SET) {
        BalanceTuning_ReadCurrent(&command.parameters);
        if (BalanceTuning_ParseComma(&text) == 0U ||
            BalanceTuning_ParseUnsigned(&text, &command.mask) == 0U ||
            command.mask == 0U ||
            (command.mask & ~BALANCE_TUNING_ALL) != 0U ||
            BalanceTuning_ParseSetValues(
                &text, command.mask, &command.parameters) == 0U ||
            BalanceTuning_ParametersValid(&command.parameters) == 0U) {
            command.type = BALANCE_TUNING_COMMAND_REJECT;
            command.status = 1U;
        }
    } else if (command.type == BALANCE_TUNING_COMMAND_TEST) {
        if (BalanceTuning_ParseComma(&text) == 0U ||
            (*text != 'I' && *text != 'O')) {
            command.type = BALANCE_TUNING_COMMAND_REJECT;
            command.status = 1U;
        } else {
            command.test_mode = (*text == 'I') ?
                BALANCE_TUNING_MODE_INNER_STEP :
                BALANCE_TUNING_MODE_OUTER_STEP;
            text++;
            if (BalanceTuning_ParseComma(&text) == 0U ||
                BalanceTuning_ParseFloat(&text, &command.test_target) == 0U ||
                BalanceTuning_ParseComma(&text) == 0U ||
                BalanceTuning_ParseUnsigned(&text, &command.duration_ms) == 0U ||
                *text != '\0' || command.duration_ms < 200U ||
                command.duration_ms > 15000U) {
                command.type = BALANCE_TUNING_COMMAND_REJECT;
                command.status = 1U;
            } else if (command.test_mode == BALANCE_TUNING_MODE_INNER_STEP &&
                       (command.test_target <
                            -balance_control_config.outer_angle_limit_deg ||
                        command.test_target >
                            balance_control_config.outer_angle_limit_deg)) {
                command.type = BALANCE_TUNING_COMMAND_REJECT;
                command.status = 1U;
            }
        }
    } else if (*text != '\0') {
        command.type = BALANCE_TUNING_COMMAND_REJECT;
        command.status = 1U;
    }

    balance_tuning_pending = command;
    balance_tuning_pending_valid = 1U;
    balance_tuning_feedback_v2 = 1U;
    return 1U;
}

uint8_t BalanceTuning_ApplyPendingAtControlBoundary(uint32_t now_ms)
{
    BalanceTuningCommand command;
    uint32_t primask;
    uint8_t action = 0U;

    BalanceTuning_Update(now_ms);
    BalanceTuning_FlushAck();
    if (balance_tuning_pending_valid == 0U) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    command = balance_tuning_pending;
    balance_tuning_pending_valid = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    if (command.type == BALANCE_TUNING_COMMAND_SET) {
        BalanceTuning_WriteCurrent(&command.parameters);
        action = BALANCE_TUNING_ACTION_RESET | BALANCE_TUNING_ACTION_STOP;
    } else if (command.type == BALANCE_TUNING_COMMAND_RESET) {
        BalanceTuning_WriteCurrent(&balance_tuning_defaults);
        balance_tuning_mode = BALANCE_TUNING_MODE_NORMAL;
        action = BALANCE_TUNING_ACTION_RESET | BALANCE_TUNING_ACTION_STOP;
    } else if (command.type == BALANCE_TUNING_COMMAND_TEST) {
        balance_tuning_mode = command.test_mode;
        balance_tuning_test_target = command.test_target;
        balance_tuning_test_started_ms = now_ms;
        balance_tuning_test_duration_ms = command.duration_ms;
        action = BALANCE_TUNING_ACTION_RESET | BALANCE_TUNING_ACTION_STOP;
    } else if (command.type == BALANCE_TUNING_COMMAND_STOP) {
        balance_tuning_mode = BALANCE_TUNING_MODE_PAUSED;
        balance_tuning_test_target = 0.0f;
        balance_tuning_test_duration_ms = 0U;
        action = BALANCE_TUNING_ACTION_STOP;
    }

    BalanceTuning_QueueAck(command.sequence, command.status);
    return action;
}

void BalanceTuning_Update(uint32_t now_ms)
{
    if ((balance_tuning_mode == BALANCE_TUNING_MODE_INNER_STEP ||
         balance_tuning_mode == BALANCE_TUNING_MODE_OUTER_STEP) &&
        (uint32_t)(now_ms - balance_tuning_test_started_ms) >=
            balance_tuning_test_duration_ms) {
        balance_tuning_mode = BALANCE_TUNING_MODE_PAUSED;
        balance_tuning_test_target = 0.0f;
        balance_tuning_test_duration_ms = 0U;
    }
}

void BalanceTuning_FlushAck(void)
{
    if (balance_tuning_ack_pending != 0U &&
        BallVision_SendTuningAck(&balance_tuning_ack) == HAL_OK) {
        balance_tuning_ack_pending = 0U;
    }
}

uint8_t BalanceTuning_FeedbackV2Enabled(void)
{
    return balance_tuning_feedback_v2;
}

uint8_t BalanceTuning_GetInnerTarget(float *target_angle_deg)
{
    if (balance_tuning_mode != BALANCE_TUNING_MODE_INNER_STEP ||
        target_angle_deg == NULL) {
        return 0U;
    }
    *target_angle_deg = balance_tuning_test_target;
    return 1U;
}

uint8_t BalanceTuning_GetOuterTarget(float *target_position_px)
{
    if (balance_tuning_mode != BALANCE_TUNING_MODE_OUTER_STEP ||
        target_position_px == NULL) {
        return 0U;
    }
    *target_position_px = balance_tuning_test_target;
    return 1U;
}

uint8_t BalanceTuning_ForceStop(void)
{
    return (balance_tuning_mode == BALANCE_TUNING_MODE_PAUSED) ? 1U : 0U;
}

uint8_t BalanceTuning_GetMode(void)
{
    return balance_tuning_mode;
}

float BalanceTuning_GetTestTarget(void)
{
    return balance_tuning_test_target;
}

uint32_t BalanceTuning_GetRemainingMs(uint32_t now_ms)
{
    uint32_t elapsed;

    if (balance_tuning_mode != BALANCE_TUNING_MODE_INNER_STEP &&
        balance_tuning_mode != BALANCE_TUNING_MODE_OUTER_STEP) {
        return 0U;
    }
    elapsed = (uint32_t)(now_ms - balance_tuning_test_started_ms);
    return (elapsed < balance_tuning_test_duration_ms) ?
        (balance_tuning_test_duration_ms - elapsed) : 0U;
}
