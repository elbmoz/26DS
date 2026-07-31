#include "Question9Telemetry.h"

#include <limits.h>

#define QUESTION9_TELEMETRY_TX_LENGTH    128U

static UART_HandleTypeDef *question9_telemetry_huart;
static uint8_t question9_telemetry_tx_buffer[
    QUESTION9_TELEMETRY_TX_LENGTH];

Question9TelemetryState question9_telemetry_state;

static int32_t Question9Telemetry_ScaleAngle(float angle_deg)
{
    float scaled = angle_deg * 10.0f;

    if (scaled >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled <= (float)INT32_MIN) {
        return INT32_MIN;
    }

    return (scaled >= 0.0f) ?
           (int32_t)(scaled + 0.5f) :
           (int32_t)(scaled - 0.5f);
}

static uint8_t Question9Telemetry_AppendChar(uint16_t *length,
                                             uint8_t character)
{
    if (*length >= (QUESTION9_TELEMETRY_TX_LENGTH - 1U)) {
        return 0U;
    }

    question9_telemetry_tx_buffer[*length] = character;
    (*length)++;
    return 1U;
}

static uint8_t Question9Telemetry_AppendUnsigned(uint16_t *length,
                                                 uint32_t value)
{
    uint8_t digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (uint8_t)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (count > 0U) {
        if (Question9Telemetry_AppendChar(
                length,
                digits[--count]) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t Question9Telemetry_AppendSigned(uint16_t *length,
                                               int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        if (Question9Telemetry_AppendChar(length, '-') == 0U) {
            return 0U;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    return Question9Telemetry_AppendUnsigned(length, magnitude);
}

static uint8_t Question9Telemetry_AppendUnsignedField(
    uint16_t *length,
    uint32_t value)
{
    return (Question9Telemetry_AppendChar(length, ',') != 0U &&
            Question9Telemetry_AppendUnsigned(length, value) != 0U) ?
           1U :
           0U;
}

static uint8_t Question9Telemetry_AppendSignedField(uint16_t *length,
                                                    int32_t value)
{
    return (Question9Telemetry_AppendChar(length, ',') != 0U &&
            Question9Telemetry_AppendSigned(length, value) != 0U) ?
           1U :
           0U;
}

static uint8_t Question9Telemetry_BuildFrame(
    const Question9TelemetrySnapshot *snapshot,
    uint32_t sequence,
    uint16_t *length)
{
    *length = 0U;

    return (
        Question9Telemetry_AppendChar(length, 'Q') != 0U &&
        Question9Telemetry_AppendChar(length, '9') != 0U &&
        Question9Telemetry_AppendUnsignedField(length, sequence) != 0U &&
        Question9Telemetry_AppendUnsignedField(
            length,
            HAL_GetTick()) != 0U &&
        Question9Telemetry_AppendSignedField(
            length,
            snapshot->motor_position) != 0U &&
        Question9Telemetry_AppendSignedField(
            length,
            Question9Telemetry_ScaleAngle(
                snapshot->angle_x_deg)) != 0U &&
        Question9Telemetry_AppendSignedField(
            length,
            Question9Telemetry_ScaleAngle(
                snapshot->angle_y_deg)) != 0U &&
        Question9Telemetry_AppendSignedField(
            length,
            Question9Telemetry_ScaleAngle(
                snapshot->angle_z_deg)) != 0U &&
        Question9Telemetry_AppendUnsignedField(
            length,
            (snapshot->imu_valid != 0U) ? 1U : 0U) != 0U &&
        Question9Telemetry_AppendUnsignedField(
            length,
            (snapshot->position_valid != 0U) ? 1U : 0U) != 0U &&
        Question9Telemetry_AppendUnsignedField(
            length,
            (uint32_t)snapshot->position_status) != 0U &&
        Question9Telemetry_AppendUnsignedField(
            length,
            snapshot->position_updates) != 0U &&
        Question9Telemetry_AppendSignedField(
            length,
            (int32_t)snapshot->move_direction) != 0U &&
        Question9Telemetry_AppendUnsignedField(
            length,
            (uint32_t)snapshot->move_status) != 0U &&
        Question9Telemetry_AppendChar(length, '\n') != 0U) ?
        1U :
        0U;
}

void Question9Telemetry_Init(UART_HandleTypeDef *huart)
{
    question9_telemetry_huart = huart;
    question9_telemetry_state.active = 0U;
    question9_telemetry_state.tx_busy = 0U;
    question9_telemetry_state.attempt_count = 0U;
    question9_telemetry_state.sent_count = 0U;
    question9_telemetry_state.drop_count = 0U;
    question9_telemetry_state.last_attempt_ms = 0U;
    question9_telemetry_state.last_status = HAL_OK;
}

void Question9Telemetry_Start(void)
{
    uint32_t now = HAL_GetTick();

    question9_telemetry_state.active =
        (question9_telemetry_huart != NULL) ? 1U : 0U;
    question9_telemetry_state.tx_busy = 0U;
    question9_telemetry_state.attempt_count = 0U;
    question9_telemetry_state.sent_count = 0U;
    question9_telemetry_state.drop_count = 0U;
    question9_telemetry_state.last_attempt_ms =
        now - QUESTION9_TELEMETRY_PERIOD_MS;
    question9_telemetry_state.last_status =
        (question9_telemetry_huart != NULL) ? HAL_OK : HAL_ERROR;
}

void Question9Telemetry_Stop(void)
{
    question9_telemetry_state.active = 0U;

    if (question9_telemetry_state.tx_busy != 0U &&
        question9_telemetry_huart != NULL) {
        (void)HAL_UART_AbortTransmit(question9_telemetry_huart);
    }

    question9_telemetry_state.tx_busy = 0U;
}

void Question9Telemetry_Update(
    const Question9TelemetrySnapshot *snapshot)
{
    HAL_StatusTypeDef status;
    uint16_t length;
    uint32_t now;
    uint32_t sequence;

    if (question9_telemetry_state.active == 0U ||
        question9_telemetry_huart == NULL ||
        snapshot == NULL) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now -
                   question9_telemetry_state.last_attempt_ms) <
        QUESTION9_TELEMETRY_PERIOD_MS) {
        return;
    }

    question9_telemetry_state.last_attempt_ms = now;
    sequence = ++question9_telemetry_state.attempt_count;

    if (question9_telemetry_state.tx_busy != 0U) {
        question9_telemetry_state.drop_count++;
        question9_telemetry_state.last_status = HAL_BUSY;
        return;
    }

    if (Question9Telemetry_BuildFrame(
            snapshot,
            sequence,
            &length) == 0U) {
        question9_telemetry_state.drop_count++;
        question9_telemetry_state.last_status = HAL_ERROR;
        return;
    }

    question9_telemetry_state.tx_busy = 1U;
    status = HAL_UART_Transmit_IT(
        question9_telemetry_huart,
        question9_telemetry_tx_buffer,
        length);
    question9_telemetry_state.last_status = status;

    if (status != HAL_OK) {
        question9_telemetry_state.tx_busy = 0U;
        question9_telemetry_state.drop_count++;
        return;
    }

    question9_telemetry_state.sent_count++;
}

void Question9Telemetry_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (question9_telemetry_huart != NULL &&
        huart->Instance == question9_telemetry_huart->Instance &&
        question9_telemetry_state.tx_busy != 0U) {
        question9_telemetry_state.tx_busy = 0U;
        question9_telemetry_state.last_status = HAL_OK;
    }
}

void Question9Telemetry_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (question9_telemetry_huart != NULL &&
        huart->Instance == question9_telemetry_huart->Instance &&
        question9_telemetry_state.active != 0U) {
        if (question9_telemetry_state.tx_busy != 0U) {
            question9_telemetry_state.drop_count++;
        }
        question9_telemetry_state.tx_busy = 0U;
        question9_telemetry_state.last_status = HAL_ERROR;
    }
}
