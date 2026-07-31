#include "BallVision.h"

#include "DS.h"
#include "BalanceTuning.h"

#include <limits.h>

#define BALL_VISION_LINE_LENGTH       160U
#define BALL_VISION_FEEDBACK_LENGTH   224U
#define BALL_VISION_STOP_TX_WAIT_MS   20U

static UART_HandleTypeDef *ball_vision_huart;
static uint8_t ball_vision_rx_byte;
static char ball_vision_rx_line[BALL_VISION_LINE_LENGTH];
static uint8_t ball_vision_rx_index;
static uint8_t ball_vision_feedback_tx_buffer[BALL_VISION_FEEDBACK_LENGTH];
static volatile uint8_t ball_vision_feedback_tx_busy;

volatile uint32_t ball_vision_parse_error_count;
volatile uint8_t ball_vision_stream_active;
volatile uint32_t ball_vision_feedback_attempt_count;
volatile uint32_t ball_vision_feedback_sent_count;
volatile uint32_t ball_vision_feedback_drop_count;

static int32_t BallVision_ScaleFloat(float value, float scale)
{
    float scaled = value * scale;

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

static uint8_t BallVision_AppendChar(uint16_t *length, uint8_t character)
{
    if (*length >= (BALL_VISION_FEEDBACK_LENGTH - 1U)) {
        return 0U;
    }
    ball_vision_feedback_tx_buffer[*length] = character;
    (*length)++;
    return 1U;
}

static uint8_t BallVision_AppendUnsigned(uint16_t *length, uint32_t value)
{
    uint8_t digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (uint8_t)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (count > 0U) {
        if (BallVision_AppendChar(length, digits[--count]) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t BallVision_AppendSigned(uint16_t *length, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        if (BallVision_AppendChar(length, '-') == 0U) {
            return 0U;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    return BallVision_AppendUnsigned(length, magnitude);
}

static uint8_t BallVision_AppendUnsignedField(uint16_t *length,
                                              uint32_t value)
{
    return (BallVision_AppendChar(length, ',') != 0U &&
            BallVision_AppendUnsigned(length, value) != 0U) ? 1U : 0U;
}

static uint8_t BallVision_AppendSignedField(uint16_t *length, int32_t value)
{
    return (BallVision_AppendChar(length, ',') != 0U &&
            BallVision_AppendSigned(length, value) != 0U) ? 1U : 0U;
}

static void BallVision_WaitForFeedbackTx(void)
{
    uint32_t start_ms = HAL_GetTick();

    while (ball_vision_feedback_tx_busy != 0U &&
           (uint32_t)(HAL_GetTick() - start_ms) <
               BALL_VISION_STOP_TX_WAIT_MS) {
        /* USART6 TX completion interrupt clears the busy flag. */
    }

    if (ball_vision_feedback_tx_busy != 0U &&
        ball_vision_huart != NULL) {
        (void)HAL_UART_AbortTransmit(ball_vision_huart);
        ball_vision_feedback_tx_busy = 0U;
    }
}

static uint8_t BallVision_ParseFloat(const char **text, float *value)
{
    const char *cursor;
    float result = 0.0f;
    float fraction_scale = 0.1f;
    float sign = 1.0f;
    uint8_t has_digit = 0U;

    if (text == NULL || *text == NULL || value == NULL) {
        return 0U;
    }

    cursor = *text;
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }

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
            result += (float)(*cursor - '0') * fraction_scale;
            fraction_scale *= 0.1f;
            cursor++;
        }
    }

    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }

    if (has_digit == 0U) {
        return 0U;
    }

    *value = sign * result;
    *text = cursor;
    return 1U;
}

static void BallVision_ProcessLine(char *line)
{
    const char *text;
    float position_error;
    float velocity;

    if (line[0] == 'n' && line[1] == 'o' && line[2] == 'n' &&
        line[3] == 'e' && line[4] == '\0') {
        DS_BallVisionUpdateFromISR(0.0f, 0.0f, 0U);
        return;
    }

    if (line[0] == 'P') {
        if (BalanceTuning_ProcessLineFromISR(line) == 0U) {
            ball_vision_parse_error_count++;
        }
        return;
    }

    if (line[0] != 'B' || line[1] != ',') {
        ball_vision_parse_error_count++;
        return;
    }

    text = &line[2];
    if (BallVision_ParseFloat(&text, &position_error) == 0U ||
        *text != ',') {
        ball_vision_parse_error_count++;
        return;
    }

    text++;
    if (BallVision_ParseFloat(&text, &velocity) == 0U ||
        *text != '\0') {
        ball_vision_parse_error_count++;
        return;
    }

    DS_BallVisionUpdateFromISR(position_error, velocity, 1U);
}

void BallVision_Init(UART_HandleTypeDef *huart)
{
    ball_vision_huart = huart;
    ball_vision_rx_index = 0U;
    ball_vision_parse_error_count = 0U;
    ball_vision_stream_active = 0U;
    ball_vision_feedback_tx_busy = 0U;
    ball_vision_feedback_attempt_count = 0U;
    ball_vision_feedback_sent_count = 0U;
    ball_vision_feedback_drop_count = 0U;
}

void BallVision_StartStream(void)
{
    uint8_t command[] = {'c', '2'};

    if (ball_vision_huart == NULL) {
        return;
    }

    ball_vision_rx_index = 0U;
    DS_BallVisionUpdateFromISR(0.0f, 0.0f, 0U);
    (void)HAL_UART_AbortReceive(ball_vision_huart);
    (void)HAL_UART_AbortTransmit(ball_vision_huart);
    __HAL_UART_CLEAR_OREFLAG(ball_vision_huart);
    ball_vision_stream_active = 1U;
    ball_vision_feedback_tx_busy = 0U;
    ball_vision_feedback_attempt_count = 0U;
    ball_vision_feedback_sent_count = 0U;
    ball_vision_feedback_drop_count = 0U;

    if (HAL_UART_Receive_IT(ball_vision_huart,
                           &ball_vision_rx_byte,
                           1U) != HAL_OK) {
        ball_vision_stream_active = 0U;
        return;
    }

    (void)HAL_UART_Transmit(ball_vision_huart,
                            command,
                            sizeof(command),
                            100U);
}

void BallVision_StopStream(void)
{
    uint8_t command[] = {'o', 'k'};

    if (ball_vision_huart != NULL) {
        BallVision_WaitForFeedbackTx();
        (void)HAL_UART_Transmit(ball_vision_huart,
                                command,
                                sizeof(command),
                                100U);
        ball_vision_stream_active = 0U;
        (void)HAL_UART_AbortReceive(ball_vision_huart);
    }

    ball_vision_rx_index = 0U;
    DS_BallVisionUpdateFromISR(0.0f, 0.0f, 0U);
}

HAL_StatusTypeDef BallVision_SendFeedback(uint32_t vision_frame,
                                          uint32_t vision_age_ms,
                                          float position,
                                          float velocity,
                                          float control_error,
                                          float p_term,
                                          float i_term,
                                          float d_term,
                                          int32_t motor_command,
                                          HAL_StatusTypeDef motor_status)
{
    HAL_StatusTypeDef status;
    uint16_t length = 0U;
    uint32_t sequence;

    if (ball_vision_huart == NULL ||
        ball_vision_stream_active == 0U) {
        return HAL_ERROR;
    }

    sequence = ++ball_vision_feedback_attempt_count;
    if (ball_vision_feedback_tx_busy != 0U) {
        ball_vision_feedback_drop_count++;
        return HAL_BUSY;
    }

    if (BallVision_AppendChar(&length, 'F') == 0U ||
        BallVision_AppendUnsignedField(&length, sequence) == 0U ||
        BallVision_AppendUnsignedField(&length, HAL_GetTick()) == 0U ||
        BallVision_AppendUnsignedField(&length, vision_frame) == 0U ||
        BallVision_AppendUnsignedField(&length, vision_age_ms) == 0U ||
        BallVision_AppendSignedField(
            &length, BallVision_ScaleFloat(position, 10.0f)) == 0U ||
        BallVision_AppendSignedField(
            &length, BallVision_ScaleFloat(velocity, 10.0f)) == 0U ||
        BallVision_AppendSignedField(
            &length, BallVision_ScaleFloat(control_error, 10.0f)) == 0U ||
        BallVision_AppendSignedField(
            &length, BallVision_ScaleFloat(p_term, 100.0f)) == 0U ||
        BallVision_AppendSignedField(
            &length, BallVision_ScaleFloat(i_term, 100.0f)) == 0U ||
        BallVision_AppendSignedField(
            &length, BallVision_ScaleFloat(d_term, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length, motor_command) == 0U ||
        BallVision_AppendUnsignedField(
            &length, (uint32_t)motor_status) == 0U ||
        BallVision_AppendChar(&length, '\n') == 0U) {
        ball_vision_feedback_drop_count++;
        return HAL_ERROR;
    }

    ball_vision_feedback_tx_busy = 1U;
    status = HAL_UART_Transmit_IT(
        ball_vision_huart,
        ball_vision_feedback_tx_buffer,
        length);
    if (status != HAL_OK) {
        ball_vision_feedback_tx_busy = 0U;
        ball_vision_feedback_drop_count++;
        return status;
    }

    ball_vision_feedback_sent_count++;
    return HAL_OK;
}

HAL_StatusTypeDef BallVision_SendFeedbackV2(
    const BallVisionFeedbackV2 *feedback)
{
    HAL_StatusTypeDef status;
    uint16_t length = 0U;
    uint32_t sequence;

    if (feedback == NULL || ball_vision_huart == NULL ||
        ball_vision_stream_active == 0U) {
        return HAL_ERROR;
    }
    sequence = ++ball_vision_feedback_attempt_count;
    if (ball_vision_feedback_tx_busy != 0U) {
        ball_vision_feedback_drop_count++;
        return HAL_BUSY;
    }

    if (BallVision_AppendChar(&length, 'F') == 0U ||
        BallVision_AppendChar(&length, '2') == 0U ||
        BallVision_AppendUnsignedField(&length, sequence) == 0U ||
        BallVision_AppendUnsignedField(&length, HAL_GetTick()) == 0U ||
        BallVision_AppendUnsignedField(&length, feedback->vision_frame) == 0U ||
        BallVision_AppendUnsignedField(&length, feedback->vision_age_ms) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->position, 10.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->velocity, 10.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->control_error, 10.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->p_term, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->i_term, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->d_term, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->target_rod_angle, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->actual_rod_angle, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->rod_rate, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->angle_error, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(feedback->desired_speed, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length, feedback->motor_command) == 0U ||
        BallVision_AppendUnsignedField(&length,
            feedback->position_age_ms) == 0U ||
        BallVision_AppendUnsignedField(&length,
            feedback->position_valid) == 0U ||
        BallVision_AppendUnsignedField(&length,
            feedback->protection_state) == 0U ||
        BallVision_AppendUnsignedField(&length,
            (uint32_t)feedback->motor_status) == 0U ||
        BallVision_AppendUnsignedField(&length,
            feedback->tuning_mode) == 0U ||
        BallVision_AppendChar(&length, '\n') == 0U) {
        ball_vision_feedback_drop_count++;
        return HAL_ERROR;
    }

    ball_vision_feedback_tx_busy = 1U;
    status = HAL_UART_Transmit_IT(
        ball_vision_huart, ball_vision_feedback_tx_buffer, length);
    if (status != HAL_OK) {
        ball_vision_feedback_tx_busy = 0U;
        ball_vision_feedback_drop_count++;
        return status;
    }
    ball_vision_feedback_sent_count++;
    return HAL_OK;
}

HAL_StatusTypeDef BallVision_SendTuningAck(
    const BallVisionTuningAck *ack)
{
    HAL_StatusTypeDef status;
    uint16_t length = 0U;

    if (ack == NULL || ball_vision_huart == NULL ||
        ball_vision_stream_active == 0U) {
        return HAL_ERROR;
    }
    if (ball_vision_feedback_tx_busy != 0U) {
        return HAL_BUSY;
    }

    if (BallVision_AppendChar(&length, 'P') == 0U ||
        BallVision_AppendChar(&length, 'A') == 0U ||
        BallVision_AppendUnsignedField(&length, ack->sequence) == 0U ||
        BallVision_AppendUnsignedField(&length, ack->status) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->outer_kp, 1000000.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->outer_kd, 1000000.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->angle_limit, 1000.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->inner_kp, 1000.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->inner_kd, 1000.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->speed_limit, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->slew, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->deadband, 100.0f)) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->min_speed, 100.0f)) == 0U ||
        BallVision_AppendUnsignedField(&length, ack->mode) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->test_target, 100.0f)) == 0U ||
        BallVision_AppendUnsignedField(&length, ack->remaining_ms) == 0U ||
        BallVision_AppendSignedField(&length,
            BallVision_ScaleFloat(ack->outer_ki, 1000000.0f)) == 0U ||
        BallVision_AppendChar(&length, '\n') == 0U) {
        return HAL_ERROR;
    }

    ball_vision_feedback_tx_busy = 1U;
    status = HAL_UART_Transmit_IT(
        ball_vision_huart, ball_vision_feedback_tx_buffer, length);
    if (status != HAL_OK) {
        ball_vision_feedback_tx_busy = 0U;
    }
    return status;
}

void BallVision_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (ball_vision_huart == NULL ||
        huart->Instance != ball_vision_huart->Instance ||
        ball_vision_stream_active == 0U) {
        return;
    }

    if (ball_vision_rx_byte == '\n') {
        ball_vision_rx_line[ball_vision_rx_index] = '\0';
        BallVision_ProcessLine(ball_vision_rx_line);
        ball_vision_rx_index = 0U;
    } else if (ball_vision_rx_byte != '\r') {
        if (ball_vision_rx_index <
            (sizeof(ball_vision_rx_line) - 1U)) {
            ball_vision_rx_line[ball_vision_rx_index++] =
                (char)ball_vision_rx_byte;
        } else {
            ball_vision_rx_index = 0U;
            ball_vision_parse_error_count++;
        }
    }

    if (ball_vision_stream_active != 0U) {
        (void)HAL_UART_Receive_IT(ball_vision_huart,
                                 &ball_vision_rx_byte,
                                 1U);
    }
}

void BallVision_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (ball_vision_huart != NULL &&
        huart->Instance == ball_vision_huart->Instance) {
        ball_vision_feedback_tx_busy = 0U;
    }
}

void BallVision_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (ball_vision_huart == NULL ||
        huart->Instance != ball_vision_huart->Instance ||
        ball_vision_stream_active == 0U) {
        return;
    }

    ball_vision_rx_index = 0U;
    (void)HAL_UART_AbortReceive_IT(ball_vision_huart);
    __HAL_UART_CLEAR_OREFLAG(ball_vision_huart);
    (void)HAL_UART_Receive_IT(ball_vision_huart,
                             &ball_vision_rx_byte,
                             1U);
}
