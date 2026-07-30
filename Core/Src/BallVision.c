#include "BallVision.h"

#include "DS.h"

#define BALL_VISION_LINE_LENGTH       32U

static UART_HandleTypeDef *ball_vision_huart;
static uint8_t ball_vision_rx_byte;
static char ball_vision_rx_line[BALL_VISION_LINE_LENGTH];
static uint8_t ball_vision_rx_index;

volatile uint32_t ball_vision_parse_error_count;

static uint8_t BallVision_ParseFloat(const char *text, float *value)
{
    float result = 0.0f;
    float fraction_scale = 0.1f;
    float sign = 1.0f;
    uint8_t has_digit = 0U;

    if (text == NULL || value == NULL) {
        return 0U;
    }

    while (*text == ' ' || *text == '\t') {
        text++;
    }

    if (*text == '-') {
        sign = -1.0f;
        text++;
    } else if (*text == '+') {
        text++;
    }

    while (*text >= '0' && *text <= '9') {
        has_digit = 1U;
        result = result * 10.0f + (float)(*text - '0');
        text++;
    }

    if (*text == '.') {
        text++;
        while (*text >= '0' && *text <= '9') {
            has_digit = 1U;
            result += (float)(*text - '0') * fraction_scale;
            fraction_scale *= 0.1f;
            text++;
        }
    }

    while (*text == ' ' || *text == '\t') {
        text++;
    }

    if (has_digit == 0U || *text != '\0') {
        return 0U;
    }

    *value = sign * result;
    return 1U;
}

static void BallVision_ProcessLine(char *line)
{
    float position;

    if (line[0] == 'n' && line[1] == 'o' && line[2] == 'n' &&
        line[3] == 'e' && line[4] == '\0') {
        DS_BallVisionUpdateFromISR(0.0f, 0U);
        return;
    }

    if (BallVision_ParseFloat(line, &position) == 0U) {
        ball_vision_parse_error_count++;
        return;
    }

    DS_BallVisionUpdateFromISR(position, 1U);
}

void BallVision_Init(UART_HandleTypeDef *huart)
{
    ball_vision_huart = huart;
    ball_vision_rx_index = 0U;
    ball_vision_parse_error_count = 0U;

    if (ball_vision_huart != NULL) {
        (void)HAL_UART_Receive_IT(ball_vision_huart,
                                 &ball_vision_rx_byte,
                                 1U);
    }
}

void BallVision_StartStream(void)
{
    uint8_t command[] = {'c', '2'};

    if (ball_vision_huart != NULL) {
        (void)HAL_UART_Transmit(ball_vision_huart,
                                command,
                                sizeof(command),
                                100U);
    }
}

void BallVision_StopStream(void)
{
    uint8_t command[] = {'o', 'k'};

    if (ball_vision_huart != NULL) {
        (void)HAL_UART_Transmit(ball_vision_huart,
                                command,
                                sizeof(command),
                                100U);
    }
}

void BallVision_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (ball_vision_huart == NULL ||
        huart->Instance != ball_vision_huart->Instance) {
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

    (void)HAL_UART_Receive_IT(ball_vision_huart,
                             &ball_vision_rx_byte,
                             1U);
}

void BallVision_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (ball_vision_huart == NULL ||
        huart->Instance != ball_vision_huart->Instance) {
        return;
    }

    ball_vision_rx_index = 0U;
    (void)HAL_UART_AbortReceive_IT(ball_vision_huart);
    __HAL_UART_CLEAR_OREFLAG(ball_vision_huart);
    (void)HAL_UART_Receive_IT(ball_vision_huart,
                             &ball_vision_rx_byte,
                             1U);
}
