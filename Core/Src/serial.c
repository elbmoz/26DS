#include "serial.h"

#include "DS.h"

static UART_HandleTypeDef *serial_huart;
static uint8_t serial_rx_byte;
static char vision_rx_line[32];
static uint8_t vision_rx_index;

static uint8_t Serial_ParseInt(const char **text, int32_t *value)
{
    int32_t sign = 1;
    int32_t result = 0;
    uint8_t has_digit = 0U;

    while (**text == ' ') {
        (*text)++;
    }

    if (**text == '-') {
        sign = -1;
        (*text)++;
    } else if (**text == '+') {
        (*text)++;
    }

    while (**text >= '0' && **text <= '9') {
        has_digit = 1U;
        result = result * 10 + (int32_t)(**text - '0');
        (*text)++;
    }

    if (has_digit == 0U) {
        return 0U;
    }

    *value = result * sign;
    return 1U;
}

static void Serial_ProcessVisionLine(char *line)
{
    const char *text = line;
    int32_t x_error;
    int32_t y_error;

    if (line[0] == 'n' && line[1] == 'o' && line[2] == 'n' &&
        line[3] == 'e' && line[4] == '\0') {
        DS_VisionUpdateFromISR(0, 0, 0U);
        return;
    }

    if (Serial_ParseInt(&text, &x_error) == 0U) {
        return;
    }

    while (*text == ' ') {
        text++;
    }

    if (*text != ',') {
        return;
    }
    text++;

    if (Serial_ParseInt(&text, &y_error) == 0U) {
        return;
    }

    while (*text == ' ') {
        text++;
    }

    if (*text != '\0') {
        return;
    }

    DS_VisionUpdateFromISR(x_error, y_error, 1U);
}

void Serial_Init(UART_HandleTypeDef *huart)
{
    serial_huart = huart;
    vision_rx_index = 0U;

    if (serial_huart != NULL) {
        (void)HAL_UART_Receive_IT(serial_huart, &serial_rx_byte, 1U);
    }
}

void Serial_SendByte(uint8_t byte)
{
    if (serial_huart != NULL) {
        (void)HAL_UART_Transmit(serial_huart, &byte, 1U, 100U);
    }
}

void Serial_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        Serial_SendByte((uint8_t)*text);
        text++;
    }
}

void Serial_Printf(const char *format, ...)
{
    char buffer[100];
    va_list args;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial_SendString(buffer);
}

void Vision_UART_StartCircle(void)
{
    const uint8_t command = 'j';

    if (serial_huart != NULL) {
        (void)HAL_UART_Transmit(serial_huart, (uint8_t *)&command, 1U, 100U);
    }
}

void Vision_UART_StartTask(uint8_t task)
{
    uint8_t command[] = {'c', '3'};

    if (task >= 1U && task <= 3U) {
        command[1] = (uint8_t)('0' + task);
    }

    if (serial_huart != NULL) {
        (void)HAL_UART_Transmit(serial_huart, command, sizeof(command), 100U);
    }
}

void Vision_UART_StartStream(void)
{
    Vision_UART_StartTask(3U);
}

void Vision_UART_StopStream(void)
{
    uint8_t command[] = {'o', 'k'};

    if (serial_huart != NULL) {
        (void)HAL_UART_Transmit(serial_huart, command, sizeof(command), 100U);
    }
}

void serial_Handler(UART_HandleTypeDef *huart)
{
    if (serial_huart == NULL || huart->Instance != serial_huart->Instance) {
        return;
    }

    if (serial_rx_byte == '\n') {
        vision_rx_line[vision_rx_index] = '\0';
        Serial_ProcessVisionLine(vision_rx_line);
        vision_rx_index = 0U;
    } else if (serial_rx_byte != '\r') {
        if (vision_rx_index < (sizeof(vision_rx_line) - 1U)) {
            vision_rx_line[vision_rx_index++] = (char)serial_rx_byte;
        } else {
            vision_rx_index = 0U;
        }
    }

    (void)HAL_UART_Receive_IT(serial_huart, &serial_rx_byte, 1U);
}

void serial_ErrorHandler(UART_HandleTypeDef *huart)
{
    if (serial_huart == NULL || huart->Instance != serial_huart->Instance) {
        return;
    }

    vision_rx_index = 0U;
    (void)HAL_UART_AbortReceive_IT(serial_huart);
    __HAL_UART_CLEAR_OREFLAG(serial_huart);
    (void)HAL_UART_Receive_IT(serial_huart, &serial_rx_byte, 1U);
}

int fputc(int ch, FILE *stream)
{
    (void)stream;
    Serial_SendByte((uint8_t)ch);
    return ch;
}
