#include "screen.h"
#include "usart.h"

#include <stdio.h>

#define SCREEN_TX_TIMEOUT_MS 100U
#define SCREEN_CMD_BUF_SIZE 64U
#define SCREEN_GYRO_COMPONENT "t1"

static UART_HandleTypeDef *screen_huart = &huart4;

void Screen_Init(UART_HandleTypeDef *huart)
{
    screen_huart = huart;
}

HAL_StatusTypeDef Screen_SendText(const char *component, const char *text)
{
    uint8_t command[SCREEN_CMD_BUF_SIZE];
    uint16_t len;
    int written;

    if (screen_huart == NULL || component == NULL || text == NULL) {
        return HAL_ERROR;
    }

    written = snprintf((char *)command,
                       sizeof(command),
                       "%s.txt=\"%s\"",
                       component,
                       text);
    if (written <= 0 || written > (int)(sizeof(command) - 3U)) {
        return HAL_ERROR;
    }

    len = (uint16_t)written;
    command[len++] = 0xFFU;
    command[len++] = 0xFFU;
    command[len++] = 0xFFU;

    return HAL_UART_Transmit(screen_huart,
                             command,
                             len,
                             SCREEN_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef Screen_SendQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3,
                                         uint8_t task4, uint8_t task5, uint8_t task6)
{
    char text[] = "000+000";

    if (task1 > 9U || task2 > 9U || task3 > 9U ||
        task4 > 9U || task5 > 9U || task6 > 9U) {
        return HAL_ERROR;
    }

    text[0] = (char)('0' + task1);
    text[1] = (char)('0' + task2);
    text[2] = (char)('0' + task3);
    text[4] = (char)('0' + task4);
    text[5] = (char)('0' + task5);
    text[6] = (char)('0' + task6);

    return Screen_SendText("t0", text);
}

HAL_StatusTypeDef Screen_SendGyroAngle(float angle)
{
    char text[24];
    int written;

    written = snprintf(text, sizeof(text), "Yaw:%.1f", angle);
    if (written <= 0 || written >= (int)sizeof(text)) {
        return HAL_ERROR;
    }

    return Screen_SendText(SCREEN_GYRO_COMPONENT, text);
}

HAL_StatusTypeDef Screen_TestSend(void)
{
    return Screen_SendText("t0", "123+123");
}
