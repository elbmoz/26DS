#include "HWT101.h"

static UART_HandleTypeDef *hwt_huart;
static uint8_t hwt_rx_byte;
static uint8_t hwt_rx_buffer[11];
static uint8_t hwt_rx_index;

static const uint8_t hwt_zero_yaw_command[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};

volatile double global_angle;
volatile uint8_t new_data_received;
volatile float angular_velocity_y;
volatile float angular_velocity_z;
volatile uint32_t hwt_yaw_frame_count;
volatile uint32_t hwt_yaw_last_rx_ms;

static uint8_t HWT101_ChecksumIsValid(const uint8_t *data)
{
    uint8_t sum = 0U;
    uint8_t index;

    for (index = 0U; index < 10U; index++) {
        sum = (uint8_t)(sum + data[index]);
    }

    return (sum == data[10]) ? 1U : 0U;
}

static void HWT101_ParseFrame(const uint8_t *data)
{
    int16_t raw_value;

    if (data[0] != 0x55U || HWT101_ChecksumIsValid(data) == 0U) {
        return;
    }

    if (data[1] == 0x53U) {
        raw_value = (int16_t)(((uint16_t)data[7] << 8) | data[6]);
        global_angle = (double)raw_value * 180.0 / 32768.0;
        hwt_yaw_frame_count++;
        hwt_yaw_last_rx_ms = HAL_GetTick();
        new_data_received = 1U;
    } else if (data[1] == 0x52U) {
        raw_value = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
        angular_velocity_y = (float)raw_value * 2000.0f / 32768.0f;

        raw_value = (int16_t)(((uint16_t)data[7] << 8) | data[6]);
        angular_velocity_z = (float)raw_value * 2000.0f / 32768.0f;
        new_data_received = 1U;
    }
}

void HWT101_Init(UART_HandleTypeDef *huart)
{
    hwt_huart = huart;
    hwt_rx_index = 0U;
    global_angle = 0.0;
    angular_velocity_y = 0.0f;
    angular_velocity_z = 0.0f;
    hwt_yaw_frame_count = 0U;
    hwt_yaw_last_rx_ms = 0U;
    new_data_received = 0U;

    if (hwt_huart == NULL) {
        return;
    }

    (void)HAL_UART_Transmit(hwt_huart,
                            (uint8_t *)hwt_zero_yaw_command,
                            sizeof(hwt_zero_yaw_command),
                            100U);
    HAL_Delay(500U);
    (void)HAL_UART_Receive_IT(hwt_huart, &hwt_rx_byte, 1U);
}

void HWT101_Clear(void)
{
    if (hwt_huart == NULL) {
        return;
    }

    (void)HAL_UART_Transmit(hwt_huart,
                            (uint8_t *)hwt_zero_yaw_command,
                            sizeof(hwt_zero_yaw_command),
                            100U);
}

void hwt_Handler(UART_HandleTypeDef *huart)
{
    if (hwt_huart == NULL || huart->Instance != hwt_huart->Instance) {
        return;
    }

    if (hwt_rx_index == 0U) {
        if (hwt_rx_byte == 0x55U) {
            hwt_rx_buffer[hwt_rx_index++] = hwt_rx_byte;
        }
    } else {
        hwt_rx_buffer[hwt_rx_index++] = hwt_rx_byte;

        if (hwt_rx_index == 2U &&
            hwt_rx_buffer[1] != 0x52U &&
            hwt_rx_buffer[1] != 0x53U) {
            if (hwt_rx_buffer[1] == 0x55U) {
                hwt_rx_buffer[0] = 0x55U;
                hwt_rx_index = 1U;
            } else {
                hwt_rx_index = 0U;
            }
        } else if (hwt_rx_index >= sizeof(hwt_rx_buffer)) {
            HWT101_ParseFrame(hwt_rx_buffer);
            hwt_rx_index = 0U;
        }
    }

    (void)HAL_UART_Receive_IT(hwt_huart, &hwt_rx_byte, 1U);
}

void HWT101_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (hwt_huart == NULL || huart->Instance != hwt_huart->Instance) {
        return;
    }

    hwt_rx_index = 0U;
    (void)HAL_UART_AbortReceive_IT(hwt_huart);
    __HAL_UART_CLEAR_OREFLAG(hwt_huart);
    (void)HAL_UART_Receive_IT(hwt_huart, &hwt_rx_byte, 1U);
}
