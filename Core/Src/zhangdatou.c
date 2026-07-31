#include "zhangdatou.h"

#define MOTOR_CACHE_MAX_ADDR              8U
#define MOTOR_RX_BUFFER_LENGTH            12U
#define MOTOR_REQUEST_TIMEOUT_MS          20U
#define MOTOR_BLOCKING_TIMEOUT_MS         100U
#define MOTOR_COMMAND_TIMEOUT_MS          100U

typedef enum
{
    MOTOR_COM_IDLE = 0,
    MOTOR_COM_WAITING_FOR_REPLY
} MotorComState;

typedef enum
{
    MOTOR_REQUEST_NONE = 0,
    MOTOR_REQUEST_SPEED,
    MOTOR_REQUEST_POSITION
} MotorRequestType;

static UART_HandleTypeDef *motor_huart;
static volatile MotorComState motor_com_state = MOTOR_COM_IDLE;
static volatile MotorRequestType motor_request_type = MOTOR_REQUEST_NONE;
static volatile uint8_t motor_request_address;
static volatile uint8_t motor_request_command;
static volatile uint8_t motor_rx_index;
static volatile uint8_t motor_expected_length;
static volatile uint32_t motor_request_tick;
static volatile HAL_StatusTypeDef motor_last_status = HAL_OK;

static uint8_t motor_rx_buffer[MOTOR_RX_BUFFER_LENGTH];
static uint8_t motor_rx_byte;
static uint8_t motor_rx_trace_buffer[MOTOR_RX_BUFFER_LENGTH];
static uint8_t motor_last_rx_buffer[MOTOR_RX_BUFFER_LENGTH];
static volatile uint8_t motor_rx_trace_length;
static volatile uint8_t motor_last_rx_length;
static volatile uint32_t motor_current_uart_error;
static volatile uint32_t motor_last_uart_error;

static volatile int32_t motor_speed_cache[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile int32_t motor_position_cache[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile uint8_t motor_speed_valid[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile uint8_t motor_position_valid[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile uint8_t motor_position_reply_length[
    MOTOR_CACHE_MAX_ADDR + 1U];

static volatile uint32_t motor_speed_tx_count;
static volatile HAL_StatusTypeDef motor_speed_tx_status = HAL_OK;
static uint8_t motor_last_speed_command[8];

static uint8_t Motor_AddressIsValid(uint8_t address)
{
    return (address > 0U && address <= MOTOR_CACHE_MAX_ADDR) ? 1U : 0U;
}

static void Motor_FinishCommunication(HAL_StatusTypeDef status)
{
    uint8_t index;
    uint8_t length = motor_rx_trace_length;

    if (length > MOTOR_RX_BUFFER_LENGTH) {
        length = MOTOR_RX_BUFFER_LENGTH;
    }
    for (index = 0U; index < length; index++) {
        motor_last_rx_buffer[index] = motor_rx_trace_buffer[index];
    }
    motor_last_rx_length = length;
    motor_last_uart_error = motor_current_uart_error;
    motor_last_status = status;
    motor_com_state = MOTOR_COM_IDLE;
    motor_request_type = MOTOR_REQUEST_NONE;
}

static HAL_StatusTypeDef Motor_Send(const uint8_t *command, uint16_t length)
{
    if (motor_huart == NULL || command == NULL || length == 0U) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(motor_huart,
                             (uint8_t *)command,
                             length,
                             MOTOR_COMMAND_TIMEOUT_MS);
}

static void Motor_ParseReply(void)
{
    uint8_t sign;
    uint32_t raw_value;
    int32_t signed_value;

    if (motor_rx_buffer[0] != motor_request_address ||
        motor_rx_buffer[motor_expected_length - 1U] != 0x6BU) {
        Motor_FinishCommunication(HAL_ERROR);
        return;
    }

    if (motor_expected_length == 4U &&
        motor_rx_buffer[1] == 0x00U &&
        motor_rx_buffer[2] == 0xEEU) {
        Motor_FinishCommunication(HAL_ERROR);
        return;
    }

    /*
     * Command 0x36 returns:
     *   address + int32 position (big-endian, two's complement) + 0x6B
     * There is no echoed command byte or separate sign byte in this reply.
     */
    if (motor_request_type == MOTOR_REQUEST_POSITION &&
        motor_expected_length == 6U) {
        raw_value = ((uint32_t)motor_rx_buffer[1] << 24) |
                    ((uint32_t)motor_rx_buffer[2] << 16) |
                    ((uint32_t)motor_rx_buffer[3] << 8) |
                    motor_rx_buffer[4];
        motor_position_cache[motor_request_address] = (int32_t)raw_value;
        motor_position_valid[motor_request_address] = 1U;
        motor_position_reply_length[motor_request_address] = 6U;
        Motor_FinishCommunication(HAL_OK);
        return;
    }

    if (motor_rx_buffer[1] != motor_request_command ||
        (motor_rx_buffer[2] != 0x00U && motor_rx_buffer[2] != 0x01U)) {
        Motor_FinishCommunication(HAL_ERROR);
        return;
    }

    sign = motor_rx_buffer[2];

    if (motor_request_type == MOTOR_REQUEST_SPEED &&
        motor_expected_length == 6U) {
        raw_value = ((uint32_t)motor_rx_buffer[3] << 8) |
                    motor_rx_buffer[4];
        signed_value = (sign == 0x01U) ?
                       -(int32_t)raw_value :
                       (int32_t)raw_value;
        motor_speed_cache[motor_request_address] = signed_value;
        motor_speed_valid[motor_request_address] = 1U;
        Motor_FinishCommunication(HAL_OK);
        return;
    }

    /*
     * Compatibility with protocol variants that include command 0x36 and a
     * separate sign byte before the four-byte position magnitude.
     */
    if (motor_request_type == MOTOR_REQUEST_POSITION &&
        motor_expected_length == 8U) {
        raw_value = ((uint32_t)motor_rx_buffer[3] << 24) |
                    ((uint32_t)motor_rx_buffer[4] << 16) |
                    ((uint32_t)motor_rx_buffer[5] << 8) |
                    motor_rx_buffer[6];
        signed_value = (sign == 0x01U) ?
                       -(int32_t)raw_value :
                       (int32_t)raw_value;
        motor_position_cache[motor_request_address] = signed_value;
        motor_position_valid[motor_request_address] = 1U;
        motor_position_reply_length[motor_request_address] = 8U;
        Motor_FinishCommunication(HAL_OK);
        return;
    }

    Motor_FinishCommunication(HAL_ERROR);
}

static HAL_StatusTypeDef Motor_StartRequest(uint8_t address,
                                            uint8_t command,
                                            MotorRequestType request_type,
                                            uint8_t expected_length)
{
    uint8_t tx_buffer[3] = {address, command, 0x6B};
    HAL_StatusTypeDef status;

    if (motor_huart == NULL || Motor_AddressIsValid(address) == 0U) {
        return HAL_ERROR;
    }

    if (motor_com_state != MOTOR_COM_IDLE) {
        return HAL_BUSY;
    }

    motor_request_address = address;
    motor_request_command = command;
    motor_request_type = request_type;
    motor_rx_index = 0U;
    motor_rx_trace_length = 0U;
    motor_expected_length = expected_length;
    motor_request_tick = HAL_GetTick();
    motor_current_uart_error = HAL_UART_ERROR_NONE;
    motor_last_status = HAL_BUSY;
    motor_com_state = MOTOR_COM_WAITING_FOR_REPLY;

    (void)HAL_UART_AbortReceive_IT(motor_huart);
    __HAL_UART_CLEAR_OREFLAG(motor_huart);
    __HAL_UART_FLUSH_DRREGISTER(motor_huart);

    status = HAL_UART_Receive_IT(motor_huart, &motor_rx_byte, 1U);
    if (status != HAL_OK) {
        Motor_FinishCommunication(status);
        return status;
    }

    status = Motor_Send(tx_buffer, sizeof(tx_buffer));
    if (status != HAL_OK) {
        Motor_FinishCommunication(status);
    } else if (motor_com_state == MOTOR_COM_IDLE) {
        /*
         * An RX error callback may have completed the request while the
         * blocking transmit call was still in progress.
         */
        status = motor_last_status;
    }

    return status;
}

void Motor_Init(UART_HandleTypeDef *huart)
{
    uint8_t address;

    motor_huart = huart;
    motor_com_state = MOTOR_COM_IDLE;
    motor_request_type = MOTOR_REQUEST_NONE;
    motor_last_status = HAL_OK;
    motor_rx_trace_length = 0U;
    motor_last_rx_length = 0U;
    motor_current_uart_error = HAL_UART_ERROR_NONE;
    motor_last_uart_error = HAL_UART_ERROR_NONE;
    motor_speed_tx_status = HAL_OK;
    motor_speed_tx_count = 0U;

    for (address = 0U; address <= MOTOR_CACHE_MAX_ADDR; address++) {
        motor_speed_valid[address] = 0U;
        motor_position_valid[address] = 0U;
        motor_position_reply_length[address] = 0U;
    }
}

HAL_StatusTypeDef Motor_Enable(uint8_t address,
                               MotorState state,
                               SNFMODE sync_mode)
{
    uint8_t command[] = {address, 0xF3, 0xAB, state, sync_mode, 0x6B};

    return Motor_Send(command, sizeof(command));
}

HAL_StatusTypeDef Motor_SetSpeedInputScale(
    uint8_t address,
    MotorSpeedInputScale scale,
    MotorSettingStorage storage)
{
    uint8_t command[] = {
        address,
        0x4FU,
        0x71U,
        (uint8_t)storage,
        (uint8_t)scale,
        0x6BU
    };
    uint8_t response[4];
    HAL_StatusTypeDef status;

    if (motor_huart == NULL ||
        Motor_AddressIsValid(address) == 0U ||
        (scale != MOTOR_SPEED_INPUT_1_RPM &&
         scale != MOTOR_SPEED_INPUT_0_1_RPM) ||
        (storage != MOTOR_SETTING_VOLATILE &&
         storage != MOTOR_SETTING_STORE) ||
        motor_com_state != MOTOR_COM_IDLE) {
        return HAL_ERROR;
    }

    /*
     * 手册规定：
     *   地址 4F 71 保存标志 S_Vel_IS 6B
     * 成功应答：
     *   地址 4F 02 6B
     *
     * 初始化阶段直接收走这条应答，避免它残留在 USART1 中干扰
     * 后续 0x36 电机位置查询。
     */
    (void)HAL_UART_AbortReceive_IT(motor_huart);
    __HAL_UART_CLEAR_OREFLAG(motor_huart);
    __HAL_UART_FLUSH_DRREGISTER(motor_huart);

    status = Motor_Send(command, sizeof(command));
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_UART_Receive(motor_huart,
                              response,
                              sizeof(response),
                              MOTOR_BLOCKING_TIMEOUT_MS);
    if (status != HAL_OK) {
        /* 确保一次配置应答超时不会占住 USART1，后续电机命令仍可执行。 */
        (void)HAL_UART_AbortReceive(motor_huart);
        __HAL_UART_CLEAR_OREFLAG(motor_huart);
        __HAL_UART_FLUSH_DRREGISTER(motor_huart);
        return status;
    }

    if (response[0] != address ||
        response[1] != 0x4FU ||
        response[2] != 0x02U ||
        response[3] != 0x6BU) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Motor_SpeedControl(uint8_t address,
                                    MotorDirection direction,
                                    uint16_t slope,
                                    uint16_t speed,
                                    SNFMODE sync_mode)
{
    uint8_t command[8] = {
        address,
        0xF6,
        direction,
        (uint8_t)(speed >> 8),
        (uint8_t)(speed & 0xFFU),
        (uint8_t)slope,
        sync_mode,
        0x6B
    };
    uint8_t index;

    for (index = 0U; index < sizeof(command); index++) {
        motor_last_speed_command[index] = command[index];
    }

    motor_speed_tx_count++;
    motor_speed_tx_status = Motor_Send(command, sizeof(command));
    return motor_speed_tx_status;
}

HAL_StatusTypeDef Motor_PositionControl(uint8_t address,
                                       MotorDirection direction,
                                       uint16_t speed,
                                       uint8_t acceleration,
                                       uint32_t pulses,
                                       PositionMode position_mode,
                                       SNFMODE sync_mode)
{
    uint8_t command[13] = {
        address,
        0xFD,
        direction,
        (uint8_t)(speed >> 8),
        (uint8_t)(speed & 0xFFU),
        acceleration,
        (uint8_t)(pulses >> 24),
        (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8),
        (uint8_t)(pulses & 0xFFU),
        position_mode,
        sync_mode,
        0x6B
    };

    return Motor_Send(command, sizeof(command));
}

HAL_StatusTypeDef Motor_Stop(uint8_t address, SNFMODE sync_mode)
{
    uint8_t command[] = {address, 0xFE, 0x98, sync_mode, 0x6B};

    return Motor_Send(command, sizeof(command));
}

HAL_StatusTypeDef Motor_SyncStart(void)
{
    uint8_t command[] = {0x00, 0xFF, 0x66, 0x6B};

    return Motor_Send(command, sizeof(command));
}

HAL_StatusTypeDef Motor_RequestSpeedUpdate(uint8_t address)
{
    return Motor_StartRequest(address,
                              0x35U,
                              MOTOR_REQUEST_SPEED,
                              6U);
}

HAL_StatusTypeDef Motor_RequestPositionUpdate(uint8_t address)
{
    uint8_t expected_length = 6U;

    if (Motor_AddressIsValid(address) != 0U &&
        motor_position_reply_length[address] == 8U) {
        expected_length = 8U;
    }

    return Motor_StartRequest(address,
                              0x36U,
                              MOTOR_REQUEST_POSITION,
                              expected_length);
}

HAL_StatusTypeDef Motor_ReadSpeed(uint8_t address, int32_t *speed_rpm)
{
    if (speed_rpm == NULL || Motor_AddressIsValid(address) == 0U) {
        return HAL_ERROR;
    }

    if (motor_speed_valid[address] == 0U) {
        return HAL_BUSY;
    }

    *speed_rpm = motor_speed_cache[address];
    return HAL_OK;
}

HAL_StatusTypeDef Motor_ReadPosition(uint8_t address,
                                     int32_t *position,
                                     float *angle)
{
    int32_t cached_position;

    if (position == NULL || Motor_AddressIsValid(address) == 0U) {
        return HAL_ERROR;
    }

    if (motor_position_valid[address] == 0U) {
        return HAL_BUSY;
    }

    cached_position = motor_position_cache[address];
    *position = cached_position;

    if (angle != NULL) {
        if (motor_position_reply_length[address] == 8U) {
            *angle = (float)cached_position / 10.0f;
        } else {
            *angle = (float)cached_position * 360.0f / 65536.0f;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef Motor_ClearPosition(uint8_t address, uint8_t *state_code)
{
    uint8_t command[] = {address, 0x0A, 0x6D, 0x6B};
    uint8_t response[4];
    HAL_StatusTypeDef status;

    if (motor_huart == NULL || Motor_AddressIsValid(address) == 0U) {
        return HAL_ERROR;
    }

    if (motor_com_state != MOTOR_COM_IDLE) {
        return HAL_BUSY;
    }

    (void)HAL_UART_AbortReceive_IT(motor_huart);

    status = Motor_Send(command, sizeof(command));
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_UART_Receive(motor_huart,
                              response,
                              sizeof(response),
                              MOTOR_BLOCKING_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    if (response[0] != address ||
        response[1] != 0x0AU ||
        response[3] != 0x6BU ||
        response[2] == 0xEEU) {
        return HAL_ERROR;
    }

    if (state_code != NULL) {
        *state_code = response[2];
    }

    motor_position_cache[address] = 0;
    motor_position_valid[address] = 1U;
    return (response[2] == 0x02U) ? HAL_OK : HAL_ERROR;
}

uint8_t Motor_IsComBusy(void)
{
    if (motor_com_state != MOTOR_COM_IDLE &&
        (uint32_t)(HAL_GetTick() - motor_request_tick) >
        MOTOR_REQUEST_TIMEOUT_MS) {
        HAL_StatusTypeDef status = HAL_TIMEOUT;

        /*
         * A position value may legitimately begin with 00 EE 6B, so command
         * 0x36 always waits for all six bytes. If only the documented
         * four-byte error frame arrived, classify it when the request expires.
         */
        if (motor_request_type == MOTOR_REQUEST_POSITION &&
            motor_rx_index == 4U &&
            motor_rx_buffer[0] == motor_request_address &&
            motor_rx_buffer[1] == 0x00U &&
            motor_rx_buffer[2] == 0xEEU &&
            motor_rx_buffer[3] == 0x6BU) {
            status = HAL_ERROR;
        }

        (void)HAL_UART_AbortReceive_IT(motor_huart);
        Motor_FinishCommunication(status);
    }

    return (motor_com_state != MOTOR_COM_IDLE) ? 1U : 0U;
}

HAL_StatusTypeDef Motor_GetLastComStatus(void)
{
    (void)Motor_IsComBusy();
    return motor_last_status;
}

void Motor_CancelRequest(void)
{
    if (motor_huart == NULL || motor_com_state == MOTOR_COM_IDLE) {
        return;
    }

    (void)HAL_UART_AbortReceive_IT(motor_huart);
    Motor_FinishCommunication(HAL_TIMEOUT);
}

uint8_t Motor_GetLastRxFrame(uint8_t *buffer,
                             uint8_t buffer_length,
                             uint32_t *uart_error)
{
    uint8_t index;
    uint8_t copy_length = motor_last_rx_length;

    if (buffer != NULL) {
        if (copy_length > buffer_length) {
            copy_length = buffer_length;
        }
        for (index = 0U; index < copy_length; index++) {
            buffer[index] = motor_last_rx_buffer[index];
        }
    }

    if (uart_error != NULL) {
        *uart_error = motor_last_uart_error;
    }

    return motor_last_rx_length;
}

uint32_t Motor_GetSpeedTxCount(void)
{
    return motor_speed_tx_count;
}

HAL_StatusTypeDef Motor_GetLastSpeedTxStatus(void)
{
    return motor_speed_tx_status;
}

void Motor_GetLastSpeedTxCommand(uint8_t *command, uint8_t length)
{
    uint8_t index;

    if (command == NULL) {
        return;
    }

    if (length > sizeof(motor_last_speed_command)) {
        length = sizeof(motor_last_speed_command);
    }

    for (index = 0U; index < length; index++) {
        command[index] = motor_last_speed_command[index];
    }
}

void Motor_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (motor_huart == NULL ||
        huart->Instance != motor_huart->Instance ||
        motor_com_state != MOTOR_COM_WAITING_FOR_REPLY) {
        return;
    }

    if (motor_rx_index >= MOTOR_RX_BUFFER_LENGTH) {
        Motor_FinishCommunication(HAL_ERROR);
        return;
    }

    motor_rx_buffer[motor_rx_index++] = motor_rx_byte;
    if (motor_rx_trace_length < MOTOR_RX_BUFFER_LENGTH) {
        motor_rx_trace_buffer[motor_rx_trace_length++] = motor_rx_byte;
    }

    if (motor_rx_index == 1U &&
        motor_rx_buffer[0] != motor_request_address) {
        /*
         * Ignore a stale byte left by a previous transaction and continue
         * looking for the requested motor address.
         */
        motor_rx_index = 0U;
    }

    if (motor_request_type == MOTOR_REQUEST_POSITION &&
        motor_position_reply_length[motor_request_address] != 6U &&
        motor_rx_index == 3U &&
        motor_rx_buffer[0] == motor_request_address &&
        motor_rx_buffer[1] == motor_request_command &&
        motor_rx_buffer[2] == 0x6BU) {
        /* Ignore a local TX-to-RX echo of address + 0x36 + 0x6B. */
        motor_rx_index = 0U;
        motor_expected_length =
            (motor_position_reply_length[motor_request_address] == 8U) ?
            8U :
            6U;
    }

    if (motor_rx_index == 3U) {
        if (motor_rx_buffer[0] != motor_request_address) {
            Motor_FinishCommunication(HAL_ERROR);
            return;
        }

        if (motor_request_type == MOTOR_REQUEST_SPEED) {
            if (motor_rx_buffer[1] == 0x00U &&
                motor_rx_buffer[2] == 0xEEU) {
                motor_expected_length = 4U;
            } else if (motor_rx_buffer[1] != motor_request_command ||
                       (motor_rx_buffer[2] != 0x00U &&
                        motor_rx_buffer[2] != 0x01U)) {
                Motor_FinishCommunication(HAL_ERROR);
                return;
            }
        } else if (motor_request_type == MOTOR_REQUEST_POSITION &&
                   motor_position_reply_length[
                       motor_request_address] == 0U &&
                   motor_rx_buffer[1] == motor_request_command &&
                   (motor_rx_buffer[2] == 0x00U ||
                    motor_rx_buffer[2] == 0x01U)) {
            motor_expected_length = 8U;
        }
    }

    if (motor_rx_index >= motor_expected_length) {
        Motor_ParseReply();
        return;
    }

    if (HAL_UART_Receive_IT(motor_huart, &motor_rx_byte, 1U) != HAL_OK) {
        Motor_FinishCommunication(HAL_ERROR);
    }
}

void Motor_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (motor_huart != NULL &&
        huart->Instance == motor_huart->Instance) {
        motor_current_uart_error = huart->ErrorCode;
        (void)HAL_UART_AbortReceive_IT(motor_huart);
        Motor_FinishCommunication(HAL_ERROR);
    }
}
