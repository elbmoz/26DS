#include "MotorPositionMonitor.h"

#include "DS.h"

MotorPositionMonitorState motor_position_monitor_state;

static uint8_t motor_position_request_pending;
static uint32_t motor_position_last_request_ms;

static void MotorPositionMonitor_RecordFailure(void)
{
    if (motor_position_monitor_state.consecutive_failures < UINT8_MAX) {
        motor_position_monitor_state.consecutive_failures++;
    }

    /*
     * Keep the last good position visible through one-off UART noise. Switch
     * to the raw diagnostic page only when startup has never succeeded or
     * three requests fail consecutively.
     */
    if (motor_position_monitor_state.update_count == 0U ||
        motor_position_monitor_state.consecutive_failures >= 3U) {
        motor_position_monitor_state.valid = 0U;
    }
}

static void MotorPositionMonitor_ClearDiagnostics(void)
{
    uint8_t index;

    motor_position_monitor_state.uart_error = HAL_UART_ERROR_NONE;
    motor_position_monitor_state.rx_length = 0U;
    for (index = 0U;
         index < MOTOR_POSITION_MONITOR_RX_BYTES;
         index++) {
        motor_position_monitor_state.rx_bytes[index] = 0U;
    }
}

static void MotorPositionMonitor_CaptureDiagnostics(void)
{
    uint8_t buffer[MOTOR_POSITION_MONITOR_RX_BYTES];
    uint8_t index;
    uint8_t length;
    uint32_t uart_error;

    length = DS_BalanceGetLastPositionRx(
        buffer,
        sizeof(buffer),
        &uart_error);
    motor_position_monitor_state.rx_length = length;
    motor_position_monitor_state.uart_error = uart_error;

    for (index = 0U;
         index < MOTOR_POSITION_MONITOR_RX_BYTES;
         index++) {
        motor_position_monitor_state.rx_bytes[index] =
            (index < length) ? buffer[index] : 0U;
    }
}

void MotorPositionMonitor_Init(void)
{
    motor_position_monitor_state.position = 0;
    motor_position_monitor_state.angle_deg = 0.0f;
    motor_position_monitor_state.request_count = 0U;
    motor_position_monitor_state.update_count = 0U;
    motor_position_monitor_state.consecutive_failures = 0U;
    MotorPositionMonitor_ClearDiagnostics();
    motor_position_monitor_state.last_status = HAL_OK;
    motor_position_monitor_state.valid = 0U;
    motor_position_monitor_state.active = 0U;
    motor_position_request_pending = 0U;
    motor_position_last_request_ms = 0U;
}

void MotorPositionMonitor_Start(void)
{
    uint32_t now = HAL_GetTick();

    motor_position_monitor_state.position = 0;
    motor_position_monitor_state.angle_deg = 0.0f;
    motor_position_monitor_state.request_count = 0U;
    motor_position_monitor_state.update_count = 0U;
    motor_position_monitor_state.consecutive_failures = 0U;
    MotorPositionMonitor_ClearDiagnostics();
    motor_position_monitor_state.last_status = HAL_BUSY;
    motor_position_monitor_state.valid = 0U;
    motor_position_monitor_state.active = 1U;
    motor_position_request_pending = 0U;
    motor_position_last_request_ms =
        now - MOTOR_POSITION_MONITOR_PERIOD_MS;
}

void MotorPositionMonitor_Update(void)
{
    HAL_StatusTypeDef status;
    int32_t position;
    float angle;
    uint32_t now;

    if (motor_position_monitor_state.active == 0U) {
        return;
    }

    now = HAL_GetTick();

    if (motor_position_request_pending != 0U) {
        status = DS_BalanceGetPositionRequestStatus();
        if (status == HAL_BUSY) {
            return;
        }

        motor_position_request_pending = 0U;
        motor_position_monitor_state.last_status = status;
        MotorPositionMonitor_CaptureDiagnostics();

        if (status == HAL_OK) {
            status = DS_BalanceReadPosition(&position, &angle);
            motor_position_monitor_state.last_status = status;
            if (status == HAL_OK) {
                motor_position_monitor_state.position = position;
                motor_position_monitor_state.angle_deg = angle;
                motor_position_monitor_state.update_count++;
                motor_position_monitor_state.consecutive_failures = 0U;
                motor_position_monitor_state.valid = 1U;
            } else {
                MotorPositionMonitor_RecordFailure();
            }
        } else {
            MotorPositionMonitor_RecordFailure();
        }
    }

    if ((uint32_t)(now - motor_position_last_request_ms) <
        MOTOR_POSITION_MONITOR_PERIOD_MS) {
        return;
    }

    status = DS_BalanceRequestPosition();
    motor_position_last_request_ms = now;
    motor_position_monitor_state.request_count++;
    if (status == HAL_OK) {
        motor_position_request_pending = 1U;
    } else {
        motor_position_monitor_state.last_status = status;
        if (status != HAL_BUSY) {
            MotorPositionMonitor_CaptureDiagnostics();
        }
        if (status != HAL_BUSY) {
            MotorPositionMonitor_RecordFailure();
        }
    }
}

void MotorPositionMonitor_Stop(void)
{
    if (motor_position_request_pending != 0U) {
        DS_BalanceCancelPositionRequest();
    }
    motor_position_monitor_state.active = 0U;
    motor_position_request_pending = 0U;
}
