#include "DS.h"

#include "BallVision.h"
#include "HWT101.h"
#include "serial.h"
#include "usart.h"
#include "zhangdatou.h"

static DS_State ds_state;
static volatile uint32_t ds_1ms_pending;

#define DS_CHASSIS_STOP_RETRY_COUNT     2U
#define DS_MOTOR_STOP_GAP_MS            2U

static GPIO_TypeDef * const ds_ir_ports[DS_IR_SENSOR_COUNT] = {
    DS_IR_1_PORT,
    DS_IR_2_PORT,
    DS_IR_3_PORT,
    DS_IR_4_PORT,
    DS_IR_5_PORT,
    DS_IR_6_PORT,
    DS_IR_7_PORT,
    DS_IR_8_PORT
};

static const uint16_t ds_ir_pins[DS_IR_SENSOR_COUNT] = {
    DS_IR_1_PIN,
    DS_IR_2_PIN,
    DS_IR_3_PIN,
    DS_IR_4_PIN,
    DS_IR_5_PIN,
    DS_IR_6_PIN,
    DS_IR_7_PIN,
    DS_IR_8_PIN
};

static uint16_t DS_ClampMotorSpeed(int32_t speed)
{
    uint32_t magnitude;

    if (speed < 0) {
        magnitude = (uint32_t)(-(speed + 1)) + 1U;
    } else {
        magnitude = (uint32_t)speed;
    }

    if (magnitude > DS_MOTOR_MAX_SPEED) {
        magnitude = DS_MOTOR_MAX_SPEED;
    }

    return (uint16_t)magnitude;
}

static MotorDirection DS_GetMotorDirection(int32_t speed, uint8_t inverted)
{
    uint8_t negative = (speed < 0) ? 1U : 0U;

    if (inverted != 0U) {
        negative ^= 1U;
    }

    return (negative != 0U) ? DIRECTION_NEGATIVE : DIRECTION_POSITIVE;
}

static void DS_IR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = DS_IR_GPIOE_PINS;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DS_IR_GPIOA_PINS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

HAL_StatusTypeDef DS_Init(void)
{
    DS_IR_Init();

    ds_state.uptime_ms = 0U;
    ds_state.ir_raw_bits = DS_IR_ReadRaw();
    ds_state.ir_active_bits = DS_IR_ReadActive();
    ds_state.vision_x_error = 0;
    ds_state.vision_y_error = 0;
    ds_state.vision_valid = 0U;
    ds_state.vision_frame_count = 0U;
    ds_state.vision_last_rx_ms = 0U;
    ds_state.ball_position = 0.0f;
    ds_state.ball_vision_valid = 0U;
    ds_state.ball_vision_frame_count = 0U;
    ds_state.ball_vision_last_rx_ms = 0U;
    ds_state.yaw_deg = 0.0f;
    ds_state.gyro_y_dps = 0.0f;
    ds_state.gyro_z_dps = 0.0f;
    ds_state.yaw_valid = 0U;
    ds_state.yaw_frame_count = 0U;
    ds_state.yaw_last_rx_ms = 0U;
    ds_1ms_pending = 0U;

    Motor_Init(&huart1);
    DS_MotorsEnable();
    DS_ChassisStop();
    DS_BalanceStop();

    Serial_Init(&huart5);
    BallVision_Init(&huart6);
    HWT101_Init(&huart2);

    return HAL_OK;
}

void DS_Run(void)
{
    uint32_t pending;
    uint32_t primask;
    uint32_t now;

    primask = __get_PRIMASK();
    __disable_irq();
    pending = ds_1ms_pending;
    ds_1ms_pending = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    ds_state.uptime_ms += pending;
    ds_state.ir_raw_bits = DS_IR_ReadRaw();
    ds_state.ir_active_bits = DS_IR_ReadActive();
    ds_state.yaw_deg = (float)global_angle;
    ds_state.gyro_y_dps = angular_velocity_y;
    ds_state.gyro_z_dps = angular_velocity_z;
    ds_state.yaw_frame_count = hwt_yaw_frame_count;
    ds_state.yaw_last_rx_ms = hwt_yaw_last_rx_ms;
    now = HAL_GetTick();
    ds_state.yaw_valid =
        (ds_state.yaw_frame_count != 0U &&
         (uint32_t)(now - ds_state.yaw_last_rx_ms) <=
         DS_IMU_TIMEOUT_MS) ? 1U : 0U;

    if (ds_state.vision_valid != 0U &&
        (uint32_t)(now - ds_state.vision_last_rx_ms) >
        DS_VISION_TIMEOUT_MS) {
        ds_state.vision_valid = 0U;
    }

    if (ds_state.ball_vision_valid != 0U &&
        (uint32_t)(now - ds_state.ball_vision_last_rx_ms) >
        DS_BALL_VISION_TIMEOUT_MS) {
        ds_state.ball_vision_valid = 0U;
    }

    /* Also services timeout state for asynchronous motor readback. */
    (void)Motor_IsComBusy();
}

void DS_1msTickFromISR(void)
{
    if (ds_1ms_pending < UINT32_MAX) {
        ds_1ms_pending++;
    }
}

const DS_State *DS_GetState(void)
{
    return &ds_state;
}

uint8_t DS_IR_ReadRaw(void)
{
    uint8_t raw = 0U;
    uint8_t index;

    for (index = 0U; index < DS_IR_SENSOR_COUNT; index++) {
        if (HAL_GPIO_ReadPin(ds_ir_ports[index], ds_ir_pins[index]) == GPIO_PIN_SET) {
            raw |= (uint8_t)(1U << index);
        }
    }

    return raw;
}

uint8_t DS_IR_ReadActive(void)
{
    uint8_t raw = DS_IR_ReadRaw();

#if DS_IR_ACTIVE_LOW
    return (uint8_t)(~raw);
#else
    return raw;
#endif
}

void DS_VisionUpdateFromISR(int32_t x_error, int32_t y_error, uint8_t valid)
{
    ds_state.vision_x_error = x_error;
    ds_state.vision_y_error = y_error;
    ds_state.vision_valid = (valid != 0U) ? 1U : 0U;
    ds_state.vision_last_rx_ms = HAL_GetTick();
    ds_state.vision_frame_count++;
}

uint8_t DS_VisionIsFresh(void)
{
    if (ds_state.vision_valid == 0U) {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - ds_state.vision_last_rx_ms) <=
            DS_VISION_TIMEOUT_MS) ? 1U : 0U;
}

void DS_BallVisionUpdateFromISR(float position, uint8_t valid)
{
    ds_state.ball_position = position;
    ds_state.ball_vision_valid = (valid != 0U) ? 1U : 0U;
    ds_state.ball_vision_last_rx_ms = HAL_GetTick();
    ds_state.ball_vision_frame_count++;
}

uint8_t DS_BallVisionIsFresh(void)
{
    if (ds_state.ball_vision_valid == 0U) {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() -
                       ds_state.ball_vision_last_rx_ms) <=
            DS_BALL_VISION_TIMEOUT_MS) ? 1U : 0U;
}

void DS_MotorsEnable(void)
{
    (void)Motor_Enable(DS_LEFT_MOTOR_ADDR, MOTOR_ENABLE, SNF_ENABLE);
    HAL_Delay(1U);
    (void)Motor_Enable(DS_RIGHT_MOTOR_ADDR, MOTOR_ENABLE, SNF_ENABLE);
    HAL_Delay(1U);
    (void)Motor_Enable(DS_BALANCE_MOTOR_ADDR, MOTOR_ENABLE, SNF_ENABLE);
    HAL_Delay(1U);
    (void)Motor_SyncStart();
}

void DS_MotorsDisable(void)
{
    DS_ChassisStop();
    DS_BalanceStop();

    (void)Motor_Enable(DS_LEFT_MOTOR_ADDR, MOTOR_DISABLE, SNF_ENABLE);
    HAL_Delay(1U);
    (void)Motor_Enable(DS_RIGHT_MOTOR_ADDR, MOTOR_DISABLE, SNF_ENABLE);
    HAL_Delay(1U);
    (void)Motor_Enable(DS_BALANCE_MOTOR_ADDR, MOTOR_DISABLE, SNF_ENABLE);
    HAL_Delay(1U);
    (void)Motor_SyncStart();
}

HAL_StatusTypeDef DS_ChassisSetSpeed(int32_t left_speed,
                                     int32_t right_speed,
                                     uint8_t slope)
{
    HAL_StatusTypeDef status;

    if (Motor_IsComBusy() != 0U) {
        return HAL_BUSY;
    }

    status = Motor_SpeedControl(
        DS_LEFT_MOTOR_ADDR,
        DS_GetMotorDirection(left_speed, DS_LEFT_MOTOR_INVERTED),
        slope,
        DS_ClampMotorSpeed(left_speed),
        SNF_ENABLE);
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(1U);
    status = Motor_SpeedControl(
        DS_RIGHT_MOTOR_ADDR,
        DS_GetMotorDirection(right_speed, DS_RIGHT_MOTOR_INVERTED),
        slope,
        DS_ClampMotorSpeed(right_speed),
        SNF_ENABLE);
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(1U);
    return Motor_SyncStart();
}

void DS_ChassisStop(void)
{
    uint8_t attempt;

    /*
     * A lap can complete immediately after Motor_SyncStart(), while the
     * drivers may still be replying on the shared bus. Leave a turnaround
     * interval before the first stop command, then use immediate stops
     * instead of queued synchronized stops. Send both commands twice so a
     * single lost frame cannot leave one wheel running indefinitely.
     */
    HAL_Delay(DS_MOTOR_STOP_GAP_MS);
    for (attempt = 0U;
         attempt < DS_CHASSIS_STOP_RETRY_COUNT;
         attempt++) {
        (void)Motor_Stop(DS_LEFT_MOTOR_ADDR, SNF_DISABLE);
        HAL_Delay(DS_MOTOR_STOP_GAP_MS);
        (void)Motor_Stop(DS_RIGHT_MOTOR_ADDR, SNF_DISABLE);
        HAL_Delay(DS_MOTOR_STOP_GAP_MS);
    }
}

HAL_StatusTypeDef DS_BalanceSetSpeed(int32_t speed, uint8_t slope)
{
    if (Motor_IsComBusy() != 0U) {
        return HAL_BUSY;
    }

    return Motor_SpeedControl(
        DS_BALANCE_MOTOR_ADDR,
        DS_GetMotorDirection(speed, DS_BALANCE_MOTOR_INVERTED),
        slope,
        DS_ClampMotorSpeed(speed),
        SNF_DISABLE);
}

HAL_StatusTypeDef DS_BalanceMoveRelative(int32_t pulses,
                                         uint16_t speed,
                                         uint8_t acceleration)
{
    MotorDirection direction;
    uint32_t magnitude;

    if (Motor_IsComBusy() != 0U) {
        return HAL_BUSY;
    }

    if (pulses == 0) {
        DS_BalanceStop();
        return HAL_OK;
    }

    direction = DS_GetMotorDirection(pulses, DS_BALANCE_MOTOR_INVERTED);
    magnitude = (pulses < 0) ?
                ((uint32_t)(-(pulses + 1)) + 1U) :
                (uint32_t)pulses;

    if (speed > DS_MOTOR_MAX_SPEED) {
        speed = DS_MOTOR_MAX_SPEED;
    }

    return Motor_PositionControl(DS_BALANCE_MOTOR_ADDR,
                                 direction,
                                 speed,
                                 acceleration,
                                 magnitude,
                                 RELATIVE_POSITION,
                                 SNF_DISABLE);
}

HAL_StatusTypeDef DS_BalanceStop(void)
{
    return Motor_Stop(DS_BALANCE_MOTOR_ADDR, SNF_DISABLE);
}

HAL_StatusTypeDef DS_BalanceRequestPosition(void)
{
    return Motor_RequestPositionUpdate(DS_BALANCE_MOTOR_ADDR);
}

HAL_StatusTypeDef DS_BalanceReadPosition(int32_t *position, float *angle)
{
    return Motor_ReadPosition(DS_BALANCE_MOTOR_ADDR, position, angle);
}
