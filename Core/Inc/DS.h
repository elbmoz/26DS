#ifndef DS_H
#define DS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * Electronic-design car hardware map.
 *
 * Keep board-specific addresses, directions and GPIO assignments in this file.
 * The current values are provisional until the final schematic is available.
 */
#define DS_LEFT_MOTOR_ADDR             0x01U
#define DS_RIGHT_MOTOR_ADDR            0x02U
#define DS_BALANCE_MOTOR_ADDR          0x03U

#define DS_LEFT_MOTOR_INVERTED         0U
#define DS_RIGHT_MOTOR_INVERTED        1U
#define DS_BALANCE_MOTOR_INVERTED      0U

#define DS_MOTOR_MAX_SPEED             20000U
#define DS_MOTOR_DEFAULT_SLOPE         10U

#define DS_IR_SENSOR_COUNT             8U
#define DS_IR_1_PORT                   GPIOE
#define DS_IR_1_PIN                    GPIO_PIN_11
#define DS_IR_2_PORT                   GPIOE
#define DS_IR_2_PIN                    GPIO_PIN_10
#define DS_IR_3_PORT                   GPIOE
#define DS_IR_3_PIN                    GPIO_PIN_9
#define DS_IR_4_PORT                   GPIOE
#define DS_IR_4_PIN                    GPIO_PIN_8
#define DS_IR_5_PORT                   GPIOE
#define DS_IR_5_PIN                    GPIO_PIN_7
#define DS_IR_6_PORT                   GPIOA
#define DS_IR_6_PIN                    GPIO_PIN_6
#define DS_IR_7_PORT                   GPIOA
#define DS_IR_7_PIN                    GPIO_PIN_11
#define DS_IR_8_PORT                   GPIOA
#define DS_IR_8_PIN                    GPIO_PIN_7

#define DS_IR_GPIOE_PINS               (DS_IR_1_PIN | DS_IR_2_PIN | \
                                        DS_IR_3_PIN | DS_IR_4_PIN | \
                                        DS_IR_5_PIN)
#define DS_IR_GPIOA_PINS               (DS_IR_6_PIN | DS_IR_7_PIN | \
                                        DS_IR_8_PIN)

/* Most digital line sensors assert low. Set to 0U if the selected board asserts high. */
#define DS_IR_ACTIVE_LOW               1U
#define DS_VISION_TIMEOUT_MS           200U

typedef struct
{
    volatile uint32_t uptime_ms;

    /* Bit 0 is IR1 and bit 7 is IR8. */
    volatile uint8_t ir_raw_bits;
    volatile uint8_t ir_active_bits;

    volatile int32_t vision_x_error;
    volatile int32_t vision_y_error;
    volatile uint8_t vision_valid;
    volatile uint32_t vision_frame_count;
    volatile uint32_t vision_last_rx_ms;

    volatile float yaw_deg;
    volatile float gyro_y_dps;
    volatile float gyro_z_dps;
} DS_State;

HAL_StatusTypeDef DS_Init(void);
void DS_Run(void);
void DS_1msTickFromISR(void);
const DS_State *DS_GetState(void);

uint8_t DS_IR_ReadRaw(void);
uint8_t DS_IR_ReadActive(void);

void DS_VisionUpdateFromISR(int32_t x_error, int32_t y_error, uint8_t valid);
uint8_t DS_VisionIsFresh(void);

void DS_MotorsEnable(void);
void DS_MotorsDisable(void);
HAL_StatusTypeDef DS_ChassisSetSpeed(int32_t left_speed,
                                     int32_t right_speed,
                                     uint8_t slope);
void DS_ChassisStop(void);

HAL_StatusTypeDef DS_BalanceSetSpeed(int32_t speed, uint8_t slope);
HAL_StatusTypeDef DS_BalanceMoveRelative(int32_t pulses,
                                         uint16_t speed,
                                         uint8_t acceleration);
void DS_BalanceStop(void);
HAL_StatusTypeDef DS_BalanceRequestPosition(void);
HAL_StatusTypeDef DS_BalanceReadPosition(int32_t *position, float *angle);

#ifdef __cplusplus
}
#endif

#endif
