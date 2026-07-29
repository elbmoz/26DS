#ifndef OLED_H
#define OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define OLED_SCL_GPIO_PORT      GPIOB
#define OLED_SCL_GPIO_PIN       GPIO_PIN_8
#define OLED_SDA_GPIO_PORT      GPIOB
#define OLED_SDA_GPIO_PIN       GPIO_PIN_9

#define OLED_ADDRESS_0          0x3CU
#define OLED_ADDRESS_1          0x3DU

HAL_StatusTypeDef OLED_Init(void);
uint8_t OLED_IsConnected(void);
uint8_t OLED_GetAddress(void);
HAL_StatusTypeDef OLED_TestAllPixels(uint32_t hold_ms);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t line, uint8_t column, char character);
void OLED_ShowString(uint8_t line, uint8_t column, const char *text);
void OLED_ShowNum(uint8_t line,
                  uint8_t column,
                  uint32_t number,
                  uint8_t length);
void OLED_ShowSignedNum(uint8_t line,
                        uint8_t column,
                        int32_t number,
                        uint8_t length);
void OLED_ShowHexNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length);
void OLED_ShowBinNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length);

#ifdef __cplusplus
}
#endif

#endif
