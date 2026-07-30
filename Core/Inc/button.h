#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define BUTTON_1_GPIO_PORT         GPIOB
#define BUTTON_1_GPIO_PIN          GPIO_PIN_6
#define BUTTON_2_GPIO_PORT         GPIOB
#define BUTTON_2_GPIO_PIN          GPIO_PIN_7
#define BUTTON_DEBOUNCE_MS         20U

typedef enum
{
    BUTTON_ID_1 = 0,
    BUTTON_ID_2,
    BUTTON_ID_COUNT
} ButtonId;

void Button_Init(void);
void Button_Scan(void);
uint8_t Button_IsPressed(ButtonId id);
uint8_t Button_GetClick(ButtonId id);
uint8_t Button1_GetClick(void);
uint8_t Button2_GetClick(void);

#ifdef __cplusplus
}
#endif

#endif
