#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define START_BUTTON_GPIO_PORT      GPIOA
#define START_BUTTON_GPIO_PIN       GPIO_PIN_6
#define START_BUTTON_DEBOUNCE_MS    30U
#define START_BUTTON_POLL_MS        5U

void Button_Init(void);
uint8_t Button_IsStartPressed(void);
void Button_WaitForStart(void);

#ifdef __cplusplus
}
#endif

#endif
