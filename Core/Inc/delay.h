#ifndef DELAY_H
#define DELAY_H

#include "stdint.h"
#include "stm32f4xx_hal.h"
uint8_t DWT_Delay_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
#endif 