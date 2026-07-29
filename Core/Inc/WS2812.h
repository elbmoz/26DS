#ifndef __WS2812_H
#define __WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define WS2812_ENABLE    1U
#define WS2812_LED_COUNT 19U

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} WS2812_Color;

void WS2812_Init(void);
void WS2812_SetRGB(uint8_t red, uint8_t green, uint8_t blue);
void WS2812_SetColor(uint32_t rgb);
void WS2812_WritePixels(const WS2812_Color *pixels, uint16_t count);
void WS2812_Rainbow(uint16_t count, uint8_t offset, uint8_t brightness);
void WS2812_RainbowFlow(uint32_t period_ms, uint8_t brightness);
void WS2812_BluePurpleFlow(uint32_t period_ms, uint8_t brightness);
void WS2812_Clear(void);

#ifdef __cplusplus
}
#endif

#endif
