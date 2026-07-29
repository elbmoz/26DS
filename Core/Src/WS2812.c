#include "WS2812.h"

#if WS2812_ENABLE

#define WS2812_GPIO_PORT      GPIOE
#define WS2812_GPIO_PIN       GPIO_PIN_8

#define WS2812_T0H_NS         350U
#define WS2812_T1H_NS         700U
#define WS2812_BIT_NS         1250U
#define WS2812_RESET_US       80U
#define WS2812_SLOW_HCLK_MAX  24000000U

#define WS2812_NOP4()         do { __NOP(); __NOP(); __NOP(); __NOP(); } while (0)
#define WS2812_NOP8()         do { WS2812_NOP4(); WS2812_NOP4(); } while (0)
#define WS2812_NOP16()        do { WS2812_NOP8(); WS2812_NOP8(); } while (0)

static uint32_t ws2812_t0h_cycles = 1U;
static uint32_t ws2812_t1h_cycles = 1U;
static uint32_t ws2812_bit_cycles = 1U;
static uint32_t ws2812_reset_cycles = 1U;
static uint8_t ws2812_use_slow_bitbang = 0U;

static uint32_t WS2812_NsToCycles(uint32_t ns)
{
    uint32_t cycles = (uint32_t)(((uint64_t)HAL_RCC_GetHCLKFreq() * ns + 999999999ULL)
                                 / 1000000000ULL);
    return (cycles == 0U) ? 1U : cycles;
}

static uint32_t WS2812_UsToCycles(uint32_t us)
{
    uint32_t cycles = (uint32_t)(((uint64_t)HAL_RCC_GetHCLKFreq() * us + 999999UL)
                                 / 1000000UL);
    return (cycles == 0U) ? 1U : cycles;
}

static void WS2812_DWTInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void WS2812_WaitUntil(uint32_t target)
{
    while ((int32_t)(DWT->CYCCNT - target) < 0) {
    }
}

__STATIC_INLINE void WS2812_PinHigh(void)
{
    WS2812_GPIO_PORT->BSRR = WS2812_GPIO_PIN;
}

__STATIC_INLINE void WS2812_PinLow(void)
{
    WS2812_GPIO_PORT->BSRR = ((uint32_t)WS2812_GPIO_PIN << 16U);
}

static void WS2812_SendByteDwt(uint8_t byte)
{
    uint8_t mask;
    uint32_t start;
    uint32_t high_cycles;

    for (mask = 0x80U; mask != 0U; mask >>= 1U) {
        high_cycles = (byte & mask) ? ws2812_t1h_cycles : ws2812_t0h_cycles;

        WS2812_PinHigh();
        start = DWT->CYCCNT;
        WS2812_WaitUntil(start + high_cycles);
        WS2812_PinLow();
        WS2812_WaitUntil(start + ws2812_bit_cycles);
    }
}

static void WS2812_SendByteSlow(uint8_t byte)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U) {
        if ((byte & mask) != 0U) {
            WS2812_PinHigh();
            WS2812_NOP8();
            WS2812_PinLow();
            WS2812_NOP8();
        } else {
            WS2812_PinHigh();
            __NOP();
            WS2812_PinLow();
            WS2812_NOP16();
        }
    }
}

static void WS2812_SendByte(uint8_t byte)
{
    if (ws2812_use_slow_bitbang != 0U) {
        WS2812_SendByteSlow(byte);
    } else {
        WS2812_SendByteDwt(byte);
    }
}

static uint8_t WS2812_Scale(uint8_t value, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)value * brightness) / 255U);
}

static WS2812_Color WS2812_Wheel(uint8_t position, uint8_t brightness)
{
    WS2812_Color color;

    if (position < 85U) {
        color.r = (uint8_t)(255U - position * 3U);
        color.g = (uint8_t)(position * 3U);
        color.b = 0U;
    } else if (position < 170U) {
        position = (uint8_t)(position - 85U);
        color.r = 0U;
        color.g = (uint8_t)(255U - position * 3U);
        color.b = (uint8_t)(position * 3U);
    } else {
        position = (uint8_t)(position - 170U);
        color.r = (uint8_t)(position * 3U);
        color.g = 0U;
        color.b = (uint8_t)(255U - position * 3U);
    }

    color.r = WS2812_Scale(color.r, brightness);
    color.g = WS2812_Scale(color.g, brightness);
    color.b = WS2812_Scale(color.b, brightness);

    return color;
}

static void WS2812_ResetPulse(void)
{
    uint32_t start;

    WS2812_PinLow();
    start = DWT->CYCCNT;
    WS2812_WaitUntil(start + ws2812_reset_cycles);
}

static void WS2812_FillRGB(uint8_t red, uint8_t green, uint8_t blue, uint16_t count)
{
    uint16_t i;
    uint32_t primask;

    if (count == 0U) {
        WS2812_ResetPulse();
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < count; i++) {
        WS2812_SendByte(green);
        WS2812_SendByte(red);
        WS2812_SendByte(blue);
    }

    if (primask == 0U) {
        __enable_irq();
    }

    WS2812_ResetPulse();
}

void WS2812_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    WS2812_DWTInit();

    ws2812_use_slow_bitbang = (HAL_RCC_GetHCLKFreq() <= WS2812_SLOW_HCLK_MAX) ? 1U : 0U;
    ws2812_t0h_cycles = WS2812_NsToCycles(WS2812_T0H_NS);
    ws2812_t1h_cycles = WS2812_NsToCycles(WS2812_T1H_NS);
    ws2812_bit_cycles = WS2812_NsToCycles(WS2812_BIT_NS);
    ws2812_reset_cycles = WS2812_UsToCycles(WS2812_RESET_US);

    GPIO_InitStruct.Pin = WS2812_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(WS2812_GPIO_PORT, &GPIO_InitStruct);

    WS2812_ResetPulse();
}

void WS2812_WritePixels(const WS2812_Color *pixels, uint16_t count)
{
    uint16_t i;
    uint32_t primask;

    if (pixels == NULL || count == 0U) {
        WS2812_ResetPulse();
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < count; i++) {
        WS2812_SendByte(pixels[i].g);
        WS2812_SendByte(pixels[i].r);
        WS2812_SendByte(pixels[i].b);
    }

    if (primask == 0U) {
        __enable_irq();
    }

    WS2812_ResetPulse();
}

void WS2812_SetRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    WS2812_FillRGB(red, green, blue, WS2812_LED_COUNT);
}

void WS2812_SetColor(uint32_t rgb)
{
    WS2812_SetRGB((uint8_t)((rgb >> 16U) & 0xFFU),
                  (uint8_t)((rgb >> 8U) & 0xFFU),
                  (uint8_t)(rgb & 0xFFU));
}

void WS2812_Rainbow(uint16_t count, uint8_t offset, uint8_t brightness)
{
    uint16_t i;
    uint32_t primask;
    WS2812_Color color;
    uint8_t position;

    if (count == 0U) {
        WS2812_ResetPulse();
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < count; i++) {
        position = (uint8_t)(((uint32_t)i * 256U / count) + offset);
        color = WS2812_Wheel(position, brightness);

        WS2812_SendByte(color.g);
        WS2812_SendByte(color.r);
        WS2812_SendByte(color.b);
    }

    if (primask == 0U) {
        __enable_irq();
    }

    WS2812_ResetPulse();
}

void WS2812_RainbowFlow(uint32_t period_ms, uint8_t brightness)
{
    static uint32_t last_tick = 0U;
    static uint8_t offset = 0U;
    uint32_t now = HAL_GetTick();

    if (period_ms == 0U) {
        period_ms = 20U;
    }

    if ((uint32_t)(now - last_tick) < period_ms) {
        return;
    }

    last_tick = now;
    WS2812_Rainbow(WS2812_LED_COUNT, offset, brightness);
    offset++;
}

void WS2812_BluePurpleFlow(uint32_t period_ms, uint8_t brightness)
{
    static uint32_t last_tick = 0U;
    static uint16_t head = 0U;
    static int8_t step = 1;
    uint32_t now = HAL_GetTick();
    uint32_t primask;
    uint16_t i;
    uint16_t distance;
    WS2812_Color color;

    if (period_ms == 0U) {
        period_ms = 30U;
    }

    if ((uint32_t)(now - last_tick) < period_ms) {
        return;
    }

    last_tick = now;

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < WS2812_LED_COUNT; i++) {
        if (step > 0) {
            distance = (i <= head) ? (uint16_t)(head - i) : WS2812_LED_COUNT;
        } else {
            distance = (i >= head) ? (uint16_t)(i - head) : WS2812_LED_COUNT;
        }

        switch (distance) {
        case 0U:
            color.r = 150U;
            color.g = 0U;
            color.b = 255U;
            break;

        case 1U:
            color.r = 95U;
            color.g = 0U;
            color.b = 230U;
            break;

        case 2U:
            color.r = 45U;
            color.g = 0U;
            color.b = 170U;
            break;

        case 3U:
            color.r = 12U;
            color.g = 0U;
            color.b = 95U;
            break;

        case 4U:
            color.r = 0U;
            color.g = 0U;
            color.b = 45U;
            break;

        default:
            color.r = 0U;
            color.g = 0U;
            color.b = 3U;
            break;
        }

        color.r = WS2812_Scale(color.r, brightness);
        color.g = WS2812_Scale(color.g, brightness);
        color.b = WS2812_Scale(color.b, brightness);

        WS2812_SendByte(color.g);
        WS2812_SendByte(color.r);
        WS2812_SendByte(color.b);
    }

    if (primask == 0U) {
        __enable_irq();
    }

    WS2812_ResetPulse();

    if (WS2812_LED_COUNT > 1U) {
        if (step > 0) {
            if (head >= (WS2812_LED_COUNT - 1U)) {
                step = -1;
                head--;
            } else {
                head++;
            }
        } else {
            if (head == 0U) {
                step = 1;
                head++;
            } else {
                head--;
            }
        }
    }
}

void WS2812_Clear(void)
{
    WS2812_SetRGB(0U, 0U, 0U);
}

#else

void WS2812_Init(void)
{
}

void WS2812_SetRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    (void)red;
    (void)green;
    (void)blue;
}

void WS2812_SetColor(uint32_t rgb)
{
    (void)rgb;
}

void WS2812_WritePixels(const WS2812_Color *pixels, uint16_t count)
{
    (void)pixels;
    (void)count;
}

void WS2812_Rainbow(uint16_t count, uint8_t offset, uint8_t brightness)
{
    (void)count;
    (void)offset;
    (void)brightness;
}

void WS2812_RainbowFlow(uint32_t period_ms, uint8_t brightness)
{
    (void)period_ms;
    (void)brightness;
}

void WS2812_BluePurpleFlow(uint32_t period_ms, uint8_t brightness)
{
    (void)period_ms;
    (void)brightness;
}

void WS2812_Clear(void)
{
}

#endif
