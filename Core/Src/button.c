#include "button.h"

void Button_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = START_BUTTON_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(START_BUTTON_GPIO_PORT, &GPIO_InitStruct);
}

uint8_t Button_IsStartPressed(void)
{
    if (HAL_GPIO_ReadPin(START_BUTTON_GPIO_PORT, START_BUTTON_GPIO_PIN) != GPIO_PIN_SET) {
        return 0U;
    }

    HAL_Delay(START_BUTTON_DEBOUNCE_MS);

    return (HAL_GPIO_ReadPin(START_BUTTON_GPIO_PORT, START_BUTTON_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

void Button_WaitForStart(void)
{
    while (Button_IsStartPressed() == 0U) {
        HAL_Delay(START_BUTTON_POLL_MS);
    }
}
