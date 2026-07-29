#include "button.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t candidate_pressed;
    uint8_t stable_pressed;
    uint8_t click_pending;
    uint32_t candidate_since_ms;
} ButtonChannel;

static ButtonChannel button_channels[BUTTON_ID_COUNT] = {
    {BUTTON_1_GPIO_PORT, BUTTON_1_GPIO_PIN, 0U, 0U, 0U, 0U},
    {BUTTON_2_GPIO_PORT, BUTTON_2_GPIO_PIN, 0U, 0U, 0U, 0U}
};

static uint8_t Button_ReadPressed(const ButtonChannel *button)
{
    return (HAL_GPIO_ReadPin(button->port, button->pin) == GPIO_PIN_RESET) ?
           1U :
           0U;
}

void Button_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t index;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = BUTTON_1_GPIO_PIN | BUTTON_2_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    for (index = 0U; index < BUTTON_ID_COUNT; index++) {
        uint8_t pressed = Button_ReadPressed(&button_channels[index]);
        button_channels[index].candidate_pressed = pressed;
        button_channels[index].stable_pressed = pressed;
        button_channels[index].click_pending = 0U;
        button_channels[index].candidate_since_ms = HAL_GetTick();
    }
}

void Button_Scan(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t index;

    for (index = 0U; index < BUTTON_ID_COUNT; index++) {
        ButtonChannel *button = &button_channels[index];
        uint8_t pressed = Button_ReadPressed(button);

        if (pressed != button->candidate_pressed) {
            button->candidate_pressed = pressed;
            button->candidate_since_ms = now;
        } else if (button->stable_pressed != button->candidate_pressed &&
                   (uint32_t)(now - button->candidate_since_ms) >=
                   BUTTON_DEBOUNCE_MS) {
            button->stable_pressed = button->candidate_pressed;
            if (button->stable_pressed != 0U) {
                button->click_pending = 1U;
            }
        }
    }
}

uint8_t Button_IsPressed(ButtonId id)
{
    if ((uint8_t)id >= BUTTON_ID_COUNT) {
        return 0U;
    }

    return button_channels[id].stable_pressed;
}

uint8_t Button_GetClick(ButtonId id)
{
    uint8_t clicked;

    if ((uint8_t)id >= BUTTON_ID_COUNT) {
        return 0U;
    }

    clicked = button_channels[id].click_pending;
    button_channels[id].click_pending = 0U;
    return clicked;
}

uint8_t Button1_GetClick(void)
{
    return Button_GetClick(BUTTON_ID_1);
}

uint8_t Button2_GetClick(void)
{
    return Button_GetClick(BUTTON_ID_2);
}
