#include "OLED.h"

#include "OLED_Font.h"
#include "stm32f4xx_hal.h"

#define OLED_WIDTH_PIXELS            128U
#define OLED_PAGE_COUNT              8U
#define OLED_CHARACTER_COLUMNS       16U
#define OLED_I2C_HALF_PERIOD_US      2U

static uint8_t oled_write_address = (uint8_t)(OLED_ADDRESS_0 << 1U);
static volatile uint8_t oled_connected;
static uint8_t oled_dwt_ready;

static void OLED_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    oled_dwt_ready =
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

static void OLED_I2C_Delay(void)
{
    uint32_t cycles;

    cycles = (SystemCoreClock / 1000000U) * OLED_I2C_HALF_PERIOD_US;
    if (cycles == 0U) {
        cycles = 1U;
    }

    if (oled_dwt_ready != 0U) {
        uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
        }
    } else {
        volatile uint32_t index;
        for (index = 0U; index < cycles; index++) {
            __NOP();
        }
    }
}

static void OLED_WriteSCL(uint8_t level)
{
    HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT,
                      OLED_SCL_GPIO_PIN,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void OLED_WriteSDA(uint8_t level)
{
    HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT,
                      OLED_SDA_GPIO_PIN,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    OLED_DelayInit();

    GPIO_InitStruct.Pin = OLED_SCL_GPIO_PIN | OLED_SDA_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    OLED_WriteSCL(1U);
    OLED_WriteSDA(1U);
    OLED_I2C_Delay();
}

static void OLED_I2C_BusRecover(void)
{
    uint8_t pulse;

    OLED_WriteSDA(1U);
    OLED_WriteSCL(1U);
    OLED_I2C_Delay();

    if (HAL_GPIO_ReadPin(OLED_SDA_GPIO_PORT,
                         OLED_SDA_GPIO_PIN) == GPIO_PIN_RESET) {
        for (pulse = 0U; pulse < 9U; pulse++) {
            OLED_WriteSCL(0U);
            OLED_I2C_Delay();
            OLED_WriteSCL(1U);
            OLED_I2C_Delay();
        }
    }

    OLED_WriteSDA(0U);
    OLED_I2C_Delay();
    OLED_WriteSCL(1U);
    OLED_I2C_Delay();
    OLED_WriteSDA(1U);
    OLED_I2C_Delay();
}

static void OLED_I2C_Start(void)
{
    OLED_WriteSDA(1U);
    OLED_WriteSCL(1U);
    OLED_I2C_Delay();
    OLED_WriteSDA(0U);
    OLED_I2C_Delay();
    OLED_WriteSCL(0U);
    OLED_I2C_Delay();
}

static void OLED_I2C_Stop(void)
{
    OLED_WriteSDA(0U);
    OLED_I2C_Delay();
    OLED_WriteSCL(1U);
    OLED_I2C_Delay();
    OLED_WriteSDA(1U);
    OLED_I2C_Delay();
}

static uint8_t OLED_I2C_SendByte(uint8_t byte)
{
    uint8_t bit;
    uint8_t acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        OLED_WriteSCL(0U);
        OLED_WriteSDA((byte & (uint8_t)(0x80U >> bit)) != 0U);
        OLED_I2C_Delay();
        OLED_WriteSCL(1U);
        OLED_I2C_Delay();
        OLED_WriteSCL(0U);
        OLED_I2C_Delay();
    }

    /* Release SDA so the OLED can pull it low during the ninth ACK clock. */
    OLED_WriteSDA(1U);
    OLED_I2C_Delay();
    OLED_WriteSCL(1U);
    OLED_I2C_Delay();
    acknowledged =
        (HAL_GPIO_ReadPin(OLED_SDA_GPIO_PORT,
                          OLED_SDA_GPIO_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    OLED_WriteSCL(0U);
    OLED_I2C_Delay();

    return acknowledged;
}

static uint8_t OLED_I2C_Probe(uint8_t write_address)
{
    uint8_t acknowledged;

    OLED_I2C_Start();
    acknowledged = OLED_I2C_SendByte(write_address);
    OLED_I2C_Stop();

    return acknowledged;
}

static HAL_StatusTypeDef OLED_WriteBlock(uint8_t control,
                                         const uint8_t *data,
                                         uint8_t length)
{
    uint8_t index;

    if (oled_connected == 0U || data == NULL || length == 0U) {
        return HAL_ERROR;
    }

    OLED_I2C_Start();
    if (OLED_I2C_SendByte(oled_write_address) == 0U ||
        OLED_I2C_SendByte(control) == 0U) {
        OLED_I2C_Stop();
        oled_connected = 0U;
        return HAL_ERROR;
    }

    for (index = 0U; index < length; index++) {
        if (OLED_I2C_SendByte(data[index]) == 0U) {
            OLED_I2C_Stop();
            oled_connected = 0U;
            return HAL_ERROR;
        }
    }
    OLED_I2C_Stop();

    return HAL_OK;
}

static HAL_StatusTypeDef OLED_WriteCommand(uint8_t command)
{
    return OLED_WriteBlock(0x00U, &command, 1U);
}

static HAL_StatusTypeDef OLED_WriteCommandBlock(const uint8_t *commands,
                                                 uint8_t length)
{
    return OLED_WriteBlock(0x00U, commands, length);
}

static HAL_StatusTypeDef OLED_WriteDataBlock(const uint8_t *data,
                                              uint8_t length)
{
    return OLED_WriteBlock(0x40U, data, length);
}

static void OLED_SetCursor(uint8_t page, uint8_t x)
{
    uint8_t commands[3] = {
        (uint8_t)(0xB0U | page),
        (uint8_t)(0x10U | ((x & 0xF0U) >> 4)),
        (uint8_t)(x & 0x0FU)
    };

    (void)OLED_WriteCommandBlock(commands, sizeof(commands));
}

static uint32_t OLED_Pow(uint32_t base, uint8_t exponent)
{
    uint32_t result = 1U;

    while (exponent > 0U) {
        result *= base;
        exponent--;
    }

    return result;
}

HAL_StatusTypeDef OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAEU,
        0x2EU,
        0xD5U, 0x80U,
        0xA8U, 0x3FU,
        0xD3U, 0x00U,
        0x20U, 0x02U,
        0x40U,
        0xA1U,
        0xC8U,
        0xDAU, 0x12U,
        0x81U, 0xCFU,
        0xD9U, 0xF1U,
        0xDBU, 0x30U,
        0xA4U,
        0xA6U,
        0x8DU, 0x14U,
        0xAFU
    };

    oled_connected = 0U;
    OLED_I2C_Init();
    HAL_Delay(100U);
    OLED_I2C_BusRecover();

    oled_write_address = (uint8_t)(OLED_ADDRESS_0 << 1U);
    if (OLED_I2C_Probe(oled_write_address) == 0U) {
        oled_write_address = (uint8_t)(OLED_ADDRESS_1 << 1U);
        if (OLED_I2C_Probe(oled_write_address) == 0U) {
            return HAL_ERROR;
        }
    }

    oled_connected = 1U;
    if (OLED_WriteCommandBlock(init_commands,
                               sizeof(init_commands)) != HAL_OK) {
        return HAL_ERROR;
    }

    OLED_Clear();
    return (oled_connected != 0U) ? HAL_OK : HAL_ERROR;
}

uint8_t OLED_IsConnected(void)
{
    return oled_connected;
}

uint8_t OLED_GetAddress(void)
{
    return (uint8_t)(oled_write_address >> 1U);
}

HAL_StatusTypeDef OLED_TestAllPixels(uint32_t hold_ms)
{
    if (OLED_WriteCommand(0xA5U) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(hold_ms);
    return OLED_WriteCommand(0xA4U);
}

void OLED_Clear(void)
{
    uint8_t blank[OLED_WIDTH_PIXELS] = {0};
    uint8_t page;

    if (oled_connected == 0U) {
        return;
    }

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        OLED_SetCursor(page, 0U);
        if (OLED_WriteDataBlock(blank, sizeof(blank)) != HAL_OK) {
            return;
        }
    }
}

void OLED_ShowChar(uint8_t line, uint8_t column, char character)
{
    uint8_t font_index;

    if (oled_connected == 0U ||
        line < 1U || line > 4U ||
        column < 1U || column > OLED_CHARACTER_COLUMNS) {
        return;
    }

    if (character < ' ' || character > '~') {
        character = '?';
    }

    font_index = (uint8_t)(character - ' ');

    OLED_SetCursor((uint8_t)((line - 1U) * 2U),
                   (uint8_t)((column - 1U) * 8U));
    OLED_WriteDataBlock(&OLED_F8x16[(uint16_t)font_index * 16U], 8U);

    OLED_SetCursor((uint8_t)((line - 1U) * 2U + 1U),
                   (uint8_t)((column - 1U) * 8U));
    OLED_WriteDataBlock(&OLED_F8x16[(uint16_t)font_index * 16U + 8U], 8U);
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0' && column <= OLED_CHARACTER_COLUMNS) {
        OLED_ShowChar(line, column, *text);
        text++;
        column++;
    }
}

void OLED_ShowNum(uint8_t line,
                  uint8_t column,
                  uint32_t number,
                  uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; index++) {
        uint32_t divisor = OLED_Pow(10U, (uint8_t)(length - index - 1U));
        OLED_ShowChar(line,
                      (uint8_t)(column + index),
                      (char)('0' + (number / divisor) % 10U));
    }
}

void OLED_ShowSignedNum(uint8_t line,
                        uint8_t column,
                        int32_t number,
                        uint8_t length)
{
    uint32_t magnitude;

    if (number < 0) {
        OLED_ShowChar(line, column, '-');
        magnitude = (uint32_t)(-(number + 1)) + 1U;
    } else {
        OLED_ShowChar(line, column, '+');
        magnitude = (uint32_t)number;
    }

    OLED_ShowNum(line, (uint8_t)(column + 1U), magnitude, length);
}

void OLED_ShowHexNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; index++) {
        uint32_t divisor = OLED_Pow(16U, (uint8_t)(length - index - 1U));
        uint8_t digit = (uint8_t)((number / divisor) % 16U);
        OLED_ShowChar(line,
                      (uint8_t)(column + index),
                      (digit < 10U) ?
                      (char)('0' + digit) :
                      (char)('A' + digit - 10U));
    }
}

void OLED_ShowBinNum(uint8_t line,
                     uint8_t column,
                     uint32_t number,
                     uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit_index = (uint8_t)(length - index - 1U);
        OLED_ShowChar(line,
                      (uint8_t)(column + index),
                      ((number >> bit_index) & 1U) != 0U ? '1' : '0');
    }
}
