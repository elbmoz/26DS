#include "OLED.h"

#include "OLED_Font.h"
#include "i2c.h"

#define OLED_WIDTH_PIXELS            128U
#define OLED_PAGE_COUNT              8U
#define OLED_CHARACTER_COLUMNS       16U
#define OLED_I2C_TIMEOUT_MS          100U
#define OLED_I2C_READY_TRIALS        3U

static uint8_t oled_write_address = (uint8_t)(OLED_ADDRESS_0 << 1U);
static volatile uint8_t oled_connected;
static uint8_t oled_tx_buffer[OLED_WIDTH_PIXELS + 1U];

static uint8_t OLED_I2C_Probe(uint8_t write_address)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1,
                                  write_address,
                                  OLED_I2C_READY_TRIALS,
                                  OLED_I2C_TIMEOUT_MS) == HAL_OK) ?
           1U :
           0U;
}

static HAL_StatusTypeDef OLED_WriteBlock(uint8_t control,
                                         const uint8_t *data,
                                         uint16_t length)
{
    HAL_StatusTypeDef status;
    uint16_t index;

    if (oled_connected == 0U ||
        data == NULL ||
        length == 0U ||
        length > OLED_WIDTH_PIXELS) {
        return HAL_ERROR;
    }

    oled_tx_buffer[0] = control;
    for (index = 0U; index < length; index++) {
        oled_tx_buffer[index + 1U] = data[index];
    }

    status = HAL_I2C_Master_Transmit(&hi2c1,
                                     oled_write_address,
                                     oled_tx_buffer,
                                     (uint16_t)(length + 1U),
                                     OLED_I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        oled_connected = 0U;
    }

    return status;
}

static HAL_StatusTypeDef OLED_WriteCommand(uint8_t command)
{
    return OLED_WriteBlock(0x00U, &command, 1U);
}

static HAL_StatusTypeDef OLED_WriteCommandBlock(const uint8_t *commands,
                                                 uint16_t length)
{
    return OLED_WriteBlock(0x00U, commands, length);
}

static HAL_StatusTypeDef OLED_WriteDataBlock(const uint8_t *data,
                                              uint16_t length)
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

    if (hi2c1.Instance != I2C1 ||
        hi2c1.State == HAL_I2C_STATE_RESET) {
        return HAL_ERROR;
    }

    HAL_Delay(100U);

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
    return (oled_connected != 0U) ?
           (uint8_t)(oled_write_address >> 1U) :
           0U;
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
    static const uint8_t blank[OLED_WIDTH_PIXELS] = {0};
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
    (void)OLED_WriteDataBlock(
        &OLED_F8x16[(uint16_t)font_index * 16U],
        8U);

    OLED_SetCursor((uint8_t)((line - 1U) * 2U + 1U),
                   (uint8_t)((column - 1U) * 8U));
    (void)OLED_WriteDataBlock(
        &OLED_F8x16[(uint16_t)font_index * 16U + 8U],
        8U);
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
