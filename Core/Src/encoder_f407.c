/*
 * encoder_f407.c — 单路编码器驱动 (STM32F407 + TIM4 CH1 + PD13 方向)
 *
 * 硬件连接:
 *   PD12 → TIM4_CH1 (AF2, 编码器脉冲输入)
 *   PD13 → GPIO_Input  (方向信号, 高=正转, 低=反转)
 *
 * 用法:
 *   main.c 初始化阶段调用 Encoder_init()
 *   主循环每 1ms 调用 Encoder_task()
 *   其他地方通过 get_encoder_point()->total_count 读取累积脉冲
 */
#include "encoder_f407.h"

/* ========== 引脚配置 (按需修改) ========== */
#define ENC_CH1_PORT         GPIOD
#define ENC_CH1_PIN          GPIO_PIN_12    /* TIM4_CH1, AF2 */
#define ENC_DIRT_PORT        GPIOD
#define ENC_DIRT_PIN         GPIO_PIN_13    /* 方向 */

static TIM_HandleTypeDef  htim4;            /* 内部定义, 不依赖 CubeMX */
static encoder_para_t     g_encoder;        /* 单例 */

/* ========== GPIO + TIM4 初始化 ========== */
static void MX_TIM4_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStruct = {0};

    /* ── 1. 时钟 ── */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* ── 2. PD12 → TIM4_CH1 (AF2) ── */
    GPIO_InitStruct.Pin       = ENC_CH1_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(ENC_CH1_PORT, &GPIO_InitStruct);

    /* ── 3. PD13 → 方向输入 (上拉) ── */
    GPIO_InitStruct.Pin       = ENC_DIRT_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(ENC_DIRT_PORT, &GPIO_InitStruct);

    /* ── 4. TIM4 基础配置 ── */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);

    /* ── 5. 从模式: External Clock Mode 1, Trigger = TI1FP1 ── */
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    sSlaveConfig.SlaveMode         = TIM_SLAVEMODE_EXTERNAL1;
    sSlaveConfig.InputTrigger      = TIM_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity   = TIM_TRIGGERPOLARITY_NONINVERTED;
    sSlaveConfig.TriggerPrescaler  = TIM_TRIGGERPRESCALER_DIV1;
    sSlaveConfig.TriggerFilter     = 0;
    HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig);
}

/* ========== 初始化 ========== */
void Encoder_init(void)
{
    MX_TIM4_Init();
    Encoder_Reset();
    HAL_TIM_Base_Start(&htim4);
}

void Encoder_Reset(void)
{
    __HAL_TIM_SetCounter(&htim4, 0);
    g_encoder.dirt = 1;
    g_encoder.count = 0;
    g_encoder.total_count = 0;
}

/* ========== 每 1ms 调用 ========== */
void Encoder_task(void)
{
    /* 1. 读方向: GPIO 电平 {0,1} → 方向 {-1, +1} */
    g_encoder.dirt = HAL_GPIO_ReadPin(ENC_DIRT_PORT, ENC_DIRT_PIN) * 2 - 1;

    /* 2. 读脉冲增量并清零 */
    g_encoder.count = (int32_t)__HAL_TIM_GetCounter(&htim4) * g_encoder.dirt;
    __HAL_TIM_SetCounter(&htim4, 0);

    /* 3. 累加 */
    g_encoder.total_count += g_encoder.count;
}

/* ========== 外部只读访问 ========== */
const encoder_para_t *get_encoder_point(void)
{
    return &g_encoder;
}
