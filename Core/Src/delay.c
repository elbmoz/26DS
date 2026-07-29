#include "delay.h"
/**
 * @brief  初始化 DWT 延时所需寄存器
 *         必须在 SystemClock_Config() 之后、任何 delay_us() 调用之前执行一次。
 * @return  0 = 成功，1 = 不支持 DWT
 */
uint8_t DWT_Delay_Init(void)
{
    // 使能对 DWT 和 ITM 的时钟访问
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 确保 DWT->CYCCNT 清零
    DWT->CYCCNT = 0;
    // 使能 CYCCNT 计数
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 检测是否真的启动
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        return 1;  // 不支持
    }
    return 0;      // 支持，已启用
}

/**
 * @brief  精确微秒级延时 (基于 DWT)
 * @param  us: 要延时的微秒数
 * @note   调用前须已执行过 DWT_Delay_Init()
 */
void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles)
        ;
}

void delay_ms(uint32_t ms)
{
	delay_us(1000*ms);
}
