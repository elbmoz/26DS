#ifndef _ENCODER_H
#define _ENCODER_H

#include "stm32f4xx_hal.h"

typedef struct {
    int8_t  dirt;           /* 方向: +1 正转, -1 反转               */
    int32_t count;          /* 本周期脉冲增量 (带符号)              */
    int32_t total_count;    /* 累积脉冲 (外部只读)                  */
} encoder_para_t;

void Encoder_init(void);
void Encoder_Reset(void);
void Encoder_task(void);                     /* 每 1ms 在主循环调用 */
const encoder_para_t *get_encoder_point(void); /* 获取只读指针      */

#endif
