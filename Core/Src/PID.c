#include "PID.h"
#include <stddef.h>

float LimitValue(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

float Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void PID_Cycle_Reset(PID_Cycle *p)
{
    if (p == NULL) return;
    p->error = 0.0f;  p->last_error = 0.0f;  p->prev_error = 0.0f;
    p->integral = 0.0f;  p->derivative = 0.0f;
    p->out = 0.0f;  p->last_out = 0.0f;
}

void PID_Cycle_SetEnable(PID_Cycle *p, uint8_t en)
{
    if (p) p->enable = en ? 1U : 0U;
}

float PID_Control(PID_Cycle *p, float target, float feedback)
{
    if (p == NULL || p->enable == 0U) return 0.0f;

    p->prev_error = p->last_error;
    p->last_error = p->error;
    p->error = target - feedback;

    if (Abs(p->error) <= p->deadband) p->error = 0.0f;

    p->integral += p->error;
    p->integral = LimitValue(p->integral, -p->i_max, p->i_max);
    p->derivative = p->error - p->last_error;

    if (p->mode == PID_INCREMENTAL) {
        float inc = p->p * (p->error - p->last_error)
                  + p->i * p->error
                  + p->d * (p->error - 2.0f * p->last_error + p->prev_error);
        p->out = p->last_out + inc;
    } else {
        p->out = p->p * p->error + p->i * p->integral + p->d * p->derivative;
    }

    p->out = LimitValue(p->out, -p->out_max, p->out_max);
    p->last_out = p->out;
    return p->out;
}
