#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_POSITION     0
#define PID_INCREMENTAL  1

typedef struct {
    float p, i, d;
    float out_max;
    float i_max;
    float deadband;
    uint8_t mode;
    uint8_t enable;

    float error;
    float last_error;
    float prev_error;
    float integral;
    float derivative;
    float out;
    float last_out;
} PID_Cycle;

void PID_Cycle_Reset(PID_Cycle *pid);
void PID_Cycle_SetEnable(PID_Cycle *pid, uint8_t enable);
float PID_Control(PID_Cycle *pid, float target, float feedback);

float LimitValue(float value, float min_value, float max_value);
float Abs(float value);

#endif
