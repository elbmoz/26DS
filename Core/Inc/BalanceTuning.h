#ifndef BALANCE_TUNING_H
#define BALANCE_TUNING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define BALANCE_TUNING_OUTER_KP       (1UL << 0)
#define BALANCE_TUNING_OUTER_KD       (1UL << 1)
#define BALANCE_TUNING_ANGLE_LIMIT    (1UL << 2)
#define BALANCE_TUNING_INNER_KP       (1UL << 3)
#define BALANCE_TUNING_INNER_KD       (1UL << 4)
#define BALANCE_TUNING_SPEED_LIMIT    (1UL << 5)
#define BALANCE_TUNING_SLEW           (1UL << 6)
#define BALANCE_TUNING_DEADBAND       (1UL << 7)
#define BALANCE_TUNING_MIN_SPEED      (1UL << 8)
#define BALANCE_TUNING_OUTER_KI       (1UL << 9)
#define BALANCE_TUNING_ALL            ((1UL << 10) - 1UL)

#define BALANCE_TUNING_ACTION_RESET   (1U << 0)
#define BALANCE_TUNING_ACTION_STOP    (1U << 1)

typedef enum
{
    BALANCE_TUNING_MODE_NORMAL = 0,
    BALANCE_TUNING_MODE_INNER_STEP = 1,
    BALANCE_TUNING_MODE_OUTER_STEP = 2,
    BALANCE_TUNING_MODE_PAUSED = 3,
    BALANCE_TUNING_MODE_INNER_PROFILE = 4,
    BALANCE_TUNING_MODE_OUTER_PROFILE = 5
} BalanceTuningMode;

/*
 * A profile is one self-contained experiment executed on the MCU:
 * zero/precondition -> positive step -> negative step -> zero/return.
 * Keeping phase timing on STM32 removes Windows/network latency from the
 * experiment and lets one parameter candidate be evaluated with one command.
 */
typedef enum
{
    BALANCE_TUNING_PHASE_PREPARE = 0,
    BALANCE_TUNING_PHASE_POSITIVE = 1,
    BALANCE_TUNING_PHASE_NEGATIVE = 2,
    BALANCE_TUNING_PHASE_RETURN = 3,
    BALANCE_TUNING_PHASE_DONE = 4,
    BALANCE_TUNING_PHASE_ABORTED = 5
} BalanceTuningPhase;

void BalanceTuning_Init(void);
void BalanceTuning_OnControllerStart(void);
uint8_t BalanceTuning_ProcessLineFromISR(const char *line);
uint8_t BalanceTuning_ApplyPendingAtControlBoundary(uint32_t now_ms);
uint8_t BalanceTuning_Update(uint32_t now_ms);
void BalanceTuning_FlushAck(void);
uint8_t BalanceTuning_FeedbackV2Enabled(void);
uint8_t BalanceTuning_FeedbackV3Enabled(void);
uint8_t BalanceTuning_GetInnerTarget(float *target_angle_deg);
uint8_t BalanceTuning_GetOuterTarget(float *target_position_px);
uint8_t BalanceTuning_ForceStop(void);
uint8_t BalanceTuning_GetMode(void);
float BalanceTuning_GetTestTarget(void);
uint32_t BalanceTuning_GetRemainingMs(uint32_t now_ms);
uint32_t BalanceTuning_GetProfileSequence(void);
uint8_t BalanceTuning_GetProfilePhase(void);
uint32_t BalanceTuning_GetProfilePhaseElapsedMs(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
