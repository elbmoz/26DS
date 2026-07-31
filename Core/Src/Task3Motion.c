#include "Task3Motion.h"

#include "DS.h"

/*
 * ======================== 任务 3 实车调参区 ========================
 *
 * 3 号电机的速度输入分辨率配置成功后，命令值 10 = 1.0 RPM。
 * 若上电配置没有收到成功回复，各段会改用下面的统一安全速度命令
 * TASK3_MOTION_SAFE_SPEED_COMMAND（当前为 2），避免把表中的 10 直接
 * 当成 10 RPM；此时实际速度可能是 0.2 RPM 或 2 RPM，但任务仍可
 * 启动，不阻塞整机流程。
 *
 * 当前约定：正方向是“上”，负方向是“下”。若实车方向相反，只把
 * TASK3_MOTION_DIRECTION_SIGN 改为 -1，不要逐项交换步骤方向。
 */
#define TASK3_MOTION_SAFE_SPEED_COMMAND      2
#define TASK3_MOTION_SLOPE                  10U
#define TASK3_MOTION_DIRECTION_SIGN          1
#define TASK3_MOTION_MAX_RUNTIME_MS       5000U

typedef struct
{
    Task3MotionDirection direction;
    uint16_t speed_command;
    uint32_t duration_ms;
} Task3MotionStep;

/*
 * 每一行就是“方向 + 速度命令 + 持续时间”。速度配置成功时，速度命令
 * 10 = 1.0 RPM、12 = 1.2 RPM，以此类推。每一段的速度和时间都能
 * 单独修改；相邻两段直接切换，不插入停车等待。要增加动作，复制一行
 * 即可，Task3Motion_Update() 会按顺序非阻塞执行。
 *
 * 题目要求总运行时间 <= 5 s，因此所有 duration_ms 的总和也必须
 * <= 5000 ms。当前前两段为 300/800 ms；后续 0.x 秒均是首轮
 * 占位值，必须根据小球实测位置逐段调整。最后一段保留下行，便于
 * 最终停在 -5 cm 一侧；开环定时本身不能保证 ±1 cm，仍需实车标定。
 */
static const Task3MotionStep task3_motion_steps[] = {
    /* 方向                 速度    时间 */
    {TASK3_MOTION_DOWN, 10U, 300U}, /* 第 1 段：下行。 */
    {TASK3_MOTION_UP,   10U, 800U}, /* 第 2 段：立即上行。 */
    {TASK3_MOTION_DOWN, 10U, 300U}, /* 第 3 段：下行。 */
    {TASK3_MOTION_UP,   10U, 500U}, /* 第 4 段：上行。 */
    {TASK3_MOTION_DOWN, 10U, 300U}, /* 第 5 段：下行。 */
    {TASK3_MOTION_UP,   10U, 500U}, /* 第 6 段：上行。 */
    {TASK3_MOTION_DOWN, 10U, 300U}  /* 第 7 段：最后下行并停车。 */
};

#define TASK3_MOTION_STEP_COUNT \
    ((uint8_t)(sizeof(task3_motion_steps) / \
               sizeof(task3_motion_steps[0])))

Task3MotionState task3_motion_state;

static uint16_t Task3Motion_GetSpeedCommand(
    uint16_t configured_speed)
{
    const DS_State *state = DS_GetState();

    if (state->balance_speed_scale_status == HAL_OK) {
        return configured_speed;
    }

    return TASK3_MOTION_SAFE_SPEED_COMMAND;
}

static int32_t Task3Motion_GetSignedSpeed(
    Task3MotionDirection direction,
    uint16_t speed_command)
{
    return (int32_t)direction *
           TASK3_MOTION_DIRECTION_SIGN *
           (int32_t)speed_command;
}

static void Task3Motion_SetFault(HAL_StatusTypeDef status)
{
    (void)DS_BalanceStop();
    task3_motion_state.active = 0U;
    task3_motion_state.completed = 0U;
    task3_motion_state.fault = 1U;
    task3_motion_state.current_direction = TASK3_MOTION_STOP;
    task3_motion_state.last_status = status;
}

/*
 * 启动指定步骤。只有 RS485 速度命令被接受后才记录起始时刻，所以
 * HAL_BUSY 重试期间不会偷走该段的有效运行时间。
 */
static void Task3Motion_TryStartStep(uint8_t step_index,
                                     uint32_t now)
{
    HAL_StatusTypeDef status;
    uint16_t speed_command;

    speed_command = Task3Motion_GetSpeedCommand(
        task3_motion_steps[step_index].speed_command);

    status = DS_BalanceSetSpeed(
        Task3Motion_GetSignedSpeed(
            task3_motion_steps[step_index].direction,
            speed_command),
        TASK3_MOTION_SLOPE);
    task3_motion_state.last_status = status;

    if (status == HAL_BUSY) {
        return;
    }
    if (status != HAL_OK) {
        Task3Motion_SetFault(status);
        return;
    }

    task3_motion_state.step_index = step_index;
    task3_motion_state.current_direction =
        task3_motion_steps[step_index].direction;
    task3_motion_state.speed_command = speed_command;
    task3_motion_state.step_started_ms = now;
}

void Task3Motion_Init(void)
{
    task3_motion_state.active = 0U;
    task3_motion_state.completed = 0U;
    task3_motion_state.fault = 0U;
    task3_motion_state.step_index = 0U;
    task3_motion_state.current_direction = TASK3_MOTION_STOP;
    task3_motion_state.speed_command =
        task3_motion_steps[0].speed_command;
    task3_motion_state.sequence_started_ms = 0U;
    task3_motion_state.step_started_ms = 0U;
    task3_motion_state.last_status = HAL_OK;
}

void Task3Motion_Start(void)
{
    uint32_t now = HAL_GetTick();

    task3_motion_state.active = 1U;
    task3_motion_state.completed = 0U;
    task3_motion_state.fault = 0U;
    task3_motion_state.step_index = 0U;
    task3_motion_state.current_direction = TASK3_MOTION_STOP;
    task3_motion_state.speed_command =
        Task3Motion_GetSpeedCommand(
            task3_motion_steps[0].speed_command);
    task3_motion_state.sequence_started_ms = now;
    task3_motion_state.step_started_ms = now;
    task3_motion_state.last_status = HAL_BUSY;

    /* 确认键启动后，立即尝试执行第 1 段下行动作。 */
    Task3Motion_TryStartStep(0U, now);
}

void Task3Motion_Update(void)
{
    uint8_t next_step;
    uint32_t now;
    HAL_StatusTypeDef stop_status;

    if (task3_motion_state.active == 0U) {
        return;
    }

    now = HAL_GetTick();

    /* 从确认键开始硬限制为 5 s，调参表写错也不会持续驱动电机。 */
    if ((uint32_t)(now -
                   task3_motion_state.sequence_started_ms) >=
        TASK3_MOTION_MAX_RUNTIME_MS) {
        Task3Motion_SetFault(HAL_TIMEOUT);
        return;
    }

    /* 第 1 段若曾遇到通信忙，在这里持续重试，成功后才开始计时。 */
    if (task3_motion_state.current_direction ==
        TASK3_MOTION_STOP) {
        Task3Motion_TryStartStep(
            task3_motion_state.step_index,
            now);
        return;
    }

    if ((uint32_t)(now - task3_motion_state.step_started_ms) <
        task3_motion_steps[task3_motion_state.step_index].duration_ms) {
        return;
    }

    next_step = (uint8_t)(task3_motion_state.step_index + 1U);
    if (next_step < TASK3_MOTION_STEP_COUNT) {
        /* 到时后直接发送反方向速度，不额外插入停车时间。 */
        Task3Motion_TryStartStep(next_step, now);
        return;
    }

    stop_status = DS_BalanceStop();
    task3_motion_state.last_status = stop_status;
    task3_motion_state.active = 0U;
    task3_motion_state.current_direction = TASK3_MOTION_STOP;

    if (stop_status == HAL_OK) {
        task3_motion_state.completed = 1U;
    } else {
        task3_motion_state.fault = 1U;
    }
}

void Task3Motion_Stop(void)
{
    HAL_StatusTypeDef status = DS_BalanceStop();

    task3_motion_state.active = 0U;
    task3_motion_state.current_direction = TASK3_MOTION_STOP;
    task3_motion_state.last_status = status;
    if (status != HAL_OK) {
        task3_motion_state.fault = 1U;
    }
}

uint8_t Task3Motion_GetStepCount(void)
{
    return TASK3_MOTION_STEP_COUNT;
}

uint8_t Task3Motion_IsComplete(void)
{
    return task3_motion_state.completed;
}

uint8_t Task3Motion_HasFault(void)
{
    return task3_motion_state.fault;
}
