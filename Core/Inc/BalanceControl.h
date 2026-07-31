#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * 任务 2 串级控制器：
 *
 * 小球位置/速度 -> 目标管道角 -> 电机速度
 *
 * 所有可配置参数集中在 BalanceControlConfig。调试时先稳定角度内环，
 * 再调小球位置外环，最后才启用近中心和高球速增益调度。
 */
typedef struct
{
    /* 小球位置外环：输入为 px/px/s，输出为目标管道角度。 */
    float outer_kp_deg_per_px;       /* 位置比例增益，单位 °/px。 */
    float outer_ki_deg_per_px_s;     /* 位置积分增益，单位 °/(px*s)。 */
    float outer_kd_deg_per_px_s;     /* 球速度阻尼增益，乘 -velocity。 */
    float outer_integral_limit_px_s; /* 误差积分绝对值上限，单位 px*s。 */
    float outer_angle_limit_deg;     /* 外环目标管道角绝对值上限，单位 °。 */

    /* 近中心增益调度与回水平保持：基础内外环调通后再调。 */
    float hold_band_px;              /* 进入近中心柔化区的误差阈值，单位 px。 */
    float fine_band_px;              /* 进入精细回水平区的误差阈值，单位 px。 */
    float fine_velocity_px_s;        /* 允许精细回水平的最大球速，单位 px/s。 */
    float soft_kp_scale;             /* 近中心柔化区的外环 Kp 倍率。 */
    float soft_kd_scale;             /* 近中心柔化区的外环 Kd 倍率。 */
    float soft_angle_limit_scale;    /* 近中心柔化区的目标角限幅倍率。 */
    float soft_ki_deg_per_px_s;      /* 近中心柔化区单独使用的 Ki。 */
    float fine_fast_kp_scale;        /* 球在精细区但速度较快时的 Kp 倍率。 */
    float fine_fast_ki_scale;        /* 精细区高速状态下 soft Ki 的倍率。 */
    float fine_fast_angle_limit_scale; /* 精细区高速状态的目标角限幅倍率。 */
    float hold_integral_decay;       /* 回水平保持时每帧积分保留比例。 */
    float fine_hold_inner_kp_scale;  /* 回水平保持时角度内环 Kp 倍率。 */

    /* 高球速阻尼调度：抑制高速冲过中心。 */
    float damping_velocity_px_s;     /* 开始高速阻尼调度的球速阈值，单位 px/s。 */
    float damping_kp_scale;          /* 高速阻尼区的外环 Kp 倍率。 */
    float damping_kd_scale;          /* 高速阻尼区的外环 Kd 倍率。 */
    float damping_angle_limit_scale; /* 高速阻尼区的目标角限幅倍率。 */
    float freeze_integral_velocity_px_s; /* 冻结积分的球速阈值，单位 px/s。 */
    float freeze_kp_scale;           /* 冻结积分区额外施加的 Kp 倍率。 */
    float freeze_kd_scale;           /* 冻结积分区额外施加的 Kd 倍率。 */
    float freeze_angle_limit_scale;  /* 冻结积分区额外施加的限幅倍率。 */
    float freeze_integral_decay;     /* 冻结积分时每帧积分保留比例。 */

    /* 电机位置到管道 X 角的拟合标定参数，不作为 PID 调参项。 */
    float motor_zero_angle_deg;      /* 管道水平时对应的电机角，单位 °。 */
    float rod_angle_per_motor_degree; /* 电机角变化到管道角变化的带符号比例。 */
    float rod_angle_limit_deg;       /* 实际管道角的软件保护范围，单位 °。 */
    uint8_t capture_motor_zero_on_start; /* 1：启动后用首个有效位置捕获水平零点。 */

    /* 管道角度内环：角度误差转换成 RS485 电机速度命令。 */
    float angle_kp_speed_per_deg;    /* 每 1°角度误差产生的速度命令。 */
    float motor_speed_limit;         /* 速度命令绝对值上限。 */
    float motor_speed_deadband;      /* 速度命令绝对值不超过该值时停车。 */
    float motor_min_speed;           /* 非零速度命令的最小绝对值。 */
    float motor_slew_per_update;     /* 每个控制周期允许的最大速度命令变化量。 */
    uint8_t motor_slope;             /* 驱动器 F6 加减速档；0 表示直接启停/换向。 */
    int8_t tilt_direction;           /* 外环输出到目标管道角的符号，取 +1/-1。 */
    int8_t motor_direction;          /* 角度误差到电机命令的符号，取 +1/-1。 */

    uint32_t control_period_ms;      /* 内外环计算周期，单位 ms。 */
    uint32_t motor_position_period_ms; /* 0x36 电机位置查询周期，单位 ms。 */
    uint32_t motor_position_timeout_ms; /* 位置数据超过该时间未更新即停机。 */

    float stable_error_px;           /* 稳定判定允许的位置误差，单位 px。 */
    float stable_velocity_px_s;      /* 稳定判定允许的球速，单位 px/s。 */
    uint16_t stable_frames;          /* 连续满足稳定条件所需的视觉帧数。 */

    float pixels_per_cm;             /* 视觉位置比例，单位 px/cm。 */
    float positive_5cm_target;       /* 后续 +5 cm 阶段的目标位置，单位 px。 */
    float negative_5cm_target;       /* 后续 -5 cm 阶段的目标位置，单位 px。 */
} BalanceControlConfig;

typedef struct
{
    volatile uint8_t enabled;        /* 1：任务 2 控制器正在运行。 */
    volatile uint8_t vision_valid;   /* 1：小球位置数据当前有效。 */
    volatile uint8_t motor_position_valid; /* 1：电机位置数据当前有效且未超时。 */
    volatile uint8_t leveling;       /* 1：当前命令管道回到水平。 */
    volatile uint8_t stable;         /* 1：小球已连续满足稳定判定。 */
    volatile uint8_t motor_zero_pending; /* 1：仍在等待捕获本次水平零点。 */

    /* 外环观测量：球位置/速度 -> 目标管道角。 */
    volatile float target_position;       /* 球的目标位置，当前回中为 0。 */
    volatile float ball_position;         /* MaixCAM 发送的球位置，单位 px。 */
    volatile float ball_velocity;         /* 球沿管道速度，单位 px/s。 */
    volatile float position_error;        /* target_position-ball_position。 */
    volatile float position_p_term;       /* 外环 P 产生的目标角分量，单位 °。 */
    volatile float position_i_term;       /* 外环 I 产生的目标角分量，单位 °。 */
    volatile float velocity_d_term;       /* 外环 D 产生的目标角分量，单位 °。 */
    volatile float outer_integral;        /* 外环误差积分，单位 px*s。 */
    volatile float outer_output_deg;      /* 方向变换前的外环输出角。 */
    volatile float target_rod_angle_deg;  /* 内环需要追踪的目标管道 X 角。 */

    /* 内环观测量：实际管道角 -> 电机速度命令。 */
    volatile int32_t motor_position;      /* 0x36 返回的电机原始位置。 */
    volatile float motor_angle_deg;       /* 底层换算后的电机角度。 */
    volatile float rod_angle_deg;         /* 拟合得到的实际管道 X 角。 */
    volatile float angle_error_deg;       /* 目标管道角-实际管道角。 */
    volatile float desired_motor_speed;   /* 已限幅、尚未量化的连续速度目标。 */
    volatile int32_t motor_command;       /* 最终发送的有符号电机速度命令。 */

    volatile uint16_t stable_count;  /* 当前连续稳定视觉帧计数。 */
    volatile uint32_t update_count;  /* 任务 2 总控制周期计数。 */
    volatile uint32_t outer_update_count; /* 实际处理的新视觉帧计数。 */
    volatile uint32_t motor_position_update_count; /* 已使用的位置回包计数。 */
    volatile HAL_StatusTypeDef last_motor_status; /* 最近一次电机控制命令的 HAL 状态。 */
} BalanceControlState;

extern BalanceControlConfig balance_control_config;
extern BalanceControlState balance_control_state;

void BalanceControl_Init(void);
void BalanceControl_Start(float target_position);
void BalanceControl_SetTarget(float target_position);
void BalanceControl_Update(void);
void BalanceControl_Stop(void);
uint8_t BalanceControl_IsStable(void);

#ifdef __cplusplus
}
#endif

#endif
