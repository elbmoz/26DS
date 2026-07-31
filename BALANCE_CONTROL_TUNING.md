# 第二题串级平衡控制与标定

## 当前控制结构

第二题已经由“视觉误差直接控制电机速度”改为参考工程使用的串级结构：

```text
MaixCAM 球位置/球速度
          |
          v
位置 PID 外环 -------> 目标杆角
                           |
0x36 电机位置 -> 实际杆角  |
          |                |
          +------ 角度误差-+
                           |
                           v
                    角度 P 内环
                           |
                           v
               RS485 有符号速度命令
```

外环只在收到新的视觉帧时更新，避免同一帧被重复积分。内环按
`control_period_ms` 运行，使用 `MotorPositionMonitor` 的最近一次有效位置。
控制计算完成后再推进非阻塞 `0x36` 查询，避免位置请求占用 USART1 时始终
无法发送速度命令。

题目 2 使用 `motor_position_period_ms` 指定位置读取周期；题目 9 独立使用
`DS_TASK_Q9_POSITION_PERIOD_MS=20 ms`，以 50 Hz 采集标定位置，不改变
题目 2 参数。

## 任务流程

1. PB6 选择题号 2。
2. PB7 启动后，控制器先停止 3 号电机、启动位置读取，并启动 USART6 视觉
   流、发送 `c2`。
3. MaixCAM 发送 `B,error_px,velocity_px_s\n`。
4. 第一次有效电机位置到达后，如果
   `capture_motor_zero_on_start=1`，该位置会被记录为本次水平零点。
5. 有效视觉控制目标杆角；视觉失效时目标杆角回到 0°。
6. 电机位置超过超时阈值时立即停止电机，不允许无角度反馈运行。
7. 再按 PB7，先取消位置查询，再停止电机并向 MaixCAM 发送 `ok`。

当前仍只持续调节到中心，不自动结束。后续 `+5 cm -> -5 cm` 状态机可调用：

```c
BalanceControl_SetTarget(balance_control_config.positive_5cm_target);
BalanceControl_SetTarget(balance_control_config.negative_5cm_target);
```

当前视觉标定为 `18.2 px/cm`，预留目标为 `+91 px` 和 `-91 px`。

### 已应用的电机位置—管道角拟合

Git 提交 `a010e37` 的稳态内点给出：

```text
rod_x_deg = -0.0020022658 * motor_position + intercept
```

当前硬件使用六字节 `0x36` 回包，底层换算为：

```text
motor_angle_deg = motor_position * 360 / 65536
```

因此任务 2 使用：

```text
rod_angle_per_motor_degree
    = -0.0020022658 / (360 / 65536)
    = -0.36450137
```

拟合的 RMSE 为 `0.179°`、R² 为 `0.999308`，验证范围为原始位置
`1817..9207`、管道 X 角 `-8.6..+6.6°`。正反向零点差等效约 `0.114°`，
当前不增加回差补偿。为了在两侧都留在拟合范围内，外环目标角限幅设为
最终不超过 `6.0°`，管道角软件保护限幅设为 `6.5°`。首轮内外环调试期间
进一步把目标角限制为 `1.5°`。

该数据中的零角位置 `P=5025.92` 来自当次采集的相对 IMU 零点，不能作为
跨上电机械零点。因此继续保持 `capture_motor_zero_on_start=1`：进入题目 2
前必须先把管道物理调平，首次有效电机角度作为本次水平锚点。只有增加可重复
的机械回零基准后，才应固定 `motor_zero_angle_deg` 并关闭自动捕获。

## 串口与控制反馈

USART6 使用 PC6=TX、PC7=RX，115200、8N1：

```text
B,<error_px>,<velocity_px_s>\n
none\n
```

误差和速度均以视觉右侧为正。MaixCAM 的 480 宽检测结果发送前乘
`640/480`，STM32 使用参考像素标定。

STM32 保持原有 13 字段反馈协议：

```text
F,<seq>,<mcu_ms>,<vision_frame>,<vision_age_ms>,
  <position_x10>,<velocity_x10>,<error_x10>,
  <p_x100>,<i_x100>,<d_x100>,<motor_command>,<motor_status>\n
```

其中 P/I/D 已变为“目标杆角外环”的三个分量，除以 100 后单位为度；
`motor_command` 是角度内环最终尝试发送的 RS485 速度命令。协议字段数不变，
现有 MaixCAM 和 Windows 记录工具仍可解析。

## 控制公式和方向

外环：

```text
error_px = target_px - ball_position_px

u_deg = Kp * error_px
      + Ki * integral(error_px * dt)
      + Kd * (-ball_velocity_px_s)

target_rod_angle_deg = tilt_direction * clamp(u_deg, angle_limit)
```

内环：

```text
rod_angle_deg =
    (motor_angle_deg - motor_zero_angle_deg)
    * rod_angle_per_motor_degree

angle_error_deg = target_rod_angle_deg - rod_angle_deg

motor_speed =
    motor_direction
    * angle_kp_speed_per_deg
    * angle_error_deg
```

当前已知关系是“正电机命令让球向视觉左侧运动”，且拟合证明正命令使
电机位置增大、管道 X 角减小，因此当前方向为：

```text
tilt_direction  = +1
motor_direction = -1
```

球位于右侧时位置误差为负，经 `tilt_direction=+1` 得到负 X 目标角；
`motor_direction=-1` 随后发出正电机命令，使管道 X 角减小并让球向左回中。
不要再用旧方向组合覆盖这三个已经配套的符号参数。

## 从参考工程迁移的调度逻辑

以下行为已经保留：

- 距目标较远时使用基础 P/I/D 和角度限幅。
- 进入 `hold_band_px` 后降低 P、D 和目标角限幅，并使用近中心积分消除静差。
- 进入 `fine_band_px` 且速度很小时，不继续倾杆追位置，而是让杆回到 0°。
- 球速超过 `damping_velocity_px_s` 后降低 P、增强 D、收紧目标角。
- 球速超过 `freeze_integral_velocity_px_s` 后暂停积分并逐步泄放。
- 电机速度经过最大值、死区、最小有效速度和每周期 slew 限制。
- 杆角越过 `rod_angle_limit_deg` 时清积分并强制目标回到 0°。
- 丢失视觉时只执行回水平；丢失电机位置时立即停机。

## 暴露的关键参数

参数集中在 `Core/Src/BalanceControl.c` 的
`balance_control_config`。

### 外环和近中心调度

| 参数 | 当前默认值 | 单位/作用 |
|---|---:|---|
| `outer_kp_deg_per_px` | 0.06044 | 位置 P，deg/px |
| `outer_ki_deg_per_px_s` | 0 | 首轮关闭；P/D 稳定后可从参考候选 0.03297 小步增加 |
| `outer_kd_deg_per_px_s` | 0.18681 | 视觉速度 D，deg/(px/s) |
| `outer_integral_limit_px_s` | 109.2 | 积分限幅，px·s |
| `outer_angle_limit_deg` | 1.5 | 首轮低风险目标角限幅；最终上限 6.0° |
| `hold_band_px` | 18.2 | 约等于离目标 1 cm |
| `fine_band_px` | 3.64 | 约等于离目标 2 mm |
| `fine_velocity_px_s` | 18.2 | 精细区回水平速度阈值 |
| `soft_ki_deg_per_px_s` | 0 | 首轮关闭；后续从参考候选 0.10989 以下逐步增加 |
| `damping_velocity_px_s` | 50.96 | 开始高速阻尼 |
| `freeze_integral_velocity_px_s` | 81.90 | 冻结积分速度 |

这些数值由参考工程的 mm/mm/s 参数按 `18.2 px/cm` 换算而来，并不代表
已经适配当前机械结构和 MaixCAM 速度噪声。

### 位置、角度和执行器

| 参数 | 当前默认值 | 作用 |
|---|---:|---|
| `motor_zero_angle_deg` | 0 | 标定后的电机水平零点 |
| `rod_angle_per_motor_degree` | -0.36450137 | 拟合得到的电机角到管道 X 角带符号比例 |
| `rod_angle_limit_deg` | 6.5 | 管道角软件保护范围，保留约 0.5°跟踪余量 |
| `capture_motor_zero_on_start` | 1 | 首个有效位置作为临时零点 |
| `angle_kp_speed_per_deg` | 现场值 | 角度内环 P，以 `BalanceControl.c` 为准 |
| `motor_speed_limit` | 30 | 首轮 RS485 最大速度命令，约 3 RPM |
| `motor_speed_deadband` | 现场值 | 小于等于该值视为停止，以 `BalanceControl.c` 为准 |
| `motor_min_speed` | 1 | 最小非零命令；S_Vel_IS 启用后对应 0.1 RPM |
| `motor_slew_per_update` | 现场值 | 每个控制周期最大速度变化，以 `BalanceControl.c` 为准 |
| `motor_slope` | 0 | 电机驱动器速度模式斜率字段 |
| `control_period_ms` | 20 ms | 串级控制周期 |
| `motor_position_period_ms` | 20 ms | 题目 2 的 0x36 查询周期 |
| `motor_position_timeout_ms` | 60 ms | 位置失效停机阈值 |

`DS_Init()` 每次上电都会给 3 号电机发送不保存的 `S_Vel_IS=Enable`
配置；收到正确应答后，速度和位置模式中的命令值 `1` 对应实际 `0.1 RPM`，
且不会在每次上电时写驱动器 Flash。配置状态可在 Keil Watch 中查看
`ds_task` 对应硬件状态里的 `DS_GetState()->balance_speed_scale_status`。
配置无应答或失败只会记录状态，不会阻断上电 `+240` 和任务菜单。

`motor_speed_deadband` 仍优先执行：只要死区大于等于 1，命令值 1 就会被
控制器置零；这是抑制零点抖动的策略，不代表驱动器不支持 0.1 RPM。如需在
任务 2 中实际发出命令值 1，应把死区调到小于 1。

角度比例已经按实测传动关系更新，但角度内环 P、速度限幅、死区和 slew
属于现场调试值，均以 `BalanceControl.c` 中的当前值为准。

稳定判定使用 `stable_error_px=18.2`、`stable_velocity_px_s=25` 和
`stable_frames=25`。计数单位是新的视觉帧，50 Hz 时约需连续 0.5 秒。

## OLED 题目 2 页面

```text
Q2 RX:V P:V S:X
E:-027.0 V:+018
M:+016 A:+001.2
TIME:0000.0s D:1
```

- `RX:V/X`：视觉有效/无效。
- `P:V/X`：电机位置反馈有效/无效。
- `S:V/X`：已经稳定/仍在调节。
- `E`、`V`：球的位置误差和轴向速度。
- `M`：最终有符号电机速度命令。
- `A`：相对水平零点的实际杆角。
- `D:1`：任务 2 OLED 正以 5 Hz 刷新。运行中按 PB6/K1 可切换到
  `OLED REFRESH OFF` 页面；该页面只写入一次，随后停止全部任务 2 OLED/I2C
  输出，而平衡控制与串口通信继续运行。再次按 PB6/K1 恢复刷新，PB7/K2
  的停止功能不变。

## 首次标定和调试顺序

1. 提交 `a010e37` 的拟合比例、方向和安全角度范围已经写入任务 2，不要再
   恢复旧的 `1.0/-1/+1` 参数组合。
2. 每次进入任务 2 前先把管道物理调平；启动后确认 `P:X` 很快变为
   `P:V`，OLED 实际管道角 `A` 从接近 0°开始。
3. 不放球、托住机构，小范围手动改变姿态，确认电机位置增大时 `A` 变负，
   且显示角变化量与实际 X 角一致。
4. 当前已经把 `outer_angle_limit_deg` 限制为 `1.5°`、`motor_speed_limit`
   限制为 `30`、slew 限制为 `8`，并关闭两个积分；确保有人随时按 PB7。
5. 托住球制造小的正、负误差，分别确认目标管道角、实际管道角和电机命令
   方向；先核对接线、坐标和拟合数据，不要单独翻转某个已配套方向来掩盖问题。
6. 先验证角度内环能快速追角且不振荡，再调球位置外环。
7. 首轮外环调试可先把两个 Ki 参数设为 0，只调 P 和 D；确认没有持续偏差后
   再恢复小积分。
8. 若球来回冲过中心，优先降低目标角限幅或 P，并检查视觉速度噪声，再调整
   D；若电机方向频繁跳变，降低 D 或增加速度滤波。
9. 最后验证视觉 `none` 时杆能回水平，拔掉位置反馈时电机能在 60 ms 左右
   停止，再逐步放开速度和角度限制。
10. 两侧中心控制可靠后，再实现 `+5 cm -> -5 cm` 状态机。

默认 stream 模式仍会把 `F` 帧保存到 `control.csv`。当前反馈能观察外环
P/I/D 和最终电机命令；目标杆角、实际杆角、角度误差及位置更新时间可直接
在 Keil Watch 中查看 `balance_control_state`。
