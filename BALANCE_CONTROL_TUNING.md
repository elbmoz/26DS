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

题目 2 使用 `motor_position_period_ms` 指定位置读取周期；题目 9 继续使用
`MOTOR_POSITION_MONITOR_PERIOD_MS=100 ms` 的诊断周期。

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

> `capture_motor_zero_on_start=1` 假定进入题目 2 时杆已经物理调平。它只是
> 临时调试方案，不是机械回零。完成零点标定后应写入
> `motor_zero_angle_deg` 并把自动捕获关闭。

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

当前已知关系是“正电机命令让球向视觉左侧运动”，因此默认：

```text
tilt_direction  = -1
motor_direction = +1
```

球位于右侧时，位置误差为负，外环输出为负，经 `tilt_direction=-1`
得到正目标杆角，内环发出正电机命令。若电机正命令与位置读数的增量方向不
一致，修改 `motor_direction`；若杆角正方向与球滚动关系相反，修改
`tilt_direction` 或带符号的 `rod_angle_per_motor_degree`。不要同时修改
多个方向参数。

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
| `outer_ki_deg_per_px_s` | 0.03297 | 位置积分，deg/(px·s) |
| `outer_kd_deg_per_px_s` | 0.18681 | 视觉速度 D，deg/(px/s) |
| `outer_integral_limit_px_s` | 109.2 | 积分限幅，px·s |
| `outer_angle_limit_deg` | 7.2 | 外环最大目标杆角 |
| `hold_band_px` | 18.2 | 约等于离目标 1 cm |
| `fine_band_px` | 3.64 | 约等于离目标 2 mm |
| `fine_velocity_px_s` | 18.2 | 精细区回水平速度阈值 |
| `soft_ki_deg_per_px_s` | 0.10989 | 近中心专用积分 |
| `damping_velocity_px_s` | 50.96 | 开始高速阻尼 |
| `freeze_integral_velocity_px_s` | 81.90 | 冻结积分速度 |

这些数值由参考工程的 mm/mm/s 参数按 `18.2 px/cm` 换算而来，并不代表
已经适配当前机械结构和 MaixCAM 速度噪声。

### 位置、角度和执行器

| 参数 | 当前默认值 | 作用 |
|---|---:|---|
| `motor_zero_angle_deg` | 0 | 标定后的电机水平零点 |
| `rod_angle_per_motor_degree` | 1.0 | 电机角到实际杆角的带符号比例 |
| `rod_angle_limit_deg` | 10.0 | 杆角软件保护范围 |
| `capture_motor_zero_on_start` | 1 | 首个有效位置作为临时零点 |
| `angle_kp_speed_per_deg` | 9.2 | 角度内环 P |
| `motor_speed_limit` | 180 | RS485 最大速度命令，约 18 RPM |
| `motor_speed_deadband` | 1 | 小于等于该值视为停止 |
| `motor_min_speed` | 4 | 非零时的最小命令 |
| `motor_slew_per_update` | 16 | 每个控制周期最大速度变化 |
| `motor_slope` | 0 | 电机驱动器速度模式斜率字段 |
| `control_period_ms` | 20 ms | 串级控制周期 |
| `motor_position_period_ms` | 20 ms | 题目 2 的 0x36 查询周期 |
| `motor_position_timeout_ms` | 60 ms | 位置失效停机阈值 |

`angle_kp_speed_per_deg=9.2` 假设电机速度协议约 `10≈1 RPM` 且杆与电机
直接连接，用来近似参考工程的角度内环响应。存在减速机构时必须重新计算。

稳定判定使用 `stable_error_px=18.2`、`stable_velocity_px_s=25` 和
`stable_frames=25`。计数单位是新的视觉帧，50 Hz 时约需连续 0.5 秒。

## OLED 题目 2 页面

```text
Q2 RX:V P:V S:X
E:-027.0 V:+018
M:+016 A:+001.2
TIME:0000.0s
```

- `RX:V/X`：视觉有效/无效。
- `P:V/X`：电机位置反馈有效/无效。
- `S:V/X`：已经稳定/仍在调节。
- `E`、`V`：球的位置误差和轴向速度。
- `M`：最终有符号电机速度命令。
- `A`：相对水平零点的实际杆角。

## 首次标定和调试顺序

1. 题目 9 在独立可调的上、下端点之间循环采集，不执行自动标定，也不修改
   `balance_control_config`。记录 `DS_TASK_Q9_UPPER_PULSES`、
   `DS_TASK_Q9_LOWER_PULSES`、IMU 安装方向、杆初始姿态和机械连接方式。
2. 确认启动后，题目 9 会先等待初始电机位置稳定，再记录 IMU 相对零点；
   每次必须检测到位置变化并等待连续稳定位置更新后，才在端点停留并反向，
   不使用可能提前截断运动的固定反向周期。
3. 采集多个完整往复周期并保存 USART6 的 `Q9` 遥测；上传数据后，根据电机
   位置、三轴姿态、运动方向和时间序列离线分析水平零点、有效倾角轴、带符号
   传动比和回差。
4. 数据分析完成前不要根据单次观察写死题目 2 参数。分析后再填写
   `motor_zero_angle_deg`、`rod_angle_per_motor_degree`、
   `motor_direction` 及必要的控制限幅，然后不放球启动题目 2，确认
   `P:X` 很快变为 `P:V`，实际杆角 `A` 接近 0°。
5. 首次带球前把 `outer_angle_limit_deg` 临时降到约 1～2°、
   `motor_speed_limit` 降到约 20～40，并确保有人随时按 PB7。
6. 托住球制造小的正、负误差，分别确认目标杆角、实际杆角和电机命令方向。
   方向错误时一次只修改一个方向参数。
7. 先验证角度内环能快速追角且不振荡，再调球位置外环。
8. 首轮外环调试可先把两个 Ki 参数设为 0，只调 P 和 D；确认没有持续偏差后
   再恢复小积分。
9. 若球来回冲过中心，优先降低目标角限幅或 P，并检查视觉速度噪声，再调整
   D；若电机方向频繁跳变，降低 D 或增加速度滤波。
10. 最后验证视觉 `none` 时杆能回水平，拔掉位置反馈时电机能在 60 ms 左右
   停止，再逐步放开速度和角度限制。
11. 两侧中心控制可靠后，再实现 `+5 cm -> -5 cm` 状态机。

默认 stream 模式仍会把 `F` 帧保存到 `control.csv`。当前反馈能观察外环
P/I/D 和最终电机命令；目标杆角、实际杆角、角度误差及位置更新时间可直接
在 Keil Watch 中查看 `balance_control_state`。
