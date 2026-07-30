# 第二题小球中心回正与 PID 调试

## 当前流程

题目 2 当前只实现把小球稳定到摆杆中心点：

1. PB6 选择题号 2。
2. PB7 确认后，STM32 启动 PID、挂接 USART6 中断接收并发送 `c2`。
3. MaixCAM 收到 `c2` 后才以 50 Hz 发送位置误差和轴向速度。
4. 再按 PB7，3 号平衡架电机停止，STM32 发送 `ok`，MaixCAM 停止输出。

当前到达中心后不会自动结束，而是继续维持中心，方便观察稳定性。后续
`+5 cm -> -5 cm` 状态机可直接调用：

```c
BalanceControl_SetTarget(balance_control_config.positive_5cm_target);
BalanceControl_SetTarget(balance_control_config.negative_5cm_target);
```

当前标定约为 `18.2 px/cm`，所以两个预留目标为 `+91 px` 和 `-91 px`。

当前固件仍处于串口与 PID 数据链路联调阶段，`output_limit=0`，所以题目 2
只更新接收、PID 和 OLED 状态，不会发送平衡架速度命令。上电时 `main.c`
仍会单独发送一次 `+350` 脉冲、速度 50、加速度 10 的相对位置命令；该动作
不是回零，重复复位会重复执行，实车上电前必须确认机械行程。

## 串口与数据符号

USART6 使用 PC6=TX、PC7=RX，115200、8N1。有效帧为：

```text
B,<error_px>,<velocity_px_s>\n
```

例如：

```text
B,-27,18
```

表示小球位于中心左侧 27 个参考像素，同时以 18 px/s 向视觉右侧运动。
误差和速度均以视觉右侧为正。丢球或预测续航结束时发送 `none\n`。

MaixCAM 的检测通道是 480 宽，但发送前乘 `CONTROL_OUTPUT_SCALE=640/480`，
因此 STM32 始终使用原 640 宽参考像素标定。连续 200 ms 没有有效帧或收到
`none` 时，控制器立即停电机并清空 PID 历史。

## 控制方向与算法

已知机构关系：

```text
电机命令为正/顺时针 -> 小球向视觉左侧运动
视觉误差为正         -> 小球在中心右侧
```

通用 PID 内部误差为：

```text
control_error = target - vision_position
```

MaixCAM 已提供滤波后的轴向速度，因此不再对整数位置二次差分，而是使用：

```text
before_direction = Kp * control_error
                 + Ki * sum(control_error)
                 + Kd * (-vision_velocity)

motor_command = motor_direction * before_direction
motor_direction = -1
```

在中心目标 `target=0` 时等价于：

```text
motor_command = Kp * vision_error
              + Ki * sum(vision_error)
              + Kd * vision_velocity
```

所以球在右侧时输出正转，使球向左回中心；球正在快速向右运动时，速度 D 项
也会提前给出正转制动。该方向与用户提供的机构关系一致。

## 当前保守参数

参数集中在 `Core/Src/BalanceControl.c` 的
`balance_control_config`：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `kp` | 0.55 | 像素位置的主要回正力度 |
| `ki` | 0.001 | 很小的长期偏差补偿 |
| `kd` | 0.08 | 直接乘视觉速度，抑制冲过中心 |
| `pid_deadband` | 3 px | 中心位置死区 |
| `velocity_deadband` | 12 px/s | 小速度死区 |
| `integral_limit` | 3000 | 原始误差积分累计限幅 |
| `output_limit` | 0 | 当前禁用闭环速度输出；实车联调后再设置正限幅 |
| `control_period_ms` | 20 ms | 与视觉 50 Hz 对齐 |
| `motor_slope` | 0 | 当前速度命令斜率参数 |
| `motor_direction` | -1 | 匹配“正转让球向视觉左走” |
| `stable_error` | 18 px | 约等于题目要求的 ±1 cm |
| `stable_velocity` | 25 px/s | 稳定时还必须接近静止 |
| `stable_cycles` | 25 | 连续约 0.5 s 才标记稳定 |

这些是限制输出的首次闭环参数，不是最终竞赛参数。电机传动比、摆杆倾角
灵敏度和实际摩擦未知，因此第一次必须有人随时按 PB7 停止。

## OLED 题目 2 页面

```text
Q2 CENTER RX:V R
E:-027.0 V:+018
OUT:-000 ST:000
TIME:0000.0s
```

- `RX:V/X`：USART6 数据有效/无效。
- `E`：视觉发送的位置误差。
- `V`：视觉发送的轴向速度。
- `OUT`：实际发给 3 号电机的有符号速度命令。
- `ST`：连续满足稳定条件的周期数。
- 右上角 `S/R`：已经稳定/仍在调节。

## 推荐调试顺序

1. 先不放球或托住机构。进入题目 2 后确认 OLED 从 `RX:X` 变成 `RX:V`；
   手动移动球时 `E` 和 `V` 都应变化。
2. 球移到视觉右侧时 `E` 必须为正，移到左侧时必须为负。如果相反，应先
   修正视觉轴线方向，不要用 PID 参数掩盖坐标错误。
3. 托住球制造正误差，确认电机命令为正且实际正转让球向左。如果实际机械
   方向与描述相反，只把 `motor_direction` 改为 `+1`。
4. 确认接收和方向都正确后，停止任务并把 `output_limit` 从 0 改为不超过
   180 的正值，再重新启动题目 2。若电机完全带不动，先小幅增加 `kp`；
   若动作过猛，先降低 `output_limit`，再降低 `kp`。
5. 若能回中但持续往返，先增加 `kd`，每次约增加 `0.01～0.02`；若输出被
   速度噪声带着抖动，则降低 `kd` 或增大 `velocity_deadband`。
6. 只有存在固定方向的长期偏差时才增加 `ki`。出现慢周期摆动应首先减小
   `ki`。
7. 最后分别从约 `+91 px` 和 `-91 px` 放球，验证两侧都能在 5 秒内进入
   `|E|<=18 px` 且速度下降，再开始实现完整 `+5 cm -> -5 cm` 状态机。

当前硬件映射没有摆杆角度、零位或限位反馈。第一次闭环前应限制机械行程；
若机构存在硬限位，后续必须加入回零和越界保护。
