# 第一题循迹算法与调参

## 当前算法

8 路红外从左到右赋予位置权重：

```text
IR1   IR2   IR3   IR4   IR5   IR6   IR7   IR8
-3500 -2500 -1500 -500  +500 +1500 +2500 +3500
```

某一路检测到黑线时参与计算。若同时有多路检测到黑线，横向误差为所有有效
权重的平均值：

```text
error = 有效传感器权重之和 / 有效传感器数量
```

因此线位于小车中心时 `error` 接近 0；线偏右时 `error > 0`。P 控制计算：

```text
target_correction = kp × error
left_speed  = running_speed + correction
right_speed = running_speed - correction
```

为提高平稳性，还做了三项处理：

1. `max_correction` 限制最大转向量，防止突发大差速。
2. `curve_slowdown` 随弯道转向量增大而降低基础速度。
3. `correction_slew_per_update` 限制每个 20 ms 周期内转向量的变化，减少抖动。

当 8 路都检测不到线时，程序依据最后一次看到线的方向，以较低速度和固定
差速搜索。这是第一版的丢线恢复策略。若赛道存在十字、宽线或特殊标志，应在
拿到实际赛道规则后单独增加状态识别。

## 可调参数

参数均位于 `Core/Src/LineFollow.c` 的全局变量 `line_follow_config` 中，也可在
Keil Watch 窗口观察和修改。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `base_speed` | 400 | 直线基础速度；驱动协议中 10 约等于 1 RPM |
| `kp` | 0.08 | 误差到左右轮差速的比例系数 |
| `max_correction` | 300 | 最大转向修正量 |
| `curve_slowdown` | 120 | 最大弯道时最多降低的基础速度 |
| `correction_slew_per_update` | 20 | 每 20 ms 最大允许改变的转向量 |
| `control_period_ms` | 20 | 控制周期，20 ms 即 50 Hz |
| `command_limit` | 1000 | 第一题输出的软件安全限幅 |
| `lost_speed` | 200 | 丢线搜索时的前进基础速度 |
| `lost_correction` | 260 | 丢线搜索时的固定转向量 |
| `motor_slope` | 10 | 步进驱动器自身加减速参数 |

`line_follow_state` 可用于 Watch 调试：

- `sensor_bits`：8 路红外有效状态。
- `error`：加权横向误差。
- `correction`：经过变化限幅后的实际转向量。
- `left_command` / `right_command`：最终左右轮指令。
- `line_lost`：是否完全丢线。
- `last_motor_status`：最近一次电机发送状态。

## 推荐调参顺序

1. 先架空底盘，把 `base_speed` 改为 200 左右，确认左右轮前进方向正确。
2. 不启动电机，用黑线依次扫过 8 路传感器，确认 OLED 显示顺序和有效电平。
3. 在直线赛道低速运行，从 `kp=0.04` 左右逐步增加，直到能可靠回到线中心。
4. 如果左右来回摆动，先减小 `kp`；若仍有快速抖动，可减小
   `correction_slew_per_update`。
5. 如果入弯太慢，减小 `curve_slowdown`；如果高速冲出弯道，则增大它或降低
   `base_speed`。
6. 确认正常循迹后再调 `lost_speed` 和 `lost_correction`，并专门测试脱线恢复。
7. 最后逐步提高 `base_speed`。每提高一档速度，都需要重新检查弯道和丢线。

## 后续升级接口

当前结构已经把任务状态机和循迹控制分开。后续可以只修改
`LineFollow_Update()`，依次增加：

1. 对 `error` 做一阶低通或滑动平均，抑制传感器跳变。
2. 增加 D 项形成 PD 控制，提前抑制快速偏移；数字红外通常不急于加入 I 项。
3. 直线段锁定 HWT101 目标航向，把陀螺仪航向误差叠加到 `correction`。
4. 增加十字、直角、终点线和全黑/全白等赛道特征状态机。

建议先用实际车把 P 控制和机械方向调通，再逐项增加滤波或陀螺仪闭环，便于
判断每一步的收益和问题来源。
