# 第二题小球中心回正与 PID 调试

## 当前阶段

题目 2 目前只实现“把小球稳定到摆杆中心点 O”：

1. 菜单选择题号 2，按确认键 PB7。
2. 单片机通过 USART6 发送 `c2`，视觉开始逐行发送小球位置。
3. `BalanceControl_Start(0.0f)` 将目标设为中心 0。
4. 再按一次 PB7，停止 3 号平衡架电机并向视觉发送 `ok`。

当前不会因到达中心而自动结束，因为调试阶段需要持续观察能否稳定。后续
`+5 cm -> -5 cm` 流程可以直接调用：

```c
BalanceControl_SetTarget(balance_control_config.positive_5cm_target);
BalanceControl_SetTarget(balance_control_config.negative_5cm_target);
```

## 视觉协议

USART6 使用 PC6=TX、PC7=RX，9600、8N1。视觉每帧发送一个有符号位置并以
换行结束：

```text
-3.2\n
0\n
+4.8\r\n
none\n
```

推荐位置单位直接使用厘米，中心为 0，一侧为正、另一侧为负。连续 200 ms
没有有效数据，或者收到 `none`，控制器会停电机并清空 PID 历史，防止丢失
小球后继续倾斜摆杆。

## 控制算法

每 20 ms 计算一次位置式 PID：

```text
error = target_position - ball_position
output = Kp * error
       + Ki * sum(error)
       + Kd * (error - previous_error)
motor_command = motor_direction * output
```

输出经过积分限幅、总输出限幅和中心死区处理，然后作为 USART1 地址 `0x03`
的步进电机速度命令。这里的 D 是相邻两帧之差，没有再除以 0.02 s，因此
调节周期改变后需要重新调整 `Kd`。

稳定判据默认是误差绝对值连续 25 个控制周期不超过 1.0，即连续约 0.5 s
位于中心 ±1 cm 内。`stable` 只作为调试和后续分段状态机的判据，不会在
当前阶段自动停机。

## 可调参数

参数都集中在 `Core/Src/BalanceControl.c` 的
`balance_control_config`，也可在 Keil Watch 中直接修改：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `kp` | 8.0 | 位置误差的主要回正力度 |
| `ki` | 0.0 | 当前关闭；存在长期稳态偏差时再小幅增加 |
| `kd` | 0.0 | 当前关闭；完成 P 调节后再用于抑制振荡 |
| `pid_deadband` | 0.5 | 中心 ±0.5 单位内命令归零 |
| `integral_limit` | 100 | 积分累计限幅 |
| `output_limit` | 300 | 电机速度命令最大绝对值 |
| `control_period_ms` | 20 | PID 周期 |
| `motor_slope` | 10 | 步进驱动器加减速斜率 |
| `motor_direction` | 1 | 机构方向，方向相反时只改为 `-1` |
| `stable_error` | 1.0 | 稳定允许误差，按厘米输入时对应 ±1 cm |
| `stable_cycles` | 25 | 连续满足稳定误差的周期数 |

以上数值是安全起步值，不是最终竞赛参数；实际数值取决于视觉单位、帧率、
摆杆长度、传动比和电机细分。

## 推荐调试顺序

1. 先让视觉固定发送几个已知数值，例如 `0`、`+5`、`-5`，确认
   `ball_vision_parse_error_count` 不增加，且
   `balance_control_state.ball_position` 符号和数值正确。
2. 托住小球或架空机构，制造一个正误差。若电机动作让误差继续增大，只把
   `motor_direction` 从 `1` 改为 `-1`；不要同时反转视觉符号。
3. 暂时将 `ki=0`、`kd=0`，从较小 `kp` 开始，逐步增加到小球能明显回中心、
   但刚出现往复振荡的位置，再略微降低。
4. 从小 `kd` 开始增加，让越过中心的幅度和次数下降。视觉噪声明显时先做
   位置滤波或降低 `kd`，不要用很大的 D 硬压。
5. 只有仍存在固定方向的稳态偏差时才逐步加入很小的 `ki`。一旦出现缓慢
   来回摆动，优先减小 `ki`。
6. 最后在 `output_limit` 内逐步提高速度，验证从 +5 cm 和 -5 cm 两侧都能
   回到中心，并检查视觉断流时电机是否在 200 ms 左右停止。

实车第一次闭环前必须有人能立即按 PB7 停机。当前硬件映射没有提供摆杆
角度、零位或限位反馈，因此先限制机械行程和输出速度；若机构存在硬限位，
后续应增加回零与越界保护后再进行高速调参。
