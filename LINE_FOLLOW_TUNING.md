# 第一题循迹算法与调参

## 当前算法

8 路红外从左到右赋予位置权重：

```text
IR1   IR2   IR3   IR4   IR5   IR6   IR7   IR8
+22000 +13000 +1600 +500 -500 -1600 -13000 -22000
```

某一路检测到黑线时参与计算。若同时有多路检测到黑线，横向误差为所有有效
权重的平均值：

```text
error = 有效传感器权重之和 / 有效传感器数量
```

因此线位于小车中心时 `error` 接近 0。当前权重正负号已经按照实车转向方向
调整。控制器使用离散 PD：

```text
error_delta = error - previous_error
d_correction = limit(kd × error_delta)
target_correction = kp × error + d_correction
left_speed  = running_speed + correction
right_speed = running_speed - correction
```

D 项根据相邻两个 20 ms 控制周期的误差变化提前加强或减弱转向，并限制在
`±max_d_correction` 内。首次看到线以及丢线后的首次重获线不计算 D，避免
误差历史不连续造成尖峰。

为提高平稳性，还做了三项处理：

1. `max_correction` 限制最大转向量，防止突发大差速。
2. `curve_slowdown` 随弯道转向量增大而降低基础速度。
3. `correction_slew_per_update` 可限制每个 20 ms 周期内转向量的变化；当前设为
   0，表示关闭软件渐变，直接采用本周期计算出的目标修正量。

当前 `lost_speed` 和 `lost_correction` 均为 0，因此 8 路都检测不到线时车辆
停止。若后续启用丢线搜索，再逐步设置这两个参数。

## 一圈自动停车

第一题开始时记录当前 HWT101 偏航角。每个新的 `0x53` 角度帧计算：

```text
yaw_delta = normalize(current_yaw - previous_yaw, -180° ... +180°)
accumulated_yaw += yaw_delta
```

因此陀螺仪读数从 `+179°` 跳到 `-179°` 时会被识别为约 `+2°`，不会误认为
突然反转 358°。累计净偏航角绝对值达到阈值后，`DS_task` 调用统一停车函数，
停止左右轮并冻结计时。

偏航角每帧跳变超过 `lap_max_yaw_step_deg` 会被拒绝；角度数据超过 200 ms
没有更新时暂停累计，不会因 IMU 断线误停。按键 2 始终保留手动停车功能。

## 可调参数

参数均位于 `Core/Src/LineFollow.c` 的全局变量 `line_follow_config` 中，也可在
Keil Watch 窗口观察和修改。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `base_speed` | 100 | 直线基础速度；驱动协议中 10 约等于 1 RPM |
| `kp` | 0.05 | 当前误差的比例系数 |
| `kd` | 0.00 | 相邻周期误差变化量的系数，当前关闭 |
| `max_correction` | 300 | 最大转向修正量 |
| `max_d_correction` | 80 | D 项对转向修正的最大贡献 |
| `curve_slowdown` | 120 | 最大弯道时最多降低的基础速度 |
| `correction_slew_per_update` | 0 | 0 表示直接改变；大于 0 时限制每 20 ms 的变化量 |
| `control_period_ms` | 20 | 控制周期，20 ms 即 50 Hz |
| `command_limit` | 1000 | 第一题输出的软件安全限幅 |
| `lost_speed` | 0 | 丢线时基础速度，当前停车 |
| `lost_correction` | 0 | 丢线时转向量，当前停车 |
| `motor_slope` | 10 | 步进驱动器自身加减速参数 |
| `lap_stop_enabled` | 1 | 是否启用陀螺仪一圈自动停车 |
| `lap_target_yaw_deg` | 350° | 自动停车的累计净偏航角 |
| `lap_max_yaw_step_deg` | 45° | 单次角度帧允许的最大跳变 |
| `lap_min_time_ms` | 3000 ms | 启动后允许自动停车的最短时间 |
| `lap_confirm_frames` | 3 | 超过一圈阈值后的确认帧数 |

`line_follow_state` 可用于 Watch 调试：

- `sensor_bits`：8 路红外有效状态。
- `error`：加权横向误差。
- `error_delta`：本周期误差相对上一周期的变化量。
- `d_correction`：经过限幅的 D 项修正量。
- `correction`：经过变化限幅后的实际转向量。
- `left_command` / `right_command`：最终左右轮指令。
- `line_lost`：是否完全丢线。
- `yaw_valid`：HWT101 偏航角是否在 200 ms 内更新。
- `yaw_delta_deg`：已处理 ±180° 回绕的本帧角度增量。
- `accumulated_yaw_deg`：从第一题启动开始累计的净偏航角。
- `lap_complete`：是否已经达到一圈停车条件。
- `rejected_yaw_steps`：被异常跳变过滤掉的角度帧数量。
- `last_motor_status`：最近一次电机发送状态。

## 推荐调参顺序

1. 先架空底盘，把 `base_speed` 改为 200 左右，确认左右轮前进方向正确。
2. 不启动电机，用黑线依次扫过 8 路传感器，确认 OLED 显示顺序和有效电平。
3. 先将 `kd=0`，在直线赛道低速运行，调整 `kp` 直到能可靠回到线中心。
4. 恢复 `kd=0.02`；入弯响应仍慢可逐步增加到 0.03～0.05，传感器切换时
   冲击明显则减小 `kd` 或 `max_d_correction`。
5. 如果左右来回摆动，先减小 `kp`；需要重新启用平滑时，再把
   `correction_slew_per_update` 设置为 20～60。
6. 如果入弯太慢，减小 `curve_slowdown`；如果高速冲出弯道，则增大它或降低
   `base_speed`。
7. 确认正常循迹后再调 `lost_speed` 和 `lost_correction`，并专门测试脱线恢复。
8. 最后逐步提高 `base_speed`。每提高一档速度，都需要重新检查弯道和丢线。

## 后续升级接口

当前结构已经把任务状态机和循迹控制分开。后续可以只修改
`LineFollow_Update()`，依次增加：

1. 对 `error` 做一阶低通或滑动平均，抑制传感器跳变。
2. 对 D 项做低通滤波，进一步抑制数字传感器切换造成的跳变。
3. 直线段锁定 HWT101 目标航向，把陀螺仪航向误差叠加到 `correction`。
4. 增加十字、直角、终点线和全黑/全白等赛道特征状态机。

建议先用实际车把 P 控制和机械方向调通，再逐项增加滤波或陀螺仪闭环，便于
判断每一步的收益和问题来源。
