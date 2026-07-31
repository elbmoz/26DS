# 第四题位置模式平衡控制

## 控制结构

第四题保留第二题的球位置外环和增益调度，但删除软件实现的管角速度内环：

```text
MaixCAM 球位置/球速度
          |
          v
位置 PID 外环 -> 目标管角
          |
          v
a010e37 管角/位置标定
          |
          v
0xFD 绝对位置目标 -> 驱动器内部位置环
```

代码位于 `Core/Src/Task4PositionControl.c` 和
`Core/Inc/Task4PositionControl.h`。它拥有独立的配置、积分器、水平零点、
稳定计数和命令调度，不读取或修改第二题的 `balance_control_config/state`。

## 标定换算

任务 9 的 reviewed 稳态拟合为：

```text
rod_deg = -0.0020022658 * (motor_position - horizontal_position)
```

同一数据中，360 个 `0xFD` 位置命令脉冲对应约 7373 个 `0x36` 原始位置
计数；当前 3200 细分的理论换算也是：

```text
position_counts_per_command_pulse = 65536 / 3200 = 20.48
```

进入第四题时，第一次有效位置回包 `P0` 被捕获为本次物理水平锚点。标准
Emm_V5.0 完整回包共 8 字节：`地址 + 0x36 + 符号 + 4 字节位置 + 0x6B`；
若不计地址则为 7 字节。底层同时保留对 6 字节兼容格式的支持：

```text
target_count_offset = target_rod_deg / -0.0020022658
zero_command_pulse  = round(P0 / 20.48)
target_pulse        = zero_command_pulse
                    + round(target_count_offset / 20.48)
```

`target_pulse` 通过 `DS_BalanceMoveAbsolute()` 作为绝对位置目标发送。因此相同
目标可以安全重试，不会像连续发送相对位移那样累积漂移。旧采集数据中的
`P=5025.92` 只属于那次 IMU 相对零点，程序不会跨上电使用该截距。

## 参数

外环初值从当前第二题复制到 `task4_position_control_config`，复制后两者独立
调参。第四题不再包含以下第二题速度内环参数：

- `angle_kp_speed_per_deg`
- `motor_speed_deadband`
- `motor_min_speed`
- `motor_slew_per_update`
- `motor_slope` 和 `motor_direction`

位置模式仍需要两个驱动器执行参数：

- `motor_move_speed = 2`
- `motor_move_acceleration = 0`

`0xFD` 绝对位置命令按 `control_period_ms = 20 ms` 连续刷新，即使量化后的目标
脉冲未改变也会重发。按下确认后，`0x36` 先按
`motor_position_startup_period_ms = 5 ms` 快速取得水平锚点；锚点建立后自动降为
`motor_position_period_ms = 200 ms`，避免运行时高频查询抢占 USART1。已有查询
尚未结束时不会重叠发送新的查询。

标定公共目标角限幅保持 `6.0°`，实际管角保护限幅为 `6.5°`。修改映射斜率、
`20.48 count/pulse` 或角度保护范围前，必须先完成并审查新的离线标定；若改变
驱动器细分，必须同步更新 count/pulse 换算。

## 运行与保护

1. 启动前先让管道物理水平，PB6 选择题号 4，PB7 启动。
2. STM32 仍发送 `c2`，因为 MaixCAM 的球位置协议没有改变。
3. 未收到第一次有效 `0x36` 回包、尚未建立水平锚点前不会发送位置目标。
4. 有效视觉帧产生新目标角；视觉超过 200 ms 无效时目标回到本次水平位置。
5. 建立锚点后，低频位置查询只刷新 `A` 和诊断信息；查询短时失败不会中断
   每 20 ms 的位置目标。映射参数非法时仍立即停止 3 号电机。
6. PB6 可关闭/恢复周期 OLED 写入而不中断控制；PB7 停止位置读取和电机，
   并向 MaixCAM 发送 `ok`。

OLED 第三行的 `C` 是绝对目标脉冲，`A` 是根据 `0x36` 反馈拟合出的本次
零点相对管角。`P=V` 表示本次任务已经成功读取位置并建立水平锚点，不要求
最近 60 ms 内持续收到位置。USART6 的 13 字段 `F,...` 帧保持兼容；第四题的
`motor_command` 字段表示绝对目标脉冲，而不是第二题的速度命令。
