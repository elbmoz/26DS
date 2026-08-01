# 管道平衡高速自动参数实验系统

本系统的目标不是让 Codex 人工看完一轮日志再改一次常量，而是把一次参数候选的
完整闭环压缩为几秒钟：

```text
联合采样参数 -> 写入 STM32 RAM -> MCU 执行组合实验 -> 50 Hz 流式评分
             -> 差候选提前停车 -> 下一候选
```

Codex 负责设计控制结构、评分方法和搜索边界，并在一批实验结束后判断是否需要
改变算法；候选之间的选择由本地优化器完成，单轮不再等待模型回复、重新编译或
烧录。

## 1. 设计初衷

旧版自动流程虽然已经支持 RAM 参数和阶跃测试，但每个候选都按以下方式运行：

```text
固定回中 5~8 s -> 正阶跃 2.5 s -> 固定回中 5~8 s -> 负阶跃 2.5 s
```

每轮坐标搜索约有 8 个候选，因此一轮通常需要约 3 分钟。已有六份内环实验日志
又显示，坐标搜索每轮只能沿一个坐标缓慢把 `inner_kp` 从 2.5 推向 9~14；不同
方向和不同轮次的分数波动明显，说明机械方向差异和实验噪声不能忽略。

高速版围绕四个原则设计：

1. **一次命令完成一个候选。** STM32 自己执行回零、正阶跃、负阶跃和最终回零，
   Windows/UDP/UART 延迟不进入相位计时。
2. **达到信息条件就切相位。** 误差和速度连续稳定 200 ms 后立即切换，不固定
   等满 5~8 秒；配置的相位时间只作为上限。
3. **联合搜索相关参数。** 基线和 Sobol 点建立覆盖后，使用 Optuna 多变量 TPE
   同时学习 `Kp/Kd` 等参数关系，不再一次只改一个参数。
4. **历史只作为候选，不直接当真值。** 远端已有 JSON 日志中的优胜参数会作为
   新实验的前几个种子重新实测，因此能利用过去数据，同时避免把偶然低分直接
   固化进源码。

## 2. 数据与控制链路

```text
iteration_client.py
  -> Windows localhost HTTP/SSE
  -> Windows UDP pid_request
  -> MaixCAM USART6
  -> STM32 20 ms 控制边界原子应用 RAM 参数
  -> STM32 自主执行 profile
  -> PA ACK + F3 50 Hz 遥测
  -> MaixCAM UDP
  -> Windows 流式剪枝、评分和下一组参数
```

所有可调参数只写 STM32 RAM：

- 不写 STM32 Flash；
- 不写步进驱动器 Flash；
- 断电、复位或执行 `pid-reset` 后恢复固件源码默认值；
- 传动比、水平零点、方向、电机地址和实际杆角保护值不属于在线搜索参数。

## 3. STM32 协议

原命令继续兼容：

```text
PG,seq                                      读取参数
PS,seq,mask,...                             原子设置参数
PR,seq                                      恢复源码默认参数
PT,seq,I|O,target,duration_ms               单方向兼容阶跃
PX,seq                                      停车
```

高速组合实验新增：

```text
PP,seq,I|O,amplitude,phase_ms,settle_band,settle_rate,settle_ms
```

例如内环 2° 组合实验：

```text
PP,17,I,2,1600,0.12,1.5,200
```

STM32 按顺序运行：

| phase | 动作 | 目的 |
|---:|---|---|
| 0 | 目标 0 | 只在候选开始时建立可比初态 |
| 1 | 目标 `+amplitude` | 测量正方向响应 |
| 2 | 目标 `-amplitude` | 用更大的跨向阶跃暴露阻尼和方向差异 |
| 3 | 目标 0 | 测量恢复能力，并为下一候选准备初态 |
| 4 | 完成并停车 | 正常结束 |
| 5 | 停车 | Windows剪枝或人工中止 |

每个活动相位满足以下条件并连续保持 `settle_ms` 后立即进入下一相位：

```text
abs(error) <= settle_band
abs(rate)  <= settle_rate
```

`phase_ms` 是每段最大时间，不是固定等待时间。

### F3 遥测

F3 保留 F2 的全部串级控制状态，并追加：

```text
tuning_sequence,tuning_phase,phase_elapsed_ms
```

`tuning_sequence` 与 PP/PA 的 MCU 序号一致，Windows 只接受当前序号的数据，避免
UDP缓存、上一次SSE连接或ACK期间产生的旧帧污染评分。F/F2解析仍保留，便于读取
旧固件和旧日志。

## 4. 在线参数

当前支持十项：

```text
outer_kp outer_ki outer_kd angle_limit
inner_kp inner_kd
speed_limit slew deadband min_speed
```

参数在下一次 20 ms 控制边界同时生效。Windows只有收到 STM32 的 `PA` ACK 才
认为设置成功，不能把“UDP已发送”或“MaixCAM已转发”当作参数已应用。

### 本次同步的参数基线

截至本分支同步到提交 `b6ae57ae334ecce0165f978d9247e5adbdca2a1c`，
`BalanceControl.c` 中会在上电、复位或 `pid-reset` 后恢复的默认值如下：

| 在线名称 | 固件字段 | 源码默认值 | 含义 |
|---|---|---:|---|
| `outer_kp` | `outer_kp_deg_per_px` | 0.026044 | 外环位置 P |
| `outer_ki` | `outer_ki_deg_per_px_s` | 0 | 外环位置 I |
| `outer_kd` | `outer_kd_deg_per_px_s` | 0.0046 | 外环速度 D |
| `angle_limit` | `outer_angle_limit_deg` | 10.5° | 外环目标杆角限幅 |
| `inner_kp` | `angle_kp_speed_per_deg` | 2.5 | 内环角度 P |
| `inner_kd` | `angle_kd_speed_per_deg_s` | 0 | 内环角速度 D |
| `speed_limit` | `motor_speed_limit` | 24 | 电机速度命令限幅 |
| `slew` | `motor_slew_per_update` | 150 / 20 ms | 每控制周期最大命令变化量 |
| `deadband` | `motor_speed_deadband` | 0.3 | 停车死区 |
| `min_speed` | `motor_min_speed` | 0.5 | 非零命令下限 |

远端同时包含六份旧版内环搜索报告。各轮优胜 `inner_kp` 依次为：

```text
2.5 -> 3.875 -> 6.00625 -> 9.3093 -> 14.42895 -> 9.37885
```

其余内环参数在这些优胜配置里均保持 `inner_kd=0`、`speed_limit=24`、
`slew=150`。这串结果说明高价值区域已经明显离开 2.5，但最后两轮又从 14.43
回到 9.38，不能证明某一个值是稳定全局最优。因此高速版会把这些配置去重后作为
种子重新测试，并以新的正/负/回零组合评分和重复实验中位数决定最终值；不会仅凭
旧日志覆盖源码默认参数。

## 5. 搜索与评分

### 默认快速阶段

默认只搜索最主要的两个自由度：

```text
inner: inner_kp, inner_kd
outer: outer_kp, outer_kd
```

原因是速度上限、slew、积分和角度限幅在基础环未收敛前会产生大量等价解，扩大
搜索空间反而减少每分钟获得的有效信息。基础增益稳定后，用 `--full` 再做完整
精调：

```text
inner --full: inner_kp inner_kd speed_limit slew
outer --full: outer_kp outer_ki outer_kd angle_limit
```

每个 study 默认执行：

1. 当前 STM32 参数作为第一个基线；
2. 读取 `Vision/runtime/pid_auto_*.json` 和 `pid_fast_*.json`，把历史优胜配置
   放入前几个候选并重新实测；
3. 用可复现的加扰 Sobol 点补足启动覆盖；
4. Optuna 多变量 TPE 联合提出后续候选；
5. 对搜索前两名各重复两次，以中位数选最终配置；
6. 最终配置写回 RAM，报告写入 `Vision/runtime/pid_fast_*.json`。

历史内环日志已经把有价值的 `inner_kp` 区域提示到约 9~14，因此高速搜索边界会
覆盖该区域，但不会未经复测直接修改 `BalanceControl.c` 的源码默认值。

### 评分

正、负方向分别计算：

- 全段 RMSE；
- 尾段 RMSE；
- 超调；
- 响应增益偏差；
- 尾段速度/角速度；
- 位置反馈无效率；
- 杆角保护占比；
- 命令/目标角饱和占比。

组合总分再加入：

- 正负方向分数差，抑制只适合一个机械方向的参数；
- 回零尾段误差与速度，确保下一候选有可比初态。

分数越低越好。报告保留每相位实际持续时间，能直接确认自适应切换节省了多少
时间。

### 流式提前剪枝

实验运行时逐帧检查：

- 控制器进入保护状态；
- 电机位置反馈连续丢失；
- 外环视觉数据超时；
- 正/负相位运行 500 ms 后误差扩大到初值的 1.6 倍。

命中任一条件立即发送 `PX` 并把 Optuna trial 标记为 `PRUNED`。这些判断主要是
性能剪枝：明显不可能获胜的候选不再占用剩余相位时间。

## 6. 安装与部署

```powershell
python -m pip install -r Vision\windows\requirements.txt
python -m unittest discover -s Vision\tests -v
```

1. 用 Keil 打开 `MDK-ARM/gc.uvprojx`，全量编译并烧录一次。
2. 部署并重启 MaixCAM：

```powershell
python Vision\windows\vision_agent.py restart --deploy
```

3. 启动上位机：

```powershell
Vision\windows\start_operator_console.cmd
```

4. 在车上进入问题2，使 STM32 已发出 `c2` 并保持控制任务运行。
5. 验证端到端 ACK：

```powershell
python Vision\windows\iteration_client.py pid-get
```

## 7. 推荐实验命令

### 单次预检

取下球，先验证内环组合实验：

```powershell
python Vision\windows\iteration_client.py pid-profile `
  --stage inner --target 2 --duration 1.6
```

放回球，验证外环：

```powershell
python Vision\windows\iteration_client.py pid-profile `
  --stage outer --target 50 --duration 1.6
```

### 最快基础搜索

```powershell
python Vision\windows\iteration_client.py pid-auto `
  --stage inner --trials 16 --duration 1.6 --target 2

python Vision\windows\iteration_client.py pid-auto `
  --stage outer --trials 16 --duration 1.6 --target 50
```

### 完整精调

只在基础搜索结果已经稳定后运行：

```powershell
python Vision\windows\iteration_client.py pid-auto `
  --stage inner --full --trials 24 --duration 1.8 --target 2

python Vision\windows\iteration_client.py pid-auto `
  --stage outer --full --trials 24 --duration 1.8 --target 50
```

可覆盖默认稳定判断：

```powershell
--settle-band 0.10 --settle-rate 1.2 --settle-ms 240   # inner
--settle-band 10   --settle-rate 25  --settle-ms 240   # outer
```

### 兼容旧坐标搜索

```powershell
python Vision\windows\iteration_client.py pid-auto `
  --stage inner --engine coordinate --rounds 1 --duration 2.5 --target 2
```

## 8. 时间预期

默认每相位上限 1.6 s，理论最坏单候选为 6.4 s；实际准备和回零相位通常会在
达到 200 ms 稳定条件后提前结束，发散候选则可能在 0.5~2 s 被剪枝。

典型预算：

```text
16个搜索候选               约 1~2 min
前2名 × 2次复测            约 15~30 s
整批基础搜索                通常少于 3 min
```

实际耗时取决于机械响应速度和视觉有效率，报告中的 `phase_durations_ms` 是最终
依据。

## 9. 结果处理

高速搜索结束后，最优配置已经留在 STM32 RAM。先用不同球初始位置复测，再决定
是否将结果人工写入 `Core/Src/BalanceControl.c`。不要自动写源码或Flash，因为：

- 实验目标和正式赛题轨迹可能不同；
- 机械装配、电压和球的状态会改变最优点；
- 保留固件默认值可随时断电恢复。

报告至少记录：初始配置、历史种子来源、搜索边界、所有完成/剪枝候选、正负方向
指标、复测中位数、最终ACK以及完整profile配置。

## 10. 分支和回退

远端功能分支为：

```text
origin/codex/automatic-pid-tuning
```

本地 worktree 使用跟踪分支 `automatic-pid-tuning`。RAM参数断电即可清除；代码
层面可切回 `main` 或对相应提交执行 `git revert`。
