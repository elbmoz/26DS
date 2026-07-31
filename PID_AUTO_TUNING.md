# 管道平衡全自动在线调参

这套链路只修改 STM32 RAM。断电、复位或执行 `pid-reset` 后恢复当前固件里的源码默认值，不写电机驱动器或 STM32 Flash。

## 控制链路

```text
iteration_client.py
  -> Windows localhost API
  -> UDP pid_request
  -> MaixCAM USART6
  -> STM32 20 ms 控制边界应用 RAM 参数
  -> PA ACK + F2 完整遥测
  -> MaixCAM UDP
  -> Windows 自动评分和下一组参数
```

STM32 支持：

- `PG,seq`：读取完整参数。
- `PS,seq,mask,...`：只修改掩码指定的参数。
- `PR,seq`：恢复本次固件启动时的源码默认参数。
- `PT,seq,I|O,target,duration_ms`：运行内环角度或外环位置阶跃。
- `PX,seq`：结束测试并停车。
- `PA,...`：参数在控制边界实际生效后才发送的确认。
- `F2,...`：50 Hz 完整串级控制遥测。

可在线修改的九项参数是：

```text
outer_kp outer_kd angle_limit
inner_kp inner_kd
speed_limit slew deadband min_speed
```

传动比、水平零点、方向、实际角度保护限值和电机地址仍然只允许离线标定后写入源码。

## 首次部署

```powershell
python -m unittest discover -s Vision\tests -v
```

1. 用 Keil 打开 `MDK-ARM/gc.uvprojx`，全量编译并烧录一次。
2. 部署并重启 MaixCAM：

```powershell
python Vision\windows\vision_agent.py restart --deploy
```

3. 启动 Windows 上位机：

```powershell
Vision\windows\start_operator_console.cmd
```

4. 在车上进入问题 2，使 STM32 已经发出 `c2` 并保持控制任务运行。
5. 验证端到端 ACK：

```powershell
python Vision\windows\iteration_client.py pid-get
```

## 在线命令

```powershell
python Vision\windows\iteration_client.py pid-set inner_kp=3.0 inner_kd=0.1 speed_limit=30 slew=5
python Vision\windows\iteration_client.py pid-test mode=inner_step target_angle=2 duration=3
python Vision\windows\iteration_client.py pid-stop
python Vision\windows\iteration_client.py pid-reset
```

参数设置成功的含义是：STM32 已在下一个 20 ms 控制边界应用，而不是 Windows 或 MaixCAM 仅仅收到了请求。

## 全自动搜索

先调内环。把管道人工调平并取下球，然后运行：

```powershell
python Vision\windows\iteration_client.py pid-auto --stage inner --rounds 1 --duration 2.5 --target 2
```

程序会自动完成：读取当前参数、正负角度阶跃、计算角度 RMSE/尾段误差/超调/响应增益/角速度/限幅率、逐参数坐标搜索、恢复最佳组合、停车并写出 JSON 报告。

内环完成后放回球，再调外环：

```powershell
python Vision\windows\iteration_client.py pid-auto --stage outer --rounds 1 --duration 3 --target 50
```

外环评分使用球位置 RMSE、尾段误差、尾段速度、目标角限幅率和反馈有效率。每轮固定测试正负两个目标，避免只对单方向机械特性过拟合。

报告保存在 `Vision/runtime/pid_auto_<stage>_<time>.json`。最佳参数仍只在 RAM 中；确认多轮结果一致后，再人工写回 `Core/Src/BalanceControl.c`。

## Git 回退

本功能位于分支 `codex/automatic-pid-tuning`。提交前后的回退都不依赖固件参数：断电即可清除 RAM 参数，Git 则可切回 `main` 或对功能提交执行 `git revert`。
