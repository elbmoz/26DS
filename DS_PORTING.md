# 电赛小车迁移说明

## 当前运行结构

`main.c` 初始化并运行以下模块：

1. `USART1`：3 个地址式步进电机驱动器。
2. `USART2`：JY61P 三轴 IMU（复用原 HWT101 协议解析器）。
3. `UART5`：保留的双误差视觉接口。
4. `USART6`：第二题位置误差/速度视觉接口，PC6/PC7。
5. `PE11、PE10、PE9、PE8、PE7、PA6、PA11、PA7`：从左到右 8 路数字红外。
6. `PB6、PB7`：题号选择键和确认键，低电平按下。
7. `PB8、PB9`：OLED 硬件 I2C1。
8. `TIM2`：1 ms 基础时钟。
9. `DS.c/.h`：统一硬件映射和数据入口。
10. `DS_task.c/.h`：选题、启动和任务状态机。
11. `LineFollow.c/.h`：第一题循迹控制。
12. `BalanceControl.c/.h`：第二题平衡架位置 PID。

主循环持续调用 `DS_Run()` 和 `DS_Task_Run()`，不能在任务里加入长时间
阻塞延时，否则会降低按键扫描和循迹控制频率。

## 当前硬件映射

| 模块 | 映射 |
|---|---|
| 左轮步进电机 | USART1，总线地址 `0x01` |
| 右轮步进电机 | USART1，总线地址 `0x02`，软件方向反相 |
| 平衡架步进电机 | USART1，总线地址 `0x03` |
| JY61P（复用 HWT101 协议解析器） | USART2，9600 |
| 原双误差视觉接口（保留） | UART5，9600 |
| 第二题小球视觉 | USART6，PC6=TX、PC7=RX，115200 |
| 红外 1~8（从左到右） | PE11、PE10、PE9、PE8、PE7、PA6、PA11、PA7 |
| 红外有效电平 | 默认低电平有效 |
| 按键 1（选题） | PB6，低电平按下，内部上拉 |
| 按键 2（确认/启动/停止） | PB7，低电平按下，内部上拉 |
| OLED | I2C1，PB8=SCL、PB9=SDA，AF4 开漏复用，100 kHz |

电机地址、方向和红外引脚集中在 `Core/Inc/DS.h`。按键映射在
`Core/Inc/button.h`，OLED 外设映射在 `Core/Src/i2c.c` 和
`Core/Src/stm32f4xx_hal_msp.c`。接线改变时应同步修改相关源文件和 `gc.ioc`。

PC6/PC7 已重新配置为 USART6，但只复用串口硬件，不恢复原舵机协议和舵机
任务。按键已迁移到 PB6/PB7。

## DS 接口

- `DS_ChassisSetSpeed(left, right, slope)`：两轮差速速度控制，正值表示小车
  前进方向。
- `DS_ChassisStop()`：左右轮分别立即停止，命令间隔 2 ms，并补发两轮以避免
  一圈结束时单个停止帧丢失后仍有一个车轮持续转动。
- `DS_BalanceSetSpeed(speed, slope)`：平衡架步进电机速度模式。
- `DS_BalanceMoveRelative(pulses, speed, acceleration)`：平衡架相对位置模式。
- `DS_BallVisionIsFresh()`：检查第二题位置量在最近 200 ms 内是否有效。
- `DS_IR_ReadRaw()`：8 路实际 GPIO 电平，bit0 对应红外 1。
- `DS_IR_ReadActive()`：按有效电平转换后的检测结果。
- `DS_GetState()`：统一读取红外、视觉和陀螺仪数据。

UART5 保留原双误差协议：

```text
x_error,y_error\n
none\n
```

有效视觉数据超过 200 ms 未更新时，`vision_valid` 会自动清零。

USART6 第二题在按键确认后由 STM32 发送 `c2`，视觉随后以 50 Hz 发送：

```text
B,-27,18\n
none\n
```

两个数分别是中心位置误差（参考像素）和轴向速度（参考像素/秒），视觉
右侧均为正。480 宽检测通道输出前乘 `CONTROL_OUTPUT_SCALE`，保持原 640 宽
标定。每次平衡电机速度/停止命令后，STM32 通过同一全双工 USART6 异步回传：

```text
F,seq,mcu_ms,vision_frame,vision_age_ms,position_x10,velocity_x10,
  error_x10,p_x100,i_x100,d_x100,motor_command,motor_status\n
```

反馈使用中断发送；若上一帧尚未发送完成则丢弃本帧并让 `seq` 留下缺口，
不阻塞控制。手动停止时 STM32 等待最后一帧最多 20 ms，然后发送 `ok`，
视觉停止输出。

## DS_task 操作

- `DS_Init()` 完成后、进入菜单前，当前 `main.c` 会向 3 号电机发送一次
  `+300` 脉冲、速度 50、加速度 10 的相对位置命令。它不是回零，复位会
  再次执行；上车前必须确认机械行程。
- 上电进入选题菜单，题号初始显示为 0。
- 每按一次按键 1，题号按 `1 → 2 → ... → 9 → 1` 循环。
- 选择题号 1 后按按键 2，开始循迹并从 0 计时。
- 第一题运行期间再次按按键 2，会立即停车并保留最终时间。
- JY61P 解绕累计偏航角达到一圈阈值后，会自动停车并保留最终时间。
- 选择题号 2 后按按键 2，启动 USART6 视觉并把小球目标设为中心 `0`。
- 第二题当前只持续调节到中心，不自动结束；再次按按键 2 会立即停止 3 号
  电机并发送 `ok`。当前参数以 `BalanceControl.c` 中用户实车调试值为准，
  后续再接入 `+5 cm -> -5 cm` 分段状态机。
- 题目 3~9 暂时显示 `NOT READY`；按按键 2 返回选题菜单。

OLED 菜单和第一题页面的第 4 行显示 HWT101 数据：`Y:` 为偏航角，`Z:`
为 Z 轴角速度，末位 `V` 表示数据有效，`X` 表示超过 200 ms 未收到新数据。
第二题页面显示接收误差 `E`、接收速度 `V`、电机命令 `OUT`、连续稳定周期
`ST`、接收有效标记 `RX:V/X`、稳定标记 `S/R` 和运行时间。
自动停车默认使用 350° 累计净偏航角、3 秒最短运行时间和 3 帧确认；按键 2
仍可随时手动结束。相关参数位于 `line_follow_config`。

## OLED 移植结果

拖入的 OLED 驱动原本是 STM32F103 标准外设库软件模拟版本，现已改成
STM32F407 HAL 硬件 I2C1，并加入 Keil 工程。默认使用常见 SSD1306 地址 `0x3C`
（写地址字节 `0x78`）。

I2C1 使用 PB8/PB9 的 AF4 开漏复用和 HAL 阻塞发送，速率为 100 kHz。初始化
通过硬件 ACK 自动探测 `0x3C` 和 `0x3D`。
可在 Keil Watch 中直接观察 `ds_task.oled_ready`：1 表示收到 ACK，0 表示
初始化通信失败。`ds_task.oled_address` 显示识别到的 7 位地址（`0x3C` 或
`0x3D`）。上电成功通信后会先全屏点亮 1 秒，再显示任务菜单。

若屏幕仍不亮，应依次检查：

1. VCC/GND、PB8/PB9 接线和共地。
2. 模块是否确实为 4 针 I2C SSD1306，而不是 SPI 或 SH1106 特殊版本。
3. 在 Watch 中检查 `ds_task.oled_ready`：0 表示两个地址都没有成功完成通信。
4. SCL/SDA 是否有外部上拉。代码启用了内部上拉，但长线或高速边沿建议使用
   约 4.7 kΩ 外部上拉。

## 模块处理结果

| 处理 | 模块 | 原因 |
|---|---|---|
| 复用并修正 | `zhangdatou` | 底层步进协议可复用；已修正 13 字节位置命令和超时 |
| 复用并简化 | `HWT101` | 陀螺仪仍需要；已改为完整 11 字节校验 |
| 复用并解耦 | `serial` | 保留视觉逐行接收，输出改接到 DS 状态 |
| 复用并移植 | `OLED` | 从 F103 标准库软件模拟改为 F407 HAL 硬件 I2C1 |
| 重写并启用 | `button` | 改为 PB6/PB7 双按键、20 ms 非阻塞消抖 |
| 新增 | `DS_task`、`LineFollow` | 选题/发车状态机和第一题循迹 |
| 新增 | `BallVision`、`BalanceControl` | USART6 误差/速度接收和第二题中心回正 PID |
| 复用 | `PID` | 位置用于 P/I，MaixCAM 滤波速度直接作为 D 反馈 |
| 暂时保留但不初始化 | `WS2812`、`screen`、`encoder_f407` | 可能用于状态提示、调试或里程反馈 |
| 删除 | `GC_task`、`action` | 原物流赛任务流程 |
| 删除 | `servo`、`LobotServoController`、`huaner_servo`、`ServoMotorControl` | 新车无舵机 |
| 删除 | `QRcode`、`laser`、`bluetooth` | 新需求不使用，且二维码/蓝牙会与视觉争用 UART5 |
| 删除并替换 | `Motor_Move`、`GC_Chassis_Control` | 四轮麦克纳姆和 Z 轴专用，已由 DS 两轮/平衡架接口替换 |

## 上车前必须确认

1. 架空底盘，确认 `DS_ChassisSetSpeed(200, 200, ...)` 时两轮都让小车向前。
2. 用黑线逐个扫过红外 1~8，确认 OLED 的 `IR:` 从左到右对应且黑线显示 1。
3. 确认按键未按时 PB6/PB7 为高电平，按下为低电平。
4. 先用较低速度完成循迹方向检查，再逐步提高速度和 P 参数。
5. 确认 JY61P 安装方向以及偏航角正负号。
6. 确认平衡架是否有零位/限位开关；若有，需要增加回零和越界保护。
7. 架空或托住平衡架，先发送一个固定正误差。当前已按“电机正转使球向
   视觉左侧运动”设置 `motor_direction=-1`；若实机恰好相反才改为 `+1`，
   不要同时改视觉符号。

循迹算法和调参顺序见 `LINE_FOLLOW_TUNING.md`；第二题见
`BALANCE_CONTROL_TUNING.md`。
