# 电赛小车迁移说明

## 当前运行结构

`main.c` 只初始化并运行以下必需模块：

1. `USART1`：3 个地址式步进电机驱动器
2. `USART2`：HWT101 陀螺仪
3. `UART5`：视觉模块
4. `PE11、PE10、PE9、PE8、PE7、PA6、PA11、PA7`：从左到右 8 路数字红外
5. `TIM2`：1 ms 基础时钟
6. `DS.c/.h`：统一硬件映射和数据入口

## 当前硬件映射

| 模块 | 临时映射 |
|---|---|
| 左轮步进电机 | USART1，总线地址 `0x01` |
| 右轮步进电机 | USART1，总线地址 `0x02`，逻辑方向反相 |
| 平衡架步进电机 | USART1，总线地址 `0x03` |
| HWT101 | USART2，115200 |
| 视觉模块 | UART5，9600 |
| 红外 1~8（从左到右） | PE11、PE10、PE9、PE8、PE7、PA6、PA11、PA7 |
| 红外有效电平 | 默认低电平有效 |

地址、方向和红外引脚都集中定义在 `Core/Inc/DS.h`。以后接线变化时只修改该文件和
`gc.ioc`，不要把硬件地址散落到任务代码中。注意 PA6 已分配给红外 6，现有
`button` 模块也曾使用 PA6，除非先重新分配按键引脚，否则不要调用 `Button_Init()`。

## DS 接口

- `DS_ChassisSetSpeed(left, right, slope)`：两轮差速速度控制，正值表示小车前进方向。
- `DS_ChassisStop()`：同步停止左右轮。
- `DS_BalanceSetSpeed(speed, slope)`：平衡架步进电机速度模式。
- `DS_BalanceMoveRelative(pulses, speed, acceleration)`：平衡架相对位置模式。
- `DS_IR_ReadRaw()`：8 路实际 GPIO 电平，bit0 对应红外 1。
- `DS_IR_ReadActive()`：按有效电平转换后的检测结果。
- `DS_GetState()`：统一读取红外、视觉和陀螺仪数据。

视觉接收暂时兼容原协议：

```text
x_error,y_error\n
none\n
```

有效视觉数据超过 200 ms 未更新时，`vision_valid` 会自动清零。

## 模块处理结果

| 处理 | 模块 | 原因 |
|---|---|---|
| 复用并修正 | `zhangdatou` | 底层步进协议可复用；已修正 13 字节位置命令和超时 |
| 复用并简化 | `HWT101` | 陀螺仪仍需要；已改为完整 11 字节校验 |
| 复用并解耦 | `serial` | 保留视觉逐行接收，输出改接到 DS 状态 |
| 保留 | `PID` | 后续循迹、航向和平衡架闭环可直接使用 |
| 暂时保留但不初始化 | `button`、`WS2812`、`screen`、`encoder_f407` | 可能用于发车、状态提示、调试或里程反馈 |
| 删除 | `GC_task`、`action` | 原物流赛任务流程 |
| 删除 | `servo`、`LobotServoController`、`huaner_servo`、`ServoMotorControl` | 新车无舵机 |
| 删除 | `QRcode`、`laser`、`bluetooth` | 新需求不使用；其中二维码/蓝牙还会与视觉争用 UART5 |
| 删除并替换 | `Motor_Move`、`GC_Chassis_Control` | 四轮麦克纳姆和 Z 轴专用，已由 DS 两轮/平衡架接口替换 |

## 上车前必须确认

1. 三个步进驱动器的正方向和最大安全速度。
2. 8 路红外的黑线有效电平和是否需要上下拉。
3. 视觉模块最终波特率、数据字段和启动命令。
4. HWT101 安装方向以及偏航角正负号。
5. 平衡架是否有零位/限位开关；若有，需要增加回零和越界保护。
