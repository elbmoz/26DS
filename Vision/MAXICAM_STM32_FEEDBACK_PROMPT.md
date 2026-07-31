# MaixCAM 接收 STM32 控制反馈的提示词

将下面整段提示词交给负责 MaixCAM 视觉项目的代码助手即可：

```text
请在现有 MaixCAM 钢球视觉项目中增加“STM32 控制反馈接收、转发、记录和
离线绘图”功能。不要改动钢球识别、管道识别、ROI、滤波器、速度估计、
相机参数或现有 50 Hz 视觉输出算法。

现有硬件和协议必须保持：
- MaixCAM 使用 UART1：/dev/ttyS1，A19=TX，A18=RX。
- STM32 使用 USART6：PC6=TX，PC7=RX。
- 115200、8N1、TX/RX交叉、共地。
- STM32发送无换行命令 c2 后，MaixCAM开始发送：
  B,<error_px>,<velocity_px_s>\n
- 无有效球时发送 none\n。
- STM32发送无换行命令 ok 后停止视觉串口输出。
- 保留现有 ASCII、逗号分隔、换行结尾的协议风格。

STM32现在会在每次平衡电机速度命令或停止命令后返回：
F,<seq>,<mcu_ms>,<vision_frame>,<vision_age_ms>,<position_x10>,
  <velocity_x10>,<error_x10>,<p_x100>,<i_x100>,<d_x100>,
  <motor_command>,<motor_status>\n

字段定义：
- seq：STM32反馈尝试序号；跳号表示STM32因USART6仍忙而主动丢了一条日志。
- mcu_ms：电机命令下发后的HAL_GetTick毫秒。
- vision_frame：本次PID使用的STM32视觉接收帧号。
- vision_age_ms：下发电机命令时视觉数据在STM32中的年龄。
- position_x10、velocity_x10、error_x10除以10后分别是参考像素位置、
  参考像素/秒和control_error=target-position。
- p_x100、i_x100、d_x100除以100后是球位置外环的P/I/D目标杆角分量，
  单位为度。
- motor_command：杆角内环经方向、限幅、slew和取整后的实际有符号速度命令。
- motor_status：0=HAL_OK、1=HAL_ERROR、2=HAL_BUSY、3=HAL_TIMEOUT。

实现要求：
1. 在现有stm32_link模块内复用同一个UART对象做全双工通信。
2. UART读取必须非阻塞，能够处理半包、多包合并、c2/ok没有换行、F帧有换行
   的混合字节流；坏帧只计数并丢弃，不影响视觉发送。
3. 不要每帧print，不要在比赛识别循环里同步写板载Flash。使用有界内存队列；
   队列满时丢最旧反馈并计数，不能阻塞识别。
4. 保留每一条有效F帧，不要把100 Hz左右的STM32反馈降采样到30 Hz tracking。
5. 默认stream模式把每条F帧封装为独立UDP JSON包：
   {"v":1,"type":"control",...}
   保持现有session、seq、device_ms和紧凑JSON风格。
6. control包必须包含：
   feedback_seq,mcu_ms,vision_frame,vision_age_ms,mcu_position_px,
   mcu_velocity_px_s,control_error_px,p_term,i_term,d_term,
   motor_command,motor_status。
7. Windows接收端把control包逐条写到独立control.csv，同时继续写入原始
   telemetry.jsonl；session.json增加control_count。
8. APP_MODE=record的本地兜底模式也写control.csv，但只按现有批次周期flush。
9. 增加离线绘图工具，读取control.csv生成control_curves.png，至少包含：
   视觉位置/控制误差、视觉速度、P/I/D与电机命令、vision_age_ms与
   motor_status四组曲线。绘图只能在Windows离线执行。
10. 增加单元测试：定点倍率、半包、多包、c2/ok与F共线、坏帧、seq跳号、
    UDP control包往返和control.csv写入。
11. 更新README和协议文档，明确F帧字段、倍率、状态码和日志丢弃策略。

验收标准：
- 原有B/none/c2/ok测试继续通过。
- 视觉识别输出结果不发生变化。
- STM32反馈不存在时，现有stream/record功能保持原行为。
- 反馈高频到达时识别循环不等待串口发送、文件写入或绘图。
- control.csv可直接用于对比“视觉输入 -> PID分量 -> 电机命令”。
```
