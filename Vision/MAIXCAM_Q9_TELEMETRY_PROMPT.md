# MaixCAM 接收任务 9 遥测数据的修改 Prompt

将下面整段 Prompt 交给负责 MaixCAM 视觉项目的代码助手。STM32 端已经实现
USART6 非阻塞发送，视觉端只需要接收、解析和显示，不能改变现有识别与第二题控制
协议。

```text
请在当前 MaixCAM 视觉项目中增加 STM32 任务 9 遥测接收功能。先阅读现有
Vision/maixcam/stm32_link.py、main.py、stream_tracking.py、
record_tracking.py 和 Vision/tests/test_stm32_link.py，再实施修改。

目标：
- 继续使用当前连接 MaixCAM 与 STM32 的同一个 UART，115200、8N1。
- 接收 STM32 以 5 Hz 发送的任务 9 ASCII 遥测帧。
- 在视觉端画面叠加显示电机位置和三轴相对角度。
- 不修改球识别、管道识别、ROI、滤波、速度估计和相机参数。
- 不改变现有 c2/ok/B/none 第二题协议和行为。

STM32 发送帧格式：
Q9,<seq>,<mcu_ms>,<motor_position>,<angle_x_x10>,<angle_y_x10>,
   <angle_z_x10>,<imu_valid>,<position_valid>,<position_status>,
   <position_updates>,<move_direction>,<move_status>\n

实际线上是一行，不包含空格。例如：
Q9,17,25340,4897,-123,48,906,1,1,0,84,-1,0\n

字段定义：
1. seq：STM32 的发送尝试序号，从 1 开始。序号跳变表示上一帧可能因 UART 忙而
   被 STM32 主动丢弃。
2. mcu_ms：STM32 的 HAL_GetTick() 毫秒时间。
3. motor_position：3 号电机按 0x36 协议读到的 int32 原始位置。
4. angle_x_x10、angle_y_x10、angle_z_x10：任务 9 点击确认时归零后的三轴
   相对角度，除以 10 得到度。
5. imu_valid：0/1，1 表示最近 200 ms 内收到过有效 JY61P 0x53 角度帧。
6. position_valid：0/1，1 表示电机位置监视器当前有可信的位置。
7. position_status：STM32 HAL 状态，0=HAL_OK、1=HAL_ERROR、
   2=HAL_BUSY、3=HAL_TIMEOUT。
8. position_updates：任务 9 本次运行中成功更新电机位置的累计次数。
9. move_direction：最后一次成功发出的相对运动方向，-1、0 或 +1。
10. move_status：最近一次任务 9 往返运动命令的 HAL 状态，编码同上。

任务 9 不发送 c9，也不要求视觉端回 ACK。STM32 进入任务 9 后自动开始发 Q9，
退出任务 9 后自动停止。视觉端应始终允许解析 Q9，但只有收到 c2 后才发送
B/none；收到 Q9 绝对不能把 streaming 改为 True。

实现要求：
1. 扩展 Stm32Link，使用持久的有界 RX 缓冲区处理 UART 分包，必须支持：
   - 一帧被拆成多次 read；
   - 一次 read 包含多帧；
   - 无换行 c2/ok 命令与有换行 Q9 帧连续或混合到达；
   - \r\n 和 \n；
   - 非 ASCII、超长行和坏帧只计数并丢弃，不阻塞主循环。
2. 不要假设 serial.read() 每次恰好返回一帧。不要调用阻塞式 readline()。
3. 严格校验 Q9：
   - 前缀必须为 Q9；
   - Q9 后必须恰好有 12 个字段；
   - 所有字段必须是十进制整数；
   - imu_valid 和 position_valid 只能为 0/1；
   - move_direction 只能为 -1/0/1；
   - position_status 和 move_status 只接受 0..3；
   - 解析失败不覆盖上一条有效数据。
4. 在 Stm32Link 中至少公开：
   - latest_q9：最新有效帧的字典，没有数据时为 None；
   - q9_frame_count；
   - q9_parse_error_count；
   - q9_sequence_gap_count；
   - get_latest_q9()，返回最新数据，角度同时提供原始 x10 整数和除以 10 后的
     angle_x_deg/angle_y_deg/angle_z_deg。
5. seq 使用无符号 32 位回绕规则统计跳号；首帧不计算 gap，重复帧不能产生负数。
6. main.py 每帧继续非阻塞调用 poll_commands()。收到过 Q9 后，在预览画面用
   小号文字叠加：
   Q9 P:<motor_position>
   X:<angle_x_deg> Y:<angle_y_deg> Z:<angle_z_deg>
   IMU:<V/X> POS:<V/X> RX:<position_status> N:<position_updates>
   DIR:<move_direction> MOVE:<move_status>
   不要每帧 print，不要改变摄像头帧率和识别结果。
7. 如果方便，请让 stream_tracking.py 和 record_tracking.py 继续使用同一个
   Stm32Link 接口；它们至少不能因为新解析器而退化或报错。是否在这两个模式中
   叠加 Q9 信息可由现有 UI 结构决定。
8. 不要因为长时间收不到 Q9 而报错或停止视觉功能。可以额外记录视觉端接收时间
   并标记 stale，但不能伪造 STM32 字段。
9. 保持代码兼容 MaixPy 当前支持的 Python 语法和串口 API，不引入 CPython 专属
   的重型依赖。

测试要求：
- 保留并通过所有现有 test_stm32_link.py 测试。
- 新增以下单元测试：
  1. 完整合法 Q9 帧；
  2. 负电机位置和负角度；
  3. Q9 半包；
  4. 一次读取两个 Q9 帧；
  5. c2 + Q9 + ok 混合流，最终 streaming=False 且 Q9 正确；
  6. Q9 后紧跟 c2，无额外换行命令仍被识别；
  7. 字段数错误、非法整数、非法 valid/status/direction；
  8. 超长帧恢复后仍能解析下一条好帧；
  9. seq 跳号和 uint32 回绕；
  10. 坏帧不会覆盖 latest_q9。

验收标准：
- Q9 帧分包、粘包和混合命令情况下均可恢复。
- 原 c2/ok/B/none 测试和实际行为保持不变。
- 未收到 Q9 时视觉项目行为与修改前一致。
- Q9 接收和画面叠加不进行阻塞文件写入，不明显降低视觉帧率。
- 视觉画面显示的角度为 x10 字段除以 10 后的度数，电机位置保持 int32 原值。

完成后请列出修改文件、协议解析入口、新增测试以及测试执行结果。
```
