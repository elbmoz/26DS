# 比赛图传与遥测协议 v1

## 1. 数据路径

```text
GC4653
  ├─ RGB 480×360 -> 绿色管道姿态 -> 动态 ROI/轴线 -> 钢球检测 -> USART6 -> STM32
  └─ NV21 448×336 -> H.264/RTSP -> Windows

STM32 USART6 feedback -> MaixCAM -> UDP stm32_feedback -> Windows 日志与实时波形
MaixCAM UDP tracking/status -> Windows 日志与画面叠加
Windows UDP set_config     -> MaixCAM 安全参数白名单
```

电机控制只经过 STM32。普通视觉配置协议不包含电机使能或运动指令；独立的
`pid_request` 白名单只允许问题2控制器的 RAM 参数、受控测试和停车操作，详见
`PID_AUTO_TUNING.md`。

## 2. 端口

| 方向 | 协议 | 默认端口 | 用途 |
|---|---|---:|---|
| MaixCAM -> Windows | RTSP/H.264 | 8554 | 图传 |
| MaixCAM -> Windows | UDP/JSON | 42101 | 逐帧遥测、状态、ACK |
| Windows -> MaixCAM | UDP/JSON | 42102 | 订阅和视觉参数更新 |
| MaixCAM UART2 -> STM32 | UART 115200 8N1 | USART6 | 位置误差与轴向速度 |

USB 网卡环境的 MaixCAM 默认地址为 `10.16.6.1`。换成 Wi-Fi 后，在
Windows 启动参数中传入新的 `--device-ip`，不要修改协议。

### 2.1 STM32 题目 2 串口帧

STM32 确认启动题目 2 后发送两个 ASCII 字节 `c2`，MaixCAM 才开始输出。
有效跟踪帧为：

```text
B,<error_px>,<velocity_px_s>\n
```

两项均使用原 640 宽参考像素标定，视觉右侧为正。丢球或预测续航结束后发送
`none\n`。STM32 停止题目时发送 `ok`，MaixCAM 停止输出。换行用于流式
重同步，`B` 帧头用于拒绝其他串口文本。

STM32 在每次平衡电机速度命令或停止命令后返回：

```text
F,<seq>,<mcu_ms>,<vision_frame>,<vision_age_ms>,<position_x10>,
  <velocity_x10>,<error_x10>,<p_x100>,<i_x100>,<d_x100>,
  <motor_command>,<motor_status>\n
```

该反馈与 `c2`/`ok` 共用 USART6 RX。MaixCAM 按换行重同步并使用 `F,`
帧头识别反馈，不改变 `c2`/`ok` 语义。字段含义如下：

| 字段 | 含义 |
|---|---|
| `seq` | STM32反馈尝试序号；跳号表示 USART6 忙时主动丢日志 |
| `mcu_ms` | 电机命令下发后的 `HAL_GetTick()` 毫秒 |
| `vision_frame` | 本次 PID 使用的 STM32视觉接收帧号 |
| `vision_age_ms` | 下发命令时 STM32内视觉数据年龄 |
| `position_x10`, `velocity_x10`, `error_x10` | 除以 10 后为参考像素位置、参考像素/秒、`target-position` |
| `p_x100`, `i_x100`, `d_x100` | 除以 100 后为球位置外环的 P/I/D 目标杆角分量，单位 ° |
| `motor_command` | 杆角内环经方向、限幅、slew 和取整后的有符号速度命令 |
| `motor_status` | 0=`HAL_OK`、1=`HAL_ERROR`、2=`HAL_BUSY`、3=`HAL_TIMEOUT` |

自动调参首次交互后，完整串级状态使用 `F2`；高速组合实验使用 `F3`。F3保留
F2全部字段，并追加：

```text
<tuning_sequence>,<tuning_phase>,<phase_elapsed_ms>
```

`tuning_sequence` 与 `PP` 命令及 `PA` ACK 的序号一致。phase 0～3依次表示
准备回零、正阶跃、负阶跃和最终回零，4表示完成，5表示提前中止。Windows只用
当前序号评分，旧序号的缓存帧会被忽略。

## 3. 会话建立

1. MaixCAM 启动 RTSP、UDP控制端口和识别循环。
2. Windows 在 UDP 42101 监听。
3. Windows 向 MaixCAM 42102 发送 `subscribe`。
4. MaixCAM 将请求来源 IP 和指定端口加入本次进程的单播订阅者集合。
5. MaixCAM 返回 `subscribe_ack`，随后开始单播 `status` 和 `tracking`。
6. Windows 从 `status.rtsp_url` 建立 RTSP连接。

不依赖全局广播，因此可用于 USB网卡、独立路由器和比赛热点。

## 4. 统一封包

所有 UDP 报文都是 UTF-8 JSON，最大 4096 bytes，协议版本固定为：

```json
{"v":1,"type":"..."}
```

未知版本、未知消息类型、过大报文、非法 JSON 和错误令牌全部拒绝。

### 4.1 subscribe

```json
{
  "v": 1,
  "type": "subscribe",
  "request_id": "sub-178532...",
  "token": "pipe-ball-local",
  "telemetry_port": 42101
}
```

### 4.2 status

每秒发送，包含：

- `session`：本次 MaixCAM 进程会话 ID。
- `device_ms`：从识别循环启动开始的单调毫秒。
- `state`：`running` 或 `stopping`。
- `rtsp_url`、`stream_size`、`stream_fps`、`stream_bitrate`。
- `camera_size`、启动兜底用的 `roi`、`axis_start`、`axis_end`。每帧动态
  管道坐标以 `tracking` 报文为准。
- 当前可在线调整的 `config`。
- `network_errors`、`control_errors`。

### 4.3 tracking

目标 30 Hz，主要字段：

| 字段 | 单位/含义 |
|---|---|
| `session`, `seq` | 会话和 UDP 序号 |
| `device_ms` | 设备单调毫秒 |
| `frame_id` | 识别帧号 |
| `loop_dt_ms` | 相邻识别帧间隔 |
| `fps`, `detect_ms` | 识别循环 FPS、当前检测耗时 |
| `measured` | 本帧是否得到真实检测 |
| `valid` | 是否可供控制器使用 |
| `coasting` | 是否为短时预测续航 |
| `track_x`, `track_y`, `radius` | 当前检测通道（默认 480×360）的滤波坐标 |
| `measurement_x`, `measurement_y`, `measurement_radius` | 当前原始实测坐标；预测帧为空 |
| `position` | 沿管道轴线归一化位置 |
| `position_px`, `error_px` | 轴向像素位置和目标误差 |
| `lateral_px` | 垂直管道轴线的偏差 |
| `velocity_px_s` | 轴向估计速度 |
| `quality`, `hits`, `misses` | 跟踪质量与连续状态 |
| `position_rejects`, `lateral_rejects`, `quality_rejects`, `jump_rejects` | 本帧各门控拒绝的候选数 |
| `raw_blob_count`, `candidate_count` | 原始色块和有效候选数 |
| `candidates` | 最多 8 个几何候选的 `[x,y,radius,quality]`，用于误检复盘 |
| `local_search`, `fell_back` | 是否使用二维预测窗口、是否同帧退回完整动态 ROI |
| `axis_x0/y0`, `axis_x1/y1` | 本帧动态管道控制轴线 |
| `roi_x/y/w/h` | 本帧动态钢球搜索范围 |
| `pipe_measured`, `pipe_valid`, `pipe_age_frames` | 本帧管道姿态是否新测、是否有效及沿用帧龄 |
| `pipe_blob_count`, `pipe_length`, `pipe_width`, `pipe_score` | 管道姿态诊断量 |

新轨迹默认只允许在动态管长的 2%～98% 内建立；可信轨迹允许到
1.5%～98.5%。管口反光即使通过色块几何筛选，也会计入
`position_rejects` 而不会成为控制输出。

### 4.4 stm32_feedback

MaixCAM 每收到一条有效 `F` 帧就立即以独立 UDP 报文转发，不等待下一个
30 Hz `tracking` 周期。UDP发送仍为非阻塞旁路，不参与 50 Hz UART调度。

报文保留 STM32整数原值，并同时提供便于绘图的换算值：

| 字段 | 含义 |
|---|---|
| `session` | MaixCAM进程会话 |
| `transport_seq` | 与 tracking/status 共用的 UDP发送序号 |
| `device_ms` | MaixCAM收到并转发该反馈的会话相对毫秒 |
| `seq`、`mcu_ms`、`vision_frame`、`vision_age_ms` | STM32原始字段 |
| `position_x10` ... `d_x100` | STM32原始定点数字段 |
| `position_px`, `velocity_px_s`, `control_error_px` | 除以 10 后的工程量 |
| `p_term`, `i_term`, `d_term` | 除以 100 后的 PID 分量 |
| `motor_command`, `motor_status`, `motor_status_name` | 实际命令与 HAL状态 |
| `seq_gap` | 与上一条有效反馈之间缺失的 STM32反馈尝试数 |
| `raw_line` | 去除换行后的原始 `F,...` 行 |

## 5. 在线参数更新

### 5.1 视觉参数

请求采用全有或全无语义：一个字段非法时整条请求不应用，并返回
`config_ack.ok=false` 和逐字段错误。程序不会执行字符串表达式。

| 参数 | 范围 |
|---|---:|
| `target_position` | 0.05～0.95 |
| `position_alpha` | 0.05～1.00 |
| `velocity_beta` | 0.00～1.00 |
| `lateral_alpha` | 0.05～1.00 |
| `max_axis_distance_px` | 5～80 |
| `max_below_axis_distance_px` | 3～80 |
| `max_frame_jump_px` | 10～240 |
| `acquire_position_margin` | 0.00～0.10，归一化管长 |
| `track_position_margin` | 0.00～0.08，归一化管长 |
| `acquire_endpoint_inset` | 0.00～0.12，两端首次建轨禁区 |
| `track_endpoint_inset` | 0.00～0.08，两端已跟踪禁区 |
| `acquire_min_quality` | 0～200 |
| `track_min_quality` | 0～200 |
| `coast_frames` | 0～15，整数 |
| `local_search_width_px` | 40～470，整数 |
| `circle_threshold` | 100～5000，越大越严格 |
| `circle_min_radius` | 6～24 像素，整数 |
| `circle_max_radius` | 8～32 像素，整数 |

动态管道 LAB阈值、姿态搜索范围、二维局部窗口高度、圆候选的金属色采样规则、
相机曝光和所有电机参数不允许网络热更新；这些参数必须在静止台架上验证、
保存到代码并重新启动后生效。两个端点禁入区已经开放为视觉热调参数，
但每次放宽后必须重跑空管负样本。`acquire_position_margin` 和
`track_position_margin` 是旧标定外延，启用物理端点禁入区时不会放开管口。

默认令牌只是避免同一局域网中的误操作，不是加密认证。比赛前同时修改
`ball_config.py` 的 `STREAM_CONTROL_TOKEN` 和 Windows 环境变量：

```powershell
$env:PIPE_BALL_CONTROL_TOKEN = "队伍自己的随机字符串"
```

### 5.2 STM32 RAM控制参数与高速profile

Windows发送 `pid_request`，MaixCAM完成白名单校验后转成USART6的
`PG/PS/PR/PT/PP/PX`。STM32在20 ms控制边界应用命令，再用`PA`返回实际参数；
在收到`PA`前不能认为请求已生效。

`PP`由STM32自主执行零/正/负/零四相profile，并以误差和速度连续稳定时间决定
提前切相位。网络只负责下发一个profile和接收F3，不参与实验计时。参数范围、
评分、剪枝和命令示例见仓库根目录的`PID_AUTO_TUNING.md`。

## 6. 时间同步与录像

- `device_ms`：设备侧识别时间轴，分析控制和检测性能时使用。
- `host_monotonic_ns`：Windows 收到 UDP 的单调时间。
- FFmpeg使用 `-use_wallclock_as_timestamps 1 -copyts` 保留每个 RTSP输入帧
  的 Windows墙钟时间；接收端据此在 UDP历史队列中选择时间最近的遥测，
  而不是把最新遥测画在缓冲的旧视频帧上。
- `video_frames.csv`：记录每个 Windows解码帧的 `video_source_epoch_ns`、
  `video_pipeline_latency_ms`、`telemetry_match_delta_ms`、主动丢帧数以及
  匹配到的 `session/seq/device_ms/frame_id`。
- `stm32_feedback.csv`：逐条记录控制反馈、Windows接收时间、STM32与
  MaixCAM时间、视觉帧龄、PID分量、实际电机命令和 HAL状态。
- `video.mp4`：Windows直接保存 MaixCAM 已编码的 H.264，结束时从临时
  MKV无损封装为 MP4；预览解码与录像共用同一条 RTSP连接，不做 x264
  二次编码。

原始画面与叠加显示分离：MP4不烧入框线，便于以后重新跑算法；框线由
Windows 根据同一时刻的 UDP 数据实时绘制。后台线程持续排空 FFmpeg
原始帧并只保留最新一张，即使 GUI或参数 ACK短时阻塞也不会累计播放旧帧。

## 7. 故障行为

- UDP遥测失败不会阻塞识别和 STM32 UART。
- UART反馈解析使用 128 条有界队列；异常行只增加解析错误计数，不影响
  `c2`/`ok` 和视觉输出。反馈序号跳号按 STM32主动丢日志处理。
- RTSP客户端断开不会停止识别。
- 视频时间戳附近找不到 UDP遥测时显示
  `VIDEO/TELEMETRY NOT SYNCED`，不得把最新位置强行画到旧画面。
- 在线参数必须收到 `config_ack.ok=true` 才视为生效。
- Windows退出时先正常结束 FFmpeg并封装 MP4，再关闭 UDP。
- 当前固件在 RTSP客户端连接后，无论显式 `Rtsp.stop()` 还是进入 C++析构器
  都可能原生崩溃。MaixVision 的正常停止信号为 `SystemExit(0)`；程序先完成
  UDP清理和输出刷新，再保留该状态码进行进程级退出，由 Linux回收多媒体
  句柄，避开有缺陷的析构路径。未知 Python异常仍以状态码 1 退出。
