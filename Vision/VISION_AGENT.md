# 无 MaixVision 的视觉迭代上位机

`Vision/windows/vision_agent.py` 通过 MaixCAM 原生 SSH/SFTP 管理板端视觉
程序；RTSP、UDP 遥测、在线参数更新、Windows 录像和自动分析仍使用现有
比赛链路。它不会控制或使能电机。

## 一次准备

```powershell
python -m pip install -r Vision\windows\requirements.txt
ffmpeg -version
```

USB 虚拟网卡默认地址为 `10.16.6.1`。MaixCAM 出厂 SSH 用户和密码均为
`root`；如果密码已经修改，不要写进仓库：

```powershell
$env:MAIXCAM_PASSWORD = "修改后的密码"
```

## 一体化图形上位机

推荐双击：

```text
Vision/windows/start_operator_console.cmd
```

浏览器会打开本机 `http://127.0.0.1:8770/`。页面包含：

- 单路 RTSP 实时画面、管道轴线、真实检测点、滤波位置和目标点。
- 识别 FPS、检测耗时、位置、速度、图传延迟和视频/遥测同步误差。
- 设备部署、重启、预览、Windows 录像、截图和实验阶段标记。
- 已安装 MaixHub 模型、当前模型、统一置信度阈值、IoU、目标点、续跟帧数
  及 MaixCAM ACK 状态。
- 受管 PID、当前源码版本、设备日志和会话目录。

上位机只管理 AI 追踪配置，不再暴露传统视觉 V1/V2 的切换或参数。模型与
阈值修改收到 ACK 后会写入板端运行配置，部署或重启不会恢复成源码默认值。
新模型加载失败时设备继续使用原模型，并把失败结果返回页面。

页面与 Codex 共用下列本机接口，不需要控制浏览器或 MaixVision：

```text
GET  http://127.0.0.1:8770/api/state
GET  http://127.0.0.1:8770/api/frame.jpg
POST http://127.0.0.1:8770/api/action
```

`/api/action` 的 `action` 支持 `preview`、`record`、`stop_record`、
`deploy_restart`、`device_start`、`device_stop`、`snapshot`、`mark`、
`config` 和 `sync`。例如：

```powershell
$body = @{
  action = "mark"
  label = "快速向右"
} | ConvertTo-Json
Invoke-RestMethod http://127.0.0.1:8770/api/action `
  -Method Post -ContentType application/json -Body $body
```

两个 HTTP 服务均只监听 `127.0.0.1`；画面由 Windows 已经解码的同一条
RTSP 生成，不会让浏览器再次连接 MaixCAM。

## 命令行使用

旧的双窗口启动器仍可完成“校验源码 → 增量部署 → 重启板端 → 打开
OpenCV 图传和独立调参窗口”：

```text
Vision/windows/start_managed_monitor.cmd
```

机器可读命令：

```powershell
# 检查本地源码、SSH、受管进程和 RTSP 端口
python Vision\windows\vision_agent.py doctor

# 按内容哈希部署并重启
python Vision\windows\vision_agent.py deploy --restart

# 查看结构化状态和板端日志
python Vision\windows\vision_agent.py status
python Vision\windows\vision_agent.py logs --lines 80

# 只启停本工具启动的进程
python Vision\windows\vision_agent.py stop
python Vision\windows\vision_agent.py start

# 录制 15 秒并直接返回 analysis.json
python Vision\windows\vision_agent.py experiment --duration 15

# 空管等无需保存视频的快速负样本
python Vision\windows\vision_agent.py experiment --duration 8 --no-record
```

非默认 IP 使用全局参数：

```powershell
python Vision\windows\vision_agent.py --device-ip 192.168.1.50 doctor
```

## 版本和回退

每次部署先在 Windows 编译检查全部板端 Python 文件，再给所有源码和参考
图计算 SHA-256。内容相同不会重复上传；不同内容保存在：

```text
/root/pipe_ball_vision/releases/<release_id>/
```

查看及回退：

```powershell
python Vision\windows\vision_agent.py releases
python Vision\windows\vision_agent.py stop
python Vision\windows\vision_agent.py rollback <release_id> --start
```

`stop` 先校验 PID、命令行和工作目录，只允许向
`/root/pipe_ball_vision/releases/` 下由本工具启动的 `main.py` 发送
`SIGINT`。超时才使用强制停止，结果中的 `forced` 会明确标出。它不会按
进程名批量结束 MaixVision、系统应用或其他 Python 程序。

MaixCAM 同时只能有一个前台应用稳定占用相机。启动受管追踪前，上位机会精确
校验并暂停系统 launcher 的触摸界面，launcher 后台服务仍保持运行；正常停止
后立即恢复界面。板端 watchdog 会在追踪进程异常退出时自动恢复 launcher。
如果设备上已有其他前台应用，上位机会拒绝启动并提示先退出该应用，不会通过
进程名强制结束未知程序。

## 推荐的自动迭代循环

1. 修改一个可解释的算法因素。
2. 运行单元测试，再执行 `deploy --restart`。
3. 先录空管负样本，验收螺丝和固定纹理均无有效测量。
4. 放入钢球，依次录静止、慢滚、快滚、换向和短遮挡。
5. 读取新会话的 `analysis.json`、原始视频、遥测和拒绝原因。
6. 效果变差时立即切回上一 `release_id`，再分析差异。

模型、置信度阈值等 AI 参数优先在浏览器控制台热更新；
`iteration_client.py set` 保留为脚本接口。检测通道分辨率、候选解析逻辑等
代码级变化使用本上位机部署。
