const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];

const ui = {
  deviceDot: $("#device-dot"),
  deviceState: $("#device-state"),
  deviceRelease: $("#device-release"),
  liveFrame: $("#live-frame"),
  frameEmpty: $("#frame-empty"),
  streamState: $("#stream-state"),
  recordState: $("#record-state"),
  recordButton: $("#record-button"),
  stopRecordButton: $("#stop-record-button"),
  activityIndicator: $("#activity-indicator"),
  activityTitle: $("#activity-title"),
  activityMessage: $("#activity-message"),
  activityLog: $("#activity-log"),
  toast: $("#toast"),
  form: $("#parameter-form"),
  simTrack: $("#sim-track"),
  simBall: $("#sim-ball"),
  simTarget: $(".sim-target"),
  signalValidDot: $("#signal-valid-dot"),
  signalValidLabel: $("#signal-valid-label"),
};

let latestState = null;
let frameLoading = false;
let frameReady = false;
let frameTimer = null;
let formDirty = false;
let configFingerprint = "";
let toastTimer = null;
let telemetrySource = null;
let telemetrySourceUrl = "";
let telemetryEventCount = 0;
let telemetryRateCount = 0;
let telemetryRateStartedMs = 0;
let lastTelemetryKey = "";
let lastTelemetrySession = "";
let historyWindowMs = 30000;
let chartsDirty = true;
let lastChartDrawMs = 0;
const telemetryHistory = [];
const MAX_HISTORY_MS = 65000;
const chartColors = {
  position: "#43d6b1",
  velocity: "#72a7ff",
  fps: "#e9c46a",
  detect: "#ff9f67",
  latency: "#b18cff",
  grid: "rgba(132, 144, 157, 0.13)",
  axis: "rgba(174, 185, 192, 0.48)",
  reference: "rgba(237, 243, 244, 0.26)",
  coast: "#ffb65c",
};
const operationLabels = {
  preview: "连接预览",
  record: "开始录制",
  stop_record: "结束录制",
  stop_monitor: "停止预览",
  deploy_restart: "部署并重启",
  device_start: "启动板端",
  device_stop: "停止板端",
  rollback: "版本回退",
};

function formatNumber(value, digits = 1, fallback = "—") {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : fallback;
}

function text(id, value) {
  const element = document.getElementById(id);
  if (element) element.textContent = value == null ? "—" : String(value);
}

function boolLabel(value, yes = "是", no = "否") {
  return value == null ? "—" : value ? yes : no;
}

function setVerdict(element, label, tone) {
  element.textContent = label;
  element.className = `verdict ${tone}`;
}

function showToast(message, error = false) {
  ui.toast.textContent = message;
  ui.toast.style.borderColor = error ? "rgba(255,107,99,.55)" : "";
  ui.toast.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => ui.toast.classList.remove("show"), 2800);
}

function finiteOrNull(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function appendTelemetrySample(
  tracking,
  video = {},
  config = {},
  sampleTimeMs = Date.now(),
) {
  if (!tracking || tracking.seq == null) return;

  const session = String(tracking.session || "");
  const key = `${session}:${tracking.seq}`;
  if (key === lastTelemetryKey) return;
  if (lastTelemetrySession && session !== lastTelemetrySession) {
    telemetryHistory.length = 0;
  }
  lastTelemetrySession = session;
  lastTelemetryKey = key;

  const valid = Boolean(tracking.valid);
  const now = Number.isFinite(Number(sampleTimeMs))
    ? Number(sampleTimeMs)
    : Date.now();
  telemetryHistory.push({
    t: now,
    position: valid ? finiteOrNull(tracking.position) : null,
    velocity: valid ? finiteOrNull(tracking.velocity_px_s) : null,
    fps: finiteOrNull(tracking.fps),
    detect: finiteOrNull(tracking.detect_ms),
    latency: finiteOrNull(video.pipeline_latency_ms),
    measured: Boolean(tracking.measured),
    valid,
    coasting: Boolean(tracking.coasting),
    target: finiteOrNull(config.target_position),
  });

  const oldest = now - MAX_HISTORY_MS;
  while (telemetryHistory.length && telemetryHistory[0].t < oldest) {
    telemetryHistory.shift();
  }
  chartsDirty = true;
}

function renderSimulator(tracking, config) {
  const position = finiteOrNull(tracking.position);
  const valid = Boolean(tracking.valid) && position != null;
  const measured = valid && Boolean(tracking.measured);
  const coasting = valid && !measured;
  const target = clamp(finiteOrNull(config.target_position) ?? 0.5, 0, 1);

  ui.simTarget.style.left = `${target * 100}%`;
  ui.simTrack.dataset.seq = tracking.seq == null ? "" : String(tracking.seq);
  ui.simTrack.dataset.state = measured ? "measured" : coasting ? "coasting" : "invalid";
  ui.signalValidDot.className = `status-dot ${measured ? "online" : coasting ? "busy" : "error"}`;

  if (valid) {
    const boundedPosition = clamp(position, 0, 1);
    ui.simBall.style.left = `${boundedPosition * 100}%`;
    text("sim-position", `${(boundedPosition * 100).toFixed(1)}% · ${measured ? "真实测量" : "短时预测"}`);
    ui.signalValidLabel.textContent = measured ? "视觉测量有效" : "短时预测输出";
  } else {
    text("sim-position", "未检测到有效钢球");
    ui.signalValidLabel.textContent = "无有效球";
  }
}

function renderLiveTelemetry(
  tracking = {},
  video = {},
  config = {},
  sampleTimeMs = Date.now(),
) {
  appendTelemetrySample(tracking, video, config, sampleTimeMs);
  renderSimulator(tracking, config);

  text("metric-fps", formatNumber(tracking.fps, 1));
  text("metric-detect", formatNumber(tracking.detect_ms, 0));
  text(
    "metric-position",
    Number.isFinite(Number(tracking.position)) && tracking.valid
      ? `${(Number(tracking.position) * 100).toFixed(1)}%`
      : "—",
  );
  text("metric-velocity", formatNumber(tracking.velocity_px_s, 0));
  text("metric-delay", formatNumber(video.pipeline_latency_ms, 0));
  text(
    "wave-position-value",
    Number.isFinite(Number(tracking.position)) && tracking.valid
      ? `${(Number(tracking.position) * 100).toFixed(1)}%`
      : "无效",
  );
  text(
    "wave-velocity-value",
    tracking.valid && Number.isFinite(Number(tracking.velocity_px_s))
      ? `${Number(tracking.velocity_px_s).toFixed(0)} px/s`
      : "—",
  );
  text(
    "wave-fps-value",
    Number.isFinite(Number(tracking.fps))
      ? `${Number(tracking.fps).toFixed(1)} FPS`
      : "—",
  );
  text(
    "wave-timing-value",
    Number.isFinite(Number(tracking.detect_ms)) ||
      Number.isFinite(Number(video.pipeline_latency_ms))
      ? `${formatNumber(tracking.detect_ms, 0)} / ${formatNumber(video.pipeline_latency_ms, 0)} ms`
      : "—",
  );
}

function ensureTelemetryStream(monitor) {
  const apiUrl = String(monitor?.api_url || "").replace(/\/+$/, "");
  const nextUrl =
    monitor?.state === "running" && apiUrl
      ? `${apiUrl}/telemetry`
      : "";
  if (nextUrl === telemetrySourceUrl && telemetrySource) return;

  if (telemetrySource) telemetrySource.close();
  telemetrySource = null;
  telemetrySourceUrl = nextUrl;
  telemetryEventCount = 0;
  telemetryRateCount = 0;
  telemetryRateStartedMs = Date.now();
  text(
    "telemetry-rate",
    nextUrl ? "实时推送 · 正在连接" : "实时推送 · 等待连接",
  );
  if (!nextUrl) return;

  const source = new EventSource(nextUrl);
  telemetrySource = source;
  source.onopen = () => {
    if (source === telemetrySource) {
      text("telemetry-rate", "实时推送 · 已连接");
    }
  };
  source.onmessage = (event) => {
    if (source !== telemetrySource) return;
    try {
      const sample = JSON.parse(event.data);
      const tracking = sample.tracking;
      if (!tracking) return;
      const receivedMs = Date.now();
      telemetryEventCount += 1;
      telemetryRateCount += 1;
      ui.simTrack.dataset.eventCount = String(
        telemetryEventCount,
      );
      const rateElapsedMs =
        receivedMs - telemetryRateStartedMs;
      if (rateElapsedMs >= 1000) {
        const rateHz =
          (telemetryRateCount * 1000) / rateElapsedMs;
        text(
          "telemetry-rate",
          `实时推送 · ${rateHz.toFixed(1)} Hz`,
        );
        telemetryRateCount = 0;
        telemetryRateStartedMs = receivedMs;
      }
      const monitorState = latestState?.monitor || {};
      monitorState.tracking = tracking;
      renderLiveTelemetry(
        tracking,
        monitorState.video || {},
        monitorState.config || {},
        Number(sample.host_epoch_ns) / 1_000_000,
      );
    } catch (_error) {
      return;
    }
  };
  source.onerror = () => {
    if (source === telemetrySource) {
      text("telemetry-rate", "实时推送 · 自动重连中");
    }
  };
}

function prepareCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, rect.width);
  const height = Math.max(1, rect.height);
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const pixelWidth = Math.round(width * ratio);
  const pixelHeight = Math.round(height * ratio);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  return { context, width, height };
}

function drawReference(context, left, top, width, height, value, minimum, maximum) {
  if (value == null || maximum <= minimum) return;
  const y = top + (1 - (value - minimum) / (maximum - minimum)) * height;
  context.save();
  context.setLineDash([4, 4]);
  context.strokeStyle = chartColors.reference;
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(left, y);
  context.lineTo(left + width, y);
  context.stroke();
  context.restore();
}

function drawSeries(context, samples, key, color, mapX, mapY) {
  context.save();
  context.strokeStyle = color;
  context.lineWidth = 1.6;
  context.lineJoin = "round";
  context.lineCap = "round";
  context.beginPath();
  let drawing = false;
  for (const sample of samples) {
    const value = sample[key];
    if (value == null || !Number.isFinite(value)) {
      drawing = false;
      continue;
    }
    const x = mapX(sample.t);
    const y = mapY(value);
    if (drawing) context.lineTo(x, y);
    else {
      context.moveTo(x, y);
      drawing = true;
    }
  }
  context.stroke();
  context.restore();
}

function drawWaveChart(canvas, samples, options) {
  const { context, width, height } = prepareCanvas(canvas);
  const padding = { left: 34, right: 9, top: 7, bottom: 18 };
  const plotWidth = Math.max(1, width - padding.left - padding.right);
  const plotHeight = Math.max(1, height - padding.top - padding.bottom);
  const minimum = options.minimum;
  const maximum = Math.max(minimum + 0.001, options.maximum);
  const endTime = samples.length ? samples[samples.length - 1].t : Date.now();
  const startTime = endTime - historyWindowMs;
  const mapX = (time) => padding.left + ((time - startTime) / historyWindowMs) * plotWidth;
  const mapY = (value) => padding.top + (1 - (value - minimum) / (maximum - minimum)) * plotHeight;

  context.save();
  context.strokeStyle = chartColors.grid;
  context.lineWidth = 1;
  for (let row = 0; row <= 3; row += 1) {
    const y = padding.top + (plotHeight * row) / 3;
    context.beginPath();
    context.moveTo(padding.left, y);
    context.lineTo(padding.left + plotWidth, y);
    context.stroke();
  }
  for (let column = 0; column <= 3; column += 1) {
    const x = padding.left + (plotWidth * column) / 3;
    context.beginPath();
    context.moveTo(x, padding.top);
    context.lineTo(x, padding.top + plotHeight);
    context.stroke();
  }
  context.fillStyle = chartColors.axis;
  context.font = '9px "Cascadia Mono", Consolas, monospace';
  context.textAlign = "right";
  context.fillText(options.formatAxis(maximum), padding.left - 5, padding.top + 7);
  context.fillText(options.formatAxis(minimum), padding.left - 5, padding.top + plotHeight);
  context.textAlign = "left";
  context.fillText(`-${Math.round(historyWindowMs / 1000)}s`, padding.left, height - 4);
  context.textAlign = "right";
  context.fillText("现在", padding.left + plotWidth, height - 4);
  context.restore();

  drawReference(
    context,
    padding.left,
    padding.top,
    plotWidth,
    plotHeight,
    options.reference,
    minimum,
    maximum,
  );
  for (const series of options.series) {
    drawSeries(context, samples, series.key, series.color, mapX, mapY);
  }

  if (options.coastDots) {
    context.save();
    context.fillStyle = chartColors.coast;
    for (const sample of samples) {
      if (!sample.coasting || sample.position == null) continue;
      context.beginPath();
      context.arc(mapX(sample.t), mapY(sample.position), 1.8, 0, Math.PI * 2);
      context.fill();
    }
    context.restore();
  }
}

function historyForCurrentWindow() {
  if (!telemetryHistory.length) return [];
  const latestTime = telemetryHistory[telemetryHistory.length - 1].t;
  const startTime = latestTime - historyWindowMs;
  return telemetryHistory.filter((sample) => sample.t >= startTime);
}

function renderCharts() {
  const samples = historyForCurrentWindow();
  const absoluteVelocity = samples
    .map((sample) => Math.abs(sample.velocity ?? 0))
    .filter(Number.isFinite);
  const velocityLimit = Math.max(100, Math.min(800, Math.ceil(Math.max(0, ...absoluteVelocity) / 100) * 100));
  const timingValues = samples.flatMap((sample) => [sample.detect, sample.latency]).filter(Number.isFinite);
  const timingMaximum = Math.max(100, Math.min(500, Math.ceil(Math.max(0, ...timingValues) / 50) * 50));
  const target = samples.length ? samples[samples.length - 1].target : 0.5;

  drawWaveChart($("#position-chart"), samples, {
    minimum: 0,
    maximum: 1,
    reference: target ?? 0.5,
    formatAxis: (value) => `${Math.round(value * 100)}%`,
    series: [{ key: "position", color: chartColors.position }],
    coastDots: true,
  });
  drawWaveChart($("#velocity-chart"), samples, {
    minimum: -velocityLimit,
    maximum: velocityLimit,
    reference: 0,
    formatAxis: (value) => `${Math.round(value)}`,
    series: [{ key: "velocity", color: chartColors.velocity }],
  });
  drawWaveChart($("#fps-chart"), samples, {
    minimum: 0,
    maximum: 60,
    reference: 30,
    formatAxis: (value) => `${Math.round(value)}`,
    series: [{ key: "fps", color: chartColors.fps }],
  });
  drawWaveChart($("#timing-chart"), samples, {
    minimum: 0,
    maximum: timingMaximum,
    reference: null,
    formatAxis: (value) => `${Math.round(value)}`,
    series: [
      { key: "detect", color: chartColors.detect },
      { key: "latency", color: chartColors.latency },
    ],
  });
}

function chartAnimationFrame(timestamp) {
  if (chartsDirty && timestamp - lastChartDrawMs >= 100) {
    renderCharts();
    chartsDirty = false;
    lastChartDrawMs = timestamp;
  }
  window.requestAnimationFrame(chartAnimationFrame);
}

async function action(name, body = {}) {
  const response = await fetch("/api/action", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: name, ...body }),
  });
  const result = await response.json();
  if (!response.ok || !result.ok) {
    throw new Error(result.error || `操作失败 (${response.status})`);
  }
  return result;
}

async function requestAction(name, body = {}, message = "操作已提交") {
  try {
    const result = await action(name, body);
    showToast(message);
    return result;
  } catch (error) {
    showToast(error.message, true);
    throw error;
  }
}

function renderState(state) {
  latestState = state;
  const device = state.device || {};
  const monitor = state.monitor;
  const operation = state.operation || {};
  const analysis = state.last_analysis || {};
  const running = Boolean(device.running);

  ui.deviceDot.className = `status-dot ${running ? "online" : device.error ? "error" : ""}`;
  ui.deviceState.textContent = running ? "MaixCAM 视觉运行中" : device.error ? "设备不可达" : "板端未运行";
  ui.deviceRelease.textContent = `${state.device_ip} · ${device.current_release || "未部署"}`;

  const streamRunning = monitor && monitor.state === "running";
  ui.streamState.textContent = streamRunning ? "STREAM ONLINE" : monitor ? monitor.state.toUpperCase() : "STREAM OFFLINE";
  ui.streamState.classList.toggle("active", Boolean(monitor));

  const recording = Boolean(monitor && monitor.recording);
  ui.recordState.textContent = recording ? "● RECORDING" : "PREVIEW ONLY";
  ui.recordState.classList.toggle("active", recording);
  ui.recordButton.hidden = recording;
  ui.stopRecordButton.hidden = !recording;

  const tracking = monitor?.tracking || {};
  const video = monitor?.video || {};
  const sync = monitor?.synchronization || {};
  const config = monitor?.config || {};

  ensureTelemetryStream(monitor);
  renderLiveTelemetry(tracking, video, config);

  text("status-measured", boolLabel(tracking.measured, "检测到", "未检测"));
  text("status-valid", boolLabel(tracking.valid, "有效", "无效"));
  text("status-pipe", boolLabel(tracking.pipe_valid, "稳定", "失效"));
  text("status-lateral", tracking.lateral_px == null ? "—" : `${formatNumber(tracking.lateral_px, 1)} px`);
  text("status-error", tracking.error_px == null ? "—" : `${formatNumber(tracking.error_px, 1)} px`);
  text("status-sync", sync.match_delta_ms == null ? "—" : `${formatNumber(sync.match_delta_ms, 1)} ms`);

  const verdict = $("#tracking-verdict");
  if (tracking.measured) setVerdict(verdict, "真实测量", "good");
  else if (tracking.valid) setVerdict(verdict, "短时预测", "warning");
  else setVerdict(verdict, "无有效球", "neutral");

  text("session-directory", monitor?.session_directory || "尚未建立");
  text("analysis-measured", Number.isFinite(Number(analysis.measured_ratio)) ? `${(Number(analysis.measured_ratio) * 100).toFixed(1)}%` : "—");
  text("analysis-valid", Number.isFinite(Number(analysis.valid_ratio)) ? `${(Number(analysis.valid_ratio) * 100).toFixed(1)}%` : "—");
  text("analysis-fps", Number.isFinite(Number(analysis.detector_fps_mean)) ? `${Number(analysis.detector_fps_mean).toFixed(1)} FPS` : "—");
  text("analysis-delay", Number.isFinite(Number(analysis.video_latency_p50_ms)) ? `${Number(analysis.video_latency_p50_ms).toFixed(1)} ms` : "—");
  text("system-ip", state.device_ip);
  text("system-process", running ? `PID ${device.process?.pid || "—"}` : "已停止");
  text("system-release", device.current_release || "—");
  text("system-hash", device.source_hash ? `${device.source_hash.slice(0, 16)}…` : "—");

  const operationBadge = $("#operation-state");
  const operationRunning = operation.state === "running";
  if (operationRunning) setVerdict(operationBadge, "执行中", "warning");
  else if (operation.state === "failed") setVerdict(operationBadge, "失败", "bad");
  else if (operation.state === "completed") setVerdict(operationBadge, "已完成", "good");
  else setVerdict(operationBadge, "空闲", "neutral");

  ui.activityIndicator.className = `status-dot ${operationRunning ? "busy" : operation.state === "failed" ? "error" : monitor ? "online" : ""}`;
  ui.activityTitle.textContent = operationRunning ? (operationLabels[operation.name] || operation.name) : operation.state === "failed" ? "操作失败" : monitor ? "实验链路在线" : "控制台就绪";
  ui.activityMessage.textContent = operation.message || (monitor?.last_command?.state ? `最近命令：${monitor.last_command.state}` : "等待操作");
  ui.activityLog.textContent = (state.logs || []).join("\n");
  ui.activityLog.scrollTop = ui.activityLog.scrollHeight;

  $$("button").forEach((button) => {
    if (button.classList.contains("tab")) return;
    button.disabled = operationRunning;
  });

  const fingerprint = JSON.stringify(config);
  if (!formDirty && fingerprint !== configFingerprint) {
    for (const input of ui.form.elements) {
      if (input.name && config[input.name] != null) input.value = config[input.name];
    }
    configFingerprint = fingerprint;
  }
  const configCommand = monitor?.last_command;
  const configBadge = $("#config-state");
  if (configCommand?.type === "set_config") {
    if (configCommand.state === "applied") setVerdict(configBadge, "设备已应用", "good");
    else if (configCommand.state === "rejected" || configCommand.state === "ack_timeout") setVerdict(configBadge, "应用失败", "bad");
    else if (configCommand.state === "waiting_for_device_ack") setVerdict(configBadge, "等待确认", "warning");
  } else if (!formDirty) {
    setVerdict(configBadge, "设备值", "neutral");
  }
}

async function pollState() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`状态 ${response.status}`);
    renderState(await response.json());
  } catch (error) {
    ui.deviceDot.className = "status-dot error";
    ui.deviceState.textContent = "控制台服务断开";
  }
}

function updateFrame() {
  clearTimeout(frameTimer);
  const streamRunning = latestState?.monitor?.state === "running";
  if (!streamRunning) {
    frameTimer = setTimeout(updateFrame, 250);
    return;
  }
  if (frameLoading) {
    frameTimer = setTimeout(updateFrame, 20);
    return;
  }
  frameLoading = true;
  const probe = new Image();
  probe.onload = () => {
    ui.liveFrame.src = probe.src;
    ui.liveFrame.classList.add("ready");
    ui.frameEmpty.classList.add("hidden");
    frameReady = true;
    frameLoading = false;
    frameTimer = setTimeout(updateFrame, 45);
  };
  probe.onerror = () => {
    frameLoading = false;
    if (!frameReady) {
      ui.liveFrame.classList.remove("ready");
      ui.frameEmpty.classList.remove("hidden");
    }
    frameTimer = setTimeout(updateFrame, 180);
  };
  probe.src = `/api/frame.jpg?t=${Date.now()}`;
}

$$(".tab").forEach((tab) => {
  tab.addEventListener("click", () => {
    $$(".tab").forEach((item) => item.classList.toggle("active", item === tab));
    $$(".tab-panel").forEach((panel) => {
      panel.classList.toggle("active", panel.dataset.panel === tab.dataset.tab);
    });
  });
});

$("#preview-button").addEventListener("click", () => requestAction("preview", {}, "正在连接低延迟预览"));
$("#record-button").addEventListener("click", () => requestAction("record", {}, "正在切换到录像模式"));
$("#stop-record-button").addEventListener("click", () => requestAction("stop_record", {}, "正在封装录像并恢复预览"));
$("#deploy-button").addEventListener("click", () => requestAction("deploy_restart", {}, "已开始校验、部署和重启"));
$("#device-start-button").addEventListener("click", () => requestAction("device_start", {}, "正在启动板端视觉"));
$("#device-stop-button").addEventListener("click", () => requestAction("device_stop", {}, "正在停止板端视觉"));
$("#snapshot-button").addEventListener("click", () => requestAction("snapshot", {}, "截图命令已提交"));

$$("[data-marker]").forEach((button) => {
  button.addEventListener("click", () => requestAction("mark", { label: button.dataset.marker }, `已标记：${button.dataset.marker}`));
});

$$("[data-window-seconds]").forEach((button) => {
  button.addEventListener("click", () => {
    historyWindowMs = Number(button.dataset.windowSeconds) * 1000;
    $$("[data-window-seconds]").forEach((item) => item.classList.toggle("active", item === button));
    chartsDirty = true;
  });
});

ui.form.addEventListener("input", () => {
  formDirty = true;
  setVerdict($("#config-state"), "未应用", "warning");
});

ui.form.addEventListener("submit", async (event) => {
  event.preventDefault();
  const params = {};
  for (const input of ui.form.elements) {
    if (!input.name || input.value === "") continue;
    params[input.name] = Number(input.value);
  }
  await requestAction("config", { params }, "参数已发送，等待设备确认");
  formDirty = false;
  setVerdict($("#config-state"), "等待确认", "warning");
});

if ("ResizeObserver" in window) {
  const chartResizeObserver = new ResizeObserver(() => {
    chartsDirty = true;
  });
  $$("canvas").forEach((canvas) => chartResizeObserver.observe(canvas));
}

setInterval(pollState, 300);
window.requestAnimationFrame(chartAnimationFrame);
pollState();
updateFrame();
