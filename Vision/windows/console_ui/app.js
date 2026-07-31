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
  modelSelect: $("#model-select"),
  confidenceSlider: $("#confidence-slider"),
  confidenceOutput: $("#confidence-output"),
  targetPercent: $("#target-percent"),
  iouPercent: $("#iou-percent"),
  coastFrames: $("#coast-frames"),
  waveGrid: $("#wave-grid"),
  chartPickerToggle: $("#chart-picker-toggle"),
  chartPicker: $("#chart-picker"),
  chartPickerClose: $("#chart-picker-close"),
  chartOptions: $("#chart-options"),
  chartSelectionCount: $("#chart-selection-count"),
};

let latestState = null;
let frameLoading = false;
let frameReady = false;
let frameTimer = null;
let formDirty = false;
let configFingerprint = "";
let modelCatalogFingerprint = "";
let toastTimer = null;
let telemetrySource = null;
let telemetrySourceUrl = "";
let telemetryEventCount = 0;
let telemetryRateCount = 0;
let telemetryRateStartedMs = 0;
let lastTelemetryKey = "";
let lastTelemetrySession = "";
let lastTelemetrySeq = null;
let lastFeedbackSession = "";
let lastFeedbackSeq = null;
let historyWindowMs = 30000;
let chartPanOffsetMs = 0;
let chartHoverTimeMs = null;
let chartDragging = false;
let chartDragStartX = 0;
let chartDragStartPanMs = 0;
let expandedChartPanel = null;
let chartResizeObserver = null;
let chartsDirty = true;
let lastHistoryPruneMs = 0;
const telemetryHistory = [];
const feedbackHistory = [];
const MAX_HISTORY_MS = 10 * 60 * 1000;
const MIN_CHART_WINDOW_MS = 1000;
const MAX_CHART_WINDOW_MS = 5 * 60 * 1000;
const chartColors = {
  position: "#43d6b1",
  positionPoint: "#a0f5df",
  velocity: "#72a7ff",
  velocityPoint: "#b9d2ff",
  fps: "#e9c46a",
  fpsPoint: "#ffe4a1",
  detect: "#ff9f67",
  detectPoint: "#ffd0b5",
  latency: "#b18cff",
  latencyPoint: "#ddc9ff",
  error: "#ffb454",
  errorPoint: "#ffd49c",
  lateral: "#ef7d91",
  lateralPoint: "#ffc0ca",
  controllerPosition: "#54c7ec",
  controllerPositionPoint: "#b7ecff",
  target: "#8ee38e",
  targetPoint: "#c8f6c8",
  controllerVelocity: "#a78bfa",
  controllerVelocityPoint: "#d8c7ff",
  pTerm: "#ff6b6b",
  pTermPoint: "#ffb2b2",
  iTerm: "#4ecdc4",
  iTermPoint: "#a8f3ee",
  dTerm: "#ffe66d",
  dTermPoint: "#fff3ae",
  motor: "#f78c6c",
  motorPoint: "#ffc1aa",
  visionAge: "#82aaff",
  visionAgePoint: "#c5d7ff",
  gap: "#f59e0b",
  gapPoint: "#ffd18a",
  motorStatus: "#ef4444",
  motorStatusPoint: "#ffaaaa",
  grid: "rgba(132, 144, 157, 0.13)",
  axis: "rgba(174, 185, 192, 0.48)",
  reference: "rgba(237, 243, 244, 0.26)",
  crosshair: "rgba(219, 231, 235, 0.52)",
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

const chartDefinitions = [
  {
    id: "position",
    group: "vision",
    groupLabel: "视觉输出",
    title: "视觉位置",
    source: "tracking",
    axis: { fixed: true, minimum: 0, maximum: 1, step: 0.25 },
    reference: (samples) =>
      samples.length ? samples[samples.length - 1].target : 0.5,
    formatAxis: (value) => `${Math.round(value * 100)}%`,
    series: [
      {
        key: "position",
        label: "位置",
        color: chartColors.position,
        pointColor: chartColors.positionPoint,
        formatValue: (value) => `${(value * 100).toFixed(2)}%`,
      },
    ],
  },
  {
    id: "velocity",
    group: "vision",
    groupLabel: "视觉输出",
    title: "视觉速度",
    source: "tracking",
    axis: {
      symmetric: true,
      includeZero: true,
      minimumSpan: 40,
      fallbackMinimum: -100,
      fallbackMaximum: 100,
    },
    reference: 0,
    series: [
      {
        key: "velocity",
        label: "速度",
        color: chartColors.velocity,
        pointColor: chartColors.velocityPoint,
        formatValue: (value) => `${value.toFixed(1)} px/s`,
      },
    ],
  },
  {
    id: "vision-errors",
    group: "vision",
    groupLabel: "视觉输出",
    title: "视觉误差",
    source: "tracking",
    axis: {
      symmetric: true,
      includeZero: true,
      minimumSpan: 20,
      fallbackMinimum: -50,
      fallbackMaximum: 50,
    },
    reference: 0,
    series: [
      {
        key: "visualError",
        label: "目标误差",
        color: chartColors.error,
        pointColor: chartColors.errorPoint,
        formatValue: (value) => `${value.toFixed(1)} px`,
      },
      {
        key: "lateral",
        label: "横向偏差",
        color: chartColors.lateral,
        pointColor: chartColors.lateralPoint,
        formatValue: (value) => `${value.toFixed(1)} px`,
      },
    ],
  },
  {
    id: "quality",
    group: "vision",
    groupLabel: "视觉输出",
    title: "AI 置信度",
    source: "tracking",
    axis: { fixed: true, minimum: 0, maximum: 100, step: 25 },
    reference: null,
    formatAxis: (value) => `${Math.round(value)}%`,
    series: [
      {
        key: "quality",
        label: "置信度",
        color: chartColors.position,
        pointColor: chartColors.positionPoint,
        formatValue: (value) => `${value.toFixed(1)}%`,
      },
    ],
  },
  {
    id: "fps",
    group: "vision",
    groupLabel: "视觉输出",
    title: "识别帧率",
    source: "tracking",
    axis: {
      minimumSpan: 10,
      hardMinimum: 0,
      fallbackMinimum: 0,
      fallbackMaximum: 60,
    },
    reference: 30,
    series: [
      {
        key: "fps",
        label: "FPS",
        color: chartColors.fps,
        pointColor: chartColors.fpsPoint,
        formatValue: (value) => `${value.toFixed(2)} FPS`,
      },
    ],
  },
  {
    id: "timing",
    group: "vision",
    groupLabel: "视觉输出",
    title: "视觉链路耗时",
    source: "tracking",
    axis: {
      includeZero: true,
      minimumSpan: 20,
      hardMinimum: 0,
      fallbackMinimum: 0,
      fallbackMaximum: 100,
    },
    reference: null,
    series: [
      {
        key: "detect",
        label: "检测",
        color: chartColors.detect,
        pointColor: chartColors.detectPoint,
        formatValue: (value) => `${value.toFixed(1)} ms`,
      },
      {
        key: "latency",
        label: "图传",
        color: chartColors.latency,
        pointColor: chartColors.latencyPoint,
        formatValue: (value) => `${value.toFixed(1)} ms`,
      },
    ],
  },
  {
    id: "controller-position",
    group: "control",
    groupLabel: "STM32 控制反馈",
    title: "控制位置与误差",
    source: "feedback",
    axis: {
      includeZero: true,
      minimumSpan: 20,
      fallbackMinimum: -100,
      fallbackMaximum: 100,
    },
    reference: null,
    series: [
      {
        key: "feedbackPosition",
        label: "位置",
        color: chartColors.controllerPosition,
        pointColor: chartColors.controllerPositionPoint,
        formatValue: (value) => `${value.toFixed(1)} px`,
      },
      {
        key: "feedbackTarget",
        label: "目标",
        color: chartColors.target,
        pointColor: chartColors.targetPoint,
        formatValue: (value) => `${value.toFixed(1)} px`,
      },
      {
        key: "controlError",
        label: "误差",
        color: chartColors.error,
        pointColor: chartColors.errorPoint,
        formatValue: (value) => `${value.toFixed(1)} px`,
      },
    ],
  },
  {
    id: "controller-velocity",
    group: "control",
    groupLabel: "STM32 控制反馈",
    title: "控制速度",
    source: "feedback",
    axis: {
      symmetric: true,
      includeZero: true,
      minimumSpan: 40,
      fallbackMinimum: -100,
      fallbackMaximum: 100,
    },
    reference: 0,
    series: [
      {
        key: "feedbackVelocity",
        label: "速度",
        color: chartColors.controllerVelocity,
        pointColor: chartColors.controllerVelocityPoint,
        formatValue: (value) => `${value.toFixed(1)} px/s`,
      },
    ],
  },
  {
    id: "pid",
    group: "control",
    groupLabel: "STM32 控制反馈",
    title: "PID 分量",
    source: "feedback",
    axis: {
      symmetric: true,
      includeZero: true,
      minimumSpan: 20,
      fallbackMinimum: -100,
      fallbackMaximum: 100,
    },
    reference: 0,
    series: [
      {
        key: "pTerm",
        label: "P",
        color: chartColors.pTerm,
        pointColor: chartColors.pTermPoint,
        formatValue: (value) => value.toFixed(2),
      },
      {
        key: "iTerm",
        label: "I",
        color: chartColors.iTerm,
        pointColor: chartColors.iTermPoint,
        formatValue: (value) => value.toFixed(2),
      },
      {
        key: "dTerm",
        label: "D",
        color: chartColors.dTerm,
        pointColor: chartColors.dTermPoint,
        formatValue: (value) => value.toFixed(2),
      },
    ],
  },
  {
    id: "motor-command",
    group: "control",
    groupLabel: "STM32 控制反馈",
    title: "实际电机命令",
    source: "feedback",
    axis: {
      symmetric: true,
      includeZero: true,
      minimumSpan: 200,
      fallbackMinimum: -1000,
      fallbackMaximum: 1000,
    },
    reference: 0,
    series: [
      {
        key: "motorCommand",
        label: "命令",
        color: chartColors.motor,
        pointColor: chartColors.motorPoint,
        formatValue: (value) => value.toFixed(0),
      },
    ],
  },
  {
    id: "vision-age",
    group: "control",
    groupLabel: "STM32 控制反馈",
    title: "控制使用的视觉帧龄",
    source: "feedback",
    axis: {
      includeZero: true,
      minimumSpan: 20,
      hardMinimum: 0,
      fallbackMinimum: 0,
      fallbackMaximum: 100,
    },
    reference: 50,
    series: [
      {
        key: "visionAge",
        label: "帧龄",
        color: chartColors.visionAge,
        pointColor: chartColors.visionAgePoint,
        formatValue: (value) => `${value.toFixed(0)} ms`,
      },
    ],
  },
  {
    id: "feedback-gaps",
    group: "control",
    groupLabel: "STM32 控制反馈",
    title: "反馈序列丢失",
    source: "feedback",
    axis: {
      includeZero: true,
      minimumSpan: 2,
      hardMinimum: 0,
      fallbackMinimum: 0,
      fallbackMaximum: 4,
    },
    reference: 0,
    series: [
      {
        key: "sequenceGap",
        label: "丢失条数",
        color: chartColors.gap,
        pointColor: chartColors.gapPoint,
        formatValue: (value) => value.toFixed(0),
      },
    ],
  },
  {
    id: "motor-status",
    group: "control",
    groupLabel: "STM32 控制反馈",
    title: "HAL 电机状态",
    source: "feedback",
    axis: { fixed: true, minimum: 0, maximum: 3, step: 1 },
    reference: 0,
    formatAxis: (value) =>
      ["OK", "ERROR", "BUSY", "TIMEOUT"][Math.round(value)] || "—",
    series: [
      {
        key: "motorStatus",
        label: "状态",
        color: chartColors.motorStatus,
        pointColor: chartColors.motorStatusPoint,
        formatValue: (value) =>
          ["HAL_OK", "HAL_ERROR", "HAL_BUSY", "HAL_TIMEOUT"][
            Math.round(value)
          ] || String(value),
      },
    ],
  },
];

const chartDefinitionById = new Map(
  chartDefinitions.map((definition) => [definition.id, definition]),
);
const chartPresets = {
  default: ["position", "controller-position", "pid", "motor-command"],
  vision: chartDefinitions
    .filter((definition) => definition.group === "vision")
    .map((definition) => definition.id),
  control: chartDefinitions
    .filter((definition) => definition.group === "control")
    .map((definition) => definition.id),
  all: chartDefinitions.map((definition) => definition.id),
};
const CHART_SELECTION_STORAGE_KEY = "pipe-ball-visible-charts-v1";
let selectedChartIds = loadChartSelection();

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

function modelIdFromPath(path) {
  const value = String(path || "");
  const match = value.match(
    /\/root\/models\/maixhub\/([^/]+)\/model_([^/]+)\.mud$/,
  );
  if (match && match[1] === match[2]) return match[1];
  const filename = value.split("/").pop();
  return filename ? filename.replace(/\.mud$/i, "") : "—";
}

function modelDisplayName(path) {
  const id = modelIdFromPath(path);
  return id === "—" ? "AI 模型未上报" : `MaixHub ${id}`;
}

function setConfidencePercent(value) {
  const percent = clamp(Math.round(Number(value) || 1), 1, 99);
  ui.confidenceSlider.value = String(percent);
  ui.confidenceOutput.value = `${percent}%`;
  ui.confidenceOutput.textContent = `${percent}%`;
  $$("[data-confidence-preset]").forEach((button) => {
    button.classList.toggle(
      "active",
      Number(button.dataset.confidencePreset) === percent,
    );
  });
}

function syncModelOptions(models, currentPath) {
  const installed = Array.isArray(models)
    ? models.filter((item) => item && item.path)
    : [];
  const paths = installed.map((item) => String(item.path));
  if (currentPath && !paths.includes(String(currentPath))) {
    installed.unshift({
      id: modelIdFromPath(currentPath),
      name: String(currentPath).split("/").pop(),
      path: String(currentPath),
      currentOnly: true,
    });
  }
  const fingerprint = JSON.stringify(
    [
      ["current", String(currentPath || "")],
      ...installed.map((item) => [item.id, item.path, item.bytes]),
    ],
  );
  if (fingerprint !== modelCatalogFingerprint) {
    ui.modelSelect.replaceChildren();
    if (!currentPath) {
      const placeholder = document.createElement("option");
      placeholder.value = "";
      placeholder.textContent = "等待设备上报当前模型";
      placeholder.selected = true;
      ui.modelSelect.append(placeholder);
    }
    for (const model of installed) {
      const option = document.createElement("option");
      option.value = String(model.path);
      const size =
        Number.isFinite(Number(model.bytes)) && Number(model.bytes) > 0
          ? ` · ${(Number(model.bytes) / 1024 / 1024).toFixed(1)} MB`
          : "";
      option.textContent =
        `MaixHub ${model.id || modelIdFromPath(model.path)}` +
        `${size}${model.currentOnly ? " · 当前" : ""}`;
      ui.modelSelect.append(option);
    }
    if (!installed.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "设备上未发现 MaixHub 模型";
      ui.modelSelect.append(option);
    }
    modelCatalogFingerprint = fingerprint;
  }
  ui.modelSelect.value = currentPath || "";
  text(
    "model-count",
    installed.length ? `${installed.length} 个已安装` : "未发现模型",
  );
}

function syncAIForm(config, models) {
  const modelPath = String(config.model || "");
  syncModelOptions(models, modelPath);
  setConfidencePercent(
    Number.isFinite(Number(config.confidence))
      ? Number(config.confidence) * 100
      : 13,
  );
  ui.targetPercent.value = Number.isFinite(Number(config.target_position))
    ? String(Math.round(Number(config.target_position) * 100))
    : "50";
  ui.iouPercent.value = Number.isFinite(Number(config.iou))
    ? String(Math.round(Number(config.iou) * 100))
    : "45";
  ui.coastFrames.value = Number.isFinite(Number(config.coast_frames))
    ? String(Math.round(Number(config.coast_frames)))
    : "2";
}

function appendTelemetrySample(
  tracking,
  video = {},
  config = {},
  sampleTimeMs = Date.now(),
) {
  if (!tracking || tracking.seq == null) return;

  const session = String(tracking.session || "");
  const sequence = Number(tracking.seq);
  const key = `${session}:${tracking.seq}`;
  const previousLatestTime =
    telemetryHistory.length || feedbackHistory.length
      ? chartLatestTime()
      : null;
  if (lastTelemetrySession && session !== lastTelemetrySession) {
    telemetryHistory.length = 0;
    chartPanOffsetMs = 0;
    chartHoverTimeMs = null;
    lastHistoryPruneMs = 0;
    lastTelemetrySeq = null;
    updateChartLiveState();
  }
  if (session === lastTelemetrySession) {
    if (
      Number.isFinite(sequence) &&
      lastTelemetrySeq != null &&
      sequence <= lastTelemetrySeq
    ) {
      return;
    }
    if (!Number.isFinite(sequence) && key === lastTelemetryKey) return;
  }
  lastTelemetrySession = session;
  lastTelemetryKey = key;
  lastTelemetrySeq = Number.isFinite(sequence) ? sequence : null;

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
    visualError: valid ? finiteOrNull(tracking.error_px) : null,
    lateral: valid ? finiteOrNull(tracking.lateral_px) : null,
    quality: finiteOrNull(tracking.quality),
    measured: Boolean(tracking.measured),
    valid,
    coasting: Boolean(tracking.coasting),
    target: finiteOrNull(config.target_position),
  });
  if (
    chartPanOffsetMs > 0 &&
    previousLatestTime != null &&
    now > previousLatestTime
  ) {
    chartPanOffsetMs += now - previousLatestTime;
  }

  pruneHistories(now);
  if (chartPanOffsetMs === 0) chartsDirty = true;
}

function appendFeedbackSample(feedback, sampleTimeMs = Date.now()) {
  if (!feedback || feedback.seq == null) return;
  const session = String(feedback.session || "");
  const sequence = Number(
    feedback.transport_seq == null
      ? feedback.seq
      : feedback.transport_seq,
  );
  if (lastFeedbackSession && session !== lastFeedbackSession) {
    feedbackHistory.length = 0;
    lastFeedbackSeq = null;
    chartPanOffsetMs = 0;
    chartHoverTimeMs = null;
    updateChartLiveState();
  }
  if (
    session === lastFeedbackSession &&
    Number.isFinite(sequence) &&
    lastFeedbackSeq != null &&
    sequence <= lastFeedbackSeq
  ) {
    return;
  }

  const previousLatestTime =
    telemetryHistory.length || feedbackHistory.length
      ? chartLatestTime()
      : null;
  lastFeedbackSession = session;
  lastFeedbackSeq = Number.isFinite(sequence) ? sequence : null;
  const now = Number.isFinite(Number(sampleTimeMs))
    ? Number(sampleTimeMs)
    : Date.now();
  const position = finiteOrNull(feedback.position_px);
  const controlError = finiteOrNull(feedback.control_error_px);
  feedbackHistory.push({
    t: now,
    feedbackPosition: position,
    feedbackTarget:
      position == null || controlError == null
        ? null
        : position + controlError,
    feedbackVelocity: finiteOrNull(feedback.velocity_px_s),
    controlError,
    pTerm: finiteOrNull(feedback.p_term),
    iTerm: finiteOrNull(feedback.i_term),
    dTerm: finiteOrNull(feedback.d_term),
    motorCommand: finiteOrNull(feedback.motor_command),
    visionAge: finiteOrNull(feedback.vision_age_ms),
    sequenceGap: finiteOrNull(feedback.seq_gap),
    motorStatus: finiteOrNull(feedback.motor_status),
  });
  if (
    chartPanOffsetMs > 0 &&
    previousLatestTime != null &&
    now > previousLatestTime
  ) {
    chartPanOffsetMs += now - previousLatestTime;
  }
  pruneHistories(now);
  if (chartPanOffsetMs === 0) chartsDirty = true;
}

function pruneHistories(now) {
  if (now - lastHistoryPruneMs < 5000) return;
  const oldest = now - MAX_HISTORY_MS;
  for (const history of [telemetryHistory, feedbackHistory]) {
    let removeCount = 0;
    while (
      removeCount < history.length &&
      history[removeCount].t < oldest
    ) {
      removeCount += 1;
    }
    if (removeCount > 0) history.splice(0, removeCount);
  }
  if (chartPanOffsetMs > 0) chartsDirty = true;
  lastHistoryPruneMs = now;
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
  appendHistory = true,
) {
  if (appendHistory) {
    appendTelemetrySample(tracking, video, config, sampleTimeMs);
    renderSimulator(tracking, config);
  }

  text("metric-fps", formatNumber(tracking.fps, 1));
  text(
    "metric-quality",
    Number.isFinite(Number(tracking.quality))
      ? Number(tracking.quality).toFixed(1)
      : "—",
  );
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
      const feedback = sample.feedback;
      if (!tracking && !feedback) return;
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
      const sampleTimeMs = Number(sample.host_epoch_ns) / 1_000_000;
      if (tracking) {
        monitorState.tracking = tracking;
        renderLiveTelemetry(
          tracking,
          monitorState.video || {},
          monitorState.config || {},
          sampleTimeMs,
        );
      }
      if (feedback) {
        monitorState.feedback = feedback;
        appendFeedbackSample(feedback, sampleTimeMs);
      }
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

function chartLatestTime() {
  const latest = [];
  if (telemetryHistory.length) {
    latest.push(telemetryHistory[telemetryHistory.length - 1].t);
  }
  if (feedbackHistory.length) {
    latest.push(feedbackHistory[feedbackHistory.length - 1].t);
  }
  return latest.length ? Math.max(...latest) : Date.now();
}

function chartOldestTime() {
  const oldest = [];
  if (telemetryHistory.length) oldest.push(telemetryHistory[0].t);
  if (feedbackHistory.length) oldest.push(feedbackHistory[0].t);
  return oldest.length ? Math.min(...oldest) : chartLatestTime();
}

function clampChartPan(value, windowMs = historyWindowMs) {
  if (!telemetryHistory.length && !feedbackHistory.length) return 0;
  const availableMs = Math.max(
    0,
    chartLatestTime() - chartOldestTime() - windowMs,
  );
  return clamp(Number(value) || 0, 0, availableMs);
}

function chartViewRange() {
  chartPanOffsetMs = clampChartPan(chartPanOffsetMs);
  const endTime = chartLatestTime() - chartPanOffsetMs;
  return {
    startTime: endTime - historyWindowMs,
    endTime,
  };
}

function lowerBoundSample(history, time) {
  let low = 0;
  let high = history.length;
  while (low < high) {
    const middle = (low + high) >> 1;
    if (history[middle].t < time) low = middle + 1;
    else high = middle;
  }
  return low;
}

function historyForCurrentWindow(history, range) {
  if (!history.length) return [];
  let startIndex = lowerBoundSample(history, range.startTime);
  let endIndex = lowerBoundSample(history, range.endTime);
  if (startIndex > 0) startIndex -= 1;
  if (
    endIndex < history.length &&
    history[endIndex].t <= range.endTime
  ) {
    endIndex += 1;
  }
  return history.slice(startIndex, endIndex);
}

function nearestSample(samples, time) {
  if (!samples.length || time == null) return null;
  let low = 0;
  let high = samples.length - 1;
  while (low < high) {
    const middle = (low + high) >> 1;
    if (samples[middle].t < time) low = middle + 1;
    else high = middle;
  }
  if (low === 0) return samples[0];
  const before = samples[low - 1];
  const after = samples[low];
  if (!after) return before;
  return time - before.t <= after.t - time ? before : after;
}

function formatTimeLabel(time, includeMilliseconds = false) {
  const date = new Date(time);
  const base = [
    String(date.getHours()).padStart(2, "0"),
    String(date.getMinutes()).padStart(2, "0"),
    String(date.getSeconds()).padStart(2, "0"),
  ].join(":");
  return includeMilliseconds
    ? `${base}.${String(date.getMilliseconds()).padStart(3, "0")}`
    : base;
}

function niceAxisStep(span, targetIntervals = 5) {
  const roughStep =
    Math.max(Number.EPSILON, span) /
    Math.max(1, targetIntervals);
  const magnitude = 10 ** Math.floor(Math.log10(roughStep));
  const normalized = roughStep / magnitude;
  let multiplier = 10;
  if (normalized <= 1) multiplier = 1;
  else if (normalized <= 2) multiplier = 2;
  else if (normalized <= 2.5) multiplier = 2.5;
  else if (normalized <= 7.5) multiplier = 5;
  return multiplier * magnitude;
}

function buildAxisTicks(minimum, maximum, step) {
  const ticks = [];
  const count = Math.min(
    10,
    Math.max(1, Math.round((maximum - minimum) / step)),
  );
  for (let index = 0; index <= count; index += 1) {
    const value = minimum + step * index;
    ticks.push(Math.abs(value) < step * 1e-6 ? 0 : value);
  }
  return ticks;
}

function resolveChartAxis(samples, options) {
  const axis = options.axis || {};
  if (axis.fixed) {
    const minimum = Number(axis.minimum);
    const maximum = Number(axis.maximum);
    const step =
      Number(axis.step) ||
      niceAxisStep(maximum - minimum);
    return {
      minimum,
      maximum,
      step,
      ticks: buildAxisTicks(minimum, maximum, step),
    };
  }

  const values = [];
  for (const sample of samples) {
    for (const series of options.series) {
      const value = sample[series.key];
      if (Number.isFinite(value)) values.push(value);
    }
  }
  if (!values.length) {
    const minimum = Number(axis.fallbackMinimum ?? 0);
    const maximum = Number(axis.fallbackMaximum ?? 1);
    const step = niceAxisStep(maximum - minimum);
    return {
      minimum,
      maximum,
      step,
      ticks: buildAxisTicks(minimum, maximum, step),
    };
  }

  if (axis.includeReference && Number.isFinite(options.reference)) {
    values.push(options.reference);
  }
  let minimum = Math.min(...values);
  let maximum = Math.max(...values);
  const minimumSpan = Math.max(
    Number.EPSILON,
    Number(axis.minimumSpan) || 1,
  );

  if (axis.symmetric) {
    const limit = Math.max(
      Math.abs(minimum),
      Math.abs(maximum),
      minimumSpan / 2,
    );
    minimum = -limit;
    maximum = limit;
  } else if (maximum - minimum < minimumSpan) {
    const center = (minimum + maximum) / 2;
    minimum = center - minimumSpan / 2;
    maximum = center + minimumSpan / 2;
  }

  const padding =
    (maximum - minimum) *
    Math.max(0, Number(axis.paddingFraction) || 0.08);
  minimum -= padding;
  maximum += padding;
  if (axis.includeZero) {
    minimum = Math.min(0, minimum);
    maximum = Math.max(0, maximum);
  }
  if (Number.isFinite(axis.hardMinimum)) {
    minimum = Math.max(Number(axis.hardMinimum), minimum);
  }
  if (Number.isFinite(axis.hardMaximum)) {
    maximum = Math.min(Number(axis.hardMaximum), maximum);
  }

  const step = niceAxisStep(maximum - minimum);
  if (axis.symmetric) {
    const limit =
      Math.ceil(
        Math.max(Math.abs(minimum), Math.abs(maximum)) / step,
      ) * step;
    minimum = -limit;
    maximum = limit;
  } else {
    minimum = Math.floor(minimum / step) * step;
    maximum = Math.ceil(maximum / step) * step;
  }
  if (Number.isFinite(axis.hardMinimum)) {
    minimum = Math.max(Number(axis.hardMinimum), minimum);
  }
  if (Number.isFinite(axis.hardMaximum)) {
    maximum = Math.min(Number(axis.hardMaximum), maximum);
  }
  if (maximum <= minimum) maximum = minimum + step;
  return {
    minimum,
    maximum,
    step,
    ticks: buildAxisTicks(minimum, maximum, step),
  };
}

function formatAxisNumber(value, step) {
  if (Math.abs(value) < step * 1e-6) return "0";
  let decimals = 0;
  if (step < 1) {
    decimals = Math.min(
      3,
      Math.max(1, Math.ceil(-Math.log10(step))),
    );
  } else if (!Number.isInteger(step)) {
    decimals = 1;
  }
  const formatted = value.toFixed(decimals);
  return decimals
    ? formatted.replace(/\.?0+$/, "")
    : formatted;
}

function buildSeriesPoints(samples, key, maxBuckets) {
  const groupSize = Math.max(
    1,
    Math.ceil(samples.length / Math.max(1, maxBuckets)),
  );
  if (groupSize === 1) {
    return {
      raw: true,
      points: samples.map((sample) => ({
        sample,
        value: sample[key],
      })),
    };
  }

  const points = [];
  for (let start = 0; start < samples.length; start += groupSize) {
    const end = Math.min(samples.length, start + groupSize);
    let minimum = null;
    let maximum = null;
    for (let index = start; index < end; index += 1) {
      const sample = samples[index];
      const value = sample[key];
      if (value == null || !Number.isFinite(value)) continue;
      if (minimum == null || value < minimum.value) {
        minimum = { sample, value };
      }
      if (maximum == null || value > maximum.value) {
        maximum = { sample, value };
      }
    }
    if (minimum == null) {
      points.push({
        sample: samples[start + Math.floor((end - start) / 2)],
        value: null,
      });
    } else if (minimum.sample.t <= maximum.sample.t) {
      points.push(minimum);
      if (maximum !== minimum) points.push(maximum);
    } else {
      points.push(maximum);
      points.push(minimum);
    }
  }
  return { raw: false, points };
}

function drawSeries(
  context,
  samples,
  series,
  mapX,
  mapY,
  plotWidth,
) {
  const display = buildSeriesPoints(
    samples,
    series.key,
    Math.ceil(plotWidth * 1.5),
  );
  context.save();
  context.strokeStyle = series.color;
  context.lineWidth = 1.45;
  context.lineJoin = "round";
  context.lineCap = "round";
  context.beginPath();
  let drawing = false;
  for (const point of display.points) {
    if (point.value == null || !Number.isFinite(point.value)) {
      drawing = false;
      continue;
    }
    const x = mapX(point.sample.t);
    const y = mapY(point.value);
    if (drawing) context.lineTo(x, y);
    else {
      context.moveTo(x, y);
      drawing = true;
    }
  }
  context.stroke();

  if (display.raw) {
    const radius =
      samples.length < plotWidth * 0.7
        ? 2.05
        : samples.length < plotWidth * 1.2
          ? 1.45
          : 0.9;
    for (const point of display.points) {
      if (point.value == null || !Number.isFinite(point.value)) continue;
      const coast =
        series.key === "position" && point.sample.coasting;
      context.fillStyle = coast
        ? chartColors.coast
        : series.pointColor;
      context.beginPath();
      context.arc(
        mapX(point.sample.t),
        mapY(point.value),
        radius,
        0,
        Math.PI * 2,
      );
      context.fill();
    }
  }
  context.restore();
}

function drawChartOverlay(
  context,
  samples,
  options,
  padding,
  plotWidth,
  plotHeight,
  mapX,
  mapY,
) {
  const hovered = nearestSample(samples, chartHoverTimeMs);
  if (!hovered) return;
  const x = mapX(hovered.t);
  if (
    x < padding.left ||
    x > padding.left + plotWidth
  ) {
    return;
  }

  context.save();
  context.setLineDash([3, 3]);
  context.strokeStyle = chartColors.crosshair;
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(x, padding.top);
  context.lineTo(x, padding.top + plotHeight);
  context.stroke();

  const primaryValue = hovered[options.series[0].key];
  if (primaryValue != null && Number.isFinite(primaryValue)) {
    const y = mapY(primaryValue);
    context.beginPath();
    context.moveTo(padding.left, y);
    context.lineTo(padding.left + plotWidth, y);
    context.stroke();
  }
  context.setLineDash([]);

  const rows = [];
  for (const series of options.series) {
    const value = hovered[series.key];
    if (value == null || !Number.isFinite(value)) continue;
    rows.push({
      color: series.pointColor,
      text: `${series.label} ${series.formatValue(value)}`,
      value,
    });
    context.fillStyle =
      series.key === "position" && hovered.coasting
        ? chartColors.coast
        : series.pointColor;
    context.strokeStyle = "#07100e";
    context.lineWidth = 1;
    context.beginPath();
    context.arc(x, mapY(value), 3.2, 0, Math.PI * 2);
    context.fill();
    context.stroke();
  }

  const timeText = formatTimeLabel(hovered.t, true);
  context.font = '9px "Cascadia Mono", Consolas, monospace';
  let boxWidth = context.measureText(timeText).width + 18;
  for (const row of rows) {
    boxWidth = Math.max(
      boxWidth,
      context.measureText(row.text).width + 29,
    );
  }
  const boxHeight = 23 + rows.length * 14;
  const boxX =
    x < padding.left + plotWidth / 2
      ? padding.left + plotWidth - boxWidth - 6
      : padding.left + 6;
  const boxY = padding.top + 6;

  context.fillStyle = "rgba(11, 16, 21, 0.94)";
  context.strokeStyle = "rgba(104, 121, 134, 0.55)";
  context.lineWidth = 1;
  context.fillRect(boxX, boxY, boxWidth, boxHeight);
  context.strokeRect(boxX + 0.5, boxY + 0.5, boxWidth - 1, boxHeight - 1);
  context.fillStyle = "#dfe8eb";
  context.textAlign = "left";
  context.fillText(timeText, boxX + 8, boxY + 14);
  rows.forEach((row, index) => {
    const rowY = boxY + 27 + index * 14;
    context.fillStyle = row.color;
    context.beginPath();
    context.arc(boxX + 10, rowY - 3, 2.5, 0, Math.PI * 2);
    context.fill();
    context.fillStyle = "#b9c4c9";
    context.fillText(row.text, boxX + 18, rowY);
  });
  context.restore();
}

function drawWaveChart(canvas, samples, range, options) {
  const { context, width, height } = prepareCanvas(canvas);
  const padding = { left: 35, right: 9, top: 7, bottom: 19 };
  const plotWidth = Math.max(1, width - padding.left - padding.right);
  const plotHeight = Math.max(1, height - padding.top - padding.bottom);
  const axis = resolveChartAxis(samples, options);
  const minimum = axis.minimum;
  const maximum = axis.maximum;
  canvas.dataset.axisMinimum = String(minimum);
  canvas.dataset.axisMaximum = String(maximum);
  canvas.dataset.axisStep = String(axis.step);
  const mapX = (time) =>
    padding.left +
    ((time - range.startTime) / historyWindowMs) * plotWidth;
  const mapY = (value) =>
    padding.top +
    (1 - (value - minimum) / (maximum - minimum)) * plotHeight;

  context.save();
  context.strokeStyle = chartColors.grid;
  context.lineWidth = 1;
  for (const tick of axis.ticks) {
    const y = mapY(tick);
    context.beginPath();
    context.moveTo(padding.left, y);
    context.lineTo(padding.left + plotWidth, y);
    context.stroke();
  }
  for (let column = 0; column <= 4; column += 1) {
    const x = padding.left + (plotWidth * column) / 4;
    context.beginPath();
    context.moveTo(x, padding.top);
    context.lineTo(x, padding.top + plotHeight);
    context.stroke();
  }
  context.fillStyle = chartColors.axis;
  context.font = '9px "Cascadia Mono", Consolas, monospace';
  context.textAlign = "right";
  context.textBaseline = "middle";
  for (const tick of axis.ticks) {
    const y = clamp(
      mapY(tick),
      padding.top + 4,
      padding.top + plotHeight - 4,
    );
    context.fillText(
      options.formatAxis(tick, axis.step),
      padding.left - 5,
      y,
    );
  }
  context.textBaseline = "alphabetic";
  context.textAlign = "left";
  context.fillText(
    formatTimeLabel(range.startTime),
    padding.left,
    height - 4,
  );
  context.textAlign = "center";
  context.fillText(
    `${(historyWindowMs / 1000).toFixed(historyWindowMs < 10000 ? 1 : 0)}s`,
    padding.left + plotWidth / 2,
    height - 4,
  );
  context.textAlign = "right";
  context.fillStyle =
    chartPanOffsetMs === 0 ? chartColors.position : chartColors.axis;
  context.fillText(
    chartPanOffsetMs === 0
      ? "LIVE"
      : formatTimeLabel(range.endTime),
    padding.left + plotWidth,
    height - 4,
  );
  context.restore();

  context.save();
  context.beginPath();
  context.rect(
    padding.left,
    padding.top,
    plotWidth,
    plotHeight,
  );
  context.clip();
  if (
    options.reference >= minimum &&
    options.reference <= maximum
  ) {
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
  }
  for (const series of options.series) {
    drawSeries(
      context,
      samples,
      series,
      mapX,
      mapY,
      plotWidth,
    );
  }
  context.restore();
  drawChartOverlay(
    context,
    samples,
    options,
    padding,
    plotWidth,
    plotHeight,
    mapX,
    mapY,
  );
}

function renderCharts() {
  const range = chartViewRange();
  for (const chartId of selectedChartIds) {
    const definition = chartDefinitionById.get(chartId);
    const canvas = document.querySelector(
      `canvas[data-chart-id="${chartId}"]`,
    );
    if (!definition || !canvas) continue;
    const history =
      definition.source === "feedback"
        ? feedbackHistory
        : telemetryHistory;
    const samples = historyForCurrentWindow(history, range);
    const reference =
      typeof definition.reference === "function"
        ? definition.reference(samples)
        : definition.reference;
    drawWaveChart(canvas, samples, range, {
      ...definition,
      reference,
      formatAxis: definition.formatAxis || formatAxisNumber,
    });
    text(
      `chart-value-${chartId}`,
      formatLatestChartValue(definition, samples),
    );
  }
}

function formatLatestChartValue(definition, samples) {
  for (let index = samples.length - 1; index >= 0; index -= 1) {
    const sample = samples[index];
    const values = definition.series
      .map((series) => {
        const value = sample[series.key];
        return Number.isFinite(value)
          ? {
              label: series.label,
              value: series.formatValue(value),
            }
          : null;
      })
      .filter(Boolean);
    if (!values.length) continue;
    if (values.length === 1) return values[0].value;
    return values
      .map((item) => `${item.label} ${item.value}`)
      .join(" · ");
  }
  return "—";
}

function loadChartSelection() {
  try {
    const stored = JSON.parse(
      localStorage.getItem(CHART_SELECTION_STORAGE_KEY) || "null",
    );
    if (Array.isArray(stored)) {
      const valid = stored.filter((id) =>
        chartDefinitionById.has(String(id)),
      );
      if (valid.length) return [...new Set(valid)];
    }
  } catch (_error) {
    // Fall back to the operator-focused default below.
  }
  return [...chartPresets.default];
}

function saveChartSelection() {
  try {
    localStorage.setItem(
      CHART_SELECTION_STORAGE_KEY,
      JSON.stringify(selectedChartIds),
    );
  } catch (_error) {
    // The charts still work when browser storage is unavailable.
  }
}

function setChartPickerOpen(open) {
  const nextOpen = Boolean(open);
  ui.chartPicker.hidden = !nextOpen;
  ui.chartPickerToggle.setAttribute(
    "aria-expanded",
    String(nextOpen),
  );
}

function setChartSelection(ids) {
  const requested = new Set(ids.map(String));
  const next = chartDefinitions
    .map((definition) => definition.id)
    .filter((id) => requested.has(id));
  if (!next.length) {
    showToast("至少保留一个波形", true);
    return;
  }
  selectedChartIds = next;
  saveChartSelection();
  renderChartPanels();
}

function renderChartOptions() {
  ui.chartOptions.replaceChildren();
  for (const group of ["vision", "control"]) {
    const definitions = chartDefinitions.filter(
      (definition) => definition.group === group,
    );
    const section = document.createElement("fieldset");
    const legend = document.createElement("legend");
    legend.textContent = definitions[0].groupLabel;
    section.append(legend);
    for (const definition of definitions) {
      const label = document.createElement("label");
      const input = document.createElement("input");
      input.type = "checkbox";
      input.value = definition.id;
      input.checked = selectedChartIds.includes(definition.id);
      input.addEventListener("change", () => {
        const next = new Set(selectedChartIds);
        if (input.checked) next.add(definition.id);
        else next.delete(definition.id);
        if (!next.size) {
          input.checked = true;
          showToast("至少保留一个波形", true);
          return;
        }
        setChartSelection([...next]);
      });
      const copy = document.createElement("span");
      copy.textContent = definition.title;
      label.append(input, copy);
      section.append(label);
    }
    ui.chartOptions.append(section);
  }
}

function createChartPanel(definition) {
  const panel = document.createElement("figure");
  panel.className = "wave-panel";
  panel.dataset.chartId = definition.id;

  const caption = document.createElement("figcaption");
  const heading = document.createElement("span");
  heading.className = "chart-panel-heading";
  const title = document.createElement("b");
  title.textContent = definition.title;
  const legends = document.createElement("span");
  legends.className = "series-legends";
  for (const series of definition.series) {
    const legend = document.createElement("span");
    const dot = document.createElement("i");
    dot.className = "legend-dot";
    dot.style.background = series.color;
    legend.append(dot, document.createTextNode(series.label));
    legends.append(legend);
  }
  heading.append(title, legends);

  const actions = document.createElement("span");
  actions.className = "wave-panel-actions";
  const value = document.createElement("strong");
  value.id = `chart-value-${definition.id}`;
  value.textContent = "—";
  const expand = document.createElement("button");
  expand.className = "chart-expand";
  expand.type = "button";
  expand.dataset.chartExpand = "";
  expand.setAttribute(
    "aria-label",
    `全屏查看${definition.title}波形`,
  );
  expand.title = "全屏查看";
  expand.innerHTML =
    '<svg class="expand-icon" viewBox="0 0 16 16" aria-hidden="true"><path d="M2.5 6V2.5H6M10 2.5h3.5V6M13.5 10v3.5H10M6 13.5H2.5V10"/></svg>' +
    '<svg class="collapse-icon" viewBox="0 0 16 16" aria-hidden="true"><path d="M6 2.5V6H2.5M13.5 6H10V2.5M10 13.5V10h3.5M2.5 10H6v3.5"/></svg>';
  actions.append(value, expand);
  caption.append(heading, actions);

  const canvas = document.createElement("canvas");
  canvas.dataset.chartId = definition.id;
  canvas.setAttribute(
    "aria-label",
    `${definition.title}历史波形`,
  );
  panel.append(caption, canvas);
  installChartInteraction(canvas);
  if (chartResizeObserver) chartResizeObserver.observe(canvas);
  expand.setAttribute("aria-expanded", "false");
  expand.addEventListener("click", () => setExpandedChart(panel));
  return panel;
}

function renderChartPanels() {
  if (expandedChartPanel) setExpandedChart(null);
  ui.waveGrid.replaceChildren();
  for (const chartId of selectedChartIds) {
    const definition = chartDefinitionById.get(chartId);
    if (definition) {
      ui.waveGrid.append(createChartPanel(definition));
    }
  }
  ui.waveGrid.style.setProperty(
    "--chart-count",
    String(selectedChartIds.length),
  );
  ui.chartSelectionCount.textContent =
    `${selectedChartIds.length} / ${chartDefinitions.length}`;
  renderChartOptions();
  chartsDirty = true;
}

function updateChartLiveState() {
  const liveButton = $("#chart-live");
  const live = chartPanOffsetMs === 0;
  liveButton.classList.toggle("active", live);
  liveButton.textContent = live ? "LIVE" : "回到实时";
}

function chartAnimationFrame() {
  if (chartsDirty) {
    renderCharts();
    chartsDirty = false;
  }
  window.requestAnimationFrame(chartAnimationFrame);
}

function chartPointerGeometry(canvas, clientX) {
  const rect = canvas.getBoundingClientRect();
  const left = 35;
  const width = Math.max(1, rect.width - left - 9);
  const ratio = clamp(
    (clientX - rect.left - left) / width,
    0,
    1,
  );
  return { ratio, width };
}

function updateChartWindowButtons() {
  $$("[data-window-seconds]").forEach((button) => {
    const buttonWindowMs =
      Number(button.dataset.windowSeconds) * 1000;
    button.classList.toggle(
      "active",
      Math.abs(buttonWindowMs - historyWindowMs) < 1,
    );
  });
}

function returnChartsToLive() {
  chartPanOffsetMs = 0;
  chartHoverTimeMs = null;
  updateChartLiveState();
  chartsDirty = true;
}

function installChartInteraction(canvas) {
  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      if (!telemetryHistory.length && !feedbackHistory.length) return;
      const geometry = chartPointerGeometry(
        canvas,
        event.clientX,
      );
      const oldRange = chartViewRange();
      const anchorTime =
        oldRange.startTime +
        geometry.ratio * historyWindowMs;
      const zoomFactor = Math.exp(event.deltaY * 0.0012);
      const nextWindowMs = clamp(
        historyWindowMs * zoomFactor,
        MIN_CHART_WINDOW_MS,
        MAX_CHART_WINDOW_MS,
      );
      const nextEndTime =
        anchorTime + (1 - geometry.ratio) * nextWindowMs;
      historyWindowMs = nextWindowMs;
      chartPanOffsetMs = clampChartPan(
        chartLatestTime() - nextEndTime,
        historyWindowMs,
      );
      chartHoverTimeMs = anchorTime;
      updateChartWindowButtons();
      updateChartLiveState();
      chartsDirty = true;
    },
    { passive: false },
  );

  canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    chartDragging = true;
    chartDragStartX = event.clientX;
    chartDragStartPanMs = chartPanOffsetMs;
    canvas.classList.add("dragging");
    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener("pointermove", (event) => {
    if (!telemetryHistory.length && !feedbackHistory.length) return;
    const geometry = chartPointerGeometry(
      canvas,
      event.clientX,
    );
    if (chartDragging) {
      const deltaX = event.clientX - chartDragStartX;
      chartPanOffsetMs = clampChartPan(
        chartDragStartPanMs +
          (deltaX / geometry.width) * historyWindowMs,
      );
      updateChartLiveState();
    }
    const range = chartViewRange();
    chartHoverTimeMs =
      range.startTime + geometry.ratio * historyWindowMs;
    chartsDirty = true;
  });

  const finishDrag = (event) => {
    if (!chartDragging) return;
    chartDragging = false;
    canvas.classList.remove("dragging");
    if (canvas.hasPointerCapture(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
  };
  canvas.addEventListener("pointerup", finishDrag);
  canvas.addEventListener("pointercancel", finishDrag);
  canvas.addEventListener("pointerleave", () => {
    if (chartDragging) return;
    chartHoverTimeMs = null;
    chartsDirty = true;
  });
  canvas.addEventListener("dblclick", returnChartsToLive);
}

function setExpandedChart(panel) {
  const nextPanel =
    panel && panel !== expandedChartPanel ? panel : null;
  if (expandedChartPanel) {
    const previousButton =
      expandedChartPanel.querySelector("[data-chart-expand]");
    expandedChartPanel.classList.remove("expanded");
    if (previousButton) {
      previousButton.setAttribute("aria-expanded", "false");
      previousButton.setAttribute(
        "aria-label",
        previousButton.dataset.openLabel ||
          "全屏查看波形",
      );
      previousButton.title = "全屏查看";
    }
  }

  expandedChartPanel = nextPanel;
  document.body.classList.toggle(
    "chart-expanded",
    Boolean(expandedChartPanel),
  );
  if (expandedChartPanel) {
    const button =
      expandedChartPanel.querySelector("[data-chart-expand]");
    expandedChartPanel.classList.add("expanded");
    if (button) {
      button.dataset.openLabel =
        button.dataset.openLabel ||
        button.getAttribute("aria-label") ||
        "全屏查看波形";
      button.setAttribute("aria-label", "退出波形全屏");
      button.setAttribute("aria-expanded", "true");
      button.title = "退出全屏";
      button.focus({ preventScroll: true });
    }
  }

  chartsDirty = true;
  window.requestAnimationFrame(() => {
    chartsDirty = true;
  });
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
  const modelPath = String(config.model || "");
  const modelName = modelDisplayName(modelPath);

  ensureTelemetryStream(monitor);
  // The 300 ms state poll can lag several packets behind the 30 Hz SSE
  // stream. It refreshes status cards only; feeding it into the chart or the
  // position simulator would periodically insert an older position between
  // two live samples.
  renderLiveTelemetry(tracking, video, config, Date.now(), false);

  text("active-model-chip", `AI · ${modelIdFromPath(modelPath)}`);
  text("status-model", modelIdFromPath(modelPath));
  text(
    "status-confidence",
    Number.isFinite(Number(tracking.quality))
      ? `${Number(tracking.quality).toFixed(1)}%`
      : "—",
  );
  text(
    "status-threshold",
    Number.isFinite(Number(config.valid_confidence))
      ? `${(Number(config.valid_confidence) * 100).toFixed(0)}%`
      : "—",
  );
  text("status-measured", boolLabel(tracking.measured, "检测到", "未检测"));
  text("status-valid", boolLabel(tracking.valid, "有效", "无效"));
  text("status-pipe", "固定安装标定");
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
  text("system-model", modelIdFromPath(modelPath));
  text("system-process", running ? `PID ${device.process?.pid || "—"}` : "已停止");
  text("system-release", device.current_release || "—");
  text("system-hash", device.source_hash ? `${device.source_hash.slice(0, 16)}…` : "—");
  text("current-model-name", modelName);
  text("current-model-path", modelPath || "—");

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
    if (
      button.classList.contains("tab") ||
      button.id === "inspector-toggle" ||
      button.id === "inspector-close" ||
      button.id === "chart-live" ||
      button.classList.contains("chart-expand") ||
      button.closest(".chart-selector") ||
      button.dataset.windowSeconds
    ) {
      return;
    }
    button.disabled = operationRunning;
  });

  const aiConfigReady = Boolean(monitor?.config?.model);
  for (const control of ui.form.elements) {
    control.disabled = operationRunning || !aiConfigReady;
  }

  const fingerprint = JSON.stringify([config, device.models || []]);
  if (!formDirty && fingerprint !== configFingerprint) {
    syncAIForm(config, device.models || []);
    configFingerprint = fingerprint;
  }
  const configCommand = monitor?.last_command;
  const configBadge = $("#config-state");
  if (configCommand?.type === "set_config") {
    if (configCommand.state === "applied") setVerdict(configBadge, "设备已应用", "good");
    else if (configCommand.state === "rejected" || configCommand.state === "ack_timeout") setVerdict(configBadge, "应用失败", "bad");
    else if (configCommand.state === "waiting_for_device_ack") setVerdict(configBadge, "等待确认", "warning");
  } else if (!formDirty) {
    setVerdict(
      configBadge,
      aiConfigReady ? "设备值" : "等待设备",
      "neutral",
    );
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

function setInspectorOpen(open) {
  const inspector = $("#inspector");
  const backdrop = $("#inspector-backdrop");
  const toggle = $("#inspector-toggle");
  inspector.classList.toggle("open", open);
  backdrop.classList.toggle("open", open);
  inspector.setAttribute("aria-hidden", String(!open));
  toggle.setAttribute("aria-expanded", String(open));
}

$("#inspector-toggle").addEventListener("click", () => setInspectorOpen(true));
$("#inspector-close").addEventListener("click", () => setInspectorOpen(false));
$("#inspector-backdrop").addEventListener("click", () => setInspectorOpen(false));
document.addEventListener("keydown", (event) => {
  if (event.key !== "Escape") return;
  if (expandedChartPanel) {
    setExpandedChart(null);
    return;
  }
  if (!ui.chartPicker.hidden) {
    setChartPickerOpen(false);
    ui.chartPickerToggle.focus();
    return;
  }
  setInspectorOpen(false);
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
    updateChartWindowButtons();
    returnChartsToLive();
  });
});

$("#chart-live").addEventListener("click", returnChartsToLive);
ui.chartPickerToggle.addEventListener("click", () => {
  setChartPickerOpen(ui.chartPicker.hidden);
});
ui.chartPickerClose.addEventListener("click", () => {
  setChartPickerOpen(false);
  ui.chartPickerToggle.focus();
});
$$("[data-chart-preset]").forEach((button) => {
  button.addEventListener("click", () => {
    setChartSelection(chartPresets[button.dataset.chartPreset] || []);
  });
});
document.addEventListener("pointerdown", (event) => {
  if (
    ui.chartPicker.hidden ||
    ui.chartPicker.contains(event.target) ||
    ui.chartPickerToggle.contains(event.target)
  ) {
    return;
  }
  setChartPickerOpen(false);
});

function aiFormMatchesDevice() {
  const config = latestState?.monitor?.config;
  if (!config || !config.model) return false;
  return (
    String(ui.modelSelect.value || "") === String(config.model) &&
    Number(ui.confidenceSlider.value) ===
      Math.round(Number(config.confidence) * 100) &&
    Number(ui.targetPercent.value) ===
      Math.round(Number(config.target_position) * 100) &&
    Number(ui.iouPercent.value) === Math.round(Number(config.iou) * 100) &&
    Number(ui.coastFrames.value) === Math.round(Number(config.coast_frames))
  );
}

function markAIFormDirty() {
  formDirty = !aiFormMatchesDevice();
  setVerdict(
    $("#config-state"),
    formDirty ? "未应用" : "设备值",
    formDirty ? "warning" : "neutral",
  );
}

ui.confidenceSlider.addEventListener("input", () => {
  setConfidencePercent(ui.confidenceSlider.value);
});

ui.form.addEventListener("input", markAIFormDirty);
ui.form.addEventListener("change", markAIFormDirty);

$$("[data-confidence-preset]").forEach((button) => {
  button.addEventListener("click", () => {
    setConfidencePercent(button.dataset.confidencePreset);
    markAIFormDirty();
  });
});

ui.form.addEventListener("submit", async (event) => {
  event.preventDefault();
  const threshold = clamp(
    Number(ui.confidenceSlider.value) / 100,
    0.01,
    0.99,
  );
  const target = clamp(
    Number(ui.targetPercent.value) / 100,
    0.05,
    0.95,
  );
  const iou = clamp(
    Number(ui.iouPercent.value) / 100,
    0.01,
    0.99,
  );
  const coastFrames = clamp(
    Math.round(Number(ui.coastFrames.value)),
    0,
    15,
  );
  const params = {
    target_position: target,
    confidence: threshold,
    valid_confidence: threshold,
    iou,
    coast_frames: coastFrames,
  };
  if (ui.modelSelect.value) {
    params.model = ui.modelSelect.value;
  }
  await requestAction("config", { params }, "参数已发送，等待设备确认");
  formDirty = false;
  setVerdict($("#config-state"), "等待确认", "warning");
});

if ("ResizeObserver" in window) {
  chartResizeObserver = new ResizeObserver(() => {
    chartsDirty = true;
  });
}
renderChartPanels();

setInterval(pollState, 300);
window.requestAnimationFrame(chartAnimationFrame);
pollState();
updateFrame();
