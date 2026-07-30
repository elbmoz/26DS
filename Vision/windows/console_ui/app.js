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
let chartPanOffsetMs = 0;
let chartHoverTimeMs = null;
let chartDragging = false;
let chartDragStartX = 0;
let chartDragStartPanMs = 0;
let chartsDirty = true;
let lastHistoryPruneMs = 0;
const telemetryHistory = [];
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
  const previousLatestTime = telemetryHistory.length
    ? telemetryHistory[telemetryHistory.length - 1].t
    : null;
  if (lastTelemetrySession && session !== lastTelemetrySession) {
    telemetryHistory.length = 0;
    chartPanOffsetMs = 0;
    chartHoverTimeMs = null;
    lastHistoryPruneMs = 0;
    updateChartLiveState();
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
  if (
    chartPanOffsetMs > 0 &&
    previousLatestTime != null &&
    now > previousLatestTime
  ) {
    chartPanOffsetMs += now - previousLatestTime;
  }

  if (now - lastHistoryPruneMs >= 5000) {
    const oldest = now - MAX_HISTORY_MS;
    let removeCount = 0;
    while (
      removeCount < telemetryHistory.length &&
      telemetryHistory[removeCount].t < oldest
    ) {
      removeCount += 1;
    }
    if (removeCount > 0) {
      telemetryHistory.splice(0, removeCount);
      if (chartPanOffsetMs > 0) chartsDirty = true;
    }
    lastHistoryPruneMs = now;
  }
  if (chartPanOffsetMs === 0) chartsDirty = true;
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

function chartLatestTime() {
  return telemetryHistory.length
    ? telemetryHistory[telemetryHistory.length - 1].t
    : Date.now();
}

function clampChartPan(value, windowMs = historyWindowMs) {
  if (!telemetryHistory.length) return 0;
  const availableMs = Math.max(
    0,
    chartLatestTime() - telemetryHistory[0].t - windowMs,
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

function lowerBoundSample(time) {
  let low = 0;
  let high = telemetryHistory.length;
  while (low < high) {
    const middle = (low + high) >> 1;
    if (telemetryHistory[middle].t < time) low = middle + 1;
    else high = middle;
  }
  return low;
}

function historyForCurrentWindow(range) {
  if (!telemetryHistory.length) return [];
  let startIndex = lowerBoundSample(range.startTime);
  let endIndex = lowerBoundSample(range.endTime);
  if (startIndex > 0) startIndex -= 1;
  if (
    endIndex < telemetryHistory.length &&
    telemetryHistory[endIndex].t <= range.endTime
  ) {
    endIndex += 1;
  }
  return telemetryHistory.slice(startIndex, endIndex);
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
  const minimum = options.minimum;
  const maximum = Math.max(minimum + 0.001, options.maximum);
  const mapX = (time) =>
    padding.left +
    ((time - range.startTime) / historyWindowMs) * plotWidth;
  const mapY = (value) =>
    padding.top +
    (1 - (value - minimum) / (maximum - minimum)) * plotHeight;

  context.save();
  context.strokeStyle = chartColors.grid;
  context.lineWidth = 1;
  for (let row = 0; row <= 4; row += 1) {
    const y = padding.top + (plotHeight * row) / 4;
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
  context.fillText(
    options.formatAxis(maximum),
    padding.left - 5,
    padding.top + 7,
  );
  context.fillText(
    options.formatAxis(minimum),
    padding.left - 5,
    padding.top + plotHeight,
  );
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
    drawSeries(
      context,
      samples,
      series,
      mapX,
      mapY,
      plotWidth,
    );
  }
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

function maximumFinite(values, fallback) {
  let maximum = fallback;
  for (const value of values) {
    if (Number.isFinite(value)) maximum = Math.max(maximum, value);
  }
  return maximum;
}

function renderCharts() {
  const range = chartViewRange();
  const samples = historyForCurrentWindow(range);
  const absoluteVelocity = samples.map((sample) =>
    Math.abs(sample.velocity ?? 0),
  );
  const velocityMaximum = maximumFinite(absoluteVelocity, 0);
  const velocityLimit = Math.max(
    100,
    Math.min(800, Math.ceil(velocityMaximum / 100) * 100),
  );
  const timingValues = [];
  for (const sample of samples) {
    timingValues.push(sample.detect, sample.latency);
  }
  const timingMaximum = Math.max(
    100,
    Math.min(
      500,
      Math.ceil(maximumFinite(timingValues, 0) / 50) * 50,
    ),
  );
  const target = samples.length
    ? samples[samples.length - 1].target
    : 0.5;

  drawWaveChart($("#position-chart"), samples, range, {
    minimum: 0,
    maximum: 1,
    reference: target ?? 0.5,
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
  });
  drawWaveChart($("#velocity-chart"), samples, range, {
    minimum: -velocityLimit,
    maximum: velocityLimit,
    reference: 0,
    formatAxis: (value) => `${Math.round(value)}`,
    series: [
      {
        key: "velocity",
        label: "速度",
        color: chartColors.velocity,
        pointColor: chartColors.velocityPoint,
        formatValue: (value) => `${value.toFixed(1)} px/s`,
      },
    ],
  });
  drawWaveChart($("#fps-chart"), samples, range, {
    minimum: 0,
    maximum: 60,
    reference: 30,
    formatAxis: (value) => `${Math.round(value)}`,
    series: [
      {
        key: "fps",
        label: "FPS",
        color: chartColors.fps,
        pointColor: chartColors.fpsPoint,
        formatValue: (value) => value.toFixed(2),
      },
    ],
  });
  drawWaveChart($("#timing-chart"), samples, range, {
    minimum: 0,
    maximum: timingMaximum,
    reference: null,
    formatAxis: (value) => `${Math.round(value)}`,
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
  });
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
      if (!telemetryHistory.length) return;
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
    if (!telemetryHistory.length) return;
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
    if (
      button.classList.contains("tab") ||
      button.id === "inspector-toggle" ||
      button.id === "inspector-close" ||
      button.id === "chart-live" ||
      button.dataset.windowSeconds
    ) {
      return;
    }
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
  if (event.key === "Escape") setInspectorOpen(false);
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
$$(".wave-panel canvas").forEach(installChartInteraction);

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
