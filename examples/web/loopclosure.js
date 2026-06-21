import {
  MightyClient,
  MightyWebDevice,
} from "mighty-protocol";

const ui = {
  connBadge: document.getElementById("connBadge"),
  connectBtn: document.getElementById("connectBtn"),
  vioBtn: document.getElementById("vioBtn"),
  sourceText: document.getElementById("sourceText"),
  keyframeText: document.getElementById("keyframeText"),
  loopText: document.getElementById("loopText"),
  frameText: document.getElementById("frameText"),
  errorText: document.getElementById("errorText"),
  eventList: document.getElementById("eventList"),
  loopPlot: document.getElementById("loopPlot"),
};

const state = {
  running: false,
  vioRunning: false,
  events: [],
  keyframes: [],
  poseHistory: [],
  plotPoses: [],
  loops: 0,
  frames: 0,
  lastError: "",
  lastStatusRenderMs: 0,
  lastPlotRenderMs: 0,
};

const DEFAULT_DEVICE_BASE_URL = "http://192.168.7.1";

function bounds(raw, opt) {
  const all = raw.concat(opt);
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  for (const p of all.length ? all : [[0, 0], [1, 1]]) {
    minX = Math.min(minX, p[0]);
    maxX = Math.max(maxX, p[0]);
    minY = Math.min(minY, p[1]);
    maxY = Math.max(maxY, p[1]);
  }
  if (Math.abs(maxX - minX) < 1e-9) {
    minX -= 1;
    maxX += 1;
  }
  if (Math.abs(maxY - minY) < 1e-9) {
    minY -= 1;
    maxY += 1;
  }
  return { minX, maxX, minY, maxY };
}

function renderPlot() {
  const plotted = state.keyframes.length ? state.keyframes : state.plotPoses;
  const raw = plotted.map((p) => {
    const pos = p.rawPositionM || p.positionM;
    return [pos[0], pos[1]];
  });
  const opt = plotted.map((p) => [p.positionM[0], p.positionM[1]]);
  const loopEdges = state.keyframes.length
    ? state.events.map((e) => [e.currentKeyframe, e.matchedKeyframe])
    : [];
  const { minX, maxX, minY, maxY } = bounds(raw, opt);
  const width = 1320;
  const height = 760;
  const margin = 48;
  const headerH = 96;
  const gap = 42;
  const plotW = (width - 2 * margin - gap) / 2;
  const plotH = height - headerH - margin;
  const scale = Math.min(plotW / (maxX - minX), plotH / (maxY - minY));
  const xOffset = (plotW - (maxX - minX) * scale) * 0.5;
  const yOffset = (plotH - (maxY - minY) * scale) * 0.5;
  const rawX = margin;
  const optX = margin + plotW + gap;
  const sx = (p, x0) => x0 + xOffset + (p[0] - minX) * scale;
  const sy = (p) => headerH + plotH - yOffset - (p[1] - minY) * scale;
  const poly = (points, x0) => points.map((p) => `${sx(p, x0).toFixed(2)},${sy(p).toFixed(2)}`).join(" ");

  let svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}">`;
  svg += `<rect width="${width}" height="${height}" fill="#f8f8f4"/>`;
  svg += `<text x="48" y="40" font-family="Space Mono, monospace" font-size="18" font-weight="700" fill="#111111">Mighty Loop Closure WASM</text>`;
  svg += `<text x="48" y="66" font-family="Space Mono, monospace" font-size="13" fill="#111111">poses: ${state.plotPoses.length} | keyframes: ${state.keyframes.length} | loops: ${loopEdges.length}</text>`;
  for (const [x0, title] of [[rawX, "Unoptimized path"], [optX, "Optimized path"]]) {
    svg += `<rect x="${x0}" y="${headerH}" width="${plotW}" height="${plotH}" fill="#fff" stroke="#d4d4ce"/>`;
    for (let i = 1; i < 10; i += 1) {
      const gx = x0 + plotW * i / 10;
      const gy = headerH + plotH * i / 10;
      svg += `<line x1="${gx}" y1="${headerH}" x2="${gx}" y2="${headerH + plotH}" stroke="#e4e4df"/>`;
      svg += `<line x1="${x0}" y1="${gy}" x2="${x0 + plotW}" y2="${gy}" stroke="#e4e4df"/>`;
    }
    svg += `<text x="${x0}" y="${headerH - 14}" font-family="Space Mono, monospace" font-size="14" font-weight="700" fill="#111111">${title}</text>`;
  }
  for (const [points, x0] of [[raw, rawX], [opt, optX]]) {
    for (const [current, matched] of loopEdges) {
      if (current < points.length && matched < points.length) {
        const a = points[current];
        const b = points[matched];
        svg += `<line x1="${sx(a, x0)}" y1="${sy(a)}" x2="${sx(b, x0)}" y2="${sy(b)}" stroke="#0099ff" stroke-width="2.5" stroke-dasharray="8 5" opacity="0.85"/>`;
      }
    }
  }
  if (raw.length >= 2) svg += `<polyline points="${poly(raw, rawX)}" fill="none" stroke="#666666" stroke-width="4" opacity="0.65" stroke-dasharray="10 8"/>`;
  if (opt.length >= 2) svg += `<polyline points="${poly(opt, optX)}" fill="none" stroke="#ff0055" stroke-width="4" opacity="0.95"/>`;
  for (const [points, x0, stroke] of [[raw, rawX, "#666666"], [opt, optX, "#ff0055"]]) {
    for (const p of points) svg += `<circle cx="${sx(p, x0)}" cy="${sy(p)}" r="3.5" fill="#ffcc00" stroke="${stroke}" stroke-width="1"/>`;
  }
  svg += "</svg>";
  ui.loopPlot.innerHTML = svg;
}

function addEventRow(event) {
  const li = document.createElement("li");
  li.innerHTML = `<span class="loop-event-type">${event.type}</span> current=${event.currentKeyframe} matched=${event.matchedKeyframe}`;
  ui.eventList.prepend(li);
  while (ui.eventList.children.length > 80) ui.eventList.lastElementChild.remove();
}

function renderStatus() {
  const source = device.getInfo?.().source || "";
  ui.connBadge.className = `badge ${state.running ? "badge-on" : "badge-off"}`;
  ui.connBadge.textContent = state.running ? "CONNECTED" : "DISCONNECTED";
  ui.connectBtn.textContent = state.running ? "Disconnect" : "Connect";
  ui.vioBtn.textContent = state.vioRunning ? "Stop VIO" : "Start VIO";
  ui.vioBtn.disabled = !state.running;
  ui.sourceText.textContent = source || "(none)";
  ui.keyframeText.textContent = `${state.keyframes.length}`;
  ui.loopText.textContent = `${state.loops}`;
  ui.frameText.textContent = `${state.frames}`;
  ui.errorText.textContent = state.lastError || "none";
}

function timestampDistance(a, b) {
  if (typeof a !== "bigint" || typeof b !== "bigint") return null;
  return a > b ? a - b : b - a;
}

function nearestPose(timestampNs) {
  let best = null;
  let bestDistance = null;
  for (const pose of state.poseHistory) {
    const distance = timestampDistance(pose.timestampNs, timestampNs);
    if (distance === null) continue;
    if (bestDistance === null || distance < bestDistance) {
      best = pose;
      bestDistance = distance;
    }
  }
  return best || state.poseHistory[state.poseHistory.length - 1] || null;
}

function rememberPose(pose) {
  if (!Array.isArray(pose.positionM)) return;
  state.poseHistory.push(pose);
  while (state.poseHistory.length > 2000) state.poseHistory.shift();
  state.plotPoses.push(pose);
  while (state.plotPoses.length > 1500) state.plotPoses.shift();
}

function refreshOptimizedKeyframes() {
  const trajectory = client?.loopclosureTrajectory?.() || [];
  if (!trajectory.length) return;
  for (const item of trajectory) {
    const index = item.keyframeIndex;
    if (index >= state.keyframes.length) continue;
    state.keyframes[index] = {
      ...state.keyframes[index],
      rawPositionM: item.rawPositionM,
      positionM: item.positionM,
    };
  }
}

function renderStatusSoon() {
  const now = performance.now();
  if (now - state.lastStatusRenderMs < 250) return;
  state.lastStatusRenderMs = now;
  renderStatus();
}

function renderPlotSoon() {
  const now = performance.now();
  if (now - state.lastPlotRenderMs < 250) return;
  state.lastPlotRenderMs = now;
  renderPlot();
}

const device = new MightyWebDevice({ baseUrl: DEFAULT_DEVICE_BASE_URL });
let client = null;

window.addEventListener("error", (event) => {
  state.lastError = event.message || String(event.error || "window error");
  renderStatus();
});

window.addEventListener("unhandledrejection", (event) => {
  const reason = event.reason;
  state.lastError = reason?.message || String(reason || "unhandled rejection");
  renderStatus();
});

function wireClient(nextClient) {
  nextClient.onImage(() => {
    state.frames += 1;
    renderStatusSoon();
  });

  nextClient.onPose((pose) => {
    rememberPose(pose);
    renderPlotSoon();
    renderStatusSoon();
  });

  nextClient.onKeyframe((keyframe) => {
    const pose = nearestPose(keyframe.timestampNs);
    if (!pose || !Array.isArray(pose.positionM)) return;
    const keyframePose = {
      ...pose,
      keyframeTimestampNs: keyframe.timestampNs,
    };
    state.keyframes.push(keyframePose);
    refreshOptimizedKeyframes();
    renderPlot();
    renderStatus();
  });

  nextClient.onLoopclosure((event) => {
    state.events.push(event);
    state.loops = state.events.length;
    refreshOptimizedKeyframes();
    addEventRow(event);
    renderPlot();
    renderStatus();
  });

  nextClient.onError((err) => {
    state.lastError = `${err.scope || "unknown"}:${err.code || "unknown"} ${err.message || ""}`;
    renderStatus();
  });
}

async function ensureClient() {
  if (client) return client;
  client = new MightyClient(device, {
    autoReconnect: true,
    reconnectDelayMs: 1000,
    loopclosure: true,
    loopclosureWasmUrl: "/mighty_loopclosure_device.wasm",
  });
  wireClient(client);
  return client;
}

async function loadCalibration(nextClient) {
  for (let i = 0; i < 40; i += 1) {
    const cfg = await nextClient.configGet("calib", { as: "text" });
    if (cfg.ok && cfg.found && cfg.value) return nextClient.setLoopclosureCalibrationYaml(cfg.value);
    await new Promise((resolve) => setTimeout(resolve, 250));
  }
  return false;
}

async function connect() {
  const nextClient = await ensureClient();
  state.running = true;
  state.lastError = "";
  renderStatus();
  await loadCalibration(nextClient);
  await nextClient.connect();
  renderStatus();
}

async function disconnect() {
  try {
    if (state.vioRunning) await client?.stopVio();
  } catch (_) {
    // ignore shutdown races
  }
  state.vioRunning = false;
  state.running = false;
  await client?.disconnect();
  renderStatus();
}

ui.connectBtn.addEventListener("click", async () => {
  try {
    if (state.running) await disconnect();
    else await connect();
  } catch (err) {
    state.lastError = err?.message || String(err);
    renderStatus();
  }
});

ui.vioBtn.addEventListener("click", async () => {
  try {
    const nextClient = await ensureClient();
    const res = state.vioRunning ? await nextClient.stopVio() : await nextClient.startVio();
    if (!res.ok) throw new Error(res.message || "vio command failed");
    state.vioRunning = !state.vioRunning;
    renderStatus();
  } catch (err) {
    state.lastError = err?.message || String(err);
    renderStatus();
  }
});

window.addEventListener("beforeunload", () => {
  void disconnect();
});

renderPlot();
renderStatus();
connect().catch((err) => {
  state.running = false;
  state.lastError = err?.message || String(err);
  renderStatus();
});
