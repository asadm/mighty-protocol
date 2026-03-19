import { MightyClient, MightyWebDevice } from "mighty-protocol";
import {
  createDashboardUI,
  setConnectionBadge,
  setConnectButtonState,
  updateStatusFields,
  pickRenderableRaw,
  drawRawFrame,
  createImuPlotter,
  createPosePlot,
} from "./uihelpers.js";

const VIO_STATE_LABELS = {
  0: "OFF",
  1: "INITIALIZING",
  2: "TRACKING",
  3: "DEGRADED",
  4: "LOST",
};

function formatVioState(code) {
  if (typeof code !== "number" || !Number.isFinite(code)) return "STATE_NA";
  const label = VIO_STATE_LABELS[code] || `STATE_${code}`;
  return `${label} (${code})`;
}
const DATA_LIVE_TIMEOUT_MS = 2500;

const ui = createDashboardUI();
const imuPlotter = createImuPlotter(ui.imuCanvas);
const posePlot = createPosePlot(ui.poseHost);

const state = {
  running: false,
  lastDataAtMs: 0,
  vioCommandInFlight: false,
  vioStateCode: null,
  latestStatus: "Waiting for data...",
  latestVio: "STATE_NA",
  hostVersion: "Unknown",
  fps: "NA",
  features: "NA",
  poseConfidence: "NA",
  lastError: "",
  imageInfo: "No frame",
  poseInfo: "No pose",
};

const device = new MightyWebDevice();
const client = new MightyClient(device, {
  autoReconnect: true,
  reconnectDelayMs: 1000,
});

function markDataActivity() {
  state.lastDataAtMs = performance.now();
}

function isLiveConnected() {
  if (!state.running) return false;
  if (!state.lastDataAtMs) return false;
  return performance.now() - state.lastDataAtMs <= DATA_LIVE_TIMEOUT_MS;
}

function renderStatusPanel() {
  const connected = isLiveConnected();
  const connText = connected ? "connected" : (state.running ? "connecting" : "disconnected");
  const source = (device.getInfo && device.getInfo().source) || "";
  const showStop = typeof state.vioStateCode === "number" && state.vioStateCode !== 0;

  ui.vioBtn.textContent = state.vioCommandInFlight
    ? (showStop ? "Stopping..." : "Starting...")
    : (showStop ? "Stop VIO" : "Start VIO");
  ui.vioBtn.disabled = !state.running || state.vioCommandInFlight;

  setConnectionBadge(ui, connected, source);
  updateStatusFields(ui, {
    conn: connText,
    vio: state.latestVio,
    source: source || "(none)",
    status: state.latestStatus,
    host: state.hostVersion,
    fps: state.fps,
    features: state.features,
    pose_conf: state.poseConfidence,
    image: state.imageInfo,
    pose: state.poseInfo,
    error: state.lastError || "none",
  });
}

client.onImage((img) => {
  markDataActivity();
  try {
    const raw = pickRenderableRaw(img);
    if (!raw) return;

    if (drawRawFrame(ui.cameraCanvas, raw)) {
      const ch = raw.channelAlias || raw.channel || "cam0";
      state.imageInfo = `raw ${raw.width}x${raw.height} ${ch} ${raw.timestampNs || 0}`;
    }
  } catch (err) {
    state.lastError = `image draw failed: ${err?.message || err}`;
  }
});

client.onPose((pose) => {
  markDataActivity();
  posePlot.updatePose(pose.position, pose.quat || undefined, pose.confidence);
  if (Array.isArray(pose.position) && pose.position.length >= 3) {
    state.poseInfo = `${pose.position.map((v) => Number(v).toFixed(2)).join(", ")}`;
  }
  if (typeof pose.confidence === "number") {
    state.poseConfidence = pose.confidence.toFixed(3);
  }
});

client.onImu((imu) => {
  markDataActivity();
  imuPlotter.push(imu);
});

client.onVioState((vs) => {
  markDataActivity();
  state.vioStateCode = typeof vs.state === "number" ? vs.state : null;
  state.latestVio = formatVioState(vs.state);
  if (typeof vs.fpsCurrent === "number") state.fps = vs.fpsCurrent.toFixed(1);
  if (typeof vs.numFeatures === "number") state.features = `${vs.numFeatures}`;
  if (typeof vs.poseConfidence === "number") state.poseConfidence = vs.poseConfidence.toFixed(3);
  if (vs.buildVersion) state.hostVersion = String(vs.buildVersion);
});

client.onStatus((st) => {
  markDataActivity();
  const text = String(st?.text || "").trim();
  if (text.startsWith("HOST_VERSION:")) {
    state.hostVersion = text.slice("HOST_VERSION:".length).trim() || state.hostVersion;
  } else {
    let normalized = text;
    if (/^status:/i.test(normalized)) {
      normalized = normalized.slice(normalized.indexOf(":") + 1).trim();
    }
    // RSET is tracked separately via onReset(); avoid clobbering status text.
    if (/^reset$/i.test(normalized)) return;
    state.latestStatus = normalized || state.latestStatus;
  }
});

client.onReset(() => {
  // Keep last status text stable; reset frames are control events.
});

client.onError((err) => {
  state.lastError = `${err.scope || "unknown"}:${err.code || "unknown"} ${err.message || ""}`;
});

async function startClient() {
  if (state.running) return;
  state.running = true;
  setConnectButtonState(ui, true);
  state.lastError = "";
  state.lastDataAtMs = 0;
  state.vioCommandInFlight = false;
  state.vioStateCode = null;
  state.latestStatus = "Connecting...";
  await client.connect();
}

async function stopClient() {
  if (!state.running) return;
  state.running = false;
  state.vioCommandInFlight = false;
  state.vioStateCode = null;
  state.lastDataAtMs = 0;
  setConnectButtonState(ui, false);
  await client.disconnect();
}

ui.connectBtn.addEventListener("click", async () => {
  try {
    if (state.running) await stopClient();
    else await startClient();
  } catch (err) {
    state.lastError = `toggle failed: ${err?.message || err}`;
  }
});

ui.vioBtn.addEventListener("click", async () => {
  if (!state.running || state.vioCommandInFlight) return;
  state.vioCommandInFlight = true;
  state.lastError = "";
  renderStatusPanel();
  try {
    const shouldStop = typeof state.vioStateCode === "number" && state.vioStateCode !== 0;
    const cmdRes = shouldStop ? await client.stopVio() : await client.startVio();
    if (!cmdRes.ok) {
      state.lastError = `${shouldStop ? "stop_vio" : "start_vio"} failed: ${cmdRes.message || "unknown"}`;
    } else {
      state.latestStatus = shouldStop ? "stop_vio sent" : "start_vio sent";
    }
  } catch (err) {
    state.lastError = `vio command error: ${err?.message || err}`;
  } finally {
    state.vioCommandInFlight = false;
  }
});

window.addEventListener("resize", () => {
  posePlot.resize();
});

window.addEventListener("beforeunload", async () => {
  try {
    await stopClient();
  } catch (_) {
    // ignore
  }
  posePlot.dispose();
});

setConnectButtonState(ui, false);
renderStatusPanel();

setInterval(() => {
  imuPlotter.render();
  renderStatusPanel();
}, 100);

void startClient();
