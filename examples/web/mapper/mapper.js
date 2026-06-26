import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { LineSegments2 } from "three/examples/jsm/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/examples/jsm/lines/LineSegmentsGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";
import {
  MightyClient,
  MightyWebDevice,
  decodeRawToRgb,
} from "mighty-protocol";

const BRAND_BLUE = 0x0099ff;
const BRAND_RED = 0xff0055;
const WHITE = 0xffffff;
const PANGOLIN_VIEW_WIDTH = 1600;
const PANGOLIN_VIEW_HEIGHT = 1000;
const PANGOLIN_FY = 900;
const PANGOLIN_FOV_DEG = THREE.MathUtils.radToDeg(2 * Math.atan(PANGOLIN_VIEW_HEIGHT / (2 * PANGOLIN_FY)));
const FOLLOW_ZOOM_SCALE = 3.0;
const FOLLOW_BACK_DISTANCE = 2.4;
const FOLLOW_UP_OFFSET = 0.9;
const FOLLOW_SIDE_OFFSET = 0.35;
const FOLLOW_POS_SMOOTH_RATE = 2.5;
const FOLLOW_HEADING_SMOOTH_RATE = 1.8;
const FOLLOW_CAM_SMOOTH_RATE = 2.0;
const FOLLOW_TARGET_SMOOTH_RATE = 2.5;
const STATUS_INTERVAL_MS = 120;

const ui = {
  sceneHost: document.getElementById("mapperScene"),
  connBadge: document.getElementById("connBadge"),
  connectBtn: document.getElementById("connectBtn"),
  followBtn: document.getElementById("followBtn"),
  resetBtn: document.getElementById("resetBtn"),
  previewCanvas: document.getElementById("previewCanvas"),
  frameText: document.getElementById("frameText"),
  mappedText: document.getElementById("mappedText"),
  pointText: document.getElementById("pointText"),
  poseText: document.getElementById("poseText"),
  pendingText: document.getElementById("pendingText"),
  sourceText: document.getElementById("sourceText"),
  statusText: document.getElementById("statusText"),
  errorText: document.getElementById("errorText"),
};

const state = {
  running: false,
  calibrationReady: false,
  frames: 0,
  mappedFrames: 0,
  droppedFrames: 0,
  pendingImages: 0,
  pendingPoses: 0,
  pointCount: 0,
  trajectoryCount: 0,
  lastError: "",
  status: "starting",
  lastDataAtMs: 0,
  lastStatusAtMs: 0,
  follow: true,
};
const params = new URLSearchParams(window.location.search);
const wasmOnlyMode = params.get("wasm-only") === "1";
const baseUrlParam = params.get("baseUrl") || "";
const latestFrameOnlyMode = params.get("latest-frame-only") === "1" ||
  params.get("latestFrameOnly") === "1";

function mapperToRenderVec(x, y, z) {
  return new THREE.Vector3(Number(x || 0), -Number(y || 0), -Number(z || 0));
}

function mapperDirectionToRender(v) {
  return new THREE.Vector3(Number(v.x || 0), -Number(v.y || 0), -Number(v.z || 0));
}

function pickPrimaryRaw(imageEvt) {
  if (!imageEvt) return null;
  if (imageEvt.kind === "raw") return imageEvt;
  if (imageEvt.kind !== "stereo_raw") return null;
  const left = imageEvt.left || null;
  const right = imageEvt.right || null;
  const name = (raw) => String(raw?.channelAlias || raw?.channel || "").toLowerCase();
  if (name(left) === "cam0" || name(left) === "preview" || name(left) === "left") return left;
  if (name(right) === "cam0" || name(right) === "preview" || name(right) === "left") return right;
  return left || right || null;
}

function drawPreview(raw) {
  const decoded = decodeRawToRgb(raw);
  if (!decoded) return false;
  if (ui.previewCanvas.width !== decoded.width || ui.previewCanvas.height !== decoded.height) {
    ui.previewCanvas.width = decoded.width;
    ui.previewCanvas.height = decoded.height;
  }
  const ctx = ui.previewCanvas.getContext("2d", { alpha: false });
  if (!ctx) return false;
  ctx.putImageData(new ImageData(decoded.rgba, decoded.width, decoded.height), 0, 0);
  return true;
}

function isMapperPose(pose) {
  if (!pose?.isPublic) return false;
  if (!Array.isArray(pose.orientationXyzw) || pose.orientationXyzw.length !== 4) return false;
  return pose.poseType === "body" || pose.poseType === "camera" || pose.poseTypeRaw === 0 || pose.poseTypeRaw === 1;
}

function updateConnectionBadge() {
  const live = state.running && state.lastDataAtMs > 0 && performance.now() - state.lastDataAtMs < 2500;
  ui.connBadge.textContent = live ? "CONNECTED" : (state.running ? "CONNECTING" : "DISCONNECTED");
  ui.connBadge.classList.toggle("badge-on", live);
  ui.connBadge.classList.toggle("badge-off", !live);
  ui.connectBtn.textContent = state.running ? "Disconnect" : "Connect";
  ui.followBtn.classList.toggle("is-on", state.follow);
  ui.followBtn.textContent = state.follow ? "Follow" : "Orbit";
}

function renderStatus(force = false) {
  const now = performance.now();
  if (!force && now - state.lastStatusAtMs < STATUS_INTERVAL_MS) return;
  state.lastStatusAtMs = now;
  updateConnectionBadge();
  ui.frameText.textContent = String(state.frames);
  ui.mappedText.textContent = String(state.mappedFrames);
  ui.pointText.textContent = String(state.pointCount);
  ui.poseText.textContent = String(state.trajectoryCount);
  ui.pendingText.textContent = `${state.pendingImages}/${state.pendingPoses}`;
  ui.sourceText.textContent = device.getInfo?.().source || "(none)";
  const dropped = state.droppedFrames > 0 ? ` dropped=${state.droppedFrames}` : "";
  ui.statusText.textContent = `${state.status}${dropped}`;
  ui.errorText.textContent = state.lastError || "none";
}

function createMapperScene(host, options = {}) {
  const latestFrameOnly = !!options.latestFrameOnly;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(WHITE);

  const camera = new THREE.PerspectiveCamera(PANGOLIN_FOV_DEG, PANGOLIN_VIEW_WIDTH / PANGOLIN_VIEW_HEIGHT, 0.05, 5000);
  camera.position.set(0, 12, 0.01);
  camera.up.set(0, 0, -1);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setClearColor(WHITE, 1);
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  host.innerHTML = "";
  host.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0, 0, 0);

  const grid = new THREE.GridHelper(40, 160, BRAND_BLUE, BRAND_BLUE);
  grid.material.opacity = 0.42;
  grid.material.transparent = true;
  scene.add(grid);

  const pointGeometry = new THREE.BufferGeometry();
  let pointCapacity = 0;
  let pointCount = 0;
  let pointPositions = new Float32Array(0);
  const framePointPositions = new Map();
  const pointMaterial = new THREE.PointsMaterial({
    color: BRAND_BLUE,
    size: 2,
    sizeAttenuation: false,
  });
  const pointCloud = new THREE.Points(pointGeometry, pointMaterial);
  pointCloud.frustumCulled = false;
  scene.add(pointCloud);

  const trajectoryMaterial = new LineMaterial({
    color: BRAND_RED,
    linewidth: 4,
    transparent: false,
  });
  let trajectoryLine = null;
  let trajectorySamples = [];

  const latestDot = new THREE.Mesh(
    new THREE.SphereGeometry(0.075, 18, 12),
    new THREE.MeshBasicMaterial({ color: BRAND_RED })
  );
  latestDot.visible = false;
  scene.add(latestDot);

  const worldUp = new THREE.Vector3(0, 1, 0);
  const followTarget = new THREE.Vector3();
  const followHeading = new THREE.Vector3(1, 0, 0);
  const desiredCamera = new THREE.Vector3();
  const right = new THREE.Vector3();
  let hasFollowTarget = false;
  let hasFollowHeading = false;
  let latestPose = null;
  let latestHeading = null;

  function resize() {
    const w = Math.max(10, host.clientWidth);
    const h = Math.max(10, host.clientHeight);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    trajectoryMaterial.resolution.set(w, h);
  }

  function ensurePointCapacity(required) {
    if (required <= pointCapacity) return false;
    const nextCapacity = Math.max(1024, 2 ** Math.ceil(Math.log2(required)));
    const next = new Float32Array(nextCapacity * 3);
    next.set(pointPositions.subarray(0, pointCount * 3));
    pointPositions = next;
    pointCapacity = nextCapacity;
    const attr = new THREE.BufferAttribute(pointPositions, 3);
    attr.setUsage(THREE.DynamicDrawUsage);
    pointGeometry.setAttribute("position", attr);
    return true;
  }

  function markPointRangeUpdated(startPoint, count) {
    const attr = pointGeometry.getAttribute("position");
    if (!attr || count <= 0) return;
    if (typeof attr.addUpdateRange === "function") {
      attr.addUpdateRange(startPoint * 3, count * 3);
    } else {
      attr.updateRange.offset = startPoint * 3;
      attr.updateRange.count = count * 3;
    }
    attr.needsUpdate = true;
  }

  function rebuildFramePoints() {
    let total = 0;
    for (const positions of framePointPositions.values()) {
      total += Math.floor(positions.length / 3);
    }
    const grew = ensurePointCapacity(total);
    let write = 0;
    for (const positions of framePointPositions.values()) {
      pointPositions.set(positions, write * 3);
      write += Math.floor(positions.length / 3);
    }
    pointCount = total;
    pointGeometry.setDrawRange(0, pointCount);
    markPointRangeUpdated(0, pointCount);
    return grew;
  }

  function applyFrameUpdates(update) {
    const frames = Array.isArray(update?.frames) ? update.frames : [];
    if (!frames.length) return false;

    if (latestFrameOnly) {
      let latestFrame = null;
      for (const frame of frames) {
        const frameId = Number(frame.frameId);
        if (!Number.isFinite(frameId)) continue;
        if (!(frame.pointPositions instanceof Float32Array) ||
            frame.pointPositions.length === 0 ||
            frame.remove) {
          continue;
        }
        if (!latestFrame || frameId >= Number(latestFrame.frameId)) {
          latestFrame = frame;
        }
      }
      framePointPositions.clear();
      if (latestFrame) {
        const incoming = latestFrame.pointPositions;
        const converted = new Float32Array(incoming.length);
        for (let i = 0; i < incoming.length; i += 3) {
          converted[i] = incoming[i];
          converted[i + 1] = -incoming[i + 1];
          converted[i + 2] = -incoming[i + 2];
        }
        framePointPositions.set(Number(latestFrame.frameId), converted);
      }
      rebuildFramePoints();
      return true;
    }

    for (const frame of frames) {
      const frameId = Number(frame.frameId);
      if (!Number.isFinite(frameId)) continue;
      if (frame.remove) {
        framePointPositions.delete(frameId);
        continue;
      }
      const incoming = frame.pointPositions;
      if (!(incoming instanceof Float32Array)) continue;
      const converted = new Float32Array(incoming.length);
      for (let i = 0; i < incoming.length; i += 3) {
        converted[i] = incoming[i];
        converted[i + 1] = -incoming[i + 1];
        converted[i + 2] = -incoming[i + 2];
      }
      framePointPositions.set(frameId, converted);
    }
    rebuildFramePoints();
    return true;
  }

  function clearTrajectory() {
    if (!trajectoryLine) return;
    scene.remove(trajectoryLine);
    trajectoryLine.geometry.dispose();
    trajectoryLine = null;
  }

  function applyMapUpdate(update) {
    const trajectory = Array.isArray(update?.trajectory) ? update.trajectory : [];

    if (update?.reset) {
      pointCount = 0;
      framePointPositions.clear();
      pointGeometry.setDrawRange(0, 0);
      trajectorySamples = [];
      clearTrajectory();
    }

    if (!applyFrameUpdates(update) &&
        Number(update?.pointCount || 0) !== pointCount) {
      return false;
    }

    if (trajectory.length > 0) {
      const start = Math.max(0, Number(update.trajectoryStart || 0) >>> 0);
      if (start === 0) {
        trajectorySamples = [];
      } else if (start !== trajectorySamples.length) {
        return false;
      }
      trajectorySamples.push(...trajectory);
    }

    clearTrajectory();
    if (trajectorySamples.length >= 2) {
      const segments = (trajectorySamples.length - 1) * 2;
      const positions = new Float32Array(segments * 6);
      for (let i = 1; i < trajectorySamples.length; i += 1) {
        const a = mapperToRenderVec(trajectorySamples[i - 1].px, trajectorySamples[i - 1].py, trajectorySamples[i - 1].pz);
        const b = mapperToRenderVec(trajectorySamples[i].px, trajectorySamples[i].py, trajectorySamples[i].pz);
        const off = (i - 1) * 12;
        positions[off] = a.x;
        positions[off + 1] = a.y;
        positions[off + 2] = a.z;
        positions[off + 3] = b.x;
        positions[off + 4] = b.y;
        positions[off + 5] = b.z;
        positions[off + 6] = a.x;
        positions[off + 7] = a.y - 0.03;
        positions[off + 8] = a.z;
        positions[off + 9] = b.x;
        positions[off + 10] = b.y - 0.03;
        positions[off + 11] = b.z;
      }
      const geometry = new LineSegmentsGeometry();
      geometry.setPositions(positions);
      trajectoryLine = new LineSegments2(geometry, trajectoryMaterial);
      trajectoryLine.computeLineDistances();
      trajectoryLine.frustumCulled = false;
      scene.add(trajectoryLine);
    }

    const latest = trajectorySamples[trajectorySamples.length - 1] || null;
    if (latest) {
      latestPose = mapperToRenderVec(latest.px, latest.py, latest.pz);
      latestDot.position.copy(latestPose);
      latestDot.visible = true;
      const q = new THREE.Quaternion(latest.qx, latest.qy, latest.qz, latest.qw).normalize();
      const cameraForwardOdom = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
      latestHeading = mapperDirectionToRender(cameraForwardOdom).multiplyScalar(-1);
      latestHeading.y = 0;
      if (latestHeading.lengthSq() > 1e-9) latestHeading.normalize();
      else latestHeading = null;
    }
    return true;
  }

  const clock = new THREE.Clock();
  let rafId = 0;
  function animate() {
    rafId = requestAnimationFrame(animate);
    const dt = Math.min(clock.getDelta(), 0.1);

    if (latestPose && !hasFollowTarget) {
      followTarget.copy(latestPose);
      hasFollowTarget = true;
    } else if (latestPose) {
      const alpha = 1 - Math.exp(-dt * FOLLOW_POS_SMOOTH_RATE);
      followTarget.lerp(latestPose, alpha);
    }

    if (latestHeading && !hasFollowHeading) {
      followHeading.copy(latestHeading);
      hasFollowHeading = true;
    } else if (latestHeading) {
      const alpha = 1 - Math.exp(-dt * FOLLOW_HEADING_SMOOTH_RATE);
      followHeading.lerp(latestHeading, alpha);
      if (followHeading.lengthSq() > 1e-9) followHeading.normalize();
    }

    if (state.follow && latestPose) {
      camera.up.copy(worldUp);
      const camAlpha = 1 - Math.exp(-dt * FOLLOW_CAM_SMOOTH_RATE);
      const targetAlpha = 1 - Math.exp(-dt * FOLLOW_TARGET_SMOOTH_RATE);
      const heading = hasFollowHeading ? followHeading : new THREE.Vector3(1, 0, 0);
      right.crossVectors(heading, worldUp);
      if (right.lengthSq() < 1e-9) right.set(0, 0, 1);
      else right.normalize();
      desiredCamera
        .copy(followTarget)
        .addScaledVector(heading, -FOLLOW_BACK_DISTANCE * FOLLOW_ZOOM_SCALE)
        .addScaledVector(right, FOLLOW_SIDE_OFFSET * FOLLOW_ZOOM_SCALE)
        .addScaledVector(worldUp, FOLLOW_UP_OFFSET * FOLLOW_ZOOM_SCALE);
      if (camera.position.distanceToSquared(desiredCamera) > 100) {
        camera.position.copy(desiredCamera);
      } else {
        camera.position.lerp(desiredCamera, camAlpha);
      }
      controls.target.lerp(followTarget, targetAlpha);
    }

    controls.update();
    renderer.render(scene, camera);
  }

  function resetView() {
    camera.position.set(0, 12, 0.01);
    camera.up.set(0, 0, -1);
    camera.lookAt(0, 0, 0);
    controls.target.set(0, 0, 0);
    hasFollowTarget = false;
    hasFollowHeading = false;
  }

  function dispose() {
    cancelAnimationFrame(rafId);
    clearTrajectory();
    pointGeometry.dispose();
    pointMaterial.dispose();
    trajectoryMaterial.dispose();
    latestDot.geometry.dispose();
    latestDot.material.dispose();
    controls.dispose();
    renderer.dispose();
  }

  resize();
  animate();
  function drawnPointCount() {
    return pointCount;
  }

  return { resize, applyMapUpdate, resetView, drawnPointCount, dispose };
}

const mapperScene = createMapperScene(ui.sceneHost, {
  latestFrameOnly: latestFrameOnlyMode,
});
const device = new MightyWebDevice({
  baseUrls: [
    ...(baseUrlParam ? [baseUrlParam] : []),
    "http://localhost:8084",
    "http://localhost:8080",
    "http://192.168.7.1",
  ],
});
const client = new MightyClient(device, {
  autoReconnect: true,
  reconnectDelayMs: 1000,
});

let lastPreviewDrawMs = 0;
const mapperWorker = new Worker(new URL("./mapper-worker.js", import.meta.url), { type: "module" });

async function ensureMapper() {
  state.status = "loading wasm";
  renderStatus(true);
  mapperWorker.postMessage({ type: "init" });
}

async function loadCalibration() {
  state.status = "fetching calib";
  renderStatus(true);
  for (let i = 0; i < 40; i += 1) {
    const cfg = await client.configGet("calib", { as: "text" });
    if (cfg.ok && cfg.found && cfg.value) {
      mapperWorker.postMessage({ type: "calibration", yaml: cfg.value });
      state.calibrationReady = true;
      state.status = "calib ready";
      renderStatus(true);
      return true;
    }
    await new Promise((resolve) => setTimeout(resolve, 250));
  }
  state.status = "waiting for calib";
  return false;
}

function updateWorkerCounters(msg) {
  if (typeof msg.pendingImages === "number") state.pendingImages = msg.pendingImages;
  if (typeof msg.pendingPoses === "number") state.pendingPoses = msg.pendingPoses;
  if (typeof msg.droppedByWorker === "number") state.droppedFrames = Math.max(state.droppedFrames, msg.droppedByWorker);
}

function ingestResult(result) {
  if (!result) return;
  if (result.statusText && result.statusText !== "not ready" && result.statusText !== "ok") {
    state.status = result.statusText;
  }
  if (typeof result.framesProcessed === "number") state.mappedFrames = result.framesProcessed;
  if (typeof result.framesDropped === "number") state.droppedFrames = result.framesDropped;
  if (latestFrameOnlyMode) {
    state.pointCount = mapperScene.drawnPointCount();
  } else if (typeof result.pointCount === "number") {
    state.pointCount = result.pointCount;
  }
  if (typeof result.trajectoryCount === "number") state.trajectoryCount = result.trajectoryCount;
  if (result.lost && state.pointCount > 0) {
    state.status = "mapping retained";
    if (String(state.lastError || "").startsWith("map_update:")) state.lastError = "";
  }
  else if (result.lost) state.status = "mapper lost";
  else if (result.initialized) state.status = "mapping";
}

mapperWorker.addEventListener("message", (event) => {
  const msg = event.data || {};
  updateWorkerCounters(msg);
  if (msg.type === "status") {
    if (msg.status) state.status = msg.status;
    if (msg.status === "calib ready") state.calibrationReady = true;
    if (msg.status === "mapping retained" && state.pointCount > 0 &&
        String(state.lastError || "").startsWith("map_update:")) {
      state.lastError = "";
    }
  } else if (msg.type === "result") {
    ingestResult(msg.result);
  } else if (msg.type === "mapUpdate") {
    const update = msg.update || {};
    state.pointCount = typeof update.pointCount === "number"
      ? update.pointCount
      : mapperScene.drawnPointCount();
    state.trajectoryCount = typeof update.trajectoryCount === "number"
      ? update.trajectoryCount
      : (Array.isArray(update.trajectory) ? update.trajectory.length : 0);
    if (state.pointCount > 0 || state.trajectoryCount > 0) {
      state.lastError = "";
    }
    if (msg.retained && state.pointCount > 0) {
      state.status = "mapping retained";
      if (String(state.lastError || "").startsWith("map_update:")) state.lastError = "";
    }
    const synced = mapperScene.applyMapUpdate(update);
    if (latestFrameOnlyMode) {
      state.pointCount = mapperScene.drawnPointCount();
    }
    if (!synced && state.running) {
      mapperWorker.postMessage({ type: "mapUpdateFull" });
    }
  } else if (msg.type === "error") {
    state.lastError = msg.message || "mapper worker error";
  }
  renderStatus();
});

mapperWorker.addEventListener("error", (event) => {
  state.lastError = event.message || "mapper worker failed";
  renderStatus(true);
});

function postImageToMapper(raw) {
  const data = raw.data instanceof Uint8Array ? raw.data : new Uint8Array(raw.data || []);
  const buffer = data.byteOffset === 0 && data.byteLength === data.buffer.byteLength
    ? data.buffer
    : data.slice().buffer;
  mapperWorker.postMessage({
    type: "image",
    raw: {
      timestampNs: raw.timestampNs,
      frameId: raw.frameId,
      width: raw.width,
      height: raw.height,
      format: raw.format,
      data: buffer,
    },
  }, [buffer]);
}

client.onImage((img) => {
  const raw = pickPrimaryRaw(img);
  if (!raw || !raw.timestampNs) return;
  state.frames += 1;
  state.lastDataAtMs = performance.now();
  const now = performance.now();
  if (now - lastPreviewDrawMs > 100) {
    lastPreviewDrawMs = now;
    try {
      drawPreview(raw);
    } catch (err) {
      state.lastError = `preview: ${err?.message || err}`;
    }
  }
  postImageToMapper(raw);
  renderStatus();
});

client.onPose((pose) => {
  if (!isMapperPose(pose)) return;
  state.lastDataAtMs = performance.now();
  mapperWorker.postMessage({ type: "pose", pose });
  renderStatus();
});

client.onStatus((status) => {
  const text = String(status?.text || "").trim();
  if (text) state.status = text;
  renderStatus();
});

client.onError((err) => {
  state.lastError = `${err.scope || "unknown"}:${err.code || "unknown"} ${err.message || ""}`;
  renderStatus(true);
});

async function connect() {
  if (state.running) return;
  state.running = true;
  state.lastError = "";
  renderStatus(true);
  try {
    await ensureMapper();
    await client.connect();
    if (!state.calibrationReady) await loadCalibration();
    state.status = state.calibrationReady ? "streaming" : "streaming without calib";
  } catch (err) {
    state.running = false;
    state.lastError = err?.message || String(err);
  }
  renderStatus(true);
}

async function disconnect() {
  if (!state.running) return;
  state.running = false;
  await client.disconnect();
  renderStatus(true);
}

ui.connectBtn.addEventListener("click", async () => {
  if (state.running) await disconnect();
  else await connect();
});

ui.followBtn.addEventListener("click", () => {
  state.follow = !state.follow;
  renderStatus(true);
});

ui.resetBtn.addEventListener("click", () => {
  mapperScene.resetView();
  mapperWorker.postMessage({ type: "reset" });
});

window.addEventListener("resize", () => mapperScene.resize());
window.addEventListener("beforeunload", () => {
  void disconnect();
  mapperWorker.postMessage({ type: "close" });
  mapperScene.dispose();
});

renderStatus(true);
if (wasmOnlyMode) {
  ensureMapper().catch((err) => {
    state.lastError = err?.message || String(err);
    renderStatus(true);
  });
} else {
  void connect();
}
