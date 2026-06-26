import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { LineSegments2 } from "three/examples/jsm/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/examples/jsm/lines/LineSegmentsGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";
import { decodeRawToRgb } from "mighty-protocol";

const DEVICE_COLOR_HEX = 0xff0055;
const HIGH_CONF_COLOR_HEX = 0x0099ff;
const IMU_COLORS = { x: "#ff0055", y: "#0099ff", z: "#ffcc00" };
const IMU_BG = "#f8f8f4";
const IMU_GRID = "rgba(17, 17, 17, 0.08)";
const IMU_ZERO = "rgba(17, 17, 17, 0.3)";
const IMU_TEXT = "#111";
const CONF_THRESHOLD = 0.4;
const FOLLOW_BACK_DISTANCE = 2.4;
const FOLLOW_UP_OFFSET = 0.9;
const FOLLOW_SIDE_OFFSET = 0.35;
const FOLLOW_BASE_DISTANCE = Math.hypot(FOLLOW_BACK_DISTANCE, FOLLOW_UP_OFFSET, FOLLOW_SIDE_OFFSET);
const FOLLOW_ZOOM_SCALE_MIN = 0.05;
const FOLLOW_ZOOM_SCALE_MAX = 4.0;
const FOLLOW_POS_SMOOTH_RATE = 2.5;
const FOLLOW_ROT_SMOOTH_RATE = 3.0;
const FOLLOW_CAM_SMOOTH_RATE = 2.0;
const FOLLOW_TARGET_SMOOTH_RATE = 2.5;
const FOLLOW_HEADING_SMOOTH_RATE = 1.8;
const AUTO_LOCK_TIMEOUT_MS = 3000;
const R_VIZ_FROM_ODOM = new THREE.Matrix3().set(
  0, -1, 0,
  0, 0, 1,
  -1, 0, 0
);
const Q_VIZ_FROM_ODOM = new THREE.Quaternion()
  .setFromRotationMatrix(
    new THREE.Matrix4().set(
      0, -1, 0, 0,
      0, 0, 1, 0,
      -1, 0, 0, 0,
      0, 0, 0, 1
    )
  )
  .normalize();

function clamp(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}

function toArray(value) {
  return Array.isArray(value) ? value : [];
}

export function createDashboardUI() {
  const statusFields = {};
  document.querySelectorAll("[data-status-field]").forEach((el) => {
    const key = el.getAttribute("data-status-field");
    if (key) statusFields[key] = el;
  });

  const refs = {
    connectBtn: document.getElementById("connectBtn"),
    vioBtn: document.getElementById("vioBtn"),
    connBadge: document.getElementById("connBadge"),
    statusFields,
    cameraCanvas: document.getElementById("cameraCanvas"),
    imuCanvas: document.getElementById("imuCanvas"),
    poseHost: document.getElementById("pose3d"),
  };

  if (
    !refs.connectBtn ||
    !refs.vioBtn ||
    !refs.connBadge ||
    !refs.cameraCanvas ||
    !refs.imuCanvas ||
    !refs.poseHost ||
    Object.keys(statusFields).length === 0
  ) {
    throw new Error("Missing required DOM nodes for web example");
  }

  return refs;
}

export function setConnectionBadge(refs, connected, source = "") {
  refs.connBadge.textContent = connected ? "CONNECTED" : "DISCONNECTED";
  refs.connBadge.classList.toggle("badge-on", connected);
  refs.connBadge.classList.toggle("badge-off", !connected);
  refs.connBadge.title = source || "";
}

export function setConnectButtonState(refs, running) {
  refs.connectBtn.textContent = running ? "Disconnect" : "Connect";
}

export function updateStatusFields(refs, values) {
  for (const [key, value] of Object.entries(values || {})) {
    const el = refs.statusFields[key];
    if (!el) continue;
    el.textContent = String(value ?? "");
  }
}

export function pickRenderableRaw(imageEvt) {
  if (!imageEvt) return null;
  if (imageEvt.kind === "raw") return imageEvt;
  if (imageEvt.kind === "stereo_raw") {
    const left = imageEvt.left || null;
    const right = imageEvt.right || null;
    if ((left?.channelAlias || left?.channel || "") === "cam0") return left;
    if ((right?.channelAlias || right?.channel || "") === "cam0") return right;
    return left || right || null;
  }
  return null;
}

export function drawRawFrame(canvas, frame) {
  const decoded = decodeRawToRgb(frame);
  if (!decoded) return false;

  if (canvas.width !== decoded.width || canvas.height !== decoded.height) {
    canvas.width = decoded.width;
    canvas.height = decoded.height;
  }

  const ctx = canvas.getContext("2d", { alpha: false });
  if (!ctx) return false;
  const imageData = new ImageData(decoded.rgba, decoded.width, decoded.height);
  ctx.putImageData(imageData, 0, 0);
  return true;
}

export function mapCanonicalPoseToViz(position, quat) {
  if (!Array.isArray(position) || position.length < 3) {
    return { position, quat };
  }
  const pViz = new THREE.Vector3(
    Number(position[0]),
    Number(position[1]),
    Number(position[2])
  ).applyMatrix3(R_VIZ_FROM_ODOM);
  let qViz = quat;
  if (Array.isArray(quat) && quat.length === 4) {
    const q = new THREE.Quaternion(
      Number(quat[0]),
      Number(quat[1]),
      Number(quat[2]),
      Number(quat[3])
    ).normalize();
    qViz = Q_VIZ_FROM_ODOM.clone().multiply(q).normalize();
    qViz = [qViz.x, qViz.y, qViz.z, qViz.w];
  }
  return { position: [pViz.x, pViz.y, pViz.z], quat: qViz };
}

function drawImuChartRect(ctx, width, height, history, units, label, accessor) {
  const nowSec = performance.now() * 0.001;
  const tMax = history.length ? Math.max(history[history.length - 1].t, nowSec) : nowSec;
  const tMin = tMax - 10.0;
  const timeSpan = Math.max(tMax - tMin, 1e-3);
  const headerH = 30;
  const plotY = headerH;
  const plotH = Math.max(12, height - headerH);

  const visible = history.filter((sample) => sample.t >= tMin - 1e-6);
  let minVal = Infinity;
  let maxVal = -Infinity;

  for (const sample of visible) {
    const values = accessor(sample);
    minVal = Math.min(minVal, values.x, values.y, values.z);
    maxVal = Math.max(maxVal, values.x, values.y, values.z);
  }

  if (!visible.length) {
    minVal = -1;
    maxVal = 1;
  } else if (minVal === maxVal) {
    minVal -= 1;
    maxVal += 1;
  }

  const margin = Math.max((maxVal - minVal) * 0.1, 1e-3);
  minVal -= margin;
  maxVal += margin;
  const valueRange = Math.max(maxVal - minVal, 1e-3);

  ctx.fillStyle = IMU_BG;
  ctx.fillRect(0, 0, width, height);

  ctx.save();
  ctx.beginPath();
  ctx.rect(0, plotY, width, plotH);
  ctx.clip();

  ctx.strokeStyle = IMU_GRID;
  ctx.lineWidth = 1;
  const gridCols = 5;
  for (let i = 0; i <= gridCols; i += 1) {
    const x = (i / gridCols) * width;
    ctx.beginPath();
    ctx.moveTo(x, plotY);
    ctx.lineTo(x, plotY + plotH);
    ctx.stroke();
  }
  const gridRows = 4;
  for (let i = 0; i <= gridRows; i += 1) {
    const y = plotY + (i / gridRows) * plotH;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  if (minVal < 0 && maxVal > 0) {
    const zeroY = plotY + (plotH - ((0 - minVal) / valueRange) * plotH);
    ctx.strokeStyle = IMU_ZERO;
    ctx.beginPath();
    ctx.moveTo(0, zeroY);
    ctx.lineTo(width, zeroY);
    ctx.stroke();
  }

  ["x", "y", "z"].forEach((axis) => {
    ctx.strokeStyle = IMU_COLORS[axis];
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    for (const sample of visible) {
      const v = accessor(sample);
      const x = ((sample.t - tMin) / timeSpan) * width;
      const y = plotY + (plotH - ((v[axis] - minVal) / valueRange) * plotH);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  });

  ctx.restore();

  ctx.fillStyle = IMU_TEXT;
  ctx.font = '11px "Space Mono", monospace';
  if (label) {
    ctx.fillText(label, 8, 12);
  }
  ctx.font = '12px "Space Mono", monospace';
  if (visible.length) {
    ctx.fillText(`range ${minVal.toFixed(2)} to ${maxVal.toFixed(2)} ${units}`, 8, 26);
  } else {
    ctx.fillText("Waiting for IMU data...", 8, 26);
  }
}

export function createImuPlotter(canvas) {
  const ctx = canvas.getContext("2d", { alpha: false });
  if (!ctx) throw new Error("imu canvas 2d context unavailable");

  const accelHistory = [];
  const gyroHistory = [];
  let imuTimeOffsetSec = null;
  let lastPlottedTs = 0;

  function push(imuEvt) {
    const incoming = toArray(imuEvt?.samples);
    if (incoming.length === 0) return;

    for (const s of incoming) {
      const nowSec = performance.now() * 0.001;
      const rawTsSec = Number.isFinite(Number(s?.timestampNs)) ? Number(s.timestampNs) * 1e-9 : NaN;

      let t;
      if (Number.isFinite(rawTsSec)) {
        if (imuTimeOffsetSec === null) {
          imuTimeOffsetSec = nowSec - rawTsSec;
        }
        t = rawTsSec + imuTimeOffsetSec;
      } else {
        t = nowSec;
      }

      if (Number.isFinite(lastPlottedTs) && Number.isFinite(t) && t < lastPlottedTs) {
        t = lastPlottedTs;
      }
      lastPlottedTs = t;

      accelHistory.push({
        t,
        x: Number(s?.ax || 0),
        y: Number(s?.ay || 0),
        z: Number(s?.az || 0),
      });
      gyroHistory.push({
        t,
        x: Number(s?.gx || 0),
        y: Number(s?.gy || 0),
        z: Number(s?.gz || 0),
      });
    }

    const cutoff = (performance.now() * 0.001) - 12.0;
    while (accelHistory.length > 0 && accelHistory[0].t < cutoff) accelHistory.shift();
    while (gyroHistory.length > 0 && gyroHistory[0].t < cutoff) gyroHistory.shift();
  }

  function render() {
    const widthCss = Math.max(1, canvas.clientWidth || 320);
    const heightCss = Math.max(1, canvas.clientHeight || 220);
    const dpr = window.devicePixelRatio || 1;
    const displayWidth = Math.max(1, Math.floor(widthCss * dpr));
    const displayHeight = Math.max(1, Math.floor(heightCss * dpr));
    if (canvas.width !== displayWidth || canvas.height !== displayHeight) {
      canvas.width = displayWidth;
      canvas.height = displayHeight;
    }

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.scale(dpr, dpr);

    const w = widthCss;
    const h = heightCss;

    const topY = 0;
    const midPad = 8;
    const topH = Math.floor((h - midPad) * 0.5);
    const bottomY = topH + midPad;
    const bottomH = h - bottomY;

    ctx.save();
    ctx.translate(0, topY);
    drawImuChartRect(ctx, w, topH, accelHistory, "m/s²", "ACC", (s) => ({ x: s.x, y: s.y, z: s.z }));
    ctx.restore();

    ctx.save();
    ctx.translate(0, bottomY);
    drawImuChartRect(ctx, w, bottomH, gyroHistory, "rad/s", "GYRO", (s) => ({ x: s.x, y: s.y, z: s.z }));
    ctx.restore();

    ctx.strokeStyle = "rgba(17, 17, 17, 0.18)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, bottomY - Math.floor(midPad * 0.5));
    ctx.lineTo(w, bottomY - Math.floor(midPad * 0.5));
    ctx.stroke();
    ctx.restore();
  }

  return { push, render };
}

function makeCameraFrustum(host, colorHex) {
  const depth = 0.125;
  const width = 0.06;
  const positions = [
    0, 0, 0, depth, -width, -width,
    0, 0, 0, depth, width, -width,
    0, 0, 0, depth, width, width,
    0, 0, 0, depth, -width, width,
    depth, -width, -width, depth, width, -width,
    depth, width, -width, depth, width, width,
    depth, width, width, depth, -width, width,
    depth, -width, width, depth, -width, -width,
  ];

  const geometry = new LineSegmentsGeometry();
  geometry.setPositions(positions);

  const material = new LineMaterial({ color: colorHex, linewidth: 2 });
  material.resolution.set(host.clientWidth || 1, host.clientHeight || 1);

  const frustum = new LineSegments2(geometry, material);
  frustum.computeLineDistances();
  frustum.frustumCulled = false;
  return frustum;
}

function confidenceColor(confidence) {
  const c = Number.isFinite(confidence) ? clamp(confidence, 0, 1) : 1.0;
  return c > CONF_THRESHOLD ? HIGH_CONF_COLOR_HEX : DEVICE_COLOR_HEX;
}

export function createPosePlot(host) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf4f4f0);

  const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 200);
  camera.position.set(0, 0.6, 2.2);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  host.innerHTML = "";
  host.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.enablePan = true;
  controls.target.set(0, 0, 0);

  scene.add(new THREE.GridHelper(80, 160, 0xff0055, 0x0099ff));
  scene.add(new THREE.AxesHelper(1.2));

  const camRig = new THREE.Group();
  const camFrustum = makeCameraFrustum(host, DEVICE_COLOR_HEX);
  camRig.add(camFrustum);
  scene.add(camRig);

  const trajMaterial = new LineMaterial({ color: 0xffffff, linewidth: 3, vertexColors: true });
  trajMaterial.resolution.set(host.clientWidth || 1, host.clientHeight || 1);
  let trajLine = null;

  const pathPoints = [];
  const pathConfidence = [];
  const worldUp = new THREE.Vector3(0, 1, 0);
  const tmpForward = new THREE.Vector3();
  const tmpRight = new THREE.Vector3();
  const tmpDesiredCamPos = new THREE.Vector3();
  const followTarget = new THREE.Vector3();
  const followQuat = new THREE.Quaternion();
  const followHeading = new THREE.Vector3(1, 0, 0);
  let followZoomScale = 1.0;
  let hasFollowTarget = false;
  let hasFollowQuat = false;
  let hasFollowHeading = false;
  let latestPose = null;
  let latestQuat = null;
  let cameraAutoLocked = true;
  let lastUserInteractMs = performance.now();
  let isUserInteracting = false;

  function captureFollowZoomScale() {
    const distance = camera.position.distanceTo(controls.target);
    if (!Number.isFinite(distance) || distance <= 1e-4) return;
    followZoomScale = THREE.MathUtils.clamp(
      distance / FOLLOW_BASE_DISTANCE,
      FOLLOW_ZOOM_SCALE_MIN,
      FOLLOW_ZOOM_SCALE_MAX
    );
  }

  function markUserInteraction() {
    cameraAutoLocked = false;
    lastUserInteractMs = performance.now();
    captureFollowZoomScale();
  }

  function beginUserInteraction() {
    isUserInteracting = true;
    markUserInteraction();
  }

  function endUserInteraction() {
    isUserInteracting = false;
    lastUserInteractMs = performance.now();
    captureFollowZoomScale();
  }

  controls.addEventListener("start", beginUserInteraction);
  controls.addEventListener("end", endUserInteraction);
  renderer.domElement.addEventListener("wheel", markUserInteraction, { passive: true });
  renderer.domElement.addEventListener("mousedown", beginUserInteraction);
  renderer.domElement.addEventListener("touchstart", beginUserInteraction);
  window.addEventListener("mouseup", endUserInteraction);
  window.addEventListener("touchend", endUserInteraction);

  function rebuildTrajectory() {
    if (trajLine) {
      scene.remove(trajLine);
      trajLine.geometry.dispose();
      trajLine = null;
    }

    if (pathPoints.length < 2) return;

    const segments = pathPoints.length - 1;
    const positions = new Float32Array(segments * 6);
    const colors = new Float32Array(segments * 6);

    for (let i = 1; i < pathPoints.length; i += 1) {
      const p0 = pathPoints[i - 1];
      const p1 = pathPoints[i];
      const off = (i - 1) * 6;

      positions[off] = p0.x;
      positions[off + 1] = p0.y;
      positions[off + 2] = p0.z;
      positions[off + 3] = p1.x;
      positions[off + 4] = p1.y;
      positions[off + 5] = p1.z;

      const col = new THREE.Color(confidenceColor(pathConfidence[i]));
      colors[off] = col.r;
      colors[off + 1] = col.g;
      colors[off + 2] = col.b;
      colors[off + 3] = col.r;
      colors[off + 4] = col.g;
      colors[off + 5] = col.b;
    }

    const geometry = new LineSegmentsGeometry();
    geometry.setPositions(positions);
    geometry.setColors(colors);

    trajLine = new LineSegments2(geometry, trajMaterial);
    trajLine.computeLineDistances();
    trajLine.frustumCulled = false;
    scene.add(trajLine);
  }

  function resize() {
    const w = Math.max(10, host.clientWidth);
    const h = Math.max(10, host.clientHeight);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    trajMaterial.resolution.set(w, h);

    if (camFrustum?.material?.resolution) {
      camFrustum.material.resolution.set(w, h);
    }
  }

  function updatePose(position, quat, confidence = 1.0) {
    if (!Array.isArray(position) || position.length < 3) return;

    const p = new THREE.Vector3(Number(position[0]), Number(position[1]), Number(position[2]));
    camRig.position.copy(p);
    latestPose = p.clone();

    if (Array.isArray(quat) && quat.length === 4) {
      camRig.quaternion.set(Number(quat[0]), Number(quat[1]), Number(quat[2]), Number(quat[3]));
      latestQuat = camRig.quaternion.clone();
    } else {
      latestQuat = null;
    }

    pathPoints.push(p);
    pathConfidence.push(Number.isFinite(confidence) ? confidence : 1.0);

    if (pathPoints.length > 3000) {
      pathPoints.shift();
      pathConfidence.shift();
    }

    rebuildTrajectory();
  }

  const clock = new THREE.Clock();
  let rafId = 0;
  function animate() {
    rafId = requestAnimationFrame(animate);
    const delta = clock.getDelta();
    const dt = Math.min(delta, 0.1);
    const posAlpha = 1 - Math.exp(-dt * FOLLOW_POS_SMOOTH_RATE);
    const rotAlpha = 1 - Math.exp(-dt * FOLLOW_ROT_SMOOTH_RATE);
    const camAlpha = 1 - Math.exp(-dt * FOLLOW_CAM_SMOOTH_RATE);
    const targetAlpha = 1 - Math.exp(-dt * FOLLOW_TARGET_SMOOTH_RATE);
    const headingAlpha = 1 - Math.exp(-dt * FOLLOW_HEADING_SMOOTH_RATE);

    if (latestPose && !hasFollowTarget) {
      followTarget.copy(latestPose);
      hasFollowTarget = true;
    } else if (latestPose) {
      followTarget.lerp(latestPose, posAlpha);
    }

    if (latestQuat && !hasFollowQuat) {
      followQuat.copy(latestQuat);
      hasFollowQuat = true;
    } else if (latestQuat) {
      followQuat.slerp(latestQuat, rotAlpha);
    } else {
      followQuat.slerp(new THREE.Quaternion(), rotAlpha);
    }

    if (hasFollowQuat) {
      tmpForward.set(1, 0, 0).applyQuaternion(followQuat);
      tmpForward.y = 0;
      if (tmpForward.lengthSq() > 1e-6) {
        tmpForward.normalize();
        if (!hasFollowHeading) {
          followHeading.copy(tmpForward);
          hasFollowHeading = true;
        } else {
          followHeading.lerp(tmpForward, headingAlpha).normalize();
        }
      }
    }

    const nowMs = performance.now();
    if (!cameraAutoLocked && !isUserInteracting && nowMs - lastUserInteractMs > AUTO_LOCK_TIMEOUT_MS) {
      cameraAutoLocked = true;
    }

    if (latestPose && cameraAutoLocked) {
      const followPosResolved = hasFollowTarget ? followTarget : latestPose;
      if (hasFollowHeading) {
        tmpForward.copy(followHeading);
        tmpRight.crossVectors(tmpForward, worldUp).normalize();
      } else {
        const followQuatResolved = hasFollowQuat ? followQuat : camRig.quaternion;
        tmpForward.set(1, 0, 0).applyQuaternion(followQuatResolved).normalize();
        tmpRight.set(0, -1, 0).applyQuaternion(followQuatResolved).normalize();
      }

      tmpDesiredCamPos
        .copy(followPosResolved)
        .addScaledVector(tmpForward, -FOLLOW_BACK_DISTANCE * followZoomScale)
        .addScaledVector(tmpRight, FOLLOW_SIDE_OFFSET * followZoomScale)
        .addScaledVector(worldUp, FOLLOW_UP_OFFSET * followZoomScale);

      if (camera.position.distanceToSquared(tmpDesiredCamPos) > 100.0) {
        camera.position.copy(tmpDesiredCamPos);
      } else {
        camera.position.lerp(tmpDesiredCamPos, camAlpha);
      }
      controls.target.lerp(followPosResolved, targetAlpha);
    }

    controls.update();
    renderer.render(scene, camera);
  }

  function dispose() {
    cancelAnimationFrame(rafId);

    if (trajLine) {
      scene.remove(trajLine);
      trajLine.geometry.dispose();
      trajLine = null;
    }

    camFrustum.geometry.dispose();
    camFrustum.material.dispose();
    trajMaterial.dispose();

    controls.removeEventListener("start", beginUserInteraction);
    controls.removeEventListener("end", endUserInteraction);
    renderer.domElement.removeEventListener("wheel", markUserInteraction);
    renderer.domElement.removeEventListener("mousedown", beginUserInteraction);
    renderer.domElement.removeEventListener("touchstart", beginUserInteraction);
    window.removeEventListener("mouseup", endUserInteraction);
    window.removeEventListener("touchend", endUserInteraction);
    controls.dispose();
    renderer.dispose();
  }

  resize();
  animate();

  return { updatePose, resize, dispose };
}
