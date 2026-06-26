import createMightyLoopClosureModule from "../../lib/loopclosure/wasm/lib/mighty_loopclosure_device.js";
import {
  NativeMapperWasm,
  createLoopClosureWasmModule,
} from "mighty-protocol";

const PUMP_INTERVAL_MS = 4;
const MAP_UPDATE_INTERVAL_MS = 120;
const MAX_PENDING_IMAGES = 300;
const MAX_PENDING_POSES = 3000;
const WASM_CACHE_BUSTER = "mapper-wasm-streaming-map";

let wasmModule = null;
let mapper = null;
let calibrationReady = false;
let pumpTimer = 0;
let mapUpdateTimer = 0;
let droppedByWorker = 0;
const pendingEvents = [];
let pendingImageCount = 0;
let pendingPoseCount = 0;

function dropOldestEvent(type) {
  const index = pendingEvents.findIndex((event) => event.type === type);
  if (index < 0) return false;
  pendingEvents.splice(index, 1);
  if (type === "image") {
    pendingImageCount = Math.max(0, pendingImageCount - 1);
    droppedByWorker += 1;
  } else if (type === "pose") {
    pendingPoseCount = Math.max(0, pendingPoseCount - 1);
  }
  return true;
}

function post(type, data = {}) {
  self.postMessage({ type, ...data });
}

function postStatus(status) {
  post("status", {
    status,
    pendingImages: pendingImageCount,
    pendingPoses: pendingPoseCount,
    droppedByWorker,
  });
}

function postWasmLog(stream, text) {
  void stream;
  void text;
}

function postMapUpdate(update, retained = false) {
  const transfer = [];
  for (const frame of update?.frames || []) {
    if (frame?.pointPositions?.buffer) transfer.push(frame.pointPositions.buffer);
  }
  self.postMessage({ type: "mapUpdate", update, retained }, transfer);
}

function hasMapUpdate(update) {
  return !!update?.reset ||
    (Array.isArray(update?.frames) && update.frames.length > 0) ||
    (Array.isArray(update?.trajectory) && update.trajectory.length > 0);
}

async function init() {
  if (mapper) {
    postStatus(calibrationReady ? "mapper ready" : "waiting for calib");
    return;
  }
  postStatus("loading wasm");
  wasmModule = await createLoopClosureWasmModule({
    moduleFactory: createMightyLoopClosureModule,
    wasmUrl: `/mighty_loopclosure_device.wasm?v=${WASM_CACHE_BUSTER}`,
    print: (text) => postWasmLog("stdout", text),
    printErr: (text) => postWasmLog("stderr", text),
  });
  mapper = new NativeMapperWasm(wasmModule, { quiet: true });
  mapper.onMapUpdate((update) => postMapUpdate(update));
  postStatus("mapper ready");
  startTimers();
}

function startTimers() {
  if (!pumpTimer) pumpTimer = self.setInterval(processQueues, PUMP_INTERVAL_MS);
  if (!mapUpdateTimer) mapUpdateTimer = self.setInterval(sendMapUpdate, MAP_UPDATE_INTERVAL_MS);
}

function ingestResult(result) {
  if (!result) return;
  post("result", {
    result,
    pendingImages: pendingImageCount,
    pendingPoses: pendingPoseCount,
    droppedByWorker,
  });
}

function processQueues() {
  if (!mapper || !calibrationReady) return;
  try {
    for (let i = 0; i < 120 && pendingEvents.length > 0; i += 1) {
      const event = pendingEvents.shift();
      if (event.type === "pose") {
        pendingPoseCount = Math.max(0, pendingPoseCount - 1);
        ingestResult(mapper.pushPose(event.pose));
      } else if (event.type === "image") {
        pendingImageCount = Math.max(0, pendingImageCount - 1);
        ingestResult(mapper.pushImage(event.raw));
      }
    }
  } catch (err) {
    post("error", { message: `mapper: ${err?.message || err}` });
  }
}

function sendMapUpdate(forceFull = false) {
  if (!mapper) return;
  try {
    const update = mapper.pollMapUpdates({ forceFull, typedPoints: true });
    if (forceFull || hasMapUpdate(update)) postMapUpdate(update);
  } catch (err) {
    post("error", { message: `map_update: ${err?.message || err}` });
  }
}

function enqueueImage(raw) {
  pendingEvents.push({ type: "image", raw });
  pendingImageCount += 1;
  while (pendingImageCount > MAX_PENDING_IMAGES) dropOldestEvent("image");
}

function enqueuePose(pose) {
  pendingEvents.push({ type: "pose", pose });
  pendingPoseCount += 1;
  while (pendingPoseCount > MAX_PENDING_POSES) dropOldestEvent("pose");
}

self.addEventListener("message", async (event) => {
  const msg = event.data || {};
  try {
    if (msg.type === "init") {
      await init();
    } else if (msg.type === "calibration") {
      await init();
      mapper.setCalibrationYaml(msg.yaml || "");
      calibrationReady = true;
      postStatus("calib ready");
    } else if (msg.type === "image") {
      const raw = msg.raw || {};
      enqueueImage({
        ...raw,
        data: new Uint8Array(raw.data || new ArrayBuffer(0)),
      });
      processQueues();
    } else if (msg.type === "pose") {
      enqueuePose(msg.pose);
      processQueues();
    } else if (msg.type === "reset") {
      if (mapper) mapper.reset();
      pendingEvents.length = 0;
      pendingImageCount = 0;
      pendingPoseCount = 0;
      droppedByWorker = 0;
      postStatus("mapper reset");
      sendMapUpdate();
    } else if (msg.type === "mapUpdateFull") {
      sendMapUpdate(true);
    } else if (msg.type === "close") {
      if (pumpTimer) self.clearInterval(pumpTimer);
      if (mapUpdateTimer) self.clearInterval(mapUpdateTimer);
      pumpTimer = 0;
      mapUpdateTimer = 0;
      if (mapper) mapper.close();
      mapper = null;
      wasmModule = null;
      calibrationReady = false;
      pendingEvents.length = 0;
      pendingImageCount = 0;
      pendingPoseCount = 0;
      postStatus("mapper closed");
    }
  } catch (err) {
    post("error", { message: err?.message || String(err) });
  }
});
