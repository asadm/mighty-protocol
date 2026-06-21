import { RAW_FORMAT } from "../core/protocol.js";

export const DEFAULT_LOOPCLOSURE_WASM_URL = "/mighty_loopclosure_device.wasm";
const LOOPCLOSURE_MODULE_URL = "../../lib/loopclosure/wasm/lib/mighty_loopclosure_device.js";

let loopclosureModuleFactoryPromise = null;

async function loadLoopClosureModuleFactory() {
  if (!loopclosureModuleFactoryPromise) {
    loopclosureModuleFactoryPromise = (async () => {
      try {
        const moduleUrl = new URL(LOOPCLOSURE_MODULE_URL, import.meta.url).href;
        const imported = await import(/* webpackIgnore: true */ moduleUrl);
        return imported.default || imported.createMightyLoopClosureModule || imported;
      } catch (err) {
        const error = new Error(
          "Mighty loop closure module package is not available. " +
          "Build or install mighty-protocol/lib/loopclosure/wasm before enabling loop closure."
        );
        error.code = "loopclosure_module_not_found";
        error.cause = err;
        throw error;
      }
    })();
  }
  return loopclosureModuleFactoryPromise;
}

const EVENT_NAMES = {
  1: "loop_closure",
};

const LAYOUT_FIELDS = [
  "rawImageSize",
  "rawImageTimestampNsOffset",
  "rawImageFrameIdOffset",
  "rawImageWidthOffset",
  "rawImageHeightOffset",
  "rawImageFormatOffset",
  "rawImageDataOffset",
  "rawImageSizeBytesOffset",
  "poseSize",
  "poseTimestampNsOffset",
  "posePxOffset",
  "poseQwOffset",
  "poseFrameOffset",
  "poseConfidenceOffset",
  "keyframeSize",
  "keyframeTimestampNsOffset",
  "keyframeFrameIdOffset",
  "keyframeDescriptorTypeOffset",
  "keyframeFlagsOffset",
  "keyframeDescriptorOffset",
  "keyframeDescriptorCountOffset",
  "eventSize",
  "eventVersionOffset",
  "eventTypeOffset",
  "eventTimestampNsOffset",
  "eventCurrentKeyframeOffset",
  "eventMatchedKeyframeOffset",
  "eventCorrectionTxOffset",
];

function asU8(data) {
  if (data instanceof Uint8Array) return data;
  if (ArrayBuffer.isView(data)) return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  if (data instanceof ArrayBuffer) return new Uint8Array(data);
  return new Uint8Array(data || []);
}

function readU64(module, ptr) {
  return Number(module.getValue(ptr, "i64"));
}

function writeU64(module, ptr, value) {
  module.setValue(ptr, BigInt(value || 0), "i64");
}

function writeCString(module, text) {
  const value = String(text || "");
  const size = module.lengthBytesUTF8(value) + 1;
  const ptr = module._malloc(size);
  module.stringToUTF8(value, ptr, size);
  return ptr;
}

function readLayout(module) {
  const bytes = LAYOUT_FIELDS.length * 4;
  const ptr = module._malloc(bytes);
  try {
    module._mlc_abi_layout(ptr);
    const out = {};
    LAYOUT_FIELDS.forEach((name, i) => {
      out[name] = module.getValue(ptr + i * 4, "i32") >>> 0;
    });
    return out;
  } finally {
    module._free(ptr);
  }
}

function isBrowserRuntime() {
  return typeof window !== "undefined" && typeof fetch === "function";
}

async function fetchWasmBinary(wasmUrl) {
  const response = await fetch(wasmUrl, { credentials: "same-origin" });
  if (!response.ok) {
    const error = new Error(
      `Mighty loop closure WASM not found at ${wasmUrl} (HTTP ${response.status}). ` +
      `Put mighty_loopclosure_device.wasm at ${DEFAULT_LOOPCLOSURE_WASM_URL}, ` +
      "or pass loopclosureWasmUrl with the URL where it is served."
    );
    error.code = "loopclosure_wasm_not_found";
    error.wasmUrl = wasmUrl;
    throw error;
  }
  return response.arrayBuffer();
}

export async function createLoopClosureWasmModule(options = {}) {
  const createMightyLoopClosureModule = options.moduleFactory
    || options.createModule
    || await loadLoopClosureModuleFactory();
  const wasmUrl = options.wasmUrl || options.url || "";
  const locateFile = options.locateFile || (wasmUrl
    ? ((name) => (name === "mighty_loopclosure_device.wasm" ? wasmUrl : name))
    : null);
  const wasmBinary = options.wasmBinary
    || (wasmUrl && isBrowserRuntime() ? await fetchWasmBinary(wasmUrl) : null);

  return createMightyLoopClosureModule({
    locateFile,
    wasmBinary,
    print: options.print,
    printErr: options.printErr,
  });
}

export class NativeLoopClosureWasm {
  constructor(module, options = {}) {
    if (!module) throw new Error("NativeLoopClosureWasm requires an initialized module");
    this.module = module;
    this.layout = readLayout(module);
    this.onEvent = typeof options.onEvent === "function" ? options.onEvent : null;
    this.hasPoseCorrection = false;
    this.poseTranslationCorrectionM = [0, 0, 0];
    this.trajectory = [];
    this.nextFrameId = 0;
    this.handle = 0;
    this.callbackPtr = 0;

    const outPtr = module._malloc(4);
    try {
      const status = module._mlc_create(0, outPtr);
      this._check(status, "mlc_create");
      this.handle = module.getValue(outPtr, "*");
    } finally {
      module._free(outPtr);
    }

    this.callbackPtr = module.addFunction((eventPtr) => {
      const event = this._readEvent(eventPtr);
      if (this.onEvent) this.onEvent(event);
    }, "vii");
    module._mlc_set_event_callback(this.handle, this.callbackPtr, 0);
    this._check(module._mlc_initialize(this.handle), "mlc_initialize");
  }

  close() {
    if (this.handle) {
      this.module._mlc_destroy(this.handle);
      this.handle = 0;
    }
    if (this.callbackPtr) {
      this.module.removeFunction(this.callbackPtr);
      this.callbackPtr = 0;
    }
  }

  setCalibrationYaml(yamlOrPath) {
    const ptr = writeCString(this.module, yamlOrPath);
    try {
      const status = this.module._mlc_set_calibration_yaml(this.handle, ptr);
      this._check(status, "mlc_set_calibration_yaml");
      return true;
    } finally {
      this.module._free(ptr);
    }
  }

  pushImage(image) {
    const raw = image?.kind === "stereo_raw" ? image.left : image;
    const data = asU8(raw?.data);
    if (!data.length) return false;
    const dataPtr = this.module._malloc(data.length);
    const msgPtr = this.module._malloc(this.layout.rawImageSize);
    try {
      this.module.HEAPU8.set(data, dataPtr);
      writeU64(this.module, msgPtr + this.layout.rawImageTimestampNsOffset, raw.timestampNs ?? raw.timestamp_ns ?? 0);
      this.module.setValue(msgPtr + this.layout.rawImageFrameIdOffset, this.nextFrameId++, "i32");
      this.module.setValue(msgPtr + this.layout.rawImageWidthOffset, raw.width || 0, "i32");
      this.module.setValue(msgPtr + this.layout.rawImageHeightOffset, raw.height || 0, "i32");
      this.module.setValue(msgPtr + this.layout.rawImageFormatOffset, raw.format ?? RAW_FORMAT.UNKNOWN, "i8");
      this.module.setValue(msgPtr + this.layout.rawImageDataOffset, dataPtr, "*");
      this.module.setValue(msgPtr + this.layout.rawImageSizeBytesOffset, data.length, "i32");
      return this.module._mlc_push_image(this.handle, msgPtr) === 0;
    } finally {
      this.module._free(msgPtr);
      this.module._free(dataPtr);
    }
  }

  pushPose(pose) {
    const pos = pose.positionM || pose.position_m || [0, 0, 0];
    const q = pose.orientationXyzw || pose.orientation_xyzw || [0, 0, 0, 1];
    const ptr = this.module._malloc(this.layout.poseSize);
    try {
      writeU64(this.module, ptr + this.layout.poseTimestampNsOffset, pose.timestampNs ?? pose.timestamp_ns ?? 0);
      for (let i = 0; i < 3; i += 1) this.module.setValue(ptr + this.layout.posePxOffset + i * 8, Number(pos[i] || 0), "double");
      this.module.setValue(ptr + this.layout.poseQwOffset, Number(q[3] ?? 1), "double");
      this.module.setValue(ptr + this.layout.poseQwOffset + 8, Number(q[0] || 0), "double");
      this.module.setValue(ptr + this.layout.poseQwOffset + 16, Number(q[1] || 0), "double");
      this.module.setValue(ptr + this.layout.poseQwOffset + 24, Number(q[2] || 0), "double");
      this.module.setValue(ptr + this.layout.poseFrameOffset, (pose.poseType || pose.pose_type) === "camera" ? 1 : 0, "i8");
      this.module.setValue(ptr + this.layout.poseConfidenceOffset, Number(pose.confidence ?? 1), "float");
      return this.module._mlc_push_pose(this.handle, ptr) === 0;
    } finally {
      this.module._free(ptr);
    }
  }

  pushKeyframe(keyframe) {
    const descriptor = keyframe.descriptor || [];
    if (!descriptor.length) return false;
    const descPtr = this.module._malloc(descriptor.length * 4);
    const msgPtr = this.module._malloc(this.layout.keyframeSize);
    try {
      this.module.HEAPF32.set(Float32Array.from(descriptor), descPtr >> 2);
      writeU64(this.module, msgPtr + this.layout.keyframeTimestampNsOffset, keyframe.timestampNs ?? keyframe.timestamp_ns ?? 0);
      this.module.setValue(msgPtr + this.layout.keyframeFrameIdOffset, keyframe.frameId ?? keyframe.frame_id ?? 0, "i32");
      this.module.setValue(msgPtr + this.layout.keyframeDescriptorTypeOffset, keyframe.descriptorType ?? keyframe.descriptor_type ?? 1, "i8");
      this.module.setValue(msgPtr + this.layout.keyframeFlagsOffset, keyframe.flags || 0, "i16");
      this.module.setValue(msgPtr + this.layout.keyframeDescriptorOffset, descPtr, "*");
      this.module.setValue(msgPtr + this.layout.keyframeDescriptorCountOffset, descriptor.length, "i32");
      return this.module._mlc_push_keyframe(this.handle, msgPtr) === 0;
    } finally {
      this.module._free(msgPtr);
      this.module._free(descPtr);
    }
  }

  correctPose(pose) {
    if (!pose?.isPublic && pose?.is_public !== true) return pose;
    if (!this.hasPoseCorrection) return pose;
    const pos = pose.positionM || pose.position_m;
    if (!pos || pos.length < 3) return pose;
    const correction = this._correctionForTimestamp(pose.timestampNs ?? pose.timestamp_ns);
    const corrected = pos.map((v, i) => Number(v) + correction[i]);
    return {
      ...pose,
      positionM: pose.positionM ? corrected : pose.positionM,
      position_m: pose.position_m ? corrected : pose.position_m,
      rawPositionM: pose.positionM ? pos.map(Number) : pose.rawPositionM,
      raw_position_m: pose.position_m ? pos.map(Number) : pose.raw_position_m,
      loopclosureCorrected: true,
      loopclosure_corrected: true,
    };
  }

  getTrajectory() {
    return this.trajectory.map((p) => ({
      keyframeIndex: p.keyframeIndex,
      timestampNs: p.timestampNs,
      rawPositionM: p.rawPositionM.slice(),
      positionM: p.positionM.slice(),
    }));
  }

  _readTrajectory() {
    const count = Number(this.module._mlc_trajectory_size(this.handle) || 0);
    if (!count) return [];
    const tsPtr = this.module._malloc(8);
    const rawPtr = this.module._malloc(3 * 8);
    const optPtr = this.module._malloc(3 * 8);
    const out = [];
    try {
      for (let i = 0; i < count; i += 1) {
        const status = this.module._mlc_trajectory_pose(this.handle, i, tsPtr, rawPtr, optPtr);
        if (status !== 0) continue;
        const raw = [0, 1, 2].map((j) => this.module.getValue(rawPtr + j * 8, "double"));
        const opt = [0, 1, 2].map((j) => this.module.getValue(optPtr + j * 8, "double"));
        out.push({
          keyframeIndex: i,
          timestampNs: readU64(this.module, tsPtr),
          rawPositionM: raw,
          positionM: opt,
        });
      }
    } finally {
      this.module._free(optPtr);
      this.module._free(rawPtr);
      this.module._free(tsPtr);
    }
    return out;
  }

  _correctionForTimestamp(timestampNs) {
    if (!this.trajectory.length) return this.poseTranslationCorrectionM;
    if (timestampNs === undefined || timestampNs === null) return this.poseTranslationCorrectionM;
    const t = Number(timestampNs);
    let prev = null;
    let next = null;
    for (const pose of this.trajectory) {
      const poseTime = Number(pose.timestampNs);
      if (poseTime <= t) prev = pose;
      if (poseTime >= t) {
        next = pose;
        break;
      }
    }
    const correctionOf = (pose) => pose.positionM.map((v, i) => Number(v) - Number(pose.rawPositionM[i]));
    if (prev && next && prev !== next) {
      const a = Number(prev.timestampNs);
      const b = Number(next.timestampNs);
      const alpha = b > a ? Math.max(0, Math.min(1, (t - a) / (b - a))) : 0;
      const pc = correctionOf(prev);
      const nc = correctionOf(next);
      return pc.map((v, i) => v + (nc[i] - v) * alpha);
    }
    if (prev) return correctionOf(prev);
    if (next) return correctionOf(next);
    return this.poseTranslationCorrectionM;
  }

  _readEvent(ptr) {
    const correction = [0, 1, 2].map((i) => this.module.getValue(ptr + this.layout.eventCorrectionTxOffset + i * 8, "double"));
    this.hasPoseCorrection = true;
    this.poseTranslationCorrectionM = correction;
    this.trajectory = this._readTrajectory();
    return {
      type: EVENT_NAMES[this.module.getValue(ptr + this.layout.eventTypeOffset, "i8")] || "unknown",
      timestampNs: readU64(this.module, ptr + this.layout.eventTimestampNsOffset),
      currentKeyframe: this.module.getValue(ptr + this.layout.eventCurrentKeyframeOffset, "i32") >>> 0,
      matchedKeyframe: this.module.getValue(ptr + this.layout.eventMatchedKeyframeOffset, "i32") >>> 0,
      poseTranslationCorrectionM: correction.slice(),
    };
  }

  _check(status, op) {
    if (status === 0) return;
    const msgPtr = this.module._mlc_status_message(status);
    const msg = msgPtr ? this.module.UTF8ToString(msgPtr) : "unknown";
    throw new Error(`${op} failed: ${msg}`);
  }
}
