import * as protocol from "../core/protocol.js";
import {
  DEFAULT_LOOPCLOSURE_WASM_URL,
  NativeLoopClosureWasm,
  createLoopClosureWasmModule,
} from "./loopclosure-wasm.js";
import { toU8, encodeText, decodeText, sleep, isAbortError } from "./utils.js";

export const VIO_STATE = protocol.VIO_STATE;
export const VIO_DEGRADED_REASON = protocol.VIO_DEGRADED_REASON;
export const VIO_INIT_REASON = protocol.VIO_INIT_REASON;

const DEFAULT_OPTS = {
  commandTimeoutMs: 2000,
  autoReconnect: true,
  reconnectDelayMs: 300,
  streamStallTimeoutMs: 3000,
  emitStatAsStatus: true,
  normalizeChannelAliases: true,
  loopclosure: false,
  loopclosureWasmUrl: DEFAULT_LOOPCLOSURE_WASM_URL,
  loopclosureCalibrationYaml: "",
  loopclosureWasmModule: null,
  loopclosureWasmOptions: null,
  loopclosureFailOpen: false,
};

const EVENT_KEYS = [
  "image",
  "pose",
  "imu",
  "vio_state",
  "viz",
  "point_cloud",
  "lcon",
  "keyframe",
  "status",
  "lua_log",
  "event",
  "reset",
  "loopclosure",
  "any",
  "error",
];

function clamp01(v) {
  if (!Number.isFinite(v)) return 0;
  if (v < 0) return 0;
  if (v > 1) return 1;
  return v;
}

async function decodeJpegToRgbaFrame(jpegData) {
  if (typeof createImageBitmap !== "function" || typeof Blob === "undefined") {
    throw new Error("JPEG loopclosure decode requires browser image APIs");
  }
  const bitmap = await createImageBitmap(new Blob([toU8(jpegData)], { type: "image/jpeg" }));
  try {
    const width = bitmap.width || 0;
    const height = bitmap.height || 0;
    if (width <= 0 || height <= 0) throw new Error("decoded JPEG has invalid dimensions");
    const canvas = typeof OffscreenCanvas === "function"
      ? new OffscreenCanvas(width, height)
      : document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d", { willReadFrequently: true });
    if (!ctx) throw new Error("unable to create JPEG decode canvas");
    ctx.drawImage(bitmap, 0, 0);
    const pixels = ctx.getImageData(0, 0, width, height).data;
    return {
      width,
      height,
      format: protocol.RAW_FORMAT.RGBA32,
      data: new Uint8Array(pixels),
    };
  } finally {
    bitmap.close?.();
  }
}

function timeoutPromise(ms, message) {
  return new Promise((_, reject) => {
    const t = setTimeout(() => {
      clearTimeout(t);
      reject(new Error(message));
    }, Math.max(1, Number(ms) || 1));
  });
}

function isLoopclosurePose(pose) {
  return pose?.poseType === "body" || pose?.poseType === "camera";
}

export class MightyClient {
  constructor(device, opts = {}) {
    if (!device || typeof device.connect !== "function" || typeof device.disconnect !== "function") {
      throw new Error("MightyClient requires a device implementing connect/disconnect");
    }
    this.device = device;
    this.opts = { ...DEFAULT_OPTS, ...(opts || {}) };

    this._listeners = {};
    for (const key of EVENT_KEYS) this._listeners[key] = new Set();

    this._dispatcher = new protocol.FrameDispatcher((frame) => this._handleFrame(frame));
    this._running = false;
    this._streamActive = false;
    this._loopTask = null;
    this._streamStallTimer = null;
    this._streamLastActivityMs = 0;
    this._transportAbortReason = null;

    this._reqId = 1;
    this._stats = {
      rxFrames: 0,
      rxBytes: 0,
      decodeErrors: 0,
      reconnects: 0,
      commandTimeouts: 0,
    };
    this._loopclosure = null;
    this._loopclosureInit = null;
  }

  async connect() {
    if (this._running) return;
    if (this.opts.loopclosure && !this._loopclosure) {
      try {
        await this.enableLoopclosureWasm();
      } catch (err) {
        if (!this.opts.loopclosureFailOpen) throw err;
      }
    }
    this._running = true;
    this._loopTask = this._runTransportLoop();
    if (this._loopclosure) {
      const res = await this.setKeyframesEnabled(true);
      if (!res.ok) {
        this._emitError({
          scope: "loopclosure",
          code: "keyframes_failed",
          message: res.message || "failed to enable keyframes",
        });
      }
    }
  }

  async disconnect() {
    if (this._loopclosure) {
      await this.setKeyframesEnabled(false).catch((err) => {
        this._emitError({
          scope: "loopclosure",
          code: "keyframes_failed",
          message: err?.message || String(err),
          cause: err,
        });
      });
    }
    this._running = false;
    this._clearStreamStallWatchdog();
    this._transportAbortReason = null;
    try {
      await this.device.disconnect();
    } catch (err) {
      this._emitError({
        scope: "transport",
        code: "disconnect_failed",
        message: err && err.message ? err.message : "disconnect failed",
        cause: err,
      });
    }
    if (this._loopTask) {
      try {
        await this._loopTask;
      } catch (_) {
        // errors are surfaced via onError
      }
    }
    this._loopTask = null;
    this._streamActive = false;
  }

  isConnected() {
    return this._streamActive;
  }

  stats() {
    return { ...this._stats };
  }

  onImage(cb) { return this._subscribe("image", cb); }
  onPose(cb) { return this._subscribe("pose", cb); }
  onImu(cb) { return this._subscribe("imu", cb); }
  onVioState(cb) { return this._subscribe("vio_state", cb); }
  onViz(cb) { return this._subscribe("viz", cb); }
  onPointCloud(cb) { return this._subscribe("point_cloud", cb); }
  onLcon(cb) { return this._subscribe("lcon", cb); }
  onConstraints(cb) { return this._subscribe("lcon", cb); }
  onKeyframe(cb) { return this._subscribe("keyframe", cb); }
  onStatus(cb) { return this._subscribe("status", cb); }
  onLuaLog(cb) { return this._subscribe("lua_log", cb); }
  onEvent(cb) { return this._subscribe("event", cb); }
  onReset(cb) { return this._subscribe("reset", cb); }
  onLoopclosure(cb) { return this._subscribe("loopclosure", cb); }
  onAny(cb) { return this._subscribe("any", cb); }
  onError(cb) { return this._subscribe("error", cb); }

  async command(name, data = new Uint8Array()) {
    const reqId = this._allocReqId();

    if (!name || typeof name !== "string") {
      return {
        ok: false,
        reqId,
        status: 1,
        message: "command name must be a non-empty string",
        data: new Uint8Array(),
      };
    }

    if (typeof this.device.sendCommandPayload !== "function") {
      return {
        ok: false,
        reqId,
        status: 1,
        message: "device does not support command request/response",
        data: new Uint8Array(),
      };
    }

    const cmdPayload = protocol.buildCommandPayload({ reqId, name, data: toU8(data) });

    try {
      const req = this.device.sendCommandPayload(cmdPayload);
      const timeoutMs = Number(this.opts.commandTimeoutMs) || 0;
      const responsePayload = timeoutMs > 0
        ? await Promise.race([req, timeoutPromise(timeoutMs, "command timeout")])
        : await req;

      const decoded = protocol.decodeCommandResponsePayload(responsePayload);
      return {
        ok: decoded.status === 0,
        reqId: decoded.reqId,
        status: decoded.status,
        message: decoded.message,
        data: toU8(decoded.data),
      };
    } catch (err) {
      const msg = err && err.message ? err.message : "command failed";
      if (msg.toLowerCase().includes("timeout")) {
        this._stats.commandTimeouts += 1;
      }
      this._emitError({ scope: "command", code: "command_failed", message: msg, cause: err });
      return {
        ok: false,
        reqId,
        status: 1,
        message: msg,
        data: new Uint8Array(),
      };
    }
  }

  async configGet(key, opts = {}) {
    const asText = opts && opts.as === "text";
    const cfgReq = protocol.buildConfigRequestPayload({
      version: 1,
      op: protocol.CONFIG_OP.GET,
      key,
      value: new Uint8Array(),
    });

    const cmdRes = await this.command("config", cfgReq);
    if (!cmdRes.ok) {
      return {
        ok: false,
        found: false,
        key,
        value: asText ? "" : new Uint8Array(),
        message: cmdRes.message,
      };
    }

    try {
      const cfg = protocol.decodeConfigResponsePayload(cmdRes.data);
      const rawValue = toU8(cfg.value || new Uint8Array());
      return {
        ok: !!cfg.success,
        found: !!cfg.hasValue,
        key: cfg.key || key,
        value: asText ? decodeText(rawValue) : rawValue,
        message: cfg.message || "",
      };
    } catch (err) {
      const msg = err && err.message ? err.message : "invalid config response";
      this._emitError({ scope: "config", code: "decode_failed", message: msg, cause: err });
      return {
        ok: false,
        found: false,
        key,
        value: asText ? "" : new Uint8Array(),
        message: msg,
      };
    }
  }

  async configSet(key, value) {
    const valueBytes = typeof value === "string" ? encodeText(value) : toU8(value || new Uint8Array());
    const cfgReq = protocol.buildConfigRequestPayload({
      version: 1,
      op: protocol.CONFIG_OP.SET,
      key,
      value: valueBytes,
    });

    const cmdRes = await this.command("config", cfgReq);
    if (!cmdRes.ok) {
      return {
        ok: false,
        key,
        value: toU8(valueBytes),
        message: cmdRes.message,
      };
    }

    try {
      const cfg = protocol.decodeConfigResponsePayload(cmdRes.data);
      return {
        ok: !!cfg.success,
        key: cfg.key || key,
        value: toU8(cfg.value || new Uint8Array()),
        message: cfg.message || "",
      };
    } catch (err) {
      const msg = err && err.message ? err.message : "invalid config response";
      this._emitError({ scope: "config", code: "decode_failed", message: msg, cause: err });
      return {
        ok: false,
        key,
        value: toU8(valueBytes),
        message: msg,
      };
    }
  }

  async startVio() {
    return this.command("start_vio");
  }

  async stopVio() {
    return this.command("stop_vio");
  }

  async resetVioPose(pose = {}) {
    const payload = protocol.buildResetVioPosePayload(
      Array.isArray(pose) ? { positionM: pose } : pose
    );
    return this.command("reset_vio_pose", payload);
  }

  async setKeyframesEnabled(enabled) {
    return this.command("keyframes", encodeText(enabled ? "on" : "off"));
  }

  async keyframesStatus() {
    return this.command("keyframes", encodeText("status"));
  }

  async enableLoopclosureWasm(options = {}) {
    if (this._loopclosure) return this._loopclosure;
    if (this._loopclosureInit && Object.keys(options || {}).length === 0) return this._loopclosureInit;

    const init = (async () => {
      const wasmOptions = {
        ...(this.opts.loopclosureWasmOptions || {}),
        ...((options && options.wasm) || {}),
      };
      if (options && Object.prototype.hasOwnProperty.call(options, "wasmUrl")) {
        wasmOptions.wasmUrl = options.wasmUrl;
      } else if (
        this.opts.loopclosureWasmUrl
        && !wasmOptions.wasmUrl
        && !wasmOptions.locateFile
        && !wasmOptions.wasmBinary
      ) {
        wasmOptions.wasmUrl = this.opts.loopclosureWasmUrl;
      }

      const wasmModule = options.module
        || this.opts.loopclosureWasmModule
        || await createLoopClosureWasmModule(wasmOptions);
      const loopclosure = new NativeLoopClosureWasm(wasmModule, {
        onEvent: (event) => this._handleLoopclosureEvent(event),
      });
      const calibration = options.calibrationYaml ?? this.opts.loopclosureCalibrationYaml;
      if (calibration) loopclosure.setCalibrationYaml(calibration);
      this._loopclosure = loopclosure;
      return loopclosure;
    })();

    this._loopclosureInit = init;
    try {
      return await init;
    } catch (err) {
      this._logLoopclosureWasmLoadError(err);
      this._emitError({
        scope: "loopclosure",
        code: "initialize_failed",
        message: err?.message || String(err),
        cause: err,
      });
      throw err;
    } finally {
      if (!this._loopclosure) this._loopclosureInit = null;
    }
  }

  _logLoopclosureWasmLoadError(err) {
    if (err && err._mightyLoopclosureLogged) return;
    if (err) err._mightyLoopclosureLogged = true;
    const wasmUrl = err?.wasmUrl || this.opts.loopclosureWasmUrl || DEFAULT_LOOPCLOSURE_WASM_URL;
    if (typeof console !== "undefined" && typeof console.error === "function") {
      if (err?.code === "loopclosure_wasm_not_found" || err?.code === "loopclosure_module_not_found") {
        console.error(
          `Mighty SDK loop closure could not load ${wasmUrl}. ` +
          `Put mighty_loopclosure_device.wasm at ${DEFAULT_LOOPCLOSURE_WASM_URL}, ` +
          "or pass loopclosureWasmUrl with the URL where your app serves it.",
          err
        );
      } else {
        console.error(
          `Mighty SDK loop closure could not initialize ${wasmUrl}. ` +
          "The JavaScript loader and WASM binary may be incompatible, or the module is missing required exports.",
          err
        );
      }
    }
  }

  setLoopclosureCalibrationYaml(yamlOrPath) {
    this.opts.loopclosureCalibrationYaml = yamlOrPath || "";
    if (!this._loopclosure) {
      return true;
    }
    try {
      return this._loopclosure.setCalibrationYaml(yamlOrPath);
    } catch (err) {
      this._emitError({
        scope: "loopclosure",
        code: "calibration_failed",
        message: err?.message || String(err),
        cause: err,
      });
      return false;
    }
  }

  closeLoopclosure() {
    if (this._loopclosure) {
      this._loopclosure.close();
      this._loopclosure = null;
    }
    this._loopclosureInit = null;
  }

  loopclosureTrajectory() {
    return this._loopclosure ? this._loopclosure.getTrajectory() : [];
  }

  _subscribe(key, cb) {
    if (!this._listeners[key]) throw new Error(`Unknown event key: ${key}`);
    if (typeof cb !== "function") throw new Error("listener callback must be a function");
    this._listeners[key].add(cb);
    return () => {
      this._listeners[key].delete(cb);
    };
  }

  _emit(key, payload) {
    const set = this._listeners[key];
    if (!set || set.size === 0) return;
    for (const cb of Array.from(set)) {
      try {
        cb(payload);
      } catch (err) {
        if (key !== "error") {
          this._emitError({
            scope: "protocol",
            code: "listener_threw",
            message: err && err.message ? err.message : "listener callback threw",
            cause: err,
          });
        }
      }
    }
  }

  _emitAny(evt) {
    this._emit("any", evt);
  }

  _emitError(errEvt) {
    const evt = {
      scope: errEvt.scope || "protocol",
      code: errEvt.code || "unknown",
      message: errEvt.message || "unknown error",
      cause: errEvt.cause,
    };
    this._emit("error", evt);
  }

  _allocReqId() {
    const out = this._reqId >>> 0;
    this._reqId = (this._reqId + 1) >>> 0;
    if (this._reqId === 0) this._reqId = 1;
    return out;
  }

  _clearStreamStallWatchdog() {
    if (!this._streamStallTimer) return;
    clearTimeout(this._streamStallTimer);
    this._streamStallTimer = null;
  }

  _armStreamStallWatchdog() {
    this._clearStreamStallWatchdog();
    const timeoutMs = Number(this.opts.streamStallTimeoutMs) || 0;
    if (timeoutMs <= 0 || !this._running || !this._streamActive) return;
    this._streamStallTimer = setTimeout(() => {
      this._streamStallTimer = null;
      if (!this._running || !this._streamActive) return;
      const idleMs = Math.max(0, Date.now() - this._streamLastActivityMs);
      this._transportAbortReason = {
        scope: "transport",
        code: "stream_closed",
        message: `stream stalled (${idleMs}ms idle)`,
      };
      Promise.resolve(this.device.disconnect()).catch(() => {
        // The transport loop will surface the disconnect outcome.
      });
    }, timeoutMs);
  }

  _markStreamActivity() {
    this._streamLastActivityMs = Date.now();
    this._armStreamStallWatchdog();
  }

  _hasListeners(key) {
    const set = this._listeners[key];
    return !!set && set.size > 0;
  }

  _handleLoopclosureEvent(event) {
    this._emit("loopclosure", event);
    if (this._hasListeners("any")) this._emitAny({ type: "loopclosure", data: event });
  }

  _pushLoopclosureImage(image) {
    if (!this._loopclosure) return;
    try {
      this._loopclosure.pushImage(image);
    } catch (err) {
      this._emitError({
        scope: "loopclosure",
        code: "push_image_failed",
        message: err?.message || String(err),
        cause: err,
      });
    }
  }

  _pushLoopclosureJpegImage(image) {
    if (!this._loopclosure) return;
    void decodeJpegToRgbaFrame(image.data).then((decoded) => {
      if (!this._loopclosure) return;
      this._pushLoopclosureImage({
        kind: "raw",
        timestampNs: image.timestampNs,
        width: decoded.width,
        height: decoded.height,
        format: decoded.format,
        channel: image.channel,
        channelAlias: image.channelAlias,
        data: decoded.data,
      });
    }).catch((err) => {
      this._emitError({
        scope: "loopclosure",
        code: "jpeg_decode_failed",
        message: err?.message || String(err),
        cause: err,
      });
    });
  }

  _pushLoopclosurePose(pose) {
    if (!this._loopclosure || !isLoopclosurePose(pose)) return;
    try {
      this._loopclosure.pushPose(pose);
    } catch (err) {
      this._emitError({
        scope: "loopclosure",
        code: "push_pose_failed",
        message: err?.message || String(err),
        cause: err,
      });
    }
  }

  _pushLoopclosureKeyframe(keyframe) {
    if (!this._loopclosure) return;
    try {
      this._loopclosure.pushKeyframe(keyframe);
    } catch (err) {
      this._emitError({
        scope: "loopclosure",
        code: "push_keyframe_failed",
        message: err?.message || String(err),
        cause: err,
      });
    }
  }

  _applyLoopclosureCorrection(pose) {
    if (!this._loopclosure || !pose.isPublic || !isLoopclosurePose(pose)) return pose;
    return this._loopclosure.correctPose(pose);
  }

  _handleBytes(chunk) {
    const u8 = toU8(chunk);
    this._stats.rxBytes += u8.length;
    this._markStreamActivity();
    this._dispatcher.feed(u8);
  }

  _mapChannelAlias(channel) {
    if (!this.opts.normalizeChannelAliases) return undefined;
    const c = String(channel || "").trim().toLowerCase();
    if (!c) return undefined;
    if (c === "cam0" || c === "preview" || c === "left") return "cam0";
    if (c === "cam1" || c === "right") return "cam1";
    return undefined;
  }

  _mapPoseType(rawPoseType) {
    if (rawPoseType === 0) return "body";
    if (rawPoseType === 1) return "camera";
    return "other";
  }

  _handleFrame(frame) {
    this._stats.rxFrames += 1;

    const wantsAny = this._hasListeners("any");
    const wantsLoopclosure = !!this._loopclosure;

    try {
      switch (frame.type) {
        case protocol.TYPE.JPG:
        case protocol.TYPE.RJPG: {
          if (!this._hasListeners("image") && !wantsAny && !wantsLoopclosure) return;
          const jpg = protocol.decodeJpgPayload(frame.payload, frame.type === protocol.TYPE.RJPG);
          const channel = frame.type === protocol.TYPE.RJPG ? "ref" : (jpg.channel || "preview");
          const mapped = {
            kind: "jpg",
            timestampNs: jpg.timestampNs,
            channel,
            channelAlias: this._mapChannelAlias(channel),
            data: toU8(jpg.data),
          };
          this._pushLoopclosureJpegImage(mapped);
          this._emit("image", mapped);
          if (wantsAny) this._emitAny({ type: "image", data: mapped });
          return;
        }
        case protocol.TYPE.RAW: {
          if (!this._hasListeners("image") && !wantsAny && !wantsLoopclosure) return;
          const raw = protocol.decodeRawPayload(frame.payload);
          const mapped = {
            kind: "raw",
            timestampNs: raw.timestampNs,
            width: raw.width,
            height: raw.height,
            format: raw.format,
            channel: raw.channel,
            channelAlias: this._mapChannelAlias(raw.channel),
            data: toU8(raw.data),
          };
          this._pushLoopclosureImage(mapped);
          this._emit("image", mapped);
          if (wantsAny) this._emitAny({ type: "image", data: mapped });
          return;
        }
        case protocol.TYPE.SRAW: {
          if (!this._hasListeners("image") && !wantsAny && !wantsLoopclosure) return;
          const stereo = protocol.decodeStereoRawPayload(frame.payload);
          const left = {
            kind: "raw",
            timestampNs: stereo.left.timestampNs,
            width: stereo.left.width,
            height: stereo.left.height,
            format: stereo.left.format,
            channel: stereo.left.channel,
            channelAlias: this._mapChannelAlias(stereo.left.channel),
            data: toU8(stereo.left.data),
          };
          const right = {
            kind: "raw",
            timestampNs: stereo.right.timestampNs,
            width: stereo.right.width,
            height: stereo.right.height,
            format: stereo.right.format,
            channel: stereo.right.channel,
            channelAlias: this._mapChannelAlias(stereo.right.channel),
            data: toU8(stereo.right.data),
          };
          const mapped = { kind: "stereo_raw", left, right };
          this._pushLoopclosureImage(mapped);
          this._emit("image", mapped);
          if (wantsAny) this._emitAny({ type: "image", data: mapped });
          return;
        }
        case protocol.TYPE.POSE:
        case protocol.TYPE.UPOSE: {
          if (!this._hasListeners("pose") && !wantsAny && !wantsLoopclosure) return;
          const p = protocol.decodePosePayload(frame.payload);
          let mapped = {
            isPublic: frame.type === protocol.TYPE.POSE,
            packetType: frame.type === protocol.TYPE.POSE ? "POSE" : "UPOS",
            poseType: this._mapPoseType(p.poseType),
            poseTypeRaw: p.poseType,
            poseFlags: p.poseFlags,
            frameId: "odom",
            childFrameId: "base_link",
            positionM: p.positionM,
            orientationXyzw: p.orientationXyzw || undefined,
            confidence: clamp01(p.confidence),
            isKeyframe: (p.poseFlags & (1 << 1)) !== 0,
            linearVelocityBodyMps: p.linearVelocityBodyMps || undefined,
            angularVelocityBodyRps: p.angularVelocityBodyRps || undefined,
            linearAccelerationBodyMps2: p.linearAccelerationBodyMps2 || undefined,
            angularAccelerationBodyRps2: p.angularAccelerationBodyRps2 || undefined,
            timestampNs: p.timestampNs === null ? undefined : p.timestampNs,
          };
          this._pushLoopclosurePose(mapped);
          mapped = this._applyLoopclosureCorrection(mapped);
          this._emit("pose", mapped);
          if (wantsAny) this._emitAny({ type: "pose", data: mapped });
          return;
        }
        case protocol.TYPE.IMU: {
          if (!this._hasListeners("imu") && !wantsAny) return;
          const samples = protocol.decodeImuPayload(frame.payload);
          const mapped = { samples };
          this._emit("imu", mapped);
          if (wantsAny) this._emitAny({ type: "imu", data: mapped });
          return;
        }
        case protocol.TYPE.VSTA: {
          if (!this._hasListeners("vio_state") && !wantsAny) return;
          const s = protocol.decodeVioStatePayload(frame.payload);
          const mapped = {
            version: s.version,
            state: s.state,
            flags: s.flags,
            timestampNs: s.timestampNs,
            fpsCurrent: s.fpsCurrent,
            fpsAverage: s.fpsAverage,
            poseConfidence: s.poseConfidence,
            trackingRate: s.trackingRate,
            numFeatures: s.numFeatures,
            loopClosures: s.loopClosures,
            buildVersion: s.buildVersion || undefined,
            imuHzCurrent: s.version >= 3 ? s.imuHzCurrent : undefined,
            imuHzAverage5s: s.version >= 3 ? s.imuHzAverage5s : undefined,
            initReasonCode: s.version >= 4 ? s.initReasonCode : protocol.VIO_INIT_REASON.NONE,
            staticInitReasonCode: s.version >= 5 ? s.staticInitReasonCode : protocol.VIO_INIT_REASON.NONE,
            dynamicInitReasonCode: s.version >= 5 ? s.dynamicInitReasonCode : protocol.VIO_INIT_REASON.NONE,
            memoryTotalBytes: s.version >= 6 ? s.memoryTotalBytes : undefined,
            memoryUsedBytes: s.version >= 6 ? s.memoryUsedBytes : undefined,
            memoryFreeBytes: s.version >= 6 ? s.memoryFreeBytes : undefined,
            lightLevel01: s.version >= 7 ? s.lightLevel01 : undefined,
            lightRequired01: s.version >= 7 ? s.lightRequired01 : undefined,
            translationConfidence01: s.version >= 8 ? s.translationConfidence01 : undefined,
            translationObservability01: s.version >= 8 ? s.translationObservability01 : undefined,
            degradedReasonFlags: s.version >= 8 ? s.degradedReasonFlags : undefined,
          };
          this._emit("vio_state", mapped);
          if (wantsAny) this._emitAny({ type: "vio_state", data: mapped });
          return;
        }
        case protocol.TYPE.VIZ: {
          if (!this._hasListeners("viz") && !wantsAny) return;
          const v = protocol.decodeVizPayload(frame.payload);
          let mapped;
          if (v.subtype === 0) mapped = { subtype: "features", features: v.features, rawPayload: toU8(frame.payload) };
          else if (v.subtype === 1) mapped = { subtype: "detections", detections: v.detections, rawPayload: toU8(frame.payload) };
          else if (v.subtype === 2) mapped = { subtype: "matches", matches: v.matches, rawPayload: toU8(frame.payload) };
          else if (v.subtype === 3) mapped = {
            subtype: "apriltags",
            apriltags: v.apriltags || [],
            tags: v.tags || v.apriltags || [],
            rawPayload: toU8(frame.payload)
          };
          else mapped = { subtype: "unknown", rawSubtype: v.subtype, rawPayload: toU8(frame.payload) };
          this._emit("viz", mapped);
          if (wantsAny) this._emitAny({ type: "viz", data: mapped });
          return;
        }
        case protocol.TYPE.PCLD: {
          if (!this._hasListeners("point_cloud") && !wantsAny) return;
          const p = protocol.decodePcldPayload(frame.payload);
          const mapped = {
            points: p.points || [],
            pointSize: p.pointSize,
          };
          this._emit("point_cloud", mapped);
          if (wantsAny) this._emitAny({ type: "point_cloud", data: mapped });
          return;
        }
        case protocol.TYPE.LCON: {
          if (!this._hasListeners("lcon") && !wantsAny) return;
          const segs = protocol.decodeConstraintsPayload(frame.payload);
          const mapped = { segments: segs };
          this._emit("lcon", mapped);
          if (wantsAny) this._emitAny({ type: "lcon", data: mapped });
          return;
        }
        case protocol.TYPE.KEYF: {
          if (!this._hasListeners("keyframe") && !wantsAny && !wantsLoopclosure) return;
          const k = protocol.decodeKeyframePayload(frame.payload);
          const mapped = {
            timestampNs: k.timestampNs,
            descriptor: k.descriptor,
            descriptorDim: k.descriptorDim,
            descriptorType: k.descriptorType,
            flags: k.flags,
            version: k.version,
          };
          this._pushLoopclosureKeyframe(mapped);
          this._emit("keyframe", mapped);
          if (wantsAny) this._emitAny({ type: "keyframe", data: mapped });
          return;
        }
        case protocol.TYPE.STAT: {
          if (!this.opts.emitStatAsStatus) return;
          if (!this._hasListeners("status") && !wantsAny) return;
          const text = protocol.decodeStatusPayload(frame.payload);
          const mapped = { text };
          this._emit("status", mapped);
          if (wantsAny) this._emitAny({ type: "status", data: mapped });
          return;
        }
        case protocol.TYPE.LLOG: {
          if (!this._hasListeners("lua_log") && !wantsAny) return;
          const log = protocol.decodeLuaLogPayload(frame.payload);
          const mapped = { seq: log.seq, text: log.text };
          this._emit("lua_log", mapped);
          if (wantsAny) this._emitAny({ type: "lua_log", data: mapped });
          return;
        }
        case protocol.TYPE.EVNT: {
          if (!this._hasListeners("event") && !wantsAny) return;
          const event = protocol.decodeEventPayload(frame.payload);
          const mapped = {
            version: event.version,
            kind: event.kind,
            json: event.json,
            data: event.data,
          };
          this._emit("event", mapped);
          if (wantsAny) this._emitAny({ type: "event", data: mapped });
          return;
        }
        case protocol.TYPE.RSET: {
          if (!this._hasListeners("reset") && !wantsAny) return;
          const mapped = { receivedAtMs: Date.now() };
          this._emit("reset", mapped);
          if (wantsAny) this._emitAny({ type: "reset", data: mapped });
          return;
        }
        default: {
          if (wantsAny) {
            this._emitAny({ type: "unknown", rawType: frame.type, payload: toU8(frame.payload) });
          }
          return;
        }
      }
    } catch (err) {
      this._stats.decodeErrors += 1;
      this._emitError({
        scope: "protocol",
        code: "decode_failed",
        message: err && err.message ? err.message : "decode failed",
        cause: err,
      });
    }
  }

  async _runTransportLoop() {
    while (this._running) {
      this._streamActive = true;
      this._transportAbortReason = null;
      this._markStreamActivity();
      try {
        await this.device.connect((chunk) => this._handleBytes(chunk));
        this._clearStreamStallWatchdog();
        this._streamActive = false;

        if (!this._running) break;
        if (this._transportAbortReason) {
          this._emitError(this._transportAbortReason);
          this._transportAbortReason = null;
          if (!this.opts.autoReconnect) break;
          this._stats.reconnects += 1;
          await sleep(this.opts.reconnectDelayMs);
          continue;
        }
        this._emitError({
          scope: "transport",
          code: "stream_closed",
          message: "stream closed",
        });
      } catch (err) {
        this._clearStreamStallWatchdog();
        this._streamActive = false;
        if (this._transportAbortReason) {
          this._emitError(this._transportAbortReason);
          this._transportAbortReason = null;
        } else if (!this._running && isAbortError(err)) {
          break;
        } else {
          this._emitError({
            scope: "transport",
            code: "stream_error",
            message: err && err.message ? err.message : "stream error",
            cause: err,
          });
        }
      }

      if (!this._running || !this.opts.autoReconnect) break;
      this._stats.reconnects += 1;
      await sleep(this.opts.reconnectDelayMs);
    }
  }
}
