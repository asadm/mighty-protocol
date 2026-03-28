import * as protocol from "../core/protocol.js";
import { toU8, encodeText, decodeText, sleep, isAbortError } from "./utils.js";

export const VIO_STATE = protocol.VIO_STATE;
export const VIO_INIT_REASON = protocol.VIO_INIT_REASON;

const DEFAULT_OPTS = {
  commandTimeoutMs: 2000,
  autoReconnect: true,
  reconnectDelayMs: 300,
  emitStatAsStatus: true,
  normalizeChannelAliases: true,
};

const EVENT_KEYS = [
  "image",
  "pose",
  "imu",
  "vio_state",
  "viz",
  "point_cloud",
  "lcon",
  "status",
  "reset",
  "any",
  "error",
];

function clamp01(v) {
  if (!Number.isFinite(v)) return 0;
  if (v < 0) return 0;
  if (v > 1) return 1;
  return v;
}

function timeoutPromise(ms, message) {
  return new Promise((_, reject) => {
    const t = setTimeout(() => {
      clearTimeout(t);
      reject(new Error(message));
    }, Math.max(1, Number(ms) || 1));
  });
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

    this._reqId = 1;
    this._stats = {
      rxFrames: 0,
      rxBytes: 0,
      decodeErrors: 0,
      reconnects: 0,
      commandTimeouts: 0,
    };
  }

  async connect() {
    if (this._running) return;
    this._running = true;
    this._loopTask = this._runTransportLoop();
  }

  async disconnect() {
    this._running = false;
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
  onStatus(cb) { return this._subscribe("status", cb); }
  onReset(cb) { return this._subscribe("reset", cb); }
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

  _hasListeners(key) {
    const set = this._listeners[key];
    return !!set && set.size > 0;
  }

  _handleBytes(chunk) {
    const u8 = toU8(chunk);
    this._stats.rxBytes += u8.length;
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

    try {
      switch (frame.type) {
        case protocol.TYPE.RAW: {
          if (!this._hasListeners("image") && !wantsAny) return;
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
          this._emit("image", mapped);
          if (wantsAny) this._emitAny({ type: "image", data: mapped });
          return;
        }
        case protocol.TYPE.SRAW: {
          if (!this._hasListeners("image") && !wantsAny) return;
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
          this._emit("image", mapped);
          if (wantsAny) this._emitAny({ type: "image", data: mapped });
          return;
        }
        case protocol.TYPE.POSE:
        case protocol.TYPE.UPOSE: {
          if (!this._hasListeners("pose") && !wantsAny) return;
          const p = protocol.decodePosePayload(frame.payload);
          const mapped = {
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
        case protocol.TYPE.STAT: {
          if (!this.opts.emitStatAsStatus) return;
          if (!this._hasListeners("status") && !wantsAny) return;
          const text = protocol.decodeStatusPayload(frame.payload);
          const mapped = { text };
          this._emit("status", mapped);
          if (wantsAny) this._emitAny({ type: "status", data: mapped });
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
      try {
        await this.device.connect((chunk) => this._handleBytes(chunk));
        this._streamActive = false;

        if (!this._running) break;
        this._emitError({
          scope: "transport",
          code: "stream_closed",
          message: "stream closed",
        });
      } catch (err) {
        this._streamActive = false;
        if (!this._running && isAbortError(err)) break;
        this._emitError({
          scope: "transport",
          code: "stream_error",
          message: err && err.message ? err.message : "stream error",
          cause: err,
        });
      }

      if (!this._running || !this.opts.autoReconnect) break;
      this._stats.reconnects += 1;
      await sleep(this.opts.reconnectDelayMs);
    }
  }
}
