// Lightweight protocol helpers shared between Node and the browser code (no transport).
// Single implementation with light Buffer-friendly bridges for Node.

const hasBuffer = typeof Buffer !== "undefined";
const textEncoder = typeof TextEncoder !== "undefined" ? new TextEncoder() : null;
const textDecoder = typeof TextDecoder !== "undefined" ? new TextDecoder() : null;

const TYPE = {
  JPG: "JPG ",
  RJPG: "RJPG",
  RAW: "RAW ",
  SRAW: "SRAW",
  POSE: "POSE",
  UPOSE: "UPOS",
  LCON: "LCON",
  VIZ: "VIZ ",
  IMU: "IMU ",
  STAT: "STAT",
  VSTA: "VSTA",
  RSET: "RSET",
  FEA3: "FEA3",
  PCLD: "PCLD",
  KEYF: "KEYF",
  CMD: "CMD ",
  CRES: "CRES",
  CFGQ: "CFGQ",
  CFGR: "CFGR",
  LLOG: "LLOG",
  EVNT: "EVNT",
};

const RAW_FORMAT = {
  UNKNOWN: 0,
  GRAY8: 1,
  RGB24: 2,
  BGR24: 3,
  RGBA32: 4,
  BGRA32: 5,
  YUV420SP: 6,
  YUV420P: 7,
};

const CONFIG_OP = {
  GET: 0,
  SET: 1,
};

const VIO_STATE = {
  OFF: 0,
  INITIALIZING: 1,
  TRACKING: 2,
  DEGRADED: 3,
  LOST: 4,
  LOW_LIGHT: 5,
};

const VIO_DEGRADED_REASON = {
  LOW_TRACKING: 1 << 0,
  LOW_TRANSLATION_OBSERVABILITY: 1 << 1,
  LOW_PARALLAX_POSE_HOLD: 1 << 2,
  STATIONARY_POSE_HOLD: 1 << 3,
  HIGH_VELOCITY_LOW_PARALLAX: 1 << 4,
  INIT_UNCERTAIN: 1 << 5,
  STATIC_TRANSLATION_CONSTRAINED: 1 << 6,
  ROTATION_ONLY_3DOF: 1 << 7,
};

const VIO_INIT_REASON = {
  NONE: 0,
  WAITING_FOR_FIRST_IMU: 1,
  WAITING_FOR_INIT_FRAMES: 2,
  WAITING_FOR_PARALLAX: 3,
  WAITING_FOR_IMU_EXCITATION: 4,
  STATIC_INSUFFICIENT_FEATURES: 5,
  STATIC_SCENE_MOTION_TOO_HIGH: 6,
  RELATIVE_POSE_UNAVAILABLE: 7,
  GLOBAL_SFM_FAILED: 8,
  PNP_INSUFFICIENT_POINTS: 9,
  PNP_RANSAC_FAILED: 10,
  VISUAL_IMU_ALIGNMENT_FAILED: 11,
  UNKNOWN: 12,
  WAITING_FOR_FIRST_IMU_NO_SAMPLES: 13,
  WAITING_FOR_FIRST_IMU_NOT_YET_ALIGNED: 14,
  WAITING_FOR_FIRST_IMU_TIME_OFFSET_INVALID: 15,
};

const HEADER_BYTES = new Uint8Array([0xde, 0xad, 0xbe, 0xef]);
const FOOTER_BYTES = new Uint8Array([0xfe, 0xed, 0xfa, 0xce]);
const MAX_PAYLOAD_BYTES = 16 * 1024 * 1024;
const isDebugEnabled = () => {
  try {
    const root = typeof globalThis !== "undefined" ? globalThis : undefined;
    if (!root || !("localStorage" in root)) return false;
    return root.localStorage?.getItem("mightyDebugProtocol") === "1";
  } catch (_) {
    return false;
  }
};

const toU8 = (data = new Uint8Array()) => {
  if (data instanceof Uint8Array) return data;
  if (hasBuffer && data instanceof Buffer) {
    return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  }
  return new Uint8Array(data);
};

const fromU8 = (u8) =>
  hasBuffer ? Buffer.from(u8.buffer, u8.byteOffset, u8.byteLength) : u8;

const HEADER_MAGIC = fromU8(HEADER_BYTES);
const FOOTER_MAGIC = fromU8(FOOTER_BYTES);

const crcTable = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; ++n) {
    let c = n;
    for (let k = 0; k < 8; ++k) {
      c = (c & 1) ? (0xedb88320 ^ (c >>> 1)) : (c >>> 1);
    }
    t[n] = c >>> 0;
  }
  return t;
})();

function crc32(buf) {
  const u8 = toU8(buf);
  let crc = 0xffffffff;
  for (let i = 0; i < u8.length; ++i) {
    crc = (crcTable[(crc ^ u8[i]) & 0xff] ^ (crc >>> 8)) >>> 0;
  }
  return (crc ^ 0xffffffff) >>> 0;
}

function arraysEqual(a, b) {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i += 1) {
    if (a[i] !== b[i]) return false;
  }
  return true;
}

const readU32BE = (arr, off) =>
  new DataView(arr.buffer, arr.byteOffset + off, 4).getUint32(0, false);
const readU16BE = (arr, off) =>
  new DataView(arr.buffer, arr.byteOffset + off, 2).getUint16(0, false);
const readU8 = (arr, off) => arr[off];
const readF32BE = (arr, off) =>
  new DataView(arr.buffer, arr.byteOffset + off, 4).getFloat32(0, false);
const readF64BE = (arr, off) =>
  new DataView(arr.buffer, arr.byteOffset + off, 8).getFloat64(0, false);
const readBigU64BE = (arr, off) =>
  new DataView(arr.buffer, arr.byteOffset + off, 8).getBigUint64(0, false);

function makePacket(type, payload = new Uint8Array()) {
  if (!TYPE[type] && typeof type !== "string") {
    throw new Error("type must be a 4-char string or known key");
  }
  const typeStr = TYPE[type] || type;
  if (typeStr.length !== 4) throw new Error("type must be 4 chars");

  const payloadU8 = toU8(payload);
  const len = payloadU8.length >>> 0;
  const out = new Uint8Array(4 + 4 + 4 + len + 4 + 4);

  out.set(HEADER_BYTES, 0);
  for (let i = 0; i < 4; ++i) out[4 + i] = typeStr.charCodeAt(i);
  out[8] = (len >>> 24) & 0xff;
  out[9] = (len >>> 16) & 0xff;
  out[10] = (len >>> 8) & 0xff;
  out[11] = len & 0xff;
  if (len) out.set(payloadU8, 12);
  const crc = len ? crc32(payloadU8) : 0;
  out[12 + len] = (crc >>> 24) & 0xff;
  out[13 + len] = (crc >>> 16) & 0xff;
  out[14 + len] = (crc >>> 8) & 0xff;
  out[15 + len] = crc & 0xff;
  out.set(FOOTER_BYTES, 16 + len);
  return fromU8(out);
}

// Parses as many complete frames as possible; returns {frames, rest}
function parseFrames(buffer) {
  const u8 = toU8(buffer);
  let offset = 0;
  const frames = [];
  const debug = isDebugEnabled();
  const findHeader = (start) => {
    for (let i = start; i <= u8.length - 4; i += 1) {
      if (
        u8[i] === HEADER_BYTES[0] &&
        u8[i + 1] === HEADER_BYTES[1] &&
        u8[i + 2] === HEADER_BYTES[2] &&
        u8[i + 3] === HEADER_BYTES[3]
      ) {
        return i;
      }
    }
    return -1;
  };
  while (u8.length - offset >= 20) {
    if (!arraysEqual(u8.subarray(offset, offset + 4), HEADER_BYTES)) {
      offset += 1;
      continue;
    }
    const type = String.fromCharCode(
      u8[offset + 4],
      u8[offset + 5],
      u8[offset + 6],
      u8[offset + 7],
    );
    const len = readU32BE(u8, offset + 8);
    if (len > MAX_PAYLOAD_BYTES) {
      if (debug) {
        // eslint-disable-next-line no-console
        console.warn("mighty-protocol: oversized payload", len);
      }
      offset += 1;
      continue;
    }
    const pktSize = 20 + len;
    if (u8.length - offset < pktSize) {
      if (debug) {
        // eslint-disable-next-line no-console
        console.warn("mighty-protocol: incomplete frame, waiting", len);
      }
      break; // need more data
    }
    const payloadView = u8.subarray(offset + 12, offset + 12 + len);
    const recvCrc = readU32BE(u8, offset + 12 + len);
    const footer = u8.subarray(offset + 16 + len, offset + pktSize);
    const footerOk = arraysEqual(footer, FOOTER_BYTES);
    const skipCrc = (type === "RAW " || type === "SRAW") && recvCrc === 0;
    const crcOk = len && !skipCrc ? crc32(payloadView) === recvCrc : true;
    if (!footerOk || !crcOk) {
      if (debug) {
        // eslint-disable-next-line no-console
        console.warn("mighty-protocol: corrupt frame", { footerOk, crcOk, len, skipCrc });
      }
      const next = findHeader(offset + 1);
      if (next >= 0) {
        offset = next;
      } else {
        offset += 1;
      }
      continue;
    }
    frames.push({ type, payload: fromU8(payloadView) });
    offset += pktSize;
  }
  return { frames, rest: fromU8(u8.subarray(offset)) };
}

class FrameDispatcher {
  constructor(onFrame) {
    this.onFrame = onFrame;
    this.buffer = new Uint8Array();
  }
  feed(chunk) {
    const incoming = toU8(chunk);
    const merged = new Uint8Array(this.buffer.length + incoming.length);
    merged.set(this.buffer, 0);
    merged.set(incoming, this.buffer.length);
    const parsed = parseFrames(merged);
    this.buffer = toU8(parsed.rest);
    if (this.onFrame) {
      for (const f of parsed.frames) this.onFrame(f);
    }
  }
}

// ---------------------------------------------------------------------------
// Payload builders
// ---------------------------------------------------------------------------
function buildJpgPayload({ timestampNs = 0n, channel = "preview", data = new Uint8Array(), isRef = false }) {
  const dataU8 = toU8(data);
  const tsBuf = new Uint8Array(8);
  new DataView(tsBuf.buffer, tsBuf.byteOffset, tsBuf.byteLength).setBigUint64(0, BigInt(timestampNs), false);
  if (isRef) {
    const out = new Uint8Array(tsBuf.length + dataU8.length);
    out.set(tsBuf, 0);
    out.set(dataU8, tsBuf.length);
    return fromU8(out);
  }
  const chanBytes = (textEncoder || new TextEncoder()).encode(channel || "");
  const chanLen = Math.min(255, chanBytes.length);
  const out = new Uint8Array(tsBuf.length + 1 + chanLen + dataU8.length);
  let off = 0;
  out.set(tsBuf, off); off += tsBuf.length;
  out[off] = chanLen; off += 1;
  out.set(chanBytes.subarray(0, chanLen), off); off += chanLen;
  out.set(dataU8, off);
  return fromU8(out);
}

function buildRawPayload({
  timestampNs = 0n,
  width = 0,
  height = 0,
  format = RAW_FORMAT.UNKNOWN,
  channel = "preview",
  data = new Uint8Array(),
}) {
  const dataU8 = toU8(data);
  const chanBytes = (textEncoder || new TextEncoder()).encode(channel || "");
  const chanLen = Math.min(255, chanBytes.length);
  const out = new Uint8Array(8 + 4 + 4 + 1 + 1 + chanLen + dataU8.length);
  const dv = new DataView(out.buffer, out.byteOffset, out.byteLength);
  let off = 0;
  dv.setBigUint64(off, BigInt(timestampNs), false); off += 8;
  dv.setUint32(off, width >>> 0, false); off += 4;
  dv.setUint32(off, height >>> 0, false); off += 4;
  out[off] = format & 0xff; off += 1;
  out[off] = chanLen; off += 1;
  out.set(chanBytes.subarray(0, chanLen), off); off += chanLen;
  out.set(dataU8, off);
  return fromU8(out);
}

function buildStereoRawPayload({ left = {}, right = {} } = {}) {
  const leftData = toU8(left.data || new Uint8Array());
  const rightData = toU8(right.data || new Uint8Array());
  const leftChanBytes = (textEncoder || new TextEncoder()).encode(left.channel || "cam0");
  const rightChanBytes = (textEncoder || new TextEncoder()).encode(right.channel || "cam1");
  const leftChanLen = Math.min(255, leftChanBytes.length);
  const rightChanLen = Math.min(255, rightChanBytes.length);
  const headerLen = 8 + 8 +
    4 + 4 + 1 + 1 + leftChanLen +
    4 + 4 + 1 + 1 + rightChanLen +
    4 + 4;
  const out = new Uint8Array(headerLen + leftData.length + rightData.length);
  const dv = new DataView(out.buffer, out.byteOffset, out.byteLength);
  let off = 0;
  dv.setBigUint64(off, BigInt(left.timestampNs || 0n), false); off += 8;
  dv.setBigUint64(off, BigInt(right.timestampNs || 0n), false); off += 8;
  dv.setUint32(off, (left.width || 0) >>> 0, false); off += 4;
  dv.setUint32(off, (left.height || 0) >>> 0, false); off += 4;
  out[off] = (left.format ?? RAW_FORMAT.UNKNOWN) & 0xff; off += 1;
  out[off] = leftChanLen; off += 1;
  out.set(leftChanBytes.subarray(0, leftChanLen), off); off += leftChanLen;
  dv.setUint32(off, (right.width || 0) >>> 0, false); off += 4;
  dv.setUint32(off, (right.height || 0) >>> 0, false); off += 4;
  out[off] = (right.format ?? RAW_FORMAT.UNKNOWN) & 0xff; off += 1;
  out[off] = rightChanLen; off += 1;
  out.set(rightChanBytes.subarray(0, rightChanLen), off); off += rightChanLen;
  dv.setUint32(off, leftData.length >>> 0, false); off += 4;
  dv.setUint32(off, rightData.length >>> 0, false); off += 4;
  out.set(leftData, off); off += leftData.length;
  out.set(rightData, off);
  return fromU8(out);
}

		function buildPosePayload({
		  poseType = 0,
		  poseFlags = 0,
		  positionM = [0, 0, 0],
		  orientationXyzw = null,
		  confidence = 1.0,
		  linearVelocityBodyMps = null,
		  angularVelocityBodyRps = null,
		  linearAccelerationBodyMps2 = null,
		  angularAccelerationBodyRps2 = null,
		  timestampNs = null,
		} = {}) {
		  const hasQuat = Array.isArray(orientationXyzw) && orientationXyzw.length === 4;
		  const hasLinVel = Array.isArray(linearVelocityBodyMps) && linearVelocityBodyMps.length === 3;
		  const hasAngVel = Array.isArray(angularVelocityBodyRps) && angularVelocityBodyRps.length === 3;
		  const hasLinAcc = Array.isArray(linearAccelerationBodyMps2) && linearAccelerationBodyMps2.length === 3;
		  const hasAngAcc = Array.isArray(angularAccelerationBodyRps2) && angularAccelerationBodyRps2.length === 3;
		  let tsVal = 0n;
		  let hasTs = false;
		  if (timestampNs !== null && timestampNs !== undefined) {
		    try {
		      tsVal = typeof timestampNs === "bigint" ? timestampNs : BigInt(timestampNs);
		      hasTs = tsVal > 0n;
		    } catch (_) {
		      hasTs = false;
		      tsVal = 0n;
		    }
		  }
		  let flags = poseFlags;
		  if (hasQuat) flags |= 0x1;
		  if (hasLinVel) flags |= (1 << 2);
		  if (hasAngVel) flags |= (1 << 3);
		  if (hasLinAcc) flags |= (1 << 4);
		  if (hasAngAcc) flags |= (1 << 5);
		  if (hasTs) flags |= (1 << 6);
		  // NOTE: Keep payload append-only for backward compatibility; new fields go at the end.
		  const len = 4 + 4 + 8 * 3 + (hasQuat ? 8 * 4 : 0) + 4 +
		              (hasLinVel ? 8 * 3 : 0) +
		              (hasAngVel ? 8 * 3 : 0) +
		              (hasLinAcc ? 8 * 3 : 0) +
		              (hasAngAcc ? 8 * 3 : 0) +
		              (hasTs ? 8 : 0);
		  const buf = new Uint8Array(len);
		  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
		  let off = 0;
	  dv.setUint32(off, poseType >>> 0, false); off += 4;
	  dv.setUint32(off, flags >>> 0, false); off += 4;
	  dv.setFloat64(off, positionM[0], false); off += 8;
	  dv.setFloat64(off, positionM[1], false); off += 8;
	  dv.setFloat64(off, positionM[2], false); off += 8;
	  if (hasQuat) {
	    dv.setFloat64(off, orientationXyzw[0], false); off += 8;
	    dv.setFloat64(off, orientationXyzw[1], false); off += 8;
	    dv.setFloat64(off, orientationXyzw[2], false); off += 8;
	    dv.setFloat64(off, orientationXyzw[3], false); off += 8;
	  }
	  let c = Number(confidence);
	  if (!Number.isFinite(c)) c = 0.0;
	  if (c < 0) c = 0.0;
	  if (c > 1) c = 1.0;
	  dv.setFloat32(off, c, false);
	  off += 4;

	  const writeVec3 = (v) => {
	    dv.setFloat64(off, v[0], false); off += 8;
	    dv.setFloat64(off, v[1], false); off += 8;
	    dv.setFloat64(off, v[2], false); off += 8;
	  };
		  if (hasLinVel) writeVec3(linearVelocityBodyMps);
		  if (hasAngVel) writeVec3(angularVelocityBodyRps);
		  if (hasLinAcc) writeVec3(linearAccelerationBodyMps2);
		  if (hasAngAcc) writeVec3(angularAccelerationBodyRps2);
		  if (hasTs) {
		    dv.setBigUint64(off, tsVal, false); off += 8;
		  }
		  return fromU8(buf);
		}

function buildConstraintsPayload(segments = []) {
  const per = 1 + 6 * 4;
  const buf = new Uint8Array(4 + segments.length * per);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  dv.setUint32(0, segments.length >>> 0, false);
  let off = 4;
  for (const s of segments) {
    dv.setUint8(off, s.type || 0); off += 1;
    for (let i = 0; i < 3; ++i) { dv.setFloat32(off, s.start?.[i] ?? 0, false); off += 4; }
    for (let i = 0; i < 3; ++i) { dv.setFloat32(off, s.end?.[i] ?? 0, false); off += 4; }
  }
  return fromU8(buf);
}

function buildVizPayload(viz) {
  const subtype = viz.subtype ?? 0;
  let body = new Uint8Array();
  if (subtype === 0) {
    body = new Uint8Array(viz.features.length * (2 + 2 + 1 + 2));
    const dv = new DataView(body.buffer, body.byteOffset, body.byteLength);
    let off = 0;
    for (const f of viz.features) {
      dv.setUint16(off, f.x, false); off += 2;
      dv.setUint16(off, f.y, false); off += 2;
      dv.setUint8(off, f.status || 0); off += 1;
      dv.setUint16(off, f.id, false); off += 2;
    }
  } else if (subtype === 1) {
    const parts = [];
    let total = 0;
    for (const d of viz.detections) {
      const lbl = (textEncoder || new TextEncoder()).encode(d.label || "");
      const ll = Math.min(255, lbl.length);
      const b = new Uint8Array(2 + 2 + 2 + 2 + 1 + ll);
      const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
      let off = 0;
      dv.setUint16(off, d.x1, false); off += 2;
      dv.setUint16(off, d.y1, false); off += 2;
      dv.setUint16(off, d.x2, false); off += 2;
      dv.setUint16(off, d.y2, false); off += 2;
      dv.setUint8(off, ll); off += 1;
      b.set(lbl.subarray(0, ll), off);
      parts.push(b);
      total += b.length;
    }
    body = new Uint8Array(total);
    let off = 0;
    for (const p of parts) { body.set(p, off); off += p.length; }
  } else if (subtype === 2) {
    body = new Uint8Array(viz.matches.length * (2 + 2 + 2 + 2 + 1));
    const dv = new DataView(body.buffer, body.byteOffset, body.byteLength);
    let off = 0;
    for (const m of viz.matches) {
      dv.setUint16(off, m.x1, false); off += 2;
      dv.setUint16(off, m.y1, false); off += 2;
      dv.setUint16(off, m.x2, false); off += 2;
      dv.setUint16(off, m.y2, false); off += 2;
      dv.setUint8(off, m.confidence || 0); off += 1;
    }
  } else if (subtype === 3) {
    const tags = viz.apriltags || viz.tags || [];
    body = new Uint8Array(tags.length * (4 + 1 + 2 * 4 + 8 * 4));
    const dv = new DataView(body.buffer, body.byteOffset, body.byteLength);
    let off = 0;
    for (const tag of tags) {
      dv.setUint32(off, tag.id >>> 0, false); off += 4;
      dv.setUint8(off, tag.hamming || 0); off += 1;
      const center = tag.center || [tag.centerX || tag.center_x || 0, tag.centerY || tag.center_y || 0];
      dv.setFloat32(off, center[0] || 0, false); off += 4;
      dv.setFloat32(off, center[1] || 0, false); off += 4;
      const corners = tag.corners || [];
      for (let i = 0; i < 4; i += 1) {
        const p = corners[i] || [];
        dv.setFloat32(off, p[0] || 0, false); off += 4;
        dv.setFloat32(off, p[1] || 0, false); off += 4;
      }
    }
  } else {
    throw new Error("unknown viz subtype");
  }
  const header = new Uint8Array(3);
  const hdv = new DataView(header.buffer, header.byteOffset, header.byteLength);
  hdv.setUint8(0, subtype);
  hdv.setUint16(
    1,
    subtype === 0 ? viz.features.length :
      subtype === 1 ? viz.detections.length :
        subtype === 2 ? viz.matches.length : (viz.apriltags || viz.tags || []).length,
    false,
  );
  const out = new Uint8Array(header.length + body.length);
  out.set(header, 0);
  out.set(body, header.length);
  return fromU8(out);
}

function buildImuPayload(samples = []) {
  if (!samples.length) return fromU8(new Uint8Array());
  const stride = 8 + 6 * 8;
  const buf = new Uint8Array(4 + samples.length * stride);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  dv.setUint32(0, samples.length >>> 0, false);
  let off = 4;
  for (const s of samples) {
    dv.setBigUint64(off, BigInt(s.timestampNs), false); off += 8;
    dv.setFloat64(off, s.ax, false); off += 8;
    dv.setFloat64(off, s.ay, false); off += 8;
    dv.setFloat64(off, s.az, false); off += 8;
    dv.setFloat64(off, s.gx, false); off += 8;
    dv.setFloat64(off, s.gy, false); off += 8;
    dv.setFloat64(off, s.gz, false); off += 8;
  }
  return fromU8(buf);
}

const buildStatusPayload = (text = "") =>
  fromU8((textEncoder || new TextEncoder()).encode(text));

function buildEventPayload({ kind = "", json = "", data = undefined } = {}) {
  const encoder = textEncoder || new TextEncoder();
  const kindBytes = encoder.encode(String(kind || ""));
  const jsonText = data !== undefined ? JSON.stringify(data) : String(json || "");
  const jsonBytes = encoder.encode(jsonText);
  const kindLen = Math.min(255, kindBytes.length);
  const buf = new Uint8Array(1 + 1 + kindLen + 4 + jsonBytes.length);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint8(off, 1); off += 1;
  dv.setUint8(off, kindLen); off += 1;
  buf.set(kindBytes.subarray(0, kindLen), off); off += kindLen;
  dv.setUint32(off, jsonBytes.length >>> 0, false); off += 4;
  buf.set(jsonBytes, off);
  return fromU8(buf);
}

function buildKeyframePayload({
  timestampNs = 0n,
  descriptor = [],
  flags = 0,
  version = 1,
  descriptorType = 1,
} = {}) {
  const values = descriptor instanceof Float32Array ? descriptor : Array.from(descriptor || []);
  const dim = values.length >>> 0;
  const buf = new Uint8Array(1 + 1 + 2 + 8 + 4 + dim * 4);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint8(off, version & 0xff); off += 1;
  dv.setUint8(off, descriptorType & 0xff); off += 1;
  dv.setUint16(off, flags & 0xffff, false); off += 2;
  dv.setBigUint64(off, BigInt(timestampNs || 0), false); off += 8;
  dv.setUint32(off, dim, false); off += 4;
  for (let i = 0; i < dim; i += 1) {
    const v = Number(values[i]);
    dv.setFloat32(off, Number.isFinite(v) ? v : 0, false);
    off += 4;
  }
  return fromU8(buf);
}

function buildVioStatePayload({
  version = 1,
  state = 0,
  flags = 0,
  timestampNs = 0n,
  fpsCurrent = 0.0,
  fpsAverage = 0.0,
  poseConfidence = 0.0,
  trackingRate = 0.0,
  numFeatures = 0,
  loopClosures = 0,
  imuHzCurrent = undefined,
  imuHzAverage5s = undefined,
  initReasonCode = undefined,
  staticInitReasonCode = undefined,
  dynamicInitReasonCode = undefined,
  memoryTotalBytes = undefined,
  memoryUsedBytes = undefined,
  memoryFreeBytes = undefined,
  lightLevel01 = undefined,
  lightRequired01 = undefined,
  translationConfidence01 = undefined,
  translationObservability01 = undefined,
  degradedReasonFlags = undefined,
  buildVersion = "",
} = {}) {
  const enc = textEncoder || new TextEncoder();
  const buildBytes = typeof buildVersion === "string" ? enc.encode(buildVersion) : new Uint8Array();
  const buildLen = Math.min(255, buildBytes.length);
  const hasImuHzCurrent = typeof imuHzCurrent === "number" && Number.isFinite(imuHzCurrent);
  const hasImuHzAverage = typeof imuHzAverage5s === "number" && Number.isFinite(imuHzAverage5s);
  const hasInitReasonCode = typeof initReasonCode === "number" && Number.isFinite(initReasonCode);
  const hasStaticInitReasonCode =
    typeof staticInitReasonCode === "number" && Number.isFinite(staticInitReasonCode);
  const hasDynamicInitReasonCode =
    typeof dynamicInitReasonCode === "number" && Number.isFinite(dynamicInitReasonCode);
  const hasMemoryTotalBytes =
    (typeof memoryTotalBytes === "bigint" && memoryTotalBytes >= 0n) ||
    (typeof memoryTotalBytes === "number" && Number.isFinite(memoryTotalBytes) && memoryTotalBytes >= 0);
  const hasMemoryUsedBytes =
    (typeof memoryUsedBytes === "bigint" && memoryUsedBytes >= 0n) ||
    (typeof memoryUsedBytes === "number" && Number.isFinite(memoryUsedBytes) && memoryUsedBytes >= 0);
  const hasMemoryFreeBytes =
    (typeof memoryFreeBytes === "bigint" && memoryFreeBytes >= 0n) ||
    (typeof memoryFreeBytes === "number" && Number.isFinite(memoryFreeBytes) && memoryFreeBytes >= 0);
  const hasLightLevel = typeof lightLevel01 === "number" && Number.isFinite(lightLevel01);
  const hasLightRequired = typeof lightRequired01 === "number" && Number.isFinite(lightRequired01);
  const hasTranslationConfidence =
    typeof translationConfidence01 === "number" && Number.isFinite(translationConfidence01);
  const hasTranslationObservability =
    typeof translationObservability01 === "number" && Number.isFinite(translationObservability01);
  const hasDegradedReasonFlags =
    typeof degradedReasonFlags === "number" && Number.isFinite(degradedReasonFlags);
  const toBigUint64 = (value) =>
    typeof value === "bigint" ? value : BigInt(Math.floor(Number(value) || 0));
  let ver = version & 0xff;
  if (buildLen > 0 && ver < 2) ver = 2;
  if ((hasImuHzCurrent || hasImuHzAverage) && ver < 3) ver = 3;
  if (hasInitReasonCode && ver < 4) ver = 4;
  if ((hasStaticInitReasonCode || hasDynamicInitReasonCode) && ver < 5) ver = 5;
  if ((hasMemoryTotalBytes || hasMemoryUsedBytes || hasMemoryFreeBytes) && ver < 6) ver = 6;
  if ((hasLightLevel || hasLightRequired) && ver < 7) ver = 7;
  if ((hasTranslationConfidence || hasTranslationObservability || hasDegradedReasonFlags) && ver < 8) ver = 8;
  const buf = new Uint8Array((1 + 1 + 2 + 8 + 4 + 4 + 4 + 4 + 4 + 4) +
    (ver >= 2 ? (1 + buildLen) : 0) +
    (ver >= 3 ? (4 + 4) : 0) +
    (ver >= 4 ? 1 : 0) +
    (ver >= 5 ? 2 : 0) +
    (ver >= 6 ? (8 + 8 + 8) : 0) +
    (ver >= 7 ? (4 + 4) : 0) +
    (ver >= 8 ? (4 + 4 + 4) : 0));
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint8(off, ver); off += 1;
  dv.setUint8(off, state & 0xff); off += 1;
  dv.setUint16(off, flags & 0xffff, false); off += 2;
  dv.setBigUint64(off, BigInt(timestampNs || 0n), false); off += 8;
  dv.setFloat32(off, Number(fpsCurrent) || 0.0, false); off += 4;
  dv.setFloat32(off, Number(fpsAverage) || 0.0, false); off += 4;
  dv.setFloat32(off, Number(poseConfidence) || 0.0, false); off += 4;
  dv.setFloat32(off, Number(trackingRate) || 0.0, false); off += 4;
  dv.setUint32(off, (Number(numFeatures) || 0) >>> 0, false); off += 4;
  dv.setUint32(off, (Number(loopClosures) || 0) >>> 0, false); off += 4;
  if (ver >= 2) {
    dv.setUint8(off, buildLen); off += 1;
    if (buildLen > 0) {
      buf.set(buildBytes.subarray(0, buildLen), off);
      off += buildLen;
    }
  }
  if (ver >= 3) {
    dv.setFloat32(off, hasImuHzCurrent ? Number(imuHzCurrent) : 0.0, false); off += 4;
    dv.setFloat32(off, hasImuHzAverage ? Number(imuHzAverage5s) : 0.0, false); off += 4;
  }
  if (ver >= 4) {
    dv.setUint8(off, hasInitReasonCode ? (Number(initReasonCode) & 0xff) : 0); off += 1;
  }
  if (ver >= 5) {
    dv.setUint8(off, hasStaticInitReasonCode ? (Number(staticInitReasonCode) & 0xff) : 0); off += 1;
    dv.setUint8(off, hasDynamicInitReasonCode ? (Number(dynamicInitReasonCode) & 0xff) : 0); off += 1;
  }
  if (ver >= 6) {
    dv.setBigUint64(off, hasMemoryTotalBytes ? toBigUint64(memoryTotalBytes) : 0n, false); off += 8;
    dv.setBigUint64(off, hasMemoryUsedBytes ? toBigUint64(memoryUsedBytes) : 0n, false); off += 8;
    dv.setBigUint64(off, hasMemoryFreeBytes ? toBigUint64(memoryFreeBytes) : 0n, false); off += 8;
  }
  if (ver >= 7) {
    dv.setFloat32(off, hasLightLevel ? Number(lightLevel01) : 1.0, false); off += 4;
    dv.setFloat32(off, hasLightRequired ? Number(lightRequired01) : 1.0, false); off += 4;
  }
  if (ver >= 8) {
    dv.setFloat32(off, hasTranslationConfidence ? Number(translationConfidence01) : 1.0, false); off += 4;
    dv.setFloat32(off, hasTranslationObservability ? Number(translationObservability01) : 1.0, false); off += 4;
    dv.setUint32(off, hasDegradedReasonFlags ? (Number(degradedReasonFlags) >>> 0) : 0, false); off += 4;
  }
  return fromU8(buf);
}

function buildFea3Payload(features = []) {
  const buf = new Uint8Array(2 + features.length * (2 + 8 * 3));
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  dv.setUint16(0, features.length, false);
  let off = 2;
  for (const f of features) {
    dv.setUint16(off, f.id, false); off += 2;
    dv.setFloat64(off, f.x, false); off += 8;
    dv.setFloat64(off, f.y, false); off += 8;
    dv.setFloat64(off, f.z, false); off += 8;
  }
  return fromU8(buf);
}

function buildPcldPayload(points = [], pointSize = null) {
  const includeSize = typeof pointSize === "number" && pointSize > 0;
  const buf = new Uint8Array((includeSize ? 8 : 4) + points.length * (4 * 3 + 3));
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  dv.setUint32(0, points.length >>> 0, false);
  let off = 4;
  if (includeSize) {
    dv.setFloat32(off, pointSize, false); off += 4;
  }
  for (const p of points) {
    dv.setFloat32(off, p.x, false); off += 4;
    dv.setFloat32(off, p.y, false); off += 4;
    dv.setFloat32(off, p.z, false); off += 4;
    dv.setUint8(off, p.r, false); off += 1;
    dv.setUint8(off, p.g, false); off += 1;
    dv.setUint8(off, p.b, false); off += 1;
  }
  return fromU8(buf);
}

function buildCommandPayload({ reqId = 0, name = "", data = new Uint8Array() } = {}) {
  const dataU8 = toU8(data);
  const nameBytes = (textEncoder || new TextEncoder()).encode(name || "");
  const nameLen = Math.min(255, nameBytes.length);
  const buf = new Uint8Array(4 + 1 + nameLen + 4 + dataU8.length);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint32(off, reqId >>> 0, false); off += 4;
  dv.setUint8(off, nameLen); off += 1;
  buf.set(nameBytes.subarray(0, nameLen), off); off += nameLen;
  dv.setUint32(off, dataU8.length >>> 0, false); off += 4;
  buf.set(dataU8, off);
  return fromU8(buf);
}

function buildResetVioPosePayload({ positionM = [0, 0, 0], orientationXyzw = null } = {}) {
  return buildPosePayload({
    poseType: 0,
    positionM,
    orientationXyzw,
    confidence: 1.0,
  });
}

function buildCommandResponsePayload({ reqId = 0, status = 0, message = "", data = new Uint8Array() } = {}) {
  const msgBytes = (textEncoder || new TextEncoder()).encode(message || "");
  const msgLen = Math.min(65535, msgBytes.length);
  const dataU8 = toU8(data);
  const buf = new Uint8Array(4 + 1 + 2 + msgLen + 4 + dataU8.length);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint32(off, reqId >>> 0, false); off += 4;
  dv.setUint8(off, status & 0xff); off += 1;
  dv.setUint16(off, msgLen, false); off += 2;
  buf.set(msgBytes.subarray(0, msgLen), off); off += msgLen;
  dv.setUint32(off, dataU8.length >>> 0, false); off += 4;
  buf.set(dataU8, off);
  return fromU8(buf);
}

function buildLuaLogPayload({ seq = 0, text = "" } = {}) {
  const textBytes = (textEncoder || new TextEncoder()).encode(text || "");
  const buf = new Uint8Array(4 + textBytes.length);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  dv.setUint32(0, seq >>> 0, false);
  buf.set(textBytes, 4);
  return fromU8(buf);
}

function buildConfigRequestPayload({ version = 1, op = CONFIG_OP.GET, key = "", value = new Uint8Array() } = {}) {
  const keyBytes = (textEncoder || new TextEncoder()).encode(key || "");
  const keyLen = Math.min(255, keyBytes.length);
  const valueU8 = toU8(value);
  const buf = new Uint8Array(1 + 1 + 1 + keyLen + 4 + valueU8.length);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint8(off, version & 0xff); off += 1;
  dv.setUint8(off, op & 0xff); off += 1;
  dv.setUint8(off, keyLen); off += 1;
  buf.set(keyBytes.subarray(0, keyLen), off); off += keyLen;
  dv.setUint32(off, valueU8.length >>> 0, false); off += 4;
  buf.set(valueU8, off);
  return fromU8(buf);
}

function buildConfigResponsePayload({
  version = 1,
  op = CONFIG_OP.GET,
  success = 0,
  hasValue = false,
  key = "",
  message = "",
  value = new Uint8Array(),
} = {}) {
  const keyBytes = (textEncoder || new TextEncoder()).encode(key || "");
  const keyLen = Math.min(255, keyBytes.length);
  const msgBytes = (textEncoder || new TextEncoder()).encode(message || "");
  const msgLen = Math.min(65535, msgBytes.length);
  const valueU8 = toU8(value);
  const buf = new Uint8Array(1 + 1 + 1 + 1 + 1 + keyLen + 2 + msgLen + 4 + valueU8.length);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint8(off, version & 0xff); off += 1;
  dv.setUint8(off, op & 0xff); off += 1;
  dv.setUint8(off, success ? 1 : 0); off += 1;
  dv.setUint8(off, hasValue ? 1 : 0); off += 1;
  dv.setUint8(off, keyLen); off += 1;
  buf.set(keyBytes.subarray(0, keyLen), off); off += keyLen;
  dv.setUint16(off, msgLen, false); off += 2;
  buf.set(msgBytes.subarray(0, msgLen), off); off += msgLen;
  dv.setUint32(off, valueU8.length >>> 0, false); off += 4;
  buf.set(valueU8, off);
  return fromU8(buf);
}

// ---------------------------------------------------------------------------
// Payload decoders
// ---------------------------------------------------------------------------
function decodeJpgPayload(payload, isRef = false) {
  const u8 = toU8(payload);
  if (u8.length < 8) throw new Error("JPG payload too short");
  let off = 0;
  const ts = readBigU64BE(u8, off); off += 8;
  if (isRef) {
    return { timestampNs: ts, channel: "", data: fromU8(u8.subarray(off)) };
  }
  const clen = readU8(u8, off); off += 1;
  const channel = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + clen)); off += clen;
  return { timestampNs: ts, channel, data: fromU8(u8.subarray(off)) };
}

function decodeRawPayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 8 + 4 + 4 + 1 + 1) throw new Error("RAW payload too short");
  let off = 0;
  const timestampNs = readBigU64BE(u8, off); off += 8;
  const width = readU32BE(u8, off); off += 4;
  const height = readU32BE(u8, off); off += 4;
  const format = readU8(u8, off); off += 1;
  const clen = readU8(u8, off); off += 1;
  if (u8.length < off + clen) throw new Error("RAW payload truncated");
  const channel = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + clen)); off += clen;
  const data = fromU8(u8.subarray(off));
  return { timestampNs, width, height, format, channel, data };
}

function decodeStereoRawPayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 8 + 8 + 4 + 4 + 1 + 1 + 4 + 4 + 1 + 1 + 4 + 4) {
    throw new Error("SRAW payload too short");
  }
  let off = 0;
  const leftTimestampNs = readBigU64BE(u8, off); off += 8;
  const rightTimestampNs = readBigU64BE(u8, off); off += 8;
  const leftWidth = readU32BE(u8, off); off += 4;
  const leftHeight = readU32BE(u8, off); off += 4;
  const leftFormat = readU8(u8, off); off += 1;
  const leftClen = readU8(u8, off); off += 1;
  if (u8.length < off + leftClen) throw new Error("SRAW payload truncated");
  const leftChannel = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + leftClen)); off += leftClen;
  if (u8.length < off + 4 + 4 + 1 + 1) throw new Error("SRAW payload truncated");
  const rightWidth = readU32BE(u8, off); off += 4;
  const rightHeight = readU32BE(u8, off); off += 4;
  const rightFormat = readU8(u8, off); off += 1;
  const rightClen = readU8(u8, off); off += 1;
  if (u8.length < off + rightClen) throw new Error("SRAW payload truncated");
  const rightChannel = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + rightClen)); off += rightClen;
  if (u8.length < off + 8) throw new Error("SRAW payload truncated");
  const leftLen = readU32BE(u8, off); off += 4;
  const rightLen = readU32BE(u8, off); off += 4;
  if (u8.length < off + leftLen + rightLen) throw new Error("SRAW payload truncated");
  const leftData = fromU8(u8.subarray(off, off + leftLen)); off += leftLen;
  const rightData = fromU8(u8.subarray(off, off + rightLen));
  return {
    left: {
      timestampNs: leftTimestampNs,
      width: leftWidth,
      height: leftHeight,
      format: leftFormat,
      channel: leftChannel,
      data: leftData,
    },
    right: {
      timestampNs: rightTimestampNs,
      width: rightWidth,
      height: rightHeight,
      format: rightFormat,
      channel: rightChannel,
      data: rightData,
    },
  };
}

		function decodePosePayload(payload) {
		  const u8 = toU8(payload);
		  let off = 0;
		  const poseType = readU32BE(u8, off); off += 4;
		  const poseFlags = readU32BE(u8, off); off += 4;
	  const x = readF64BE(u8, off); off += 8;
	  const y = readF64BE(u8, off); off += 8;
	  const z = readF64BE(u8, off); off += 8;
	  let orientationXyzw = null;
	  if (poseFlags & 0x1) {
	    orientationXyzw = [
	      readF64BE(u8, off), readF64BE(u8, off + 8),
	      readF64BE(u8, off + 16), readF64BE(u8, off + 24),
	    ];
	    off += 32;
	  }
	  let confidence = 1.0;
	  if (u8.length >= off + 4) {
	    confidence = readF32BE(u8, off);
	  }
	  if (!Number.isFinite(confidence)) confidence = 0.0;
	  if (confidence < 0) confidence = 0.0;
	  if (confidence > 1) confidence = 1.0;
	  off += 4;

	  const readVec3 = () => {
	    if (u8.length < off + 24) return null;
	    const v = [readF64BE(u8, off), readF64BE(u8, off + 8), readF64BE(u8, off + 16)];
	    off += 24;
	    return v;
	  };
		  const linearVelocityBodyMps = (poseFlags & (1 << 2)) ? readVec3() : null;
		  const angularVelocityBodyRps = (poseFlags & (1 << 3)) ? readVec3() : null;
		  const linearAccelerationBodyMps2 = (poseFlags & (1 << 4)) ? readVec3() : null;
		  const angularAccelerationBodyRps2 = (poseFlags & (1 << 5)) ? readVec3() : null;
		  const timestampNs =
		    (poseFlags & (1 << 6)) && u8.length >= off + 8 ? readBigU64BE(u8, off) : null;
		  if (timestampNs !== null) off += 8;

		  return {
		    poseType,
		    poseFlags,
		    positionM: [x, y, z],
		    orientationXyzw,
		    confidence,
		    linearVelocityBodyMps,
		    angularVelocityBodyRps,
		    linearAccelerationBodyMps2,
		    angularAccelerationBodyRps2,
		    timestampNs,
		  };
		}

function decodeConstraintsPayload(payload) {
  const u8 = toU8(payload);
  let off = 0;
  const count = readU32BE(u8, off); off += 4;
  const segments = [];
  for (let i = 0; i < count; ++i) {
    const type = readU8(u8, off); off += 1;
    const start = [readF32BE(u8, off), readF32BE(u8, off + 4), readF32BE(u8, off + 8)]; off += 12;
    const end = [readF32BE(u8, off), readF32BE(u8, off + 4), readF32BE(u8, off + 8)]; off += 12;
    segments.push({ type, start, end });
  }
  return segments;
}

function decodeVizPayload(payload) {
  const u8 = toU8(payload);
  let off = 0;
  const subtype = readU8(u8, off); off += 1;
  const count = readU16BE(u8, off); off += 2;
  if (subtype === 0) {
    const features = [];
    for (let i = 0; i < count; ++i) {
      const x = readU16BE(u8, off); off += 2;
      const y = readU16BE(u8, off); off += 2;
      const status = readU8(u8, off); off += 1;
      const id = readU16BE(u8, off); off += 2;
      features.push({ x, y, status, id });
    }
    return { subtype, features };
  }
  if (subtype === 1) {
    const detections = [];
    for (let i = 0; i < count; ++i) {
      const x1 = readU16BE(u8, off); off += 2;
      const y1 = readU16BE(u8, off); off += 2;
      const x2 = readU16BE(u8, off); off += 2;
      const y2 = readU16BE(u8, off); off += 2;
      const ll = readU8(u8, off); off += 1;
      const label = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + ll)); off += ll;
      detections.push({ x1, y1, x2, y2, label });
    }
    return { subtype, detections };
  }
  if (subtype === 2) {
    const matches = [];
    for (let i = 0; i < count; ++i) {
      const x1 = readU16BE(u8, off); off += 2;
      const y1 = readU16BE(u8, off); off += 2;
      const x2 = readU16BE(u8, off); off += 2;
      const y2 = readU16BE(u8, off); off += 2;
      const confidence = readU8(u8, off); off += 1;
      matches.push({ x1, y1, x2, y2, confidence });
    }
    return { subtype, matches };
  }
  if (subtype === 3) {
    const apriltags = [];
    for (let i = 0; i < count; ++i) {
      const id = readU32BE(u8, off); off += 4;
      const hamming = readU8(u8, off); off += 1;
      const center = [readF32BE(u8, off), readF32BE(u8, off + 4)]; off += 8;
      const corners = [];
      for (let c = 0; c < 4; c += 1) {
        corners.push([readF32BE(u8, off), readF32BE(u8, off + 4)]);
        off += 8;
      }
      apriltags.push({ id, hamming, center, corners });
    }
    return { subtype, apriltags, tags: apriltags };
  }
  throw new Error("Unknown viz subtype");
}

function decodeImuPayload(payload) {
  const u8 = toU8(payload);
  let off = 0;
  const count = readU32BE(u8, off); off += 4;
  const samples = [];
  for (let i = 0; i < count; ++i) {
    const timestampNs = readBigU64BE(u8, off); off += 8;
    const ax = readF64BE(u8, off); off += 8;
    const ay = readF64BE(u8, off); off += 8;
    const az = readF64BE(u8, off); off += 8;
    const gx = readF64BE(u8, off); off += 8;
    const gy = readF64BE(u8, off); off += 8;
    const gz = readF64BE(u8, off); off += 8;
    samples.push({ timestampNs, ax, ay, az, gx, gy, gz });
  }
  return samples;
}

const decodeStatusPayload = (payload) =>
  (textDecoder || new TextDecoder()).decode(toU8(payload));

function decodeEventPayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 1 + 1 + 4) throw new Error("EVNT payload too short");
  const decoder = textDecoder || new TextDecoder();
  let off = 0;
  const version = readU8(u8, off); off += 1;
  const kindLen = readU8(u8, off); off += 1;
  if (u8.length < off + kindLen + 4) throw new Error("EVNT payload truncated");
  const kind = decoder.decode(u8.subarray(off, off + kindLen)); off += kindLen;
  const jsonLen = readU32BE(u8, off); off += 4;
  if (u8.length < off + jsonLen) throw new Error("EVNT json truncated");
  const json = decoder.decode(u8.subarray(off, off + jsonLen));
  let data = null;
  if (json) {
    try {
      data = JSON.parse(json);
    } catch (_) {
      data = null;
    }
  }
  return { version, kind, json, data };
}

function decodeKeyframePayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 16) throw new Error("KEYF payload too short");
  let off = 0;
  const version = readU8(u8, off); off += 1;
  const descriptorType = readU8(u8, off); off += 1;
  const flags = readU16BE(u8, off); off += 2;
  const timestampNs = readBigU64BE(u8, off); off += 8;
  const descriptorDim = readU32BE(u8, off); off += 4;
  if (version !== 1) throw new Error(`unsupported KEYF version ${version}`);
  if (descriptorType !== 1) throw new Error(`unsupported KEYF descriptor type ${descriptorType}`);
  if (u8.length < off + descriptorDim * 4) throw new Error("KEYF descriptor truncated");
  const descriptor = new Float32Array(descriptorDim);
  for (let i = 0; i < descriptorDim; i += 1) {
    descriptor[i] = readF32BE(u8, off);
    off += 4;
  }
  return { version, descriptorType, flags, timestampNs, descriptorDim, descriptor };
}

function decodeVioStatePayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 1 + 1 + 2 + 8 + 4 + 4 + 4 + 4 + 4 + 4) {
    throw new Error("VSTA payload too short");
  }
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  let off = 0;
  const version = dv.getUint8(off); off += 1;
  const state = dv.getUint8(off); off += 1;
  const flags = dv.getUint16(off, false); off += 2;
  const timestampNs = dv.getBigUint64(off, false); off += 8;
  const fpsCurrent = dv.getFloat32(off, false); off += 4;
  const fpsAverage = dv.getFloat32(off, false); off += 4;
  const poseConfidence = dv.getFloat32(off, false); off += 4;
  const trackingRate = dv.getFloat32(off, false); off += 4;
  const numFeatures = dv.getUint32(off, false); off += 4;
  const loopClosures = dv.getUint32(off, false); off += 4;
  const dec = textDecoder || new TextDecoder();
  let buildVersion = "";
  if (version >= 2 && off < u8.length) {
    const ll = dv.getUint8(off); off += 1;
    if (ll > 0) {
      if (off + ll > u8.length) throw new Error("VSTA buildVersion truncated");
      buildVersion = dec.decode(u8.subarray(off, off + ll));
      off += ll;
    }
  }
  let imuHzCurrent = 0.0;
  let imuHzAverage5s = 0.0;
  if (version >= 3 && off + 8 <= u8.length) {
    imuHzCurrent = dv.getFloat32(off, false); off += 4;
    imuHzAverage5s = dv.getFloat32(off, false); off += 4;
  }
  let initReasonCode = 0;
  if (version >= 4 && off < u8.length) {
    initReasonCode = dv.getUint8(off); off += 1;
  }
  let staticInitReasonCode = 0;
  let dynamicInitReasonCode = 0;
  if (version >= 5 && off + 2 <= u8.length) {
    staticInitReasonCode = dv.getUint8(off); off += 1;
    dynamicInitReasonCode = dv.getUint8(off); off += 1;
  }
  let memoryTotalBytes = 0n;
  let memoryUsedBytes = 0n;
  let memoryFreeBytes = 0n;
  if (version >= 6 && off + 24 <= u8.length) {
    memoryTotalBytes = dv.getBigUint64(off, false); off += 8;
    memoryUsedBytes = dv.getBigUint64(off, false); off += 8;
    memoryFreeBytes = dv.getBigUint64(off, false); off += 8;
  }
  let lightLevel01 = 1.0;
  let lightRequired01 = 1.0;
  if (version >= 7 && off + 8 <= u8.length) {
    lightLevel01 = dv.getFloat32(off, false); off += 4;
    lightRequired01 = dv.getFloat32(off, false); off += 4;
  }
  let translationConfidence01 = 1.0;
  let translationObservability01 = 1.0;
  let degradedReasonFlags = 0;
  if (version >= 8 && off + 12 <= u8.length) {
    translationConfidence01 = dv.getFloat32(off, false); off += 4;
    translationObservability01 = dv.getFloat32(off, false); off += 4;
    degradedReasonFlags = dv.getUint32(off, false); off += 4;
  }
  return {
    version,
    state,
    flags,
    timestampNs,
    fpsCurrent,
    fpsAverage,
    poseConfidence,
    trackingRate,
    numFeatures,
    loopClosures,
    buildVersion,
    imuHzCurrent,
    imuHzAverage5s,
    initReasonCode,
    staticInitReasonCode,
    dynamicInitReasonCode,
    memoryTotalBytes,
    memoryUsedBytes,
    memoryFreeBytes,
    lightLevel01,
    lightRequired01,
    translationConfidence01,
    translationObservability01,
    degradedReasonFlags,
  };
}

function decodeFea3Payload(payload) {
  const u8 = toU8(payload);
  let off = 0;
  const count = readU16BE(u8, off); off += 2;
  const features = [];
  for (let i = 0; i < count; ++i) {
    const id = readU16BE(u8, off); off += 2;
    const x = readF64BE(u8, off); off += 8;
    const y = readF64BE(u8, off); off += 8;
    const z = readF64BE(u8, off); off += 8;
    features.push({ id, x, y, z });
  }
  return features;
}

function decodePcldPayload(payload) {
  const u8 = toU8(payload);
  let off = 0;
  const count = readU32BE(u8, off); off += 4;
  let pointSize = null;
  const perPoint = 4 * 3 + 3;
  const expectedWithoutSize = 4 + count * perPoint;
  if (u8.length >= expectedWithoutSize + 4) {
    pointSize = readF32BE(u8, off); off += 4;
  }
  const points = [];
  for (let i = 0; i < count; ++i) {
    const x = readF32BE(u8, off); off += 4;
    const y = readF32BE(u8, off); off += 4;
    const z = readF32BE(u8, off); off += 4;
    const r = readU8(u8, off); off += 1;
    const g = readU8(u8, off); off += 1;
    const b = readU8(u8, off); off += 1;
    points.push({ x, y, z, r, g, b });
  }
  return { points, pointSize };
}

function decodeCommandPayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 4 + 1 + 4) throw new Error("CMD payload too short");
  let off = 0;
  const reqId = readU32BE(u8, off); off += 4;
  const nameLen = readU8(u8, off); off += 1;
  if (u8.length < off + nameLen + 4) throw new Error("CMD payload truncated");
  const name = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + nameLen)); off += nameLen;
  const dataLen = readU32BE(u8, off); off += 4;
  if (u8.length < off + dataLen) throw new Error("CMD payload data truncated");
  const data = fromU8(u8.subarray(off, off + dataLen));
  return { reqId, name, data };
}

function decodeCommandResponsePayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 4 + 1 + 2 + 4) throw new Error("CRES payload too short");
  let off = 0;
  const reqId = readU32BE(u8, off); off += 4;
  const status = readU8(u8, off); off += 1;
  const msgLen = readU16BE(u8, off); off += 2;
  if (u8.length < off + msgLen + 4) throw new Error("CRES payload truncated");
  const message = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + msgLen)); off += msgLen;
  const dataLen = readU32BE(u8, off); off += 4;
  if (u8.length < off + dataLen) throw new Error("CRES data truncated");
  const data = fromU8(u8.subarray(off, off + dataLen));
  return { reqId, status, message, data };
}

function decodeLuaLogPayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 4) throw new Error("LLOG payload too short");
  const seq = readU32BE(u8, 0);
  const text = (textDecoder || new TextDecoder()).decode(u8.subarray(4));
  return { seq, text };
}

function decodeConfigRequestPayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 1 + 1 + 1 + 4) throw new Error("CFGQ payload too short");
  let off = 0;
  const version = readU8(u8, off); off += 1;
  const op = readU8(u8, off); off += 1;
  const keyLen = readU8(u8, off); off += 1;
  if (u8.length < off + keyLen + 4) throw new Error("CFGQ payload truncated");
  const key = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + keyLen)); off += keyLen;
  const valueLen = readU32BE(u8, off); off += 4;
  if (u8.length < off + valueLen) throw new Error("CFGQ value truncated");
  const value = fromU8(u8.subarray(off, off + valueLen));
  return { version, op, key, value };
}

function decodeResetVioPosePayload(payload) {
  const pose = decodePosePayload(payload);
  if (pose.poseType !== 0) throw new Error("reset_vio_pose requires body pose type");
  const positionM = pose.positionM || [];
  if (
    positionM.length !== 3 ||
    !positionM.every((v) => Number.isFinite(Number(v)))
  ) {
    throw new Error("reset_vio_pose position invalid");
  }
  if (
    pose.orientationXyzw !== null &&
    pose.orientationXyzw !== undefined &&
    (
      !Array.isArray(pose.orientationXyzw) ||
      pose.orientationXyzw.length !== 4 ||
      !pose.orientationXyzw.every((v) => Number.isFinite(Number(v)))
    )
  ) {
    throw new Error("reset_vio_pose orientation invalid");
  }
  return {
    poseType: pose.poseType,
    poseFlags: pose.poseFlags,
    positionM,
    orientationXyzw: pose.orientationXyzw || null,
  };
}

function decodeConfigResponsePayload(payload) {
  const u8 = toU8(payload);
  if (u8.length < 1 + 1 + 1 + 1 + 1 + 2 + 4) throw new Error("CFGR payload too short");
  let off = 0;
  const version = readU8(u8, off); off += 1;
  const op = readU8(u8, off); off += 1;
  const success = readU8(u8, off); off += 1;
  const hasValue = readU8(u8, off) !== 0; off += 1;
  const keyLen = readU8(u8, off); off += 1;
  if (u8.length < off + keyLen + 2 + 4) throw new Error("CFGR payload truncated");
  const key = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + keyLen)); off += keyLen;
  const msgLen = readU16BE(u8, off); off += 2;
  if (u8.length < off + msgLen + 4) throw new Error("CFGR payload truncated");
  const message = (textDecoder || new TextDecoder()).decode(u8.subarray(off, off + msgLen)); off += msgLen;
  const valueLen = readU32BE(u8, off); off += 4;
  if (u8.length < off + valueLen) throw new Error("CFGR value truncated");
  const value = fromU8(u8.subarray(off, off + valueLen));
  return { version, op, success, hasValue, key, message, value };
}

const api = {
  TYPE,
  RAW_FORMAT,
  CONFIG_OP,
  VIO_STATE,
  VIO_DEGRADED_REASON,
  VIO_INIT_REASON,
  HEADER_MAGIC,
  FOOTER_MAGIC,
  makePacket,
  parseFrames,
  FrameDispatcher,
  buildJpgPayload,
  buildRawPayload,
  buildStereoRawPayload,
  buildPosePayload,
  buildConstraintsPayload,
  buildVizPayload,
  buildImuPayload,
  buildStatusPayload,
  buildEventPayload,
  buildKeyframePayload,
  buildVioStatePayload,
  buildFea3Payload,
  buildPcldPayload,
  buildCommandPayload,
  buildResetVioPosePayload,
  buildCommandResponsePayload,
  buildLuaLogPayload,
  buildConfigRequestPayload,
  buildConfigResponsePayload,
  decodeJpgPayload,
  decodeRawPayload,
  decodeStereoRawPayload,
  decodePosePayload,
  decodeConstraintsPayload,
  decodeVizPayload,
  decodeImuPayload,
  decodeStatusPayload,
  decodeEventPayload,
  decodeKeyframePayload,
  decodeVioStatePayload,
  decodeFea3Payload,
  decodePcldPayload,
  decodeCommandPayload,
  decodeResetVioPosePayload,
  decodeCommandResponsePayload,
  decodeLuaLogPayload,
  decodeConfigRequestPayload,
  decodeConfigResponsePayload,
};

export {
  TYPE,
  RAW_FORMAT,
  CONFIG_OP,
  VIO_STATE,
  VIO_DEGRADED_REASON,
  VIO_INIT_REASON,
  HEADER_MAGIC,
  FOOTER_MAGIC,
  makePacket,
  parseFrames,
  FrameDispatcher,
  buildJpgPayload,
  buildRawPayload,
  buildStereoRawPayload,
  buildPosePayload,
  buildConstraintsPayload,
  buildVizPayload,
  buildImuPayload,
  buildStatusPayload,
  buildEventPayload,
  buildKeyframePayload,
  buildVioStatePayload,
  buildFea3Payload,
  buildPcldPayload,
  buildCommandPayload,
  buildResetVioPosePayload,
  buildCommandResponsePayload,
  buildLuaLogPayload,
  buildConfigRequestPayload,
  buildConfigResponsePayload,
  decodeJpgPayload,
  decodeRawPayload,
  decodeStereoRawPayload,
  decodePosePayload,
  decodeConstraintsPayload,
  decodeVizPayload,
  decodeImuPayload,
  decodeStatusPayload,
  decodeEventPayload,
  decodeKeyframePayload,
  decodeVioStatePayload,
  decodeFea3Payload,
  decodePcldPayload,
  decodeCommandPayload,
  decodeResetVioPosePayload,
  decodeCommandResponsePayload,
  decodeLuaLogPayload,
  decodeConfigRequestPayload,
  decodeConfigResponsePayload,
};

export default api;
