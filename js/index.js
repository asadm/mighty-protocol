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
  RSET: "RSET",
  FEA3: "FEA3",
  PCLD: "PCLD",
  CMD: "CMD ",
  CRES: "CRES",
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

function buildPosePayload({ poseType = 0, poseFlags = 0, position = [0, 0, 0], quat = null }) {
  const hasQuat = Array.isArray(quat) && quat.length === 4;
  let flags = poseFlags;
  if (hasQuat) flags |= 0x1;
  const len = 4 + 4 + 8 * 3 + (hasQuat ? 8 * 4 : 0);
  const buf = new Uint8Array(len);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint32(off, poseType >>> 0, false); off += 4;
  dv.setUint32(off, flags >>> 0, false); off += 4;
  dv.setFloat64(off, position[0], false); off += 8;
  dv.setFloat64(off, position[1], false); off += 8;
  dv.setFloat64(off, position[2], false); off += 8;
  if (hasQuat) {
    dv.setFloat64(off, quat[0], false); off += 8;
    dv.setFloat64(off, quat[1], false); off += 8;
    dv.setFloat64(off, quat[2], false); off += 8;
    dv.setFloat64(off, quat[3], false); off += 8;
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
  } else {
    throw new Error("unknown viz subtype");
  }
  const header = new Uint8Array(3);
  const hdv = new DataView(header.buffer, header.byteOffset, header.byteLength);
  hdv.setUint8(0, subtype);
  hdv.setUint16(
    1,
    subtype === 0 ? viz.features.length : subtype === 1 ? viz.detections.length : viz.matches.length,
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
  let quat = null;
  if (poseFlags & 0x1) {
    quat = [
      readF64BE(u8, off), readF64BE(u8, off + 8),
      readF64BE(u8, off + 16), readF64BE(u8, off + 24),
    ];
  }
  return { poseType, poseFlags, position: [x, y, z], quat };
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

const api = {
  TYPE,
  RAW_FORMAT,
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
  buildFea3Payload,
  buildPcldPayload,
  buildCommandPayload,
  buildCommandResponsePayload,
  decodeJpgPayload,
  decodeRawPayload,
  decodeStereoRawPayload,
  decodePosePayload,
  decodeConstraintsPayload,
  decodeVizPayload,
  decodeImuPayload,
  decodeStatusPayload,
  decodeFea3Payload,
  decodePcldPayload,
  decodeCommandPayload,
  decodeCommandResponsePayload,
};

if (typeof module !== "undefined" && module.exports) {
  module.exports = api;
  module.exports.default = api;
}
if (typeof globalThis !== "undefined" && globalThis.window === globalThis) {
  globalThis.MightyProtocol = api;
}
