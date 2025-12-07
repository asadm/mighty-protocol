// Lightweight protocol helpers shared between Node and the browser code (no transport).
// Provides framing, CRC, encode/decode for all packet types.

const HEADER_MAGIC = Buffer.from([0xde, 0xad, 0xbe, 0xef]);
const FOOTER_MAGIC = Buffer.from([0xfe, 0xed, 0xfa, 0xce]);

const TYPE = {
  JPG: 'JPG ',
  RJPG: 'RJPG',
  POSE: 'POSE',
  UPOSE: 'UPOS',
  LCON: 'LCON',
  VIZ: 'VIZ ',
  IMU: 'IMU ',
  STAT: 'STAT',
  RSET: 'RSET',
  FEA3: 'FEA3',
  PCLD: 'PCLD',
};

// ---------------------------------------------------------------------------
// CRC32
// ---------------------------------------------------------------------------
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
  let crc = 0xffffffff;
  for (let i = 0; i < buf.length; ++i) {
    crc = (crcTable[(crc ^ buf[i]) & 0xff] ^ (crc >>> 8)) >>> 0;
  }
  return (crc ^ 0xffffffff) >>> 0;
}

// ---------------------------------------------------------------------------
// Framing helpers
// ---------------------------------------------------------------------------
function makePacket(type, payload = Buffer.alloc(0)) {
  if (!TYPE[type] && typeof type !== 'string') {
    throw new Error('type must be a 4-char string or known key');
  }
  const typeStr = TYPE[type] || type;
  if (typeStr.length !== 4) throw new Error('type must be 4 chars');
  const len = payload.length >>> 0;
  const out = Buffer.alloc(4 + 4 + 4 + len + 4 + 4);
  HEADER_MAGIC.copy(out, 0);
  out.write(typeStr, 4, 4, 'ascii');
  out.writeUInt32BE(len, 8);
  if (len) payload.copy(out, 12);
  const crc = len ? crc32(payload) : 0;
  out.writeUInt32BE(crc >>> 0, 12 + len);
  FOOTER_MAGIC.copy(out, 16 + len);
  return out;
}

// Parses as many complete frames as possible; returns {frames, rest}
function parseFrames(buffer) {
  let offset = 0;
  const frames = [];
  while (buffer.length - offset >= 20) {
    if (!buffer.slice(offset, offset + 4).equals(HEADER_MAGIC)) break;
    const type = buffer.slice(offset + 4, offset + 8).toString('ascii');
    const len = buffer.readUInt32BE(offset + 8);
    const pktSize = 20 + len;
    if (buffer.length - offset < pktSize) break; // need more
    const payload = buffer.slice(offset + 12, offset + 12 + len);
    const recvCrc = buffer.readUInt32BE(offset + 12 + len);
    const footer = buffer.slice(offset + 16 + len, offset + pktSize);
    if (!footer.equals(FOOTER_MAGIC) || (len && crc32(payload) !== recvCrc)) {
      offset += pktSize;
      continue;
    }
    frames.push({ type, payload });
    offset += pktSize;
  }
  return { frames, rest: buffer.slice(offset) };
}

// ---------------------------------------------------------------------------
// Dispatcher helper
// ---------------------------------------------------------------------------
class FrameDispatcher {
  constructor(onFrame) {
    this.onFrame = onFrame;
    this.buffer = Buffer.alloc(0);
  }
  feed(chunk) {
    if (!(chunk instanceof Buffer)) chunk = Buffer.from(chunk);
    this.buffer = Buffer.concat([this.buffer, chunk]);
    const parsed = parseFrames(this.buffer);
    this.buffer = parsed.rest;
    if (this.onFrame) {
      for (const f of parsed.frames) this.onFrame(f);
    }
  }
}

// ---------------------------------------------------------------------------
// Payload builders
// ---------------------------------------------------------------------------
function buildJpgPayload({ timestampNs = 0n, channel = 'preview', data = Buffer.alloc(0), isRef = false }) {
  if (!(data instanceof Buffer)) data = Buffer.from(data);
  const tsBuf = Buffer.alloc(8);
  tsBuf.writeBigUInt64BE(BigInt(timestampNs));
  if (isRef) {
    return Buffer.concat([tsBuf, data]);
  }
  const chanBuf = Buffer.from(channel || '', 'utf8');
  const chanLen = Math.min(255, chanBuf.length);
  return Buffer.concat([tsBuf, Buffer.from([chanLen]), chanBuf.slice(0, chanLen), data]);
}

function buildPosePayload({ poseType = 0, poseFlags = 0, position = [0, 0, 0], quat = null }) {
  const hasQuat = Array.isArray(quat) && quat.length === 4;
  let flags = poseFlags;
  if (hasQuat) flags |= 0x1;
  const buf = Buffer.alloc(4 + 4 + 8 * 3 + (hasQuat ? 8 * 4 : 0));
  let off = 0;
  buf.writeUInt32BE(poseType >>> 0, off); off += 4;
  buf.writeUInt32BE(flags >>> 0, off); off += 4;
  buf.writeDoubleBE(position[0], off); off += 8;
  buf.writeDoubleBE(position[1], off); off += 8;
  buf.writeDoubleBE(position[2], off); off += 8;
  if (hasQuat) {
    buf.writeDoubleBE(quat[0], off); off += 8;
    buf.writeDoubleBE(quat[1], off); off += 8;
    buf.writeDoubleBE(quat[2], off); off += 8;
    buf.writeDoubleBE(quat[3], off); off += 8;
  }
  return buf;
}

function buildConstraintsPayload(segments = []) {
  const per = 1 + 6 * 4;
  const buf = Buffer.alloc(4 + segments.length * per);
  buf.writeUInt32BE(segments.length >>> 0, 0);
  let off = 4;
  for (const s of segments) {
    buf.writeUInt8(s.type || 0, off); off += 1;
    for (let i = 0; i < 3; ++i) { buf.writeFloatBE(s.start[i] ?? 0, off); off += 4; }
    for (let i = 0; i < 3; ++i) { buf.writeFloatBE(s.end[i] ?? 0, off); off += 4; }
  }
  return buf;
}

function buildVizPayload(viz) {
  const subtype = viz.subtype ?? 0;
  let body = Buffer.alloc(0);
  if (subtype === 0) {
    body = Buffer.alloc(viz.features.length * (2 + 2 + 1 + 2));
    let off = 0;
    for (const f of viz.features) {
      body.writeUInt16BE(f.x, off); off += 2;
      body.writeUInt16BE(f.y, off); off += 2;
      body.writeUInt8(f.status || 0, off); off += 1;
      body.writeUInt16BE(f.id, off); off += 2;
    }
  } else if (subtype === 1) {
    const parts = [];
    for (const d of viz.detections) {
      const lbl = Buffer.from(d.label || '', 'utf8');
      const ll = Math.min(255, lbl.length);
      const b = Buffer.alloc(2 + 2 + 2 + 2 + 1 + ll);
      let off = 0;
      b.writeUInt16BE(d.x1, off); off += 2;
      b.writeUInt16BE(d.y1, off); off += 2;
      b.writeUInt16BE(d.x2, off); off += 2;
      b.writeUInt16BE(d.y2, off); off += 2;
      b.writeUInt8(ll, off); off += 1;
      lbl.copy(b, off, 0, ll);
      parts.push(b);
    }
    body = Buffer.concat(parts);
  } else if (subtype === 2) {
    body = Buffer.alloc(viz.matches.length * (2 + 2 + 2 + 2 + 1));
    let off = 0;
    for (const m of viz.matches) {
      body.writeUInt16BE(m.x1, off); off += 2;
      body.writeUInt16BE(m.y1, off); off += 2;
      body.writeUInt16BE(m.x2, off); off += 2;
      body.writeUInt16BE(m.y2, off); off += 2;
      body.writeUInt8(m.confidence || 0, off); off += 1;
    }
  } else {
    throw new Error('unknown viz subtype');
  }
  const header = Buffer.alloc(3);
  header.writeUInt8(subtype, 0);
  header.writeUInt16BE(subtype === 0 ? viz.features.length : subtype === 1 ? viz.detections.length : viz.matches.length, 1);
  return Buffer.concat([header, body]);
}

function buildImuPayload(samples = []) {
  if (!samples.length) return Buffer.alloc(0);
  const stride = 8 + 6 * 8;
  const buf = Buffer.alloc(4 + samples.length * stride);
  buf.writeUInt32BE(samples.length >>> 0, 0);
  let off = 4;
  for (const s of samples) {
    buf.writeBigUInt64BE(BigInt(s.timestampNs), off); off += 8;
    buf.writeDoubleBE(s.ax, off); off += 8;
    buf.writeDoubleBE(s.ay, off); off += 8;
    buf.writeDoubleBE(s.az, off); off += 8;
    buf.writeDoubleBE(s.gx, off); off += 8;
    buf.writeDoubleBE(s.gy, off); off += 8;
    buf.writeDoubleBE(s.gz, off); off += 8;
  }
  return buf;
}

const buildStatusPayload = (text = '') => Buffer.from(text, 'utf8');

function buildFea3Payload(features = []) {
  const buf = Buffer.alloc(2 + features.length * (2 + 8 * 3));
  buf.writeUInt16BE(features.length, 0);
  let off = 2;
  for (const f of features) {
    buf.writeUInt16BE(f.id, off); off += 2;
    buf.writeDoubleBE(f.x, off); off += 8;
    buf.writeDoubleBE(f.y, off); off += 8;
    buf.writeDoubleBE(f.z, off); off += 8;
  }
  return buf;
}

function buildPcldPayload(points = [], pointSize = null) {
  const includeSize = typeof pointSize === 'number' && pointSize > 0;
  const buf = Buffer.alloc((includeSize ? 8 : 4) + points.length * (4 * 3 + 3));
  buf.writeUInt32BE(points.length, 0);
  let off = 4;
  if (includeSize) {
    buf.writeFloatBE(pointSize, off); off += 4;
  }
  for (const p of points) {
    buf.writeFloatBE(p.x, off); off += 4;
    buf.writeFloatBE(p.y, off); off += 4;
    buf.writeFloatBE(p.z, off); off += 4;
    buf.writeUInt8(p.r, off); off += 1;
    buf.writeUInt8(p.g, off); off += 1;
    buf.writeUInt8(p.b, off); off += 1;
  }
  return buf;
}

// ---------------------------------------------------------------------------
// Payload decoders
// ---------------------------------------------------------------------------
function decodeJpgPayload(payload, isRef = false) {
  if (payload.length < 8) throw new Error('JPG payload too short');
  let off = 0;
  const ts = payload.readBigUInt64BE(off); off += 8;
  if (isRef) {
    return { timestampNs: ts, channel: '', data: payload.slice(off) };
  }
  const clen = payload.readUInt8(off); off += 1;
  const channel = payload.slice(off, off + clen).toString('utf8'); off += clen;
  return { timestampNs: ts, channel, data: payload.slice(off) };
}

function decodePosePayload(payload) {
  let off = 0;
  const poseType = payload.readUInt32BE(off); off += 4;
  const poseFlags = payload.readUInt32BE(off); off += 4;
  const x = payload.readDoubleBE(off); off += 8;
  const y = payload.readDoubleBE(off); off += 8;
  const z = payload.readDoubleBE(off); off += 8;
  let quat = null;
  if (poseFlags & 0x1) {
    quat = [
      payload.readDoubleBE(off), payload.readDoubleBE(off + 8),
      payload.readDoubleBE(off + 16), payload.readDoubleBE(off + 24),
    ];
  }
  return { poseType, poseFlags, position: [x, y, z], quat };
}

function decodeConstraintsPayload(payload) {
  let off = 0;
  const count = payload.readUInt32BE(off); off += 4;
  const segments = [];
  for (let i = 0; i < count; ++i) {
    const type = payload.readUInt8(off); off += 1;
    const start = [payload.readFloatBE(off), payload.readFloatBE(off + 4), payload.readFloatBE(off + 8)]; off += 12;
    const end = [payload.readFloatBE(off), payload.readFloatBE(off + 4), payload.readFloatBE(off + 8)]; off += 12;
    segments.push({ type, start, end });
  }
  return segments;
}

function decodeVizPayload(payload) {
  let off = 0;
  const subtype = payload.readUInt8(off); off += 1;
  const count = payload.readUInt16BE(off); off += 2;
  if (subtype === 0) {
    const features = [];
    for (let i = 0; i < count; ++i) {
      const x = payload.readUInt16BE(off); off += 2;
      const y = payload.readUInt16BE(off); off += 2;
      const status = payload.readUInt8(off); off += 1;
      const id = payload.readUInt16BE(off); off += 2;
      features.push({ x, y, status, id });
    }
    return { subtype, features };
  }
  if (subtype === 1) {
    const detections = [];
    for (let i = 0; i < count; ++i) {
      const x1 = payload.readUInt16BE(off); off += 2;
      const y1 = payload.readUInt16BE(off); off += 2;
      const x2 = payload.readUInt16BE(off); off += 2;
      const y2 = payload.readUInt16BE(off); off += 2;
      const ll = payload.readUInt8(off); off += 1;
      const label = payload.slice(off, off + ll).toString('utf8'); off += ll;
      detections.push({ x1, y1, x2, y2, label });
    }
    return { subtype, detections };
  }
  if (subtype === 2) {
    const matches = [];
    for (let i = 0; i < count; ++i) {
      const x1 = payload.readUInt16BE(off); off += 2;
      const y1 = payload.readUInt16BE(off); off += 2;
      const x2 = payload.readUInt16BE(off); off += 2;
      const y2 = payload.readUInt16BE(off); off += 2;
      const confidence = payload.readUInt8(off); off += 1;
      matches.push({ x1, y1, x2, y2, confidence });
    }
    return { subtype, matches };
  }
  throw new Error('Unknown viz subtype');
}

function decodeImuPayload(payload) {
  let off = 0;
  const count = payload.readUInt32BE(off); off += 4;
  const samples = [];
  for (let i = 0; i < count; ++i) {
    const timestampNs = payload.readBigUInt64BE(off); off += 8;
    const ax = payload.readDoubleBE(off); off += 8;
    const ay = payload.readDoubleBE(off); off += 8;
    const az = payload.readDoubleBE(off); off += 8;
    const gx = payload.readDoubleBE(off); off += 8;
    const gy = payload.readDoubleBE(off); off += 8;
    const gz = payload.readDoubleBE(off); off += 8;
    samples.push({ timestampNs, ax, ay, az, gx, gy, gz });
  }
  return samples;
}

const decodeStatusPayload = (payload) => payload.toString('utf8');

function decodeFea3Payload(payload) {
  let off = 0;
  const count = payload.readUInt16BE(off); off += 2;
  const features = [];
  for (let i = 0; i < count; ++i) {
    const id = payload.readUInt16BE(off); off += 2;
    const x = payload.readDoubleBE(off); off += 8;
    const y = payload.readDoubleBE(off); off += 8;
    const z = payload.readDoubleBE(off); off += 8;
    features.push({ id, x, y, z });
  }
  return features;
}

function decodePcldPayload(payload) {
  let off = 0;
  const count = payload.readUInt32BE(off); off += 4;
  let pointSize = null;
  if (payload.length >= 8 + count * (3 * 4 + 3)) {
    pointSize = payload.readFloatBE(off); off += 4;
  }
  const points = [];
  for (let i = 0; i < count; ++i) {
    const x = payload.readFloatBE(off); off += 4;
    const y = payload.readFloatBE(off); off += 4;
    const z = payload.readFloatBE(off); off += 4;
    const r = payload.readUInt8(off); off += 1;
    const g = payload.readUInt8(off); off += 1;
    const b = payload.readUInt8(off); off += 1;
    points.push({ x, y, z, r, g, b });
  }
  return { points, pointSize };
}

const api = {
  TYPE,
  HEADER_MAGIC,
  FOOTER_MAGIC,
  makePacket,
  parseFrames,
  FrameDispatcher,
  buildJpgPayload,
  buildPosePayload,
  buildConstraintsPayload,
  buildVizPayload,
  buildImuPayload,
  buildStatusPayload,
  buildFea3Payload,
  buildPcldPayload,
  decodeJpgPayload,
  decodePosePayload,
  decodeConstraintsPayload,
  decodeVizPayload,
  decodeImuPayload,
  decodeStatusPayload,
  decodeFea3Payload,
  decodePcldPayload,
};

if (typeof module !== 'undefined' && module.exports) {
  module.exports = api;
}
if (typeof window !== 'undefined') {
  window.MightyProtocol = api;
}
