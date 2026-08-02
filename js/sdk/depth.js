import {
  DEPTH_CAMERA_MODEL,
  DEPTH_DISTORTION_MODEL,
  DEPTH_ENCODING,
} from "../core/protocol.js";
import { decodeRawToRgb } from "./image.js";

function canonicalChannel(channel) {
  const value = String(channel || "").trim().toLowerCase();
  if (value === "preview" || value === "left") return "cam0";
  if (value === "right") return "cam1";
  return value;
}

function frameTimestamp(frame) {
  const value = frame?.timestampNs ?? frame?.timestamp_ns ?? 0n;
  try {
    return BigInt(value || 0);
  } catch (_) {
    return 0n;
  }
}

function rgbdKey(channel, timestampNs) {
  return `${canonicalChannel(channel)}:${BigInt(timestampNs || 0).toString()}`;
}

export function depthAtMeters(frame, x, y) {
  const width = Number(frame?.width || 0);
  const height = Number(frame?.height || 0);
  const px = Math.floor(Number(x));
  const py = Math.floor(Number(y));
  const values = frame?.depthMm;
  if (!values || width <= 0 || height <= 0 ||
      px < 0 || py < 0 || px >= width || py >= height) return null;
  const sample = Number(values[py * width + px]);
  const invalid = Number(frame?.invalidValue ?? 0);
  const scale = Number(frame?.depthScaleM ?? 0.001);
  if (!Number.isFinite(sample) || sample === invalid || !Number.isFinite(scale) || scale <= 0) return null;
  return sample * scale;
}

// Google's compact Turbo approximation. The protocol stores metric values;
// this palette is presentation-only and can be replaced by applications.
function turboRgb(value) {
  const x = Math.max(0, Math.min(1, value));
  const k = [1, x, x * x, x * x * x, x ** 4, x ** 5];
  const dot = (coeffs) => coeffs.reduce((sum, c, i) => sum + c * k[i], 0);
  const clampByte = (v) => Math.max(0, Math.min(255, Math.round(v * 255)));
  return [
    clampByte(dot([0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943])),
    clampByte(dot([0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604])),
    clampByte(dot([0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973])),
  ];
}

export function colorizeDepth(frame, { nearM = 0, farM = 10 } = {}) {
  const width = Number(frame?.width || 0);
  const height = Number(frame?.height || 0);
  const values = frame?.depthMm;
  if (!values || width <= 0 || height <= 0 || values.length < width * height) return null;
  const near = Number(nearM);
  const far = Number(farM);
  const span = far - near;
  if (!Number.isFinite(span) || span <= 0) return null;
  const invalid = Number(frame?.invalidValue ?? 0);
  const scale = Number(frame?.depthScaleM ?? 0.001);
  const rgba = new Uint8ClampedArray(width * height * 4);
  for (let i = 0; i < width * height; i += 1) {
    const sample = Number(values[i]);
    const out = i * 4;
    if (sample === invalid) {
      rgba[out + 3] = 255;
      continue;
    }
    const depthM = sample * scale;
    const [r, g, b] = turboRgb((far - depthM) / span);
    rgba[out] = r;
    rgba[out + 1] = g;
    rgba[out + 2] = b;
    rgba[out + 3] = 255;
  }
  return { width, height, rgba };
}

function distortNormalized(x, y, frame) {
  const coeffs = Array.from(frame?.distortion || [0, 0, 0, 0], Number);
  while (coeffs.length < 4) coeffs.push(0);
  const cameraModel = Number(frame?.cameraModel ?? DEPTH_CAMERA_MODEL.PINHOLE);
  const distortionModel = Number(frame?.distortionModel ?? DEPTH_DISTORTION_MODEL.NONE);
  if (cameraModel === DEPTH_CAMERA_MODEL.DOUBLE_SPHERE) {
    const [xi, alpha] = coeffs;
    const r2 = x * x + y * y;
    if (r2 < 1e-16) return [x, y];
    const d1 = Math.sqrt(r2 + 1);
    const shifted = xi * d1 + 1;
    const d2 = Math.sqrt(r2 + shifted * shifted);
    const denominator = alpha * d2 + (1 - alpha) * shifted;
    return Number.isFinite(denominator) && Math.abs(denominator) > 1e-12
      ? [x / denominator, y / denominator]
      : [Number.NaN, Number.NaN];
  }
  if (distortionModel === DEPTH_DISTORTION_MODEL.NONE) return [x, y];
  if (distortionModel === DEPTH_DISTORTION_MODEL.EQUIDISTANT) {
    const [k1, k2, k3, k4] = coeffs;
    const r = Math.hypot(x, y);
    if (r < 1e-8) return [x, y];
    const theta = Math.atan(r);
    const t2 = theta * theta;
    const thetaD = theta * (1 + k1 * t2 + k2 * t2 ** 2 + k3 * t2 ** 3 + k4 * t2 ** 4);
    const scale = thetaD / r;
    return [x * scale, y * scale];
  }
  const [k1, k2, p1, p2] = coeffs;
  const x2 = x * x;
  const y2 = y * y;
  const xy = x * y;
  const r2 = x2 + y2;
  const radial = 1 + k1 * r2 + k2 * r2 * r2;
  return [
    x * radial + 2 * p1 * xy + p2 * (r2 + 2 * x2),
    y * radial + p1 * (r2 + 2 * y2) + 2 * p2 * xy,
  ];
}

function sampleBilinearRgba(rgba, width, height, x, y, out, offset) {
  if (!Number.isFinite(x) || !Number.isFinite(y) || x < 0 || y < 0 || x > width - 1 || y > height - 1) {
    out[offset + 3] = 255;
    return;
  }
  const x0 = Math.floor(x);
  const y0 = Math.floor(y);
  const x1 = Math.min(width - 1, x0 + 1);
  const y1 = Math.min(height - 1, y0 + 1);
  const ax = x - x0;
  const ay = y - y0;
  const i00 = (y0 * width + x0) * 4;
  const i01 = (y0 * width + x1) * 4;
  const i10 = (y1 * width + x0) * 4;
  const i11 = (y1 * width + x1) * 4;
  for (let c = 0; c < 4; c += 1) {
    const top = rgba[i00 + c] + (rgba[i01 + c] - rgba[i00 + c]) * ax;
    const bottom = rgba[i10 + c] + (rgba[i11 + c] - rgba[i10 + c]) * ax;
    out[offset + c] = Math.round(top + (bottom - top) * ay);
  }
}

export function rectifyImageToDepth(imageFrame, depthFrame, { requireMatchingTimestamp = true } = {}) {
  if (!imageFrame || !depthFrame) return null;
  const imageTs = frameTimestamp(imageFrame);
  const depthTs = frameTimestamp(depthFrame);
  if (requireMatchingTimestamp && imageTs > 0n && depthTs > 0n && imageTs !== depthTs) {
    throw new Error("source image and depth timestamps do not match");
  }
  const decoded = imageFrame.rgba
    ? { width: Number(imageFrame.width), height: Number(imageFrame.height), rgba: imageFrame.rgba }
    : decodeRawToRgb(imageFrame);
  if (!decoded) throw new Error("source image format cannot be decoded to RGBA");
  const width = Number(depthFrame.width || 0);
  const height = Number(depthFrame.height || 0);
  const [dfx, dfy, dcx, dcy] = Array.from(depthFrame.depthIntrinsics || [], Number);
  const sourceK = Array.from(depthFrame.sourceIntrinsics || [], Number);
  if (width <= 0 || height <= 0 || ![dfx, dfy, dcx, dcy, ...sourceK].every(Number.isFinite) ||
      dfx <= 0 || dfy <= 0 || sourceK[0] <= 0 || sourceK[1] <= 0) {
    throw new Error("depth frame has invalid camera geometry");
  }
  const calibrationWidth = Number(depthFrame.sourceWidth || decoded.width);
  const calibrationHeight = Number(depthFrame.sourceHeight || decoded.height);
  const sx = decoded.width / calibrationWidth;
  const sy = decoded.height / calibrationHeight;
  const sfx = sourceK[0] * sx;
  const sfy = sourceK[1] * sy;
  const scx = sourceK[2] * sx;
  const scy = sourceK[3] * sy;
  const rgba = new Uint8ClampedArray(width * height * 4);
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const rayX = (x - dcx) / dfx;
      const rayY = (y - dcy) / dfy;
      const [distortedX, distortedY] = distortNormalized(rayX, rayY, depthFrame);
      sampleBilinearRgba(
        decoded.rgba, decoded.width, decoded.height,
        sfx * distortedX + scx, sfy * distortedY + scy,
        rgba, (y * width + x) * 4,
      );
    }
  }
  return {
    width,
    height,
    rgba,
    timestampNs: depthTs,
    frameId: depthFrame.frameId || "cam0_rectified",
  };
}

export class RgbdSynchronizer {
  constructor(onPair, { maxEntries = 64 } = {}) {
    if (typeof onPair !== "function") throw new Error("RgbdSynchronizer requires an onPair callback");
    this.onPair = onPair;
    this.maxEntries = Math.max(2, Number(maxEntries) || 64);
    this.images = new Map();
    this.depth = new Map();
  }

  _trim(cache) {
    while (cache.size > this.maxEntries) cache.delete(cache.keys().next().value);
  }

  pushImage(image) {
    if (image?.kind === "stereo_raw") {
      this.pushImage(image.left);
      this.pushImage(image.right);
      return;
    }
    const timestampNs = frameTimestamp(image);
    if (!image || timestampNs <= 0n) return;
    const key = rgbdKey(image.channelAlias || image.channel, timestampNs);
    const depth = this.depth.get(key);
    if (depth) {
      this.depth.delete(key);
      this.onPair({ image, depth });
      return;
    }
    this.images.set(key, image);
    this._trim(this.images);
  }

  pushDepth(depth) {
    const timestampNs = frameTimestamp(depth);
    if (!depth || timestampNs <= 0n) return;
    const key = rgbdKey(depth.sourceChannel, timestampNs);
    const image = this.images.get(key);
    if (image) {
      this.images.delete(key);
      this.onPair({ image, depth });
      return;
    }
    this.depth.set(key, depth);
    this._trim(this.depth);
  }

  clear() {
    this.images.clear();
    this.depth.clear();
  }
}

export function isUint16MetricDepth(frame) {
  return Number(frame?.encoding) === DEPTH_ENCODING.UINT16_MILLIMETERS;
}
