import { RAW_FORMAT } from "../core/protocol.js";

function toU8(data) {
  if (data instanceof Uint8Array) return data;
  if (data instanceof ArrayBuffer) return new Uint8Array(data);
  if (ArrayBuffer.isView(data)) return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  return new Uint8Array(data || []);
}

export function decodeRawToRgb(frame, rawFormat = RAW_FORMAT) {
  const width = Number(frame?.width || 0);
  const height = Number(frame?.height || 0);
  const data = frame?.data instanceof Uint8Array ? frame.data : null;
  if (!width || !height || !data) return null;

  const out = new Uint8ClampedArray(width * height * 4);
  const format = Number(frame?.format || 0);

  if (format === rawFormat.GRAY8) {
    const need = width * height;
    if (data.length < need) return null;
    for (let i = 0; i < need; i += 1) {
      const v = data[i];
      const j = i * 4;
      out[j] = v;
      out[j + 1] = v;
      out[j + 2] = v;
      out[j + 3] = 255;
    }
    return { width, height, rgba: out };
  }

  if (format === rawFormat.RGB24) {
    const need = width * height * 3;
    if (data.length < need) return null;
    for (let i = 0, j = 0; i < need; i += 3, j += 4) {
      out[j] = data[i];
      out[j + 1] = data[i + 1];
      out[j + 2] = data[i + 2];
      out[j + 3] = 255;
    }
    return { width, height, rgba: out };
  }

  if (format === rawFormat.BGR24) {
    const need = width * height * 3;
    if (data.length < need) return null;
    for (let i = 0, j = 0; i < need; i += 3, j += 4) {
      out[j] = data[i + 2];
      out[j + 1] = data[i + 1];
      out[j + 2] = data[i];
      out[j + 3] = 255;
    }
    return { width, height, rgba: out };
  }

  if (format === rawFormat.RGBA32) {
    const need = width * height * 4;
    if (data.length < need) return null;
    out.set(data.slice(0, need));
    return { width, height, rgba: out };
  }

  if (format === rawFormat.BGRA32) {
    const need = width * height * 4;
    if (data.length < need) return null;
    for (let i = 0; i < need; i += 4) {
      out[i] = data[i + 2];
      out[i + 1] = data[i + 1];
      out[i + 2] = data[i];
      out[i + 3] = data[i + 3];
    }
    return { width, height, rgba: out };
  }

  if (format === rawFormat.YUV420SP || format === rawFormat.YUV420P) {
    const need = width * height;
    if (data.length < need) return null;
    for (let i = 0; i < need; i += 1) {
      const y = data[i];
      const j = i * 4;
      out[j] = y;
      out[j + 1] = y;
      out[j + 2] = y;
      out[j + 3] = 255;
    }
    return { width, height, rgba: out };
  }

  return null;
}

export async function decodeJpegToRaw(frame) {
  if (!frame || frame.kind !== "jpg") throw new Error("decodeJpegToRaw requires a kind='jpg' image");
  if (typeof createImageBitmap !== "function" || typeof Blob === "undefined") {
    throw new Error("JPEG pixel decoding requires browser image APIs or a custom jpegDecoder");
  }
  const bitmap = await createImageBitmap(new Blob([toU8(frame.data)], { type: "image/jpeg" }));
  try {
    const width = Number(bitmap.width || 0);
    const height = Number(bitmap.height || 0);
    if (width <= 0 || height <= 0) throw new Error("decoded JPEG has invalid dimensions");
    const canvas = typeof OffscreenCanvas === "function"
      ? new OffscreenCanvas(width, height)
      : document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const context = canvas.getContext("2d", { willReadFrequently: true });
    if (!context) throw new Error("unable to create JPEG decode canvas");
    context.drawImage(bitmap, 0, 0);
    const pixels = context.getImageData(0, 0, width, height).data;
    return {
      kind: "raw",
      timestampNs: frame.timestampNs ?? 0n,
      width,
      height,
      format: RAW_FORMAT.RGBA32,
      channel: frame.channel || "preview",
      channelAlias: frame.channelAlias,
      data: new Uint8Array(pixels),
    };
  } finally {
    bitmap.close?.();
  }
}

export async function imageToRaw(frame, { jpegDecoder, includeReference = false } = {}) {
  if (!frame) return null;
  if (frame.kind === "raw") return frame;
  if (frame.kind === "stereo_raw") {
    const candidates = [frame.left, frame.right].filter(Boolean);
    const primary = candidates.find((candidate) => {
      const channel = String(candidate.channelAlias || candidate.channel || "").toLowerCase();
      return channel === "cam0" || channel === "preview" || channel === "left";
    });
    return primary || candidates[0] || null;
  }
  if (frame.kind !== "jpg") return null;
  if (frame.isReference && !includeReference) return null;
  const decoder = typeof jpegDecoder === "function" ? jpegDecoder : decodeJpegToRaw;
  return decoder(frame);
}

export async function decodeImageToRgb(frame, options = {}) {
  const raw = await imageToRaw(frame, options);
  return raw ? decodeRawToRgb(raw) : null;
}
