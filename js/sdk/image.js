import { RAW_FORMAT } from "../core/protocol.js";

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
