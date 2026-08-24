import jpeg from "jpeg-js";

import { RAW_FORMAT } from "../../js/index.js";

export function decodeNodeJpegToRaw(image) {
  if (!image || image.kind !== "jpg") {
    throw new Error("decodeNodeJpegToRaw requires a kind='jpg' image");
  }
  const encoded = Buffer.from(image.data || []);
  const decoded = jpeg.decode(encoded, { useTArray: true });
  if (!decoded?.width || !decoded?.height || !decoded?.data) {
    throw new Error("jpeg-js returned an invalid image");
  }
  return {
    kind: "raw",
    timestampNs: image.timestampNs ?? 0n,
    width: decoded.width,
    height: decoded.height,
    format: RAW_FORMAT.RGBA32,
    channel: image.channel || "preview",
    channelAlias: image.channelAlias,
    data: decoded.data instanceof Uint8Array ? decoded.data : new Uint8Array(decoded.data),
  };
}
