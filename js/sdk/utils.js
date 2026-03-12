const hasBuffer = typeof Buffer !== "undefined";

function toU8(data = new Uint8Array()) {
  if (data instanceof Uint8Array) return data;
  if (hasBuffer && data instanceof Buffer) {
    return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  }
  return new Uint8Array(data);
}

function encodeText(text = "") {
  if (typeof TextEncoder !== "undefined") {
    return new TextEncoder().encode(text);
  }
  if (hasBuffer) {
    return Buffer.from(String(text), "utf8");
  }
  throw new Error("No UTF-8 encoder available");
}

function decodeText(bytes) {
  const u8 = toU8(bytes);
  if (typeof TextDecoder !== "undefined") {
    return new TextDecoder().decode(u8);
  }
  if (hasBuffer) {
    return Buffer.from(u8.buffer, u8.byteOffset, u8.byteLength).toString("utf8");
  }
  let out = "";
  for (let i = 0; i < u8.length; i += 1) out += String.fromCharCode(u8[i]);
  return out;
}

function sleep(ms) {
  return new Promise((resolve) => {
    setTimeout(resolve, Math.max(0, Number(ms) || 0));
  });
}

function isAbortError(err) {
  if (!err) return false;
  if (typeof err === "object" && err !== null) {
    const name = err.name || "";
    const message = err.message || "";
    if (name === "AbortError") return true;
    if (typeof message === "string" && message.toLowerCase().includes("aborted")) {
      return true;
    }
  }
  return false;
}

module.exports = {
  toU8,
  encodeText,
  decodeText,
  sleep,
  isAbortError,
};
