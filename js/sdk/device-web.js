const { toU8, isAbortError } = require("./utils");

function getDefaultFetch() {
  if (typeof fetch === "function") return fetch.bind(globalThis);
  return null;
}

function normalizeBaseUrl(baseUrl) {
  if (baseUrl && typeof baseUrl === "string") {
    return baseUrl.endsWith("/") ? baseUrl.slice(0, -1) : baseUrl;
  }
  if (typeof window !== "undefined" && window.location && window.location.origin) {
    return window.location.origin;
  }
  throw new Error("MightyWebDevice requires baseUrl outside browsers");
}

class MightyWebDevice {
  constructor({
    baseUrl = "",
    streamPath = "/stream",
    commandPath = "/command",
    fetchImpl = null,
    headers = null,
  } = {}) {
    this.baseUrl = normalizeBaseUrl(baseUrl);
    this.streamPath = streamPath || "/stream";
    this.commandPath = commandPath || "/command";
    this.fetchImpl = fetchImpl || getDefaultFetch();
    this.headers = headers && typeof headers === "object" ? { ...headers } : null;

    this._connectPromise = null;
    this._abortController = null;
  }

  getInfo() {
    return { transport: "http", source: this.baseUrl };
  }

  _url(path) {
    if (!path) return this.baseUrl;
    if (/^https?:\/\//i.test(path)) return path;
    if (path.startsWith("/")) return `${this.baseUrl}${path}`;
    return `${this.baseUrl}/${path}`;
  }

  async connect(onBytes) {
    if (typeof onBytes !== "function") {
      throw new Error("MightyWebDevice.connect requires an onBytes callback");
    }
    if (!this.fetchImpl) {
      throw new Error("MightyWebDevice requires a fetch implementation");
    }
    if (this._connectPromise) {
      throw new Error("MightyWebDevice stream is already connected");
    }

    this._abortController = typeof AbortController !== "undefined" ? new AbortController() : null;
    const signal = this._abortController ? this._abortController.signal : undefined;

    const run = this._runStream(onBytes, signal);
    this._connectPromise = run.finally(() => {
      this._connectPromise = null;
      this._abortController = null;
    });
    return this._connectPromise;
  }

  async _runStream(onBytes, signal) {
    const headers = { Accept: "application/octet-stream", ...(this.headers || {}) };
    const response = await this.fetchImpl(this._url(this.streamPath), {
      method: "GET",
      headers,
      signal,
    });

    if (!response || !response.ok) {
      const status = response ? `${response.status}` : "no_response";
      throw new Error(`stream request failed (${status})`);
    }

    const body = response.body;
    if (!body) return;

    if (typeof body.getReader === "function") {
      const reader = body.getReader();
      try {
        // Stream raw framed bytes into the protocol parser.
        while (true) {
          const { done, value } = await reader.read();
          if (done) break;
          if (value && value.length) onBytes(toU8(value));
        }
      } finally {
        try {
          reader.releaseLock();
        } catch (_) {
          // ignore release errors
        }
      }
      return;
    }

    // Fallback for environments without readable-stream reader support.
    const arr = await response.arrayBuffer();
    const u8 = new Uint8Array(arr);
    if (u8.length) onBytes(u8);
  }

  async disconnect() {
    if (this._abortController) {
      try {
        this._abortController.abort();
      } catch (_) {
        // ignore abort failures
      }
    }
    if (!this._connectPromise) return;
    try {
      await this._connectPromise;
    } catch (err) {
      if (!isAbortError(err)) throw err;
    }
  }

  async sendCommandPayload(cmdPayload) {
    if (!this.fetchImpl) {
      throw new Error("MightyWebDevice requires a fetch implementation");
    }
    const response = await this.fetchImpl(this._url(this.commandPath), {
      method: "POST",
      headers: {
        "Content-Type": "application/octet-stream",
        Accept: "application/octet-stream",
        ...(this.headers || {}),
      },
      body: toU8(cmdPayload),
    });

    if (!response || !response.ok) {
      const status = response ? `${response.status}` : "no_response";
      throw new Error(`command request failed (${status})`);
    }

    const arr = await response.arrayBuffer();
    return new Uint8Array(arr);
  }
}

module.exports = {
  MightyWebDevice,
};
