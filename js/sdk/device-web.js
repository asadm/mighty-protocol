import { toU8, isAbortError } from "./utils.js";

export const DEFAULT_BASE_URLS = [
  "http://localhost:8080",
  "http://localhost:8084",
  "http://192.168.7.1:80",
  "http://192.168.7.1:8080",
];

function getDefaultFetch() {
  if (typeof fetch === "function") return fetch.bind(globalThis);
  return null;
}

function normalizeBaseUrl(baseUrl) {
  if (!baseUrl || typeof baseUrl !== "string") return "";
  return baseUrl.endsWith("/") ? baseUrl.slice(0, -1) : baseUrl;
}

function appendUnique(out, value) {
  const v = normalizeBaseUrl(value);
  if (!v) return;
  if (!/^https?:\/\//i.test(v)) return;
  if (out.includes(v)) return;
  out.push(v);
}

function resolveBaseUrls(baseUrl, baseUrls) {
  const out = [];
  if (Array.isArray(baseUrls) && baseUrls.length > 0) {
    for (const v of baseUrls) appendUnique(out, v);
  } else if (baseUrl && typeof baseUrl === "string") {
    appendUnique(out, baseUrl);
  }

  if (out.length > 0) return out;

  if (typeof window !== "undefined" && window.location && window.location.origin) {
    appendUnique(out, window.location.origin);
  }
  for (const v of DEFAULT_BASE_URLS) appendUnique(out, v);
  if (out.length === 0) {
    throw new Error("MightyWebDevice requires at least one base URL");
  }
  return out;
}

export class MightyWebDevice {
  constructor({
    baseUrl = "",
    baseUrls = null,
    streamPath = "/stream",
    commandPath = "/command",
    fetchImpl = null,
    headers = null,
  } = {}) {
    this.baseUrls = resolveBaseUrls(baseUrl, baseUrls);
    this.baseUrl = this.baseUrls[0] || "";
    this.streamPath = streamPath || "/stream";
    this.commandPath = commandPath || "/command";
    this.fetchImpl = fetchImpl || getDefaultFetch();
    this.headers = headers && typeof headers === "object" ? { ...headers } : null;

    this._connectPromise = null;
    this._abortController = null;
    this._activeBaseUrl = this.baseUrl;
  }

  getInfo() {
    return { transport: "http", source: this._activeBaseUrl || this.baseUrl || "" };
  }

  _url(baseUrl, path) {
    const base = normalizeBaseUrl(baseUrl || this.baseUrl);
    if (!path) return base;
    if (/^https?:\/\//i.test(path)) return path;
    if (path.startsWith("/")) return `${base}${path}`;
    return `${base}/${path}`;
  }

  _orderedBases() {
    const out = [];
    if (this._activeBaseUrl) appendUnique(out, this._activeBaseUrl);
    for (const b of this.baseUrls) appendUnique(out, b);
    return out;
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
    const bases = this._orderedBases();
    const headers = { Accept: "application/octet-stream", ...(this.headers || {}) };
    let lastErr = null;

    for (const base of bases) {
      try {
        const response = await this.fetchImpl(this._url(base, this.streamPath), {
          method: "GET",
          headers,
          signal,
        });

        if (!response || !response.ok) {
          const status = response ? `${response.status}` : "no_response";
          lastErr = new Error(`stream request failed (${status})`);
          continue;
        }

        this._activeBaseUrl = base;
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
        return;
      } catch (err) {
        if (isAbortError(err)) throw err;
        lastErr = err;
        try {
          if (signal && signal.aborted) throw err;
        } catch (_) {
          // no-op
        }
      }
    }

    if (lastErr) throw lastErr;
    throw new Error("stream request failed (no host)");
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

    const bases = this._orderedBases();
    let lastErr = null;
    for (const base of bases) {
      try {
        const response = await this.fetchImpl(this._url(base, this.commandPath), {
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
          lastErr = new Error(`command request failed (${status})`);
          continue;
        }

        this._activeBaseUrl = base;
        const arr = await response.arrayBuffer();
        return new Uint8Array(arr);
      } catch (err) {
        if (isAbortError(err)) throw err;
        lastErr = err;
      }
    }

    if (lastErr) throw lastErr;
    throw new Error("command request failed (no host)");
  }
}
