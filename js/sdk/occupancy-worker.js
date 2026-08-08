export class MightyOccupancyGridWorker {
  constructor(client, options = {}) {
    if (!client?.onImage || !client?.onPose) {
      throw new Error("MightyOccupancyGridWorker requires a MightyClient");
    }
    if (typeof Worker !== "function") {
      throw new Error("occupancy worker requires Web Worker support");
    }

    this.client = client;
    this.closed = false;
    this.readyState = false;
    this.workerBusy = false;
    this.pendingImage = null;
    this.pendingMessages = [];
    this.latestStats = null;
    this.onUpdate = typeof options.onUpdate === "function" ? options.onUpdate : null;
    this.onError = typeof options.onError === "function" ? options.onError : null;
    this.worker = new Worker(new URL("./occupancy-worker-runtime.js", import.meta.url), {
      type: "module",
      name: "mighty-occupancy-grid",
    });

    this.ready = new Promise((resolve, reject) => {
      this.resolveReady = resolve;
      this.rejectReady = reject;
    });
    this.worker.onmessage = (event) => this._handleWorkerMessage(event.data || {});
    this.worker.onerror = (event) => {
      const error = new Error(event?.message || "occupancy worker failed");
      this._reportError(error);
      this.rejectReady?.(error);
      this.rejectReady = null;
    };

    const {
      onUpdate: _onUpdate,
      onError: _onError,
      worker: _worker,
      moduleUrl,
      wasmUrl,
      ...workerOptions
    } = options;
    const pageUrl = globalThis.location?.href;
    const absoluteModuleUrl = moduleUrl && pageUrl ? new URL(moduleUrl, pageUrl).href : moduleUrl;
    const absoluteWasmUrl = wasmUrl && pageUrl ? new URL(wasmUrl, pageUrl).href : wasmUrl;
    this.worker.postMessage({
      type: "init",
      moduleUrl: absoluteModuleUrl,
      wasmUrl: absoluteWasmUrl,
      options: workerOptions,
    });

    this.unsubscribers = [
      client.onImage((image) => this._enqueueImage(image)),
      client.onPose((pose) => this._post({ type: "event", eventType: "pose", value: pose })),
      client.onVioState((metrics) => this._post({ type: "event", eventType: "metrics", value: metrics })),
      client.onReset(() => this._post({ type: "event", eventType: "reset" })),
    ];
  }

  close() {
    if (this.closed) return;
    this.closed = true;
    this.unsubscribers.splice(0).forEach((unsubscribe) => unsubscribe?.());
    this.pendingMessages.length = 0;
    this.pendingImage = null;
    this.worker.terminate();
  }

  clear() {
    if (!this.closed) this._post({ type: "clear" });
    return null;
  }

  process() {
    return false;
  }

  stats() {
    if (!this.closed) this._post({ type: "stats" });
    return this.latestStats;
  }

  _enqueueImage(image) {
    if (this.closed) return;
    if (this.workerBusy) {
      this.pendingImage = image;
      return;
    }
    this.workerBusy = true;
    this._post({ type: "event", eventType: "image", value: image });
  }

  _post(message) {
    if (this.closed) return;
    if (!this.readyState && message.type !== "init") {
      this.pendingMessages.push(message);
      return;
    }
    this.worker.postMessage(message);
  }

  _handleWorkerMessage(message) {
    if (this.closed) return;
    if (message.type === "ready") {
      this.readyState = true;
      for (const pending of this.pendingMessages.splice(0)) {
        this.worker.postMessage(pending);
      }
      this.resolveReady?.(this);
      this.resolveReady = null;
      this.rejectReady = null;
    } else if (message.type === "update") {
      this.onUpdate?.(message.update);
    } else if (message.type === "frame_done") {
      this.workerBusy = false;
      if (this.pendingImage) {
        const image = this.pendingImage;
        this.pendingImage = null;
        this._enqueueImage(image);
      }
    } else if (message.type === "stats") {
      this.latestStats = message.stats || null;
    } else if (message.type === "error") {
      const error = new Error(message.error?.message || "occupancy worker failed");
      error.code = message.error?.code || "occupancy_worker_failed";
      this._reportError(error);
      this.rejectReady?.(error);
      this.rejectReady = null;
    }
  }

  _reportError(error) {
    if (this.onError) this.onError(error);
    else if (typeof console !== "undefined") console.error("Mighty occupancy grid error", error);
  }
}
