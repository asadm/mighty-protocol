/* global self */

import { createAlgorithmsWasmModule } from "./loopclosure-wasm.js";
import { MightyOccupancyGrid } from "./occupancy-wasm.js";

class ForwardedClient {
  constructor() {
    this.listeners = new Map();
  }

  subscribe(type, callback) {
    this.listeners.set(type, callback);
    return () => this.listeners.delete(type);
  }

  onImage(callback) { return this.subscribe("image", callback); }
  onPose(callback) { return this.subscribe("pose", callback); }
  onVioState(callback) { return this.subscribe("metrics", callback); }
  onReset(callback) { return this.subscribe("reset", callback); }

  emit(type, value) {
    this.listeners.get(type)?.(value);
  }
}

let client = null;
let grid = null;
let initializePromise = null;

function reportError(error) {
  self.postMessage({
    type: "error",
    error: {
      message: error?.message || String(error),
      code: error?.code || "occupancy_worker_failed",
    },
  });
}

function postUpdate(update) {
  if (!update) return;
  const transfers = [
    update.indices?.buffer,
    update.states?.buffer,
    update.occupancy?.buffer,
    update.intensity?.buffer,
    update.support?.buffer,
    update.visible?.buffer,
  ].filter(Boolean);
  self.postMessage({ type: "update", update }, transfers);
}

async function initialize(message) {
  const module = await createAlgorithmsWasmModule({
    moduleUrl: message.moduleUrl,
    wasmUrl: message.wasmUrl,
  });
  client = new ForwardedClient();
  grid = new MightyOccupancyGrid(client, module, {
    ...(message.options || {}),
    autoProcess: false,
    onUpdate: postUpdate,
    onError: reportError,
  });
  self.postMessage({ type: "ready" });
}

async function handleMessage(message) {
  if (message.type === "init") {
    if (!initializePromise) {
      initializePromise = initialize(message).catch((error) => {
        reportError(error);
        throw error;
      });
    }
    return;
  }
  if (!initializePromise) throw new Error("occupancy worker was not initialized");
  await initializePromise;
  if (!grid || !client) return;

  if (message.type === "event") {
    client.emit(message.eventType, message.value);
    grid.process();
    if (message.eventType === "image") {
      self.postMessage({ type: "frame_done" });
    }
  } else if (message.type === "clear") {
    grid.clear();
  } else if (message.type === "stats") {
    self.postMessage({ type: "stats", stats: grid.stats() });
  }
}

self.onmessage = (event) => {
  void handleMessage(event.data || {}).catch(reportError);
};
