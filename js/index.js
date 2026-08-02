import * as core from "./core/protocol.js";
import {
  MightyClient,
  MightyWebDevice,
  DEFAULT_BASE_URLS,
  DEFAULT_LOOPCLOSURE_WASM_URL,
  NativeLoopClosureWasm,
  NativeMapperWasm,
  createLoopClosureWasmModule,
  decodeRawToRgb,
  RgbdSynchronizer,
  colorizeDepth,
  depthAtMeters,
  isUint16MetricDepth,
  rectifyImageToDepth,
} from "./sdk/index.js";

const { default: _coreDefault, ...coreNamed } = core;

const sdk = {
  MightyClient,
  MightyWebDevice,
  DEFAULT_BASE_URLS,
  DEFAULT_LOOPCLOSURE_WASM_URL,
  NativeLoopClosureWasm,
  NativeMapperWasm,
  createLoopClosureWasmModule,
  decodeRawToRgb,
  RgbdSynchronizer,
  colorizeDepth,
  depthAtMeters,
  isUint16MetricDepth,
  rectifyImageToDepth,
};

const api = {
  ...coreNamed,
  ...sdk,
  sdk,
  core: coreNamed,
};

export * from "./core/protocol.js";
export {
  MightyClient,
  MightyWebDevice,
  DEFAULT_BASE_URLS,
  DEFAULT_LOOPCLOSURE_WASM_URL,
  NativeLoopClosureWasm,
  NativeMapperWasm,
  createLoopClosureWasmModule,
  decodeRawToRgb,
  RgbdSynchronizer,
  colorizeDepth,
  depthAtMeters,
  isUint16MetricDepth,
  rectifyImageToDepth,
  sdk,
  coreNamed as core,
};
export default api;
