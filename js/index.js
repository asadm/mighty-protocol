import * as core from "./core/protocol.js";
import { MightyClient, MightyWebDevice, DEFAULT_BASE_URLS, decodeRawToRgb } from "./sdk/index.js";

const { default: _coreDefault, ...coreNamed } = core;

const sdk = {
  MightyClient,
  MightyWebDevice,
  DEFAULT_BASE_URLS,
  decodeRawToRgb,
};

const api = {
  ...coreNamed,
  ...sdk,
  sdk,
  core: coreNamed,
};

export * from "./core/protocol.js";
export { MightyClient, MightyWebDevice, DEFAULT_BASE_URLS, decodeRawToRgb, sdk, coreNamed as core };
export default api;
