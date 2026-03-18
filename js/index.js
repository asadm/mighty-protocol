import * as core from "./core/protocol.js";
import { MightyClient, MightyWebDevice, DEFAULT_BASE_URLS } from "./sdk/index.js";

const { default: _coreDefault, ...coreNamed } = core;

const sdk = {
  MightyClient,
  MightyWebDevice,
  DEFAULT_BASE_URLS,
};

const api = {
  ...coreNamed,
  ...sdk,
  sdk,
  core: coreNamed,
};

export * from "./core/protocol.js";
export { MightyClient, MightyWebDevice, DEFAULT_BASE_URLS, sdk, coreNamed as core };
export default api;
