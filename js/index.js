const protocol = require("./core/protocol");
const sdk = require("./sdk");

const api = {
  ...protocol,
  ...sdk,
  sdk,
  core: protocol,
};

if (typeof module !== "undefined" && module.exports) {
  module.exports = api;
  module.exports.default = api;
}
if (typeof globalThis !== "undefined" && globalThis.window === globalThis) {
  globalThis.MightyProtocol = api;
}
