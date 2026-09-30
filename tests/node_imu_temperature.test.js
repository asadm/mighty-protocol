import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import proto from "../js/index.js";

const samples = Array.from({ length: 5 }, (_, i) => ({
  timestampNs: 1000000000n + BigInt(i) * 1250000n, ax: i, ay: 2, az: 3, gx: 4, gy: 5, gz: 6,
}));
const legacy = Buffer.from(proto.buildImuPayload(samples));
samples[0].temperatureC = -12.5;
samples[1].temperatureC = 0;
samples[3].temperatureC = 46.5;
samples[4].temperatureC = Infinity;
const modern = Buffer.from(proto.buildImuPayload(samples));
const expected = samples.map(({ temperatureC, ...s }) =>
  Number.isFinite(temperatureC) ? { ...s, temperatureC } : s);
assert.deepEqual(proto.decodeImuPayload(modern), expected);
assert.deepEqual(modern.subarray(0, legacy.length), legacy);
assert.deepEqual(proto.decodeImuPayload(legacy), expected.map(({ temperatureC, ...s }) => s));
assert.deepEqual(Buffer.from(proto.buildImuPayload(samples.map((s) => ({ ...s, temperatureC: NaN })))), legacy);
// A legacy reader consumes count * 56 bytes even when a new trailer is present.
for (let i = 0; i < modern.readUInt32BE(0); ++i) {
  assert.equal(modern.readBigUInt64BE(4 + 56 * i), samples[i].timestampNs);
  assert.equal(modern.readDoubleBE(4 + 56 * i + 8), samples[i].ax);
  assert.equal(modern.readDoubleBE(4 + 56 * i + 48), samples[i].gz);
}
const client = new proto.MightyClient({ connect() {}, disconnect() {} });
let received;
client.onImu((batch) => { received = batch.samples; });
client._handleFrame({ type: proto.TYPE.IMU, payload: modern });
assert.deepEqual(received, expected);
for (const name of ["legacy", "modern", "truncated", "unknown", "bad-count"]) {
  if (!process.argv[2]) continue;
  const bytes = fs.readFileSync(path.join(process.argv[2], `${name}.bin`));
  assert.deepEqual(proto.decodeImuPayload(bytes), name === "modern"
    ? expected : expected.map(({ temperatureC, ...s }) => s));
  if (name === "modern") assert.deepEqual(bytes, modern);
  if (name === "legacy") assert.deepEqual(bytes, legacy);
}
assert.throws(() => proto.decodeImuPayload(Buffer.from([0xff, 0xff, 0xff, 0xff])));
console.log("IMU temperature: JavaScript/SDK and C++ wire parity passed");
