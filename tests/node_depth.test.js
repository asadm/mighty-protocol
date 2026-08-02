import assert from "node:assert";
import proto from "../js/index.js";

const timestampNs = 123456789n;

function makeDepthPayload() {
  return proto.buildDepthPayload({
    timestampNs,
    width: 2,
    height: 2,
    sourceWidth: 2,
    sourceHeight: 2,
    depthIntrinsics: [1, 1, 0, 0],
    sourceIntrinsics: [1, 1, 0, 0],
    distortion: [0, 0, 0, 0],
    sourceChannel: "cam0",
    frameId: "cam0_rectified",
    depthMm: new Uint16Array([0, 1250, 2500, 10000]),
  });
}

class MockDevice {
  async connect(onBytes) {
    this.onBytes = onBytes;
    return new Promise((resolve) => { this.resolve = resolve; });
  }

  async disconnect() {
    this.resolve?.();
  }

  emit(type, payload) {
    this.onBytes?.(proto.makePacket(type, payload));
  }
}

async function main() {
  const payload = makeDepthPayload();
  const decoded = proto.decodeDepthPayload(payload);
  assert.strictEqual(decoded.timestampNs, timestampNs);
  assert.deepStrictEqual(Array.from(decoded.depthMm), [0, 1250, 2500, 10000]);
  assert.ok(Math.abs(proto.depthAtMeters(decoded, 1, 0) - 1.25) < 1e-6);
  assert.strictEqual(proto.depthAtMeters(decoded, 0, 0), null);

  const colored = proto.colorizeDepth(decoded, { nearM: 0, farM: 10 });
  assert.strictEqual(colored.rgba.length, 16);
  assert.strictEqual(colored.rgba[3], 255);

  const raw = {
    kind: "raw",
    timestampNs,
    width: 2,
    height: 2,
    format: proto.RAW_FORMAT.GRAY8,
    channel: "preview",
    data: new Uint8Array([10, 20, 30, 40]),
  };
  const rectified = proto.rectifyImageToDepth(raw, decoded);
  assert.deepStrictEqual(
    Array.from(rectified.rgba),
    [10, 10, 10, 255, 20, 20, 20, 255, 30, 30, 30, 255, 40, 40, 40, 255],
  );
  assert.throws(
    () => proto.rectifyImageToDepth({ ...raw, timestampNs: timestampNs + 1n }, decoded),
    /timestamps do not match/,
  );

  const device = new MockDevice();
  const client = new proto.MightyClient(device, { autoReconnect: false });
  const depths = [];
  const pairs = [];
  client.onDepth((frame) => depths.push(frame));
  client.onRgbd((pair) => pairs.push(pair));
  await client.connect();
  device.emit(proto.TYPE.RAW, proto.buildRawPayload(raw));
  device.emit(proto.TYPE.DPT, payload);
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.strictEqual(depths.length, 1);
  assert.strictEqual(pairs.length, 1);
  assert.strictEqual(pairs[0].image.timestampNs, timestampNs);
  assert.strictEqual(pairs[0].depth.timestampNs, timestampNs);
  await client.disconnect();

  console.log("node depth tests passed");
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
