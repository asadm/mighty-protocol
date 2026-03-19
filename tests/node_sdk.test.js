import assert from "node:assert";
import proto from "../js/index.js";

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

class MockDevice {
  constructor() {
    this._onBytes = null;
    this._resolve = null;
    this._connected = false;
    this._calib = "%YAML:1.0\ncam0:\n  intrinsics: [1,2,3,4]\n";
  }

  getInfo() {
    return { transport: "mock" };
  }

  async connect(onBytes) {
    if (this._connected) throw new Error("already connected");
    this._connected = true;
    this._onBytes = onBytes;
    return new Promise((resolve) => {
      this._resolve = () => {
        this._connected = false;
        this._onBytes = null;
        this._resolve = null;
        resolve();
      };
    });
  }

  async disconnect() {
    if (this._resolve) this._resolve();
  }

  emitPacket(packetBytes) {
    if (this._onBytes) this._onBytes(packetBytes);
  }

  async sendCommandPayload(cmdPayload) {
    const cmd = proto.decodeCommandPayload(cmdPayload);

    if (cmd.name === "start_vio") {
      return proto.buildCommandResponsePayload({
        reqId: cmd.reqId,
        status: 0,
        message: "ok",
        data: new Uint8Array(),
      });
    }

    if (cmd.name === "stop_vio") {
      return proto.buildCommandResponsePayload({
        reqId: cmd.reqId,
        status: 0,
        message: "ok",
        data: new Uint8Array(),
      });
    }

    if (cmd.name === "config") {
      const cfgReq = proto.decodeConfigRequestPayload(cmd.data);
      if (cfgReq.key !== "calib") {
        const cfgErr = proto.buildConfigResponsePayload({
          version: cfgReq.version,
          op: cfgReq.op,
          success: 0,
          hasValue: false,
          key: cfgReq.key,
          message: "unknown key",
          value: new Uint8Array(),
        });
        return proto.buildCommandResponsePayload({
          reqId: cmd.reqId,
          status: 1,
          message: "config failed",
          data: cfgErr,
        });
      }

      if (cfgReq.op === proto.CONFIG_OP.GET) {
        const cfgRes = proto.buildConfigResponsePayload({
          version: cfgReq.version,
          op: cfgReq.op,
          success: 1,
          hasValue: true,
          key: "calib",
          message: "loaded",
          value: Buffer.from(this._calib, "utf8"),
        });
        return proto.buildCommandResponsePayload({
          reqId: cmd.reqId,
          status: 0,
          message: "ok",
          data: cfgRes,
        });
      }

      if (cfgReq.op === proto.CONFIG_OP.SET) {
        this._calib = Buffer.from(cfgReq.value).toString("utf8");
        const cfgRes = proto.buildConfigResponsePayload({
          version: cfgReq.version,
          op: cfgReq.op,
          success: 1,
          hasValue: true,
          key: "calib",
          message: "saved",
          value: Buffer.from(this._calib, "utf8"),
        });
        return proto.buildCommandResponsePayload({
          reqId: cmd.reqId,
          status: 0,
          message: "ok",
          data: cfgRes,
        });
      }
    }

    return proto.buildCommandResponsePayload({
      reqId: cmd.reqId,
      status: 1,
      message: "unknown command",
      data: new Uint8Array(),
    });
  }
}

async function main() {
  assert.strictEqual(typeof proto.MightyClient, "function");
  assert.strictEqual(typeof proto.MightyWebDevice, "function");

  const device = new MockDevice();
  const client = new proto.MightyClient(device, { autoReconnect: false });

  const seen = {
    image: 0,
    pose: 0,
    imu: 0,
    vsta: 0,
    pcld: 0,
    lcon: 0,
    reset: 0,
    status: 0,
    any: 0,
  };

  let lastImage = null;
  let lastPose = null;
  let lastPointCloud = null;

  client.onImage((f) => {
    seen.image += 1;
    lastImage = f;
  });
  client.onPose((p) => {
    seen.pose += 1;
    lastPose = p;
  });
  client.onImu(() => { seen.imu += 1; });
  client.onVioState(() => { seen.vsta += 1; });
  client.onPointCloud((p) => {
    seen.pcld += 1;
    lastPointCloud = p;
  });
  client.onLcon(() => { seen.lcon += 1; });
  client.onReset(() => { seen.reset += 1; });
  client.onStatus(() => { seen.status += 1; });
  client.onAny(() => { seen.any += 1; });

  await client.connect();
  await sleep(5);

  device.emitPacket(proto.makePacket(proto.TYPE.RAW, proto.buildRawPayload({
    timestampNs: 10n,
    width: 2,
    height: 1,
    format: proto.RAW_FORMAT.GRAY8,
    channel: "cam0",
    data: Buffer.from([0x01, 0x02]),
  })));

  device.emitPacket(proto.makePacket(proto.TYPE.POSE, proto.buildPosePayload({
    poseType: 0,
    poseFlags: 0,
    position: [1, 2, 3],
    confidence: 0.5,
    timestampNs: 11n,
  })));

  device.emitPacket(proto.makePacket(proto.TYPE.IMU, proto.buildImuPayload([
    { timestampNs: 12n, ax: 0.1, ay: 0.2, az: 0.3, gx: 0.4, gy: 0.5, gz: 0.6 },
  ])));

  device.emitPacket(proto.makePacket(proto.TYPE.VSTA, proto.buildVioStatePayload({
    version: 3,
    state: 2,
    flags: 3,
    timestampNs: 13n,
    fpsCurrent: 30,
    fpsAverage: 29,
    poseConfidence: 0.8,
    trackingRate: 0.9,
    numFeatures: 100,
    loopClosures: 2,
    buildVersion: "test",
    imuHzCurrent: 200,
    imuHzAverage5s: 199,
  })));

  device.emitPacket(proto.makePacket(proto.TYPE.LCON, proto.buildConstraintsPayload([
    { type: 1, start: [0, 0, 0], end: [1, 1, 1] },
  ])));
  device.emitPacket(proto.makePacket(proto.TYPE.PCLD, proto.buildPcldPayload([
    { x: 1, y: 2, z: 3, r: 10, g: 20, b: 30 },
  ], 0.01)));
  device.emitPacket(proto.makePacket(proto.TYPE.STAT, proto.buildStatusPayload("hello")));
  device.emitPacket(proto.makePacket(proto.TYPE.RSET));
  device.emitPacket(proto.makePacket("ZZZZ", Buffer.from([0xaa])));

  await sleep(20);

  assert.strictEqual(seen.image, 1);
  assert.strictEqual(lastImage.kind, "raw");
  assert.strictEqual(lastImage.channel, "cam0");
  assert.strictEqual(lastImage.channelAlias, "cam0");
  assert.strictEqual(seen.pose, 1);
  assert.strictEqual(lastPose.stream, "optimized");
  assert.strictEqual(lastPose.poseType, "body");
  assert.strictEqual(seen.imu, 1);
  assert.strictEqual(seen.vsta, 1);
  assert.strictEqual(seen.pcld, 1);
  assert.ok(Array.isArray(lastPointCloud.points));
  assert.strictEqual(lastPointCloud.points.length, 1);
  assert.strictEqual(seen.lcon, 1);
  assert.strictEqual(seen.status, 1);
  assert.strictEqual(seen.reset, 1);
  assert.ok(seen.any >= 9);

  const cmdRes = await client.startVio();
  assert.strictEqual(cmdRes.ok, true);

  const cfgGet = await client.configGet("calib", { as: "text" });
  assert.strictEqual(cfgGet.ok, true);
  assert.strictEqual(cfgGet.found, true);
  assert.ok(cfgGet.value.includes("intrinsics"));

  const setText = "%YAML:1.0\nfoo: 1\n";
  const cfgSet = await client.configSet("calib", setText);
  assert.strictEqual(cfgSet.ok, true);

  const cfgGet2 = await client.configGet("calib", { as: "text" });
  assert.strictEqual(cfgGet2.ok, true);
  assert.strictEqual(cfgGet2.value, setText);

  const stats = client.stats();
  assert.ok(stats.rxFrames >= 8);
  assert.ok(stats.rxBytes > 0);

  await client.disconnect();
  assert.strictEqual(client.isConnected(), false);

  console.log("Node SDK client test passed");
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
