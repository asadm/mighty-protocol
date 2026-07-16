import assert from "node:assert";
import proto from "../js/index.js";

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function almost(a, b, eps = 1e-6) {
  return Math.abs(Number(a) - Number(b)) < eps;
}

class MockDevice {
  constructor() {
    this._onBytes = null;
    this._resolve = null;
    this._connected = false;
    this._calib = "%YAML:1.0\ncam0:\n  intrinsics: [1,2,3,4]\n";
    this.lastResetPose = null;
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

    if (cmd.name === "reset_vio_pose") {
      this.lastResetPose = proto.decodeResetVioPosePayload(cmd.data);
      return proto.buildCommandResponsePayload({
        reqId: cmd.reqId,
        status: 0,
        message: "pose reset",
        data: new Uint8Array(),
      });
    }

    if (cmd.name === "keyframes") {
      const action = Buffer.from(cmd.data || new Uint8Array()).toString("utf8");
      return proto.buildCommandResponsePayload({
        reqId: cmd.reqId,
        status: 0,
        message: `keyframes ${action === "status" ? "disabled" : action}`,
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

async function testLoopclosureConnectFailurePolicy() {
  const originalConsoleError = console.error;
  console.error = () => {};

  try {
    const strictDevice = new MockDevice();
    const strictErrors = [];
    const strictClient = new proto.MightyClient(strictDevice, {
      autoReconnect: false,
      loopclosure: true,
      loopclosureWasmModule: {},
    });
    strictClient.onError((error) => strictErrors.push(error));

    await assert.rejects(strictClient.connect(), /_malloc/);
    assert.strictEqual(strictDevice._connected, false);
    assert.strictEqual(strictClient.isConnected(), false);
    assert.strictEqual(strictErrors.length, 1);
    assert.strictEqual(strictErrors[0].scope, "loopclosure");
    assert.strictEqual(strictErrors[0].code, "initialize_failed");
    await strictClient.disconnect();

    const failOpenDevice = new MockDevice();
    const failOpenErrors = [];
    let statuses = 0;
    const failOpenClient = new proto.MightyClient(failOpenDevice, {
      autoReconnect: false,
      loopclosure: true,
      loopclosureWasmModule: {},
      loopclosureFailOpen: true,
    });
    failOpenClient.onError((error) => failOpenErrors.push(error));
    failOpenClient.onStatus(() => { statuses += 1; });

    await failOpenClient.connect();
    assert.strictEqual(failOpenDevice._connected, true);
    assert.strictEqual(failOpenClient.isConnected(), true);
    assert.strictEqual(failOpenErrors.length, 1);
    assert.strictEqual(failOpenErrors[0].scope, "loopclosure");
    assert.strictEqual(failOpenErrors[0].code, "initialize_failed");
    assert.ok(failOpenErrors[0].cause instanceof TypeError);
    assert.match(failOpenErrors[0].cause.message, /_malloc/);
    assert.strictEqual(failOpenClient._loopclosure, null);

    failOpenDevice.emitPacket(proto.makePacket(
      proto.TYPE.STAT,
      proto.buildStatusPayload("still connected"),
    ));
    await sleep(5);
    assert.strictEqual(statuses, 1);

    const result = await failOpenClient.startVio();
    assert.strictEqual(result.ok, true);

    await failOpenClient.disconnect();
    assert.strictEqual(failOpenClient.isConnected(), false);
  } finally {
    console.error = originalConsoleError;
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
    keyframe: 0,
    reset: 0,
    status: 0,
    any: 0,
  };

  let lastImage = null;
  let lastPose = null;
  let lastVsta = null;
  let lastPointCloud = null;
  let lastKeyframe = null;

  client.onImage((f) => {
    seen.image += 1;
    lastImage = f;
  });
  client.onPose((p) => {
    seen.pose += 1;
    lastPose = p;
  });
  client.onImu(() => { seen.imu += 1; });
  client.onVioState((v) => {
    seen.vsta += 1;
    lastVsta = v;
  });
  client.onPointCloud((p) => {
    seen.pcld += 1;
    lastPointCloud = p;
  });
  client.onLcon(() => { seen.lcon += 1; });
  client.onKeyframe((k) => {
    seen.keyframe += 1;
    lastKeyframe = k;
  });
  client.onReset(() => { seen.reset += 1; });
  client.onStatus(() => { seen.status += 1; });
  client.onAny(() => { seen.any += 1; });

  await client.connect();
  await sleep(5);

  const poseSample = {
    poseType: 0,
    poseFlags: 0,
    positionM: [1, 2, 3],
    orientationXyzw: [0.1, 0.2, 0.3, 0.9],
    confidence: 0.5,
    linearVelocityBodyMps: [4.0, 5.0, 6.0],
    angularVelocityBodyRps: [0.4, 0.5, 0.6],
    linearAccelerationBodyMps2: [7.0, 8.0, 9.0],
    angularAccelerationBodyRps2: [0.7, 0.8, 0.9],
    timestampNs: 11n,
  };

  device.emitPacket(proto.makePacket(proto.TYPE.RAW, proto.buildRawPayload({
    timestampNs: 10n,
    width: 2,
    height: 1,
    format: proto.RAW_FORMAT.GRAY8,
    channel: "cam0",
    data: Buffer.from([0x01, 0x02]),
  })));

  device.emitPacket(proto.makePacket(proto.TYPE.POSE, proto.buildPosePayload(poseSample)));

  device.emitPacket(proto.makePacket(proto.TYPE.IMU, proto.buildImuPayload([
    { timestampNs: 12n, ax: 0.1, ay: 0.2, az: 0.3, gx: 0.4, gy: 0.5, gz: 0.6 },
  ])));

  device.emitPacket(proto.makePacket(proto.TYPE.VSTA, proto.buildVioStatePayload({
    version: 8,
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
    initReasonCode: proto.VIO_INIT_REASON.NONE,
    translationConfidence01: 0.34,
    translationObservability01: 0.21,
    degradedReasonFlags:
      proto.VIO_DEGRADED_REASON.LOW_TRANSLATION_OBSERVABILITY |
      proto.VIO_DEGRADED_REASON.LOW_PARALLAX_POSE_HOLD |
      proto.VIO_DEGRADED_REASON.STATIC_TRANSLATION_CONSTRAINED |
      proto.VIO_DEGRADED_REASON.ROTATION_ONLY_3DOF,
  })));

  device.emitPacket(proto.makePacket(proto.TYPE.LCON, proto.buildConstraintsPayload([
    { type: 1, start: [0, 0, 0], end: [1, 1, 1] },
  ])));
  device.emitPacket(proto.makePacket(proto.TYPE.PCLD, proto.buildPcldPayload([
    { x: 1, y: 2, z: 3, r: 10, g: 20, b: 30 },
  ], 0.01)));
  device.emitPacket(proto.makePacket(proto.TYPE.KEYF, proto.buildKeyframePayload({
    version: 2,
    flags: proto.KEYFRAME_FLAG_LOCAL_FEATURES,
    timestampNs: 14n,
    descriptor: [0.25, -0.5, 1.0],
    imageWidth: 640,
    imageHeight: 400,
    featureDescriptorDim: 2,
    features: [
      { x: 22.5, y: 380, score: 0.8, descriptor: [0.1, -0.2] },
    ],
  })));
  device.emitPacket(proto.makePacket(proto.TYPE.STAT, proto.buildStatusPayload("hello")));
  device.emitPacket(proto.makePacket(proto.TYPE.RSET));
  device.emitPacket(proto.makePacket("ZZZZ", Buffer.from([0xaa])));

  await sleep(20);

  assert.strictEqual(seen.image, 1);
  assert.strictEqual(lastImage.kind, "raw");
  assert.strictEqual(lastImage.channel, "cam0");
  assert.strictEqual(lastImage.channelAlias, "cam0");
  assert.strictEqual(seen.pose, 1);
  assert.strictEqual(lastPose.isPublic, true);
  assert.strictEqual(lastPose.packetType, "POSE");
  assert.strictEqual(lastPose.poseType, "body");
  assert.strictEqual(lastPose.poseTypeRaw, 0);
  assert.strictEqual(lastPose.frameId, "odom");
  assert.strictEqual(lastPose.childFrameId, "base_link");
  assert.ok((lastPose.poseFlags & 0x1) !== 0);
  assert.ok((lastPose.poseFlags & (1 << 2)) !== 0);
  assert.ok((lastPose.poseFlags & (1 << 3)) !== 0);
  assert.ok((lastPose.poseFlags & (1 << 4)) !== 0);
  assert.ok((lastPose.poseFlags & (1 << 5)) !== 0);
  assert.ok((lastPose.poseFlags & (1 << 6)) !== 0);
  assert.strictEqual(lastPose.timestampNs, 11n);
  assert.ok(Array.isArray(lastPose.positionM));
  assert.ok(Array.isArray(lastPose.orientationXyzw));
  assert.ok(Array.isArray(lastPose.linearVelocityBodyMps));
  assert.ok(Array.isArray(lastPose.angularVelocityBodyRps));
  assert.ok(Array.isArray(lastPose.linearAccelerationBodyMps2));
  assert.ok(Array.isArray(lastPose.angularAccelerationBodyRps2));
  assert.ok(almost(lastPose.orientationXyzw[0], poseSample.orientationXyzw[0]));
  assert.ok(almost(lastPose.orientationXyzw[3], poseSample.orientationXyzw[3]));
  assert.ok(almost(lastPose.linearVelocityBodyMps[2], poseSample.linearVelocityBodyMps[2]));
  assert.ok(almost(lastPose.angularVelocityBodyRps[1], poseSample.angularVelocityBodyRps[1]));
  assert.ok(almost(lastPose.linearAccelerationBodyMps2[0], poseSample.linearAccelerationBodyMps2[0]));
  assert.ok(almost(lastPose.angularAccelerationBodyRps2[2], poseSample.angularAccelerationBodyRps2[2]));

  const pushedLoopclosurePoses = [];
  let correctedLoopclosurePoses = 0;
  client._loopclosure = {
    pushPose(pose) {
      pushedLoopclosurePoses.push(pose);
    },
    correctPose(pose) {
      correctedLoopclosurePoses += 1;
      return {
        ...pose,
        rawPositionM: pose.positionM.map(Number),
        positionM: pose.positionM.map((v) => Number(v) + 10),
        loopclosureCorrected: true,
      };
    },
  };

  device.emitPacket(proto.makePacket(proto.TYPE.POSE, proto.buildPosePayload({
    poseType: 2,
    positionM: [9, 8, 7],
    timestampNs: 15n,
  })));
  await sleep(5);
  assert.strictEqual(lastPose.poseType, "other");
  assert.strictEqual(lastPose.poseTypeRaw, 2);
  assert.deepStrictEqual(lastPose.positionM, [9, 8, 7]);
  assert.strictEqual(lastPose.rawPositionM, undefined);
  assert.strictEqual(lastPose.loopclosureCorrected, undefined);
  assert.strictEqual(pushedLoopclosurePoses.length, 0);
  assert.strictEqual(correctedLoopclosurePoses, 0);

  device.emitPacket(proto.makePacket(proto.TYPE.POSE, proto.buildPosePayload({
    poseType: 0,
    positionM: [2, 3, 4],
    timestampNs: 16n,
  })));
  await sleep(5);
  assert.strictEqual(lastPose.poseType, "body");
  assert.deepStrictEqual(lastPose.rawPositionM, [2, 3, 4]);
  assert.deepStrictEqual(lastPose.positionM, [12, 13, 14]);
  assert.strictEqual(lastPose.loopclosureCorrected, true);
  assert.strictEqual(pushedLoopclosurePoses.length, 1);
  assert.strictEqual(correctedLoopclosurePoses, 1);
  client._loopclosure = null;

  assert.strictEqual(seen.imu, 1);
  assert.strictEqual(seen.vsta, 1);
  assert.strictEqual(lastVsta.initReasonCode, proto.VIO_INIT_REASON.NONE);
  assert.ok(almost(lastVsta.translationConfidence01, 0.34));
  assert.ok(almost(lastVsta.translationObservability01, 0.21));
  assert.strictEqual(
    lastVsta.degradedReasonFlags,
    proto.VIO_DEGRADED_REASON.LOW_TRANSLATION_OBSERVABILITY |
      proto.VIO_DEGRADED_REASON.LOW_PARALLAX_POSE_HOLD |
      proto.VIO_DEGRADED_REASON.STATIC_TRANSLATION_CONSTRAINED |
      proto.VIO_DEGRADED_REASON.ROTATION_ONLY_3DOF,
  );
  assert.strictEqual(seen.pcld, 1);
  assert.ok(Array.isArray(lastPointCloud.points));
  assert.strictEqual(lastPointCloud.points.length, 1);
  assert.strictEqual(seen.lcon, 1);
  assert.strictEqual(seen.keyframe, 1);
  assert.strictEqual(lastKeyframe.timestampNs, 14n);
  assert.strictEqual(lastKeyframe.descriptorDim, 3);
  assert.ok(almost(lastKeyframe.descriptor[1], -0.5));
  assert.strictEqual(lastKeyframe.version, 2);
  assert.strictEqual(lastKeyframe.imageWidth, 640);
  assert.strictEqual(lastKeyframe.imageHeight, 400);
  assert.strictEqual(lastKeyframe.featureCount, 1);
  assert.strictEqual(lastKeyframe.featureDescriptorDim, 2);
  assert.ok(almost(lastKeyframe.features[0].y, 380));
  assert.ok(almost(lastKeyframe.features[0].score, 0.8));
  assert.strictEqual(seen.status, 1);
  assert.strictEqual(seen.reset, 1);
  assert.ok(seen.any >= 10);

  const cmdRes = await client.startVio();
  assert.strictEqual(cmdRes.ok, true);

  const resetPoseRes = await client.resetVioPose({ positionM: [0, 0, 0] });
  assert.strictEqual(resetPoseRes.ok, true);
  assert.deepStrictEqual(device.lastResetPose.positionM, [0, 0, 0]);
  assert.strictEqual(device.lastResetPose.orientationXyzw, null);

  const resetPoseQuatRes = await client.resetVioPose({
    positionM: [1, 2, 3],
    orientationXyzw: [0, 0, 0, 1],
  });
  assert.strictEqual(resetPoseQuatRes.ok, true);
  assert.deepStrictEqual(device.lastResetPose.positionM, [1, 2, 3]);
  assert.deepStrictEqual(device.lastResetPose.orientationXyzw, [0, 0, 0, 1]);

  const keyframesOn = await client.setKeyframesEnabled(true);
  assert.strictEqual(keyframesOn.ok, true);
  assert.strictEqual(keyframesOn.message, "keyframes on");

  const keyframesStatus = await client.keyframesStatus();
  assert.strictEqual(keyframesStatus.ok, true);
  assert.strictEqual(keyframesStatus.message, "keyframes disabled");

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

  await testLoopclosureConnectFailurePolicy();

  console.log("Node SDK client test passed");
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
