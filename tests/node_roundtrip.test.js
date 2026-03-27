import net from "node:net";
import path from "node:path";
import assert from "node:assert";
import crypto from "node:crypto";
import { spawn } from "node:child_process";
import { fileURLToPath } from "node:url";

import proto from "../js/index.js";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const SAMPLE = {
  jpgTs: 111n,
  jpgChannel: 'preview',
  jpgData: Buffer.from([0x01, 0x02, 0x03]),
  rjpgTs: 222n,
  rjpgData: Buffer.from([0xaa, 0xbb]),
  rawTs: 333n,
  rawChannel: 'cam0',
  rawWidth: 4,
  rawHeight: 2,
  rawFormat: proto.RAW_FORMAT.GRAY8,
  rawData: Buffer.from([0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]),
  srawLeftTs: 444n,
  srawRightTs: 445n,
  srawLeftChannel: 'cam0',
  srawRightChannel: 'cam1',
  srawLeftWidth: 2,
  srawLeftHeight: 1,
  srawLeftFormat: proto.RAW_FORMAT.GRAY8,
  srawLeftData: Buffer.from([0x21, 0x22]),
  srawRightWidth: 2,
  srawRightHeight: 1,
  srawRightFormat: proto.RAW_FORMAT.GRAY8,
  srawRightData: Buffer.from([0x31, 0x32]),
  pose: {
    poseType: 0,
    poseFlags: 0x3,
    positionM: [1.1, 2.2, 3.3],
    orientationXyzw: [0.1, 0.2, 0.3, 0.9],
    confidence: 0.82,
    linearVelocityBodyMps: [4.0, 5.0, 6.0],
    angularVelocityBodyRps: [0.4, 0.5, 0.6],
    linearAccelerationBodyMps2: [7.0, 8.0, 9.0],
    angularAccelerationBodyRps2: [0.7, 0.8, 0.9],
    timestampNs: 777n
  },
  upose: {
    poseType: 0,
    poseFlags: 0x1,
    positionM: [4.4, 5.5, 6.6],
    orientationXyzw: [0.4, 0.5, 0.6, 0.7],
    confidence: 0.41,
    linearVelocityBodyMps: [1.0, 1.1, 1.2],
    angularVelocityBodyRps: [0.1, 0.2, 0.3],
    linearAccelerationBodyMps2: [2.0, 2.1, 2.2],
    angularAccelerationBodyRps2: [0.01, 0.02, 0.03],
    timestampNs: 778n
  },
  constraints: [
    { type: 0, start: [0.1, 0.2, 0.3], end: [0.4, 0.5, 0.6] },
    { type: 1, start: [1.0, 1.1, 1.2], end: [1.3, 1.4, 1.5] },
  ],
  viz0: { subtype: 0, features: [{ x: 10, y: 20, status: 1, id: 7 }, { x: 30, y: 40, status: 4, id: 8 }] },
  viz1: { subtype: 1, detections: [{ x1: 5, y1: 6, x2: 25, y2: 26, label: 'car' }] },
  viz2: { subtype: 2, matches: [{ x1: 100, y1: 110, x2: 120, y2: 130, confidence: 200 }] },
  imu: [
    { timestampNs: 1000n, ax: 0.1, ay: 0.2, az: 0.3, gx: 1.1, gy: 1.2, gz: 1.3 },
    { timestampNs: 2000n, ax: 0.4, ay: 0.5, az: 0.6, gx: 1.4, gy: 1.5, gz: 1.6 },
  ],
  status: 'STATUS_OK',
  fea3: [
    { id: 1, x: 0.1, y: 0.2, z: 0.3 },
    { id: 2, x: 1.1, y: 1.2, z: 1.3 },
  ],
  pcld: {
    points: [
      { x: 1, y: 2, z: 3, r: 10, g: 20, b: 30 },
      { x: 4, y: 5, z: 6, r: 40, g: 50, b: 60 },
    ],
    pointSize: 1.5,
  },
  vsta: {
    version: 4,
    state: 2,
    flags: 0x1234,
    timestampNs: 999n,
    fpsCurrent: 31.5,
    fpsAverage: 30.0,
    imuHzCurrent: 60.0,
    imuHzAverage5s: 59.7,
    poseConfidence: 0.75,
    trackingRate: 0.88,
    numFeatures: 321,
    loopClosures: 7,
    buildVersion: "Mighty v.20260208-deadbeef",
    initReasonCode: proto.VIO_INIT_REASON.NONE,
  },
  cfgq: {
    version: 1,
    op: proto.CONFIG_OP.SET,
    key: "calib",
    value: Buffer.from("cam0:\n  intrinsics: [369.5, 369.2, 311.2, 200.4]\n"),
  },
  cfgr: {
    version: 1,
    op: proto.CONFIG_OP.SET,
    success: 1,
    hasValue: true,
    key: "calib",
    message: "saved calib.yaml",
    value: Buffer.from("%YAML:1.0\ncam0:\n  intrinsics: [369.5, 369.2, 311.2, 200.4]\n"),
  },
};

const EXPECTED_COUNT = 18;

function buildPackets() {
  return [
    proto.makePacket(proto.TYPE.RSET),
    proto.makePacket(proto.TYPE.JPG, proto.buildJpgPayload({ timestampNs: SAMPLE.jpgTs, channel: SAMPLE.jpgChannel, data: SAMPLE.jpgData, isRef: false })),
    proto.makePacket(proto.TYPE.RJPG, proto.buildJpgPayload({ timestampNs: SAMPLE.rjpgTs, data: SAMPLE.rjpgData, isRef: true })),
    proto.makePacket(proto.TYPE.RAW, proto.buildRawPayload({
      timestampNs: SAMPLE.rawTs,
      width: SAMPLE.rawWidth,
      height: SAMPLE.rawHeight,
      format: SAMPLE.rawFormat,
      channel: SAMPLE.rawChannel,
      data: SAMPLE.rawData,
    })),
    proto.makePacket(proto.TYPE.SRAW, proto.buildStereoRawPayload({
      left: {
        timestampNs: SAMPLE.srawLeftTs,
        width: SAMPLE.srawLeftWidth,
        height: SAMPLE.srawLeftHeight,
        format: SAMPLE.srawLeftFormat,
        channel: SAMPLE.srawLeftChannel,
        data: SAMPLE.srawLeftData,
      },
      right: {
        timestampNs: SAMPLE.srawRightTs,
        width: SAMPLE.srawRightWidth,
        height: SAMPLE.srawRightHeight,
        format: SAMPLE.srawRightFormat,
        channel: SAMPLE.srawRightChannel,
        data: SAMPLE.srawRightData,
      },
    })),
    proto.makePacket(proto.TYPE.POSE, proto.buildPosePayload(SAMPLE.pose)),
    proto.makePacket(proto.TYPE.UPOSE, proto.buildPosePayload(SAMPLE.upose)),
    proto.makePacket(proto.TYPE.LCON, proto.buildConstraintsPayload(SAMPLE.constraints)),
    proto.makePacket(proto.TYPE.VIZ, proto.buildVizPayload(SAMPLE.viz0)),
    proto.makePacket(proto.TYPE.VIZ, proto.buildVizPayload(SAMPLE.viz1)),
    proto.makePacket(proto.TYPE.VIZ, proto.buildVizPayload(SAMPLE.viz2)),
    proto.makePacket(proto.TYPE.IMU, proto.buildImuPayload(SAMPLE.imu)),
    proto.makePacket(proto.TYPE.STAT, proto.buildStatusPayload(SAMPLE.status)),
    proto.makePacket(proto.TYPE.VSTA, proto.buildVioStatePayload(SAMPLE.vsta)),
    proto.makePacket(proto.TYPE.FEA3, proto.buildFea3Payload(SAMPLE.fea3)),
    proto.makePacket(proto.TYPE.PCLD, proto.buildPcldPayload(SAMPLE.pcld.points, SAMPLE.pcld.pointSize)),
    proto.makePacket(proto.TYPE.CFGQ, proto.buildConfigRequestPayload(SAMPLE.cfgq)),
    proto.makePacket(proto.TYPE.CFGR, proto.buildConfigResponsePayload(SAMPLE.cfgr)),
  ];
}

function almost(a, b, eps = 1e-6) {
  return Math.abs(a - b) < eps;
}

function verifyFrame(frame, index) {
  const { type, payload } = frame;
  switch (type) {
    case proto.TYPE.RSET:
      assert.strictEqual(index, 0);
      assert.strictEqual(payload.length, 0);
      break;
    case proto.TYPE.JPG: {
      const res = proto.decodeJpgPayload(payload, false);
      assert.strictEqual(res.timestampNs, SAMPLE.jpgTs);
      assert.strictEqual(res.channel, SAMPLE.jpgChannel);
      assert.deepStrictEqual(res.data, SAMPLE.jpgData);
      break;
    }
    case proto.TYPE.RJPG: {
      const res = proto.decodeJpgPayload(payload, true);
      assert.strictEqual(res.timestampNs, SAMPLE.rjpgTs);
      assert.deepStrictEqual(res.data, SAMPLE.rjpgData);
      break;
    }
    case proto.TYPE.RAW: {
      const res = proto.decodeRawPayload(payload);
      assert.strictEqual(res.timestampNs, SAMPLE.rawTs);
      assert.strictEqual(res.width, SAMPLE.rawWidth);
      assert.strictEqual(res.height, SAMPLE.rawHeight);
      assert.strictEqual(res.format, SAMPLE.rawFormat);
      assert.strictEqual(res.channel, SAMPLE.rawChannel);
      assert.deepStrictEqual(res.data, SAMPLE.rawData);
      break;
    }
    case proto.TYPE.SRAW: {
      const res = proto.decodeStereoRawPayload(payload);
      assert.strictEqual(res.left.timestampNs, SAMPLE.srawLeftTs);
      assert.strictEqual(res.left.width, SAMPLE.srawLeftWidth);
      assert.strictEqual(res.left.height, SAMPLE.srawLeftHeight);
      assert.strictEqual(res.left.format, SAMPLE.srawLeftFormat);
      assert.strictEqual(res.left.channel, SAMPLE.srawLeftChannel);
      assert.deepStrictEqual(res.left.data, SAMPLE.srawLeftData);
      assert.strictEqual(res.right.timestampNs, SAMPLE.srawRightTs);
      assert.strictEqual(res.right.width, SAMPLE.srawRightWidth);
      assert.strictEqual(res.right.height, SAMPLE.srawRightHeight);
      assert.strictEqual(res.right.format, SAMPLE.srawRightFormat);
      assert.strictEqual(res.right.channel, SAMPLE.srawRightChannel);
      assert.deepStrictEqual(res.right.data, SAMPLE.srawRightData);
      break;
    }
	    case proto.TYPE.POSE: {
	      const res = proto.decodePosePayload(payload);
	      assert.strictEqual(res.poseType, SAMPLE.pose.poseType);
	      assert.strictEqual(res.poseFlags & 0x3, 0x3);
	      assert.ok((res.poseFlags & (1 << 2)) !== 0);
	      assert.ok((res.poseFlags & (1 << 3)) !== 0);
	      assert.ok((res.poseFlags & (1 << 4)) !== 0);
	      assert.ok((res.poseFlags & (1 << 5)) !== 0);
	      assert.ok((res.poseFlags & (1 << 6)) !== 0);
	      res.positionM.forEach((v, i) => assert(almost(v, SAMPLE.pose.positionM[i])));
	      res.orientationXyzw.forEach((v, i) => assert(almost(v, SAMPLE.pose.orientationXyzw[i])));
	      res.linearVelocityBodyMps.forEach((v, i) => assert(almost(v, SAMPLE.pose.linearVelocityBodyMps[i])));
	      res.angularVelocityBodyRps.forEach((v, i) => assert(almost(v, SAMPLE.pose.angularVelocityBodyRps[i])));
	      res.linearAccelerationBodyMps2.forEach((v, i) => assert(almost(v, SAMPLE.pose.linearAccelerationBodyMps2[i])));
	      res.angularAccelerationBodyRps2.forEach((v, i) => assert(almost(v, SAMPLE.pose.angularAccelerationBodyRps2[i])));
	      assert.strictEqual(res.timestampNs, SAMPLE.pose.timestampNs);
	      assert(almost(res.confidence, SAMPLE.pose.confidence, 1e-3));
	      break;
	    }
	    case proto.TYPE.UPOSE: {
	      const res = proto.decodePosePayload(payload);
	      assert.strictEqual(res.poseType, SAMPLE.upose.poseType);
	      assert(res.poseFlags & 0x1);
	      assert(res.poseFlags & (1 << 2));
	      assert(res.poseFlags & (1 << 3));
	      assert(res.poseFlags & (1 << 4));
	      assert(res.poseFlags & (1 << 5));
	      assert(res.poseFlags & (1 << 6));
	      assert(almost(res.positionM[2], SAMPLE.upose.positionM[2]));
	      res.orientationXyzw.forEach((v, i) => assert(almost(v, SAMPLE.upose.orientationXyzw[i])));
	      res.linearVelocityBodyMps.forEach((v, i) => assert(almost(v, SAMPLE.upose.linearVelocityBodyMps[i])));
	      res.angularVelocityBodyRps.forEach((v, i) => assert(almost(v, SAMPLE.upose.angularVelocityBodyRps[i])));
	      res.linearAccelerationBodyMps2.forEach((v, i) => assert(almost(v, SAMPLE.upose.linearAccelerationBodyMps2[i])));
	      res.angularAccelerationBodyRps2.forEach((v, i) => assert(almost(v, SAMPLE.upose.angularAccelerationBodyRps2[i])));
	      assert.strictEqual(res.timestampNs, SAMPLE.upose.timestampNs);
	      assert(almost(res.confidence, SAMPLE.upose.confidence, 1e-3));
	      break;
	    }
    case proto.TYPE.LCON: {
      const segs = proto.decodeConstraintsPayload(payload);
      assert.strictEqual(segs.length, SAMPLE.constraints.length);
      assert.strictEqual(segs[1].type, 1);
      break;
    }
    case proto.TYPE.VIZ: {
      const res = proto.decodeVizPayload(payload);
      if (res.subtype === 0) assert.strictEqual(res.features.length, SAMPLE.viz0.features.length);
      if (res.subtype === 1) assert.strictEqual(res.detections[0].label, 'car');
      if (res.subtype === 2) assert.strictEqual(res.matches[0].confidence, 200);
      break;
    }
    case proto.TYPE.IMU: {
      const imu = proto.decodeImuPayload(payload);
      assert.strictEqual(imu.length, SAMPLE.imu.length);
      assert(almost(Number(imu[0].gx), SAMPLE.imu[0].gx));
      break;
    }
    case proto.TYPE.STAT:
      assert.strictEqual(proto.decodeStatusPayload(payload), SAMPLE.status);
      break;
    case proto.TYPE.VSTA: {
      const s = proto.decodeVioStatePayload(payload);
      assert.strictEqual(s.version, SAMPLE.vsta.version);
      assert.strictEqual(s.state, SAMPLE.vsta.state);
      assert.strictEqual(s.flags, SAMPLE.vsta.flags);
      assert.strictEqual(s.timestampNs, SAMPLE.vsta.timestampNs);
      assert(almost(s.fpsCurrent, SAMPLE.vsta.fpsCurrent, 1e-3));
      assert(almost(s.fpsAverage, SAMPLE.vsta.fpsAverage, 1e-3));
      assert(almost(s.imuHzCurrent, SAMPLE.vsta.imuHzCurrent, 1e-3));
      assert(almost(s.imuHzAverage5s, SAMPLE.vsta.imuHzAverage5s, 1e-3));
      assert(almost(s.poseConfidence, SAMPLE.vsta.poseConfidence, 1e-3));
      assert(almost(s.trackingRate, SAMPLE.vsta.trackingRate, 1e-3));
      assert.strictEqual(s.numFeatures, SAMPLE.vsta.numFeatures);
      assert.strictEqual(s.loopClosures, SAMPLE.vsta.loopClosures);
      assert.strictEqual(s.buildVersion, SAMPLE.vsta.buildVersion);
      assert.strictEqual(s.initReasonCode, SAMPLE.vsta.initReasonCode);
      break;
    }
    case proto.TYPE.FEA3: {
      const feats = proto.decodeFea3Payload(payload);
      assert.strictEqual(feats.length, SAMPLE.fea3.length);
      assert.strictEqual(feats[1].id, 2);
      break;
    }
    case proto.TYPE.PCLD: {
      const res = proto.decodePcldPayload(payload);
      assert.strictEqual(res.points.length, SAMPLE.pcld.points.length);
      assert(almost(res.pointSize, SAMPLE.pcld.pointSize, 1e-4));
      break;
    }
    case proto.TYPE.CFGQ: {
      const res = proto.decodeConfigRequestPayload(payload);
      assert.strictEqual(res.version, SAMPLE.cfgq.version);
      assert.strictEqual(res.op, SAMPLE.cfgq.op);
      assert.strictEqual(res.key, SAMPLE.cfgq.key);
      assert.deepStrictEqual(res.value, SAMPLE.cfgq.value);
      break;
    }
    case proto.TYPE.CFGR: {
      const res = proto.decodeConfigResponsePayload(payload);
      assert.strictEqual(res.version, SAMPLE.cfgr.version);
      assert.strictEqual(res.op, SAMPLE.cfgr.op);
      assert.strictEqual(res.success, SAMPLE.cfgr.success);
      assert.strictEqual(res.hasValue, SAMPLE.cfgr.hasValue);
      assert.strictEqual(res.key, SAMPLE.cfgr.key);
      assert.strictEqual(res.message, SAMPLE.cfgr.message);
      assert.deepStrictEqual(res.value, SAMPLE.cfgr.value);
      break;
    }
    default:
      throw new Error(`Unknown type ${type}`);
  }
}

function randomPort() {
  return 40000 + Math.floor(Math.random() * 20000);
}

async function main() {
  const packets = buildPackets();
  const port = randomPort();
  const bin = path.join(__dirname, 'bin', 'cpp_roundtrip');

  const server = spawn(bin, ['--port', String(port)], { stdio: ['ignore', 'inherit', 'inherit'] });
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

  // Robust connect with retries to avoid races with server startup.
  let sock = null;
  for (let attempt = 0; attempt < 20; attempt++) {
    await sleep(150);
    const s = net.createConnection({ host: '127.0.0.1', port });
    const ok = await new Promise((resolve) => {
      s.once('connect', () => resolve(true));
      s.once('error', () => resolve(false));
    });
    if (ok) {
      sock = s;
      for (const p of packets) sock.write(p);
      break;
    }
    s.destroy();
  }
  if (!sock) {
    server.kill();
    throw new Error(`connect failed: 127.0.0.1:${port}`);
  }

  let buffer = Buffer.alloc(0);
  let recvCount = 0;

  const done = new Promise((resolve, reject) => {
    sock.on('data', (chunk) => {
      buffer = Buffer.concat([buffer, chunk]);
      let parsed = proto.parseFrames(buffer);
      buffer = parsed.rest;
      for (const f of parsed.frames) {
        verifyFrame(f, recvCount);
        recvCount += 1;
        if (recvCount === EXPECTED_COUNT) {
          sock.end();
        }
      }
    });
    sock.on('error', reject);
    sock.on('end', resolve);
    sock.on('close', resolve);
  });

  await done;

  await new Promise((resolve, reject) => {
    server.on('exit', (code) => code === 0 ? resolve() : reject(new Error(`cpp exited ${code}`)));
    server.on('error', reject);
  });

  assert.strictEqual(recvCount, EXPECTED_COUNT);
  console.log('Node/C++ TCP roundtrip test passed');

  // Additional sanity: dispatcher over known packets
  const dispatcherFrames = [];
  const disp = new proto.FrameDispatcher((f) => dispatcherFrames.push(f));
  for (const p of packets) disp.feed(p);
  assert.strictEqual(dispatcherFrames.length, EXPECTED_COUNT);

  await runFuzzTests();
}

async function runFuzzTests() {
	  const fuzzTypes = [
	    proto.TYPE.JPG,
	    proto.TYPE.RJPG,
	    proto.TYPE.RAW,
	    proto.TYPE.SRAW,
	    proto.TYPE.POSE,
	    proto.TYPE.UPOSE,
	    proto.TYPE.LCON,
	    proto.TYPE.IMU,
	    proto.TYPE.STAT,
	    proto.TYPE.VSTA,
      proto.TYPE.CFGQ,
      proto.TYPE.CFGR,
	  ];

  function randomJpgPayload(isRef) {
    const data = crypto.randomBytes(16);
    return proto.buildJpgPayload({ timestampNs: BigInt(Math.floor(Math.random() * 1e6)), channel: 'ch', data, isRef });
  }

  function randomRawPayload() {
    const width = 4;
    const height = 2;
    const data = crypto.randomBytes(width * height);
    return proto.buildRawPayload({
      timestampNs: BigInt(Math.floor(Math.random() * 1e6)),
      width,
      height,
      format: proto.RAW_FORMAT.GRAY8,
      channel: 'raw',
      data,
    });
  }

  function randomPosePayload() {
    return proto.buildPosePayload({
      poseType: Math.random() > 0.5 ? 0 : 1,
      poseFlags: 0x3,
      positionM: [Math.random(), Math.random(), Math.random()],
      orientationXyzw: [0.1, 0.2, 0.3, 0.9],
    });
  }

  function randomConstraintsPayload() {
    return proto.buildConstraintsPayload([
      { type: 0, start: [Math.random(), 0, 0], end: [0, Math.random(), 0] },
    ]);
  }

  function randomImuPayload() {
    return proto.buildImuPayload([
      { timestampNs: BigInt(Math.floor(Math.random() * 1e6)), ax: 0.1, ay: 0.2, az: 0.3, gx: 0.4, gy: 0.5, gz: 0.6 },
    ]);
  }

	  function randomStatPayload() {
	    return proto.buildStatusPayload('fuzz');
	  }

	  function randomVstaPayload() {
	    return proto.buildVioStatePayload({
	      version: 4,
	      state: 2,
	      flags: 0,
	      timestampNs: BigInt(Math.floor(Math.random() * 1e6)),
	      fpsCurrent: Math.random() * 60,
	      fpsAverage: Math.random() * 60,
        imuHzCurrent: Math.random() * 200,
        imuHzAverage5s: Math.random() * 200,
        initReasonCode: Math.floor(Math.random() * 13),
	      poseConfidence: Math.random(),
	      trackingRate: Math.random(),
	      numFeatures: Math.floor(Math.random() * 1000),
	      loopClosures: Math.floor(Math.random() * 100),
	    });
	  }

  function randomCfgqPayload() {
    return proto.buildConfigRequestPayload({
      version: 1,
      op: Math.random() > 0.5 ? proto.CONFIG_OP.GET : proto.CONFIG_OP.SET,
      key: "calib",
      value: crypto.randomBytes(12),
    });
  }

  function randomCfgrPayload() {
    return proto.buildConfigResponsePayload({
      version: 1,
      op: Math.random() > 0.5 ? proto.CONFIG_OP.GET : proto.CONFIG_OP.SET,
      success: Math.random() > 0.5 ? 1 : 0,
      hasValue: Math.random() > 0.5,
      key: "calib",
      message: "fuzz",
      value: crypto.randomBytes(14),
    });
  }

  const builders = {
    [proto.TYPE.JPG]: () => randomJpgPayload(false),
    [proto.TYPE.RJPG]: () => randomJpgPayload(true),
    [proto.TYPE.RAW]: () => randomRawPayload(),
    [proto.TYPE.SRAW]: () => proto.buildStereoRawPayload({
      left: { timestampNs: 11n, width: 2, height: 1, format: proto.RAW_FORMAT.GRAY8, channel: 'cam0', data: Buffer.from([0x01, 0x02]) },
      right: { timestampNs: 12n, width: 2, height: 1, format: proto.RAW_FORMAT.GRAY8, channel: 'cam1', data: Buffer.from([0x03, 0x04]) },
    }),
    [proto.TYPE.POSE]: () => randomPosePayload(),
    [proto.TYPE.UPOSE]: () => randomPosePayload(),
    [proto.TYPE.LCON]: () => randomConstraintsPayload(),
	    [proto.TYPE.IMU]: () => randomImuPayload(),
	    [proto.TYPE.STAT]: () => randomStatPayload(),
	    [proto.TYPE.VSTA]: () => randomVstaPayload(),
      [proto.TYPE.CFGQ]: () => randomCfgqPayload(),
      [proto.TYPE.CFGR]: () => randomCfgrPayload(),
	  };

  for (let iter = 0; iter < 50; iter++) {
    const pkts = [];
    const expected = [];
    for (let i = 0; i < 10; i++) {
      const t = fuzzTypes[Math.floor(Math.random() * fuzzTypes.length)];
      const payload = builders[t]();
      pkts.push(proto.makePacket(t, payload));
      expected.push(t.trim());
    }

    const dispFrames = [];
    const disp = new proto.FrameDispatcher((f) => dispFrames.push(f.type.trim()));
    for (const p of pkts) disp.feed(p);
    assert.deepStrictEqual(dispFrames, expected);

    // Ensure decode path does not throw
    const { frames } = proto.parseFrames(Buffer.concat(pkts));
    for (const f of frames) {
      switch (f.type) {
        case proto.TYPE.JPG: proto.decodeJpgPayload(f.payload, false); break;
        case proto.TYPE.RJPG: proto.decodeJpgPayload(f.payload, true); break;
        case proto.TYPE.RAW: proto.decodeRawPayload(f.payload); break;
        case proto.TYPE.SRAW: proto.decodeStereoRawPayload(f.payload); break;
        case proto.TYPE.POSE:
        case proto.TYPE.UPOSE: proto.decodePosePayload(f.payload); break;
        case proto.TYPE.LCON: proto.decodeConstraintsPayload(f.payload); break;
	        case proto.TYPE.IMU: proto.decodeImuPayload(f.payload); break;
	        case proto.TYPE.STAT: proto.decodeStatusPayload(f.payload); break;
	        case proto.TYPE.VSTA: proto.decodeVioStatePayload(f.payload); break;
          case proto.TYPE.CFGQ: proto.decodeConfigRequestPayload(f.payload); break;
          case proto.TYPE.CFGR: proto.decodeConfigResponsePayload(f.payload); break;
	      }
	    }
	  }
  console.log('Node fuzz/dispatcher decode test passed');
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
