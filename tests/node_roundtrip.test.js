const net = require('net');
const path = require('path');
const assert = require('assert');
const { spawn } = require('child_process');

const proto = require('../js');

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
  pose: { poseType: 0, poseFlags: 0x3, position: [1.1, 2.2, 3.3], quat: [0.1, 0.2, 0.3, 0.9] },
  upose: { poseType: 0, poseFlags: 0x1, position: [4.4, 5.5, 6.6], quat: [0.4, 0.5, 0.6, 0.7] },
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
};

const EXPECTED_COUNT = 15;

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
    proto.makePacket(proto.TYPE.FEA3, proto.buildFea3Payload(SAMPLE.fea3)),
    proto.makePacket(proto.TYPE.PCLD, proto.buildPcldPayload(SAMPLE.pcld.points, SAMPLE.pcld.pointSize)),
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
      res.position.forEach((v, i) => assert(almost(v, SAMPLE.pose.position[i])));
      break;
    }
    case proto.TYPE.UPOSE: {
      const res = proto.decodePosePayload(payload);
      assert.strictEqual(res.poseType, SAMPLE.upose.poseType);
      assert(res.poseFlags & 0x1);
      assert(almost(res.position[2], SAMPLE.upose.position[2]));
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
  let sock;
  for (let attempt = 0; attempt < 10; attempt++) {
    await sleep(300);
    try {
      sock = net.createConnection({ port }, () => {
        for (const p of packets) sock.write(p);
      });
      break;
    } catch (e) {
      if (attempt === 9) throw e;
    }
  }
  if (!sock) throw new Error('failed to create socket');

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
  const crypto = require('crypto');
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
      position: [Math.random(), Math.random(), Math.random()],
      quat: [0.1, 0.2, 0.3, 0.9],
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
      }
    }
  }
  console.log('Node fuzz/dispatcher decode test passed');
}

if (require.main === module) {
  main().catch((err) => {
    console.error(err);
    process.exit(1);
  });
}
