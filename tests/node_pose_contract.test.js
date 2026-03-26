import assert from "node:assert";
import proto from "../js/index.js";

function almost(a, b, eps = 1e-9) {
  return Math.abs(Number(a) - Number(b)) <= eps;
}

function mapPositionOdomToViz(p) {
  return [
    0.0 * p[0] + -1.0 * p[1] + 0.0 * p[2],
    0.0 * p[0] + 0.0 * p[1] + 1.0 * p[2],
    -1.0 * p[0] + 0.0 * p[1] + 0.0 * p[2],
  ];
}

function buildManualPosePayload() {
  const poseType = 0;
  const flags = 0x1 | (1 << 2) | (1 << 3) | (1 << 6);
  const positionM = [4.25, -2.5, 0.75];
  const orientationXyzw = [0.11, -0.22, 0.33, -0.44]; // xyzw
  const linearVelocityBodyMps = [1.0, 2.0, 3.0];
  const angularVelocityBodyRps = [-1.0, -2.0, -3.0];
  const ts = 123456789n;

  const buf = new Uint8Array(4 + 4 + 24 + 32 + 4 + 24 + 24 + 8);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let off = 0;
  dv.setUint32(off, poseType, false); off += 4;
  dv.setUint32(off, flags, false); off += 4;
  dv.setFloat64(off, positionM[0], false); off += 8;
  dv.setFloat64(off, positionM[1], false); off += 8;
  dv.setFloat64(off, positionM[2], false); off += 8;
  dv.setFloat64(off, orientationXyzw[0], false); off += 8;
  dv.setFloat64(off, orientationXyzw[1], false); off += 8;
  dv.setFloat64(off, orientationXyzw[2], false); off += 8;
  dv.setFloat64(off, orientationXyzw[3], false); off += 8;
  dv.setFloat32(off, 1.25, false); off += 4; // should clamp to 1.0
  dv.setFloat64(off, linearVelocityBodyMps[0], false); off += 8;
  dv.setFloat64(off, linearVelocityBodyMps[1], false); off += 8;
  dv.setFloat64(off, linearVelocityBodyMps[2], false); off += 8;
  dv.setFloat64(off, angularVelocityBodyRps[0], false); off += 8;
  dv.setFloat64(off, angularVelocityBodyRps[1], false); off += 8;
  dv.setFloat64(off, angularVelocityBodyRps[2], false); off += 8;
  dv.setBigUint64(off, ts, false); off += 8;
  assert.strictEqual(off, buf.length);

  return { payload: buf, orientationXyzw, positionM, linearVelocityBodyMps, angularVelocityBodyRps, ts };
}

(function main() {
  const fixture = buildManualPosePayload();
  const out = proto.decodePosePayload(fixture.payload);

  // Explicit XYZW wire-order regression check.
  assert.ok(Array.isArray(out.orientationXyzw));
  assert.ok(almost(out.orientationXyzw[0], fixture.orientationXyzw[0]));
  assert.ok(almost(out.orientationXyzw[1], fixture.orientationXyzw[1]));
  assert.ok(almost(out.orientationXyzw[2], fixture.orientationXyzw[2]));
  assert.ok(almost(out.orientationXyzw[3], fixture.orientationXyzw[3]));

  assert.ok(Array.isArray(out.positionM));
  assert.ok(almost(out.positionM[0], fixture.positionM[0]));
  assert.ok(almost(out.positionM[1], fixture.positionM[1]));
  assert.ok(almost(out.positionM[2], fixture.positionM[2]));

  assert.ok(Array.isArray(out.linearVelocityBodyMps));
  assert.ok(Array.isArray(out.angularVelocityBodyRps));
  assert.ok(almost(out.linearVelocityBodyMps[0], fixture.linearVelocityBodyMps[0]));
  assert.ok(almost(out.angularVelocityBodyRps[2], fixture.angularVelocityBodyRps[2]));
  assert.strictEqual(out.timestampNs, fixture.ts);
  assert.ok(almost(out.confidence, 1.0));

  // Signed-axis sanity for canonical->viz basis.
  const ex = mapPositionOdomToViz([1, 0, 0]);
  const ey = mapPositionOdomToViz([0, 1, 0]);
  const ez = mapPositionOdomToViz([0, 0, 1]);
  assert.deepStrictEqual(ex.map((v) => Math.round(v)), [0, 0, -1]);
  assert.deepStrictEqual(ey.map((v) => Math.round(v)), [-1, 0, 0]);
  assert.deepStrictEqual(ez.map((v) => Math.round(v)), [0, 1, 0]);

  console.log("Node pose contract test passed");
})();
