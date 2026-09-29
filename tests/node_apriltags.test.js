import assert from "node:assert/strict";
import fs from "node:fs";
import proto, { parseWpilibFieldLayout, serializeApriltagMapYaml, wpilibTagQuaternionToMighty, mightyTagQuaternionToWpilib } from "../js/index.js";
const source = fs.readFileSync(new URL("./fixtures/wpilib-2026-rebuilt-welded.json", import.meta.url), "utf8");
const map = parseWpilibFieldLayout(source);
assert.equal(map.tags.length, 32);
assert.deepEqual(map.field, { length: 16.541, width: 8.069 });
assert.deepEqual(map.tags[0].positionM, [11.8779798, 7.4247756, 0.889]);
assert.equal(map.tags[0].tagId, 1);
assert.equal(map.tags[0].sizeM, 0.1651);
assert.equal(parseWpilibFieldLayout(source, { tagSizeM: 0.2 }).tags[0].sizeM, 0.2);
assert.equal(proto.parseWpilibFieldLayout, parseWpilibFieldLayout);
const yaml = serializeApriltagMapYaml(map);
assert.equal((yaml.match(/orientation: quaternion/g) || []).length, 32);
assert.match(yaml, /position_m: \[11.8779798, 7.4247756, 0.889\]/);
assert.deepEqual(wpilibTagQuaternionToMighty([0, 0, 0, 1]), [-0.5, 0.5, -0.5, 0.5]);
for (const q of [[0, 0, 0, 1], [1, 0, 0, 1], [0.2, -0.3, 0.4, 0.5], [0, 0, 1e-300, 1e-300], [0, 0, 1e300, 1e300]]) {
  const actual = mightyTagQuaternionToWpilib(wpilibTagQuaternionToMighty(q));
  const scale = Math.max(...q.map(Math.abs));
  const norm = Math.hypot(...q.map(v => v / scale));
  actual.forEach((v, i) => assert.ok(Math.abs(v - q[i] / scale / norm) < 1e-14));
}
const measured = JSON.parse(source);
measured.mighty = { tagFamily: "tag36h11", tagSizeM: 0.12 };
assert.equal(parseWpilibFieldLayout(measured).tags[0].sizeM, 0.12);
assert.equal(parseWpilibFieldLayout(measured, {tagSizeM: 0.1651}).tags[0].sizeM, 0.1651);
for (const metadata of [null, [], {tagSizeM: 0}, {tagSizeM: "0.12"}, {tagSizeM: null}, {tagFamily: "tag16h5"}]) {
  assert.throws(() => parseWpilibFieldLayout({...measured, mighty: metadata}));
}
const bad = [];
for (const mutate of [
  m => m.tags.push(m.tags[0]), m => m.tags[0].ID = -1,
  m => m.tags[0].ID = 0.5, m => m.tags[0].ID = true,
  m => m.tags[0].pose.rotation.quaternion = { X: 0, Y: 0, Z: 0, W: 0 },
  m => delete m.tags[0].pose.rotation.quaternion.W,
  m => m.tags[0].pose.translation.x = Infinity,
  m => m.tags[0].pose.translation.x = "1", m => m.field.width = -1,
]) { const m = JSON.parse(source); mutate(m); bad.push(m); }
bad.push({}, { tags: [] }, source + "garbage", "{broken");
for (const input of bad) assert.throws(() => parseWpilibFieldLayout(input));
assert.throws(() => parseWpilibFieldLayout(source, { tagSizeM: 0 }));
assert.throws(() => serializeApriltagMapYaml({ tags: [] }));
let writes = 0;
const device = {
  connect() {}, disconnect() {},
  async sendCommandPayload(payload) {
    const cmd = proto.decodeCommandPayload(payload);
    const cfg = proto.decodeConfigRequestPayload(cmd.data);
    assert.equal(cmd.name, "config"); assert.equal(cfg.key, "apriltags");
    assert.equal(cfg.op, proto.CONFIG_OP.SET);
    assert.equal(new TextDecoder().decode(cfg.value), yaml); writes++;
    return proto.buildCommandResponsePayload({ reqId: cmd.reqId, status: 0, message: "saved",
      data: proto.buildConfigResponsePayload({ version: 1, op: cfg.op, success: 1, hasValue: true,
        key: cfg.key, value: cfg.value, message: "saved" }) });
  },
};
const client = new proto.MightyClient(device, { commandTimeoutMs: 0 });
assert.equal((await client.importWpilibFieldLayout(source)).ok, true);
for (const input of bad) assert.equal((await client.importWpilibFieldLayout(input)).ok, false);
const measuredYaml = serializeApriltagMapYaml(parseWpilibFieldLayout(measured));
const measuredClient = new proto.MightyClient({...device, async sendCommandPayload(payload) {
  const cmd = proto.decodeCommandPayload(payload);
  const cfg = proto.decodeConfigRequestPayload(cmd.data);
  assert.equal(new TextDecoder().decode(cfg.value), measuredYaml);
  return proto.buildCommandResponsePayload({reqId:cmd.reqId, status:0, data:proto.buildConfigResponsePayload({version:1,op:cfg.op,success:1,key:cfg.key,hasValue:true,value:cfg.value})});
}}, {commandTimeoutMs:0});
assert.equal((await measuredClient.importWpilibFieldLayout(measured)).ok, true);
assert.equal(writes, 1, "invalid layouts must not write a partial map");
if (process.argv[2]) fs.writeFileSync(process.argv[2], yaml);
console.log("node AprilTag importer: official 32-tag layout, full poses, validation, and upload passed");
