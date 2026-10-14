import assert from 'node:assert/strict';
import proto from '../js/index.js';

const pair = {
  left: { timestampNs: 123n, channel: 'cam0', data: Uint8Array.of(255, 216, 1, 255, 217) },
  right: { timestampNs: 456n, channel: 'cam1', data: Uint8Array.of(255, 216, 2, 3, 255, 217) },
};
const payload = proto.buildStereoJpgPayload(pair);
const decoded = proto.decodeStereoJpgPayload(payload);
assert.equal(decoded.left.timestampNs, 123n);
assert.equal(decoded.right.timestampNs, 456n);
assert.equal(decoded.right.channel, 'cam1');
assert.deepEqual([...decoded.left.data], [...pair.left.data]);
assert.deepEqual([...decoded.right.data], [...pair.right.data]);
assert.throws(() => proto.decodeStereoJpgPayload(payload.subarray(0, payload.length - 1)));
assert.throws(() => proto.decodeStereoJpgPayload(Buffer.concat([payload, Buffer.of(0)])));
const malformed = Buffer.from(payload); malformed.writeUInt32BE(0xffffffff, 0);
assert.throws(() => proto.decodeStereoJpgPayload(malformed));
const client = new proto.MightyClient({ connect() {}, disconnect() {} }, { autoReconnect: false });
const images = []; client.onImage(image => images.push(image));
const packet = proto.makePacket(proto.TYPE.SJPG, payload);
client._dispatcher.feed(packet.subarray(0, packet.length - 1));
assert.equal(images.length, 0);
client._dispatcher.feed(packet.subarray(packet.length - 1));
assert.equal(images.length, 1);
assert.equal(images[0].kind, 'stereo_jpg');
assert.equal(images[0].left.kind, 'jpg');
assert.equal(images[0].right.timestampNs, 456n);
const raw = await proto.imageToRaw(images[0], { jpegDecoder: frame => ({ kind: 'raw', timestampNs: frame.timestampNs }) });
assert.equal(raw.timestampNs, 123n);
console.log(Buffer.from(payload).toString('hex'));
