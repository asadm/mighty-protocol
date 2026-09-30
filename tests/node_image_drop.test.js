import assert from 'node:assert/strict';
import proto from '../js/index.js';
const payload=proto.buildImageDropPayload({timestampNs:123n,channel:'preview',droppedCount:19n});
const d=proto.decodeImageDropPayload(payload);
assert.equal(d.dropped,true); assert.equal(d.data.length,0);
assert.equal(d.timestampNs,123n); assert.equal(d.droppedCount,19n); assert.equal(d.channel,'preview');
assert.throws(()=>proto.decodeImageDropPayload(payload.subarray(0,payload.length-1)));
assert.throws(()=>proto.decodeImageDropPayload(new Uint8Array(18)));
assert.equal(await proto.imageToRaw({...d,kind:'jpg'}, {jpegDecoder:()=>{throw new Error('must not decode marker')}}),null);
console.log('PASS image-drop wire fields, empty image data, validation and decode bypass');
class Device {
 async connect(onBytes) { this.onBytes=onBytes; return new Promise(resolve=>{this.finish=resolve;}); }
 async disconnect(){this.finish?.();}
}
const device=new Device();const client=new proto.MightyClient(device,{autoReconnect:false});
const images=[];client.onImage(i=>images.push(i));
const connection=client.connect();
await new Promise(r=>setTimeout(r,0));
device.onBytes(proto.makePacket(proto.TYPE.IDRP,payload));
assert.equal(images.length,1);assert.equal(images[0].droppedCount,19n);assert.equal(images[0].data.length,0);
await client.disconnect();await connection;
console.log('PASS SDK image-drop delivery');
