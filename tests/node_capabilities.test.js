import test from 'node:test';
import assert from 'node:assert/strict';
import { decodeVioStatePayload } from '../js/core/protocol.js';
// v8 with empty build-version is 92 bytes: base36 + len1 + IMU8 + reasons3
// + memory24 + light8 + translation12.
for (const version of [8,9]) test(`VSTA v${version} capabilities`, () => {
 const p = new Uint8Array(version === 9 ? 94 : 92); p[0] = version;
 if(version===9){p[92]=1;p[93]=1;}
 const s=decodeVioStatePayload(p);assert.equal(s.hasTof,version===9);assert.equal(s.hasWifi,version===9);
 if(version===9)assert.throws(()=>decodeVioStatePayload(p.subarray(0,93)),/truncated/);
});

import proto from '../js/index.js';
test('SDK preserves capability flags', async () => {
 const device = {
  async connect(onBytes) { this.onBytes=onBytes; return new Promise(resolve=>{this.done=resolve;}); },
  async disconnect() { this.done?.(); },
 };
 const client=new proto.MightyClient(device,{autoReconnect:false});
 let result;client.onVioState(s=>{result=s;});await client.connect();
 const p=new Uint8Array(94);p[0]=9;p[92]=1;p[93]=1;
 device.onBytes(proto.makePacket(proto.TYPE.VSTA,p));
 await new Promise(resolve=>setTimeout(resolve,0));
 assert.equal(result.hasTof,true);assert.equal(result.hasWifi,true);
 await client.disconnect();
});
