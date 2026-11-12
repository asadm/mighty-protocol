import assert from 'node:assert/strict';
import fs from 'node:fs';
import {decodeVoxelPayload} from '../js/core/protocol.js';
const bytes=process.argv[2]?fs.readFileSync(process.argv[2]):Uint8Array.from([1,0,0,1,0,0,0,0,0,0,0,2,0,50,0,0,0,0,0,43,201,65,21]);
const map=decodeVoxelPayload(bytes);
assert.equal(map.source,'device');assert.equal(map.reset,true);assert.equal(map.changeCount,1);
assert.equal(map.resolutionM,.05);assert.deepEqual([...map.indices],[0,0,43]);
assert.deepEqual([...map.rgb],[201,65,21]);assert.equal(map.timestampNs,2n);
assert.throws(()=>decodeVoxelPayload(bytes.subarray(0,-1)));
const invalid=Uint8Array.from(bytes);invalid[0]=2;assert.throws(()=>decodeVoxelPayload(invalid));
console.log('Device voxel wire decoding passed');

const full=new Uint8Array(14+32768*9);full.set(bytes.subarray(0,14));full[2]=128;full[3]=0;
assert.equal(decodeVoxelPayload(full).changeCount,32768);
