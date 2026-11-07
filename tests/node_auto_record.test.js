import assert from 'node:assert/strict';
import { test } from 'node:test';
import proto from '../js/index.js';

test('automatic recording SDK encodes actions and validates status', async () => {
  const client = new proto.MightyClient({connect(){}, disconnect(){}}, {autoReconnect:false});
  const calls=[];
  let response={ok:true,data:new TextEncoder().encode(JSON.stringify({enabled:true,active:false,sd_present:false,image_paused:false}))};
  client.command=async (name,data)=>{calls.push([name,new TextDecoder().decode(data)]);return response;};
  assert.equal((await client.setAutoRecordWithVio(true)).value.enabled,true);
  await client.setAutoRecordWithVio(false);
  await client.getAutoRecordWithVio();
  assert.deepEqual(calls,[['auto_record_vio','on'],['auto_record_vio','off'],['auto_record_vio','status']]);
  await assert.rejects(()=>client.setAutoRecordWithVio('false'),TypeError);
  for(const data of ['null','{}','broken']) {
    response={ok:true,data:new TextEncoder().encode(data)};
    assert.equal((await client.getAutoRecordWithVio()).ok,false);
  }
  response={ok:false,message:'unknown command'};
  assert.equal(await client.getAutoRecordWithVio(),response);
});
