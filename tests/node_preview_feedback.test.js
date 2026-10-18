import { test } from 'node:test';
import assert from 'node:assert/strict';
import proto from '../js/index.js';

test('one stream negotiates preview feedback and old firmware falls back without extra streams', async () => {
  for (const legacy of [false, true]) {
    const requests=[];
    const device=new proto.MightyWebDevice({baseUrl:'http://device',previewFeedback:true,
      fetchImpl:async (url,options)=>{
        requests.push(url);
        if(legacy&&url.endsWith('?preview=ack'))return new Response('',{status:404});
        return new Response(new ReadableStream({start(controller){
          options.signal.addEventListener('abort',()=>controller.close(),{once:true});
        }}),{headers:legacy?{}:{'X-Mighty-Stream-Id':'42'}});
      }});
    const task=device.connect(()=>{});
    await new Promise(r=>setTimeout(r,0));
    assert.deepEqual(requests,legacy?['http://device/stream?preview=ack','http://device/stream']:['http://device/stream?preview=ack']);
    const payload=device.previewAcknowledgementPayload(123n,'cam0');
    if(legacy)assert.equal(payload,null);
    else {assert.equal(new DataView(payload.buffer).getBigUint64(0,false),42n);assert.equal(new DataView(payload.buffer).getBigUint64(8,false),123n);}
    await device.disconnect();await task;
  }
});

test('mono JPEG waits for decoder credit; stereo waits for both images; old stream callbacks stay bound', async () => {
  const commands=[];
  const device=new proto.MightyWebDevice({baseUrl:'http://device',fetchImpl:async (_url,options)=>{
    const command=proto.decodeCommandPayload(options.body);commands.push(command);
    return new Response(proto.buildCommandResponsePayload({reqId:command.reqId,status:0,message:'ok'}));
  }});
  device._previewStreamId=42n;
  const client=new proto.MightyClient(device,{autoReconnect:false});
  const images=[],errors=[];client.onImage(i=>images.push(i));client.onError(e=>errors.push(e));
  client._handleBytes(proto.makePacket(proto.TYPE.JPG,proto.buildJpgPayload({timestampNs:123n,channel:'cam0',data:new Uint8Array([1,2])})));
  assert.equal(commands.length,0);
  device._previewStreamId=43n; // An old decoded frame must not grant new-stream credit.
  images[0].acknowledgePreview();images[0].acknowledgePreview();
  assert.equal(commands.length,1);assert.equal(commands[0].name,'preview_ready');
  assert.equal(new DataView(commands[0].data.buffer,commands[0].data.byteOffset).getBigUint64(0,false),42n);
  client._handleBytes(proto.makePacket(proto.TYPE.SJPG,proto.buildStereoJpgPayload({
    left:{timestampNs:200n,channel:'cam0',data:new Uint8Array([1])},
    right:{timestampNs:201n,channel:'cam1',data:new Uint8Array([2])},
  })));
  images[1].left.acknowledgePreview();images[1].left.acknowledgePreview();assert.equal(commands.length,1);
  images[1].right.acknowledgePreview();assert.equal(commands.length,2);
  assert.equal(new TextDecoder().decode(commands[1].data.subarray(17)),'cam0+cam1');
  await new Promise(r=>setTimeout(r,0));assert.deepEqual(errors,[]);
});
