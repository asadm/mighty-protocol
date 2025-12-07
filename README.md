# Mighty Protocol

Shared protocol helpers for framing and payload encoding/decoding used by the web UI and server.

- C++ helpers: `mighty-protocol/cpp/mighty_protocol.h` (builders/decoders) and `mighty-protocol/cpp/mighty_protocol_consumer.h` (stream consumer).
- JS helpers: `mighty-protocol/js/index.js` (Node export + `window.MightyProtocol` in browsers).
- Python helpers: `mighty-protocol/python/mighty_protocol.py`.
- Optional decoded dispatchers: C++ `DecodedDispatcher` and Python `decoded_dispatcher.py` (per-type callbacks with decoded payloads).
- Tests: cross-language TCP roundtrip covering all packet types.

## Running tests

From repo root:
```bash
mighty-protocol/tests/run_tests.sh
```
This builds the C++ test binary and runs the Node roundtrip test plus a Python consumer test.

## Quickstart

- C++ send: `auto payload = build_jpg_payload(ts,false,"preview",data,len); auto &pkt = make_packet(payload, TYPE_JPG); send(pkt.data(), pkt.size());`
- C++ recv (decoded): `DecodedDispatcher dd; dd.on_jpg = [](auto ts, auto ch, auto &d, bool is_ref){...}; dd.feed(bytes,len);`
- JS recv (browser/Node): `const disp = new proto.FrameDispatcher(({type,payload}) => { ...switch on type... }); disp.feed(chunk);`
- Python recv (decoded): `from decoded_dispatcher import DecodedDispatcher; dd=DecodedDispatcher(); dd.on_pose=lambda p,is_unopt:...; dd.feed(raw_bytes)`

## API reference & examples

### C++ (producer/consumer)
- Produce
  - `make_packet(payload, TYPE_*)` – wrap framing + CRC.
  - Builders: `build_jpg_payload`, `build_pose_payload`, `build_constraints_payload`, `build_viz_payload`, `build_imu_payload`, `build_status_payload`, `build_fea3_payload`, `build_pcld_payload`.
  - Type codes: `TYPE_JPG`, `TYPE_RJPG`, `TYPE_POSE`, `TYPE_UPOSE`, `TYPE_LCON`, `TYPE_VIZ`, `TYPE_IMU`, `TYPE_STAT`, `TYPE_RSET`, `TYPE_FEA3`, `TYPE_PCLD`.
- Consume
  - `parse_frame` (single-frame parser) or `FrameConsumer` (`feed`, `try_pop`, `drain`) or `FrameDispatcher` (`set_handler`, `feed`).
  - `DecodedDispatcher` for per-type decoded callbacks (e.g., `on_jpg`, `on_pose`, `on_constraints`, `on_features`, `on_pointcloud`, `on_viz`, `on_imu`, `on_status`, `on_reset`).
  - Decoders: `decode_jpg_payload`, `decode_pose_payload`, `decode_constraints_payload`, `decode_viz_payload`, `decode_imu_payload`, `decode_status_payload`, `decode_fea3_payload`, `decode_pcld_payload`.

```cpp
#include "mighty-protocol/cpp/mighty_protocol.h"
#include "mighty-protocol/cpp/mighty_protocol_consumer.h"

using namespace mighty_protocol;

// Build a JPG packet
auto payload = build_jpg_payload(/*timestamp_ns*/123, /*is_ref*/false, "preview",
                                 jpeg_ptr, jpeg_len);
auto &pkt = make_packet(payload, TYPE_JPG);
// send pkt over any transport

// Consume a stream of bytes
FrameConsumer consumer;
consumer.feed(bytes, len);
auto frames = consumer.drain();
for (auto &f : frames) {
  if (std::string(f.type.data(), 4) == "POSE") {
    uint32_t type, flags; double x, y, z; std::optional<std::array<double,4>> q;
    decode_pose_payload(f.payload, type, flags, x, y, z, q);
  }
}
```

#### C++ DecodedDispatcher (per-type callbacks)
```cpp
#include "mighty-protocol/cpp/mighty_protocol_consumer.h"
using namespace mighty_protocol;

DecodedDispatcher dd;
dd.on_jpg = [](uint64_t ts, const std::string& ch, const std::vector<uint8_t>& data, bool is_ref){ /* use JPEG */ };
dd.on_pose = [](const DecodedDispatcher::Pose& p, bool unopt){ /* p.x/y/z, p.quat */ };
dd.on_constraints = [](const DecodedDispatcher::Constraints& c){ /* c.segments */ };
dd.on_pointcloud = [](const DecodedDispatcher::PointCloud& pc){ /* pc.points, pc.point_size */ };
// ... set other handlers as needed
dd.feed(bytes, len);  // can be called repeatedly with stream chunks
```

### JavaScript / Node / browser
- Produce
  - `makePacket(type, payload)`
  - Builders: `buildJpgPayload`, `buildPosePayload`, `buildConstraintsPayload`, `buildVizPayload`, `buildImuPayload`, `buildStatusPayload`, `buildFea3Payload`, `buildPcldPayload`
  - Types: `TYPE` map (`TYPE.JPG`, `TYPE.RJPG`, `TYPE.POSE`, `TYPE.UPOSE`, `TYPE.LCON`, `TYPE.VIZ`, `TYPE.IMU`, `TYPE.STAT`, `TYPE.RSET`, `TYPE.FEA3`, `TYPE.PCLD`)
- Consume
  - `parseFrames(buffer) -> {frames, rest}` or `new FrameDispatcher(onFrame).feed(bytes)` (browser-friendly version is in `main/web/mighty-protocol.js`)
  - Decoders: `decodeJpgPayload`, `decodePosePayload`, `decodeConstraintsPayload`, `decodeVizPayload`, `decodeImuPayload`, `decodeStatusPayload`, `decodeFea3Payload`, `decodePcldPayload`

```js
import proto from './mighty-protocol/js/index.js'; // Node (or use global MightyProtocol in browser)

const jpgPayload = proto.buildJpgPayload({ timestampNs: 123n, channel: 'preview', data: jpegBuf });
const packet = proto.makePacket(proto.TYPE.JPG, jpgPayload);

let buffer = Buffer.concat([packet, anotherPacket]);
const { frames, rest } = proto.parseFrames(buffer);
for (const f of frames) {
  if (f.type === proto.TYPE.POSE) {
    const pose = proto.decodePosePayload(f.payload);
    console.log(pose.position);
  }
}

// Dispatcher usage
const disp = new proto.FrameDispatcher((frame) => {
  if (frame.type === proto.TYPE.IMU) {
    const samples = proto.decodeImuPayload(frame.payload);
    console.log(samples.length);
  }
});
disp.feed(chunkFromSocket);
```

### Python (consumer + packet builder)
- Produce
  - `make_packet(tcode: bytes, payload: bytes=b"")`
  - Type dict: `TYPE["JPG"]`, `TYPE["RJPG"]`, `TYPE["POSE"]`, `TYPE["UPOSE"]`, `TYPE["LCON"]`, `TYPE["VIZ"]`, `TYPE["IMU"]`, `TYPE["STAT"]`, `TYPE["RSET"]`, `TYPE["FEA3"]`, `TYPE["PCLD"]`
- Consume
  - `parse_frames(buf) -> (frames, rest)` or `FrameDispatcher(on_frame).feed(bytes)`
  - `DecodedDispatcher` in `python/decoded_dispatcher.py` for per-type decoded callbacks (`on_jpg`, `on_pose`, etc.).
  - Decoders: `decode_jpg_payload(payload, is_ref)`, `decode_pose_payload`, `decode_constraints_payload`, `decode_viz_payload`, `decode_imu_payload`, `decode_status_payload`, `decode_fea3_payload`, `decode_pcld_payload`

```python
# PYTHONPATH needs to include mighty-protocol/python
import mighty_protocol as mp

jpeg_bytes = b"\xff\xd8..."
jpg_payload = (123).to_bytes(8, "big") + b"\x07preview" + jpeg_bytes
packet = mp.make_packet(mp.TYPE["JPG"], jpg_payload)

frames, rest = mp.parse_frames(packet)
for f in frames:
    if f["type"] == "POSE":
        pose = mp.decode_pose_payload(f["payload"])
        print(pose["position"])

# Decoded dispatcher usage
from decoded_dispatcher import DecodedDispatcher
dd = DecodedDispatcher()
dd.on_jpg = lambda ts,ch,data,is_ref: print("jpg", ts, ch, len(data))
dd.on_pose = lambda pose,is_unopt: print("pose", pose["pose_type"], is_unopt)
dd.feed(raw_stream_bytes)
```
