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

## Config Requests (CFGQ/CFGR)

`CFGQ` and `CFGR` carry typed config get/set payloads.

- `CFGQ` (`ConfigRequest`) fields:
  - `version` (u8)
  - `op` (u8): `0=get`, `1=set`
  - `key_len` (u8)
  - `key` (bytes)
  - `value_len` (u32)
  - `value` (bytes)
- `CFGR` (`ConfigResponse`) fields:
  - `version` (u8)
  - `op` (u8): mirrors request
  - `success` (u8): `0/1`
  - `has_value` (u8): `0/1` (useful for GET missing key)
  - `key_len` (u8)
  - `key` (bytes)
  - `msg_len` (u16)
  - `message` (bytes)
  - `value_len` (u32)
  - `value` (bytes)

Notes:
- In current app integration, config exchange is tunneled via `CMD`/`CRES` (`name="config"`) with `CFGQ`/`CFGR` payloads in `data`.
- `DecodedDispatcher` supports direct `CFGQ`/`CFGR` frame handling as well.

## Pose Conventions (POSE/UPOSE)

`POSE` and `UPOSE` share the same payload layout and are used for different pose streams.

- `poseType` identifies the semantic frame of the pose:
  - `0` = body/IMU state (`W<-B`), intended as the primary robotics/drone state
  - `1` = camera pose (`W<-C`), derived from body pose + extrinsics (mainly for visualization)
  - other values are allowed (e.g. ground truth streams)
- Quaternion convention: `q` is **source->world** (e.g. `q_WB` maps vectors from `B` into `W`).
- The protocol is **append-only**: new optional fields are appended after `confidence` and are gated by `poseFlags` bits so older decoders can safely ignore them.

### POSE `poseFlags` bit table

- Bit 0: has quaternion `quat` (float64[4] = `qx,qy,qz,qw`)
- Bit 1: keyframe marker (no extra bytes)
- Bit 2: `linvel` present (float64[3]): **`v_WB`** linear velocity in world frame
- Bit 3: `angvel` present (float64[3]): **`omega_B`** angular velocity in body frame
- Bit 4: `linacc` present (float64[3]): **`spec_force_B`** specific force (accelerometer) in body frame, gravity not included
- Bit 5: `angacc` present (float64[3]): **`alpha_B`** angular acceleration in body frame (typically finite-differenced; noisy)
- Bit 6: `timestamp_ns` present (uint64): pose timestamp in nanoseconds (appended at the end)

## VIO State (VSTA)

`VSTA` is a low-rate structured telemetry packet intended to replace ad-hoc parsing of `STAT` strings.

Payload (big-endian):

Version 1:
- `version` (u8) = 1
- `state` (u8): producer-defined enum (recommended: 0=OFF, 1=INITIALIZING, 2=TRACKING, 3=DEGRADED, 4=LOST)
- `flags` (u16): producer-defined bitfield (e.g. initialized/have_pose/have_kinematics)
- `timestamp_ns` (u64): timestamp associated with the state sample (often the latest pose/image timestamp)
- `fps_current` (f32)
- `fps_average` (f32)
- `pose_confidence01` (f32)
- `tracking_rate` (f32)
- `num_features` (u32)
- `loop_closures` (u32)

Version 2: appends
- `build_version_len` (u8)
- `build_version` (u8[build_version_len]) ASCII build/version string (e.g. `Mighty v.YYYYMMDD-<hash>`)

Version 3: appends (after optional `build_version`)
- `imu_hz_current` (f32)
- `imu_hz_average_5s` (f32)

### C++ (producer/consumer)
- Produce
  - `make_packet(payload, TYPE_*)` – wrap framing + CRC.
  - Builders: `build_jpg_payload`, `build_raw_payload`, `build_stereo_raw_payload`, `build_pose_payload`, `build_constraints_payload`, `build_viz_payload`, `build_imu_payload`, `build_status_payload`, `build_fea3_payload`, `build_pcld_payload`, `build_command_payload`, `build_command_response_payload`, `build_config_request_payload`, `build_config_response_payload`.
  - Type codes: `TYPE_JPG`, `TYPE_RJPG`, `TYPE_RAW`, `TYPE_SRAW`, `TYPE_POSE`, `TYPE_UPOSE`, `TYPE_LCON`, `TYPE_VIZ`, `TYPE_IMU`, `TYPE_STAT`, `TYPE_VSTA`, `TYPE_RSET`, `TYPE_FEA3`, `TYPE_PCLD`, `TYPE_CMD`, `TYPE_CRES`, `TYPE_CFGQ`, `TYPE_CFGR`.
- Consume
  - `parse_frame` (single-frame parser) or `FrameConsumer` (`feed`, `try_pop`, `drain`) or `FrameDispatcher` (`set_handler`, `feed`).
  - `DecodedDispatcher` for per-type decoded callbacks (e.g., `on_jpg`, `on_raw`, `on_stereo_raw`, `on_pose`, `on_constraints`, `on_features`, `on_pointcloud`, `on_viz`, `on_imu`, `on_status`, `on_vio_state`, `on_reset`, `on_command`, `on_command_response`, `on_config_request`, `on_config_response`).
  - Decoders: `decode_jpg_payload`, `decode_raw_payload`, `decode_stereo_raw_payload`, `decode_pose_payload`, `decode_constraints_payload`, `decode_viz_payload`, `decode_imu_payload`, `decode_status_payload`, `decode_vio_state_payload`, `decode_fea3_payload`, `decode_pcld_payload`, `decode_command_payload`, `decode_command_response_payload`, `decode_config_request_payload`, `decode_config_response_payload`.

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
  - Builders: `buildJpgPayload`, `buildRawPayload`, `buildStereoRawPayload`, `buildPosePayload`, `buildConstraintsPayload`, `buildVizPayload`, `buildImuPayload`, `buildStatusPayload`, `buildVioStatePayload`, `buildFea3Payload`, `buildPcldPayload`, `buildCommandPayload`, `buildCommandResponsePayload`, `buildConfigRequestPayload`, `buildConfigResponsePayload`
  - Config op constants: `CONFIG_OP.GET`, `CONFIG_OP.SET`
  - Types: `TYPE` map (`TYPE.JPG`, `TYPE.RJPG`, `TYPE.RAW`, `TYPE.SRAW`, `TYPE.POSE`, `TYPE.UPOSE`, `TYPE.LCON`, `TYPE.VIZ`, `TYPE.IMU`, `TYPE.STAT`, `TYPE.VSTA`, `TYPE.RSET`, `TYPE.FEA3`, `TYPE.PCLD`, `TYPE.CMD`, `TYPE.CRES`, `TYPE.CFGQ`, `TYPE.CFGR`)
- Consume
  - `parseFrames(buffer) -> {frames, rest}` or `new FrameDispatcher(onFrame).feed(bytes)` (browser-friendly version is in `main/web/mighty-protocol.js`)
  - Decoders: `decodeJpgPayload`, `decodeRawPayload`, `decodeStereoRawPayload`, `decodePosePayload`, `decodeConstraintsPayload`, `decodeVizPayload`, `decodeImuPayload`, `decodeStatusPayload`, `decodeVioStatePayload`, `decodeFea3Payload`, `decodePcldPayload`, `decodeCommandPayload`, `decodeCommandResponsePayload`, `decodeConfigRequestPayload`, `decodeConfigResponsePayload`

#### JS config-over-command example
```js
const req = MightyProtocol.buildConfigRequestPayload({
  version: 1,
  op: MightyProtocol.CONFIG_OP.GET,
  key: "calib",
  value: new Uint8Array(),
});

const cmd = MightyProtocol.buildCommandPayload({
  reqId: 1,
  name: "config",
  data: req,
});

// send `cmd` to /command endpoint
// decode CRES body with decodeCommandResponsePayload, then:
// const cfg = MightyProtocol.decodeConfigResponsePayload(commandRes.data);
```

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
  - Type dict: `TYPE["JPG"]`, `TYPE["RJPG"]`, `TYPE["RAW"]`, `TYPE["SRAW"]`, `TYPE["POSE"]`, `TYPE["UPOSE"]`, `TYPE["LCON"]`, `TYPE["VIZ"]`, `TYPE["IMU"]`, `TYPE["STAT"]`, `TYPE["RSET"]`, `TYPE["FEA3"]`, `TYPE["PCLD"]`
- Consume
  - `parse_frames(buf) -> (frames, rest)` or `FrameDispatcher(on_frame).feed(bytes)`
  - `DecodedDispatcher` in `python/decoded_dispatcher.py` for per-type decoded callbacks (`on_jpg`, `on_pose`, etc.).
  - Decoders: `decode_jpg_payload(payload, is_ref)`, `decode_raw_payload`, `decode_stereo_raw_payload`, `decode_pose_payload`, `decode_constraints_payload`, `decode_viz_payload`, `decode_imu_payload`, `decode_status_payload`, `decode_fea3_payload`, `decode_pcld_payload`

Python note:
- `CFGQ`/`CFGR` helper builders/decoders are currently implemented in C++ and JS helpers.

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
