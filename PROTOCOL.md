# Mighty Protocol Specification

This document describes the **wire protocol** used by Mighty camera streams and command/control RPC.

For SDK onboarding and app-level usage examples, see [README.md](./README.md).

## 1. Packet Framing

Each frame on the wire is:

```text
[4B header][4B type][4B payload_len][payload][4B crc32][4B footer]
```

- Header magic: `0xDEADBEEF`
- Footer magic: `0xFEEDFACE`
- Endianness: **big-endian** for numeric fields
- CRC: CRC32 of payload bytes (zero when payload is empty)

Raw-frame CRC compatibility rule:
- For `RAW ` and `SRAW`, decoders may accept `crc32 == 0` as a skip-CRC fast path (for legacy/device compatibility).

## 2. Type Codes

4-byte ASCII type values:

- `JPG `: JPEG image
- `RJPG`: reference JPEG image
- `RAW `: mono raw image
- `SRAW`: stereo raw image (left + right)
- `POSE`: optimized pose
- `UPOS`: unoptimized pose
- `LCON`: loop constraints / line segments
- `VIZ `: visualization payload (subtyped)
- `IMU `: IMU samples batch
- `STAT`: textual status
- `VSTA`: structured VIO status
- `RSET`: reset event
- `FEA3`: 3D features
- `PCLD`: point cloud
- `CMD `: command request
- `CRES`: command response
- `CFGQ`: config request payload
- `CFGR`: config response payload

## 3. Payload Layouts (High-Level)

The canonical builders/decoders are implemented in:
- JS: `js/core/protocol.js`
- C++: `cpp/mighty_protocol.h`
- Python: `python/mighty_protocol.py`

### 3.1 Images

#### `JPG ` / `RJPG`
- timestamp (u64)
- channel name length + channel bytes (`RJPG` may omit/normalize channel)
- JPEG bytes

#### `RAW `
- timestamp (u64)
- width (u32)
- height (u32)
- format (u8)
- channel name
- raw bytes

Formats (SDK constant: `RAW_FORMAT`):
- `UNKNOWN=0`
- `GRAY8=1`
- `RGB24=2`
- `BGR24=3`
- `RGBA32=4`
- `BGRA32=5`
- `YUV420SP=6`
- `YUV420P=7`

#### `SRAW`
- left raw image payload
- right raw image payload

### 3.2 Pose (`POSE` / `UPOS`)

Both share identical payload layout.

Core fields:
- `poseType` (u32)
- `poseFlags` (u32)
- `position` (f64 x3)
- `confidence` (f32)
- optional fields gated by `poseFlags`

`poseType` semantics:
- `0`: body/IMU frame pose (`W<-B`)
- `1`: camera frame pose (`W<-C`)
- other values: reserved / custom streams

Quaternion convention:
- quaternion is source-to-world (`q_WB` for body pose)

`poseFlags` bits:
- bit 0: quaternion present (`quat`, f64 x4)
- bit 1: keyframe marker
- bit 2: linear velocity (`linvel`, f64 x3)
- bit 3: angular velocity (`angvel`, f64 x3)
- bit 4: linear acceleration (`linacc`, f64 x3)
- bit 5: angular acceleration (`angacc`, f64 x3)
- bit 6: `timestamp_ns` present (u64)

Protocol evolution rule:
- append-only; new optional fields must be gated via new flags.

### 3.3 IMU (`IMU `)

Batch payload containing repeated samples:
- `timestamp_ns` (u64)
- accel `(ax, ay, az)` (f32)
- gyro `(gx, gy, gz)` (f32)

### 3.4 VIO State (`VSTA`)

Structured low-rate status telemetry.

Version 1 fields:
- `version` (u8)
- `state` (u8)
- `flags` (u16)
- `timestamp_ns` (u64)
- `fps_current` (f32)
- `fps_average` (f32)
- `pose_confidence01` (f32)
- `tracking_rate` (f32)
- `num_features` (u32)
- `loop_closures` (u32)

Version 2 appends:
- `build_version_len` (u8)
- `build_version` (bytes)

Version 3 appends:
- `imu_hz_current` (f32)
- `imu_hz_average_5s` (f32)

Recommended state mapping:
- `0=OFF, 1=INITIALIZING, 2=TRACKING, 3=DEGRADED, 4=LOST`

### 3.5 Status / Reset

#### `STAT`
- UTF-8 status text

#### `RSET`
- no payload

### 3.6 Viz / Constraints / Point Cloud

#### `VIZ `
- subtype-based payload:
  - `0`: features
  - `1`: detections
  - `2`: matches

#### `LCON`
- constraint segments (type + start/end 3D points)

#### `PCLD`
- points: `(x,y,z,r,g,b)`
- optional `point_size`

### 3.7 Command and Config

#### `CMD `
- `req_id` (u32)
- command name length + name bytes
- data length + data bytes

#### `CRES`
- `req_id` (u32)
- status (u8) (`0` success)
- message length + UTF-8 message
- data length + data bytes

#### `CFGQ`
- `version` (u8)
- `op` (u8): `0=GET`, `1=SET`
- key length + key bytes
- value length + value bytes

#### `CFGR`
- `version` (u8)
- `op` (u8)
- `success` (u8)
- `has_value` (u8)
- key length + key bytes
- message length (u16) + message bytes
- value length + value bytes

Config-over-command convention used by SDK:
- send `CMD(name="config", data=<CFGQ payload>)`
- receive `CRES(data=<CFGR payload>)`

## 4. Low-Level API Surface by Language

### JavaScript (`js/core/protocol.js`)

- framing: `makePacket`, `parseFrames`, `FrameDispatcher`
- builders: `build*Payload` helpers for each type
- decoders: `decode*Payload` helpers for each type
- constants: `TYPE`, `RAW_FORMAT`, `CONFIG_OP`

### Python (`python/mighty_protocol.py`)

- framing: `make_packet`, `parse_frames`, `FrameDispatcher`
- builders/decoders: type-specific functions matching protocol payloads
- constants: `TYPE`, `RAW_FORMAT`, `CONFIG_OP`

### C++ (`cpp/mighty_protocol.h`, `cpp/mighty_protocol_consumer.h`)

- framing: `make_packet`, `parse_frame`, `FrameConsumer`, `FrameDispatcher`
- builders/decoders: `build_*_payload`, `decode_*_payload`
- optional decoded callbacks: `DecodedDispatcher`

## 5. Compatibility Notes

- Preserve big-endian encoding across all languages.
- Keep frame types 4-byte ASCII.
- Extend payloads in append-only form where possible.
- Use explicit versioning (e.g., `VSTA`) when adding structured fields.
- Keep `CMD/CRES` request IDs unique per in-flight command to correlate responses.

## 6. Testing

Cross-language protocol conformance lives in:
- `tests/node_roundtrip.test.js`
- `tests/cpp_roundtrip.cpp`
- `tests/python_consumer_test.py`

Run all protocol + SDK tests from repo root:

```bash
./tests/run_tests.sh
```
