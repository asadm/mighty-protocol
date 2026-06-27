import struct
import math
from typing import List, Tuple, Dict, Any, Optional

HEADER_MAGIC = bytes([0xDE, 0xAD, 0xBE, 0xEF])
FOOTER_MAGIC = bytes([0xFE, 0xED, 0xFA, 0xCE])

TYPE = {
    "JPG": b"JPG ",
    "RJPG": b"RJPG",
    "RAW": b"RAW ",
    "SRAW": b"SRAW",
    "POSE": b"POSE",
    "UPOSE": b"UPOS",
    "LCON": b"LCON",
    "VIZ": b"VIZ ",
    "IMU": b"IMU ",
    "STAT": b"STAT",
    "VSTA": b"VSTA",
    "RSET": b"RSET",
    "FEA3": b"FEA3",
    "PCLD": b"PCLD",
    "KEYF": b"KEYF",
    "CMD": b"CMD ",
    "CRES": b"CRES",
    "CFGQ": b"CFGQ",
    "CFGR": b"CFGR",
}

CONFIG_OP = {
    "GET": 0,
    "SET": 1,
}

VIO_STATE = {
    "OFF": 0,
    "INITIALIZING": 1,
    "TRACKING": 2,
    "DEGRADED": 3,
    "LOST": 4,
    "LOW_LIGHT": 5,
}

VIO_INIT_REASON = {
    "NONE": 0,
    "WAITING_FOR_FIRST_IMU": 1,
    "WAITING_FOR_INIT_FRAMES": 2,
    "WAITING_FOR_PARALLAX": 3,
    "WAITING_FOR_IMU_EXCITATION": 4,
    "STATIC_INSUFFICIENT_FEATURES": 5,
    "STATIC_SCENE_MOTION_TOO_HIGH": 6,
    "RELATIVE_POSE_UNAVAILABLE": 7,
    "GLOBAL_SFM_FAILED": 8,
    "PNP_INSUFFICIENT_POINTS": 9,
    "PNP_RANSAC_FAILED": 10,
    "VISUAL_IMU_ALIGNMENT_FAILED": 11,
    "UNKNOWN": 12,
    "WAITING_FOR_FIRST_IMU_NO_SAMPLES": 13,
    "WAITING_FOR_FIRST_IMU_NOT_YET_ALIGNED": 14,
    "WAITING_FOR_FIRST_IMU_TIME_OFFSET_INVALID": 15,
}

RAW_FORMAT = {
    "UNKNOWN": 0,
    "GRAY8": 1,
    "RGB24": 2,
    "BGR24": 3,
    "RGBA32": 4,
    "BGRA32": 5,
    "YUV420SP": 6,
    "YUV420P": 7,
}

def _crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (0xEDB88320 ^ (crc >> 1)) if (crc & 1) else (crc >> 1)
    return crc ^ 0xFFFFFFFF

def make_packet(tcode: bytes, payload: bytes = b"") -> bytes:
    if len(tcode) != 4:
        raise ValueError("type code must be 4 bytes")
    length = len(payload)
    crc = _crc32(payload) if length else 0
    return b"".join([
        HEADER_MAGIC,
        tcode,
        struct.pack(">I", length),
        payload,
        struct.pack(">I", crc),
        FOOTER_MAGIC
    ])

def parse_frames(buf: bytes) -> Tuple[List[Dict[str, Any]], bytes]:
    frames = []
    offset = 0
    min_size = 4 + 4 + 4 + 4 + 4
    while len(buf) - offset >= min_size:
        if buf[offset:offset+4] != HEADER_MAGIC:
            offset += 1
            continue
        tcode = buf[offset+4:offset+8]
        length = struct.unpack(">I", buf[offset+8:offset+12])[0]
        pkt_size = min_size + length
        if len(buf) - offset < pkt_size:
            break
        payload = buf[offset+12:offset+12+length]
        recv_crc = struct.unpack(">I", buf[offset+12+length:offset+16+length])[0]
        footer = buf[offset+16+length:offset+pkt_size]
        offset += pkt_size
        if footer != FOOTER_MAGIC:
            continue
        skip_crc = tcode in (b"RAW ", b"SRAW") and recv_crc == 0
        if length and not skip_crc and _crc32(payload) != recv_crc:
            continue
        frames.append({"type": tcode.decode("ascii"), "payload": payload})
    return frames, buf[offset:]

def build_raw_payload(timestamp_ns: int,
                      width: int,
                      height: int,
                      fmt: int,
                      channel: str,
                      data: bytes) -> bytes:
    chan_bytes = (channel or "").encode("utf-8")
    chan_len = min(255, len(chan_bytes))
    data = data or b""
    header = struct.pack(">QII", int(timestamp_ns), int(width), int(height))
    header += bytes([fmt & 0xFF, chan_len])
    return header + chan_bytes[:chan_len] + data

def build_stereo_raw_payload(left_timestamp_ns: int,
                             right_timestamp_ns: int,
                             left_width: int,
                             left_height: int,
                             left_fmt: int,
                             left_channel: str,
                             left_data: bytes,
                             right_width: int,
                             right_height: int,
                             right_fmt: int,
                             right_channel: str,
                             right_data: bytes) -> bytes:
    left_chan = (left_channel or "").encode("utf-8")
    right_chan = (right_channel or "").encode("utf-8")
    left_chan_len = min(255, len(left_chan))
    right_chan_len = min(255, len(right_chan))
    left_data = left_data or b""
    right_data = right_data or b""
    header = struct.pack(">QQII", int(left_timestamp_ns), int(right_timestamp_ns), int(left_width), int(left_height))
    header += bytes([left_fmt & 0xFF, left_chan_len]) + left_chan[:left_chan_len]
    header += struct.pack(">II", int(right_width), int(right_height))
    header += bytes([right_fmt & 0xFF, right_chan_len]) + right_chan[:right_chan_len]
    header += struct.pack(">II", len(left_data), len(right_data))
    return header + left_data + right_data

# Payload decoders
def decode_jpg_payload(payload: bytes, is_ref: bool):
    if len(payload) < 8:
        raise ValueError("payload too short")
    ts = struct.unpack(">Q", payload[0:8])[0]
    if is_ref:
        return {"timestamp_ns": ts, "channel": "", "data": payload[8:]}
    clen = payload[8]
    channel = payload[9:9+clen].decode("utf-8")
    data = payload[9+clen:]
    return {"timestamp_ns": ts, "channel": channel, "data": data}

def decode_raw_payload(payload: bytes):
    if len(payload) < 8 + 4 + 4 + 1 + 1:
        raise ValueError("payload too short")
    ts = struct.unpack(">Q", payload[0:8])[0]
    width = struct.unpack(">I", payload[8:12])[0]
    height = struct.unpack(">I", payload[12:16])[0]
    fmt = payload[16]
    clen = payload[17]
    if len(payload) < 18 + clen:
        raise ValueError("payload too short")
    channel = payload[18:18+clen].decode("utf-8")
    data = payload[18+clen:]
    return {"timestamp_ns": ts, "width": width, "height": height, "format": fmt, "channel": channel, "data": data}

def decode_stereo_raw_payload(payload: bytes):
    if len(payload) < 8 + 8 + 4 + 4 + 1 + 1 + 4 + 4 + 1 + 1 + 4 + 4:
        raise ValueError("payload too short")
    off = 0
    left_ts = struct.unpack(">Q", payload[off:off+8])[0]; off += 8
    right_ts = struct.unpack(">Q", payload[off:off+8])[0]; off += 8
    left_width = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    left_height = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    left_fmt = payload[off]; off += 1
    left_clen = payload[off]; off += 1
    if len(payload) < off + left_clen:
        raise ValueError("payload too short")
    left_channel = payload[off:off+left_clen].decode("utf-8"); off += left_clen
    if len(payload) < off + 4 + 4 + 1 + 1:
        raise ValueError("payload too short")
    right_width = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    right_height = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    right_fmt = payload[off]; off += 1
    right_clen = payload[off]; off += 1
    if len(payload) < off + right_clen:
        raise ValueError("payload too short")
    right_channel = payload[off:off+right_clen].decode("utf-8"); off += right_clen
    if len(payload) < off + 8:
        raise ValueError("payload too short")
    left_len, right_len = struct.unpack(">II", payload[off:off+8]); off += 8
    if len(payload) < off + left_len + right_len:
        raise ValueError("payload too short")
    left_data = payload[off:off+left_len]; off += left_len
    right_data = payload[off:off+right_len]
    return {
        "left": {
            "timestamp_ns": left_ts,
            "width": left_width,
            "height": left_height,
            "format": left_fmt,
            "channel": left_channel,
            "data": left_data,
        },
        "right": {
            "timestamp_ns": right_ts,
            "width": right_width,
            "height": right_height,
            "format": right_fmt,
            "channel": right_channel,
            "data": right_data,
        },
    }

def decode_pose_payload(payload: bytes):
    if len(payload) < 4+4+8*3:
        raise ValueError("payload too short")
    pose_type, flags = struct.unpack(">II", payload[0:8])
    x, y, z = struct.unpack(">ddd", payload[8:32])
    off = 32
    orientation_xyzw = None
    if flags & 0x1:
        if len(payload) < off + 32:
            raise ValueError("payload too short for quaternion")
        orientation_xyzw = struct.unpack(">dddd", payload[off:off+32])
        off += 32
    confidence = 1.0
    if len(payload) >= off + 4:
        confidence = struct.unpack(">f", payload[off:off+4])[0]
    if not math.isfinite(confidence):
        confidence = 0.0
    confidence = min(1.0, max(0.0, float(confidence)))
    off += 4

    def read_vec3(flag_bit: int):
        nonlocal off
        if (flags & (1 << flag_bit)) == 0:
            return None
        if len(payload) < off + 24:
            return None
        v = struct.unpack(">ddd", payload[off:off+24])
        off += 24
        return v

    linear_velocity_body_mps = read_vec3(2)
    angular_velocity_body_rps = read_vec3(3)
    linear_acceleration_body_mps2 = read_vec3(4)
    angular_acceleration_body_rps2 = read_vec3(5)
    timestamp_ns = None
    if (flags & (1 << 6)) != 0:
        if len(payload) >= off + 8:
            timestamp_ns = struct.unpack(">Q", payload[off:off+8])[0]
            off += 8
    return {
        "pose_type": pose_type,
        "pose_flags": flags,
        "position_m": (x, y, z),
        "orientation_xyzw": orientation_xyzw,
        "confidence": confidence,
        "linear_velocity_body_mps": linear_velocity_body_mps,
        "angular_velocity_body_rps": angular_velocity_body_rps,
        "linear_acceleration_body_mps2": linear_acceleration_body_mps2,
        "angular_acceleration_body_rps2": angular_acceleration_body_rps2,
        "timestamp_ns": timestamp_ns,
    }

def build_reset_vio_pose_payload(position_m=(0.0, 0.0, 0.0), orientation_xyzw=None) -> bytes:
    if position_m is None or len(position_m) != 3:
        raise ValueError("position_m must contain 3 values")
    flags = 0
    payload = [
        struct.pack(">II", 0, flags),
        struct.pack(">ddd", float(position_m[0]), float(position_m[1]), float(position_m[2])),
    ]
    if orientation_xyzw is not None:
        if len(orientation_xyzw) != 4:
            raise ValueError("orientation_xyzw must contain 4 values")
        flags |= 0x1
        payload[0] = struct.pack(">II", 0, flags)
        payload.append(struct.pack(
            ">dddd",
            float(orientation_xyzw[0]),
            float(orientation_xyzw[1]),
            float(orientation_xyzw[2]),
            float(orientation_xyzw[3]),
        ))
    payload.append(struct.pack(">f", 1.0))
    return b"".join(payload)

def decode_reset_vio_pose_payload(payload: bytes):
    pose = decode_pose_payload(payload)
    if int(pose.get("pose_type", -1)) != 0:
        raise ValueError("reset_vio_pose requires body pose type")
    position = pose.get("position_m")
    if position is None or len(position) != 3 or not all(math.isfinite(float(v)) for v in position):
        raise ValueError("reset_vio_pose position invalid")
    orientation = pose.get("orientation_xyzw")
    if orientation is not None and (
        len(orientation) != 4 or not all(math.isfinite(float(v)) for v in orientation)
    ):
        raise ValueError("reset_vio_pose orientation invalid")
    return {
        "pose_type": pose["pose_type"],
        "pose_flags": pose["pose_flags"],
        "position_m": position,
        "orientation_xyzw": orientation,
    }

def decode_constraints_payload(payload: bytes):
    if len(payload) < 4:
        raise ValueError("payload too short")
    count = struct.unpack(">I", payload[0:4])[0]
    segs = []
    off = 4
    for _ in range(count):
        if off + 1 + 6*4 > len(payload):
            break
        t = payload[off]; off += 1
        start = struct.unpack(">fff", payload[off:off+12]); off += 12
        end = struct.unpack(">fff", payload[off:off+12]); off += 12
        segs.append({"type": t, "start": start, "end": end})
    return segs

def decode_viz_payload(payload: bytes):
    if len(payload) < 3:
        raise ValueError("payload too short")
    subtype = payload[0]
    count = struct.unpack(">H", payload[1:3])[0]
    off = 3
    if subtype == 0:
        feats = []
        for _ in range(count):
            x, y, status, fid = struct.unpack(">HHBH", payload[off:off+7])
            off += 7
            feats.append({"x": x, "y": y, "status": status, "id": fid})
        return {"subtype": subtype, "features": feats}
    if subtype == 1:
        dets = []
        for _ in range(count):
            x1, y1, x2, y2 = struct.unpack(">HHHH", payload[off:off+8]); off += 8
            ll = payload[off]; off += 1
            label = payload[off:off+ll].decode("utf-8"); off += ll
            dets.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "label": label})
        return {"subtype": subtype, "detections": dets}
    if subtype == 2:
        matches = []
        for _ in range(count):
            x1, y1, x2, y2, conf = struct.unpack(">HHHHB", payload[off:off+9]); off += 9
            matches.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "confidence": conf})
        return {"subtype": subtype, "matches": matches}
    raise ValueError("unknown viz subtype")

def decode_imu_payload(payload: bytes):
    if len(payload) < 4:
        raise ValueError("payload too short")
    count = struct.unpack(">I", payload[0:4])[0]
    off = 4
    samples = []
    stride = 8 + 6*8
    for _ in range(count):
        if off + stride > len(payload):
            break
        ts = struct.unpack(">Q", payload[off:off+8])[0]; off += 8
        ax, ay, az, gx, gy, gz = struct.unpack(">dddddd", payload[off:off+48]); off += 48
        samples.append({"timestamp_ns": ts, "ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz})
    return samples

def decode_status_payload(payload: bytes):
    return payload.decode("utf-8")

def build_keyframe_payload(timestamp_ns: int,
                           descriptor,
                           flags: int = 0,
                           version: int = 1,
                           descriptor_type: int = 1) -> bytes:
    values = list(descriptor or [])
    header = struct.pack(
        ">BBHQI",
        int(version) & 0xFF,
        int(descriptor_type) & 0xFF,
        int(flags) & 0xFFFF,
        int(timestamp_ns) & 0xFFFFFFFFFFFFFFFF,
        len(values) & 0xFFFFFFFF,
    )
    if not values:
        return header
    return header + struct.pack(">" + "f" * len(values), *[float(v) for v in values])

def decode_keyframe_payload(payload: bytes):
    if len(payload) < 16:
        raise ValueError("payload too short")
    version, descriptor_type, flags, timestamp_ns, dim = struct.unpack(">BBHQI", payload[:16])
    if version != 1:
        raise ValueError(f"unsupported KEYF version {version}")
    if descriptor_type != 1:
        raise ValueError(f"unsupported KEYF descriptor type {descriptor_type}")
    need = 16 + dim * 4
    if len(payload) < need:
        raise ValueError("KEYF descriptor truncated")
    descriptor = list(struct.unpack(">" + "f" * dim, payload[16:need])) if dim else []
    return {
        "version": version,
        "descriptor_type": descriptor_type,
        "flags": flags,
        "timestamp_ns": timestamp_ns,
        "descriptor_dim": dim,
        "descriptor": descriptor,
    }

def decode_vio_state_payload(payload: bytes):
    if len(payload) < 1+1+2+8+4+4+4+4+4+4:
        raise ValueError("payload too short")
    off = 0
    version = payload[off]; off += 1
    state = payload[off]; off += 1
    flags = struct.unpack(">H", payload[off:off+2])[0]; off += 2
    timestamp_ns = struct.unpack(">Q", payload[off:off+8])[0]; off += 8
    fps_current = struct.unpack(">f", payload[off:off+4])[0]; off += 4
    fps_average = struct.unpack(">f", payload[off:off+4])[0]; off += 4
    pose_confidence = struct.unpack(">f", payload[off:off+4])[0]; off += 4
    tracking_rate = struct.unpack(">f", payload[off:off+4])[0]; off += 4
    num_features = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    loop_closures = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    build_version = ""
    imu_hz_current = 0.0
    imu_hz_average_5s = 0.0
    init_reason_code = VIO_INIT_REASON["NONE"]
    static_init_reason_code = VIO_INIT_REASON["NONE"]
    dynamic_init_reason_code = VIO_INIT_REASON["NONE"]
    memory_total_bytes = 0
    memory_used_bytes = 0
    memory_free_bytes = 0
    light_level01 = 1.0
    light_required01 = 1.0
    if version >= 2 and off < len(payload):
        ll = payload[off]; off += 1
        if ll > 0:
            if off + ll > len(payload):
                raise ValueError("VSTA build_version truncated")
            build_version = payload[off:off+ll].decode("utf-8", errors="replace")
            off += ll
    if version >= 3 and off + 8 <= len(payload):
        imu_hz_current = struct.unpack(">f", payload[off:off+4])[0]; off += 4
        imu_hz_average_5s = struct.unpack(">f", payload[off:off+4])[0]; off += 4
    if version >= 4 and off < len(payload):
        init_reason_code = int(payload[off]); off += 1
    if version >= 5 and off + 2 <= len(payload):
        static_init_reason_code = int(payload[off]); off += 1
        dynamic_init_reason_code = int(payload[off]); off += 1
    if version >= 6 and off + 24 <= len(payload):
        memory_total_bytes = struct.unpack(">Q", payload[off:off+8])[0]; off += 8
        memory_used_bytes = struct.unpack(">Q", payload[off:off+8])[0]; off += 8
        memory_free_bytes = struct.unpack(">Q", payload[off:off+8])[0]; off += 8
    if version >= 7 and off + 8 <= len(payload):
        light_level01 = struct.unpack(">f", payload[off:off+4])[0]; off += 4
        light_required01 = struct.unpack(">f", payload[off:off+4])[0]; off += 4
    return {
        "version": version,
        "state": state,
        "flags": flags,
        "timestamp_ns": timestamp_ns,
        "fps_current": fps_current,
        "fps_average": fps_average,
        "pose_confidence": pose_confidence,
        "tracking_rate": tracking_rate,
        "num_features": num_features,
        "loop_closures": loop_closures,
        "build_version": build_version,
        "imu_hz_current": imu_hz_current,
        "imu_hz_average_5s": imu_hz_average_5s,
        "init_reason_code": init_reason_code,
        "static_init_reason_code": static_init_reason_code,
        "dynamic_init_reason_code": dynamic_init_reason_code,
        "memory_total_bytes": memory_total_bytes,
        "memory_used_bytes": memory_used_bytes,
        "memory_free_bytes": memory_free_bytes,
        "light_level01": light_level01,
        "light_required01": light_required01,
    }

def decode_fea3_payload(payload: bytes):
    if len(payload) < 2:
        raise ValueError("payload too short")
    count = struct.unpack(">H", payload[0:2])[0]
    feats = []
    off = 2
    for _ in range(count):
        if off + 2 + 24 > len(payload):
            break
        fid = struct.unpack(">H", payload[off:off+2])[0]; off += 2
        x, y, z = struct.unpack(">ddd", payload[off:off+24]); off += 24
        feats.append({"id": fid, "x": x, "y": y, "z": z})
    return feats

def decode_pcld_payload(payload: bytes):
    if len(payload) < 4:
        raise ValueError("payload too short")
    count = struct.unpack(">I", payload[0:4])[0]
    off = 4
    point_size: Optional[float] = None
    if len(payload) >= 8 + count * (3*4 + 3):
        point_size = struct.unpack(">f", payload[off:off+4])[0]
        off += 4
    points = []
    for _ in range(count):
        if off + 3*4 + 3 > len(payload):
            break
        x, y, z = struct.unpack(">fff", payload[off:off+12]); off += 12
        r, g, b = struct.unpack(">BBB", payload[off:off+3]); off += 3
        points.append({"x": x, "y": y, "z": z, "r": r, "g": g, "b": b})
    return {"points": points, "point_size": point_size}

# Command helpers
def build_command_payload(req_id: int, name: str, data: bytes = b"") -> bytes:
    name_bytes = (name or "").encode("utf-8")
    name_len = min(255, len(name_bytes))
    data = data or b""
    return b"".join([
        struct.pack(">I", req_id & 0xFFFFFFFF),
        struct.pack("B", name_len),
        name_bytes[:name_len],
        struct.pack(">I", len(data)),
        data
    ])

def build_command_response_payload(req_id: int, status: int, message: str = "", data: bytes = b"") -> bytes:
    msg_bytes = (message or "").encode("utf-8")
    msg_len = min(65535, len(msg_bytes))
    data = data or b""
    return b"".join([
        struct.pack(">I", req_id & 0xFFFFFFFF),
        struct.pack("B", status & 0xFF),
        struct.pack(">H", msg_len),
        msg_bytes[:msg_len],
        struct.pack(">I", len(data)),
        data
    ])

def decode_command_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 4 + 1 + 4:
        raise ValueError("payload too short")
    off = 0
    req_id = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    name_len = payload[off]; off += 1
    if len(payload) < off + name_len + 4:
        raise ValueError("payload truncated")
    name = payload[off:off+name_len].decode("utf-8"); off += name_len
    data_len = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    if len(payload) < off + data_len:
        raise ValueError("payload truncated for data")
    data = payload[off:off+data_len]
    return {"req_id": req_id, "name": name, "data": data}

def decode_command_response_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 4 + 1 + 2 + 4:
        raise ValueError("payload too short")
    off = 0
    req_id = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    status = payload[off]; off += 1
    msg_len = struct.unpack(">H", payload[off:off+2])[0]; off += 2
    if len(payload) < off + msg_len + 4:
        raise ValueError("payload truncated")
    message = payload[off:off+msg_len].decode("utf-8"); off += msg_len
    data_len = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    if len(payload) < off + data_len:
        raise ValueError("payload truncated (data)")
    data = payload[off:off+data_len]
    return {"req_id": req_id, "status": status, "message": message, "data": data}


# Config helpers
def build_config_request_payload(version: int = 1,
                                 op: int = CONFIG_OP["GET"],
                                 key: str = "",
                                 value: bytes = b"") -> bytes:
    key_bytes = (key or "").encode("utf-8")
    key_len = min(255, len(key_bytes))
    value = value or b""
    return b"".join([
        struct.pack("B", version & 0xFF),
        struct.pack("B", op & 0xFF),
        struct.pack("B", key_len),
        key_bytes[:key_len],
        struct.pack(">I", len(value)),
        value,
    ])


def build_config_response_payload(version: int = 1,
                                  op: int = CONFIG_OP["GET"],
                                  success: int = 0,
                                  has_value: bool = False,
                                  key: str = "",
                                  message: str = "",
                                  value: bytes = b"") -> bytes:
    key_bytes = (key or "").encode("utf-8")
    key_len = min(255, len(key_bytes))
    msg_bytes = (message or "").encode("utf-8")
    msg_len = min(65535, len(msg_bytes))
    value = value or b""
    return b"".join([
        struct.pack("B", version & 0xFF),
        struct.pack("B", op & 0xFF),
        struct.pack("B", 1 if success else 0),
        struct.pack("B", 1 if has_value else 0),
        struct.pack("B", key_len),
        key_bytes[:key_len],
        struct.pack(">H", msg_len),
        msg_bytes[:msg_len],
        struct.pack(">I", len(value)),
        value,
    ])


def decode_config_request_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 1 + 1 + 1 + 4:
        raise ValueError("payload too short")
    off = 0
    version = payload[off]; off += 1
    op = payload[off]; off += 1
    key_len = payload[off]; off += 1
    if len(payload) < off + key_len + 4:
        raise ValueError("payload truncated")
    key = payload[off:off+key_len].decode("utf-8"); off += key_len
    value_len = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    if len(payload) < off + value_len:
        raise ValueError("payload truncated for value")
    value = payload[off:off+value_len]
    return {"version": version, "op": op, "key": key, "value": value}


def decode_config_response_payload(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 1 + 1 + 1 + 1 + 1 + 2 + 4:
        raise ValueError("payload too short")
    off = 0
    version = payload[off]; off += 1
    op = payload[off]; off += 1
    success = payload[off]; off += 1
    has_value = payload[off] != 0; off += 1
    key_len = payload[off]; off += 1
    if len(payload) < off + key_len + 2 + 4:
        raise ValueError("payload truncated")
    key = payload[off:off+key_len].decode("utf-8"); off += key_len
    msg_len = struct.unpack(">H", payload[off:off+2])[0]; off += 2
    if len(payload) < off + msg_len + 4:
        raise ValueError("payload truncated")
    message = payload[off:off+msg_len].decode("utf-8"); off += msg_len
    value_len = struct.unpack(">I", payload[off:off+4])[0]; off += 4
    if len(payload) < off + value_len:
        raise ValueError("payload truncated for value")
    value = payload[off:off+value_len]
    return {
        "version": version,
        "op": op,
        "success": success,
        "has_value": has_value,
        "key": key,
        "message": message,
        "value": value,
    }
