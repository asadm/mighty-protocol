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
    "RSET": b"RSET",
    "FEA3": b"FEA3",
    "PCLD": b"PCLD",
    "CMD": b"CMD ",
    "CRES": b"CRES",
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
    quat = None
    if flags & 0x1:
        if len(payload) < off + 32:
            raise ValueError("payload too short for quaternion")
        quat = struct.unpack(">dddd", payload[off:off+32])
        off += 32
    confidence = 1.0
    if len(payload) >= off + 4:
        confidence = struct.unpack(">f", payload[off:off+4])[0]
    if not math.isfinite(confidence):
        confidence = 0.0
    confidence = min(1.0, max(0.0, float(confidence)))
    return {
        "pose_type": pose_type,
        "pose_flags": flags,
        "position": (x, y, z),
        "quat": quat,
        "confidence": confidence,
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
