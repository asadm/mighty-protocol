import math
import os
import sys
import random
import secrets

HERE = os.path.dirname(__file__)
sys.path.append(os.path.join(HERE, "..", "python"))
import mighty_protocol as mp  # noqa: E402

SAMPLE = {
    "jpg_ts": 111,
    "jpg_channel": "preview",
    "jpg_data": b"\x01\x02\x03",
    "rjpg_ts": 222,
    "rjpg_data": b"\xaa\xbb",
    "raw_ts": 333,
    "raw_channel": "cam0",
    "raw_width": 4,
    "raw_height": 2,
    "raw_format": mp.RAW_FORMAT["GRAY8"],
    "raw_data": b"\x10\x11\x12\x13\x14\x15\x16\x17",
    "sraw_left_ts": 444,
    "sraw_right_ts": 445,
    "sraw_left_channel": "cam0",
    "sraw_right_channel": "cam1",
    "sraw_left_width": 2,
    "sraw_left_height": 1,
    "sraw_left_format": mp.RAW_FORMAT["GRAY8"],
    "sraw_left_data": b"\x21\x22",
    "sraw_right_width": 2,
    "sraw_right_height": 1,
    "sraw_right_format": mp.RAW_FORMAT["GRAY8"],
    "sraw_right_data": b"\x31\x32",
    "pose": {
        "pose_type": 0,
        "pose_flags": 0x3 | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6),
        "position_m": (1.1, 2.2, 3.3),
        "orientation_xyzw": (0.1, 0.2, 0.3, 0.9),
        "confidence": 0.82,
        "linear_velocity_body_mps": (4.0, 5.0, 6.0),
        "angular_velocity_body_rps": (0.4, 0.5, 0.6),
        "linear_acceleration_body_mps2": (7.0, 8.0, 9.0),
        "angular_acceleration_body_rps2": (0.7, 0.8, 0.9),
        "timestamp_ns": 777,
    },
    "upose": {
        "pose_type": 0,
        "pose_flags": 0x1 | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6),
        "position_m": (4.4, 5.5, 6.6),
        "orientation_xyzw": (0.4, 0.5, 0.6, 0.7),
        "confidence": 0.41,
        "linear_velocity_body_mps": (1.0, 1.1, 1.2),
        "angular_velocity_body_rps": (0.1, 0.2, 0.3),
        "linear_acceleration_body_mps2": (2.0, 2.1, 2.2),
        "angular_acceleration_body_rps2": (0.01, 0.02, 0.03),
        "timestamp_ns": 778,
    },
    "constraints": [
        {"type": 0, "start": (0.1, 0.2, 0.3), "end": (0.4, 0.5, 0.6)},
        {"type": 1, "start": (1.0, 1.1, 1.2), "end": (1.3, 1.4, 1.5)},
    ],
    "viz0": {"subtype": 0, "features": [{"x": 10, "y": 20, "status": 1, "id": 7}, {"x": 30, "y": 40, "status": 4, "id": 8}]},
    "viz1": {"subtype": 1, "detections": [{"x1": 5, "y1": 6, "x2": 25, "y2": 26, "label": "car"}]},
    "viz2": {"subtype": 2, "matches": [{"x1": 100, "y1": 110, "x2": 120, "y2": 130, "confidence": 200}]},
    "imu": [
        {"timestamp_ns": 1000, "ax": 0.1, "ay": 0.2, "az": 0.3, "gx": 1.1, "gy": 1.2, "gz": 1.3},
        {"timestamp_ns": 2000, "ax": 0.4, "ay": 0.5, "az": 0.6, "gx": 1.4, "gy": 1.5, "gz": 1.6},
    ],
    "status": "STATUS_OK",
    "fea3": [
        {"id": 1, "x": 0.1, "y": 0.2, "z": 0.3},
        {"id": 2, "x": 1.1, "y": 1.2, "z": 1.3},
    ],
    "pcld": {
        "points": [
            {"x": 1, "y": 2, "z": 3, "r": 10, "g": 20, "b": 30},
            {"x": 4, "y": 5, "z": 6, "r": 40, "g": 50, "b": 60},
        ],
        "point_size": 1.5,
    },
    "keyframe": {
        "timestamp_ns": 123456789,
        "descriptor": [0.125, -0.25, 0.5, 1.0],
    },
    "vsta": {
        "version": 8,
        "state": 2,
        "flags": 0x1234,
        "timestamp_ns": 999,
        "fps_current": 31.5,
        "fps_average": 30.0,
        "imu_hz_current": 60.0,
        "imu_hz_average_5s": 59.7,
        "pose_confidence": 0.75,
        "tracking_rate": 0.88,
        "num_features": 321,
        "loop_closures": 7,
        "build_version": "Mighty v.20260208-deadbeef",
        "init_reason_code": mp.VIO_INIT_REASON["NONE"],
        "static_init_reason_code": mp.VIO_INIT_REASON["NONE"],
        "dynamic_init_reason_code": mp.VIO_INIT_REASON["NONE"],
        "memory_total_bytes": 1024,
        "memory_used_bytes": 512,
        "memory_free_bytes": 256,
        "light_level01": 0.8,
        "light_required01": 0.3,
        "translation_confidence01": 0.34,
        "translation_observability01": 0.21,
        "degraded_reason_flags": (
            mp.VIO_DEGRADED_REASON["LOW_TRANSLATION_OBSERVABILITY"] |
            mp.VIO_DEGRADED_REASON["LOW_PARALLAX_POSE_HOLD"]
        ),
    },
}

def almost(a, b, eps=1e-6):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) < eps

def build_packets():
    pkts = []
    pkts.append(mp.make_packet(mp.TYPE["RSET"]))
    pkts.append(mp.make_packet(mp.TYPE["JPG"], struct_jpg(False)))
    pkts.append(mp.make_packet(mp.TYPE["RJPG"], struct_jpg(True)))
    pkts.append(mp.make_packet(mp.TYPE["RAW"], struct_raw()))
    pkts.append(mp.make_packet(mp.TYPE["SRAW"], struct_sraw()))
    pkts.append(mp.make_packet(mp.TYPE["POSE"], struct_pose(SAMPLE["pose"])))
    pkts.append(mp.make_packet(mp.TYPE["UPOSE"], struct_pose(SAMPLE["upose"])))
    pkts.append(mp.make_packet(mp.TYPE["LCON"], struct_constraints()))
    pkts.append(mp.make_packet(mp.TYPE["VIZ"], struct_viz(SAMPLE["viz0"])))
    pkts.append(mp.make_packet(mp.TYPE["VIZ"], struct_viz(SAMPLE["viz1"])))
    pkts.append(mp.make_packet(mp.TYPE["VIZ"], struct_viz(SAMPLE["viz2"])))
    pkts.append(mp.make_packet(mp.TYPE["IMU"], struct_imu()))
    pkts.append(mp.make_packet(mp.TYPE["STAT"], SAMPLE["status"].encode()))
    pkts.append(mp.make_packet(mp.TYPE["VSTA"], struct_vsta()))
    pkts.append(mp.make_packet(mp.TYPE["FEA3"], struct_fea3()))
    pkts.append(mp.make_packet(mp.TYPE["PCLD"], struct_pcld()))
    pkts.append(mp.make_packet(mp.TYPE["KEYF"], mp.build_keyframe_payload(
        SAMPLE["keyframe"]["timestamp_ns"],
        SAMPLE["keyframe"]["descriptor"],
    )))
    return pkts

def struct_vsta():
    import struct
    s = SAMPLE["vsta"]
    base = struct.pack(">BBHQffffII",
                       int(s["version"]) & 0xff,
                       int(s["state"]) & 0xff,
                       int(s["flags"]) & 0xffff,
                       int(s["timestamp_ns"]),
                       float(s["fps_current"]),
                       float(s["fps_average"]),
                       float(s["pose_confidence"]),
                       float(s["tracking_rate"]),
                       int(s["num_features"]) & 0xffffffff,
                       int(s["loop_closures"]) & 0xffffffff)
    if int(s["version"]) >= 2:
        bv = (s.get("build_version") or "").encode("utf-8")[:255]
        base += bytes([len(bv)]) + bv
    if int(s["version"]) >= 3:
        base += struct.pack(">ff",
                            float(s.get("imu_hz_current", 0.0)),
                            float(s.get("imu_hz_average_5s", 0.0)))
    if int(s["version"]) >= 4:
        base += struct.pack(">B", int(s.get("init_reason_code", mp.VIO_INIT_REASON["NONE"])) & 0xff)
    if int(s["version"]) >= 5:
        base += struct.pack(">BB",
                            int(s.get("static_init_reason_code", mp.VIO_INIT_REASON["NONE"])) & 0xff,
                            int(s.get("dynamic_init_reason_code", mp.VIO_INIT_REASON["NONE"])) & 0xff)
    if int(s["version"]) >= 6:
        base += struct.pack(">QQQ",
                            int(s.get("memory_total_bytes", 0)) & 0xffffffffffffffff,
                            int(s.get("memory_used_bytes", 0)) & 0xffffffffffffffff,
                            int(s.get("memory_free_bytes", 0)) & 0xffffffffffffffff)
    if int(s["version"]) >= 7:
        base += struct.pack(">ff",
                            float(s.get("light_level01", 1.0)),
                            float(s.get("light_required01", 1.0)))
    if int(s["version"]) >= 8:
        base += struct.pack(">ffI",
                            float(s.get("translation_confidence01", 1.0)),
                            float(s.get("translation_observability01", 1.0)),
                            int(s.get("degraded_reason_flags", 0)) & 0xffffffff)
    return base

def struct_jpg(is_ref):
    import struct
    ts = struct.pack(">Q", SAMPLE["rjpg_ts"] if is_ref else SAMPLE["jpg_ts"])
    if is_ref:
        return ts + SAMPLE["rjpg_data"]
    chan = SAMPLE["jpg_channel"].encode()
    return ts + bytes([len(chan)]) + chan + SAMPLE["jpg_data"]

def struct_raw():
    import struct
    ts = struct.pack(">Q", SAMPLE["raw_ts"])
    header = ts + struct.pack(">I", SAMPLE["raw_width"]) + struct.pack(">I", SAMPLE["raw_height"])
    chan = SAMPLE["raw_channel"].encode()
    header += bytes([SAMPLE["raw_format"], len(chan)]) + chan
    return header + SAMPLE["raw_data"]

def struct_sraw():
    import struct
    left_chan = SAMPLE["sraw_left_channel"].encode()
    right_chan = SAMPLE["sraw_right_channel"].encode()
    header = struct.pack(">QQII", SAMPLE["sraw_left_ts"], SAMPLE["sraw_right_ts"],
                         SAMPLE["sraw_left_width"], SAMPLE["sraw_left_height"])
    header += bytes([SAMPLE["sraw_left_format"], len(left_chan)]) + left_chan
    header += struct.pack(">II", SAMPLE["sraw_right_width"], SAMPLE["sraw_right_height"])
    header += bytes([SAMPLE["sraw_right_format"], len(right_chan)]) + right_chan
    header += struct.pack(">II", len(SAMPLE["sraw_left_data"]), len(SAMPLE["sraw_right_data"]))
    return header + SAMPLE["sraw_left_data"] + SAMPLE["sraw_right_data"]

def struct_pose(data):
    import struct
    pose_flags = data["pose_flags"]
    buf = struct.pack(">II", data["pose_type"], pose_flags)
    buf += struct.pack(">ddd", *data["position_m"])
    if pose_flags & 0x1:
        buf += struct.pack(">dddd", *data["orientation_xyzw"])
    buf += struct.pack(">f", float(data.get("confidence", 1.0)))
    if pose_flags & (1 << 2):
        buf += struct.pack(">ddd", *data["linear_velocity_body_mps"])
    if pose_flags & (1 << 3):
        buf += struct.pack(">ddd", *data["angular_velocity_body_rps"])
    if pose_flags & (1 << 4):
        buf += struct.pack(">ddd", *data["linear_acceleration_body_mps2"])
    if pose_flags & (1 << 5):
        buf += struct.pack(">ddd", *data["angular_acceleration_body_rps2"])
    if pose_flags & (1 << 6):
        buf += struct.pack(">Q", int(data["timestamp_ns"]))
    return buf

def struct_constraints():
    import struct
    buf = struct.pack(">I", len(SAMPLE["constraints"]))
    for c in SAMPLE["constraints"]:
        buf += struct.pack(">B", c["type"])
        buf += struct.pack(">fff", *c["start"])
        buf += struct.pack(">fff", *c["end"])
    return buf

def struct_viz(v):
    import struct
    subtype = v["subtype"]
    body = b""
    if subtype == 0:
        for f in v["features"]:
            body += struct.pack(">HHBH", f["x"], f["y"], f.get("status", 0), f["id"])
        count = len(v["features"])
    elif subtype == 1:
        for d in v["detections"]:
            lbl = d["label"].encode()
            body += struct.pack(">HHHHB", d["x1"], d["y1"], d["x2"], d["y2"], len(lbl)) + lbl
        count = len(v["detections"])
    else:
        for m in v["matches"]:
            body += struct.pack(">HHHHB", m["x1"], m["y1"], m["x2"], m["y2"], m["confidence"])
        count = len(v["matches"])
    header = struct.pack(">B H", subtype, count)
    return header + body

def struct_imu():
    import struct
    buf = struct.pack(">I", len(SAMPLE["imu"]))
    for s in SAMPLE["imu"]:
        buf += struct.pack(">Q ddd ddd", s["timestamp_ns"], s["ax"], s["ay"], s["az"], s["gx"], s["gy"], s["gz"])
    return buf

def struct_fea3():
    import struct
    buf = struct.pack(">H", len(SAMPLE["fea3"]))
    for f in SAMPLE["fea3"]:
        buf += struct.pack(">H ddd", f["id"], f["x"], f["y"], f["z"])
    return buf

def struct_pcld():
    import struct
    buf = struct.pack(">I", len(SAMPLE["pcld"]["points"]))
    buf += struct.pack(">f", SAMPLE["pcld"]["point_size"])
    for p in SAMPLE["pcld"]["points"]:
        buf += struct.pack(">fffBBB", p["x"], p["y"], p["z"], p["r"], p["g"], p["b"])
    return buf

def random_jpg(is_ref=False):
    import struct, secrets
    ts = struct.pack(">Q", random.randint(0, 10_000))
    data = secrets.token_bytes(8)
    if is_ref:
        return ts + data
    chan = b"ch"
    return ts + bytes([len(chan)]) + chan + data

def main():
    stream = b"".join(build_packets())
    frames, rest = mp.parse_frames(stream)
    assert not rest
    assert len(frames) == 17

    idx = 0
    assert frames[idx]["type"] == "RSET"; idx += 1
    jpg = mp.decode_jpg_payload(frames[idx]["payload"], False); idx += 1
    assert jpg["timestamp_ns"] == SAMPLE["jpg_ts"]
    rjpg = mp.decode_jpg_payload(frames[idx]["payload"], True); idx += 1
    assert rjpg["timestamp_ns"] == SAMPLE["rjpg_ts"]
    raw = mp.decode_raw_payload(frames[idx]["payload"]); idx += 1
    assert raw["timestamp_ns"] == SAMPLE["raw_ts"]
    assert raw["width"] == SAMPLE["raw_width"]
    assert raw["height"] == SAMPLE["raw_height"]
    assert raw["format"] == SAMPLE["raw_format"]
    assert raw["channel"] == SAMPLE["raw_channel"]
    assert raw["data"] == SAMPLE["raw_data"]
    sraw = mp.decode_stereo_raw_payload(frames[idx]["payload"]); idx += 1
    assert sraw["left"]["timestamp_ns"] == SAMPLE["sraw_left_ts"]
    assert sraw["left"]["width"] == SAMPLE["sraw_left_width"]
    assert sraw["left"]["height"] == SAMPLE["sraw_left_height"]
    assert sraw["left"]["format"] == SAMPLE["sraw_left_format"]
    assert sraw["left"]["channel"] == SAMPLE["sraw_left_channel"]
    assert sraw["left"]["data"] == SAMPLE["sraw_left_data"]
    assert sraw["right"]["timestamp_ns"] == SAMPLE["sraw_right_ts"]
    assert sraw["right"]["width"] == SAMPLE["sraw_right_width"]
    assert sraw["right"]["height"] == SAMPLE["sraw_right_height"]
    assert sraw["right"]["format"] == SAMPLE["sraw_right_format"]
    assert sraw["right"]["channel"] == SAMPLE["sraw_right_channel"]
    assert sraw["right"]["data"] == SAMPLE["sraw_right_data"]
    pose = mp.decode_pose_payload(frames[idx]["payload"]); idx += 1
    assert pose["pose_type"] == SAMPLE["pose"]["pose_type"]
    assert almost(pose.get("confidence", 1.0), SAMPLE["pose"]["confidence"], 1e-6)
    assert pose["timestamp_ns"] == SAMPLE["pose"]["timestamp_ns"]
    assert pose["linear_velocity_body_mps"] is not None
    assert pose["angular_velocity_body_rps"] is not None
    assert pose["linear_acceleration_body_mps2"] is not None
    assert pose["angular_acceleration_body_rps2"] is not None
    assert almost(pose["linear_velocity_body_mps"][2], SAMPLE["pose"]["linear_velocity_body_mps"][2], 1e-6)
    assert almost(pose["angular_velocity_body_rps"][1], SAMPLE["pose"]["angular_velocity_body_rps"][1], 1e-6)
    upose = mp.decode_pose_payload(frames[idx]["payload"]); idx += 1
    assert almost(upose["position_m"][2], SAMPLE["upose"]["position_m"][2])
    assert almost(upose.get("confidence", 1.0), SAMPLE["upose"]["confidence"], 1e-6)
    assert upose["timestamp_ns"] == SAMPLE["upose"]["timestamp_ns"]
    lcon = mp.decode_constraints_payload(frames[idx]["payload"]); idx += 1
    assert len(lcon) == len(SAMPLE["constraints"])
    viz0 = mp.decode_viz_payload(frames[idx]["payload"]); idx += 1
    viz1 = mp.decode_viz_payload(frames[idx]["payload"]); idx += 1
    viz2 = mp.decode_viz_payload(frames[idx]["payload"]); idx += 1
    imu = mp.decode_imu_payload(frames[idx]["payload"]); idx += 1
    assert len(imu) == len(SAMPLE["imu"])
    stat = mp.decode_status_payload(frames[idx]["payload"]); idx += 1
    assert stat == SAMPLE["status"]
    vsta = mp.decode_vio_state_payload(frames[idx]["payload"]); idx += 1
    assert vsta["version"] == SAMPLE["vsta"]["version"]
    assert vsta["state"] == SAMPLE["vsta"]["state"]
    assert vsta["flags"] == SAMPLE["vsta"]["flags"]
    assert vsta["timestamp_ns"] == SAMPLE["vsta"]["timestamp_ns"]
    assert almost(vsta["fps_current"], SAMPLE["vsta"]["fps_current"], 1e-3)
    assert almost(vsta["fps_average"], SAMPLE["vsta"]["fps_average"], 1e-3)
    assert almost(vsta["imu_hz_current"], SAMPLE["vsta"]["imu_hz_current"], 1e-3)
    assert almost(vsta["imu_hz_average_5s"], SAMPLE["vsta"]["imu_hz_average_5s"], 1e-3)
    assert almost(vsta["pose_confidence"], SAMPLE["vsta"]["pose_confidence"], 1e-3)
    assert almost(vsta["tracking_rate"], SAMPLE["vsta"]["tracking_rate"], 1e-3)
    assert vsta["num_features"] == SAMPLE["vsta"]["num_features"]
    assert vsta["loop_closures"] == SAMPLE["vsta"]["loop_closures"]
    assert vsta.get("build_version", "") == SAMPLE["vsta"]["build_version"]
    assert vsta["init_reason_code"] == SAMPLE["vsta"]["init_reason_code"]
    assert vsta["static_init_reason_code"] == SAMPLE["vsta"]["static_init_reason_code"]
    assert vsta["dynamic_init_reason_code"] == SAMPLE["vsta"]["dynamic_init_reason_code"]
    assert vsta["memory_total_bytes"] == SAMPLE["vsta"]["memory_total_bytes"]
    assert vsta["memory_used_bytes"] == SAMPLE["vsta"]["memory_used_bytes"]
    assert vsta["memory_free_bytes"] == SAMPLE["vsta"]["memory_free_bytes"]
    assert almost(vsta["light_level01"], SAMPLE["vsta"]["light_level01"], 1e-3)
    assert almost(vsta["light_required01"], SAMPLE["vsta"]["light_required01"], 1e-3)
    assert almost(vsta["translation_confidence01"], SAMPLE["vsta"]["translation_confidence01"], 1e-3)
    assert almost(vsta["translation_observability01"], SAMPLE["vsta"]["translation_observability01"], 1e-3)
    assert vsta["degraded_reason_flags"] == SAMPLE["vsta"]["degraded_reason_flags"]
    fea3 = mp.decode_fea3_payload(frames[idx]["payload"]); idx += 1
    assert fea3[1]["id"] == 2
    pcld = mp.decode_pcld_payload(frames[idx]["payload"]); idx += 1
    assert len(pcld["points"]) == len(SAMPLE["pcld"]["points"])
    keyf = mp.decode_keyframe_payload(frames[idx]["payload"]); idx += 1
    assert keyf["timestamp_ns"] == SAMPLE["keyframe"]["timestamp_ns"]
    assert keyf["descriptor_dim"] == len(SAMPLE["keyframe"]["descriptor"])
    assert almost(keyf["descriptor"][2], SAMPLE["keyframe"]["descriptor"][2], 1e-6)

    # Dispatcher sanity
    from dispatcher import FrameDispatcher
    import random
    seen = []
    d = FrameDispatcher(lambda f: seen.append(f["type"]))
    chunked = stream[:20] + stream[20:]  # two chunks
    d.feed(chunked)
    assert len(seen) == 17

    # Decoded dispatcher sanity
    from decoded_dispatcher import DecodedDispatcher
    decoded_seen = []
    dd = DecodedDispatcher()
    dd.on_jpg = lambda ts, ch, data, is_ref: decoded_seen.append(("jpg", is_ref))
    dd.on_raw = lambda ts, w, h, fmt, ch, data: decoded_seen.append(("raw", (w, h, fmt)))
    dd.on_pose = lambda pose, is_unopt: decoded_seen.append(("pose", is_unopt))
    dd.on_constraints = lambda segs: decoded_seen.append(("lcon", len(segs)))
    dd.on_features = lambda feats: decoded_seen.append(("fea3", len(feats)))
    dd.on_pointcloud = lambda pts, ps: decoded_seen.append(("pcld", len(pts)))
    dd.on_keyframe = lambda keyf: decoded_seen.append(("keyf", keyf["descriptor_dim"]))
    dd.on_viz = lambda _: decoded_seen.append(("viz", None))
    dd.on_imu = lambda samples: decoded_seen.append(("imu", len(samples)))
    dd.on_status = lambda txt: decoded_seen.append(("stat", txt))
    dd.on_reset = lambda: decoded_seen.append(("rset", True))
    dd.feed(stream)
    assert len(decoded_seen) >= 8

    # Fuzz decode/dispatch
    types = ["JPG", "RJPG", "RAW", "SRAW", "POSE", "UPOSE", "LCON", "IMU", "STAT", "VSTA"]
    for _ in range(20):
      pkts = []
      for _ in range(5):
        t = random.choice(types)
        if t == "JPG":
          payload = random_jpg(False)
        elif t == "RJPG":
          payload = random_jpg(True)
        elif t == "RAW":
          payload = mp.build_raw_payload(123, 4, 2, mp.RAW_FORMAT["GRAY8"], "raw", secrets.token_bytes(8))
        elif t == "SRAW":
          payload = mp.build_stereo_raw_payload(1, 2, 2, 1, mp.RAW_FORMAT["GRAY8"], "cam0", secrets.token_bytes(2),
                                                2, 1, mp.RAW_FORMAT["GRAY8"], "cam1", secrets.token_bytes(2))
        elif t in ("POSE", "UPOSE"):
          payload = struct_pose({
              "pose_type": 0,
              "pose_flags": 0x3,
              "position_m": (0.1, 0.2, 0.3),
              "orientation_xyzw": (0.1, 0.2, 0.3, 0.9),
          })
        elif t == "LCON":
          payload = struct_constraints()
        elif t == "IMU":
          payload = struct_imu()
        elif t == "STAT":
          payload = b"fuzz"
        elif t == "VSTA":
          payload = struct_vsta()
        pkts.append(mp.make_packet(mp.TYPE[t], payload))
      merged = b"".join(pkts)
      # parse_frames robustness
      frames, rest = mp.parse_frames(merged)
      assert rest == b""
      dd = DecodedDispatcher()
      dd.feed(merged)

    print("Python consumer test passed")

if __name__ == "__main__":
    main()
