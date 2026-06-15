import os
import struct
import sys
import threading
import time

HERE = os.path.dirname(__file__)
sys.path.append(os.path.join(HERE, "..", "python"))

import mighty_protocol as mp  # noqa: E402
from mighty_sdk import MightyClient  # noqa: E402


def build_pose_payload(
    pose_type=0,
    pose_flags=0,
    position=(0.0, 0.0, 0.0),
    quat=None,
    confidence=1.0,
    linvel=None,
    angvel=None,
    linacc=None,
    angacc=None,
    timestamp_ns=None,
):
    flags = int(pose_flags)
    if quat is not None and len(quat) == 4:
        flags |= 0x1
    if linvel is not None and len(linvel) == 3:
        flags |= (1 << 2)
    if angvel is not None and len(angvel) == 3:
        flags |= (1 << 3)
    if linacc is not None and len(linacc) == 3:
        flags |= (1 << 4)
    if angacc is not None and len(angacc) == 3:
        flags |= (1 << 5)
    if timestamp_ns is not None:
        flags |= (1 << 6)

    buf = struct.pack(">II", int(pose_type), flags)
    buf += struct.pack(">ddd", float(position[0]), float(position[1]), float(position[2]))
    if flags & 0x1:
        q = quat if quat is not None else (0.0, 0.0, 0.0, 1.0)
        buf += struct.pack(">dddd", float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    buf += struct.pack(">f", float(confidence))
    if flags & (1 << 2):
        buf += struct.pack(">ddd", float(linvel[0]), float(linvel[1]), float(linvel[2]))
    if flags & (1 << 3):
        buf += struct.pack(">ddd", float(angvel[0]), float(angvel[1]), float(angvel[2]))
    if flags & (1 << 4):
        buf += struct.pack(">ddd", float(linacc[0]), float(linacc[1]), float(linacc[2]))
    if flags & (1 << 5):
        buf += struct.pack(">ddd", float(angacc[0]), float(angacc[1]), float(angacc[2]))
    if (flags & (1 << 6)) and timestamp_ns is not None:
        buf += struct.pack(">Q", int(timestamp_ns))
    return buf


def build_imu_payload(samples):
    buf = struct.pack(">I", len(samples))
    for s in samples:
        buf += struct.pack(">Q ddd ddd",
                           int(s["timestamp_ns"]),
                           float(s["ax"]), float(s["ay"]), float(s["az"]),
                           float(s["gx"]), float(s["gy"]), float(s["gz"]))
    return buf


def build_vsta_payload():
    return b"".join([
        struct.pack(">B", 4),              # version
        struct.pack(">B", 2),              # state
        struct.pack(">H", 3),              # flags
        struct.pack(">Q", 13),             # timestamp_ns
        struct.pack(">f", 30.0),           # fps_current
        struct.pack(">f", 29.0),           # fps_average
        struct.pack(">f", 0.8),            # pose_confidence
        struct.pack(">f", 0.9),            # tracking_rate
        struct.pack(">I", 100),            # num_features
        struct.pack(">I", 2),              # loop_closures
        bytes([4]) + b"test",              # build_version
        struct.pack(">f", 200.0),          # imu_hz_current
        struct.pack(">f", 199.0),          # imu_hz_average_5s
        struct.pack(">B", mp.VIO_INIT_REASON["NONE"]),  # init_reason_code
    ])


def build_lcon_payload():
    return b"".join([
        struct.pack(">I", 1),
        struct.pack(">B", 1),
        struct.pack(">fff", 0.0, 0.0, 0.0),
        struct.pack(">fff", 1.0, 1.0, 1.0),
    ])


class MockDevice:
    def __init__(self):
        self._on_bytes = None
        self._connected = False
        self._stop = threading.Event()
        self._calib = b"%YAML:1.0\ncam0:\n  intrinsics: [1,2,3,4]\n"

    def get_info(self):
        return {"transport": "mock"}

    def connect(self, on_bytes):
        if self._connected:
            raise RuntimeError("already connected")
        self._connected = True
        self._on_bytes = on_bytes
        self._stop.clear()
        self._stop.wait()
        self._connected = False
        self._on_bytes = None

    def disconnect(self):
        self._stop.set()

    def emit_packet(self, pkt):
        if self._on_bytes:
            self._on_bytes(pkt)

    def send_command_payload(self, cmd_payload):
        cmd = mp.decode_command_payload(cmd_payload)
        name = cmd["name"]

        if name in ("start_vio", "stop_vio"):
            return mp.build_command_response_payload(cmd["req_id"], 0, "ok", b"")

        if name == "keyframes":
            action = bytes(cmd["data"]).decode("utf-8")
            message = "keyframes disabled" if action == "status" else f"keyframes {action}"
            return mp.build_command_response_payload(cmd["req_id"], 0, message, b"")

        if name == "config":
            cfgq = mp.decode_config_request_payload(cmd["data"])
            if cfgq["key"] != "calib":
                cfg_err = mp.build_config_response_payload(
                    version=cfgq["version"],
                    op=cfgq["op"],
                    success=0,
                    has_value=False,
                    key=cfgq["key"],
                    message="unknown key",
                    value=b"",
                )
                return mp.build_command_response_payload(cmd["req_id"], 1, "config failed", cfg_err)

            if cfgq["op"] == mp.CONFIG_OP["GET"]:
                cfgr = mp.build_config_response_payload(
                    version=cfgq["version"],
                    op=cfgq["op"],
                    success=1,
                    has_value=True,
                    key="calib",
                    message="loaded",
                    value=self._calib,
                )
                return mp.build_command_response_payload(cmd["req_id"], 0, "ok", cfgr)

            if cfgq["op"] == mp.CONFIG_OP["SET"]:
                self._calib = cfgq["value"]
                cfgr = mp.build_config_response_payload(
                    version=cfgq["version"],
                    op=cfgq["op"],
                    success=1,
                    has_value=True,
                    key="calib",
                    message="saved",
                    value=self._calib,
                )
                return mp.build_command_response_payload(cmd["req_id"], 0, "ok", cfgr)

        return mp.build_command_response_payload(cmd["req_id"], 1, "unknown command", b"")


def wait_until(pred, timeout_s=2.0):
    start = time.time()
    while time.time() - start < timeout_s:
        if pred():
            return True
        time.sleep(0.01)
    return False


def main():
    device = MockDevice()
    client = MightyClient(device, auto_reconnect=False)

    seen = {
        "image": 0,
        "pose": 0,
        "imu": 0,
        "vsta": 0,
        "lcon": 0,
        "keyframe": 0,
        "status": 0,
        "reset": 0,
        "any": 0,
        "error": 0,
    }

    last = {"image": None, "pose": None, "vsta": None, "keyframe": None}

    client.on_image(lambda v: (seen.__setitem__("image", seen["image"] + 1), last.__setitem__("image", v)))
    client.on_pose(lambda v: (seen.__setitem__("pose", seen["pose"] + 1), last.__setitem__("pose", v)))
    client.on_imu(lambda _: seen.__setitem__("imu", seen["imu"] + 1))
    client.on_vio_state(lambda v: (seen.__setitem__("vsta", seen["vsta"] + 1), last.__setitem__("vsta", v)))
    client.on_lcon(lambda _: seen.__setitem__("lcon", seen["lcon"] + 1))
    client.on_keyframe(lambda v: (seen.__setitem__("keyframe", seen["keyframe"] + 1), last.__setitem__("keyframe", v)))
    client.on_status(lambda _: seen.__setitem__("status", seen["status"] + 1))
    client.on_reset(lambda _: seen.__setitem__("reset", seen["reset"] + 1))
    client.on_any(lambda _: seen.__setitem__("any", seen["any"] + 1))
    client.on_error(lambda _: seen.__setitem__("error", seen["error"] + 1))

    client.connect()
    assert wait_until(lambda: client.is_connected(), timeout_s=1.0)

    device.emit_packet(mp.make_packet(mp.TYPE["RAW"], mp.build_raw_payload(
        10,
        2,
        1,
        mp.RAW_FORMAT["GRAY8"],
        "cam0",
        b"\x01\x02",
    )))

    device.emit_packet(mp.make_packet(mp.TYPE["POSE"], build_pose_payload(
        pose_type=0,
        position=(1.0, 2.0, 3.0),
        quat=(0.1, 0.2, 0.3, 0.9),
        confidence=0.5,
        linvel=(4.0, 5.0, 6.0),
        angvel=(0.4, 0.5, 0.6),
        linacc=(7.0, 8.0, 9.0),
        angacc=(0.7, 0.8, 0.9),
        timestamp_ns=11)))

    device.emit_packet(mp.make_packet(mp.TYPE["IMU"], build_imu_payload([
        {"timestamp_ns": 12, "ax": 0.1, "ay": 0.2, "az": 0.3, "gx": 0.4, "gy": 0.5, "gz": 0.6}])))

    device.emit_packet(mp.make_packet(mp.TYPE["VSTA"], build_vsta_payload()))

    device.emit_packet(mp.make_packet(mp.TYPE["LCON"], build_lcon_payload()))

    device.emit_packet(mp.make_packet(mp.TYPE["KEYF"], mp.build_keyframe_payload(
        14,
        [0.25, -0.5, 1.0],
    )))

    device.emit_packet(mp.make_packet(mp.TYPE["STAT"], b"hello"))
    device.emit_packet(mp.make_packet(mp.TYPE["RSET"]))
    device.emit_packet(mp.make_packet(b"ZZZZ", b"\xaa"))

    assert wait_until(lambda: seen["any"] >= 9)

    assert seen["image"] == 1
    assert seen["pose"] == 1
    assert seen["imu"] == 1
    assert seen["vsta"] == 1
    assert int(last["vsta"]["init_reason_code"]) == mp.VIO_INIT_REASON["NONE"]
    assert seen["lcon"] == 1
    assert seen["keyframe"] == 1
    assert last["keyframe"]["timestamp_ns"] == 14
    assert last["keyframe"]["descriptor_dim"] == 3
    assert abs(float(last["keyframe"]["descriptor"][1]) + 0.5) < 1e-6
    assert seen["status"] == 1
    assert seen["reset"] == 1
    assert last["image"]["kind"] == "raw"
    assert last["image"]["channel"] == "cam0"
    assert last["image"]["channel_alias"] == "cam0"
    assert bool(last["pose"]["is_public"]) is True
    assert last["pose"]["packet_type"] == "POSE"
    assert last["pose"]["pose_type"] == "body"
    assert int(last["pose"]["pose_type_raw"]) == 0
    assert last["pose"]["frame_id"] == "odom"
    assert last["pose"]["child_frame_id"] == "base_link"
    assert last["pose"]["is_keyframe"] is False
    flags = int(last["pose"]["pose_flags"])
    assert (flags & 0x1) != 0
    assert (flags & (1 << 2)) != 0
    assert (flags & (1 << 3)) != 0
    assert (flags & (1 << 4)) != 0
    assert (flags & (1 << 5)) != 0
    assert (flags & (1 << 6)) != 0
    assert last["pose"]["timestamp_ns"] == 11
    assert last["pose"]["orientation_xyzw"] is not None
    assert last["pose"]["linear_velocity_body_mps"] is not None
    assert last["pose"]["angular_velocity_body_rps"] is not None
    assert last["pose"]["linear_acceleration_body_mps2"] is not None
    assert last["pose"]["angular_acceleration_body_rps2"] is not None
    assert abs(float(last["pose"]["orientation_xyzw"][3]) - 0.9) < 1e-6
    assert abs(float(last["pose"]["linear_velocity_body_mps"][2]) - 6.0) < 1e-6
    assert abs(float(last["pose"]["angular_velocity_body_rps"][1]) - 0.5) < 1e-6
    assert abs(float(last["pose"]["linear_acceleration_body_mps2"][0]) - 7.0) < 1e-6
    assert abs(float(last["pose"]["angular_acceleration_body_rps2"][2]) - 0.9) < 1e-6

    start_res = client.start_vio()
    assert start_res["ok"]

    keyframes_on = client.set_keyframes_enabled(True)
    assert keyframes_on["ok"]
    assert keyframes_on["message"] == "keyframes on"

    keyframes_status = client.keyframes_status()
    assert keyframes_status["ok"]
    assert keyframes_status["message"] == "keyframes disabled"

    cfg_get = client.config_get("calib", as_text=True)
    assert cfg_get["ok"]
    assert cfg_get["found"]
    assert "intrinsics" in cfg_get["value"]

    payload_text = "%YAML:1.0\nfoo: 1\n"
    cfg_set = client.config_set("calib", payload_text)
    assert cfg_set["ok"]

    cfg_get2 = client.config_get("calib", as_text=True)
    assert cfg_get2["ok"]
    assert cfg_get2["value"] == payload_text

    stats = client.stats()
    assert stats["rx_frames"] >= 8
    assert stats["rx_bytes"] > 0

    client.disconnect()
    assert not client.is_connected()

    print("Python SDK unit test passed")


if __name__ == "__main__":
    main()
