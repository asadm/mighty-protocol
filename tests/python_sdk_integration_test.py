import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(__file__)
sys.path.append(os.path.join(HERE, "..", "python"))

import mighty_protocol as mp  # noqa: E402
from mighty_sdk import MightyClient, MightyWebDevice  # noqa: E402


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
    degraded_reason_flags = (
        mp.VIO_DEGRADED_REASON["LOW_TRANSLATION_OBSERVABILITY"] |
        mp.VIO_DEGRADED_REASON["LOW_PARALLAX_POSE_HOLD"] |
        mp.VIO_DEGRADED_REASON["STATIC_TRANSLATION_CONSTRAINED"] |
        mp.VIO_DEGRADED_REASON["ROTATION_ONLY_3DOF"]
    )
    return b"".join([
        struct.pack(">B", 8),
        struct.pack(">B", 2),
        struct.pack(">H", 1),
        struct.pack(">Q", 123),
        struct.pack(">f", 30.0),
        struct.pack(">f", 29.0),
        struct.pack(">f", 0.7),
        struct.pack(">f", 0.8),
        struct.pack(">I", 120),
        struct.pack(">I", 3),
        bytes([4]) + b"test",
        struct.pack(">f", 200.0),
        struct.pack(">f", 199.0),
        struct.pack(">B", mp.VIO_INIT_REASON["NONE"]),
        struct.pack(">B", mp.VIO_INIT_REASON["NONE"]),
        struct.pack(">B", mp.VIO_INIT_REASON["NONE"]),
        struct.pack(">Q", 1024),
        struct.pack(">Q", 512),
        struct.pack(">Q", 256),
        struct.pack(">f", 0.8),
        struct.pack(">f", 0.3),
        struct.pack(">f", 0.34),
        struct.pack(">f", 0.21),
        struct.pack(">I", degraded_reason_flags),
    ])


class IntegrationState:
    def __init__(self):
        self.lock = threading.Lock()
        self.calib = b"%YAML:1.0\ncam0:\n  intrinsics: [9,8,7,6]\n"

        self.stream_chunks = [
            mp.make_packet(mp.TYPE["RSET"]),
            mp.make_packet(mp.TYPE["RAW"], mp.build_raw_payload(10, 2, 1, mp.RAW_FORMAT["GRAY8"], "cam0", b"\x01\x02")),
            mp.make_packet(mp.TYPE["POSE"], build_pose_payload(
                pose_type=0,
                position=(1.0, 2.0, 3.0),
                quat=(0.1, 0.2, 0.3, 0.9),
                confidence=0.9,
                linvel=(4.0, 5.0, 6.0),
                angvel=(0.4, 0.5, 0.6),
                linacc=(7.0, 8.0, 9.0),
                angacc=(0.7, 0.8, 0.9),
                timestamp_ns=11)),
            mp.make_packet(mp.TYPE["IMU"], build_imu_payload([
                {"timestamp_ns": 12, "ax": 0.1, "ay": 0.2, "az": 0.3, "gx": 0.4, "gy": 0.5, "gz": 0.6},
            ])),
            mp.make_packet(mp.TYPE["VSTA"], build_vsta_payload()),
            mp.make_packet(mp.TYPE["STAT"], b"hello"),
        ]


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    state = None

    def log_message(self, format, *args):  # noqa: A003
        return

    def do_GET(self):  # noqa: N802
        if self.path != "/stream":
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()

        for chunk in list(self.state.stream_chunks):
            # split to stress stream parser buffering
            mid = max(1, len(chunk) // 2)
            for part in (chunk[:mid], chunk[mid:]):
                if not part:
                    continue
                self.wfile.write(f"{len(part):X}\r\n".encode("ascii"))
                self.wfile.write(part)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(0.005)

        self.wfile.write(b"0\r\n\r\n")
        self.wfile.flush()

    def do_POST(self):  # noqa: N802
        if self.path != "/command":
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return

        length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(length)
        cmd = mp.decode_command_payload(body)

        if cmd["name"] in ("start_vio", "stop_vio"):
            payload = mp.build_command_response_payload(cmd["req_id"], 0, "ok", b"")
            self._send_octet(payload)
            return

        if cmd["name"] == "keyframes":
            action = bytes(cmd["data"]).decode("utf-8")
            message = "keyframes disabled" if action == "status" else f"keyframes {action}"
            payload = mp.build_command_response_payload(cmd["req_id"], 0, message, b"")
            self._send_octet(payload)
            return

        if cmd["name"] == "config":
            cfgq = mp.decode_config_request_payload(cmd["data"])
            if cfgq["key"] != "calib":
                cfgr = mp.build_config_response_payload(
                    version=cfgq["version"],
                    op=cfgq["op"],
                    success=0,
                    has_value=False,
                    key=cfgq["key"],
                    message="unknown key",
                    value=b"",
                )
                payload = mp.build_command_response_payload(cmd["req_id"], 1, "config failed", cfgr)
                self._send_octet(payload)
                return

            if cfgq["op"] == mp.CONFIG_OP["GET"]:
                with self.state.lock:
                    value = self.state.calib
                cfgr = mp.build_config_response_payload(
                    version=cfgq["version"],
                    op=cfgq["op"],
                    success=1,
                    has_value=True,
                    key="calib",
                    message="loaded",
                    value=value,
                )
                payload = mp.build_command_response_payload(cmd["req_id"], 0, "ok", cfgr)
                self._send_octet(payload)
                return

            if cfgq["op"] == mp.CONFIG_OP["SET"]:
                with self.state.lock:
                    self.state.calib = cfgq["value"]
                    value = self.state.calib
                cfgr = mp.build_config_response_payload(
                    version=cfgq["version"],
                    op=cfgq["op"],
                    success=1,
                    has_value=True,
                    key="calib",
                    message="saved",
                    value=value,
                )
                payload = mp.build_command_response_payload(cmd["req_id"], 0, "ok", cfgr)
                self._send_octet(payload)
                return

        payload = mp.build_command_response_payload(cmd["req_id"], 1, "unknown command", b"")
        self._send_octet(payload)

    def _send_octet(self, payload: bytes):
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)
        self.wfile.flush()


def wait_until(pred, timeout_s=3.0):
    start = time.time()
    while time.time() - start < timeout_s:
        if pred():
            return True
        time.sleep(0.01)
    return False


def main():
    state = IntegrationState()
    Handler.state = state

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    port = server.server_address[1]
    base_url = f"http://127.0.0.1:{port}"

    try:
        device = MightyWebDevice(base_url=base_url, read_timeout_s=0.2)
        client = MightyClient(device, auto_reconnect=False)

        seen = {
            "image": 0,
            "pose": 0,
            "imu": 0,
            "vsta": 0,
            "status": 0,
            "reset": 0,
            "any": 0,
        }
        last_pose = {"value": None}
        last_vsta = {"value": None}

        client.on_image(lambda _: seen.__setitem__("image", seen["image"] + 1))
        client.on_pose(lambda p: (seen.__setitem__("pose", seen["pose"] + 1), last_pose.__setitem__("value", p)))
        client.on_imu(lambda _: seen.__setitem__("imu", seen["imu"] + 1))
        client.on_vio_state(lambda v: (seen.__setitem__("vsta", seen["vsta"] + 1), last_vsta.__setitem__("value", v)))
        client.on_status(lambda _: seen.__setitem__("status", seen["status"] + 1))
        client.on_reset(lambda _: seen.__setitem__("reset", seen["reset"] + 1))
        client.on_any(lambda _: seen.__setitem__("any", seen["any"] + 1))

        client.connect()
        assert wait_until(lambda: seen["any"] >= 6)

        assert seen["image"] >= 1
        assert seen["pose"] >= 1
        assert seen["imu"] >= 1
        assert seen["vsta"] >= 1
        assert last_vsta["value"] is not None
        assert abs(float(last_vsta["value"]["translation_confidence01"]) - 0.34) < 1e-3
        assert abs(float(last_vsta["value"]["translation_observability01"]) - 0.21) < 1e-3
        assert int(last_vsta["value"]["degraded_reason_flags"]) == (
            mp.VIO_DEGRADED_REASON["LOW_TRANSLATION_OBSERVABILITY"] |
            mp.VIO_DEGRADED_REASON["LOW_PARALLAX_POSE_HOLD"] |
            mp.VIO_DEGRADED_REASON["STATIC_TRANSLATION_CONSTRAINED"] |
            mp.VIO_DEGRADED_REASON["ROTATION_ONLY_3DOF"]
        )
        assert seen["status"] >= 1
        assert seen["reset"] >= 1
        assert last_pose["value"] is not None
        p = last_pose["value"]
        flags = int(p.get("pose_flags", 0))
        assert bool(p.get("is_public")) is True
        assert p.get("packet_type") == "POSE"
        assert p.get("pose_type") == "body"
        assert p.get("pose_type_raw") == 0
        assert p.get("frame_id") == "odom"
        assert p.get("child_frame_id") == "base_link"
        assert (flags & 0x1) != 0
        assert (flags & (1 << 2)) != 0
        assert (flags & (1 << 3)) != 0
        assert (flags & (1 << 4)) != 0
        assert (flags & (1 << 5)) != 0
        assert (flags & (1 << 6)) != 0
        assert int(p.get("timestamp_ns") or 0) == 11
        assert abs(float((p.get("orientation_xyzw") or [0, 0, 0, 0])[3]) - 0.9) < 1e-6
        assert abs(float((p.get("linear_velocity_body_mps") or [0, 0, 0])[2]) - 6.0) < 1e-6

        cmd = client.start_vio()
        assert cmd["ok"]

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

        cfg_set = client.config_set("calib", "%YAML:1.0\nfoo: 9\n")
        assert cfg_set["ok"]
        cfg_get2 = client.config_get("calib", as_text=True)
        assert cfg_get2["ok"]
        assert cfg_get2["value"] == "%YAML:1.0\nfoo: 9\n"

        client.disconnect()
        print("Python SDK HTTP integration test passed")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
