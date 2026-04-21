import threading
import time
from typing import Any, Callable, Dict, Optional

import mighty_protocol as mp
from dispatcher import FrameDispatcher

from .utils import clamp01, sleep_seconds, to_bytes


DEFAULT_OPTS = {
    "command_timeout_s": 2.0,
    "auto_reconnect": True,
    "reconnect_delay_s": 0.3,
    "emit_stat_as_status": True,
    "normalize_channel_aliases": True,
}

VIO_STATE = mp.VIO_STATE
VIO_INIT_REASON = mp.VIO_INIT_REASON


class MightyClient:
    def __init__(self, device: Any, **opts: Any):
        if device is None or not hasattr(device, "connect") or not hasattr(device, "disconnect"):
            raise ValueError("device must implement connect() and disconnect()")

        self.device = device
        self.opts = dict(DEFAULT_OPTS)
        self.opts.update(opts)

        self._listeners: Dict[str, set] = {
            "image": set(),
            "pose": set(),
            "imu": set(),
            "vio_state": set(),
            "viz": set(),
            "lcon": set(),
            "status": set(),
            "reset": set(),
            "any": set(),
            "error": set(),
        }

        self._state_lock = threading.RLock()
        self._running = False
        self._stream_active = False
        self._loop_thread: Optional[threading.Thread] = None
        self._req_id = 1

        self._stats = {
            "rx_frames": 0,
            "rx_bytes": 0,
            "decode_errors": 0,
            "reconnects": 0,
            "command_timeouts": 0,
        }

        self._frame_dispatcher = FrameDispatcher(self._handle_frame)

    def connect(self) -> None:
        with self._state_lock:
            if self._running:
                return
            self._running = True
            self._loop_thread = threading.Thread(target=self._transport_loop, name="MightyClientLoop", daemon=True)
            self._loop_thread.start()

    def disconnect(self) -> None:
        with self._state_lock:
            self._running = False
        try:
            self.device.disconnect()
        except Exception as exc:
            self._emit_error("transport", "disconnect_failed", str(exc), exc)

        t = None
        with self._state_lock:
            t = self._loop_thread
        if t is not None:
            t.join(timeout=3.0)

        with self._state_lock:
            self._loop_thread = None
            self._stream_active = False

    def is_connected(self) -> bool:
        with self._state_lock:
            return bool(self._stream_active)

    def stats(self) -> Dict[str, int]:
        with self._state_lock:
            return dict(self._stats)

    def on_image(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("image", cb)

    def on_pose(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("pose", cb)

    def on_imu(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("imu", cb)

    def on_vio_state(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("vio_state", cb)

    def on_viz(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("viz", cb)

    def on_lcon(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("lcon", cb)

    def on_constraints(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("lcon", cb)

    def on_status(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("status", cb)

    def on_reset(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("reset", cb)

    def on_any(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("any", cb)

    def on_error(self, cb: Callable[[dict], None]) -> Callable[[], None]:
        return self._subscribe("error", cb)

    def command(self, name: str, data: bytes = b"") -> Dict[str, Any]:
        req_id = self._alloc_req_id()

        if not isinstance(name, str) or not name:
            return {
                "ok": False,
                "req_id": req_id,
                "status": 1,
                "message": "command name must be a non-empty string",
                "data": b"",
            }

        if not hasattr(self.device, "send_command_payload"):
            return {
                "ok": False,
                "req_id": req_id,
                "status": 1,
                "message": "device does not support command request/response",
                "data": b"",
            }

        cmd_payload = mp.build_command_payload(req_id, name, to_bytes(data))

        timeout_s = float(self.opts.get("command_timeout_s", 0.0) or 0.0)
        try:
            raw = self._call_with_timeout(lambda: self.device.send_command_payload(cmd_payload), timeout_s)
            decoded = mp.decode_command_response_payload(to_bytes(raw))
            return {
                "ok": int(decoded.get("status", 1)) == 0,
                "req_id": decoded.get("req_id", req_id),
                "status": int(decoded.get("status", 1)),
                "message": decoded.get("message", ""),
                "data": to_bytes(decoded.get("data", b"")),
            }
        except TimeoutError as exc:
            with self._state_lock:
                self._stats["command_timeouts"] += 1
            self._emit_error("command", "command_timeout", str(exc), exc)
            return {
                "ok": False,
                "req_id": req_id,
                "status": 1,
                "message": str(exc),
                "data": b"",
            }
        except Exception as exc:
            self._emit_error("command", "command_failed", str(exc), exc)
            return {
                "ok": False,
                "req_id": req_id,
                "status": 1,
                "message": str(exc),
                "data": b"",
            }

    def config_get(self, key: str, as_text: bool = False) -> Dict[str, Any]:
        cfgq = mp.build_config_request_payload(version=1, op=mp.CONFIG_OP["GET"], key=key, value=b"")
        cmd = self.command("config", cfgq)
        if not cmd.get("ok"):
            return {
                "ok": False,
                "found": False,
                "key": key,
                "value": "" if as_text else b"",
                "message": cmd.get("message", ""),
            }
        try:
            cfgr = mp.decode_config_response_payload(cmd.get("data", b""))
            value = to_bytes(cfgr.get("value", b""))
            return {
                "ok": int(cfgr.get("success", 0)) == 1,
                "found": bool(cfgr.get("has_value", False)),
                "key": cfgr.get("key", key),
                "value": value.decode("utf-8") if as_text else value,
                "message": cfgr.get("message", ""),
            }
        except Exception as exc:
            self._emit_error("config", "decode_failed", str(exc), exc)
            return {
                "ok": False,
                "found": False,
                "key": key,
                "value": "" if as_text else b"",
                "message": str(exc),
            }

    def config_set(self, key: str, value: Any) -> Dict[str, Any]:
        value_bytes = to_bytes(value)
        cfgq = mp.build_config_request_payload(version=1, op=mp.CONFIG_OP["SET"], key=key, value=value_bytes)
        cmd = self.command("config", cfgq)
        if not cmd.get("ok"):
            return {
                "ok": False,
                "key": key,
                "value": value_bytes,
                "message": cmd.get("message", ""),
            }
        try:
            cfgr = mp.decode_config_response_payload(cmd.get("data", b""))
            return {
                "ok": int(cfgr.get("success", 0)) == 1,
                "key": cfgr.get("key", key),
                "value": to_bytes(cfgr.get("value", b"")),
                "message": cfgr.get("message", ""),
            }
        except Exception as exc:
            self._emit_error("config", "decode_failed", str(exc), exc)
            return {
                "ok": False,
                "key": key,
                "value": value_bytes,
                "message": str(exc),
            }

    def start_vio(self) -> Dict[str, Any]:
        return self.command("start_vio")

    def stop_vio(self) -> Dict[str, Any]:
        return self.command("stop_vio")

    def _subscribe(self, key: str, cb: Callable[[dict], None]) -> Callable[[], None]:
        if key not in self._listeners:
            raise ValueError(f"unknown event key: {key}")
        if not callable(cb):
            raise ValueError("callback must be callable")
        with self._state_lock:
            self._listeners[key].add(cb)

        def _unsubscribe() -> None:
            with self._state_lock:
                self._listeners[key].discard(cb)

        return _unsubscribe

    def _emit(self, key: str, payload: dict) -> None:
        with self._state_lock:
            callbacks = list(self._listeners.get(key, []))
        for cb in callbacks:
            try:
                cb(payload)
            except Exception as exc:
                if key != "error":
                    self._emit_error("protocol", "listener_threw", str(exc), exc)

    def _emit_any(self, payload: dict) -> None:
        self._emit("any", payload)

    def _emit_error(self, scope: str, code: str, message: str, cause: Optional[Exception] = None) -> None:
        self._emit(
            "error",
            {
                "scope": scope,
                "code": code,
                "message": message,
                "cause": cause,
            },
        )

    def _alloc_req_id(self) -> int:
        with self._state_lock:
            out = int(self._req_id) & 0xFFFFFFFF
            self._req_id = (self._req_id + 1) & 0xFFFFFFFF
            if self._req_id == 0:
                self._req_id = 1
            return out

    def _has_listeners(self, key: str) -> bool:
        with self._state_lock:
            return len(self._listeners.get(key, [])) > 0

    def _handle_bytes(self, chunk: bytes) -> None:
        b = to_bytes(chunk)
        with self._state_lock:
            self._stats["rx_bytes"] += len(b)
        self._frame_dispatcher.feed(b)

    def _map_channel_alias(self, channel: str) -> Optional[str]:
        if not bool(self.opts.get("normalize_channel_aliases", True)):
            return None
        c = (channel or "").strip().lower()
        if c in ("cam0", "preview", "left"):
            return "cam0"
        if c in ("cam1", "right"):
            return "cam1"
        return None

    @staticmethod
    def _map_pose_type(pose_type: int) -> str:
        if pose_type == 0:
            return "body"
        if pose_type == 1:
            return "camera"
        return "other"

    def _handle_frame(self, frame: dict) -> None:
        with self._state_lock:
            self._stats["rx_frames"] += 1

        frame_type = frame.get("type", "")
        payload = frame.get("payload", b"")
        wants_any = self._has_listeners("any")

        try:
            if frame_type == "RAW ":
                if not self._has_listeners("image") and not wants_any:
                    return
                raw = mp.decode_raw_payload(payload)
                mapped = {
                    "kind": "raw",
                    "timestamp_ns": raw.get("timestamp_ns"),
                    "width": raw.get("width"),
                    "height": raw.get("height"),
                    "format": raw.get("format"),
                    "channel": raw.get("channel"),
                    "channel_alias": self._map_channel_alias(raw.get("channel", "")),
                    "data": to_bytes(raw.get("data", b"")),
                }
                self._emit("image", mapped)
                if wants_any:
                    self._emit_any({"type": "image", "data": mapped})
                return

            if frame_type == "SRAW":
                if not self._has_listeners("image") and not wants_any:
                    return
                sraw = mp.decode_stereo_raw_payload(payload)
                left_raw = sraw.get("left", {})
                right_raw = sraw.get("right", {})
                left = {
                    "kind": "raw",
                    "timestamp_ns": left_raw.get("timestamp_ns"),
                    "width": left_raw.get("width"),
                    "height": left_raw.get("height"),
                    "format": left_raw.get("format"),
                    "channel": left_raw.get("channel"),
                    "channel_alias": self._map_channel_alias(left_raw.get("channel", "")),
                    "data": to_bytes(left_raw.get("data", b"")),
                }
                right = {
                    "kind": "raw",
                    "timestamp_ns": right_raw.get("timestamp_ns"),
                    "width": right_raw.get("width"),
                    "height": right_raw.get("height"),
                    "format": right_raw.get("format"),
                    "channel": right_raw.get("channel"),
                    "channel_alias": self._map_channel_alias(right_raw.get("channel", "")),
                    "data": to_bytes(right_raw.get("data", b"")),
                }
                mapped = {"kind": "stereo_raw", "left": left, "right": right}
                self._emit("image", mapped)
                if wants_any:
                    self._emit_any({"type": "image", "data": mapped})
                return

            if frame_type in ("POSE", "UPOS"):
                if not self._has_listeners("pose") and not wants_any:
                    return
                p = mp.decode_pose_payload(payload)
                pose_flags = int(p.get("pose_flags", 0))
                mapped = {
                    "is_public": frame_type == "POSE",
                    "packet_type": frame_type.strip(),
                    "pose_type": self._map_pose_type(int(p.get("pose_type", -1))),
                    "pose_type_raw": int(p.get("pose_type", -1)),
                    "pose_flags": pose_flags,
                    "frame_id": "odom",
                    "child_frame_id": "base_link",
                    "position_m": p.get("position_m"),
                    "orientation_xyzw": p.get("orientation_xyzw"),
                    "confidence": clamp01(float(p.get("confidence", 1.0))),
                    "is_keyframe": (pose_flags & (1 << 1)) != 0,
                    "linear_velocity_body_mps": p.get("linear_velocity_body_mps"),
                    "angular_velocity_body_rps": p.get("angular_velocity_body_rps"),
                    "linear_acceleration_body_mps2": p.get("linear_acceleration_body_mps2"),
                    "angular_acceleration_body_rps2": p.get("angular_acceleration_body_rps2"),
                    "timestamp_ns": p.get("timestamp_ns"),
                }
                self._emit("pose", mapped)
                if wants_any:
                    self._emit_any({"type": "pose", "data": mapped})
                return

            if frame_type == "IMU ":
                if not self._has_listeners("imu") and not wants_any:
                    return
                samples = mp.decode_imu_payload(payload)
                mapped = {"samples": samples}
                self._emit("imu", mapped)
                if wants_any:
                    self._emit_any({"type": "imu", "data": mapped})
                return

            if frame_type == "VSTA":
                if not self._has_listeners("vio_state") and not wants_any:
                    return
                s = mp.decode_vio_state_payload(payload)
                mapped = {
                    "version": s.get("version"),
                    "state": s.get("state"),
                    "flags": s.get("flags"),
                    "timestamp_ns": s.get("timestamp_ns"),
                    "fps_current": s.get("fps_current"),
                    "fps_average": s.get("fps_average"),
                    "pose_confidence": s.get("pose_confidence"),
                    "tracking_rate": s.get("tracking_rate"),
                    "num_features": s.get("num_features"),
                    "loop_closures": s.get("loop_closures"),
                    "build_version": s.get("build_version") or "",
                    "imu_hz_current": s.get("imu_hz_current"),
                    "imu_hz_average_5s": s.get("imu_hz_average_5s"),
                    "init_reason_code": s.get("init_reason_code", VIO_INIT_REASON["NONE"]),
                    "static_init_reason_code": s.get("static_init_reason_code", VIO_INIT_REASON["NONE"]),
                    "dynamic_init_reason_code": s.get("dynamic_init_reason_code", VIO_INIT_REASON["NONE"]),
                    "memory_total_bytes": s.get("memory_total_bytes"),
                    "memory_used_bytes": s.get("memory_used_bytes"),
                    "memory_free_bytes": s.get("memory_free_bytes"),
                    "light_level01": s.get("light_level01"),
                    "light_required01": s.get("light_required01"),
                }
                self._emit("vio_state", mapped)
                if wants_any:
                    self._emit_any({"type": "vio_state", "data": mapped})
                return

            if frame_type == "VIZ ":
                if not self._has_listeners("viz") and not wants_any:
                    return
                v = mp.decode_viz_payload(payload)
                subtype = int(v.get("subtype", 255))
                if subtype == 0:
                    mapped = {"subtype": "features", "features": v.get("features", [])}
                elif subtype == 1:
                    mapped = {"subtype": "detections", "detections": v.get("detections", [])}
                elif subtype == 2:
                    mapped = {"subtype": "matches", "matches": v.get("matches", [])}
                else:
                    mapped = {"subtype": "unknown", "raw_subtype": subtype, "raw": to_bytes(payload)}
                self._emit("viz", mapped)
                if wants_any:
                    self._emit_any({"type": "viz", "data": mapped})
                return

            if frame_type == "LCON":
                if not self._has_listeners("lcon") and not wants_any:
                    return
                segs = mp.decode_constraints_payload(payload)
                mapped = {"segments": segs}
                self._emit("lcon", mapped)
                if wants_any:
                    self._emit_any({"type": "lcon", "data": mapped})
                return

            if frame_type == "STAT":
                if not bool(self.opts.get("emit_stat_as_status", True)):
                    return
                if not self._has_listeners("status") and not wants_any:
                    return
                mapped = {"text": mp.decode_status_payload(payload)}
                self._emit("status", mapped)
                if wants_any:
                    self._emit_any({"type": "status", "data": mapped})
                return

            if frame_type == "RSET":
                if not self._has_listeners("reset") and not wants_any:
                    return
                mapped = {"received_at_ms": int(time.time() * 1000)}
                self._emit("reset", mapped)
                if wants_any:
                    self._emit_any({"type": "reset", "data": mapped})
                return

            if wants_any:
                self._emit_any({"type": "unknown", "raw_type": frame_type, "payload": to_bytes(payload)})
        except Exception as exc:
            with self._state_lock:
                self._stats["decode_errors"] += 1
            self._emit_error("protocol", "decode_failed", str(exc), exc)

    def _transport_loop(self) -> None:
        while True:
            with self._state_lock:
                if not self._running:
                    self._stream_active = False
                    return
                self._stream_active = True

            try:
                self.device.connect(self._handle_bytes)
                with self._state_lock:
                    self._stream_active = False
                with self._state_lock:
                    running = self._running
                if running:
                    self._emit_error("transport", "stream_closed", "stream closed")
            except Exception as exc:
                with self._state_lock:
                    self._stream_active = False
                    running = self._running
                if running:
                    self._emit_error("transport", "stream_error", str(exc), exc)

            with self._state_lock:
                running = self._running
                auto_reconnect = bool(self.opts.get("auto_reconnect", True))
            if not running or not auto_reconnect:
                return

            with self._state_lock:
                self._stats["reconnects"] += 1
            sleep_seconds(float(self.opts.get("reconnect_delay_s", 0.3)))

    @staticmethod
    def _call_with_timeout(fn: Callable[[], Any], timeout_s: float) -> Any:
        if timeout_s <= 0:
            return fn()

        done = threading.Event()
        result_holder: Dict[str, Any] = {}

        def _run() -> None:
            try:
                result_holder["result"] = fn()
            except Exception as exc:
                result_holder["error"] = exc
            finally:
                done.set()

        t = threading.Thread(target=_run, name="MightyClientCommand", daemon=True)
        t.start()
        if not done.wait(timeout_s):
            raise TimeoutError("command timeout")
        if "error" in result_holder:
            raise result_holder["error"]
        return result_holder.get("result")
