import ctypes
import os
import platform
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import mighty_protocol as mp


class LoopClosureError(RuntimeError):
    pass


class MlcOptions(ctypes.Structure):
    _fields_ = [
        ("version", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
    ]


class MlcRawImage(ctypes.Structure):
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("frame_id", ctypes.c_int),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("format", ctypes.c_uint8),
        ("data", ctypes.POINTER(ctypes.c_uint8)),
        ("size_bytes", ctypes.c_size_t),
    ]


class MlcPose(ctypes.Structure):
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("px", ctypes.c_double),
        ("py", ctypes.c_double),
        ("pz", ctypes.c_double),
        ("qw", ctypes.c_double),
        ("qx", ctypes.c_double),
        ("qy", ctypes.c_double),
        ("qz", ctypes.c_double),
        ("frame", ctypes.c_uint8),
        ("confidence", ctypes.c_float),
    ]


class MlcDeviceKeyframe(ctypes.Structure):
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("frame_id", ctypes.c_int),
        ("descriptor_type", ctypes.c_uint8),
        ("flags", ctypes.c_uint16),
        ("descriptor", ctypes.POINTER(ctypes.c_float)),
        ("descriptor_count", ctypes.c_size_t),
    ]


class MlcEvent(ctypes.Structure):
    _fields_ = [
        ("version", ctypes.c_uint32),
        ("type", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8),
        ("timestamp_ns", ctypes.c_uint64),
        ("current_keyframe", ctypes.c_size_t),
        ("matched_keyframe", ctypes.c_size_t),
        ("correction_tx", ctypes.c_double),
        ("correction_ty", ctypes.c_double),
        ("correction_tz", ctypes.c_double),
    ]


_EVENT_NAMES = {
    1: "loop_closure",
}


def default_library_path() -> Optional[Path]:
    override = os.environ.get("MIGHTY_LOOPCLOSURE_LIBRARY")
    if override:
        return Path(override)

    system = platform.system().lower()
    machine = platform.machine().lower()
    if system == "darwin":
        os_name = "macos"
        ext = "dylib"
    elif system == "linux":
        os_name = "linux"
        ext = "so"
    elif system == "windows":
        os_name = "windows"
        ext = "dll"
    else:
        return None

    if machine in ("arm64", "aarch64"):
        arch = "arm64"
    elif machine in ("x86_64", "amd64"):
        arch = "x64"
    elif machine.startswith("armv7") or machine in ("armv7l", "armv7"):
        arch = "armv7"
    else:
        arch = machine

    lib_name = "mighty_loopclosure_device.dll" if ext == "dll" else f"libmighty_loopclosure_device.{ext}"
    roots = [
        Path(__file__).resolve().parents[2],
        Path(__file__).resolve().parents[1],
        Path.cwd(),
    ]
    variants = [
        f"{os_name}-{arch}-static",
        f"{os_name}-{arch}",
        f"{os_name}-{arch}-wasm",
    ]
    for root in roots:
        for variant in variants:
            candidate = root / "lib" / "loopclosure" / variant / "lib" / lib_name
            if candidate.exists():
                return candidate
    return None


class NativeLoopClosure:
    def __init__(
        self,
        library_path: Optional[str] = None,
        options: Optional[Dict[str, Any]] = None,
        on_event: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        path = Path(library_path) if library_path else default_library_path()
        if path is None or not path.exists():
            raise LoopClosureError("loopclosure native library not found")

        self.library_path = str(path)
        self._lib = ctypes.CDLL(self.library_path)
        self._handle = ctypes.c_void_p()
        self._callback_user = None
        self._event_callback = None
        self._on_event = on_event
        self._has_pose_correction = False
        self._pose_translation_correction_m = [0.0, 0.0, 0.0]
        self._next_frame_id = 0

        self._bind()
        native_opts = MlcOptions()
        self._lib.mlc_options_default(ctypes.byref(native_opts))
        for key, value in (options or {}).items():
            if hasattr(native_opts, key):
                setattr(native_opts, key, value)

        status = self._lib.mlc_create(ctypes.byref(native_opts), ctypes.byref(self._handle))
        self._check(status, "mlc_create")
        self._install_callback()
        self._check(self._lib.mlc_initialize(self._handle), "mlc_initialize")

    def close(self) -> None:
        if self._handle:
            self._lib.mlc_destroy(self._handle)
            self._handle = ctypes.c_void_p()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    @property
    def pose_translation_correction_m(self) -> List[float]:
        return list(self._pose_translation_correction_m)

    def set_calibration_yaml(self, yaml_or_path: str) -> bool:
        status = self._lib.mlc_set_calibration_yaml(self._handle, str(yaml_or_path).encode("utf-8"))
        if status != 0:
            return False
        return True

    def push_image(self, image: Dict[str, Any]) -> bool:
        raw = image
        if image.get("kind") == "stereo_raw":
            raw = image.get("left") or {}
        data = bytes(raw.get("data") or b"")
        if not data:
            return False
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        frame_id = self._next_frame_id
        self._next_frame_id += 1
        msg = MlcRawImage(
            int(raw.get("timestamp_ns") or raw.get("timestampNs") or 0),
            frame_id,
            int(raw.get("width") or 0),
            int(raw.get("height") or 0),
            int(raw.get("format") or mp.RAW_FORMAT["UNKNOWN"]),
            ctypes.cast(buf, ctypes.POINTER(ctypes.c_uint8)),
            len(data),
        )
        return self._lib.mlc_push_image(self._handle, ctypes.byref(msg)) == 0

    def push_pose(self, pose: Dict[str, Any]) -> bool:
        pos = pose.get("position_m") or pose.get("positionM") or [0.0, 0.0, 0.0]
        quat_xyzw = pose.get("orientation_xyzw") or pose.get("orientationXyzw") or [0.0, 0.0, 0.0, 1.0]
        frame = 1 if (pose.get("pose_type") or pose.get("poseType")) == "camera" else 0
        msg = MlcPose(
            int(pose.get("timestamp_ns") or pose.get("timestampNs") or 0),
            float(pos[0]),
            float(pos[1]),
            float(pos[2]),
            float(quat_xyzw[3]),
            float(quat_xyzw[0]),
            float(quat_xyzw[1]),
            float(quat_xyzw[2]),
            frame,
            float(pose.get("confidence", 1.0)),
        )
        return self._lib.mlc_push_pose(self._handle, ctypes.byref(msg)) == 0

    def push_keyframe(self, keyframe: Dict[str, Any]) -> bool:
        desc = list(keyframe.get("descriptor") or [])
        if not desc:
            return False
        arr = (ctypes.c_float * len(desc))(*[float(v) for v in desc])
        msg = MlcDeviceKeyframe(
            int(keyframe.get("timestamp_ns") or keyframe.get("timestampNs") or 0),
            int(keyframe.get("frame_id") or keyframe.get("frameId") or 0),
            int(keyframe.get("descriptor_type") or keyframe.get("descriptorType") or 1),
            int(keyframe.get("flags") or 0),
            ctypes.cast(arr, ctypes.POINTER(ctypes.c_float)),
            len(desc),
        )
        return self._lib.mlc_push_keyframe(self._handle, ctypes.byref(msg)) == 0

    def correct_pose(self, pose: Dict[str, Any]) -> Dict[str, Any]:
        if not self._has_pose_correction:
            return pose
        out = dict(pose)
        pos = list(out.get("position_m") or out.get("positionM") or [])
        if len(pos) >= 3:
            corrected = [float(pos[i]) + self._pose_translation_correction_m[i] for i in range(3)]
            if "position_m" in out:
                out["raw_position_m"] = [float(pos[i]) for i in range(3)]
                out["position_m"] = corrected
            else:
                out["rawPositionM"] = [float(pos[i]) for i in range(3)]
                out["positionM"] = corrected
            out["loopclosure_corrected"] = True
            out["loopclosureCorrected"] = True
        return out

    def _bind(self) -> None:
        self._lib.mlc_status_message.argtypes = [ctypes.c_int]
        self._lib.mlc_status_message.restype = ctypes.c_char_p
        self._lib.mlc_options_default.argtypes = [ctypes.POINTER(MlcOptions)]
        self._lib.mlc_options_default.restype = None
        self._lib.mlc_create.argtypes = [ctypes.POINTER(MlcOptions), ctypes.POINTER(ctypes.c_void_p)]
        self._lib.mlc_create.restype = ctypes.c_int
        self._lib.mlc_destroy.argtypes = [ctypes.c_void_p]
        self._lib.mlc_destroy.restype = None
        self._lib.mlc_initialize.argtypes = [ctypes.c_void_p]
        self._lib.mlc_initialize.restype = ctypes.c_int
        self._lib.mlc_set_calibration_yaml.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self._lib.mlc_set_calibration_yaml.restype = ctypes.c_int
        self._lib.mlc_push_image.argtypes = [ctypes.c_void_p, ctypes.POINTER(MlcRawImage)]
        self._lib.mlc_push_image.restype = ctypes.c_int
        self._lib.mlc_push_pose.argtypes = [ctypes.c_void_p, ctypes.POINTER(MlcPose)]
        self._lib.mlc_push_pose.restype = ctypes.c_int
        self._lib.mlc_push_keyframe.argtypes = [ctypes.c_void_p, ctypes.POINTER(MlcDeviceKeyframe)]
        self._lib.mlc_push_keyframe.restype = ctypes.c_int
    def _install_callback(self) -> None:
        callback_type = ctypes.CFUNCTYPE(None, ctypes.POINTER(MlcEvent), ctypes.c_void_p)

        def _callback(event_ptr, _user):
            if not event_ptr:
                return
            evt = self._event_to_dict(event_ptr.contents)
            if self._on_event:
                self._on_event(evt)

        self._event_callback = callback_type(_callback)
        self._lib.mlc_set_event_callback.argtypes = [ctypes.c_void_p, callback_type, ctypes.c_void_p]
        self._lib.mlc_set_event_callback.restype = None
        self._lib.mlc_set_event_callback(self._handle, self._event_callback, None)

    def _event_to_dict(self, event: MlcEvent) -> Dict[str, Any]:
        correction = [float(event.correction_tx), float(event.correction_ty), float(event.correction_tz)]
        self._has_pose_correction = True
        self._pose_translation_correction_m = correction
        return {
            "type": _EVENT_NAMES.get(int(event.type), "unknown"),
            "timestamp_ns": int(event.timestamp_ns),
            "current_keyframe": int(event.current_keyframe),
            "matched_keyframe": int(event.matched_keyframe),
            "pose_translation_correction_m": correction,
        }

    def _check(self, status: int, op: str) -> None:
        if int(status) == 0:
            return
        msg = self._lib.mlc_status_message(int(status))
        text = msg.decode("utf-8", errors="replace") if msg else "unknown"
        raise LoopClosureError(f"{op} failed: {text}")
