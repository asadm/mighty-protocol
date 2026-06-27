from __future__ import annotations

import datetime as _dt
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from state import CameraState, ImagePairRefiner, select_synced_image_pair


def stamp_parts(stamp_ns: int) -> Tuple[int, int]:
    value = max(0, int(stamp_ns))
    return value // 1_000_000_000, value % 1_000_000_000


def default_bag_path(base_dir: Path = Path.home()) -> Path:
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path(base_dir).expanduser() / f"synced_stereo_{stamp}.bag"


class RosbagRecorder:
    def __init__(
        self,
        path: Path,
        *,
        left_image_topic: str,
        right_image_topic: str,
        left_imu_topic: str,
        right_imu_topic: str,
        jpeg_quality: int = 90,
        pair_max_delta_ms: float = 8.0,
        require_refined_pairs: bool = True,
    ):
        self.path = Path(path)
        self.left_image_topic = left_image_topic
        self.right_image_topic = right_image_topic
        self.left_imu_topic = left_imu_topic
        self.right_imu_topic = right_imu_topic
        self.jpeg_quality = int(max(1, min(100, jpeg_quality)))
        self.pair_max_delta_ms = float(pair_max_delta_ms)
        self.require_refined_pairs = bool(require_refined_pairs)

        self.lock = threading.RLock()
        self.writer = None
        self.typestore = None
        self.types: Dict[str, Any] = {}
        self.connections: Dict[str, Any] = {}
        self.seq: Dict[str, int] = {}
        self.recorded_pairs: set[Tuple[int, int]] = set()
        self.started_at_s = 0.0
        self.last_error = ""
        self.image_pairs = 0
        self.image_messages = 0
        self.left_imu_messages = 0
        self.right_imu_messages = 0

    def start(self) -> None:
        try:
            from rosbags.rosbag1 import Writer
            from rosbags.typesys import Stores, get_typestore
        except Exception as exc:
            raise RuntimeError("rosbags is required for recording. Install with: pip install rosbags") from exc

        self.path.parent.mkdir(parents=True, exist_ok=True)
        typestore = get_typestore(Stores.ROS1_NOETIC)
        writer = Writer(self.path)
        writer.open()

        self.typestore = typestore
        self.types = {
            "Header": typestore.types["std_msgs/msg/Header"],
            "Time": typestore.types["builtin_interfaces/msg/Time"],
            "CompressedImage": typestore.types["sensor_msgs/msg/CompressedImage"],
            "Imu": typestore.types["sensor_msgs/msg/Imu"],
            "Quaternion": typestore.types["geometry_msgs/msg/Quaternion"],
            "Vector3": typestore.types["geometry_msgs/msg/Vector3"],
        }
        self.connections = {
            "left_image": writer.add_connection(
                self.left_image_topic,
                "sensor_msgs/msg/CompressedImage",
                typestore=typestore,
            ),
            "right_image": writer.add_connection(
                self.right_image_topic,
                "sensor_msgs/msg/CompressedImage",
                typestore=typestore,
            ),
            "left_imu": writer.add_connection(
                self.left_imu_topic,
                "sensor_msgs/msg/Imu",
                typestore=typestore,
            ),
            "right_imu": writer.add_connection(
                self.right_imu_topic,
                "sensor_msgs/msg/Imu",
                typestore=typestore,
            ),
        }
        self.writer = writer
        self.started_at_s = time.monotonic()

    def close(self) -> None:
        with self.lock:
            writer = self.writer
            self.writer = None
        if writer is not None:
            writer.close()

    def active(self) -> bool:
        with self.lock:
            return self.writer is not None

    def _next_seq(self, key: str) -> int:
        seq = int(self.seq.get(key, 0))
        self.seq[key] = seq + 1
        return seq

    def _header(self, stamp_ns: int, frame_id: str, seq_key: str) -> Any:
        sec, nanosec = stamp_parts(stamp_ns)
        return self.types["Header"](self._next_seq(seq_key), self.types["Time"](sec, nanosec), frame_id)

    def _write(self, connection_key: str, stamp_ns: int, typename: str, msg: Any) -> None:
        typestore = self.typestore
        with self.lock:
            writer = self.writer
            conn = self.connections.get(connection_key)
            if writer is None or conn is None or typestore is None:
                return
            raw = typestore.serialize_ros1(msg, typename)
            writer.write(conn, int(stamp_ns), raw)

    def _encode_jpeg(self, image: np.ndarray) -> Optional[np.ndarray]:
        try:
            import cv2
        except Exception as exc:
            self.last_error = f"cv2 unavailable: {exc}"
            return None
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.last_error = "JPEG encode failed"
            return None
        return encoded.reshape(-1).astype(np.uint8, copy=False)

    def record_pair_if_ready(
        self,
        left_state: CameraState,
        right_state: CameraState,
        refiner: ImagePairRefiner,
    ) -> None:
        if not self.active():
            return
        refiner_stats = refiner.snapshot()
        if self.require_refined_pairs and refiner_stats.get("enabled") and int(refiner_stats.get("pairs") or 0) < refiner.min_pairs:
            return

        pair = select_synced_image_pair(
            left_state.frame_buffer_snapshot(copy_image=False),
            right_state.frame_buffer_snapshot(copy_image=False),
            max_delta_ms=self.pair_max_delta_ms,
        )
        if pair is None:
            return

        left = pair["left"]
        right = pair["right"]
        left_raw = int(left.get("raw_ns") or 0)
        right_raw = int(right.get("raw_ns") or 0)
        left_synced = left.get("synced_ns")
        right_synced = right.get("synced_ns")
        if left_raw <= 0 or right_raw <= 0 or left_synced is None or right_synced is None:
            return

        key = (left_raw, right_raw)
        with self.lock:
            if key in self.recorded_pairs:
                return
            self.recorded_pairs.add(key)

        left_jpeg = self._encode_jpeg(left["image"])
        right_jpeg = self._encode_jpeg(right["image"])
        if left_jpeg is None or right_jpeg is None:
            return

        common_stamp_ns = int(round((int(left_synced) + int(right_synced)) * 0.5))
        image_type = "sensor_msgs/msg/CompressedImage"
        left_msg = self.types["CompressedImage"](
            self._header(common_stamp_ns, "cam0", "left_image"),
            "jpeg",
            left_jpeg,
        )
        right_msg = self.types["CompressedImage"](
            self._header(common_stamp_ns, "cam1", "right_image"),
            "jpeg",
            right_jpeg,
        )
        self._write("left_image", common_stamp_ns, image_type, left_msg)
        self._write("right_image", common_stamp_ns, image_type, right_msg)
        with self.lock:
            self.image_pairs += 1
            self.image_messages += 2

    def record_imu_samples(self, role: str, state: CameraState, samples: Sequence[Dict[str, Any]]) -> None:
        if not self.active() or not samples:
            return

        is_left = role == "left"
        conn_key = "left_imu" if is_left else "right_imu"
        frame_id = "imu0" if is_left else "imu1"
        seq_key = conn_key
        imu_type = "sensor_msgs/msg/Imu"
        quat = self.types["Quaternion"](0.0, 0.0, 0.0, 1.0)
        orient_cov = np.full(9, -1.0, dtype=np.float64)
        zero_cov = np.zeros(9, dtype=np.float64)

        count = 0
        for sample in samples:
            raw_ts = sample.get("timestamp_ns")
            if not isinstance(raw_ts, int) or raw_ts <= 0:
                continue
            stamp_ns = state.synced_ns_for(raw_ts)
            if stamp_ns is None:
                continue
            angular_velocity = self.types["Vector3"](
                float(sample.get("gx", 0.0)),
                float(sample.get("gy", 0.0)),
                float(sample.get("gz", 0.0)),
            )
            linear_acceleration = self.types["Vector3"](
                float(sample.get("ax", 0.0)),
                float(sample.get("ay", 0.0)),
                float(sample.get("az", 0.0)),
            )
            msg = self.types["Imu"](
                self._header(stamp_ns, frame_id, seq_key),
                quat,
                orient_cov,
                angular_velocity,
                zero_cov,
                linear_acceleration,
                zero_cov,
            )
            self._write(conn_key, int(stamp_ns), imu_type, msg)
            count += 1

        if count:
            with self.lock:
                if is_left:
                    self.left_imu_messages += count
                else:
                    self.right_imu_messages += count

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "active": self.writer is not None,
                "path": str(self.path),
                "image_pairs": self.image_pairs,
                "image_messages": self.image_messages,
                "left_imu_messages": self.left_imu_messages,
                "right_imu_messages": self.right_imu_messages,
                "last_error": self.last_error,
            }


class RecorderController:
    def __init__(
        self,
        *,
        record_dir: Path,
        record_out: str,
        left_image_topic: str,
        right_image_topic: str,
        left_imu_topic: str,
        right_imu_topic: str,
        jpeg_quality: int,
        pair_max_delta_ms: float,
        require_refined_pairs: bool,
    ):
        self.record_dir = Path(record_dir).expanduser()
        self.record_out = str(record_out or "").strip()
        self.left_image_topic = left_image_topic
        self.right_image_topic = right_image_topic
        self.left_imu_topic = left_imu_topic
        self.right_imu_topic = right_imu_topic
        self.jpeg_quality = int(jpeg_quality)
        self.pair_max_delta_ms = float(pair_max_delta_ms)
        self.require_refined_pairs = bool(require_refined_pairs)

        self.lock = threading.RLock()
        self.current: Optional[RosbagRecorder] = None
        self.last_path = ""
        self.last_error = ""
        self.last_stats: Dict[str, Any] = {
            "active": False,
            "path": "",
            "image_pairs": 0,
            "image_messages": 0,
            "left_imu_messages": 0,
            "right_imu_messages": 0,
            "last_error": "",
        }

    def _next_path(self) -> Path:
        if self.record_out:
            path = Path(self.record_out).expanduser()
            if not path.is_absolute():
                path = Path.cwd() / path
            if path.exists():
                stem = path.stem
                suffix = path.suffix or ".bag"
                stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
                path = path.with_name(f"{stem}_{stamp}{suffix}")
            return path
        return default_bag_path(self.record_dir)

    def start(self) -> Tuple[bool, str]:
        with self.lock:
            if self.current is not None and self.current.active():
                return True, self.current.path.as_posix()
            path = self._next_path()
            recorder = RosbagRecorder(
                path,
                left_image_topic=self.left_image_topic,
                right_image_topic=self.right_image_topic,
                left_imu_topic=self.left_imu_topic,
                right_imu_topic=self.right_imu_topic,
                jpeg_quality=self.jpeg_quality,
                pair_max_delta_ms=self.pair_max_delta_ms,
                require_refined_pairs=self.require_refined_pairs,
            )
            try:
                recorder.start()
            except Exception as exc:
                self.current = None
                self.last_error = str(exc)
                self.last_stats = {**self.last_stats, "active": False, "last_error": self.last_error}
                return False, self.last_error
            self.current = recorder
            self.last_path = recorder.path.as_posix()
            self.last_error = ""
            return True, self.last_path

    def stop(self) -> Tuple[bool, str]:
        with self.lock:
            recorder = self.current
            self.current = None
        if recorder is None:
            return True, self.last_path
        try:
            recorder.close()
            stats = recorder.snapshot()
            self.last_stats = {**stats, "active": False}
            self.last_path = recorder.path.as_posix()
            self.last_error = ""
            return True, self.last_path
        except Exception as exc:
            self.last_error = str(exc)
            self.last_stats = {**self.last_stats, "active": False, "last_error": self.last_error}
            return False, self.last_error

    def close(self) -> None:
        self.stop()

    def active(self) -> bool:
        with self.lock:
            return self.current is not None and self.current.active()

    def record_pair_if_ready(
        self,
        left_state: CameraState,
        right_state: CameraState,
        refiner: ImagePairRefiner,
    ) -> None:
        with self.lock:
            recorder = self.current
        if recorder is not None:
            recorder.record_pair_if_ready(left_state, right_state, refiner)

    def record_imu_samples(self, role: str, state: CameraState, samples: Sequence[Dict[str, Any]]) -> None:
        with self.lock:
            recorder = self.current
        if recorder is not None:
            recorder.record_imu_samples(role, state, samples)

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            recorder = self.current
            if recorder is not None:
                stats = recorder.snapshot()
                self.last_stats = stats
                self.last_path = str(stats.get("path") or self.last_path)
                self.last_error = str(stats.get("last_error") or "")
                return stats
            return {
                **self.last_stats,
                "active": False,
                "path": self.last_path,
                "last_error": self.last_error or self.last_stats.get("last_error", ""),
            }
