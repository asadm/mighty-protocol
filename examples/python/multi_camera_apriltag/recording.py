from __future__ import annotations

import datetime as _dt
import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


def stamp_parts(stamp_ns: int) -> Tuple[int, int]:
    value = max(0, int(stamp_ns))
    return value // 1_000_000_000, value % 1_000_000_000


def default_bag_path(base_dir: Path = Path.home()) -> Path:
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path(base_dir).expanduser() / f"multi_camera_apriltag_{stamp}.bag"


def event_stamp_ns(event: Dict[str, Any]) -> int:
    data = event.get("data")
    if isinstance(data, dict):
        for key in ("timestampNs", "timestamp_ns"):
            value = data.get(key)
            if isinstance(value, int) and value > 0:
                return int(value)
    return time.time_ns()


class MultiCameraRosbagRecorder:
    def __init__(
        self,
        path: Path,
        *,
        red_image_topic: str,
        blue_image_topic: str,
        red_imu_topic: str,
        blue_imu_topic: str,
        red_pose_topic: str,
        blue_pose_topic: str,
        red_tags_topic: str,
        blue_tags_topic: str,
        red_events_topic: str,
        blue_events_topic: str,
        jpeg_quality: int = 90,
    ):
        self.path = Path(path)
        self.topics = {
            "red_image": red_image_topic,
            "blue_image": blue_image_topic,
            "red_imu": red_imu_topic,
            "blue_imu": blue_imu_topic,
            "red_pose": red_pose_topic,
            "blue_pose": blue_pose_topic,
            "red_tags": red_tags_topic,
            "blue_tags": blue_tags_topic,
            "red_events": red_events_topic,
            "blue_events": blue_events_topic,
        }
        self.jpeg_quality = int(max(1, min(100, jpeg_quality)))
        self.lock = threading.RLock()
        self.writer = None
        self.typestore = None
        self.types: Dict[str, Any] = {}
        self.connections: Dict[str, Any] = {}
        self.seq: Dict[str, int] = {}
        self.started_at_s = 0.0
        self.last_error = ""
        self.image_messages = 0
        self.imu_messages = {"red": 0, "blue": 0}
        self.pose_messages = {"red": 0, "blue": 0}
        self.tag_messages = {"red": 0, "blue": 0}
        self.event_messages = {"red": 0, "blue": 0}

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
            "PoseStamped": typestore.types["geometry_msgs/msg/PoseStamped"],
            "Pose": typestore.types["geometry_msgs/msg/Pose"],
            "Point": typestore.types["geometry_msgs/msg/Point"],
            "Quaternion": typestore.types["geometry_msgs/msg/Quaternion"],
            "Vector3": typestore.types["geometry_msgs/msg/Vector3"],
            "String": typestore.types["std_msgs/msg/String"],
        }
        self.connections = {
            "red_image": writer.add_connection(self.topics["red_image"], "sensor_msgs/msg/CompressedImage", typestore=typestore),
            "blue_image": writer.add_connection(self.topics["blue_image"], "sensor_msgs/msg/CompressedImage", typestore=typestore),
            "red_imu": writer.add_connection(self.topics["red_imu"], "sensor_msgs/msg/Imu", typestore=typestore),
            "blue_imu": writer.add_connection(self.topics["blue_imu"], "sensor_msgs/msg/Imu", typestore=typestore),
            "red_pose": writer.add_connection(self.topics["red_pose"], "geometry_msgs/msg/PoseStamped", typestore=typestore),
            "blue_pose": writer.add_connection(self.topics["blue_pose"], "geometry_msgs/msg/PoseStamped", typestore=typestore),
            "red_tags": writer.add_connection(self.topics["red_tags"], "std_msgs/msg/String", typestore=typestore),
            "blue_tags": writer.add_connection(self.topics["blue_tags"], "std_msgs/msg/String", typestore=typestore),
            "red_events": writer.add_connection(self.topics["red_events"], "std_msgs/msg/String", typestore=typestore),
            "blue_events": writer.add_connection(self.topics["blue_events"], "std_msgs/msg/String", typestore=typestore),
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

    def _encode_jpeg(self, rgb: np.ndarray) -> Optional[np.ndarray]:
        try:
            import cv2
        except Exception as exc:
            self.last_error = f"cv2 unavailable: {exc}"
            return None
        if rgb.ndim == 2:
            bgr = rgb
        else:
            bgr = rgb[:, :, ::-1]
        ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.last_error = "JPEG encode failed"
            return None
        return encoded.reshape(-1).astype(np.uint8, copy=False)

    def record_image(self, role: str, rgb: np.ndarray, timestamp_ns: int) -> None:
        if not self.active() or rgb is None:
            return
        stamp_ns = time.time_ns()
        role_key = "red" if role == "red" else "blue"
        jpeg = self._encode_jpeg(rgb)
        if jpeg is None:
            return
        msg = self.types["CompressedImage"](
            self._header(stamp_ns, f"{role_key}_cam", f"{role_key}_image"),
            "jpeg",
            jpeg,
        )
        self._write(f"{role_key}_image", stamp_ns, "sensor_msgs/msg/CompressedImage", msg)
        with self.lock:
            self.image_messages += 1

    def record_imu_samples(self, role: str, samples: Sequence[Dict[str, Any]]) -> None:
        if not self.active() or not samples:
            return
        role_key = "red" if role == "red" else "blue"
        conn_key = f"{role_key}_imu"
        seq_key = conn_key
        frame_id = f"{role_key}_imu"
        imu_type = "sensor_msgs/msg/Imu"
        quat = self.types["Quaternion"](0.0, 0.0, 0.0, 1.0)
        orient_cov = np.full(9, -1.0, dtype=np.float64)
        zero_cov = np.zeros(9, dtype=np.float64)
        count = 0
        base_stamp_ns = time.time_ns()
        for index, sample in enumerate(samples):
            stamp_ns = base_stamp_ns + int(index)
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
            self._write(conn_key, stamp_ns, imu_type, msg)
            count += 1
        if count:
            with self.lock:
                self.imu_messages[role_key] += count

    def record_pose(
        self,
        role: str,
        position_viz: Optional[Sequence[float]],
        quat_viz: Optional[Sequence[float]],
        timestamp_ns: int,
    ) -> None:
        if not self.active() or position_viz is None:
            return
        role_key = "red" if role == "red" else "blue"
        stamp_ns = time.time_ns()
        q = quat_viz if quat_viz is not None and len(quat_viz) == 4 else (0.0, 0.0, 0.0, 1.0)
        pose = self.types["Pose"](
            self.types["Point"](float(position_viz[0]), float(position_viz[1]), float(position_viz[2])),
            self.types["Quaternion"](float(q[0]), float(q[1]), float(q[2]), float(q[3])),
        )
        msg = self.types["PoseStamped"](
            self._header(stamp_ns, f"{role_key}_base_link", f"{role_key}_pose"),
            pose,
        )
        self._write(f"{role_key}_pose", stamp_ns, "geometry_msgs/msg/PoseStamped", msg)
        with self.lock:
            self.pose_messages[role_key] += 1

    def record_tags(self, role: str, tags: Sequence[Dict[str, Any]], timestamp_ns: int) -> None:
        if not self.active():
            return
        role_key = "red" if role == "red" else "blue"
        device_timestamp_ns = int(timestamp_ns or 0)
        stamp_ns = time.time_ns()
        payload = json.dumps(
            {"timestamp_ns": stamp_ns, "device_timestamp_ns": device_timestamp_ns, "tags": list(tags)},
            separators=(",", ":"),
        )
        msg = self.types["String"](payload)
        self._write(f"{role_key}_tags", stamp_ns, "std_msgs/msg/String", msg)
        with self.lock:
            self.tag_messages[role_key] += 1

    def record_event(self, role: str, event: Dict[str, Any]) -> None:
        if not self.active():
            return
        role_key = "red" if role == "red" else "blue"
        device_event_timestamp_ns = event_stamp_ns(event)
        stamp_ns = time.time_ns()
        event = {**event, "device_event_timestamp_ns": device_event_timestamp_ns}
        payload = json.dumps(event, separators=(",", ":"), default=str)
        msg = self.types["String"](payload)
        self._write(f"{role_key}_events", stamp_ns, "std_msgs/msg/String", msg)
        with self.lock:
            self.event_messages[role_key] += 1

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "active": self.writer is not None,
                "path": str(self.path),
                "image_messages": self.image_messages,
                "red_imu_messages": self.imu_messages["red"],
                "blue_imu_messages": self.imu_messages["blue"],
                "red_pose_messages": self.pose_messages["red"],
                "blue_pose_messages": self.pose_messages["blue"],
                "red_tag_messages": self.tag_messages["red"],
                "blue_tag_messages": self.tag_messages["blue"],
                "red_event_messages": self.event_messages["red"],
                "blue_event_messages": self.event_messages["blue"],
                "last_error": self.last_error,
            }


class RecorderController:
    def __init__(
        self,
        *,
        record_dir: Path,
        record_out: str,
        red_image_topic: str,
        blue_image_topic: str,
        red_imu_topic: str,
        blue_imu_topic: str,
        red_pose_topic: str,
        blue_pose_topic: str,
        red_tags_topic: str,
        blue_tags_topic: str,
        red_events_topic: str,
        blue_events_topic: str,
        jpeg_quality: int,
    ):
        self.record_dir = Path(record_dir).expanduser()
        self.record_out = str(record_out or "").strip()
        self.kwargs = {
            "red_image_topic": red_image_topic,
            "blue_image_topic": blue_image_topic,
            "red_imu_topic": red_imu_topic,
            "blue_imu_topic": blue_imu_topic,
            "red_pose_topic": red_pose_topic,
            "blue_pose_topic": blue_pose_topic,
            "red_tags_topic": red_tags_topic,
            "blue_tags_topic": blue_tags_topic,
            "red_events_topic": red_events_topic,
            "blue_events_topic": blue_events_topic,
            "jpeg_quality": int(jpeg_quality),
        }
        self.lock = threading.RLock()
        self.current: Optional[MultiCameraRosbagRecorder] = None
        self.last_path = ""
        self.last_error = ""
        self.last_stats: Dict[str, Any] = {"active": False, "path": "", "last_error": ""}

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
            recorder = MultiCameraRosbagRecorder(self._next_path(), **self.kwargs)
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

    def record_image(self, role: str, rgb: np.ndarray, timestamp_ns: int) -> None:
        with self.lock:
            recorder = self.current
        if recorder is not None:
            recorder.record_image(role, rgb, timestamp_ns)

    def record_imu_samples(self, role: str, samples: Sequence[Dict[str, Any]]) -> None:
        with self.lock:
            recorder = self.current
        if recorder is not None:
            recorder.record_imu_samples(role, samples)

    def record_pose(
        self,
        role: str,
        position_viz: Optional[Sequence[float]],
        quat_viz: Optional[Sequence[float]],
        timestamp_ns: int,
    ) -> None:
        with self.lock:
            recorder = self.current
        if recorder is not None:
            recorder.record_pose(role, position_viz, quat_viz, timestamp_ns)

    def record_tags(self, role: str, tags: Sequence[Dict[str, Any]], timestamp_ns: int) -> None:
        with self.lock:
            recorder = self.current
        if recorder is not None:
            recorder.record_tags(role, tags, timestamp_ns)

    def record_event(self, role: str, event: Dict[str, Any]) -> None:
        with self.lock:
            recorder = self.current
        if recorder is not None:
            recorder.record_event(role, event)
