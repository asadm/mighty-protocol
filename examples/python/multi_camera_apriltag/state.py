from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np


STATE_LABELS = {
    0: "OFF",
    1: "INITIALIZING",
    2: "TRACKING",
    3: "DEGRADED",
    4: "LOST",
    5: "LOW_LIGHT",
}

R_VIZ_FROM_ODOM = (
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
)
Q_VIZ_FROM_ODOM = (0.5, -0.5, 0.5, 0.5)  # xyzw from R_VIZ_FROM_ODOM


def map_position_odom_to_viz(position: Optional[Sequence[float]]) -> Optional[Tuple[float, float, float]]:
    if position is None or len(position) < 3:
        return None
    x = float(position[0])
    y = float(position[1])
    z = float(position[2])
    return (
        R_VIZ_FROM_ODOM[0][0] * x + R_VIZ_FROM_ODOM[0][1] * y + R_VIZ_FROM_ODOM[0][2] * z,
        R_VIZ_FROM_ODOM[1][0] * x + R_VIZ_FROM_ODOM[1][1] * y + R_VIZ_FROM_ODOM[1][2] * z,
        R_VIZ_FROM_ODOM[2][0] * x + R_VIZ_FROM_ODOM[2][1] * y + R_VIZ_FROM_ODOM[2][2] * z,
    )


def quat_multiply_xyzw(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = [float(v) for v in a]
    bx, by, bz, bw = [float(v) for v in b]
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def map_quat_odom_to_viz(quat: Optional[Sequence[float]]) -> Optional[Tuple[float, float, float, float]]:
    if quat is None or len(quat) != 4:
        return None
    q = quat_multiply_xyzw(Q_VIZ_FROM_ODOM, quat)
    n = math.sqrt(sum(float(v) * float(v) for v in q))
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(v) / n for v in q)


def quat_xyzw_to_rotmat(quat: Optional[Sequence[float]]) -> np.ndarray:
    if quat is None or len(quat) != 4:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = [float(v) for v in quat]
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def set_axis_limits_from_points(ax: Any, points: np.ndarray) -> float:
    if points.size == 0:
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(-1.0, 1.0)
        return 1.0
    mn = points.min(axis=0)
    mx = points.max(axis=0)
    center = (mn + mx) * 0.5
    radius = float(np.max(mx - mn) * 0.5)
    radius = max(radius, 0.5)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    return radius


def event_summary(event: Dict[str, Any]) -> str:
    kind = str(event.get("kind") or "")
    data = event.get("data")
    if kind == "apriltag_relocalize" and isinstance(data, dict):
        tag_id = data.get("tagId", "?")
        correction = data.get("correctionM")
        if isinstance(correction, (int, float)):
            return f"tag #{tag_id} detected, relocalizing ({float(correction):.3f} m)"
        return f"tag #{tag_id} detected, relocalizing"
    if isinstance(data, dict):
        compact = json.dumps(data, separators=(",", ":"))
        return f"{kind}: {compact[:96]}"
    text = str(event.get("json") or "")
    return f"{kind}: {text[:96]}" if text else kind


@dataclass
class CameraPanelState:
    name: str
    base_url: str
    color: str
    lock: threading.RLock = field(default_factory=threading.RLock)

    connection_state: str = "disconnected"
    connection_source: str = ""
    last_error: str = ""
    status_text: str = "N/A"
    host_version: str = "Unknown"
    vio_state_text: str = "STATE_NA"
    fps_text: str = "FPS: NA"
    pose_conf_text: str = "Pose Conf: NA"

    rx_events: int = 0
    rx_images: int = 0
    rx_poses: int = 0
    rx_imu_samples: int = 0
    rx_tags: int = 0
    last_data_time_s: float = 0.0

    image_rgb: Optional[np.ndarray] = None
    image_channel: str = ""
    image_timestamp_ns: int = 0

    pose_path: Deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=6000))
    pose_latest: Optional[np.ndarray] = None
    pose_quat_latest: Optional[np.ndarray] = None
    pose_timestamp_ns: int = 0

    tags: List[Dict[str, Any]] = field(default_factory=list)
    tag_timestamp_ns: int = 0
    last_relocalized_tag_id: Optional[int] = None
    last_relocalized_at_s: float = 0.0
    events: Deque[Dict[str, Any]] = field(default_factory=lambda: deque(maxlen=80))

    def set_connection(self, state: str, source: str = "") -> None:
        with self.lock:
            self.connection_state = state
            self.connection_source = source

    def set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = str(message)

    def update_status(self, text: str) -> None:
        value = str(text or "").strip()
        with self.lock:
            if value.startswith("HOST_VERSION:"):
                self.host_version = value.replace("HOST_VERSION:", "").strip() or "Unknown"
            elif value:
                self.status_text = value
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_vio_state(self, state: Dict[str, Any]) -> None:
        code = state.get("state")
        try:
            code_int = int(code)
        except Exception:
            code_int = None
        with self.lock:
            self.vio_state_text = STATE_LABELS.get(code_int, f"STATE_{code_int}")
            if state.get("build_version"):
                self.host_version = str(state.get("build_version"))
            fps_cur = state.get("fps_current")
            fps_avg = state.get("fps_average")
            if isinstance(fps_cur, (int, float)) and isinstance(fps_avg, (int, float)):
                self.fps_text = f"FPS: {float(fps_cur):.1f} avg {float(fps_avg):.1f}"
            conf = state.get("pose_confidence")
            if isinstance(conf, (int, float)):
                self.pose_conf_text = f"Pose Conf: {float(conf):.3f}"
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_image(self, rgb: np.ndarray, channel: str, timestamp_ns: int) -> None:
        with self.lock:
            self.image_rgb = rgb
            self.image_channel = str(channel or "cam0")
            self.image_timestamp_ns = int(timestamp_ns or 0)
            self.rx_images += 1
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_pose(
        self,
        position_viz: Optional[Sequence[float]],
        quat_viz: Optional[Sequence[float]],
        timestamp_ns: int = 0,
    ) -> None:
        if position_viz is None or len(position_viz) < 3:
            return
        pos = np.array([float(position_viz[0]), float(position_viz[1]), float(position_viz[2])], dtype=np.float64)
        quat = None
        if quat_viz is not None and len(quat_viz) == 4:
            quat = np.array([float(quat_viz[0]), float(quat_viz[1]), float(quat_viz[2]), float(quat_viz[3])], dtype=np.float64)
        with self.lock:
            self.pose_latest = pos
            self.pose_quat_latest = quat
            self.pose_timestamp_ns = int(timestamp_ns or 0)
            self.pose_path.append(pos)
            self.rx_poses += 1
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def clear_trajectory(self) -> None:
        with self.lock:
            self.pose_path.clear()
            self.pose_latest = None
            self.pose_quat_latest = None
            self.pose_timestamp_ns = 0

    def update_imu(self, samples: Sequence[Dict[str, Any]]) -> None:
        if not samples:
            return
        with self.lock:
            self.rx_imu_samples += len(samples)
            self.rx_events += len(samples)
            self.last_data_time_s = time.time()

    def update_tags(self, tags: Sequence[Dict[str, Any]], timestamp_ns: int = 0) -> None:
        normalized: List[Dict[str, Any]] = []
        for tag in tags:
            corners = []
            for corner in tag.get("corners", []) or []:
                if len(corner) >= 2:
                    corners.append((float(corner[0]), float(corner[1])))
            center = tag.get("center", (0.0, 0.0))
            if len(center) >= 2:
                center_xy = (float(center[0]), float(center[1]))
            elif corners:
                arr = np.asarray(corners, dtype=np.float64)
                center_xy = (float(arr[:, 0].mean()), float(arr[:, 1].mean()))
            else:
                center_xy = (0.0, 0.0)
            normalized.append(
                {
                    "id": int(tag.get("id", 0)),
                    "hamming": int(tag.get("hamming", 0)),
                    "center": center_xy,
                    "corners": corners,
                }
            )
        with self.lock:
            self.tags = normalized
            self.tag_timestamp_ns = int(timestamp_ns or 0)
            self.rx_tags += len(normalized)
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_event(self, event: Dict[str, Any]) -> None:
        now_s = time.time()
        entry = {
            "time_s": now_s,
            "kind": str(event.get("kind") or ""),
            "json": str(event.get("json") or ""),
            "data": event.get("data"),
        }
        with self.lock:
            if entry["kind"] == "apriltag_relocalize" and isinstance(entry["data"], dict):
                tag_id = entry["data"].get("tagId")
                if isinstance(tag_id, int):
                    self.last_relocalized_tag_id = int(tag_id)
                    self.last_relocalized_at_s = now_s
            self.events.appendleft(entry)
            self.rx_events += 1
            self.last_data_time_s = now_s

    def snapshot(self, copy_image: bool = True) -> Dict[str, Any]:
        with self.lock:
            return {
                "name": self.name,
                "base_url": self.base_url,
                "color": self.color,
                "connection_state": self.connection_state,
                "connection_source": self.connection_source,
                "last_error": self.last_error,
                "status_text": self.status_text,
                "host_version": self.host_version,
                "vio_state_text": self.vio_state_text,
                "fps_text": self.fps_text,
                "pose_conf_text": self.pose_conf_text,
                "rx_events": self.rx_events,
                "rx_images": self.rx_images,
                "rx_poses": self.rx_poses,
                "rx_imu_samples": self.rx_imu_samples,
                "rx_tags": self.rx_tags,
                "last_data_time_s": self.last_data_time_s,
                "image_rgb": None if self.image_rgb is None else self.image_rgb.copy() if copy_image else self.image_rgb,
                "image_channel": self.image_channel,
                "image_timestamp_ns": self.image_timestamp_ns,
                "pose_latest": None if self.pose_latest is None else self.pose_latest.copy(),
                "pose_quat_latest": None if self.pose_quat_latest is None else self.pose_quat_latest.copy(),
                "pose_timestamp_ns": self.pose_timestamp_ns,
                "pose_path": [p.copy() for p in self.pose_path],
                "tags": [dict(tag) for tag in self.tags],
                "tag_timestamp_ns": self.tag_timestamp_ns,
                "last_relocalized_tag_id": self.last_relocalized_tag_id,
                "last_relocalized_at_s": self.last_relocalized_at_s,
                "events": [dict(event) for event in self.events],
            }
