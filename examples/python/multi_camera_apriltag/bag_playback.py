from __future__ import annotations

import json
import time
from pathlib import Path
from threading import Event
from typing import Any, Dict, Optional

import numpy as np

from state import CameraPanelState


def stamp_to_ns(stamp: Any, fallback_ns: int) -> int:
    sec = int(getattr(stamp, "sec", 0) or 0)
    nanosec = int(getattr(stamp, "nanosec", getattr(stamp, "nsec", 0)) or 0)
    value = sec * 1_000_000_000 + nanosec
    return value if value > 0 else int(fallback_ns)


def decode_compressed_image(cv2: Any, data: Any) -> Optional[np.ndarray]:
    arr = np.asarray(data, dtype=np.uint8).reshape(-1)
    if arr.size == 0:
        return None
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return bgr[:, :, ::-1]


def imu_sample_from_msg(msg: Any, stamp_ns: int) -> Dict[str, float]:
    angular_velocity = msg.angular_velocity
    linear_acceleration = msg.linear_acceleration
    return {
        "timestamp_ns": int(stamp_ns),
        "ax": float(linear_acceleration.x),
        "ay": float(linear_acceleration.y),
        "az": float(linear_acceleration.z),
        "gx": float(angular_velocity.x),
        "gy": float(angular_velocity.y),
        "gz": float(angular_velocity.z),
    }


def pose_from_msg(msg: Any) -> Dict[str, Any]:
    pose = msg.pose
    position = pose.position
    orientation = pose.orientation
    return {
        "position": (float(position.x), float(position.y), float(position.z)),
        "orientation": (float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)),
    }


def run_bag_playback(
    *,
    bag_path: Path,
    red_state: CameraPanelState,
    blue_state: CameraPanelState,
    stop_event: Event,
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
    playback_rate: float = 1.0,
    loop: bool = False,
) -> None:
    try:
        import cv2
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except Exception as exc:
        message = f"bag playback dependencies missing: {exc}"
        red_state.set_error(message)
        blue_state.set_error(message)
        return

    path = Path(bag_path).expanduser()
    if not path.exists():
        message = f"bag not found: {path}"
        red_state.set_error(message)
        blue_state.set_error(message)
        return

    topic_roles = {
        str(red_image_topic): ("red_image", red_state),
        str(blue_image_topic): ("blue_image", blue_state),
        str(red_imu_topic): ("red_imu", red_state),
        str(blue_imu_topic): ("blue_imu", blue_state),
        str(red_pose_topic): ("red_pose", red_state),
        str(blue_pose_topic): ("blue_pose", blue_state),
        str(red_tags_topic): ("red_tags", red_state),
        str(blue_tags_topic): ("blue_tags", blue_state),
        str(red_events_topic): ("red_events", red_state),
        str(blue_events_topic): ("blue_events", blue_state),
    }
    typestore = get_typestore(Stores.ROS1_NOETIC)
    for state in (red_state, blue_state):
        state.set_connection("playing bag", path.as_posix())
        state.update_status("bag playback")

    rate = max(0.0, float(playback_rate))
    while not stop_event.is_set():
        first_bag_ns: Optional[int] = None
        started_s = time.monotonic()
        seen_any = False

        try:
            with Reader(path) as reader:
                connections = [conn for conn in reader.connections if conn.topic in topic_roles]
                if not connections:
                    raise RuntimeError(f"bag contains none of the configured topics: {', '.join(topic_roles)}")

                for conn, bag_ns, raw in reader.messages(connections=connections):
                    if stop_event.is_set():
                        break
                    seen_any = True
                    if first_bag_ns is None:
                        first_bag_ns = int(bag_ns)
                    if rate > 0.0:
                        target_s = float(int(bag_ns) - int(first_bag_ns)) / 1e9 / rate
                        while not stop_event.is_set():
                            remaining_s = target_s - (time.monotonic() - started_s)
                            if remaining_s <= 0.0:
                                break
                            stop_event.wait(min(remaining_s, 0.05))
                    if stop_event.is_set():
                        break

                    role, state = topic_roles[conn.topic]
                    msg = typestore.deserialize_ros1(raw, conn.msgtype)
                    header = getattr(msg, "header", None)
                    stamp_ns = stamp_to_ns(getattr(header, "stamp", None), int(bag_ns))

                    if role.endswith("_image"):
                        image = decode_compressed_image(cv2, msg.data)
                        if image is None:
                            state.set_error(f"failed to decode {conn.topic}")
                            continue
                        state.update_image(image, role, stamp_ns)
                    elif role.endswith("_imu"):
                        state.update_imu([imu_sample_from_msg(msg, stamp_ns)])
                    elif role.endswith("_pose"):
                        pose = pose_from_msg(msg)
                        state.update_pose(pose["position"], pose["orientation"], stamp_ns)
                    elif role.endswith("_tags"):
                        try:
                            payload = json.loads(str(msg.data or "{}"))
                            tags = payload.get("tags", [])
                            tag_stamp = int(payload.get("timestamp_ns") or stamp_ns)
                            state.update_tags(tags, tag_stamp)
                        except Exception as exc:
                            state.set_error(f"failed to decode {conn.topic}: {exc}")
                    elif role.endswith("_events"):
                        try:
                            payload = json.loads(str(msg.data or "{}"))
                            if isinstance(payload, dict):
                                state.update_event(payload)
                        except Exception as exc:
                            state.set_error(f"failed to decode {conn.topic}: {exc}")

        except Exception as exc:
            message = f"bag playback failed: {exc}"
            red_state.set_error(message)
            blue_state.set_error(message)
            break

        if not loop or stop_event.is_set() or not seen_any:
            break

    for state in (red_state, blue_state):
        state.set_connection("bag done", path.as_posix())
