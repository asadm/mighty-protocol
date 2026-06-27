from __future__ import annotations

import time
from pathlib import Path
from threading import Event
from typing import Any, Dict, Optional

import numpy as np

from state import CameraState


def stamp_to_ns(stamp: Any, fallback_ns: int) -> int:
    sec = int(getattr(stamp, "sec", 0) or 0)
    nanosec = int(getattr(stamp, "nanosec", getattr(stamp, "nsec", 0)) or 0)
    value = sec * 1_000_000_000 + nanosec
    return value if value > 0 else int(fallback_ns)


def decode_compressed_image(cv2: Any, data: Any) -> Optional[np.ndarray]:
    arr = np.asarray(data, dtype=np.uint8).reshape(-1)
    if arr.size == 0:
        return None
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


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


def run_bag_playback(
    *,
    bag_path: Path,
    left_state: CameraState,
    right_state: CameraState,
    stop_event: Event,
    left_image_topic: str,
    right_image_topic: str,
    left_imu_topic: str,
    right_imu_topic: str,
    playback_rate: float = 1.0,
    loop: bool = False,
) -> None:
    try:
        import cv2
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except Exception as exc:
        message = f"bag playback dependencies missing: {exc}"
        left_state.set_error(message)
        right_state.set_error(message)
        return

    path = Path(bag_path).expanduser()
    if not path.exists():
        message = f"bag not found: {path}"
        left_state.set_error(message)
        right_state.set_error(message)
        return

    topic_roles = {
        str(left_image_topic): ("left_image", left_state),
        str(right_image_topic): ("right_image", right_state),
        str(left_imu_topic): ("left_imu", left_state),
        str(right_imu_topic): ("right_imu", right_state),
    }
    typestore = get_typestore(Stores.ROS1_NOETIC)

    for state in (left_state, right_state):
        state.set_clock_alignment(0, None, 0, "bag")
        state.set_frame_correction(0, 0)
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
                        channel = "cam0" if role == "left_image" else "cam1"
                        state.update_image(image, channel, stamp_ns)
                    else:
                        state.update_imu([imu_sample_from_msg(msg, stamp_ns)])

        except Exception as exc:
            message = f"bag playback failed: {exc}"
            left_state.set_error(message)
            right_state.set_error(message)
            break

        if not loop or stop_event.is_set():
            break
        if not seen_any:
            break

    for state in (left_state, right_state):
        state.set_connection("bag done", path.as_posix())
