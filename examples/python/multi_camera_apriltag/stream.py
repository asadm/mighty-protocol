from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

import numpy as np

import mighty_protocol as mp

from state import (
    CameraPanelState,
    map_position_odom_to_viz,
    map_quat_odom_to_viz,
)


def select_preview_frame(image: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    kind = image.get("kind")
    if kind == "raw":
        return image
    if kind != "stereo_raw":
        return None
    left = image.get("left") or {}
    right = image.get("right") or {}
    for frame in (left, right):
        channel = str(frame.get("channel_alias") or frame.get("channel") or "").lower()
        if channel in ("cam0", "preview", "left"):
            return frame
    return left or right or None


def decode_raw_to_rgb(raw: Dict[str, Any]) -> Optional[np.ndarray]:
    width = int(raw.get("width", 0) or 0)
    height = int(raw.get("height", 0) or 0)
    if width <= 0 or height <= 0:
        return None

    data = raw.get("data", b"") or b""
    arr = np.frombuffer(data, dtype=np.uint8)
    fmt = int(raw.get("format", mp.RAW_FORMAT["UNKNOWN"]) or 0)
    need = width * height

    if fmt == mp.RAW_FORMAT["GRAY8"]:
        if arr.size < need:
            return None
        gray = arr[:need].reshape((height, width))
        return np.stack([gray, gray, gray], axis=-1)

    if fmt == mp.RAW_FORMAT["RGB24"]:
        if arr.size < need * 3:
            return None
        return arr[: need * 3].reshape((height, width, 3))

    if fmt == mp.RAW_FORMAT["BGR24"]:
        if arr.size < need * 3:
            return None
        bgr = arr[: need * 3].reshape((height, width, 3))
        return bgr[:, :, ::-1]

    if fmt == mp.RAW_FORMAT["RGBA32"]:
        if arr.size < need * 4:
            return None
        rgba = arr[: need * 4].reshape((height, width, 4))
        return rgba[:, :, :3]

    if fmt == mp.RAW_FORMAT["BGRA32"]:
        if arr.size < need * 4:
            return None
        bgra = arr[: need * 4].reshape((height, width, 4))
        return bgra[:, :, [2, 1, 0]]

    if fmt in (mp.RAW_FORMAT["YUV420SP"], mp.RAW_FORMAT["YUV420P"]):
        if arr.size < need:
            return None
        y = arr[:need].reshape((height, width))
        return np.stack([y, y, y], axis=-1)

    return None


def wire_client_callbacks(
    client: Any,
    state: CameraPanelState,
    *,
    role: str,
    recorder: Optional[Any] = None,
) -> None:
    def on_image(image: Dict[str, Any]) -> None:
        frame = select_preview_frame(image)
        if not frame:
            return
        rgb = decode_raw_to_rgb(frame)
        if rgb is None:
            return
        channel = str(frame.get("channel_alias") or frame.get("channel") or "cam0")
        timestamp_ns = int(frame.get("timestamp_ns") or 0)
        state.update_image(rgb, channel, timestamp_ns)
        if recorder is not None:
            recorder.record_image(role, rgb, timestamp_ns)

    def on_pose(pose: Dict[str, Any]) -> None:
        position = map_position_odom_to_viz(pose.get("position_m"))
        quat = map_quat_odom_to_viz(pose.get("orientation_xyzw"))
        timestamp_ns = int(pose.get("timestamp_ns") or 0)
        state.update_pose(position, quat, timestamp_ns)
        if recorder is not None:
            recorder.record_pose(role, position, quat, timestamp_ns)

    def on_imu(imu: Dict[str, Any]) -> None:
        samples = imu.get("samples", []) or []
        state.update_imu(samples)
        if recorder is not None:
            recorder.record_imu_samples(role, samples)

    def on_viz(viz: Dict[str, Any]) -> None:
        if viz.get("subtype") != "apriltags":
            return
        tags = viz.get("apriltags", viz.get("tags", [])) or []
        state.update_tags(tags)
        if recorder is not None:
            recorder.record_tags(role, tags, state.snapshot(copy_image=False).get("image_timestamp_ns", 0))

    def on_event(event: Dict[str, Any]) -> None:
        state.update_event(event)
        if recorder is not None:
            recorder.record_event(role, event)

    def on_vio_state(vio_state: Dict[str, Any]) -> None:
        state.update_vio_state(vio_state)

    def on_status(status: Dict[str, Any]) -> None:
        state.update_status(str(status.get("text", "") or ""))

    def on_error(error: Dict[str, Any]) -> None:
        message = f"{error.get('scope', 'unknown')}:{error.get('code', 'unknown')} {error.get('message', '')}"
        state.set_error(message.strip())

    client.on_image(on_image)
    client.on_pose(on_pose)
    client.on_imu(on_imu)
    client.on_viz(on_viz)
    client.on_event(on_event)
    client.on_vio_state(on_vio_state)
    client.on_status(on_status)
    client.on_error(on_error)


def run_camera_lifecycle(
    state: CameraPanelState,
    client: Any,
    device: Any,
    stop_event: threading.Event,
    start_vio: bool = False,
) -> None:
    state.set_connection("connecting", state.base_url)
    if start_vio:
        state.clear_trajectory()
        result = client.start_vio()
        if result.get("ok"):
            state.update_status("start_vio sent")
        else:
            state.set_error(f"start_vio failed: {result.get('message', 'unknown')}")

    client.connect()
    while not stop_event.is_set():
        source = str(device.get_info().get("source", "") or state.base_url)
        connected = bool(client.is_connected())
        state.set_connection("connected" if connected else "disconnected", source)
        stop_event.wait(0.2)
    state.set_connection("stopping", state.base_url)
