from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import mighty_protocol as mp

from utils import decode_raw_for_opencv, median_int, parse_clock_payload


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


def wire_client_callbacks(
    client: MightyClient,
    state: CameraState,
    refiner: ImagePairRefiner,
    *,
    role: str,
    recorder: Optional[RecorderController] = None,
    left_state: Optional[CameraState] = None,
    right_state: Optional[CameraState] = None,
) -> None:
    def on_image(image: Dict[str, Any]) -> None:
        frame = select_preview_frame(image)
        if not frame:
            return
        image = decode_raw_for_opencv(frame, mp.RAW_FORMAT)
        if image is None:
            return
        channel = str(frame.get("channel_alias") or frame.get("channel") or "cam0")
        raw_ns = int(frame.get("timestamp_ns") or 0)
        state.update_image(image, channel, raw_ns)
        refiner.observe()
        if recorder is not None and left_state is not None and right_state is not None:
            recorder.record_pair_if_ready(left_state, right_state, refiner)

    def on_imu(imu: Dict[str, Any]) -> None:
        samples = imu.get("samples", [])
        state.update_imu(samples)
        if recorder is not None:
            recorder.record_imu_samples(role, state, samples)

    def on_vio_state(vio_state: Dict[str, Any]) -> None:
        state.update_vio_state(vio_state)

    def on_status(status: Dict[str, Any]) -> None:
        state.update_status(str(status.get("text", "") or ""))

    def on_error(error: Dict[str, Any]) -> None:
        message = f"{error.get('scope', 'unknown')}:{error.get('code', 'unknown')} {error.get('message', '')}"
        state.set_error(message.strip())

    client.on_image(on_image)
    client.on_imu(on_imu)
    client.on_vio_state(on_vio_state)
    client.on_status(on_status)
    client.on_error(on_error)


def sync_device_clock(
    client: MightyClient,
    state: CameraState,
    samples: int,
    sample_delay_s: float,
) -> bool:
    observations: List[Tuple[int, int]] = []
    errors: List[str] = []
    count = max(1, int(samples))

    for _ in range(count):
        start_ns = time.monotonic_ns()
        result = client.command("clock")
        end_ns = time.monotonic_ns()
        if not result.get("ok"):
            errors.append(str(result.get("message", "clock command failed")))
            time.sleep(max(0.0, float(sample_delay_s)))
            continue

        payload = parse_clock_payload(result.get("data", b"") or b"")
        device_ns = payload.get("steady_ns")
        if device_ns is None or device_ns <= 0:
            errors.append("clock response missing steady_ns")
            time.sleep(max(0.0, float(sample_delay_s)))
            continue

        rtt_ns = max(0, end_ns - start_ns)
        host_mid_ns = start_ns + rtt_ns // 2
        offset_ns = host_mid_ns - int(device_ns)
        observations.append((rtt_ns, offset_ns))
        time.sleep(max(0.0, float(sample_delay_s)))

    if not observations:
        err = "; ".join(errors[-3:]) or "no valid clock samples"
        state.set_clock_alignment(None, None, 0, "clock_failed", err)
        return False

    observations.sort(key=lambda item: item[0])
    best = observations[: max(1, min(3, len(observations)))]
    offset_ns = median_int([offset for _, offset in best])
    rtt_ns = median_int([rtt for rtt, _ in best])
    state.set_clock_alignment(offset_ns, rtt_ns, len(observations), "clock")
    return True


def run_camera_lifecycle(
    state: CameraState,
    client: MightyClient,
    device: MightyWebDevice,
    stop_event: threading.Event,
    clock_samples: int,
    clock_sample_delay_s: float,
    resync_interval_s: float,
    start_vio: bool,
) -> None:
    state.set_connection("syncing clock", state.base_url)
    sync_device_clock(client, state, clock_samples, clock_sample_delay_s)

    if start_vio:
        result = client.start_vio()
        if result.get("ok"):
            state.update_status("start_vio sent")
        else:
            state.set_error(f"start_vio failed: {result.get('message', 'unknown')}")

    state.set_connection("connecting", state.base_url)
    client.connect()
    next_resync_s = time.monotonic() + max(0.0, float(resync_interval_s))

    while not stop_event.is_set():
        source = str(device.get_info().get("source", "") or state.base_url)
        connected = bool(client.is_connected())
        state.set_connection("connected" if connected else "disconnected", source)
        if resync_interval_s > 0.0 and time.monotonic() >= next_resync_s:
            sync_device_clock(client, state, clock_samples, clock_sample_delay_s)
            next_resync_s = time.monotonic() + float(resync_interval_s)
        stop_event.wait(0.2)
