from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from utils import median_int


NsTuple = Tuple[float, float, float, float, float, float, float, int, Optional[int]]


@dataclass
class CameraState:
    name: str
    base_url: str
    imu_maxlen: int = 8000
    image_maxlen: int = 90
    lock: threading.RLock = field(default_factory=threading.RLock)

    connection_state: str = "disconnected"
    connection_source: str = ""
    last_error: str = ""
    status_text: str = "N/A"
    vio_state_text: str = "STATE_NA"
    fps_text: str = "FPS: NA"
    imu_hz_text: str = "IMU Hz: NA"

    clock_offset_ns: Optional[int] = None
    clock_rtt_ns: Optional[int] = None
    clock_samples: int = 0
    clock_source: str = "pending"
    clock_updated_at_s: float = 0.0
    alignment_epoch: int = 0
    frame_correction_ns: int = 0
    frame_correction_samples: int = 0

    rx_events: int = 0
    rx_images: int = 0
    rx_imu_samples: int = 0
    last_data_time_s: float = 0.0

    image_rgb: Optional[np.ndarray] = None
    image_channel: str = ""
    image_raw_ns: int = 0
    image_synced_ns: Optional[int] = None
    image_host_arrival_ns: int = 0
    image_frames: Deque[Dict[str, Any]] = field(default_factory=deque)

    imu_samples: Deque[NsTuple] = field(default_factory=deque)
    imu_raw_ns: int = 0
    imu_synced_ns: Optional[int] = None

    def __post_init__(self) -> None:
        self.imu_samples = deque(maxlen=max(1, int(self.imu_maxlen)))
        self.image_frames = deque(maxlen=max(2, int(self.image_maxlen)))

    def _base_offset_unlocked(self) -> Optional[int]:
        return self.clock_offset_ns

    def _total_offset_unlocked(self) -> Optional[int]:
        base = self._base_offset_unlocked()
        if base is None:
            return None
        return int(base) + int(self.frame_correction_ns)

    def synced_ns_for(self, raw_ns: Optional[int]) -> Optional[int]:
        if raw_ns is None or int(raw_ns) <= 0:
            return None
        with self.lock:
            total = self._total_offset_unlocked()
            if total is None:
                return None
            return int(raw_ns) + total

    def base_synced_ns_for(self, raw_ns: Optional[int]) -> Optional[int]:
        if raw_ns is None or int(raw_ns) <= 0:
            return None
        with self.lock:
            base = self._base_offset_unlocked()
            if base is None:
                return None
            return int(raw_ns) + int(base)

    def alignment_epoch_value(self) -> int:
        with self.lock:
            return int(self.alignment_epoch)

    def set_connection(self, state: str, source: str = "") -> None:
        with self.lock:
            self.connection_state = state
            self.connection_source = source

    def set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = message

    def set_clock_alignment(
        self,
        offset_ns: Optional[int],
        rtt_ns: Optional[int],
        samples: int,
        source: str,
        error: str = "",
    ) -> None:
        with self.lock:
            if offset_ns is not None:
                old_offset = self.clock_offset_ns
                self.clock_offset_ns = int(offset_ns)
                self.clock_rtt_ns = None if rtt_ns is None else int(rtt_ns)
                self.clock_samples = int(samples)
                self.clock_source = source
                self.clock_updated_at_s = time.monotonic()
                self.last_error = error
                if old_offset != self.clock_offset_ns:
                    self.alignment_epoch += 1
            else:
                self.clock_source = source
                if error:
                    self.last_error = error

    def ensure_image_anchor(self) -> None:
        with self.lock:
            if self.clock_offset_ns is None:
                self.clock_offset_ns = 0
                self.clock_rtt_ns = None
                self.clock_samples = 0
                self.clock_source = "image"
                self.clock_updated_at_s = time.monotonic()
                self.alignment_epoch += 1

    def set_frame_correction(self, correction_ns: int, samples: int) -> None:
        with self.lock:
            self.frame_correction_ns = int(correction_ns)
            self.frame_correction_samples = int(samples)

    def update_image(self, rgb: np.ndarray, channel: str, raw_ns: int) -> None:
        arrival_ns = time.monotonic_ns()
        with self.lock:
            total = self._total_offset_unlocked()
            synced_ns = int(raw_ns) + total if total is not None and int(raw_ns) > 0 else None
            self.image_rgb = rgb
            self.image_channel = channel
            self.image_raw_ns = int(raw_ns)
            self.image_synced_ns = synced_ns
            self.image_host_arrival_ns = arrival_ns
            self.image_frames.append(
                {
                    "image": rgb,
                    "channel": channel,
                    "raw_ns": int(raw_ns),
                    "host_arrival_ns": arrival_ns,
                }
            )
            self.rx_events += 1
            self.rx_images += 1
            self.last_data_time_s = time.time()

    def update_imu(self, samples: Sequence[Dict[str, Any]]) -> None:
        if not samples:
            return
        now_ns = time.monotonic_ns()
        with self.lock:
            total = self._total_offset_unlocked()
            for sample in samples:
                raw_ts = sample.get("timestamp_ns")
                raw_ns = int(raw_ts) if isinstance(raw_ts, int) and raw_ts > 0 else 0
                synced_ns = int(raw_ns) + total if total is not None and raw_ns > 0 else None
                plot_t = float(synced_ns if synced_ns is not None else now_ns) / 1e9
                self.imu_samples.append(
                    (
                        plot_t,
                        float(sample.get("ax", 0.0)),
                        float(sample.get("ay", 0.0)),
                        float(sample.get("az", 0.0)),
                        float(sample.get("gx", 0.0)),
                        float(sample.get("gy", 0.0)),
                        float(sample.get("gz", 0.0)),
                        raw_ns,
                        synced_ns,
                    )
                )
                if raw_ns > 0:
                    self.imu_raw_ns = raw_ns
                    self.imu_synced_ns = synced_ns
            self.rx_events += len(samples)
            self.rx_imu_samples += len(samples)
            self.last_data_time_s = time.time()

    def update_status(self, text: str) -> None:
        text = text.strip()
        with self.lock:
            self.status_text = text or self.status_text
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_vio_state(self, state: Dict[str, Any]) -> None:
        with self.lock:
            code = state.get("state")
            self.vio_state_text = f"STATE_{code}" if code is not None else "STATE_NA"
            fps_cur = state.get("fps_current")
            fps_avg = state.get("fps_average")
            if isinstance(fps_cur, (int, float)) and isinstance(fps_avg, (int, float)):
                self.fps_text = f"FPS: {fps_cur:.1f} avg {fps_avg:.1f}"
            imu_cur = state.get("imu_hz_current")
            imu_avg = state.get("imu_hz_average_5s")
            if isinstance(imu_cur, (int, float)) and isinstance(imu_avg, (int, float)):
                self.imu_hz_text = f"IMU Hz: {imu_cur:.1f} avg5s {imu_avg:.1f}"
            build = state.get("build_version")
            if build:
                self.status_text = f"build {build}"
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def image_meta(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "raw_ns": self.image_raw_ns,
                "host_arrival_ns": self.image_host_arrival_ns,
                "base_synced_ns": self.base_synced_ns_for(self.image_raw_ns),
            }

    def snapshot(self, copy_image: bool = True) -> Dict[str, Any]:
        with self.lock:
            return {
                "name": self.name,
                "base_url": self.base_url,
                "connection_state": self.connection_state,
                "connection_source": self.connection_source,
                "last_error": self.last_error,
                "status_text": self.status_text,
                "vio_state_text": self.vio_state_text,
                "fps_text": self.fps_text,
                "imu_hz_text": self.imu_hz_text,
                "clock_offset_ns": self.clock_offset_ns,
                "clock_rtt_ns": self.clock_rtt_ns,
                "clock_samples": self.clock_samples,
                "clock_source": self.clock_source,
                "alignment_epoch": self.alignment_epoch,
                "frame_correction_ns": self.frame_correction_ns,
                "frame_correction_samples": self.frame_correction_samples,
                "rx_events": self.rx_events,
                "rx_images": self.rx_images,
                "rx_imu_samples": self.rx_imu_samples,
                "last_data_time_s": self.last_data_time_s,
                "image_rgb": (
                    None
                    if self.image_rgb is None
                    else self.image_rgb.copy()
                    if copy_image
                    else self.image_rgb
                ),
                "image_channel": self.image_channel,
                "image_raw_ns": self.image_raw_ns,
                "image_synced_ns": self.image_synced_ns,
                "imu_samples": list(self.imu_samples),
                "imu_raw_ns": self.imu_raw_ns,
                "imu_synced_ns": self.imu_synced_ns,
            }

    def frame_buffer_snapshot(self, copy_image: bool = False) -> List[Dict[str, Any]]:
        with self.lock:
            total = self._total_offset_unlocked()
            frames: List[Dict[str, Any]] = []
            for frame in self.image_frames:
                raw_ns = int(frame.get("raw_ns") or 0)
                synced_ns = raw_ns + total if total is not None and raw_ns > 0 else None
                image = frame.get("image")
                frames.append(
                    {
                        "image": image.copy() if copy_image and image is not None else image,
                        "channel": frame.get("channel", ""),
                        "raw_ns": raw_ns,
                        "synced_ns": synced_ns,
                        "host_arrival_ns": int(frame.get("host_arrival_ns") or 0),
                    }
                )
            return frames


class ImagePairRefiner:
    def __init__(
        self,
        left: CameraState,
        right: CameraState,
        enabled: bool = True,
        min_pairs: int = 6,
        max_pairs: int = 60,
        arrival_window_ms: float = 50.0,
    ):
        self.left = left
        self.right = right
        self.enabled = bool(enabled)
        self.min_pairs = max(1, int(min_pairs))
        self.arrival_window_ns = int(max(1.0, float(arrival_window_ms)) * 1e6)
        self.lock = threading.Lock()
        self.base_deltas_ns: Deque[int] = deque(maxlen=max(self.min_pairs, int(max_pairs)))
        self.last_pair_key: Tuple[int, int] = (0, 0)
        self.last_epoch_key: Tuple[int, int] = (-1, -1)
        self.last_raw_delta_ns: Optional[int] = None
        self.last_synced_delta_ns: Optional[int] = None

    def observe(self) -> None:
        if not self.enabled:
            return
        left = self.left.image_meta()
        right = self.right.image_meta()
        left_raw = int(left.get("raw_ns") or 0)
        right_raw = int(right.get("raw_ns") or 0)
        if left_raw <= 0 or right_raw <= 0:
            return
        pair_key = (left_raw, right_raw)
        left_arrival = int(left.get("host_arrival_ns") or 0)
        right_arrival = int(right.get("host_arrival_ns") or 0)
        if left_arrival <= 0 or right_arrival <= 0:
            return
        if abs(right_arrival - left_arrival) > self.arrival_window_ns:
            return

        with self.lock:
            if pair_key == self.last_pair_key:
                return
            self.last_pair_key = pair_key

        self.left.ensure_image_anchor()
        self.right.ensure_image_anchor()
        left_epoch = self.left.alignment_epoch_value()
        right_epoch = self.right.alignment_epoch_value()
        left_base = self.left.base_synced_ns_for(left_raw)
        right_base = self.right.base_synced_ns_for(right_raw)
        if left_base is None or right_base is None:
            return

        base_delta = int(right_base) - int(left_base)
        with self.lock:
            epoch_key = (left_epoch, right_epoch)
            if epoch_key != self.last_epoch_key:
                self.base_deltas_ns.clear()
                self.last_epoch_key = epoch_key
                self.last_synced_delta_ns = None
            self.base_deltas_ns.append(base_delta)
            self.last_raw_delta_ns = right_raw - left_raw
            if len(self.base_deltas_ns) >= self.min_pairs:
                correction = -median_int(list(self.base_deltas_ns))
                self.right.set_frame_correction(correction, len(self.base_deltas_ns))
                synced_left = self.left.synced_ns_for(left_raw)
                synced_right = self.right.synced_ns_for(right_raw)
                if synced_left is not None and synced_right is not None:
                    self.last_synced_delta_ns = int(synced_right) - int(synced_left)

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            samples = list(self.base_deltas_ns)
            return {
                "enabled": self.enabled,
                "pairs": len(samples),
                "last_raw_delta_ns": self.last_raw_delta_ns,
                "median_base_delta_ns": median_int(samples) if samples else None,
                "last_synced_delta_ns": self.last_synced_delta_ns,
            }


def select_synced_image_pair(
    left_frames: Sequence[Dict[str, Any]],
    right_frames: Sequence[Dict[str, Any]],
    max_delta_ms: float,
) -> Optional[Dict[str, Any]]:
    max_delta_ns = int(max(0.0, float(max_delta_ms)) * 1e6)
    best: Optional[Dict[str, Any]] = None

    for left in reversed(left_frames):
        left_synced = left.get("synced_ns")
        if left_synced is None:
            continue
        closest_right: Optional[Dict[str, Any]] = None
        closest_delta_ns: Optional[int] = None
        for right in reversed(right_frames):
            right_synced = right.get("synced_ns")
            if right_synced is None:
                continue
            delta_ns = int(right_synced) - int(left_synced)
            abs_delta_ns = abs(delta_ns)
            if closest_delta_ns is None or abs_delta_ns < abs(closest_delta_ns):
                closest_delta_ns = delta_ns
                closest_right = right
        if closest_right is None or closest_delta_ns is None:
            continue
        if abs(closest_delta_ns) <= max_delta_ns:
            best = {
                "left": left,
                "right": closest_right,
                "delta_ns": closest_delta_ns,
                "max_delta_ms": float(max_delta_ms),
            }
            break

    return best
