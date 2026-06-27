from __future__ import annotations

import statistics
import time
from typing import Any, Dict, Optional, Sequence

import numpy as np


def parse_clock_payload(data: bytes) -> Dict[str, int]:
    out: Dict[str, int] = {}
    text = data.decode("utf-8", errors="replace")
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key or not value:
            continue
        try:
            out[key] = int(value)
        except ValueError:
            continue
    return out


def fmt_ns(value: Optional[int]) -> str:
    if value is None or int(value) <= 0:
        return "NA"
    return str(int(value))


def fmt_ms(value_ns: Optional[int]) -> str:
    if value_ns is None:
        return "NA"
    return f"{float(value_ns) / 1e6:.3f} ms"


def fmt_rel_s(value_ns: Optional[int], zero_ns: Optional[int]) -> str:
    if value_ns is None or zero_ns is None:
        return "NA"
    return f"{(float(value_ns) - float(zero_ns)) / 1e9:.6f} s"


def median_int(values: Sequence[int]) -> int:
    if not values:
        return 0
    return int(round(float(statistics.median(values))))


def image_pair_age_ms(pair: Optional[Dict[str, Any]]) -> Optional[float]:
    if pair is None:
        return None
    left_arrival = int((pair.get("left") or {}).get("host_arrival_ns") or 0)
    right_arrival = int((pair.get("right") or {}).get("host_arrival_ns") or 0)
    newest_arrival = max(left_arrival, right_arrival)
    if newest_arrival <= 0:
        return None
    return max(0.0, float(time.monotonic_ns() - newest_arrival) / 1e6)


def decode_raw_for_opencv(raw: Dict[str, Any], raw_format: Dict[str, int]) -> Optional[np.ndarray]:
    width = int(raw.get("width", 0) or 0)
    height = int(raw.get("height", 0) or 0)
    if width <= 0 or height <= 0:
        return None

    data = raw.get("data", b"") or b""
    arr = np.frombuffer(data, dtype=np.uint8)
    fmt = int(raw.get("format", raw_format["UNKNOWN"]) or 0)
    need = width * height

    if fmt == raw_format["GRAY8"]:
        if arr.size < need:
            return None
        return arr[:need].reshape((height, width))

    if fmt == raw_format["RGB24"]:
        if arr.size < need * 3:
            return None
        rgb = arr[: need * 3].reshape((height, width, 3))
        return rgb[:, :, ::-1]

    if fmt == raw_format["BGR24"]:
        if arr.size < need * 3:
            return None
        return arr[: need * 3].reshape((height, width, 3))

    if fmt == raw_format["RGBA32"]:
        if arr.size < need * 4:
            return None
        rgba = arr[: need * 4].reshape((height, width, 4))
        return rgba[:, :, [2, 1, 0]]

    if fmt == raw_format["BGRA32"]:
        if arr.size < need * 4:
            return None
        bgra = arr[: need * 4].reshape((height, width, 4))
        return bgra[:, :, :3]

    if fmt in (raw_format["YUV420SP"], raw_format["YUV420P"]):
        if arr.size < need:
            return None
        return arr[:need].reshape((height, width))

    return None
