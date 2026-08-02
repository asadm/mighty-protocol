import math
from typing import Any, Callable, Dict, Optional

import mighty_protocol as mp


def _timestamp_ns(frame: Dict[str, Any]) -> int:
    try:
        return int(frame.get("timestamp_ns", 0) or 0)
    except (TypeError, ValueError):
        return 0


def _canonical_channel(channel: str) -> str:
    value = str(channel or "").strip().lower()
    if value in ("preview", "left"):
        return "cam0"
    if value == "right":
        return "cam1"
    return value


def depth_at_meters(frame: Dict[str, Any], x: int, y: int) -> Optional[float]:
    width = int(frame.get("width", 0) or 0)
    height = int(frame.get("height", 0) or 0)
    px = int(x)
    py = int(y)
    values = frame.get("depth_mm")
    if values is None or px < 0 or py < 0 or px >= width or py >= height:
        return None
    sample = int(values[py * width + px])
    if sample == int(frame.get("invalid_value", 0)):
        return None
    scale = float(frame.get("depth_scale_m", 0.001))
    return sample * scale if math.isfinite(scale) and scale > 0.0 else None


def _raw_to_rgba(image: Dict[str, Any]):
    width = int(image.get("width", 0) or 0)
    height = int(image.get("height", 0) or 0)
    data = bytes(image.get("data", b""))
    fmt = int(image.get("format", 0) or 0)
    pixels = width * height
    if width <= 0 or height <= 0:
        return None
    rgba = bytearray(pixels * 4)
    if fmt == mp.RAW_FORMAT["GRAY8"] or fmt in (mp.RAW_FORMAT["YUV420SP"], mp.RAW_FORMAT["YUV420P"]):
        if len(data) < pixels:
            return None
        for i in range(pixels):
            value = data[i]
            rgba[i * 4:i * 4 + 4] = bytes((value, value, value, 255))
    elif fmt in (mp.RAW_FORMAT["RGB24"], mp.RAW_FORMAT["BGR24"]):
        if len(data) < pixels * 3:
            return None
        bgr = fmt == mp.RAW_FORMAT["BGR24"]
        for i in range(pixels):
            source = i * 3
            red, green, blue = data[source:source + 3]
            if bgr:
                red, blue = blue, red
            rgba[i * 4:i * 4 + 4] = bytes((red, green, blue, 255))
    elif fmt in (mp.RAW_FORMAT["RGBA32"], mp.RAW_FORMAT["BGRA32"]):
        if len(data) < pixels * 4:
            return None
        bgra = fmt == mp.RAW_FORMAT["BGRA32"]
        for i in range(pixels):
            source = i * 4
            red, green, blue, alpha = data[source:source + 4]
            if bgra:
                red, blue = blue, red
            rgba[i * 4:i * 4 + 4] = bytes((red, green, blue, alpha))
    else:
        return None
    return {"width": width, "height": height, "rgba": rgba}


def _distort_normalized(x: float, y: float, frame: Dict[str, Any]):
    coeffs = list(frame.get("distortion", (0.0, 0.0, 0.0, 0.0)))[:4]
    coeffs += [0.0] * (4 - len(coeffs))
    camera_model = int(frame.get("camera_model", mp.DEPTH_CAMERA_MODEL["PINHOLE"]))
    distortion_model = int(frame.get("distortion_model", mp.DEPTH_DISTORTION_MODEL["NONE"]))
    if camera_model == mp.DEPTH_CAMERA_MODEL["DOUBLE_SPHERE"]:
        xi, alpha = coeffs[:2]
        radius2 = x * x + y * y
        if radius2 < 1e-16:
            return x, y
        d1 = math.sqrt(radius2 + 1.0)
        shifted = xi * d1 + 1.0
        d2 = math.sqrt(radius2 + shifted * shifted)
        denominator = alpha * d2 + (1.0 - alpha) * shifted
        return (x / denominator, y / denominator) if abs(denominator) > 1e-12 else (math.nan, math.nan)
    if distortion_model == mp.DEPTH_DISTORTION_MODEL["NONE"]:
        return x, y
    if distortion_model == mp.DEPTH_DISTORTION_MODEL["EQUIDISTANT"]:
        k1, k2, k3, k4 = coeffs
        radius = math.hypot(x, y)
        if radius < 1e-8:
            return x, y
        theta = math.atan(radius)
        theta2 = theta * theta
        theta_d = theta * (1.0 + k1 * theta2 + k2 * theta2 ** 2 + k3 * theta2 ** 3 + k4 * theta2 ** 4)
        scale = theta_d / radius
        return x * scale, y * scale
    k1, k2, p1, p2 = coeffs
    x2, y2 = x * x, y * y
    xy = x * y
    radius2 = x2 + y2
    radial = 1.0 + k1 * radius2 + k2 * radius2 * radius2
    return (
        x * radial + 2.0 * p1 * xy + p2 * (radius2 + 2.0 * x2),
        y * radial + p1 * (radius2 + 2.0 * y2) + 2.0 * p2 * xy,
    )


def rectify_image_to_depth(image: Dict[str, Any], depth: Dict[str, Any], require_matching_timestamp: bool = True):
    image_ts = _timestamp_ns(image)
    depth_ts = _timestamp_ns(depth)
    if require_matching_timestamp and image_ts > 0 and depth_ts > 0 and image_ts != depth_ts:
        raise ValueError("source image and depth timestamps do not match")
    decoded = _raw_to_rgba(image)
    if decoded is None:
        raise ValueError("source image format cannot be decoded to RGBA")
    width = int(depth.get("width", 0) or 0)
    height = int(depth.get("height", 0) or 0)
    depth_k = tuple(float(v) for v in depth.get("depth_intrinsics", ()))
    source_k = tuple(float(v) for v in depth.get("source_intrinsics", ()))
    if width <= 0 or height <= 0 or len(depth_k) != 4 or len(source_k) != 4 or \
            depth_k[0] <= 0.0 or depth_k[1] <= 0.0 or source_k[0] <= 0.0 or source_k[1] <= 0.0:
        raise ValueError("depth frame has invalid camera geometry")
    calibration_width = int(depth.get("source_width", decoded["width"]) or decoded["width"])
    calibration_height = int(depth.get("source_height", decoded["height"]) or decoded["height"])
    scale_x = decoded["width"] / calibration_width
    scale_y = decoded["height"] / calibration_height
    sfx, sfy = source_k[0] * scale_x, source_k[1] * scale_y
    scx, scy = source_k[2] * scale_x, source_k[3] * scale_y
    source_rgba = decoded["rgba"]
    output = bytearray(width * height * 4)
    for py in range(height):
        for px in range(width):
            ray_x = (px - depth_k[2]) / depth_k[0]
            ray_y = (py - depth_k[3]) / depth_k[1]
            distorted_x, distorted_y = _distort_normalized(ray_x, ray_y, depth)
            sample_x = sfx * distorted_x + scx
            sample_y = sfy * distorted_y + scy
            destination = (py * width + px) * 4
            if not math.isfinite(sample_x) or not math.isfinite(sample_y) or \
                    sample_x < 0.0 or sample_y < 0.0 or \
                    sample_x > decoded["width"] - 1 or sample_y > decoded["height"] - 1:
                output[destination + 3] = 255
                continue
            x0, y0 = int(math.floor(sample_x)), int(math.floor(sample_y))
            x1, y1 = min(decoded["width"] - 1, x0 + 1), min(decoded["height"] - 1, y0 + 1)
            ax, ay = sample_x - x0, sample_y - y0
            indexes = [
                (y0 * decoded["width"] + x0) * 4,
                (y0 * decoded["width"] + x1) * 4,
                (y1 * decoded["width"] + x0) * 4,
                (y1 * decoded["width"] + x1) * 4,
            ]
            for channel in range(4):
                top = source_rgba[indexes[0] + channel] + (source_rgba[indexes[1] + channel] - source_rgba[indexes[0] + channel]) * ax
                bottom = source_rgba[indexes[2] + channel] + (source_rgba[indexes[3] + channel] - source_rgba[indexes[2] + channel]) * ax
                output[destination + channel] = round(top + (bottom - top) * ay)
    return {
        "width": width,
        "height": height,
        "rgba": output,
        "timestamp_ns": depth_ts,
        "frame_id": depth.get("frame_id", "cam0_rectified"),
    }


class RgbdSynchronizer:
    def __init__(self, on_pair: Callable[[Dict[str, Any]], None], max_entries: int = 64):
        if not callable(on_pair):
            raise ValueError("RgbdSynchronizer requires an on_pair callback")
        self.on_pair = on_pair
        self.max_entries = max(2, int(max_entries))
        self.images: Dict[str, Dict[str, Any]] = {}
        self.depth: Dict[str, Dict[str, Any]] = {}

    def _key(self, channel: str, timestamp_ns: int) -> str:
        return f"{_canonical_channel(channel)}:{int(timestamp_ns)}"

    def _trim(self, cache: Dict[str, Dict[str, Any]]) -> None:
        while len(cache) > self.max_entries:
            cache.pop(next(iter(cache)))

    def push_image(self, image: Dict[str, Any]) -> None:
        if image and image.get("kind") == "stereo_raw":
            self.push_image(image.get("left"))
            self.push_image(image.get("right"))
            return
        if not image:
            return
        timestamp_ns = _timestamp_ns(image)
        if timestamp_ns <= 0:
            return
        key = self._key(image.get("channel_alias") or image.get("channel", ""), timestamp_ns)
        depth = self.depth.pop(key, None)
        if depth is not None:
            self.on_pair({"image": image, "depth": depth})
            return
        self.images[key] = image
        self._trim(self.images)

    def push_depth(self, depth: Dict[str, Any]) -> None:
        if not depth:
            return
        timestamp_ns = _timestamp_ns(depth)
        if timestamp_ns <= 0:
            return
        key = self._key(depth.get("source_channel", ""), timestamp_ns)
        image = self.images.pop(key, None)
        if image is not None:
            self.on_pair({"image": image, "depth": depth})
            return
        self.depth[key] = depth
        self._trim(self.depth)

    def clear(self) -> None:
        self.images.clear()
        self.depth.clear()
