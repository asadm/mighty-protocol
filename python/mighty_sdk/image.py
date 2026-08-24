from io import BytesIO
from typing import Any, Dict, Optional

import mighty_protocol as mp

from .utils import to_bytes


def select_primary_image(
    image: Optional[Dict[str, Any]],
    include_reference: bool = False,
) -> Optional[Dict[str, Any]]:
    """Return the mono/primary member of a Mighty image event."""
    if not image:
        return None
    kind = image.get("kind")
    if kind == "jpg" and image.get("is_reference") and not include_reference:
        return None
    if kind in ("raw", "jpg"):
        return image
    if kind != "stereo_raw":
        return None
    left = image.get("left") or {}
    right = image.get("right") or {}
    for candidate in (left, right):
        channel = str(
            candidate.get("channel_alias") or candidate.get("channel") or ""
        ).strip().lower()
        if channel in ("cam0", "preview", "left"):
            return candidate
    return left or right or None


def decode_jpeg_to_raw(
    image: Dict[str, Any],
    output_format: str = "gray8",
) -> Dict[str, Any]:
    """Decode a ``kind='jpg'`` event to a normal raw-image mapping.

    Pillow is imported lazily so receiving or forwarding compressed images does
    not add a mandatory image-library dependency to the transport SDK.
    """
    if not image or image.get("kind") != "jpg":
        raise ValueError("decode_jpeg_to_raw requires a kind='jpg' image")
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError(
            "JPEG pixel decoding requires Pillow; install mighty-protocol[images]"
        ) from exc

    requested = str(output_format or "gray8").strip().lower()
    modes = {
        "gray8": ("L", mp.RAW_FORMAT["GRAY8"]),
        "rgb24": ("RGB", mp.RAW_FORMAT["RGB24"]),
        "rgba32": ("RGBA", mp.RAW_FORMAT["RGBA32"]),
    }
    if requested not in modes:
        raise ValueError("output_format must be gray8, rgb24, or rgba32")
    pillow_mode, raw_format = modes[requested]

    encoded = to_bytes(image.get("data", b""))
    if not encoded:
        raise ValueError("JPEG image data is empty")
    with Image.open(BytesIO(encoded)) as source:
        decoded = source.convert(pillow_mode)
        width, height = decoded.size
        data = decoded.tobytes()

    return {
        "kind": "raw",
        "timestamp_ns": int(image.get("timestamp_ns") or 0),
        "width": int(width),
        "height": int(height),
        "format": raw_format,
        "channel": str(image.get("channel") or "preview"),
        "channel_alias": image.get("channel_alias"),
        "data": data,
    }


def image_to_raw(
    image: Optional[Dict[str, Any]],
    jpeg_output_format: str = "gray8",
    include_reference: bool = False,
) -> Optional[Dict[str, Any]]:
    """Select the primary image and decode it when it is JPEG-compressed."""
    selected = select_primary_image(image, include_reference=include_reference)
    if not selected:
        return None
    if selected.get("kind") == "jpg":
        return decode_jpeg_to_raw(selected, jpeg_output_format)
    return selected
