import math
import re
from typing import Any, Dict, List, Optional, Tuple


_NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


def _canonical_camera_model(raw: Optional[str]) -> str:
    token = re.sub(r"[-_\s]", "", str(raw or "").strip(" \t\r\n'\"").lower())
    if token in ("ds", "doublesphere"):
        return "double_sphere"
    if token == "pinhole":
        return "pinhole"
    return "unknown"


def _canonical_distortion_model(raw: Optional[str]) -> str:
    if raw is None or not str(raw).strip():
        return "unknown"
    token = re.sub(r"[-_\s]", "", str(raw or "").strip(" \t\r\n'\"").lower())
    if token in ("none", "off", "disabled"):
        return "none"
    if token in ("radtan", "radialtangential"):
        return "radtan"
    if token in ("equidistant", "fisheye"):
        return "equidistant"
    return "unknown"


def _find_key(text: str, key: str) -> Optional[int]:
    match = re.search(r"^[ \t]*" + re.escape(key) + r"[ \t]*:", text, re.MULTILINE)
    return None if match is None else match.end()


def _scalar_text(text: str, key: str) -> Optional[str]:
    start = _find_key(text, key)
    if start is None:
        return None
    newline = text.find("\n", start)
    value = text[start : len(text) if newline < 0 else newline].strip()
    if "#" in value:
        value = value.split("#", 1)[0].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        value = value[1:-1]
    return value or None


def _scalar_number(text: str, key: str) -> Optional[float]:
    raw = _scalar_text(text, key)
    if raw is None:
        return None
    try:
        value = float(raw)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _matching_bracket(text: str, opening: int) -> int:
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "[":
            depth += 1
        elif text[index] == "]":
            depth -= 1
            if depth == 0:
                return index
    return -1


def _numbers(text: str) -> List[float]:
    values = [float(match.group(0)) for match in _NUMBER_RE.finditer(text)]
    if any(not math.isfinite(value) for value in values):
        raise ValueError("calibration contains a non-finite number")
    return values


def _bracket_numbers(text: str, key: str, adjacent_row_target: int = 0) -> Optional[List[float]]:
    value_start = _find_key(text, key)
    if value_start is None:
        return None
    opening = text.find("[", value_start)
    if opening < 0 or opening - value_start > 512:
        return None
    prefix = text[value_start:opening]
    first_prefix_line = prefix.split("\n", 1)[0]
    if "!!opencv-matrix" not in first_prefix_line and re.fullmatch(r"[\s-]*", prefix) is None:
        return None

    values: List[float] = []
    while opening >= 0:
        closing = _matching_bracket(text, opening)
        if closing < 0:
            return None
        values.extend(_numbers(text[opening + 1 : closing]))
        if adjacent_row_target <= 0 or len(values) >= adjacent_row_target:
            return values
        next_opening = text.find("[", closing + 1)
        if (
            next_opening < 0
            or next_opening - closing > 128
            or re.fullmatch(r"[\s-]*", text[closing + 1 : next_opening]) is None
        ):
            return None
        opening = next_opening
    return None


def _determinant3(matrix: List[float]) -> float:
    return (
        matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9])
        - matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8])
        + matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8])
    )


def _is_rigid_matrix(matrix: Optional[List[float]]) -> bool:
    if matrix is None or len(matrix) != 16 or any(not math.isfinite(value) for value in matrix):
        return False
    if (
        abs(matrix[12]) > 1e-6
        or abs(matrix[13]) > 1e-6
        or abs(matrix[14]) > 1e-6
        or abs(matrix[15] - 1.0) > 1e-6
        or abs(_determinant3(matrix) - 1.0) > 0.1
    ):
        return False
    error_squared = 0.0
    for row in range(3):
        for column in range(3):
            dot = sum(matrix[row * 4 + index] * matrix[column * 4 + index] for index in range(3))
            error = dot - (1.0 if row == column else 0.0)
            error_squared += error * error
    return math.sqrt(error_squared) <= 0.1


def _transform(matrix: Optional[List[float]] = None) -> Dict[str, Any]:
    return {
        "matrix": list(matrix) if matrix is not None else [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ],
        "valid": matrix is not None,
    }


def _inverse_rigid(matrix: List[float]) -> Dict[str, Any]:
    if not _is_rigid_matrix(matrix):
        return _transform()
    output = _transform()["matrix"]
    for row in range(3):
        for column in range(3):
            output[row * 4 + column] = matrix[column * 4 + row]
        output[row * 4 + 3] = -sum(output[row * 4 + index] * matrix[index * 4 + 3] for index in range(3))
    return _transform(output)


def _derive_body_from_camera(camera_from_imu: List[float]) -> Dict[str, Any]:
    if not _is_rigid_matrix(camera_from_imu):
        return _transform()
    return _transform([
        0.0, 0.0, 1.0, -camera_from_imu[11],
        -1.0, 0.0, 0.0, camera_from_imu[3],
        0.0, -1.0, 0.0, camera_from_imu[7],
        0.0, 0.0, 0.0, 1.0,
    ])


def _matrix_for_key(text: str, key: str) -> Optional[List[float]]:
    values = _bracket_numbers(text, key, 16)
    return values if values is not None and len(values) == 16 and _is_rigid_matrix(values) else None


def _positive_int(value: Optional[float]) -> int:
    return int(round(value)) if value is not None and math.isfinite(value) and value > 0 else 0


def _parse_camera(text: str, camera_id: str) -> Dict[str, Any]:
    camera_model = _canonical_camera_model(_scalar_text(text, "camera_model"))
    distortion_model = _canonical_distortion_model(_scalar_text(text, "distortion_model"))
    intrinsics_values = _bracket_numbers(text, "intrinsics")
    intrinsics = {"fx": 0.0, "fy": 0.0, "cx": 0.0, "cy": 0.0}
    projection_parameters: List[float] = []
    if intrinsics_values is not None and len(intrinsics_values) >= 4:
        intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"] = intrinsics_values[-4:]
        projection_parameters = intrinsics_values[:-4]
    else:
        intrinsics = {
            "fx": _scalar_number(text, "fx") or 0.0,
            "fy": _scalar_number(text, "fy") or 0.0,
            "cx": _scalar_number(text, "cx") or 0.0,
            "cy": _scalar_number(text, "cy") or 0.0,
        }
        xi = _scalar_number(text, "xi")
        alpha = _scalar_number(text, "alpha")
        if camera_model == "double_sphere" and xi is not None and alpha is not None:
            projection_parameters = [xi, alpha]

    distortion_coefficients = _bracket_numbers(text, "distortion_coeffs") or []
    if not distortion_coefficients and distortion_model != "none" and camera_model != "double_sphere":
        distortion_coefficients = [
            value for value in (_scalar_number(text, key) for key in ("k1", "k2", "p1", "p2"))
            if value is not None
        ]

    resolution_values = _bracket_numbers(text, "resolution")
    resolution = (
        {"width": _positive_int(resolution_values[0]), "height": _positive_int(resolution_values[1])}
        if resolution_values is not None and len(resolution_values) >= 2
        else {
            "width": _positive_int(_scalar_number(text, "resolution_width")),
            "height": _positive_int(_scalar_number(text, "resolution_height")),
        }
    )

    camera_from_imu = _transform()
    body_from_camera = _transform()
    t_camera_imu = _matrix_for_key(text, "T_cam_imu")
    t_imu_camera = _matrix_for_key(text, "T_imu_cam")
    t_camera_body = _matrix_for_key(text, "T_cam_body")
    if t_camera_imu is not None:
        camera_from_imu = _transform(t_camera_imu)
        body_from_camera = _derive_body_from_camera(t_camera_imu)
    elif t_imu_camera is not None:
        camera_from_imu = _inverse_rigid(t_imu_camera)
        if camera_from_imu["valid"]:
            body_from_camera = _derive_body_from_camera(camera_from_imu["matrix"])
    elif t_camera_body is not None:
        body_from_camera = _inverse_rigid(t_camera_body)

    time_shift = _scalar_number(text, "timeshift_cam_imu")
    if time_shift is None:
        time_shift = _scalar_number(text, "td")
    valid = (
        math.isfinite(intrinsics["fx"])
        and intrinsics["fx"] > 0
        and math.isfinite(intrinsics["fy"])
        and intrinsics["fy"] > 0
        and math.isfinite(intrinsics["cx"])
        and math.isfinite(intrinsics["cy"])
    )
    return {
        "id": camera_id,
        "camera_model": camera_model,
        "distortion_model": distortion_model,
        "intrinsics": intrinsics,
        "projection_parameters": projection_parameters,
        "distortion_coefficients": distortion_coefficients,
        "resolution": resolution,
        "topic": _scalar_text(text, "rostopic") or "",
        "time_shift_camera_imu_s": time_shift or 0.0,
        "camera_from_imu": camera_from_imu,
        "body_from_camera": body_from_camera,
        "valid": valid,
    }


def _camera_sections(yaml: str) -> List[Tuple[str, str]]:
    matches = list(re.finditer(r"^(cam\d+):\s*$", yaml, re.MULTILINE))
    return [
        (
            match.group(1),
            yaml[match.start() : matches[index + 1].start() if index + 1 < len(matches) else len(yaml)],
        )
        for index, match in enumerate(matches)
    ]


def parse_calibration_yaml(yaml: str) -> Dict[str, Any]:
    """Normalize Mighty OpenCV or Kalibr calibration YAML into the SDK schema."""
    if not isinstance(yaml, str) or not yaml.strip():
        raise ValueError("calibration YAML is empty")
    sections = _camera_sections(yaml)
    source_format = "kalibr" if sections else "opencv"
    if not sections:
        sections = [("cam0", yaml)]
    cameras = [camera for camera in (_parse_camera(text, camera_id) for camera_id, text in sections) if camera["valid"]]
    if not cameras:
        raise ValueError("calibration has no camera with valid intrinsics")
    return {"source_format": source_format, "cameras": cameras}
