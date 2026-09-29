"""WPILib field layouts and full AprilTag poses, with no WPILib dependency."""
import json
import math

FRC_TAG_SIZE_M = 0.1651


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite number".format(label))
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError("{} must be a finite number".format(label)) from exc
    if not math.isfinite(result):
        raise ValueError("{} must be a finite number".format(label))
    return result


def _quaternion(values):
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError("orientation_xyzw must contain four numbers")
    q = [_number(v, "quaternion component") for v in values]
    scale = max(abs(v) for v in q)
    if scale == 0:
        raise ValueError("quaternion must be nonzero")
    q = [v / scale for v in q]
    norm = math.sqrt(sum(v * v for v in q))
    return [v / norm for v in q]


def _tag(tag, seen):
    tag_id = tag.get("tag_id")
    if isinstance(tag_id, bool) or not isinstance(tag_id, (int, float)) or not 0 <= tag_id <= 2147483647 or int(tag_id) != tag_id:
        raise ValueError("tag_id must be a non-negative 32-bit integer")
    if tag_id in seen:
        raise ValueError("duplicate tag ID {}".format(tag_id))
    seen.add(tag_id)
    size = _number(tag.get("size_m"), "size_m")
    if size <= 0:
        raise ValueError("size_m must be positive")
    position = tag.get("position_m")
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ValueError("position_m must contain three numbers")
    return {"tag_id": int(tag_id), "size_m": size,
            "position_m": [_number(v, "position") for v in position],
            "orientation_xyzw": _quaternion(tag.get("orientation_xyzw"))}


def parse_wpilib_field_layout(source, tag_size_m=FRC_TAG_SIZE_M):
    """Read JSON text/bytes or a dict. Returned rotations use WPILib tag axes.

    Keep the file's field origin and units (meters). WPILib JSON does not
    include tag size; pass the measured black-square size for custom layouts.
    """
    if isinstance(source, (str, bytes, bytearray)):
        source = json.loads(source)
    if not isinstance(source, dict) or not isinstance(source.get("tags"), list) or not source["tags"]:
        raise ValueError("WPILib layout must contain a nonempty tags array")
    size = _number(tag_size_m, "tag_size_m")
    if size <= 0:
        raise ValueError("tag_size_m must be positive")
    tags, seen = [], set()
    for row in source["tags"]:
        try:
            p = row["pose"]["translation"]
            q = row["pose"]["rotation"]["quaternion"]
            tag = {"tag_id": row["ID"], "size_m": size,
                   "position_m": [p[axis] for axis in "xyz"],
                   "orientation_xyzw": [q[axis] for axis in "XYZW"]}
        except (KeyError, TypeError) as exc:
            raise ValueError("each tag must contain ID, translation and quaternion") from exc
        tags.append(_tag(tag, seen))
    field = source.get("field", {})
    if not isinstance(field, dict):
        raise ValueError("field must be an object")
    dimensions = {}
    for key in ("length", "width"):
        if key in field:
            dimensions[key] = _number(field[key], "field." + key)
            if dimensions[key] <= 0:
                raise ValueError("field dimensions must be positive")
    return {"tags": tags, "field": dimensions}


def serialize_apriltag_map_yaml(tag_map):
    """Encode full WPILib-frame tag poses for Mighty config_set('apriltags')."""
    if not isinstance(tag_map, dict) or not isinstance(tag_map.get("tags"), list) or not tag_map["tags"]:
        raise ValueError("tag map must contain a nonempty tags array")
    lines, seen = ["%YAML:1.0", "---", "apriltags:"], set()
    for row in tag_map["tags"]:
        if not isinstance(row, dict):
            raise ValueError("each tag must be an object")
        tag = _tag(row, seen)
        x, y, z, w = tag["orientation_xyzw"]
        # q_field_wpilibTag * q_wpilibTag_pnpTag. PnP right/down/normal
        # equals WPILib -Y/-Z/+X; field coordinates stay unchanged.
        native = [(-w + x - y - z) / 2, (w + x + y - z) / 2,
                  (-w + x + y + z) / 2, (w + x - y + z) / 2]
        fmt = lambda v: format(v, ".17g")
        lines += ["  - tag_id: {}".format(tag["tag_id"]),
                  "    size_m: " + fmt(tag["size_m"]),
                  "    position_m: [" + ", ".join(map(fmt, tag["position_m"])) + "]",
                  # This marker makes legacy firmware reject full-pose maps.
                  "    orientation: quaternion",
                  "    orientation_xyzw: [" + ", ".join(map(fmt, native)) + "]"]
    return "\n".join(lines) + "\n"
