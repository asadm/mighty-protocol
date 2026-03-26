import math
import os
import struct
import sys

HERE = os.path.dirname(__file__)
sys.path.append(os.path.join(HERE, "..", "python"))

import mighty_protocol as mp  # noqa: E402


def almost(a, b, eps=1e-9):
    return abs(float(a) - float(b)) <= eps


def map_position_odom_to_viz(p):
    return (
        0.0 * p[0] + -1.0 * p[1] + 0.0 * p[2],
        0.0 * p[0] + 0.0 * p[1] + 1.0 * p[2],
        -1.0 * p[0] + 0.0 * p[1] + 0.0 * p[2],
    )


def build_manual_pose_payload():
    pose_type = 0
    flags = 0x1 | (1 << 2) | (1 << 3) | (1 << 6)
    position = (4.25, -2.5, 0.75)
    quat = (0.11, -0.22, 0.33, -0.44)  # xyzw
    linvel = (1.0, 2.0, 3.0)
    angvel = (-1.0, -2.0, -3.0)
    ts = 123456789

    payload = b""
    payload += struct.pack(">II", pose_type, flags)
    payload += struct.pack(">ddd", *position)
    payload += struct.pack(">dddd", *quat)
    payload += struct.pack(">f", 1.25)  # clamp to 1.0
    payload += struct.pack(">ddd", *linvel)
    payload += struct.pack(">ddd", *angvel)
    payload += struct.pack(">Q", ts)
    return payload, position, quat, linvel, angvel, ts


def main():
    payload, position, quat, linvel, angvel, ts = build_manual_pose_payload()
    out = mp.decode_pose_payload(payload)

    # Explicit XYZW wire-order regression check.
    assert out["orientation_xyzw"] is not None
    assert almost(out["orientation_xyzw"][0], quat[0])
    assert almost(out["orientation_xyzw"][1], quat[1])
    assert almost(out["orientation_xyzw"][2], quat[2])
    assert almost(out["orientation_xyzw"][3], quat[3])

    assert almost(out["position_m"][0], position[0])
    assert almost(out["position_m"][1], position[1])
    assert almost(out["position_m"][2], position[2])

    assert out["linear_velocity_body_mps"] is not None
    assert out["angular_velocity_body_rps"] is not None
    assert almost(out["linear_velocity_body_mps"][0], linvel[0])
    assert almost(out["angular_velocity_body_rps"][2], angvel[2])
    assert int(out["timestamp_ns"]) == ts
    assert almost(out["confidence"], 1.0)

    # Signed-axis sanity for canonical->viz basis.
    ex = map_position_odom_to_viz((1.0, 0.0, 0.0))
    ey = map_position_odom_to_viz((0.0, 1.0, 0.0))
    ez = map_position_odom_to_viz((0.0, 0.0, 1.0))
    assert tuple(int(round(v)) for v in ex) == (0, 0, -1)
    assert tuple(int(round(v)) for v in ey) == (-1, 0, 0)
    assert tuple(int(round(v)) for v in ez) == (0, 1, 0)

    print("Python pose contract test passed")


if __name__ == "__main__":
    main()
