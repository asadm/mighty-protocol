import json
import math
import os

HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, "pose_viz_cases.json"), "r", encoding="utf-8") as f:
    fixture = json.load(f)

R = (
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
)
Q_VIZ_FROM_ODOM = (0.5, -0.5, 0.5, 0.5)  # xyzw


def mat_vec3(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def q_mul_xyzw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_norm(q):
    n = math.sqrt(float(q[0]) * float(q[0]) + float(q[1]) * float(q[1]) + float(q[2]) * float(q[2]) + float(q[3]) * float(q[3]))
    if not math.isfinite(n) or n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (float(q[0]) / n, float(q[1]) / n, float(q[2]) / n, float(q[3]) / n)


out = {
    "language": "python",
    "cases": [],
}
for c in fixture["cases"]:
    q_in = q_norm(tuple(float(v) for v in c["quat"]))
    q_viz = q_norm(q_mul_xyzw(Q_VIZ_FROM_ODOM, q_in))
    out["cases"].append(
        {
            "name": c["name"],
            "position": list(mat_vec3(R, tuple(float(v) for v in c["position"]))),
            "quat": list(q_viz),
        }
    )

print(json.dumps(out))
