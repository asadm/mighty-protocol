import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, ".."))
CPP_BIN = os.path.join(HERE, "bin", "cpp_pose_viz_map")


def run_json(cmd):
    proc = subprocess.run(cmd, cwd=ROOT, check=True, capture_output=True, text=True)
    text = proc.stdout.strip()
    if not text:
        raise RuntimeError(f"empty output from: {' '.join(cmd)}")
    return json.loads(text)


def vec_close(a, b, eps=1e-6):
    if len(a) != len(b):
        return False
    for x, y in zip(a, b):
        if abs(float(x) - float(y)) > eps:
            return False
    return True


def quat_equiv(a, b, eps=1e-6):
    if len(a) != 4 or len(b) != 4:
        return False

    def norm(q):
        n = math.sqrt(sum(float(v) * float(v) for v in q))
        if not math.isfinite(n) or n <= 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        return [float(v) / n for v in q]

    qa = norm(a)
    qb = norm(b)
    dot = abs(sum(x * y for x, y in zip(qa, qb)))
    return abs(1.0 - dot) <= eps


def main():
    js = run_json(["node", os.path.join("tests", "node_pose_viz_map.js")])
    py = run_json(["python3", os.path.join("tests", "python_pose_viz_map.py")])
    cpp = run_json([CPP_BIN])

    by_lang = {"js": js, "python": py, "cpp": cpp}
    expected_count = None
    expected_names = None

    for lang, data in by_lang.items():
        cases = data.get("cases", [])
        if expected_count is None:
            expected_count = len(cases)
        if len(cases) != expected_count:
            raise AssertionError(f"{lang}: case count mismatch {len(cases)} != {expected_count}")

        names = [c.get("name") for c in cases]
        if expected_names is None:
            expected_names = names
        if names != expected_names:
            raise AssertionError(f"{lang}: case ordering mismatch {names} != {expected_names}")

    ref = by_lang["js"]["cases"]
    for lang in ("python", "cpp"):
        cur = by_lang[lang]["cases"]
        for i, (a, b) in enumerate(zip(ref, cur)):
            if not vec_close(a["position"], b["position"]):
                raise AssertionError(
                    f"position mismatch case={a['name']} js={a['position']} {lang}={b['position']}"
                )
            if not quat_equiv(a["quat"], b["quat"]):
                raise AssertionError(
                    f"quat mismatch case={a['name']} js={a['quat']} {lang}={b['quat']}"
                )

    print("Cross-client pose parity test passed")


if __name__ == "__main__":
    main()
