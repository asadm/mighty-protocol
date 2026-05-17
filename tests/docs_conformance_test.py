import os

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, ".."))


REQUIRED_DOC_SNIPPETS = [
    'frame_id = "odom"',
    'child_frame_id = "base_link"',
    'equation: `p_odom = R_odom_body * p_body + t_odom_body`',
    'component order is `xyzw`',
    'bit 2: linear velocity in body frame (`linearVelocityBodyMps`, f64 x3)',
    'bit 3: angular velocity in body frame (`angularVelocityBodyRps`, f64 x3)',
    'SDK pose callback fields:',
    'positionM',
    'position_m',
    'position_m`, `orientation_xyzw`',
]


def read_text(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def assert_contains(text, needle, where):
    if needle not in text:
        raise AssertionError(f"missing snippet in {where}: {needle}")


def main():
    protocol_doc = read_text(os.path.join(ROOT, "docs", "sdk", "protocol.mdx"))
    for snippet in REQUIRED_DOC_SNIPPETS:
        assert_contains(protocol_doc, snippet, "docs/sdk/protocol.mdx")

    cpp_example = read_text(os.path.join(ROOT, "examples", "cpp", "main.cpp"))
    py_example = read_text(os.path.join(ROOT, "examples", "python", "mightyapp.py"))
    js_example = read_text(os.path.join(ROOT, "examples", "web", "uihelpers.js"))

    # Ensure all example clients explicitly apply canonical->viz conversion.
    assert_contains(cpp_example, "kRvizFromOdom", "examples/cpp/main.cpp")
    assert_contains(cpp_example, "map_pose_position_odom_to_viz", "examples/cpp/main.cpp")
    assert_contains(cpp_example, "map_pose_quat_odom_to_viz", "examples/cpp/main.cpp")

    assert_contains(py_example, "R_VIZ_FROM_ODOM", "examples/python/mightyapp.py")
    assert_contains(py_example, "Q_VIZ_FROM_ODOM", "examples/python/mightyapp.py")
    assert_contains(py_example, "_map_position_odom_to_viz", "examples/python/mightyapp.py")
    assert_contains(py_example, "_map_quat_odom_to_viz", "examples/python/mightyapp.py")

    assert_contains(js_example, "R_VIZ_FROM_ODOM", "examples/web/uihelpers.js")
    assert_contains(js_example, "Q_VIZ_FROM_ODOM", "examples/web/uihelpers.js")
    assert_contains(js_example, "mapCanonicalPoseToViz", "examples/web/uihelpers.js")

    # Basis rows should stay aligned across docs/sdk examples.
    cpp_rows = ["0.0, -1.0, 0.0", "0.0, 0.0, 1.0", "-1.0, 0.0, 0.0"]
    py_rows = ["(0.0, -1.0, 0.0)", "(0.0, 0.0, 1.0)", "(-1.0, 0.0, 0.0)"]
    js_rows = ["0, -1, 0", "0, 0, 1", "-1, 0, 0"]
    for row in cpp_rows:
        assert_contains(cpp_example, row, "examples/cpp/main.cpp")
    for row in py_rows:
        assert_contains(py_example, row, "examples/python/mightyapp.py")
    for row in js_rows:
        assert_contains(js_example, row, "examples/web/uihelpers.js")

    print("Docs/sdk examples conformance test passed")


if __name__ == "__main__":
    main()
