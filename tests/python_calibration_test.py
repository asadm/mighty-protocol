import os
import sys


HERE = os.path.dirname(__file__)
sys.path.append(os.path.join(HERE, "..", "python"))

from mighty_sdk import parse_calibration_yaml  # noqa: E402


KALIBR_YAML = """%YAML:1.0
cam0:
  T_cam_imu:
    - [1.0, 0.0, 0.0, -0.01]
    - [0.0, -1.0, 0.0, 0.02]
    - [0.0, 0.0, -1.0, -0.03]
    - [0.0, 0.0, 0.0, 1.0]
  camera_model: ds
  distortion_coeffs: []
  distortion_model: none
  intrinsics: [-0.2, 0.6, 205.0, 206.0, 281.0, 183.0]
  resolution: [640, 400]
  rostopic: /cam0/image_raw/compressed
  timeshift_cam_imu: -0.0028
cam1:
  T_imu_cam:
    - [1.0, 0.0, 0.0, 0.1]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [0.1, -0.2, 0.01, -0.01]
  distortion_model: radtan
  intrinsics: [300.0, 301.0, 320.0, 200.0]
  resolution: [640, 400]
"""

OPENCV_YAML = """%YAML:1.0
camera_model: "ds"
distortion_model: "none"
resolution_width: 640
resolution_height: 400
intrinsics:
   fx: 205.0
   fy: 206.0
   cx: 281.0
   cy: 183.0
distortion_coeffs:
   xi: -0.2
   alpha: 0.6
T_cam_imu: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [1.0, 0.0, 0.0, -0.01,
          0.0, -1.0, 0.0, 0.02,
          0.0, 0.0, -1.0, -0.03,
          0.0, 0.0, 0.0, 1.0]
td: -0.0028
"""


def verify_cam0(camera):
    assert camera["id"] == "cam0"
    assert camera["camera_model"] == "double_sphere"
    assert camera["distortion_model"] == "none"
    assert camera["intrinsics"] == {"fx": 205.0, "fy": 206.0, "cx": 281.0, "cy": 183.0}
    assert camera["projection_parameters"] == [-0.2, 0.6]
    assert camera["resolution"] == {"width": 640, "height": 400}
    assert camera["time_shift_camera_imu_s"] == -0.0028
    assert camera["camera_from_imu"]["valid"]
    assert camera["body_from_camera"]["valid"]
    assert [camera["body_from_camera"]["matrix"][index] for index in (3, 7, 11)] == [0.03, -0.01, 0.02]


kalibr = parse_calibration_yaml(KALIBR_YAML)
assert kalibr["source_format"] == "kalibr"
assert len(kalibr["cameras"]) == 2
verify_cam0(kalibr["cameras"][0])
assert kalibr["cameras"][0]["topic"] == "/cam0/image_raw/compressed"
assert kalibr["cameras"][1]["camera_from_imu"]["matrix"][3] == -0.1
assert kalibr["cameras"][1]["distortion_coefficients"] == [0.1, -0.2, 0.01, -0.01]

opencv = parse_calibration_yaml(OPENCV_YAML)
assert opencv["source_format"] == "opencv"
assert len(opencv["cameras"]) == 1
verify_cam0(opencv["cameras"][0])

try:
    parse_calibration_yaml("%YAML:1.0\nfoo: 1\n")
    raise AssertionError("invalid calibration unexpectedly parsed")
except ValueError as exc:
    assert "valid intrinsics" in str(exc)

minimal = parse_calibration_yaml("cam0:\n  intrinsics: [1, 2, 3, 4]\n")
assert minimal["cameras"][0]["camera_model"] == "unknown"
assert minimal["cameras"][0]["distortion_model"] == "unknown"

print("Python calibration test passed")
