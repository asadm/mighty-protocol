#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "../cpp/mighty_sdk.h"

using mighty_protocol::sdk::Calibration;
using mighty_protocol::sdk::CameraCalibration;
using mighty_protocol::sdk::parse_calibration_yaml;

namespace {

bool approx(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

const char* kKalibrYaml = R"YAML(%YAML:1.0
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
)YAML";

const char* kOpenCvYaml = R"YAML(%YAML:1.0
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
)YAML";

void verify_cam0(const CameraCalibration& camera) {
  assert(camera.id == "cam0");
  assert(camera.camera_model == "double_sphere");
  assert(camera.distortion_model == "none");
  assert(approx(camera.intrinsics.fx, 205.0));
  assert(approx(camera.intrinsics.fy, 206.0));
  assert(approx(camera.intrinsics.cx, 281.0));
  assert(approx(camera.intrinsics.cy, 183.0));
  assert(camera.projection_parameters.size() == 2);
  assert(approx(camera.projection_parameters[0], -0.2));
  assert(approx(camera.projection_parameters[1], 0.6));
  assert(camera.resolution_width == 640);
  assert(camera.resolution_height == 400);
  assert(approx(camera.time_shift_camera_imu_s, -0.0028));
  assert(camera.camera_from_imu.valid);
  assert(camera.body_from_camera.valid);
  // T_body_camera translation is -R_body_camera * t_camera_imu.
  assert(approx(camera.body_from_camera.matrix[3], 0.03));
  assert(approx(camera.body_from_camera.matrix[7], -0.01));
  assert(approx(camera.body_from_camera.matrix[11], 0.02));
}

}  // namespace

int main() {
  Calibration calibration;
  std::string error;
  assert(parse_calibration_yaml(kKalibrYaml, &calibration, &error));
  assert(calibration.source_format == "kalibr");
  assert(calibration.cameras.size() == 2);
  verify_cam0(*calibration.camera("cam0"));
  assert(calibration.camera("cam0")->topic == "/cam0/image_raw/compressed");

  const CameraCalibration* cam1 = calibration.camera("cam1");
  assert(cam1 != nullptr);
  assert(cam1->camera_model == "pinhole");
  assert(cam1->distortion_coefficients.size() == 4);
  assert(cam1->camera_from_imu.valid);
  assert(approx(cam1->camera_from_imu.matrix[3], -0.1));

  assert(parse_calibration_yaml(kOpenCvYaml, &calibration, &error));
  assert(calibration.source_format == "opencv");
  assert(calibration.cameras.size() == 1);
  verify_cam0(calibration.cameras.front());

  assert(!parse_calibration_yaml("%YAML:1.0\nfoo: 1\n", &calibration, &error));
  assert(error.find("valid intrinsics") != std::string::npos);

  assert(parse_calibration_yaml(
      "cam0:\n  intrinsics: [1, 2, 3, 4]\n",
      &calibration, &error));
  assert(calibration.cameras.front().camera_model == "unknown");
  assert(calibration.cameras.front().distortion_model == "unknown");

  std::cout << "C++ calibration test passed" << std::endl;
  return 0;
}
