import assert from "node:assert";
import { parseCalibrationYaml } from "../js/index.js";

const kalibrYaml = `%YAML:1.0
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
`;

const openCvYaml = `%YAML:1.0
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
`;

function verifyCam0(camera) {
  assert.equal(camera.id, "cam0");
  assert.equal(camera.cameraModel, "double_sphere");
  assert.equal(camera.distortionModel, "none");
  assert.deepEqual(camera.intrinsics, { fx: 205, fy: 206, cx: 281, cy: 183 });
  assert.deepEqual(camera.projectionParameters, [-0.2, 0.6]);
  assert.deepEqual(camera.resolution, { width: 640, height: 400 });
  assert.equal(camera.timeShiftCameraImuS, -0.0028);
  assert.equal(camera.cameraFromImu.valid, true);
  assert.equal(camera.bodyFromCamera.valid, true);
  assert.deepEqual(
    [camera.bodyFromCamera.matrix[3], camera.bodyFromCamera.matrix[7], camera.bodyFromCamera.matrix[11]],
    [0.03, -0.01, 0.02],
  );
}

const kalibr = parseCalibrationYaml(kalibrYaml);
assert.equal(kalibr.sourceFormat, "kalibr");
assert.equal(kalibr.cameras.length, 2);
verifyCam0(kalibr.cameras[0]);
assert.equal(kalibr.cameras[0].topic, "/cam0/image_raw/compressed");
assert.equal(kalibr.cameras[1].cameraFromImu.matrix[3], -0.1);
assert.deepEqual(kalibr.cameras[1].distortionCoefficients, [0.1, -0.2, 0.01, -0.01]);

const opencv = parseCalibrationYaml(openCvYaml);
assert.equal(opencv.sourceFormat, "opencv");
assert.equal(opencv.cameras.length, 1);
verifyCam0(opencv.cameras[0]);

assert.throws(() => parseCalibrationYaml("%YAML:1.0\nfoo: 1\n"), /valid intrinsics/);
const minimal = parseCalibrationYaml("cam0:\n  intrinsics: [1, 2, 3, 4]\n");
assert.equal(minimal.cameras[0].cameraModel, "unknown");
assert.equal(minimal.cameras[0].distortionModel, "unknown");
console.log("Node calibration test passed");
