#include "mighty_algorithms/mighty_algorithms.hpp"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: loopclosure_sdk_example /path/to/place_model.{onnx,pt} "
                 "[/path/to/calibration.yaml]\n";
    return 2;
  }

  auto options = mighty_algorithms::defaultLoopClosureOptions();
  const std::string model_path = argv[1];
  options.keyframe_translation_m = 0.10;
  options.min_loop_gap = 1;
  options.min_orb_matches = 1;
  options.min_orb_inliers = 1;
  options.pnp_min_matches = 1;
  options.pnp_min_inliers = 1;

  mighty_algorithms::LoopClosure loopclosure(options);
  loopclosure.initialize(model_path);
  if (argc >= 3) {
    loopclosure.setCalibrationYaml(argv[2]);
  }

  constexpr int kWidth = 320;
  constexpr int kHeight = 240;
  std::vector<uint8_t> bgr(static_cast<size_t>(kWidth) * kHeight * 3, 0);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const size_t off = (static_cast<size_t>(y) * kWidth + x) * 3;
      bgr[off + 0] = static_cast<uint8_t>((x + y) & 255);
      bgr[off + 1] = static_cast<uint8_t>((2 * x) & 255);
      bgr[off + 2] = static_cast<uint8_t>((2 * y) & 255);
    }
  }

  for (int i = 0; i < 3; ++i) {
    ma_loopclosure_frame_t frame{};
    frame.frame_id = i;
    frame.timestamp_ns = static_cast<uint64_t>(i) * 33333333ull;
    frame.image.data = bgr.data();
    frame.image.size_bytes = bgr.size();
    frame.image.width = kWidth;
    frame.image.height = kHeight;
    frame.image.stride_bytes = static_cast<size_t>(kWidth) * 3;
    frame.image.format = MA_PIXEL_BGR8;
    frame.pose.timestamp_ns = frame.timestamp_ns;
    frame.pose.px = 0.10 * i;
    frame.pose.qw = 1.0;
    frame.pose.frame = MA_POSE_FRAME_CAMERA;
    frame.pose.confidence = 1.0f;
    frame.pose.is_keyframe_hint = 1;

    mighty_algorithms::LoopClosureResult result(loopclosure.pushFrame(frame));
    std::cout << "frame=" << i
              << " keyframes=" << result.get().keyframes_added
              << " loops=" << result.get().loops_accepted
              << " trajectory=" << result.get().trajectory_count << "\n";
  }

  mighty_algorithms::LoopClosureResult trajectory(loopclosure.trajectory());
  std::cout << "final trajectory=" << trajectory.get().trajectory_count << "\n";
  return 0;
}
