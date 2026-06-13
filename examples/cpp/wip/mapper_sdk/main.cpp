#include "mighty_algorithms/mighty_algorithms.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

int main() {
  auto options = mighty_algorithms::defaultMapperOptions();
  options.camera.width = 424;
  options.camera.height = 320;
  options.camera.fx = 210.0;
  options.camera.fy = 210.0;
  options.camera.cx = 212.0;
  options.camera.cy = 160.0;
  options.preset = 2;
  options.quiet = 1;

  mighty_algorithms::Mapper mapper(options);

  std::vector<uint8_t> image(
      static_cast<size_t>(options.camera.width) * options.camera.height, 127);

  for (int i = 0; i < 3; ++i) {
    ma_mapper_frame_t frame{};
    frame.frame_id = i;
    frame.timestamp_ns = static_cast<uint64_t>(i) * 33333333ull;
    frame.image.data = image.data();
    frame.image.size_bytes = image.size();
    frame.image.width = options.camera.width;
    frame.image.height = options.camera.height;
    frame.image.stride_bytes = static_cast<size_t>(options.camera.width);
    frame.image.format = MA_PIXEL_GRAY8;
    frame.has_camera_pose = 1;
    frame.camera_pose.timestamp_ns = frame.timestamp_ns;
    frame.camera_pose.qw = 1.0;
    frame.camera_pose.px = 0.03 * i;
    frame.camera_pose.frame = MA_POSE_FRAME_CAMERA;
    frame.camera_pose.confidence = 1.0f;

    const auto result = mapper.pushFrame(frame);
    std::cout << "frame=" << i << " status=" << ma_status_message(result.status)
              << " initialized=" << result.initialized
              << " lost=" << result.lost << "\n";
  }

  mapper.finish();
  mighty_algorithms::MapperSnapshot snapshot(mapper.snapshot());
  std::cout << "points=" << snapshot.get().point_count
            << " trajectory=" << snapshot.get().trajectory_count << "\n";
  return 0;
}

