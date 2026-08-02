#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "voxblox_mapper.h"

namespace mighty_voxblox {

struct ViewerPose {
  std::uint64_t timestamp_ns = 0;
  std::array<double, 3> position_m{0.0, 0.0, 0.0};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct DepthPreview {
  std::uint64_t timestamp_ns = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::uint8_t> rgb;
};

struct ViewerStats {
  bool connected = false;
  bool inspection_paused = false;
  std::string source;
  std::string vio_state;
  std::uint64_t poses_received = 0;
  std::uint64_t depth_received = 0;
  std::uint64_t matched_frames = 0;
  std::uint64_t unmatched_depth_drops = 0;
  std::uint64_t integrated_frames = 0;
  std::uint64_t rejected_frames = 0;
  std::uint64_t queue_drops = 0;
  std::size_t allocated_blocks = 0;
  std::uint64_t integrated_points = 0;
};

class ViewerState {
 public:
  struct Snapshot {
    std::shared_ptr<const MeshSnapshot> mesh;
    std::shared_ptr<const DepthPreview> depth;
    std::vector<ViewerPose> trajectory;
    std::string status;
  };

  void set_status(std::string status);
  void set_mesh(std::shared_ptr<const MeshSnapshot> mesh);
  void update_depth(const mighty_protocol::DepthFrame& depth,
                    float min_depth_m,
                    float max_depth_m);
  void add_pose(const ViewerPose& pose);
  void clear_map_visuals();
  Snapshot snapshot() const;

 private:
  static constexpr std::size_t kMaxTrajectoryPoses = 10000;

  mutable std::mutex mutex_;
  std::shared_ptr<const MeshSnapshot> mesh_ =
      std::make_shared<MeshSnapshot>();
  std::shared_ptr<const DepthPreview> depth_;
  std::vector<ViewerPose> trajectory_;
  std::string status_ = "Starting";
};

struct ViewerActions {
  std::function<void()> tick;
  std::function<void()> start_vio;
  std::function<void()> stop_vio;
  std::function<void()> clear_map;
  std::function<void()> save_mesh;
  std::function<void()> save_map;
  std::function<void()> save_outputs;
  std::function<ViewerStats()> read_stats;
};

struct ViewerOptions {
  bool follow_pose = false;
};

// Blocks until the user closes the native Pangolin window or a signal requests
// shutdown. All Mighty commands are supplied by the application through
// ViewerActions so the viewer itself remains protocol-agnostic.
void run_voxblox_viewer(ViewerState* state,
                        const ViewerActions& actions,
                        const ViewerOptions& options,
                        const std::function<bool()>& should_stop);

}  // namespace mighty_voxblox
