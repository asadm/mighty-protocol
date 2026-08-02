#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mighty_sdk.h"

namespace mighty_voxblox {

struct MapperConfig {
  float voxel_size_m = 0.05f;
  std::size_t voxels_per_side = 16;
  float truncation_distance_m = 0.15f;
  float min_depth_m = 0.20f;
  float max_depth_m = 5.0f;
  int pixel_stride = 2;
  std::size_t max_pending_frames = 2;
  std::string integrator = "fast";  // fast | merged | simple
};

struct FusionInput {
  mighty_protocol::DepthFrame depth;
  mighty_protocol::sdk::PoseFrame pose;
};

struct MapperStats {
  std::uint64_t queued_frames = 0;
  std::uint64_t queue_drops = 0;
  std::uint64_t integrated_frames = 0;
  std::uint64_t rejected_frames = 0;
  std::uint64_t integrated_points = 0;
  std::size_t pending_frames = 0;
  std::size_t allocated_blocks = 0;
};

struct MeshVertex {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  std::uint8_t r = 255;
  std::uint8_t g = 255;
  std::uint8_t b = 255;
};

// Triangle-list snapshot for GUI rendering. Every consecutive three vertices
// form one triangle, so the viewer does not need to depend on voxblox types.
struct MeshSnapshot {
  std::uint64_t revision = 0;
  std::vector<MeshVertex> vertices;

  std::size_t triangle_count() const { return vertices.size() / 3; }
};

class VoxbloxMapper {
 public:
  VoxbloxMapper(const MapperConfig& config,
                const mighty_protocol::sdk::CameraCalibration& calibration);
  ~VoxbloxMapper();

  VoxbloxMapper(const VoxbloxMapper&) = delete;
  VoxbloxMapper& operator=(const VoxbloxMapper&) = delete;

  // Keeps the newest frames when integration cannot keep up.
  bool enqueue(FusionInput input);

  // Clears both queued work and the current TSDF. Used when Mighty reports a
  // VIO reset so two different odom frames are never fused into one map.
  void clear();

  // Finishes queued frames and joins the integration worker. Idempotent.
  void stop();

  bool save_mesh(const std::string& path, std::string* error = nullptr);
  bool save_map(const std::string& path, std::string* error = nullptr);
  bool mesh_snapshot(MeshSnapshot* snapshot, std::string* error = nullptr);
  MapperStats stats() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mighty_voxblox
