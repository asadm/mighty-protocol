#include "voxblox_mapper.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace mighty_voxblox {
namespace {

using mighty_protocol::DEPTH_FLAG_RECTIFIED;
using mighty_protocol::DepthConvention;
using mighty_protocol::DepthEncoding;
using mighty_protocol::DepthFrame;
using mighty_protocol::sdk::PoseFrame;

bool finite_transform(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite();
}

bool extract_matrix_values(const std::string& yaml,
                           const std::string& key,
                           std::array<double, 16>* values) {
  if (!values) return false;
  const std::size_t key_pos = yaml.find(key);
  if (key_pos == std::string::npos) return false;

  const std::size_t colon_pos = yaml.find(':', key_pos + key.size());
  if (colon_pos == std::string::npos) return false;

  std::size_t search_pos = colon_pos + 1;
  const std::size_t matrix_tag = yaml.find("!!opencv-matrix", search_pos);
  if (matrix_tag != std::string::npos && matrix_tag - search_pos < 96) {
    const std::size_t data_pos = yaml.find("data", matrix_tag);
    if (data_pos == std::string::npos || data_pos - matrix_tag > 512) {
      return false;
    }
    search_pos = data_pos + 4;
  }

  const std::size_t open_bracket = yaml.find('[', search_pos);
  if (open_bracket == std::string::npos || open_bracket - search_pos > 512) {
    return false;
  }

  std::size_t close_bracket = open_bracket;
  int bracket_depth = 0;
  for (; close_bracket < yaml.size(); ++close_bracket) {
    if (yaml[close_bracket] == '[') {
      ++bracket_depth;
    } else if (yaml[close_bracket] == ']') {
      --bracket_depth;
      if (bracket_depth == 0) break;
    }
  }
  if (close_bracket >= yaml.size()) return false;

  std::size_t count = 0;
  const char* cursor = yaml.c_str() + open_bracket + 1;
  const char* end = yaml.c_str() + close_bracket;
  while (cursor < end && count < values->size()) {
    char* parsed_end = nullptr;
    const double value = std::strtod(cursor, &parsed_end);
    if (parsed_end != cursor) {
      if (!std::isfinite(value)) return false;
      (*values)[count++] = value;
      cursor = parsed_end;
    } else {
      ++cursor;
    }
  }
  return count == values->size();
}

Eigen::Isometry3d matrix_from_row_major(
    const std::array<double, 16>& values) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      transform.matrix()(row, col) = values[static_cast<std::size_t>(row * 4 + col)];
    }
  }
  return transform;
}

std::array<double, 16> row_major_from_matrix(
    const Eigen::Isometry3d& transform) {
  std::array<double, 16> values{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      values[static_cast<std::size_t>(row * 4 + col)] =
          transform.matrix()(row, col);
    }
  }
  return values;
}

Eigen::Matrix3d canonical_base_from_camera_rotation() {
  Eigen::Matrix3d rotation;
  // base_link is FLU and the depth frame is OpenCV optical (right/down/forward).
  // x_base=+z_cam, y_base=-x_cam, z_base=-y_cam.
  rotation << 0.0, 0.0, 1.0,
             -1.0, 0.0, 0.0,
              0.0, -1.0, 0.0;
  return rotation;
}

bool pose_is_body(const PoseFrame& pose) {
  return pose.pose_type == "body" || pose.pose_type_raw == 0;
}

bool pose_is_camera(const PoseFrame& pose) {
  return pose.pose_type == "camera" || pose.pose_type_raw == 1;
}

bool pose_to_world_camera(const PoseFrame& pose,
                          const BodyCameraCalibration& calibration,
                          Eigen::Isometry3d* world_camera) {
  if (!world_camera || !pose.orientation_xyzw.has_value()) return false;
  if (!pose_is_body(pose) && !pose_is_camera(pose)) return false;

  const auto& position = pose.position_m;
  const auto& orientation = *pose.orientation_xyzw;
  for (double value : position) {
    if (!std::isfinite(value)) return false;
  }
  for (double value : orientation) {
    if (!std::isfinite(value)) return false;
  }

  Eigen::Quaterniond quaternion(
      orientation[3], orientation[0], orientation[1], orientation[2]);
  if (quaternion.norm() < 1e-9) return false;
  quaternion.normalize();

  Eigen::Isometry3d world_pose = Eigen::Isometry3d::Identity();
  world_pose.linear() = quaternion.toRotationMatrix();
  world_pose.translation() =
      Eigen::Vector3d(position[0], position[1], position[2]);

  if (pose_is_camera(pose)) {
    *world_camera = world_pose;
    return finite_transform(*world_camera);
  }
  if (!calibration.valid) return false;

  *world_camera =
      world_pose * matrix_from_row_major(calibration.T_body_camera);
  return finite_transform(*world_camera);
}

voxblox::Color depth_color(float depth_m, float min_depth_m,
                           float max_depth_m) {
  const float denominator = std::max(1e-6f, max_depth_m - min_depth_m);
  const float t = std::clamp(
      (depth_m - min_depth_m) / denominator, 0.0f, 1.0f);
  const float mid = 1.0f - std::abs(2.0f * t - 1.0f);
  return voxblox::Color(
      static_cast<std::uint8_t>(std::lround(255.0f * t)),
      static_cast<std::uint8_t>(std::lround(190.0f * mid + 35.0f)),
      static_cast<std::uint8_t>(std::lround(255.0f * (1.0f - t))));
}

bool depth_to_pointcloud(const DepthFrame& depth,
                         const MapperConfig& config,
                         voxblox::Pointcloud* points,
                         voxblox::Colors* colors) {
  if (!points || !colors || depth.width == 0 || depth.height == 0) return false;
  if (depth.encoding !=
      static_cast<std::uint8_t>(DepthEncoding::kUint16Millimeters)) {
    return false;
  }
  if ((depth.flags & DEPTH_FLAG_RECTIFIED) == 0u) return false;
  if (!std::isfinite(depth.depth_scale_m) || depth.depth_scale_m <= 0.0f) {
    return false;
  }

  const std::uint64_t expected_samples =
      static_cast<std::uint64_t>(depth.width) * depth.height;
  if (expected_samples != depth.depth_mm.size()) return false;

  const float fx = depth.depth_intrinsics[0];
  const float fy = depth.depth_intrinsics[1];
  const float cx = depth.depth_intrinsics[2];
  const float cy = depth.depth_intrinsics[3];
  if (!std::isfinite(fx) || !std::isfinite(fy) ||
      !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0.0f || fy <= 0.0f) {
    return false;
  }

  const auto convention = static_cast<DepthConvention>(depth.depth_convention);
  if (convention != DepthConvention::kZDepth &&
      convention != DepthConvention::kRayRange) {
    return false;
  }

  points->clear();
  colors->clear();
  const int stride = std::max(1, config.pixel_stride);
  const std::size_t reserve_count =
      (static_cast<std::size_t>(depth.width) + stride - 1) / stride *
      ((static_cast<std::size_t>(depth.height) + stride - 1) / stride);
  points->reserve(reserve_count);
  colors->reserve(reserve_count);

  for (std::uint32_t v = 0; v < depth.height;
       v += static_cast<std::uint32_t>(stride)) {
    for (std::uint32_t u = 0; u < depth.width;
         u += static_cast<std::uint32_t>(stride)) {
      const std::size_t index =
          static_cast<std::size_t>(v) * depth.width + u;
      const std::uint16_t sample = depth.depth_mm[index];
      if (sample == depth.invalid_value) continue;

      const float measured_depth = sample * depth.depth_scale_m;
      if (!std::isfinite(measured_depth) || measured_depth <= 0.0f) continue;

      voxblox::Point ray(
          (static_cast<float>(u) - cx) / fx,
          (static_cast<float>(v) - cy) / fy,
          1.0f);
      voxblox::Point point;
      if (convention == DepthConvention::kRayRange) {
        point = ray.normalized() * measured_depth;
      } else {
        point = ray * measured_depth;
      }

      const float range_m = point.norm();
      if (!std::isfinite(range_m) || range_m < config.min_depth_m ||
          range_m > config.max_depth_m) {
        continue;
      }
      points->push_back(point);
      colors->push_back(
          depth_color(range_m, config.min_depth_m, config.max_depth_m));
    }
  }
  return !points->empty();
}

voxblox::Transformation to_voxblox_transform(
    const Eigen::Isometry3d& world_camera) {
  Eigen::Quaternionf quaternion(world_camera.linear().cast<float>());
  quaternion.normalize();
  const voxblox::Point translation = world_camera.translation().cast<float>();
  return voxblox::Transformation(quaternion, translation);
}

}  // namespace

BodyCameraCalibration parse_body_camera_calibration(
    const std::string& calibration_yaml) {
  BodyCameraCalibration result;
  if (calibration_yaml.empty()) {
    result.message = "calibration text is empty";
    return result;
  }

  std::array<double, 16> values{};
  std::string key = "T_cam_imu";
  if (!extract_matrix_values(calibration_yaml, key, &values)) {
    key = "T_cam_body";
    if (!extract_matrix_values(calibration_yaml, key, &values)) {
      result.message = "calibration has no readable T_cam_imu matrix";
      return result;
    }
  }

  const Eigen::Isometry3d camera_imu = matrix_from_row_major(values);
  if (!finite_transform(camera_imu)) {
    result.message = key + " contains non-finite values";
    return result;
  }
  if (std::abs(camera_imu.matrix()(3, 0)) > 1e-6 ||
      std::abs(camera_imu.matrix()(3, 1)) > 1e-6 ||
      std::abs(camera_imu.matrix()(3, 2)) > 1e-6 ||
      std::abs(camera_imu.matrix()(3, 3) - 1.0) > 1e-6) {
    result.message = key + " does not contain a homogeneous rigid transform";
    return result;
  }
  const Eigen::Matrix3d camera_imu_rotation = camera_imu.linear();
  const double determinant = camera_imu_rotation.determinant();
  const double orthogonality_error =
      (camera_imu_rotation * camera_imu_rotation.transpose() -
       Eigen::Matrix3d::Identity()).norm();
  if (std::abs(determinant - 1.0) > 0.1 || orthogonality_error > 0.1) {
    result.message = key + " does not contain a valid rigid rotation";
    return result;
  }

  const Eigen::Isometry3d imu_camera = camera_imu.inverse();
  const Eigen::Matrix3d base_camera_rotation =
      canonical_base_from_camera_rotation();
  const Eigen::Matrix3d base_imu_rotation =
      base_camera_rotation * camera_imu_rotation;

  Eigen::Isometry3d body_camera = Eigen::Isometry3d::Identity();
  body_camera.linear() = base_camera_rotation;
  body_camera.translation() = base_imu_rotation * imu_camera.translation();

  result.valid = finite_transform(body_camera);
  result.T_body_camera = row_major_from_matrix(body_camera);
  result.message = result.valid
      ? "loaded T_body_camera from protocol calibration " + key
      : "derived T_body_camera is invalid";
  return result;
}

class VoxbloxMapper::Impl {
 public:
  Impl(const MapperConfig& config,
       const BodyCameraCalibration& calibration)
      : config_(config), calibration_(calibration) {
    if (!std::isfinite(config_.voxel_size_m) || config_.voxel_size_m <= 0.0f) {
      throw std::invalid_argument("voxel_size_m must be positive");
    }
    if (config_.voxels_per_side == 0 ||
        (config_.voxels_per_side & (config_.voxels_per_side - 1)) != 0) {
      throw std::invalid_argument("voxels_per_side must be a power of two");
    }
    if (!std::isfinite(config_.truncation_distance_m) ||
        config_.truncation_distance_m <= 0.0f) {
      throw std::invalid_argument("truncation_distance_m must be positive");
    }
    if (!std::isfinite(config_.min_depth_m) ||
        !std::isfinite(config_.max_depth_m) || config_.min_depth_m < 0.0f ||
        config_.max_depth_m <= config_.min_depth_m) {
      throw std::invalid_argument("depth range is invalid");
    }
    if (config_.pixel_stride <= 0 || config_.max_pending_frames == 0) {
      throw std::invalid_argument("pixel stride and queue size must be positive");
    }
    if (config_.integrator != "fast" && config_.integrator != "merged" &&
        config_.integrator != "simple") {
      throw std::invalid_argument("unknown voxblox integrator: " +
                                  config_.integrator);
    }

    voxblox::TsdfMap::Config map_config;
    map_config.tsdf_voxel_size = config_.voxel_size_m;
    map_config.tsdf_voxels_per_side = config_.voxels_per_side;
    map_ = std::make_unique<voxblox::TsdfMap>(map_config);

    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.default_truncation_distance =
        config_.truncation_distance_m;
    integrator_config.min_ray_length_m = config_.min_depth_m;
    integrator_config.max_ray_length_m = config_.max_depth_m;
    integrator_config.allow_clear = false;
    integrator_ = voxblox::TsdfIntegratorFactory::create(
        config_.integrator, integrator_config, map_->getTsdfLayerPtr());
    if (!integrator_) throw std::runtime_error("voxblox integrator creation failed");

    worker_ = std::thread([this]() { worker_loop(); });
  }

  ~Impl() { stop(); }

  bool enqueue(FusionInput input) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stopping_) return false;
    if (queue_.size() >= config_.max_pending_frames) {
      queue_.pop_front();
      queue_drops_.fetch_add(1, std::memory_order_relaxed);
    }
    QueuedInput queued;
    queued.input = std::move(input);
    queued.generation = generation_.load(std::memory_order_relaxed);
    queue_.push_back(std::move(queued));
    queued_frames_.fetch_add(1, std::memory_order_relaxed);
    queue_cv_.notify_one();
    return true;
  }

  void clear() {
    generation_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_drops_.fetch_add(queue_.size(), std::memory_order_relaxed);
      queue_.clear();
    }
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    map_->getTsdfLayerPtr()->removeAllBlocks();
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) {
        if (!worker_.joinable()) return;
      } else {
        stopping_ = true;
      }
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  bool save_mesh(const std::string& path, std::string* error) {
    if (path.empty()) {
      if (error) *error = "mesh path is empty";
      return false;
    }
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    if (map_->getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
      if (error) *error = "TSDF has no allocated blocks";
      return false;
    }

    voxblox::MeshLayer mesh_layer(map_->block_size());
    voxblox::MeshIntegratorConfig mesh_config;
    mesh_config.use_color = true;
    voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
        mesh_config, map_->getTsdfLayerPtr(), &mesh_layer);
    mesh_integrator.generateMesh(false, false);
    if (!voxblox::outputMeshLayerAsPly(path, mesh_layer)) {
      if (error) *error = "voxblox could not write a non-empty PLY mesh";
      return false;
    }
    return true;
  }

  bool save_map(const std::string& path, std::string* error) {
    if (path.empty()) {
      if (error) *error = "map path is empty";
      return false;
    }
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    if (!voxblox::io::SaveLayer(map_->getTsdfLayer(), path, true)) {
      if (error) *error = "voxblox SaveLayer failed";
      return false;
    }
    return true;
  }

  MapperStats stats() const {
    MapperStats out;
    out.queued_frames = queued_frames_.load(std::memory_order_relaxed);
    out.queue_drops = queue_drops_.load(std::memory_order_relaxed);
    out.integrated_frames = integrated_frames_.load(std::memory_order_relaxed);
    out.rejected_frames = rejected_frames_.load(std::memory_order_relaxed);
    out.integrated_points = integrated_points_.load(std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      out.pending_frames = queue_.size();
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      out.allocated_blocks =
          map_->getTsdfLayer().getNumberOfAllocatedBlocks();
    }
    return out;
  }

 private:
  struct QueuedInput {
    FusionInput input;
    std::uint64_t generation = 0;
  };

  void worker_loop() {
    while (true) {
      QueuedInput queued;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { return stopping_ || !queue_.empty(); });
        if (queue_.empty()) {
          if (stopping_) return;
          continue;
        }
        queued = std::move(queue_.front());
        queue_.pop_front();
      }

      Eigen::Isometry3d world_camera = Eigen::Isometry3d::Identity();
      voxblox::Pointcloud points;
      voxblox::Colors colors;
      if (!pose_to_world_camera(
              queued.input.pose, calibration_, &world_camera) ||
          !depth_to_pointcloud(queued.input.depth, config_, &points, &colors)) {
        rejected_frames_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }

      try {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        if (queued.generation != generation_.load(std::memory_order_relaxed)) {
          rejected_frames_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
        integrator_->integratePointCloud(
            to_voxblox_transform(world_camera), points, colors, false);
        integrated_frames_.fetch_add(1, std::memory_order_relaxed);
        integrated_points_.fetch_add(points.size(), std::memory_order_relaxed);
      } catch (const std::exception&) {
        rejected_frames_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  MapperConfig config_;
  BodyCameraCalibration calibration_;
  std::unique_ptr<voxblox::TsdfMap> map_;
  voxblox::TsdfIntegratorBase::Ptr integrator_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<QueuedInput> queue_;
  bool stopping_ = false;
  std::thread worker_;

  mutable std::mutex map_mutex_;
  std::atomic<std::uint64_t> generation_{0};
  std::atomic<std::uint64_t> queued_frames_{0};
  std::atomic<std::uint64_t> queue_drops_{0};
  std::atomic<std::uint64_t> integrated_frames_{0};
  std::atomic<std::uint64_t> rejected_frames_{0};
  std::atomic<std::uint64_t> integrated_points_{0};
};

VoxbloxMapper::VoxbloxMapper(
    const MapperConfig& config,
    const BodyCameraCalibration& calibration)
    : impl_(std::make_unique<Impl>(config, calibration)) {}

VoxbloxMapper::~VoxbloxMapper() = default;

bool VoxbloxMapper::enqueue(FusionInput input) {
  return impl_->enqueue(std::move(input));
}

void VoxbloxMapper::clear() { impl_->clear(); }

void VoxbloxMapper::stop() { impl_->stop(); }

bool VoxbloxMapper::save_mesh(const std::string& path, std::string* error) {
  return impl_->save_mesh(path, error);
}

bool VoxbloxMapper::save_map(const std::string& path, std::string* error) {
  return impl_->save_map(path, error);
}

MapperStats VoxbloxMapper::stats() const { return impl_->stats(); }

}  // namespace mighty_voxblox
