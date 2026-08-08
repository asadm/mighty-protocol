#pragma once

#if defined(MIGHTY_PROTOCOL_ENABLE_OCCUPANCY_GRID)

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <mighty_algorithms/mighty_algorithms.h>

#include "mighty_client.h"

namespace mighty_protocol {
namespace sdk {

template <typename T>
class Span {
 public:
  Span() = default;
  Span(const T* data, std::size_t size) : data_(data), size_(size) {}

  const T* data() const { return data_; }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  const T& operator[](std::size_t index) const { return data_[index]; }
  const T* begin() const { return data_; }
  const T* end() const { return size_ == 0 ? data_ : data_ + size_; }

 private:
  const T* data_ = nullptr;
  std::size_t size_ = 0;
};

struct GridPoint3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct GridBox3f {
  GridPoint3f minimum;
  GridPoint3f maximum;
};

struct VoxelIndex {
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::int32_t z = 0;

  bool operator==(const VoxelIndex& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

enum class OccupancyState : std::uint8_t {
  kUnknown = 0,
  kFree = 1,
  kOccupied = 2,
};

struct OccupancyCell {
  VoxelIndex index;
  OccupancyState state = OccupancyState::kUnknown;
  std::uint8_t occupancy = 0;
  std::uint8_t intensity = 0;
  std::uint8_t support = 0;
};

struct OccupancyGridUpdate {
  std::uint64_t base_revision = 0;
  std::uint64_t revision = 0;
  std::uint64_t timestamp_ns = 0;
  bool reset = false;
  float resolution_m = 0.0f;
  std::optional<GridPoint3f> rolling_center;
  std::optional<float> rolling_radius_m;
  // Valid only for the duration of the update callback. Copy cells before
  // forwarding the update to another thread.
  Span<const OccupancyCell> changes;
};

struct OccupancyGridSnapshot {
  std::uint64_t revision = 0;
  std::uint64_t timestamp_ns = 0;
  float resolution_m = 0.0f;
  std::optional<GridPoint3f> rolling_center;
  std::optional<float> rolling_radius_m;
  std::vector<OccupancyCell> cells;
};

struct MightyOccupancyGridOptions {
  float resolution_m = 0.05f;
  std::optional<float> rolling_radius_m = 4.0f;
  std::optional<std::size_t> max_pending_frames = std::nullopt;
};

struct MightyOccupancyGridStats {
  std::uint64_t received_frames = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t processed_frames = 0;
  std::uint64_t integrated_frames = 0;
  std::uint64_t rejected_frames = 0;
  std::uint64_t integrated_points = 0;
  std::size_t pending_frames = 0;
  std::size_t cell_count = 0;
  std::size_t occupied_cell_count = 0;
  std::size_t memory_bytes = 0;
};

class MightyOccupancyGrid {
 public:
  using UpdateHandler = std::function<void(const OccupancyGridUpdate&)>;
  using CellVisitor = std::function<void(const OccupancyCell&)>;

  explicit MightyOccupancyGrid(
      std::shared_ptr<MightyClient> client,
      MightyOccupancyGridOptions options = MightyOccupancyGridOptions())
      : client_(std::move(client)), options_(std::move(options)) {
    if (!client_) throw std::invalid_argument("MightyClient is required");
    if (!std::isfinite(options_.resolution_m) || options_.resolution_m <= 0.0f) {
      throw std::invalid_argument("resolution_m must be positive");
    }
    if (options_.rolling_radius_m &&
        (!std::isfinite(*options_.rolling_radius_m) ||
         *options_.rolling_radius_m <= options_.resolution_m)) {
      throw std::invalid_argument("rolling_radius_m is invalid");
    }
    if (options_.max_pending_frames && *options_.max_pending_frames == 0) {
      throw std::invalid_argument("max_pending_frames must be positive");
    }

    const CalibrationGetResult calibration_result = client_->get_calibration();
    if (!calibration_result.ok || !calibration_result.found) {
      throw std::runtime_error(
          calibration_result.message.empty()
              ? "camera calibration is unavailable"
              : calibration_result.message);
    }
    const CameraCalibration* selected =
        calibration_result.value.camera("cam0");
    if (!selected && !calibration_result.value.cameras.empty()) {
      selected = &calibration_result.value.cameras.front();
    }
    if (!selected || !selected->valid() ||
        !selected->body_from_camera.valid) {
      throw std::runtime_error("camera calibration is incomplete");
    }

    state_ = std::make_shared<State>();
    state_->max_pending_frames = options_.max_pending_frames;
    initialize(*selected);
    subscribe();
  }

  ~MightyOccupancyGrid() { close(); }

  MightyOccupancyGrid(const MightyOccupancyGrid&) = delete;
  MightyOccupancyGrid& operator=(const MightyOccupancyGrid&) = delete;

  // The callback runs synchronously on the thread that calls process().
  void setUpdateCallback(UpdateHandler handler) {
    std::lock_guard<std::mutex> lock(state_->handler_mutex);
    state_->handler = std::move(handler);
  }

  // Waits efficiently for input, detaches everything currently queued, and
  // processes that finite batch. Input arriving meanwhile remains for the
  // next call. Returns false after close().
  bool process() { return processImpl(true); }
  // Processes one currently queued batch without waiting.
  bool tryProcess() { return processImpl(false); }

  void close() {
    const std::shared_ptr<State> state = state_;
    if (!state) return;
    bool expected = false;
    if (!state->closed.compare_exchange_strong(expected, true)) return;

    client_->unsubscribe(image_subscription_);
    client_->unsubscribe(pose_subscription_);
    client_->unsubscribe(vio_subscription_);
    client_->unsubscribe(reset_subscription_);
    {
      std::lock_guard<std::mutex> lock(state->queue_mutex);
      state->queue.clear();
      state->queued_images = 0;
    }
    state->queue_condition.notify_all();
  }

  bool closed() const { return !state_ || state_->closed.load(); }

  void clear() {
    const std::shared_ptr<State> state = requireState();
    std::unique_lock<std::mutex> process_lock(state->process_mutex);
    state->pending_images.clear();
    state->poses.clear();
    ma_occupancy_update_t native{};
    throwIfError(ma_occupancy_clear(state->grid, &native));
    OwnedUpdate update = translateUpdate(native);
    ma_occupancy_update_destroy(&native);
    process_lock.unlock();
    emitUpdate(state, update);
  }

  OccupancyCell cell(const VoxelIndex& index) const {
    const std::shared_ptr<State> state = requireState();
    ma_occupancy_cell_t native{};
    throwIfError(ma_occupancy_cell(
        state->grid, ma_voxel_index_t{index.x, index.y, index.z}, &native));
    return translateCell(native);
  }

  OccupancyCell cellAt(const GridPoint3f& point) const {
    const std::shared_ptr<State> state = requireState();
    ma_occupancy_cell_t native{};
    throwIfError(ma_occupancy_cell_at(
        state->grid, point.x, point.y, point.z, &native));
    return translateCell(native);
  }

  bool intersectsOccupied(const GridBox3f& box) const {
    const std::shared_ptr<State> state = requireState();
    const ma_box3f_t native = nativeBox(box);
    int value = 0;
    throwIfError(
        ma_occupancy_intersects_occupied(state->grid, &native, &value));
    return value != 0;
  }

  bool isKnownFree(const GridBox3f& box) const {
    const std::shared_ptr<State> state = requireState();
    const ma_box3f_t native = nativeBox(box);
    int value = 0;
    throwIfError(ma_occupancy_is_known_free(state->grid, &native, &value));
    return value != 0;
  }

  OccupancyGridSnapshot snapshot() const {
    const std::shared_ptr<State> state = requireState();
    ma_occupancy_snapshot_t native{};
    throwIfError(ma_occupancy_snapshot(state->grid, &native));
    OccupancyGridSnapshot output;
    output.revision = native.revision;
    output.timestamp_ns = native.timestamp_ns;
    output.resolution_m = native.resolution_m;
    if (native.has_rolling_center) {
      output.rolling_center = GridPoint3f{native.rolling_center[0],
                                         native.rolling_center[1],
                                         native.rolling_center[2]};
    }
    if (native.has_rolling_radius) {
      output.rolling_radius_m = native.rolling_radius_m;
    }
    output.cells.reserve(native.cell_count);
    for (std::size_t i = 0; i < native.cell_count; ++i) {
      output.cells.push_back(translateCell(native.cells[i]));
    }
    ma_occupancy_snapshot_destroy(&native);
    return output;
  }

  void visitCells(const CellVisitor& visitor) const {
    if (!visitor) return;
    const OccupancyGridSnapshot copy = snapshot();
    for (const OccupancyCell& value : copy.cells) visitor(value);
  }

  void visitOccupiedCells(const CellVisitor& visitor) const {
    if (!visitor) return;
    const OccupancyGridSnapshot copy = snapshot();
    for (const OccupancyCell& value : copy.cells) {
      if (value.state == OccupancyState::kOccupied) visitor(value);
    }
  }

  MightyOccupancyGridStats stats() const {
    const std::shared_ptr<State> state = requireState();
    std::size_t pending_frames = 0;
    {
      std::scoped_lock<std::mutex, std::mutex> lock(state->process_mutex,
                                                     state->queue_mutex);
      pending_frames = state->queued_images + state->pending_images.size();
    }
    ma_occupancy_stats_t native{};
    throwIfError(ma_occupancy_stats(state->grid, &native));
    MightyOccupancyGridStats output;
    output.received_frames = state->received_images.load();
    output.dropped_frames = state->dropped_images.load();
    output.processed_frames = native.processed_frames;
    output.integrated_frames = native.integrated_frames;
    output.rejected_frames = native.rejected_frames;
    output.integrated_points = native.integrated_points;
    output.cell_count = native.cell_count;
    output.occupied_cell_count = native.occupied_cell_count;
    output.memory_bytes = native.memory_bytes;
    output.pending_frames = pending_frames;
    return output;
  }

  VoxelIndex worldToIndex(const GridPoint3f& point) const {
    const float resolution = options_.resolution_m;
    return VoxelIndex{
        static_cast<std::int32_t>(std::floor(point.x / resolution)),
        static_cast<std::int32_t>(std::floor(point.y / resolution)),
        static_cast<std::int32_t>(std::floor(point.z / resolution))};
  }

  GridPoint3f indexCenter(const VoxelIndex& index) const {
    const float resolution = options_.resolution_m;
    return GridPoint3f{(static_cast<float>(index.x) + 0.5f) * resolution,
                       (static_cast<float>(index.y) + 0.5f) * resolution,
                       (static_cast<float>(index.z) + 0.5f) * resolution};
  }

 private:
  struct Metrics {
    bool available = false;
    float translation_confidence = 1.0f;
    float translation_observability = 1.0f;
    std::uint32_t degraded_reason_flags = 0;
  };

  struct TimedPose {
    PoseFrame pose;
    Metrics metrics;
  };

  struct PendingEvent {
    enum class Type { kImage, kPose, kMetrics, kReset };
    Type type = Type::kReset;
    RawImageFrame image;
    PoseFrame pose;
    VioStateFrame metrics;
  };

  struct ReadyFrame {
    RawImageFrame image;
    TimedPose pose;
  };

  struct OwnedUpdate {
    std::uint64_t base_revision = 0;
    std::uint64_t revision = 0;
    std::uint64_t timestamp_ns = 0;
    bool reset = false;
    float resolution_m = 0.0f;
    std::optional<GridPoint3f> rolling_center;
    std::optional<float> rolling_radius_m;
    std::vector<OccupancyCell> changes;
  };

  struct State {
    ~State() { ma_occupancy_destroy(grid); }

    ma_occupancy_grid_t* grid = nullptr;
    std::atomic<bool> closed{false};
    std::optional<std::size_t> max_pending_frames;
    std::mutex queue_mutex;
    std::condition_variable queue_condition;
    std::deque<PendingEvent> queue;
    std::size_t queued_images = 0;
    std::atomic<std::uint64_t> received_images{0};
    std::atomic<std::uint64_t> dropped_images{0};

    mutable std::mutex process_mutex;
    std::deque<RawImageFrame> pending_images;
    std::deque<TimedPose> poses;
    Metrics latest_metrics;

    std::mutex handler_mutex;
    UpdateHandler handler;
  };

  static void throwIfError(ma_status_t status) {
    if (status != MA_STATUS_OK) {
      throw std::runtime_error(ma_status_message(status));
    }
  }

  std::shared_ptr<State> requireState() const {
    if (!state_) throw std::runtime_error("occupancy grid is unavailable");
    return state_;
  }

  void initialize(const CameraCalibration& calibration) {
    std::vector<double> projection = calibration.projection_parameters;
    std::vector<double> distortion = calibration.distortion_coefficients;
    ma_occupancy_calibration_t native{};
    native.width = static_cast<int>(calibration.resolution_width);
    native.height = static_cast<int>(calibration.resolution_height);
    native.fx = calibration.intrinsics.fx;
    native.fy = calibration.intrinsics.fy;
    native.cx = calibration.intrinsics.cx;
    native.cy = calibration.intrinsics.cy;
    native.camera_model = calibration.camera_model == "double_sphere"
                              ? MA_CAMERA_MODEL_DOUBLE_SPHERE
                              : MA_CAMERA_MODEL_PINHOLE;
    if (calibration.distortion_model == "radtan") {
      native.distortion_model = MA_DISTORTION_RADTAN;
    } else if (calibration.distortion_model == "equidistant") {
      native.distortion_model = MA_DISTORTION_EQUIDISTANT;
    } else {
      native.distortion_model = MA_DISTORTION_NONE;
    }
    native.projection_parameters = projection.data();
    native.projection_parameter_count = projection.size();
    native.distortion_coefficients = distortion.data();
    native.distortion_coefficient_count = distortion.size();
    std::copy(calibration.body_from_camera.matrix.begin(),
              calibration.body_from_camera.matrix.end(),
              native.body_from_camera);

    ma_occupancy_options_t native_options{};
    ma_occupancy_options_default(&native_options);
    native_options.resolution_m = options_.resolution_m;
    native_options.has_rolling_radius = options_.rolling_radius_m ? 1 : 0;
    native_options.rolling_radius_m =
        options_.rolling_radius_m.value_or(0.0f);
    throwIfError(
        ma_occupancy_create(&native_options, &native, &state_->grid));
  }

  void subscribe() {
    const std::weak_ptr<State> weak = state_;
    image_subscription_ = client_->on_image([weak](const ImageFrame& image) {
      const std::shared_ptr<State> state = weak.lock();
      if (!state || state->closed.load()) return;
      const RawImageFrame* selected = primaryImage(image);
      if (!selected || selected->timestamp_ns == 0) return;
      PendingEvent event;
      event.type = PendingEvent::Type::kImage;
      event.image = *selected;
      enqueue(state, std::move(event));
      state->received_images.fetch_add(1);
    });
    pose_subscription_ = client_->on_pose([weak](const PoseFrame& pose) {
      const std::shared_ptr<State> state = weak.lock();
      if (!state || state->closed.load() || !usablePose(pose)) return;
      PendingEvent event;
      event.type = PendingEvent::Type::kPose;
      event.pose = pose;
      enqueue(state, std::move(event));
    });
    vio_subscription_ = client_->on_vio_state(
        [weak](const VioStateFrame& metrics) {
          const std::shared_ptr<State> state = weak.lock();
          if (!state || state->closed.load()) return;
          PendingEvent event;
          event.type = PendingEvent::Type::kMetrics;
          event.metrics = metrics;
          enqueue(state, std::move(event));
        });
    reset_subscription_ = client_->on_reset([weak](const ResetEvent&) {
      const std::shared_ptr<State> state = weak.lock();
      if (!state || state->closed.load()) return;
      PendingEvent event;
      event.type = PendingEvent::Type::kReset;
      enqueue(state, std::move(event));
    });
  }

  static bool primaryChannel(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char character) {
                     return static_cast<char>(std::tolower(character));
                   });
    return value.empty() || value == "cam0" || value == "preview" ||
           value == "left";
  }

  static const RawImageFrame* primaryImage(const ImageFrame& image) {
    if (image.kind == ImageFrame::Kind::kRaw) return &image.left;
    const auto channel = [](const RawImageFrame& value) {
      return value.channel_alias.empty() ? value.channel : value.channel_alias;
    };
    if (primaryChannel(channel(image.left))) return &image.left;
    if (image.right && primaryChannel(channel(*image.right))) {
      return &*image.right;
    }
    return &image.left;
  }

  static bool usablePose(const PoseFrame& pose) {
    return pose.is_public && pose.timestamp_ns && *pose.timestamp_ns != 0 &&
           pose.orientation_xyzw &&
           (pose.pose_type == "body" || pose.pose_type == "camera" ||
            pose.pose_type.empty() || pose.pose_type_raw <= 1);
  }

  static void enqueue(const std::shared_ptr<State>& state,
                      PendingEvent event) {
    {
      std::lock_guard<std::mutex> lock(state->queue_mutex);
      if (state->closed.load()) return;
      if (event.type == PendingEvent::Type::kImage &&
          state->max_pending_frames &&
          state->queued_images >= *state->max_pending_frames) {
        const auto oldest = std::find_if(
            state->queue.begin(), state->queue.end(),
            [](const PendingEvent& candidate) {
              return candidate.type == PendingEvent::Type::kImage;
            });
        if (oldest != state->queue.end()) {
          state->queue.erase(oldest);
          --state->queued_images;
          state->dropped_images.fetch_add(1);
        }
      }
      if (event.type == PendingEvent::Type::kImage) ++state->queued_images;
      state->queue.push_back(std::move(event));
    }
    state->queue_condition.notify_one();
  }

  bool processImpl(bool wait) {
    const std::shared_ptr<State> state = requireState();
    std::unique_lock<std::mutex> process_lock(state->process_mutex);
    std::deque<PendingEvent> batch;
    {
      std::unique_lock<std::mutex> queue_lock(state->queue_mutex);
      if (wait) {
        state->queue_condition.wait(queue_lock, [&]() {
          return state->closed.load() || !state->queue.empty();
        });
      }
      if (state->closed.load()) return false;
      if (state->queue.empty()) return false;
      batch.swap(state->queue);
      state->queued_images = 0;
    }

    bool reset = false;
    std::uint64_t reset_base_revision = 0;
    for (PendingEvent& event : batch) {
      switch (event.type) {
        case PendingEvent::Type::kMetrics:
          state->latest_metrics.available =
              event.metrics.translation_confidence01.has_value() ||
              event.metrics.translation_observability01.has_value();
          state->latest_metrics.translation_confidence =
              event.metrics.translation_confidence01.value_or(1.0f);
          state->latest_metrics.translation_observability =
              event.metrics.translation_observability01.value_or(1.0f);
          state->latest_metrics.degraded_reason_flags =
              event.metrics.degraded_reason_flags.value_or(0u);
          break;
        case PendingEvent::Type::kPose: {
          ma_pose_t native = nativePose(event.pose);
          throwIfError(ma_occupancy_update_pose(state->grid, &native));
          state->poses.push_back(
              TimedPose{std::move(event.pose), state->latest_metrics});
          while (state->poses.size() > 240) state->poses.pop_front();
          break;
        }
        case PendingEvent::Type::kImage:
          state->pending_images.push_back(std::move(event.image));
          break;
        case PendingEvent::Type::kReset: {
          state->pending_images.clear();
          state->poses.clear();
          ma_occupancy_update_t cleared{};
          throwIfError(ma_occupancy_clear(state->grid, &cleared));
          if (!reset) reset_base_revision = cleared.base_revision;
          reset = true;
          ma_occupancy_update_destroy(&cleared);
          break;
        }
      }
    }

    std::vector<ReadyFrame> ready = match(state);
    std::vector<ma_occupancy_frame_t> native_frames;
    native_frames.reserve(ready.size());
    for (const ReadyFrame& value : ready) {
      ma_occupancy_frame_t native{};
      native.timestamp_ns = value.image.timestamp_ns;
      native.image = nativeImage(value.image);
      native.pose = nativePose(value.pose.pose);
      native.translation_metrics_available = value.pose.metrics.available ? 1 : 0;
      native.translation_confidence =
          value.pose.metrics.translation_confidence;
      native.translation_observability =
          value.pose.metrics.translation_observability;
      native.degraded_reason_flags =
          value.pose.metrics.degraded_reason_flags;
      native_frames.push_back(native);
    }

    ma_occupancy_update_t native_update{};
    throwIfError(ma_occupancy_process_frames(
        state->grid, native_frames.data(), native_frames.size(),
        &native_update));
    OwnedUpdate update = translateUpdate(native_update);
    ma_occupancy_update_destroy(&native_update);
    if (reset) {
      update.reset = true;
      update.base_revision = reset_base_revision;
    }
    process_lock.unlock();
    emitUpdate(state, update);
    return true;
  }

  static std::vector<ReadyFrame> match(const std::shared_ptr<State>& state) {
    constexpr std::uint64_t kToleranceNs = 5ull * 1000ull * 1000ull;
    std::vector<ReadyFrame> ready;
    auto image = state->pending_images.begin();
    while (image != state->pending_images.end()) {
      auto best = state->poses.end();
      std::uint64_t best_delta = std::numeric_limits<std::uint64_t>::max();
      for (auto pose = state->poses.begin(); pose != state->poses.end(); ++pose) {
        const std::uint64_t timestamp = *pose->pose.timestamp_ns;
        const std::uint64_t delta = image->timestamp_ns > timestamp
                                        ? image->timestamp_ns - timestamp
                                        : timestamp - image->timestamp_ns;
        if (delta < best_delta) {
          best_delta = delta;
          best = pose;
        }
      }
      if (best != state->poses.end() && best_delta <= kToleranceNs) {
        ready.push_back(ReadyFrame{std::move(*image), *best});
        image = state->pending_images.erase(image);
        continue;
      }
      if (!state->poses.empty() &&
          *state->poses.back().pose.timestamp_ns >
              image->timestamp_ns + kToleranceNs) {
        state->dropped_images.fetch_add(1);
        image = state->pending_images.erase(image);
        continue;
      }
      ++image;
    }
    if (!state->pending_images.empty()) {
      const std::uint64_t oldest = state->pending_images.front().timestamp_ns;
      const std::uint64_t minimum =
          oldest > kToleranceNs ? oldest - kToleranceNs : 0;
      while (!state->poses.empty() &&
             *state->poses.front().pose.timestamp_ns < minimum) {
        state->poses.pop_front();
      }
    }
    return ready;
  }

  static ma_pose_t nativePose(const PoseFrame& pose) {
    ma_pose_t output{};
    output.timestamp_ns = pose.timestamp_ns.value_or(0);
    const std::array<double, 3>& position =
        pose.raw_position_m ? *pose.raw_position_m : pose.position_m;
    output.px = position[0];
    output.py = position[1];
    output.pz = position[2];
    if (pose.orientation_xyzw) {
      output.qx = (*pose.orientation_xyzw)[0];
      output.qy = (*pose.orientation_xyzw)[1];
      output.qz = (*pose.orientation_xyzw)[2];
      output.qw = (*pose.orientation_xyzw)[3];
    }
    output.frame =
        pose.pose_type == "camera" || pose.pose_type_raw == 1
            ? MA_POSE_FRAME_CAMERA
            : MA_POSE_FRAME_BODY;
    output.confidence = pose.confidence;
    output.is_keyframe_hint = pose.is_keyframe ? 1 : 0;
    return output;
  }

  static ma_pixel_format_t nativeFormat(std::uint8_t format) {
    switch (static_cast<RawFormat>(format)) {
      case RawFormat::kGray8:
        return MA_PIXEL_GRAY8;
      case RawFormat::kRGB24:
        return MA_PIXEL_RGB8;
      case RawFormat::kBGR24:
        return MA_PIXEL_BGR8;
      case RawFormat::kRGBA32:
        return MA_PIXEL_RGBA8;
      case RawFormat::kBGRA32:
        return MA_PIXEL_BGRA8;
      case RawFormat::kYUV420SP:
        return MA_PIXEL_YUV420SP;
      case RawFormat::kYUV420P:
        return MA_PIXEL_YUV420P;
      case RawFormat::kUnknown:
      default:
        return MA_PIXEL_GRAY8;
    }
  }

  static ma_image_view_t nativeImage(const RawImageFrame& image) {
    ma_image_view_t output{};
    output.data = image.data.data();
    output.size_bytes = image.data.size();
    output.width = static_cast<int>(image.width);
    output.height = static_cast<int>(image.height);
    output.format = nativeFormat(image.format);
    std::size_t channels = 1;
    if (output.format == MA_PIXEL_RGB8 || output.format == MA_PIXEL_BGR8) {
      channels = 3;
    } else if (output.format == MA_PIXEL_RGBA8 ||
               output.format == MA_PIXEL_BGRA8) {
      channels = 4;
    }
    output.stride_bytes = image.width * channels;
    return output;
  }

  static OccupancyState translateState(ma_occupancy_state_t state) {
    switch (state) {
      case MA_OCCUPANCY_FREE:
        return OccupancyState::kFree;
      case MA_OCCUPANCY_OCCUPIED:
        return OccupancyState::kOccupied;
      case MA_OCCUPANCY_UNKNOWN:
      default:
        return OccupancyState::kUnknown;
    }
  }

  static OccupancyCell translateCell(const ma_occupancy_cell_t& native) {
    OccupancyCell output;
    output.index = {native.index.x, native.index.y, native.index.z};
    output.state = translateState(native.state);
    output.occupancy = native.occupancy;
    output.intensity = native.intensity;
    output.support = native.support;
    return output;
  }

  static OwnedUpdate translateUpdate(const ma_occupancy_update_t& native) {
    OwnedUpdate output;
    output.base_revision = native.base_revision;
    output.revision = native.revision;
    output.timestamp_ns = native.timestamp_ns;
    output.reset = native.reset != 0;
    output.resolution_m = native.resolution_m;
    if (native.has_rolling_center) {
      output.rolling_center = GridPoint3f{native.rolling_center[0],
                                         native.rolling_center[1],
                                         native.rolling_center[2]};
    }
    if (native.has_rolling_radius) {
      output.rolling_radius_m = native.rolling_radius_m;
    }
    output.changes.reserve(native.change_count);
    for (std::size_t i = 0; i < native.change_count; ++i) {
      output.changes.push_back(translateCell(native.changes[i]));
    }
    return output;
  }

  static ma_box3f_t nativeBox(const GridBox3f& box) {
    return ma_box3f_t{box.minimum.x, box.minimum.y, box.minimum.z,
                      box.maximum.x, box.maximum.y, box.maximum.z};
  }

  static void emitUpdate(const std::shared_ptr<State>& state,
                         const OwnedUpdate& owned) {
    UpdateHandler handler;
    {
      std::lock_guard<std::mutex> lock(state->handler_mutex);
      handler = state->handler;
    }
    if (!handler) return;
    OccupancyGridUpdate view;
    view.base_revision = owned.base_revision;
    view.revision = owned.revision;
    view.timestamp_ns = owned.timestamp_ns;
    view.reset = owned.reset;
    view.resolution_m = owned.resolution_m;
    view.rolling_center = owned.rolling_center;
    view.rolling_radius_m = owned.rolling_radius_m;
    view.changes = Span<const OccupancyCell>(owned.changes.data(),
                                             owned.changes.size());
    handler(view);
  }

  std::shared_ptr<MightyClient> client_;
  MightyOccupancyGridOptions options_;
  std::shared_ptr<State> state_;
  MightyClient::Subscription image_subscription_;
  MightyClient::Subscription pose_subscription_;
  MightyClient::Subscription vio_subscription_;
  MightyClient::Subscription reset_subscription_;
};

}  // namespace sdk
}  // namespace mighty_protocol

#endif  // MIGHTY_PROTOCOL_ENABLE_OCCUPANCY_GRID
