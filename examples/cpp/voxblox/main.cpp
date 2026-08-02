#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/select.h>
#include <unistd.h>

#include "mighty_sdk.h"
#include "voxblox_mapper.h"

namespace {

using mighty_protocol::VioStateCode;
using mighty_protocol::sdk::CommandResult;
using mighty_protocol::sdk::MightyClient;
using mighty_protocol::sdk::MightyClientOptions;
using mighty_protocol::sdk::MightyErrorEvent;
using mighty_protocol::sdk::MightyWebDevice;
using mighty_protocol::sdk::MightyWebDeviceOptions;
using mighty_protocol::sdk::PoseFrame;
using mighty_protocol::sdk::ResetEvent;
using mighty_protocol::sdk::VioStateFrame;
using mighty_voxblox::BodyCameraCalibration;
using mighty_voxblox::FusionInput;
using mighty_voxblox::MapperConfig;
using mighty_voxblox::MapperStats;
using mighty_voxblox::VoxbloxMapper;

std::atomic<bool> g_stop{false};

void handle_signal(int) { g_stop.store(true, std::memory_order_relaxed); }

struct Options {
  std::string host;
  MapperConfig mapper;
  double minimum_pose_confidence = 0.5;
  double pose_tolerance_ms = 15.0;
  std::string mesh_path = "mighty_voxblox_mesh.ply";
  std::string map_path = "mighty_voxblox_map.voxblox";
  bool start_vio = false;
  bool leave_depth_on = false;
  bool save_on_exit = true;
};

std::string trim(std::string value) {
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
              value.end());
  return value;
}

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

bool parse_double(const char* text, double* value) {
  if (!text || !value) return false;
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

bool parse_positive_size(const char* text, std::size_t* value) {
  if (!text || !value || text[0] == '-') return false;
  char* end = nullptr;
  errno = 0;
  const unsigned long long parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed == 0 ||
      parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  *value = static_cast<std::size_t>(parsed);
  return true;
}

void print_usage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n\n"
      << "Mighty connection:\n"
      << "  --host URL                  Mighty HTTP base URL\n"
      << "  --start-vio                 Start VIO after enabling depth\n"
      << "  --leave-depth-on            Do not disable depth on exit\n\n"
      << "Fusion:\n"
      << "  --voxel-size M              TSDF voxel size (default 0.05)\n"
      << "  --voxels-per-side N         Power-of-two block side (default 16)\n"
      << "  --truncation-distance M     TSDF truncation distance (default 0.15)\n"
      << "  --min-depth M               Minimum integrated range (default 0.20)\n"
      << "  --max-depth M               Maximum integrated range (default 5.0)\n"
      << "  --stride N                  Depth pixel stride (default 2)\n"
      << "  --integrator NAME           fast, merged, or simple (default fast)\n"
      << "  --queue-size N              Pending fusion frames (default 2)\n"
      << "  --pose-confidence X         Minimum public-pose confidence (default 0.5)\n"
      << "  --pose-tolerance-ms X       Max pose/depth timestamp delta (default 15)\n\n"
      << "Output:\n"
      << "  --mesh PATH                 PLY output path\n"
      << "  --map PATH                  Serialized voxblox layer path\n"
      << "  --no-save-on-exit           Only save when requested interactively\n";
}

bool parse_options(int argc, char** argv, Options* options) {
  if (!options) return false;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index] ? argv[index] : "";
    const auto require_value = [&](const char* name) -> const char* {
      if (index + 1 >= argc) {
        std::cerr << name << " requires a value\n";
        return nullptr;
      }
      return argv[++index];
    };

    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (arg == "--host") {
      const char* value = require_value("--host");
      if (!value) return false;
      options->host = value;
    } else if (arg == "--start-vio") {
      options->start_vio = true;
    } else if (arg == "--leave-depth-on") {
      options->leave_depth_on = true;
    } else if (arg == "--no-save-on-exit") {
      options->save_on_exit = false;
    } else if (arg == "--voxel-size") {
      const char* value = require_value("--voxel-size");
      double parsed = 0.0;
      if (!value || !parse_double(value, &parsed) || parsed <= 0.0) return false;
      options->mapper.voxel_size_m = static_cast<float>(parsed);
    } else if (arg == "--voxels-per-side") {
      const char* value = require_value("--voxels-per-side");
      if (!value ||
          !parse_positive_size(value, &options->mapper.voxels_per_side)) {
        return false;
      }
    } else if (arg == "--truncation-distance") {
      const char* value = require_value("--truncation-distance");
      double parsed = 0.0;
      if (!value || !parse_double(value, &parsed) || parsed <= 0.0) return false;
      options->mapper.truncation_distance_m = static_cast<float>(parsed);
    } else if (arg == "--min-depth") {
      const char* value = require_value("--min-depth");
      double parsed = 0.0;
      if (!value || !parse_double(value, &parsed) || parsed < 0.0) return false;
      options->mapper.min_depth_m = static_cast<float>(parsed);
    } else if (arg == "--max-depth") {
      const char* value = require_value("--max-depth");
      double parsed = 0.0;
      if (!value || !parse_double(value, &parsed) || parsed <= 0.0) return false;
      options->mapper.max_depth_m = static_cast<float>(parsed);
    } else if (arg == "--stride") {
      const char* value = require_value("--stride");
      std::size_t parsed = 0;
      if (!value || !parse_positive_size(value, &parsed) ||
          parsed > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return false;
      }
      options->mapper.pixel_stride = static_cast<int>(parsed);
    } else if (arg == "--queue-size") {
      const char* value = require_value("--queue-size");
      if (!value ||
          !parse_positive_size(value, &options->mapper.max_pending_frames)) {
        return false;
      }
    } else if (arg == "--integrator") {
      const char* value = require_value("--integrator");
      if (!value) return false;
      options->mapper.integrator = lower(value);
    } else if (arg == "--pose-confidence") {
      const char* value = require_value("--pose-confidence");
      if (!value || !parse_double(value, &options->minimum_pose_confidence) ||
          options->minimum_pose_confidence < 0.0 ||
          options->minimum_pose_confidence > 1.0) {
        return false;
      }
    } else if (arg == "--pose-tolerance-ms") {
      const char* value = require_value("--pose-tolerance-ms");
      if (!value || !parse_double(value, &options->pose_tolerance_ms) ||
          options->pose_tolerance_ms < 0.0) {
        return false;
      }
    } else if (arg == "--mesh") {
      const char* value = require_value("--mesh");
      if (!value) return false;
      options->mesh_path = value;
    } else if (arg == "--map") {
      const char* value = require_value("--map");
      if (!value) return false;
      options->map_path = value;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      return false;
    }
  }

  const bool integrator_valid = options->mapper.integrator == "fast" ||
                                options->mapper.integrator == "merged" ||
                                options->mapper.integrator == "simple";
  const bool block_side_valid = options->mapper.voxels_per_side > 0 &&
      (options->mapper.voxels_per_side &
       (options->mapper.voxels_per_side - 1)) == 0;
  if (!integrator_valid) {
    std::cerr << "--integrator must be fast, merged, or simple\n";
    return false;
  }
  if (!block_side_valid) {
    std::cerr << "--voxels-per-side must be a power of two\n";
    return false;
  }
  if (options->mapper.max_depth_m <= options->mapper.min_depth_m) {
    std::cerr << "--max-depth must exceed --min-depth\n";
    return false;
  }
  return true;
}

const char* vio_state_name(int state) {
  switch (static_cast<VioStateCode>(state)) {
    case VioStateCode::kOff: return "OFF";
    case VioStateCode::kInitializing: return "INITIALIZING";
    case VioStateCode::kTracking: return "TRACKING";
    case VioStateCode::kDegraded: return "DEGRADED";
    case VioStateCode::kLost: return "LOST";
    case VioStateCode::kLowLight: return "LOW_LIGHT";
    case VioStateCode::kRecovering: return "RECOVERING";
  }
  return "UNKNOWN";
}

class DepthPoseSynchronizer {
 public:
  struct Stats {
    std::uint64_t poses_received = 0;
    std::uint64_t depth_received = 0;
    std::uint64_t matched_frames = 0;
    std::uint64_t rejected_poses = 0;
    std::uint64_t skipped_not_tracking = 0;
    std::uint64_t unmatched_depth_drops = 0;
    std::size_t pending_depth = 0;
  };

  DepthPoseSynchronizer(VoxbloxMapper* mapper,
                        double minimum_pose_confidence,
                        double tolerance_ms)
      : mapper_(mapper),
        minimum_pose_confidence_(minimum_pose_confidence),
        tolerance_ns_(static_cast<std::uint64_t>(
            std::max(0.0, tolerance_ms) * 1e6)) {}

  void on_vio_state(const VioStateFrame& state) {
    const int code = static_cast<int>(state.state);
    vio_state_.store(code, std::memory_order_relaxed);
    if (code != static_cast<int>(VioStateCode::kTracking)) {
      std::lock_guard<std::mutex> lock(mutex_);
      poses_.clear();
      pending_depth_.clear();
    }
  }

  void on_pose(const PoseFrame& pose) {
    poses_received_.fetch_add(1, std::memory_order_relaxed);
    if (!tracking()) {
      skipped_not_tracking_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (!usable_pose(pose)) {
      rejected_poses_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    std::vector<FusionInput> ready;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      poses_.push_back(pose);
      prune_poses_locked();

      auto depth = pending_depth_.begin();
      while (depth != pending_depth_.end()) {
        PoseFrame matched_pose;
        const MatchResult result = match_locked(*depth, &matched_pose);
        if (result == MatchResult::kMatched) {
          FusionInput input;
          input.depth = std::move(*depth);
          input.pose = std::move(matched_pose);
          ready.push_back(std::move(input));
          depth = pending_depth_.erase(depth);
        } else if (result == MatchResult::kExpired) {
          unmatched_depth_drops_.fetch_add(1, std::memory_order_relaxed);
          depth = pending_depth_.erase(depth);
        } else {
          ++depth;
        }
      }
    }
    submit(std::move(ready));
  }

  void on_depth(const mighty_protocol::DepthFrame& depth) {
    depth_received_.fetch_add(1, std::memory_order_relaxed);
    if (!tracking()) {
      skipped_not_tracking_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (depth.timestamp_ns == 0) {
      unmatched_depth_drops_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    std::optional<FusionInput> ready;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      PoseFrame matched_pose;
      const MatchResult result = match_locked(depth, &matched_pose);
      if (result == MatchResult::kMatched) {
        FusionInput input;
        input.depth = depth;
        input.pose = std::move(matched_pose);
        ready = std::move(input);
      } else if (result == MatchResult::kExpired) {
        unmatched_depth_drops_.fetch_add(1, std::memory_order_relaxed);
      } else {
        pending_depth_.push_back(depth);
        while (pending_depth_.size() > kMaxPendingDepth) {
          pending_depth_.pop_front();
          unmatched_depth_drops_.fetch_add(1, std::memory_order_relaxed);
        }
      }
    }
    if (ready.has_value() && mapper_ && mapper_->enqueue(std::move(*ready))) {
      matched_frames_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    poses_.clear();
    pending_depth_.clear();
  }

  int vio_state() const { return vio_state_.load(std::memory_order_relaxed); }

  Stats stats() const {
    Stats out;
    out.poses_received = poses_received_.load(std::memory_order_relaxed);
    out.depth_received = depth_received_.load(std::memory_order_relaxed);
    out.matched_frames = matched_frames_.load(std::memory_order_relaxed);
    out.rejected_poses = rejected_poses_.load(std::memory_order_relaxed);
    out.skipped_not_tracking =
        skipped_not_tracking_.load(std::memory_order_relaxed);
    out.unmatched_depth_drops =
        unmatched_depth_drops_.load(std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      out.pending_depth = pending_depth_.size();
    }
    return out;
  }

 private:
  enum class MatchResult { kWait, kMatched, kExpired };
  static constexpr std::size_t kMaxPoseHistory = 512;
  static constexpr std::size_t kMaxPendingDepth = 8;
  static constexpr std::uint64_t kPoseHistoryWindowNs = 10'000'000'000ull;

  bool tracking() const {
    return vio_state_.load(std::memory_order_relaxed) ==
           static_cast<int>(VioStateCode::kTracking);
  }

  bool usable_pose(const PoseFrame& pose) const {
    if (!pose.is_public || !pose.timestamp_ns.has_value() ||
        *pose.timestamp_ns == 0 || !pose.orientation_xyzw.has_value()) {
      return false;
    }
    if (!std::isfinite(pose.confidence) ||
        pose.confidence < minimum_pose_confidence_) {
      return false;
    }
    return pose.pose_type == "body" || pose.pose_type == "camera" ||
           pose.pose_type_raw == 0 || pose.pose_type_raw == 1;
  }

  void prune_poses_locked() {
    while (poses_.size() > kMaxPoseHistory) poses_.pop_front();
    std::uint64_t latest = 0;
    for (const PoseFrame& pose : poses_) {
      if (pose.timestamp_ns.has_value()) latest = std::max(latest, *pose.timestamp_ns);
    }
    while (!poses_.empty() && poses_.front().timestamp_ns.has_value() &&
           latest > *poses_.front().timestamp_ns &&
           latest - *poses_.front().timestamp_ns > kPoseHistoryWindowNs) {
      poses_.pop_front();
    }
  }

  MatchResult match_locked(const mighty_protocol::DepthFrame& depth,
                           PoseFrame* matched_pose) const {
    if (!matched_pose || poses_.empty()) return MatchResult::kWait;

    const PoseFrame* best = nullptr;
    std::uint64_t best_delta = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t latest_pose_timestamp = 0;
    for (const PoseFrame& pose : poses_) {
      if (!pose.timestamp_ns.has_value()) continue;
      const std::uint64_t timestamp = *pose.timestamp_ns;
      latest_pose_timestamp = std::max(latest_pose_timestamp, timestamp);
      const std::uint64_t delta = timestamp >= depth.timestamp_ns
          ? timestamp - depth.timestamp_ns
          : depth.timestamp_ns - timestamp;
      if (delta < best_delta) {
        best_delta = delta;
        best = &pose;
      }
    }
    if (!best) return MatchResult::kWait;

    // Waiting until the pose stream reaches the depth timestamp prevents an
    // older near pose from winning just before the exact pose arrives.
    if (best_delta <= tolerance_ns_ &&
        (best_delta == 0 || latest_pose_timestamp >= depth.timestamp_ns)) {
      *matched_pose = *best;
      return MatchResult::kMatched;
    }
    if (latest_pose_timestamp > depth.timestamp_ns &&
        latest_pose_timestamp - depth.timestamp_ns > tolerance_ns_) {
      return MatchResult::kExpired;
    }
    return MatchResult::kWait;
  }

  void submit(std::vector<FusionInput> ready) {
    if (!mapper_) return;
    for (FusionInput& input : ready) {
      if (mapper_->enqueue(std::move(input))) {
        matched_frames_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  VoxbloxMapper* mapper_ = nullptr;
  double minimum_pose_confidence_ = 0.5;
  std::uint64_t tolerance_ns_ = 0;
  std::atomic<int> vio_state_{-1};

  mutable std::mutex mutex_;
  std::deque<PoseFrame> poses_;
  std::deque<mighty_protocol::DepthFrame> pending_depth_;

  std::atomic<std::uint64_t> poses_received_{0};
  std::atomic<std::uint64_t> depth_received_{0};
  std::atomic<std::uint64_t> matched_frames_{0};
  std::atomic<std::uint64_t> rejected_poses_{0};
  std::atomic<std::uint64_t> skipped_not_tracking_{0};
  std::atomic<std::uint64_t> unmatched_depth_drops_{0};
};

void print_command_result(const std::string& command,
                          const CommandResult& result) {
  std::cout << command << ": " << (result.ok ? "ok" : "failed")
            << " (status=" << static_cast<int>(result.status) << ")";
  if (!result.message.empty()) std::cout << " " << result.message;
  std::cout << '\n';
}

void print_controls() {
  std::cout
      << "Commands:\n"
      << "  start | stop | toggle   control VIO through Mighty Protocol\n"
      << "  status                  show connection, synchronization, and map stats\n"
      << "  depth                   query depth-estimation status\n"
      << "  mesh [path]             write a PLY mesh snapshot\n"
      << "  map [path]              write a serialized voxblox TSDF layer\n"
      << "  save                    write both configured outputs\n"
      << "  clear                   clear the local TSDF\n"
      << "  help                    show these commands\n"
      << "  quit                    disable depth and exit\n";
}

void save_mesh(VoxbloxMapper* mapper, const std::string& path) {
  std::string error;
  if (mapper && mapper->save_mesh(path, &error)) {
    std::cout << "wrote mesh: " << path << '\n';
  } else {
    std::cout << "mesh not written: " << error << '\n';
  }
}

void save_map(VoxbloxMapper* mapper, const std::string& path) {
  std::string error;
  if (mapper && mapper->save_map(path, &error)) {
    std::cout << "wrote map: " << path << '\n';
  } else {
    std::cout << "map not written: " << error << '\n';
  }
}

void print_status(const MightyClient& client,
                  const DepthPoseSynchronizer& synchronizer,
                  const VoxbloxMapper& mapper) {
  const auto sync = synchronizer.stats();
  const MapperStats map = mapper.stats();
  std::cout << "connected=" << (client.is_connected() ? "yes" : "no")
            << " vio=" << vio_state_name(synchronizer.vio_state())
            << " poses=" << sync.poses_received
            << " depth=" << sync.depth_received
            << " matched=" << sync.matched_frames
            << " pending_depth=" << sync.pending_depth << '\n'
            << "sync_rejected_poses=" << sync.rejected_poses
            << " sync_not_tracking=" << sync.skipped_not_tracking
            << " sync_unmatched_depth=" << sync.unmatched_depth_drops
            << " mapper_integrated=" << map.integrated_frames
            << " mapper_rejected=" << map.rejected_frames
            << " queue_drops=" << map.queue_drops
            << " blocks=" << map.allocated_blocks
            << " points=" << map.integrated_points << '\n';
}

enum class InputPollResult { kTimeout, kLine, kEof };

InputPollResult poll_stdin(std::string* line) {
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(STDIN_FILENO, &read_set);
  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 200000;
  const int result = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
  if (result < 0) {
    if (errno == EINTR) return InputPollResult::kTimeout;
    return InputPollResult::kEof;
  }
  if (result == 0) return InputPollResult::kTimeout;
  if (!std::getline(std::cin, *line)) return InputPollResult::kEof;
  return InputPollResult::kLine;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse_options(argc, argv, &options)) {
    print_usage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  MightyWebDeviceOptions device_options;
  if (!options.host.empty()) device_options.base_url = options.host;
  auto device = std::make_shared<MightyWebDevice>(device_options);

  MightyClientOptions client_options;
  client_options.auto_reconnect = true;
  client_options.reconnect_delay_ms = 1000;
  MightyClient client(device, client_options);

  client.on_error([](const MightyErrorEvent& error) {
    std::cerr << "[mighty] " << error.scope << " " << error.code << ": "
              << error.message << '\n';
  });
  client.connect();

  const auto connect_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!g_stop.load(std::memory_order_relaxed) && !client.is_connected() &&
         std::chrono::steady_clock::now() < connect_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!client.is_connected()) {
    std::cerr << "Mighty stream did not connect within 10 seconds\n";
    client.disconnect();
    return 1;
  }

  const auto device_info = device->get_info();
  std::cout << "connected to " << device_info.source << '\n';

  BodyCameraCalibration calibration;
  const auto calibration_result = client.config_get_text("calib");
  if (calibration_result.ok && calibration_result.found) {
    calibration = mighty_voxblox::parse_body_camera_calibration(
        calibration_result.value);
  } else {
    calibration.message = "protocol calibration fetch failed: " +
        calibration_result.message;
  }
  std::cout << "calibration: " << calibration.message << '\n';
  if (!calibration.valid) {
    std::cout << "warning: body poses will be rejected; camera-type poses can "
                 "still be fused\n";
  }

  std::unique_ptr<VoxbloxMapper> mapper;
  try {
    mapper = std::make_unique<VoxbloxMapper>(options.mapper, calibration);
  } catch (const std::exception& error) {
    std::cerr << "could not create voxblox mapper: " << error.what() << '\n';
    client.disconnect();
    return 1;
  }

  DepthPoseSynchronizer synchronizer(
      mapper.get(), options.minimum_pose_confidence,
      options.pose_tolerance_ms);

  client.on_pose([&synchronizer](const PoseFrame& pose) {
    synchronizer.on_pose(pose);
  });
  client.on_depth([&synchronizer](const mighty_protocol::DepthFrame& depth) {
    synchronizer.on_depth(depth);
  });
  client.on_vio_state([&synchronizer](const VioStateFrame& state) {
    synchronizer.on_vio_state(state);
  });
  client.on_reset([&synchronizer, &mapper](const ResetEvent&) {
    synchronizer.clear();
    mapper->clear();
    std::cout << "[mighty] VIO reset: cleared local TSDF\n";
  });

  const CommandResult depth_on = client.set_depth_estimation_enabled(true);
  print_command_result("depth on", depth_on);
  if (!depth_on.ok) {
    client.disconnect();
    mapper->stop();
    return 1;
  }

  if (options.start_vio) {
    print_command_result("start VIO", client.start_vio());
  }

  std::cout << "voxblox: voxel=" << options.mapper.voxel_size_m
            << "m truncation=" << options.mapper.truncation_distance_m
            << "m range=[" << options.mapper.min_depth_m << ", "
            << options.mapper.max_depth_m << "]m stride="
            << options.mapper.pixel_stride << " integrator="
            << options.mapper.integrator << '\n';
  print_controls();
  std::cout << "mighty-voxblox> " << std::flush;

  bool stream_was_connected = true;
  while (!g_stop.load(std::memory_order_relaxed)) {
    const bool stream_is_connected = client.is_connected();
    if (!stream_was_connected && stream_is_connected) {
      print_command_result(
          "depth on after reconnect",
          client.set_depth_estimation_enabled(true));
      std::cout << "mighty-voxblox> " << std::flush;
    }
    stream_was_connected = stream_is_connected;

    std::string line;
    const InputPollResult poll_result = poll_stdin(&line);
    if (poll_result == InputPollResult::kTimeout) continue;
    if (poll_result == InputPollResult::kEof) break;

    line = trim(line);
    std::istringstream input(line);
    std::string command;
    input >> command;
    command = lower(command);
    std::string argument;
    std::getline(input, argument);
    argument = trim(argument);

    if (command.empty()) {
      // Just redisplay the prompt.
    } else if (command == "start") {
      print_command_result("start VIO", client.start_vio());
    } else if (command == "stop") {
      print_command_result("stop VIO", client.stop_vio());
    } else if (command == "toggle") {
      const bool is_off = synchronizer.vio_state() ==
          static_cast<int>(VioStateCode::kOff);
      print_command_result(is_off ? "start VIO" : "stop VIO",
                           is_off ? client.start_vio() : client.stop_vio());
    } else if (command == "status") {
      print_status(client, synchronizer, *mapper);
    } else if (command == "depth") {
      print_command_result("depth status", client.depth_estimation_status());
    } else if (command == "mesh") {
      save_mesh(mapper.get(), argument.empty() ? options.mesh_path : argument);
    } else if (command == "map") {
      save_map(mapper.get(), argument.empty() ? options.map_path : argument);
    } else if (command == "save") {
      save_mesh(mapper.get(), options.mesh_path);
      save_map(mapper.get(), options.map_path);
    } else if (command == "clear") {
      synchronizer.clear();
      mapper->clear();
      std::cout << "cleared local TSDF\n";
    } else if (command == "help" || command == "?") {
      print_controls();
    } else if (command == "quit" || command == "exit" || command == "q") {
      break;
    } else {
      std::cout << "unknown command: " << command << " (type help)\n";
    }
    std::cout << "mighty-voxblox> " << std::flush;
  }
  std::cout << '\n';

  if (!options.leave_depth_on && client.is_connected()) {
    print_command_result("depth off", client.set_depth_estimation_enabled(false));
  }
  client.disconnect();
  mapper->stop();

  const MapperStats final_stats = mapper->stats();
  if (options.save_on_exit && final_stats.integrated_frames > 0) {
    save_mesh(mapper.get(), options.mesh_path);
    save_map(mapper.get(), options.map_path);
  }
  print_status(client, synchronizer, *mapper);
  return 0;
}
