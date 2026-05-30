#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "mighty_mapper/mighty_mapper.h"
#include "mighty_sdk.h"

namespace {

using Clock = std::chrono::steady_clock;
using mighty_protocol::RawFormat;
using mighty_protocol::sdk::ImageFrame;
using mighty_protocol::sdk::MightyClient;
using mighty_protocol::sdk::MightyClientOptions;
using mighty_protocol::sdk::MightyWebDevice;
using mighty_protocol::sdk::MightyWebDeviceOptions;
using mighty_protocol::sdk::PoseFrame;
using mighty_protocol::sdk::RawImageFrame;

constexpr size_t kMaxQueuedImagesDefault = 3000;
constexpr size_t kMaxPoseHistory = 3000;
constexpr uint64_t kPoseMaxDeltaNsDefault = 5ull * 1000ull * 1000ull;
constexpr double kAutoExitIdleSecDefault = 5.0;
constexpr int kStartFrameDefault = 12;
constexpr int kUiPanelWidthPx = 180;
constexpr double kFollowBackDistance = 2.4;
constexpr double kFollowUpOffset = 0.9;
constexpr double kFollowSideOffset = 0.35;
constexpr double kFollowPosSmoothRate = 2.5;
constexpr double kFollowHeadingSmoothRate = 1.8;
constexpr double kFollowCamSmoothRate = 2.0;
constexpr double kFollowTargetSmoothRate = 2.5;
constexpr int kPreviewWidthPx = 320;
constexpr int kPreviewHeightPx = 180;
constexpr int kPreviewMarginPx = 12;
constexpr float kBrandBlueR = 0.0f;
constexpr float kBrandBlueG = 153.0f / 255.0f;
constexpr float kBrandBlueB = 1.0f;
constexpr float kBrandRedR = 1.0f;
constexpr float kBrandRedG = 0.0f;
constexpr float kBrandRedB = 85.0f / 255.0f;

struct Options {
  std::string base_url;
  uint64_t pose_max_delta_ns = kPoseMaxDeltaNsDefault;
  double auto_exit_idle_sec = kAutoExitIdleSecDefault;
  size_t max_queued_images = kMaxQueuedImagesDefault;
  int start_frame = kStartFrameDefault;
  int preset = 0;
  int mode = 1;
  int mapper_width = 0;
  int mapper_height = 0;
  float point_density = 0.0f;
  float candidate_density = 0.0f;
  int min_frames = 0;
  int max_frames = 0;
  int max_opt_iterations = -1;
  int min_opt_iterations = -1;
  int pyramid_levels = 0;
  int point_stride = 1;
  bool auto_exit_on_idle = false;
  bool profile = false;
  bool quiet = false;
  bool follow_pose = false;
};

struct Calibration {
  bool ready = false;
  bool intrinsics_ready = false;
  bool distortion_ready = false;
  bool extrinsics_ready = false;
  std::string camera_model;
  std::string distortion_model;
  cv::Vec4d intrinsics{0.0, 0.0, 0.0, 0.0};
  cv::Vec4d distortion{0.0, 0.0, 0.0, 0.0};
  int source_width = 0;
  int source_height = 0;
  Eigen::Isometry3d T_body_cam = Eigen::Isometry3d::Identity();
};

struct ImageItem {
  uint64_t timestamp_ns = 0;
  int frame_id = 0;
  cv::Mat bgr;
};

struct PoseItem {
  PoseFrame pose;
};

struct SharedState {
  std::mutex mu;
  std::condition_variable cv;
  std::deque<ImageItem> images;
  std::deque<PoseItem> poses;
  Calibration calib;
  bool stop = false;
  bool stream_connected = false;
  bool mapper_ready = false;
  size_t accepted_frames = 0;
  size_t received_images = 0;
  size_t dropped_images = 0;
  size_t pushed_points = 0;
  bool saw_stream_data = false;
  bool mapping_active = false;
  Clock::time_point last_stream_at = Clock::now();
  std::string status = "starting";
  std::shared_ptr<mighty_mapper::Mapper> mapper;
  cv::Mat latest_preview_bgr;
  uint64_t latest_preview_timestamp_ns = 0;
};

std::string lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool starts_with(const std::string& s, const std::string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

bool parse_size(const std::string& value, int* width, int* height) {
  if (!width || !height) return false;
  int w = 0;
  int h = 0;
  if (std::sscanf(value.c_str(), "%dx%d", &w, &h) != 2) return false;
  if (w <= 0 || h <= 0) return false;
  *width = w;
  *height = h;
  return true;
}

double elapsed_ms(Clock::time_point start, Clock::time_point end = Clock::now()) {
  return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
}

struct StageStats {
  size_t count = 0;
  double total_ms = 0.0;
  double max_ms = 0.0;

  void add(double ms) {
    ++count;
    total_ms += ms;
    max_ms = std::max(max_ms, ms);
  }

  double avg() const { return count > 0 ? total_ms / static_cast<double>(count) : 0.0; }

  void reset() {
    count = 0;
    total_ms = 0.0;
    max_ms = 0.0;
  }
};

struct MapperProfileWindow {
  Clock::time_point started_at = Clock::now();
  Clock::time_point window_at = Clock::now();
  size_t window_frames = 0;
  size_t total_frames = 0;
  bool saw_points = false;
  StageStats queue_select;
  StageStats undistort;
  StageStats configure;
  StageStats grayscale;
  StageStats pose_convert;
  StageStats push_frame;
  StageStats snapshot;
  StageStats status_update;
  StageStats total;

  void maybe_report(int frame_id, size_t point_count) {
    ++window_frames;
    ++total_frames;
    if (!saw_points && point_count > 0) {
      saw_points = true;
      std::cerr << "[mapper-profile] first_points frame_id=" << frame_id
                << " accepted=" << total_frames
                << " t=" << elapsed_ms(started_at) * 0.001
                << "s points=" << point_count << "\n";
    }
    if (window_frames < 30 && elapsed_ms(window_at) < 2000.0) return;
    std::cerr << "[mapper-profile] window frames=" << window_frames
              << " last_id=" << frame_id
              << " points=" << point_count
              << " total.avg/max=" << total.avg() << "/" << total.max_ms
              << " queue.avg/max=" << queue_select.avg() << "/" << queue_select.max_ms
              << " undistort.avg/max=" << undistort.avg() << "/" << undistort.max_ms
              << " configure.avg/max=" << configure.avg() << "/" << configure.max_ms
              << " gray.avg/max=" << grayscale.avg() << "/" << grayscale.max_ms
              << " pose.avg/max=" << pose_convert.avg() << "/" << pose_convert.max_ms
              << " push.avg/max=" << push_frame.avg() << "/" << push_frame.max_ms
              << " snapshot.avg/max=" << snapshot.avg() << "/" << snapshot.max_ms
              << " status.avg/max=" << status_update.avg() << "/" << status_update.max_ms
              << "\n";
    window_frames = 0;
    window_at = Clock::now();
    queue_select.reset();
    undistort.reset();
    configure.reset();
    grayscale.reset();
    pose_convert.reset();
    push_frame.reset();
    snapshot.reset();
    status_update.reset();
    total.reset();
  }
};

struct RenderProfileWindow {
  Clock::time_point window_at = Clock::now();
  size_t window_frames = 0;
  StageStats snapshot;
  StageStats clear_activate;
  StageStats grid;
  StageStats points;
  StageStats trajectory;
  StageStats text;
  StageStats finish;
  StageStats total;

  void maybe_report(size_t point_count) {
    ++window_frames;
    if (window_frames < 120 && elapsed_ms(window_at) < 2000.0) return;
    std::cerr << "[render-profile] window frames=" << window_frames
              << " points=" << point_count
              << " total.avg/max=" << total.avg() << "/" << total.max_ms
              << " snapshot.avg/max=" << snapshot.avg() << "/" << snapshot.max_ms
              << " clear_activate.avg/max=" << clear_activate.avg() << "/" << clear_activate.max_ms
              << " grid.avg/max=" << grid.avg() << "/" << grid.max_ms
              << " points.avg/max=" << points.avg() << "/" << points.max_ms
              << " trajectory.avg/max=" << trajectory.avg() << "/" << trajectory.max_ms
              << " text.avg/max=" << text.avg() << "/" << text.max_ms
              << " finish.avg/max=" << finish.avg() << "/" << finish.max_ms
              << "\n";
    window_frames = 0;
    window_at = Clock::now();
    snapshot.reset();
    clear_activate.reset();
    grid.reset();
    points.reset();
    trajectory.reset();
    text.reset();
    finish.reset();
    total.reset();
  }
};

struct FollowCameraState {
  bool has_target = false;
  bool has_heading = false;
  bool has_camera_position = false;
  double zoom_scale = 1.0;
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  Eigen::Vector3d heading = Eigen::Vector3d::UnitX();
  Eigen::Vector3d camera_position = Eigen::Vector3d::Zero();

  void reset() {
    has_target = false;
    has_heading = false;
    has_camera_position = false;
    zoom_scale = 1.0;
    target.setZero();
    heading = Eigen::Vector3d::UnitX();
    camera_position.setZero();
  }
};

bool parse_args(int argc, char** argv, Options* opts) {
  if (!opts) return false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") return false;
    if (starts_with(arg, "--base-url=")) {
      opts->base_url = arg.substr(std::string("--base-url=").size());
    } else if (arg == "--base-url" && i + 1 < argc) {
      opts->base_url = argv[++i];
    } else if (starts_with(arg, "--pose-max-dt-ms=")) {
      const double ms = std::max(1.0, std::stod(arg.substr(std::string("--pose-max-dt-ms=").size())));
      opts->pose_max_delta_ns = static_cast<uint64_t>(ms * 1e6);
    } else if (starts_with(arg, "--idle-exit-sec=")) {
      opts->auto_exit_idle_sec =
          std::max(0.5, std::stod(arg.substr(std::string("--idle-exit-sec=").size())));
    } else if (starts_with(arg, "--start-frame=")) {
      opts->start_frame = std::max(0, std::stoi(arg.substr(std::string("--start-frame=").size())));
    } else if (starts_with(arg, "--max-queued-images=")) {
      opts->max_queued_images = std::max<size_t>(
          1, static_cast<size_t>(std::stoull(arg.substr(std::string("--max-queued-images=").size()))));
    } else if (arg == "--no-auto-exit") {
      opts->auto_exit_on_idle = false;
    } else if (arg == "--auto-exit") {
      opts->auto_exit_on_idle = true;
    } else if (arg == "--profile") {
      opts->profile = true;
    } else if (arg == "--quiet") {
      opts->quiet = true;
    } else if (arg == "--follow") {
      opts->follow_pose = true;
    } else if (starts_with(arg, "--preset=")) {
      opts->preset = std::stoi(arg.substr(std::string("--preset=").size()));
    } else if (starts_with(arg, "--mode=")) {
      opts->mode = std::stoi(arg.substr(std::string("--mode=").size()));
    } else if (starts_with(arg, "--mapper-size=")) {
      if (!parse_size(arg.substr(std::string("--mapper-size=").size()),
                      &opts->mapper_width, &opts->mapper_height)) {
        std::cerr << "invalid --mapper-size, expected WIDTHxHEIGHT\n";
        return false;
      }
    } else if (starts_with(arg, "--mapper-width=")) {
      opts->mapper_width =
          std::max(1, std::stoi(arg.substr(std::string("--mapper-width=").size())));
    } else if (starts_with(arg, "--mapper-height=")) {
      opts->mapper_height =
          std::max(1, std::stoi(arg.substr(std::string("--mapper-height=").size())));
    } else if (starts_with(arg, "--point-density=")) {
      opts->point_density =
          std::max(1.0f, std::stof(arg.substr(std::string("--point-density=").size())));
    } else if (starts_with(arg, "--candidate-density=")) {
      opts->candidate_density =
          std::max(1.0f, std::stof(arg.substr(std::string("--candidate-density=").size())));
    } else if (starts_with(arg, "--min-frames=")) {
      opts->min_frames =
          std::max(2, std::stoi(arg.substr(std::string("--min-frames=").size())));
    } else if (starts_with(arg, "--max-frames=")) {
      opts->max_frames =
          std::max(2, std::stoi(arg.substr(std::string("--max-frames=").size())));
    } else if (starts_with(arg, "--max-opt=")) {
      opts->max_opt_iterations =
          std::max(0, std::stoi(arg.substr(std::string("--max-opt=").size())));
    } else if (starts_with(arg, "--min-opt=")) {
      opts->min_opt_iterations =
          std::max(0, std::stoi(arg.substr(std::string("--min-opt=").size())));
    } else if (starts_with(arg, "--pyr-levels=")) {
      opts->pyramid_levels =
          std::max(1, std::stoi(arg.substr(std::string("--pyr-levels=").size())));
    } else if (starts_with(arg, "--point-stride=")) {
      opts->point_stride = std::max(1, std::stoi(arg.substr(std::string("--point-stride=").size())));
    } else {
      std::cerr << "unknown option: " << arg << "\n";
      return false;
    }
  }
  if ((opts->mapper_width > 0) != (opts->mapper_height > 0)) {
    std::cerr << "--mapper-width and --mapper-height must be provided together\n";
    return false;
  }
  return true;
}

void print_usage() {
  std::cout
      << "Usage:\n"
      << "  ./build/mighty_mapper_live [--base-url http://127.0.0.1:8084]\n\n"
      << "Run VIO separately, then start this example. The viewer uses Pangolin,\n"
      << "white background, blue map points, and red pose trajectory.\n\n"
      << "Options:\n"
      << "  --pose-max-dt-ms=N  max image/pose timestamp delta (default 5)\n"
      << "  --start-frame=N     first stream image id pushed to mapper (default 12)\n"
      << "  --max-queued-images=N\n"
      << "                       image queue before dropping old frames (default 3000)\n"
      << "  --profile          print mapper/render timing windows to stderr\n"
      << "  --quiet            suppress most core mapper logs\n"
      << "  --follow           start viewer with trajectory follow mode enabled\n"
      << "  --preset=N         mapper preset (default 0; use 2 for fast)\n"
      << "  --mode=N           photometric mode (default 1; use 2 for affine-off)\n"
      << "  --mapper-size=WxH  resize frames before mapper, e.g. 320x200\n"
      << "  --point-density=N  override active point target\n"
      << "  --candidate-density=N\n"
      << "                       override candidate point target\n"
      << "  --min-frames=N     override active window min frames\n"
      << "  --max-frames=N     override active window max frames\n"
      << "  --min-opt=N        override min optimizer iterations\n"
      << "  --max-opt=N        override max optimizer iterations\n"
      << "  --pyr-levels=N     cap mapper pyramid levels\n"
      << "  --auto-exit        close after idle stream timeout\n"
      << "  --idle-exit-sec=N   idle timeout for --auto-exit (default 5)\n"
      << "  --no-auto-exit      keep viewer open until closed (default)\n";
}

bool is_primary_channel(const std::string& channel_or_alias) {
  const std::string s = lower_copy(channel_or_alias);
  return s == "cam0" || s == "preview" || s == "left";
}

const RawImageFrame* pick_render_frame(const ImageFrame& image) {
  const RawImageFrame* left = &image.left;
  const RawImageFrame* right = image.right ? &image.right.value() : nullptr;
  const auto name = [](const RawImageFrame* f) {
    if (!f) return std::string();
    return f->channel_alias.empty() ? f->channel : f->channel_alias;
  };
  if (left && is_primary_channel(name(left))) return left;
  if (right && is_primary_channel(name(right))) return right;
  return left ? left : right;
}

bool decode_raw_to_bgr(const RawImageFrame& raw, cv::Mat* out) {
  if (!out || raw.width == 0 || raw.height == 0) return false;
  const int width = static_cast<int>(raw.width);
  const int height = static_cast<int>(raw.height);
  const size_t pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
  const auto fmt = static_cast<RawFormat>(raw.format);

  if (fmt == RawFormat::kGray8) {
    if (raw.data.size() < pixels) return false;
    cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t*>(raw.data.data()));
    cv::cvtColor(gray, *out, cv::COLOR_GRAY2BGR);
    return true;
  }
  if (fmt == RawFormat::kBGR24) {
    if (raw.data.size() < pixels * 3) return false;
    cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8_t*>(raw.data.data()));
    *out = bgr.clone();
    return true;
  }
  if (fmt == RawFormat::kRGB24) {
    if (raw.data.size() < pixels * 3) return false;
    cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t*>(raw.data.data()));
    cv::cvtColor(rgb, *out, cv::COLOR_RGB2BGR);
    return true;
  }
  if (fmt == RawFormat::kRGBA32) {
    if (raw.data.size() < pixels * 4) return false;
    cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(raw.data.data()));
    cv::cvtColor(rgba, *out, cv::COLOR_RGBA2BGR);
    return true;
  }
  if (fmt == RawFormat::kBGRA32) {
    if (raw.data.size() < pixels * 4) return false;
    cv::Mat bgra(height, width, CV_8UC4, const_cast<uint8_t*>(raw.data.data()));
    cv::cvtColor(bgra, *out, cv::COLOR_BGRA2BGR);
    return true;
  }
  if (fmt == RawFormat::kYUV420P || fmt == RawFormat::kYUV420SP) {
    if (raw.data.size() < pixels) return false;
    cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t*>(raw.data.data()));
    cv::cvtColor(gray, *out, cv::COLOR_GRAY2BGR);
    return true;
  }
  return false;
}

bool read_intrinsics_node(const cv::FileNode& node,
                          const std::string& camera_model,
                          cv::Vec4d* out) {
  if (!out || node.empty()) return false;
  if (node.isSeq() && node.size() >= 4) {
    if (lower_copy(camera_model) == "ds" && node.size() >= 6) {
      *out = cv::Vec4d(static_cast<double>(node[2]),
                       static_cast<double>(node[3]),
                       static_cast<double>(node[4]),
                       static_cast<double>(node[5]));
      return true;
    }
    *out = cv::Vec4d(static_cast<double>(node[0]),
                     static_cast<double>(node[1]),
                     static_cast<double>(node[2]),
                     static_cast<double>(node[3]));
    return true;
  }
  if (node.isMap()) {
    const cv::FileNode fx = node["fx"];
    const cv::FileNode fy = node["fy"];
    const cv::FileNode cx = node["cx"];
    const cv::FileNode cy = node["cy"];
    if (!fx.empty() && !fy.empty() && !cx.empty() && !cy.empty()) {
      *out = cv::Vec4d(static_cast<double>(fx), static_cast<double>(fy),
                       static_cast<double>(cx), static_cast<double>(cy));
      return true;
    }
  }
  return false;
}

bool read_vec4_node(const cv::FileNode& node, cv::Vec4d* out) {
  if (!out || node.empty()) return false;
  if (node.isSeq() && node.size() >= 4) {
    *out = cv::Vec4d(static_cast<double>(node[0]), static_cast<double>(node[1]),
                     static_cast<double>(node[2]), static_cast<double>(node[3]));
    return true;
  }
  if (node.isMap()) {
    const cv::FileNode k1 = node["k1"];
    const cv::FileNode k2 = node["k2"];
    const cv::FileNode p1 = node["p1"];
    const cv::FileNode p2 = node["p2"];
    const cv::FileNode xi = node["xi"];
    const cv::FileNode alpha = node["alpha"];
    if (!xi.empty() && !alpha.empty()) {
      *out = cv::Vec4d(static_cast<double>(xi), static_cast<double>(alpha), 0.0, 0.0);
      return true;
    }
    if (!k1.empty() && !k2.empty() && !p1.empty() && !p2.empty()) {
      *out = cv::Vec4d(static_cast<double>(k1), static_cast<double>(k2),
                       static_cast<double>(p1), static_cast<double>(p2));
      return true;
    }
  }
  return false;
}

bool read_resolution_node(const cv::FileNode& node, int* width, int* height) {
  if (!width || !height || node.empty() || !node.isSeq() || node.size() < 2) return false;
  *width = static_cast<int>(node[0]);
  *height = static_cast<int>(node[1]);
  return *width > 0 && *height > 0;
}

bool read_transform44_node(const cv::FileNode& node, Eigen::Isometry3d* out) {
  if (!out || node.empty()) return false;
  cv::Mat raw;
  if (node.isMap()) {
    try {
      node >> raw;
    } catch (const cv::Exception&) {
      raw.release();
    }
  }
  if (raw.empty() && node.isSeq() && node.size() == 4) {
    raw = cv::Mat::zeros(4, 4, CV_64F);
    for (int r = 0; r < 4; ++r) {
      const cv::FileNode row = node[r];
      if (!row.isSeq() || row.size() < 4) return false;
      for (int c = 0; c < 4; ++c) raw.at<double>(r, c) = static_cast<double>(row[c]);
    }
  }
  if (raw.empty() && node.isSeq() && node.size() == 16) {
    raw = cv::Mat::zeros(4, 4, CV_64F);
    for (int i = 0; i < 16; ++i) raw.at<double>(i / 4, i % 4) = static_cast<double>(node[i]);
  }
  if (raw.empty() || raw.rows != 4 || raw.cols != 4) return false;
  if (raw.type() != CV_64F) raw.convertTo(raw, CV_64F);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) T.matrix()(r, c) = raw.at<double>(r, c);
  }
  *out = T;
  return true;
}

Eigen::Matrix3d canonical_base_from_camera_rotation() {
  Eigen::Matrix3d R;
  R << 0.0, 0.0, 1.0,
      -1.0, 0.0, 0.0,
       0.0, -1.0, 0.0;
  return R;
}

Eigen::Isometry3d canonical_body_to_camera_from_t_cam_imu(const Eigen::Isometry3d& T_cam_imu) {
  const Eigen::Isometry3d T_imu_cam = T_cam_imu.inverse();
  const Eigen::Matrix3d R_base_cam = canonical_base_from_camera_rotation();
  const Eigen::Matrix3d R_base_imu = R_base_cam * T_cam_imu.linear();
  Eigen::Isometry3d T_body_cam = Eigen::Isometry3d::Identity();
  T_body_cam.linear() = R_base_cam;
  T_body_cam.translation() = R_base_imu * T_imu_cam.translation();
  return T_body_cam;
}

cv::FileNode node_or_cam0(cv::FileStorage& fs, const std::string& key) {
  cv::FileNode n = fs[key];
  if (!n.empty()) return n;
  cv::FileNode cam0 = fs["cam0"];
  if (!cam0.empty()) return cam0[key];
  return cv::FileNode();
}

Calibration parse_calibration_text(const std::string& yaml_text) {
  Calibration calib;
  if (yaml_text.empty()) return calib;
  cv::FileStorage fs(yaml_text, cv::FileStorage::READ | cv::FileStorage::MEMORY);
  if (!fs.isOpened()) return calib;

  cv::FileNode model_node = node_or_cam0(fs, "camera_model");
  if (!model_node.empty()) calib.camera_model = lower_copy(static_cast<std::string>(model_node));

  calib.intrinsics_ready =
      read_intrinsics_node(node_or_cam0(fs, "intrinsics"), calib.camera_model, &calib.intrinsics);

  (void)read_resolution_node(node_or_cam0(fs, "resolution"),
                             &calib.source_width, &calib.source_height);

  cv::FileNode distortion = node_or_cam0(fs, "distortion_coeffs");
  calib.distortion_ready = read_vec4_node(distortion, &calib.distortion);
  if (!calib.distortion_ready && calib.camera_model == "ds") {
    cv::FileNode intr = node_or_cam0(fs, "intrinsics");
    if (intr.isSeq() && intr.size() >= 2) {
      calib.distortion = cv::Vec4d(static_cast<double>(intr[0]),
                                   static_cast<double>(intr[1]), 0.0, 0.0);
      calib.distortion_ready = true;
    }
  }

  cv::FileNode distortion_model = node_or_cam0(fs, "distortion_model");
  if (!distortion_model.empty()) {
    calib.distortion_model = lower_copy(static_cast<std::string>(distortion_model));
  }

  Eigen::Isometry3d T_cam_imu = Eigen::Isometry3d::Identity();
  cv::FileNode T_node = node_or_cam0(fs, "T_cam_imu");
  if (T_node.empty()) T_node = node_or_cam0(fs, "T_cam_body");
  if (read_transform44_node(T_node, &T_cam_imu)) {
    calib.T_body_cam = canonical_body_to_camera_from_t_cam_imu(T_cam_imu);
    calib.extrinsics_ready = true;
  }

  calib.ready = calib.intrinsics_ready;
  return calib;
}

bool has_nonzero_distortion(const Calibration& calib) {
  if (!calib.distortion_ready) return false;
  return std::abs(calib.distortion[0]) + std::abs(calib.distortion[1]) +
             std::abs(calib.distortion[2]) + std::abs(calib.distortion[3]) >
         1e-12;
}

bool is_double_sphere(const Calibration& calib) {
  return calib.camera_model == "ds" && calib.distortion_ready;
}

cv::Vec4d intrinsics_for_image_size(const Calibration& calib, int width, int height) {
  if (calib.source_width <= 0 || calib.source_height <= 0 || width <= 0 || height <= 0) {
    return calib.intrinsics;
  }
  const double sx = static_cast<double>(width) / static_cast<double>(calib.source_width);
  const double sy = static_cast<double>(height) / static_cast<double>(calib.source_height);
  return cv::Vec4d(calib.intrinsics[0] * sx, calib.intrinsics[1] * sy,
                   calib.intrinsics[2] * sx, calib.intrinsics[3] * sy);
}

cv::Mat build_camera_matrix(const cv::Vec4d& intr) {
  return (cv::Mat_<double>(3, 3) << intr[0], 0.0, intr[2],
                                   0.0, intr[1], intr[3],
                                   0.0, 0.0, 1.0);
}

cv::Mat rectify_double_sphere_to_pinhole(const cv::Mat& image_bgr, const Calibration& calib) {
  const int width = image_bgr.cols;
  const int height = image_bgr.rows;
  const cv::Vec4d intr = intrinsics_for_image_size(calib, width, height);
  const double fx = intr[0];
  const double fy = intr[1];
  const double cx = intr[2];
  const double cy = intr[3];
  const double xi = calib.distortion[0];
  const double alpha = std::clamp(calib.distortion[1], 0.0, 1.0);
  if (fx <= 0.0 || fy <= 0.0) return image_bgr;

  cv::Mat map_x(height, width, CV_32F);
  cv::Mat map_y(height, width, CV_32F);
  for (int y = 0; y < height; ++y) {
    float* mx = map_x.ptr<float>(y);
    float* my = map_y.ptr<float>(y);
    const double yn = (static_cast<double>(y) - cy) / fy;
    for (int x = 0; x < width; ++x) {
      const double xn = (static_cast<double>(x) - cx) / fx;
      const double d1 = std::sqrt(xn * xn + yn * yn + 1.0);
      const double z_xi = xi * d1 + 1.0;
      const double d2 = std::sqrt(xn * xn + yn * yn + z_xi * z_xi);
      const double denom = alpha * d2 + (1.0 - alpha) * z_xi;
      if (denom <= 1e-9 || !std::isfinite(denom)) {
        mx[x] = -1.0f;
        my[x] = -1.0f;
      } else {
        mx[x] = static_cast<float>(fx * xn / denom + cx);
        my[x] = static_cast<float>(fy * yn / denom + cy);
      }
    }
  }
  cv::Mat rectified;
  cv::remap(image_bgr, rectified, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  return rectified.empty() ? image_bgr : rectified;
}

cv::Mat undistort_for_mapper(const cv::Mat& image_bgr, const Calibration& calib) {
  if (image_bgr.empty()) return image_bgr;
  if (is_double_sphere(calib)) return rectify_double_sphere_to_pinhole(image_bgr, calib);
  if (!has_nonzero_distortion(calib)) return image_bgr;

  const cv::Vec4d intr = intrinsics_for_image_size(calib, image_bgr.cols, image_bgr.rows);
  const cv::Mat K = build_camera_matrix(intr);
  const cv::Mat D = (cv::Mat_<double>(1, 4) << calib.distortion[0], calib.distortion[1],
                                              calib.distortion[2], calib.distortion[3]);
  cv::Mat out;
  if (calib.distortion_model == "equidistant") {
    cv::fisheye::undistortImage(image_bgr, out, K, D, K);
  } else {
    cv::undistort(image_bgr, out, K, D, K);
  }
  return out.empty() ? image_bgr : out;
}

bool pose_is_body_frame(const PoseFrame& pose) {
  return pose.pose_type == "body" || pose.pose_type_raw == 0;
}

bool pose_is_camera_frame(const PoseFrame& pose) {
  return pose.pose_type == "camera" || pose.pose_type_raw == 1;
}

bool pose_can_drive_mapper(const PoseFrame& pose) {
  return pose.is_public && pose.orientation_xyzw.has_value() &&
         (pose_is_body_frame(pose) || pose_is_camera_frame(pose));
}

Eigen::Isometry3d pose_to_world_body(const PoseFrame& pose) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(pose.position_m[0], pose.position_m[1], pose.position_m[2]);
  const auto& qxyzw = pose.orientation_xyzw.value();
  Eigen::Quaterniond q(qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]);
  if (q.norm() > 1e-12) {
    q.normalize();
    T.linear() = q.toRotationMatrix();
  }
  return T;
}

std::optional<Eigen::Isometry3d> pose_to_camera_pose(const PoseFrame& pose,
                                                     const Calibration& calib) {
  if (!pose_can_drive_mapper(pose)) return std::nullopt;
  const Eigen::Isometry3d T_w_b = pose_to_world_body(pose);
  if (pose_is_camera_frame(pose)) return T_w_b;
  if (calib.extrinsics_ready) return T_w_b * calib.T_body_cam;
  return T_w_b;
}

std::optional<PoseFrame> select_pose(const std::deque<PoseItem>& poses,
                                     uint64_t image_ts_ns,
                                     uint64_t max_delta_ns,
                                     double* dt_ms) {
  if (poses.empty()) return std::nullopt;
  uint64_t best_delta = std::numeric_limits<uint64_t>::max();
  const PoseFrame* best = nullptr;
  for (const auto& item : poses) {
    const PoseFrame& pose = item.pose;
    if (!pose_can_drive_mapper(pose) || !pose.timestamp_ns.has_value()) continue;
    const uint64_t pose_ts = pose.timestamp_ns.value();
    const uint64_t delta = pose_ts >= image_ts_ns ? pose_ts - image_ts_ns : image_ts_ns - pose_ts;
    if (delta < best_delta) {
      best_delta = delta;
      best = &pose;
      if (dt_ms) {
        *dt_ms = pose_ts >= image_ts_ns ? static_cast<double>(delta) * 1e-6
                                        : -static_cast<double>(delta) * 1e-6;
      }
    }
  }
  if (!best || best_delta > max_delta_ns) return std::nullopt;
  return *best;
}

mighty_mapper::MapperConfig make_mapper_config(const Calibration& calib,
                                               int width,
                                               int height,
                                               const Options& opts) {
  const cv::Vec4d intr = intrinsics_for_image_size(calib, width, height);
  mighty_mapper::MapperConfig config;
  config.camera.width = width;
  config.camera.height = height;
  config.camera.fx = intr[0];
  config.camera.fy = intr[1];
  config.camera.cx = intr[2];
  config.camera.cy = intr[3];
  config.runtime.preset = opts.preset;
  config.runtime.photometric_mode = opts.mode;
  config.runtime.point_density = opts.point_density;
  config.runtime.candidate_density = opts.candidate_density;
  config.runtime.min_frames = opts.min_frames;
  config.runtime.max_frames = opts.max_frames;
  config.runtime.max_opt_iterations = opts.max_opt_iterations;
  config.runtime.min_opt_iterations = opts.min_opt_iterations;
  config.runtime.pyramid_levels = opts.pyramid_levels;
  config.runtime.enable_display = false;
  config.runtime.quiet = opts.quiet;
  config.pose_prior.enabled = true;
  config.pose_prior.use_metric_initializer_scale = true;
  config.pose_prior.use_backend_prior = true;
  config.pose_prior.metric_initializer_min_baseline_m = 0.10;
  config.pose_prior.backend_prior_translation_weight = 25.0;
  config.pose_prior.backend_prior_rotation_weight = 100.0;
  config.point_cloud.sparsity = 1;
  return config;
}

class MappingActiveGuard {
 public:
  explicit MappingActiveGuard(SharedState* state) : state_(state) {}
  ~MappingActiveGuard() {
    if (!state_) return;
    {
      std::lock_guard<std::mutex> lock(state_->mu);
      state_->mapping_active = false;
    }
    state_->cv.notify_all();
  }

 private:
  SharedState* state_ = nullptr;
};

void mapping_thread(SharedState* state, Options opts) {
  int configured_w = 0;
  int configured_h = 0;
  MapperProfileWindow profile;

  while (true) {
    const auto frame_start = Clock::now();
    ImageItem image;
    Calibration calib;
    PoseFrame pose;
    double dt_ms = 0.0;

    auto queue_start = Clock::now();
    {
      std::unique_lock<std::mutex> lock(state->mu);
      state->cv.wait(lock, [&]() {
        return state->stop || (state->calib.ready && !state->images.empty() && !state->poses.empty());
      });
      if (state->stop) break;

      bool have_item = false;
      while (!state->images.empty()) {
        ImageItem candidate = std::move(state->images.front());
        state->images.pop_front();
        if (candidate.frame_id < opts.start_frame) {
          std::ostringstream ss;
          ss << "skipping frame " << candidate.frame_id << " before start=" << opts.start_frame;
          state->status = ss.str();
          continue;
        }
        auto selected = select_pose(state->poses, candidate.timestamp_ns, opts.pose_max_delta_ns, &dt_ms);
        if (!selected.has_value()) {
          ++state->dropped_images;
          state->status = "dropping image: no close pose";
          continue;
        }
        image = std::move(candidate);
        pose = selected.value();
        calib = state->calib;
        have_item = true;
        state->mapping_active = true;
        break;
      }
      while (!state->poses.empty() && state->poses.front().pose.timestamp_ns.has_value() &&
             !state->images.empty() &&
             state->poses.front().pose.timestamp_ns.value() + 2ull * 1000ull * 1000ull * 1000ull <
                 state->images.front().timestamp_ns) {
        state->poses.pop_front();
      }
      if (!have_item) continue;
    }
    if (opts.profile) profile.queue_select.add(elapsed_ms(queue_start));
    MappingActiveGuard active_guard(state);

    const auto undistort_start = Clock::now();
    cv::Mat mapper_bgr = undistort_for_mapper(image.bgr, calib);
    if (!mapper_bgr.empty() && opts.mapper_width > 0 && opts.mapper_height > 0 &&
        (mapper_bgr.cols != opts.mapper_width || mapper_bgr.rows != opts.mapper_height)) {
      cv::Mat resized;
      cv::resize(mapper_bgr, resized, cv::Size(opts.mapper_width, opts.mapper_height),
                 0.0, 0.0, cv::INTER_AREA);
      mapper_bgr = std::move(resized);
    }
    if (opts.profile) profile.undistort.add(elapsed_ms(undistort_start));
    if (mapper_bgr.empty()) continue;

    std::shared_ptr<mighty_mapper::Mapper> mapper;
    const auto configure_start = Clock::now();
    if (mapper_bgr.cols != configured_w || mapper_bgr.rows != configured_h) {
      configured_w = mapper_bgr.cols;
      configured_h = mapper_bgr.rows;
      auto config = make_mapper_config(calib, configured_w, configured_h, opts);
      mapper = std::make_shared<mighty_mapper::Mapper>(config);
      {
        std::lock_guard<std::mutex> lock(state->mu);
        state->mapper = mapper;
        state->mapper_ready = true;
        std::ostringstream ss;
        ss << "mapper configured " << configured_w << "x" << configured_h
           << " fx=" << config.camera.fx << " fy=" << config.camera.fy;
        state->status = ss.str();
      }
    } else {
      std::lock_guard<std::mutex> lock(state->mu);
      mapper = state->mapper;
    }
    if (opts.profile) profile.configure.add(elapsed_ms(configure_start));
    if (!mapper) continue;

    const auto grayscale_start = Clock::now();
    cv::Mat gray;
    cv::cvtColor(mapper_bgr, gray, cv::COLOR_BGR2GRAY);
    if (!gray.isContinuous()) gray = gray.clone();
    if (opts.profile) profile.grayscale.add(elapsed_ms(grayscale_start));

    const auto pose_start = Clock::now();
    auto T_w_c = pose_to_camera_pose(pose, calib);
    if (opts.profile) profile.pose_convert.add(elapsed_ms(pose_start));
    if (!T_w_c.has_value()) continue;

    mighty_mapper::FrameInput frame;
    frame.frame_id = image.frame_id;
    frame.timestamp_ns = image.timestamp_ns;
    frame.timestamp_seconds = static_cast<double>(image.timestamp_ns) * 1e-9;
    frame.image.data = gray.ptr<uint8_t>(0);
    frame.image.size_bytes = gray.step[0] * gray.rows;
    frame.image.width = gray.cols;
    frame.image.height = gray.rows;
    frame.image.stride_bytes = gray.step[0];
    frame.image.format = mighty_mapper::PixelFormat::kGray8;
    frame.has_camera_pose = true;
    frame.T_odom_camera.q_odom_from_camera = Eigen::Quaterniond(T_w_c->linear());
    frame.T_odom_camera.t_odom_from_camera = T_w_c->translation();
    frame.pose_confidence = pose.confidence;
    frame.keyframe_hint = pose.is_keyframe;

    const auto push_start = Clock::now();
    const auto result = mapper->pushFrame(frame);
    if (opts.profile) profile.push_frame.add(elapsed_ms(push_start));

    const auto snapshot_start = Clock::now();
    const auto snapshot = mapper->snapshot();
    if (opts.profile) profile.snapshot.add(elapsed_ms(snapshot_start));
    const size_t point_count = snapshot.points.size();

    const auto status_start = Clock::now();
    {
      std::lock_guard<std::mutex> lock(state->mu);
      ++state->accepted_frames;
      state->pushed_points = point_count;
      std::ostringstream ss;
      ss << "frames=" << state->accepted_frames << " pts=" << state->pushed_points
         << " id=" << image.frame_id
         << " dt=" << dt_ms << "ms"
         << " init=" << (result.initialized ? "yes" : "no");
      if (state->dropped_images > 0) ss << " dropped=" << state->dropped_images;
      if (result.lost) ss << " lost";
      state->status = ss.str();
    }
    if (opts.profile) {
      profile.status_update.add(elapsed_ms(status_start));
      profile.total.add(elapsed_ms(frame_start));
      profile.maybe_report(image.frame_id, point_count);
    }
  }

  std::shared_ptr<mighty_mapper::Mapper> mapper;
  {
    std::lock_guard<std::mutex> lock(state->mu);
    mapper = state->mapper;
  }
  if (mapper) mapper->finish();
}

Eigen::Vector3d render_from_mapper(const Eigen::Vector3d& p) {
  return Eigen::Vector3d(p.x(), -p.y(), -p.z());
}

pangolin::OpenGlMatrix default_view_matrix() {
  return pangolin::ModelViewLookAt(0.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0);
}

void draw_grid(float extent, float step) {
  glColor3f(kBrandBlueR, kBrandBlueG, kBrandBlueB);
  glLineWidth(1.0f);
  glBegin(GL_LINES);
  for (float v = -extent; v <= extent + 1e-5f; v += step) {
    glVertex3f(-extent, 0.0f, v);
    glVertex3f(extent, 0.0f, v);
    glVertex3f(v, 0.0f, -extent);
    glVertex3f(v, 0.0f, extent);
  }
  glEnd();
}

void draw_trajectory(const std::vector<mighty_mapper::Pose, Eigen::aligned_allocator<mighty_mapper::Pose>>& poses,
                     float r,
                     float g,
                     float b,
                     float y_offset) {
  if (poses.size() < 2) return;
  glColor3f(r, g, b);
  glLineWidth(4.0f);
  glBegin(GL_LINE_STRIP);
  for (const auto& pose : poses) {
    Eigen::Vector3d t = render_from_mapper(pose.t_odom_from_camera);
    t.y() += y_offset;
    glVertex3d(t.x(), t.y(), t.z());
  }
  glEnd();
}

void draw_points(const std::vector<mighty_mapper::MapPoint>& points, int stride) {
  glColor3f(kBrandBlueR, kBrandBlueG, kBrandBlueB);
  glPointSize(2.0f);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < points.size(); i += static_cast<size_t>(std::max(1, stride))) {
    const auto& p = points[i];
    const Eigen::Vector3d q = render_from_mapper(Eigen::Vector3d(p.x, p.y, p.z));
    glVertex3f(q.x(), q.y(), q.z());
  }
  glEnd();
}

void draw_latest_pose_dot(const std::vector<mighty_mapper::Pose, Eigen::aligned_allocator<mighty_mapper::Pose>>& poses) {
  if (poses.empty()) return;
  const Eigen::Vector3d p = render_from_mapper(poses.back().t_odom_from_camera);

  glColor3f(kBrandRedR, kBrandRedG, kBrandRedB);
  glPointSize(14.0f);
  glBegin(GL_POINTS);
  glVertex3d(p.x(), p.y(), p.z());
  glEnd();

  constexpr int kLatSteps = 8;
  constexpr int kLonSteps = 12;
  constexpr double kRadius = 0.075;
  glPushMatrix();
  glTranslated(p.x(), p.y(), p.z());
  for (int i = 0; i < kLatSteps; ++i) {
    const double theta0 = M_PI * static_cast<double>(i) / static_cast<double>(kLatSteps);
    const double theta1 = M_PI * static_cast<double>(i + 1) / static_cast<double>(kLatSteps);
    glBegin(GL_TRIANGLE_STRIP);
    for (int j = 0; j <= kLonSteps; ++j) {
      const double phi = 2.0 * M_PI * static_cast<double>(j) / static_cast<double>(kLonSteps);
      const double c = std::cos(phi);
      const double s = std::sin(phi);
      glVertex3d(kRadius * std::sin(theta0) * c,
                 kRadius * std::sin(theta0) * s,
                 kRadius * std::cos(theta0));
      glVertex3d(kRadius * std::sin(theta1) * c,
                 kRadius * std::sin(theta1) * s,
                 kRadius * std::cos(theta1));
    }
    glEnd();
  }
  glPopMatrix();
}

Eigen::Vector3d horizontal_heading_from_pose(const mighty_mapper::Pose& pose) {
  const Eigen::Vector3d camera_forward_odom =
      pose.q_odom_from_camera.normalized() * Eigen::Vector3d(0.0, 0.0, -1.0);
  Eigen::Vector3d heading = -render_from_mapper(camera_forward_odom);
  heading.y() = 0.0;
  if (heading.squaredNorm() < 1e-9) return Eigen::Vector3d::UnitX();
  return heading.normalized();
}

Eigen::Vector3d lerp_vec3(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double alpha) {
  return a + (b - a) * std::max(0.0, std::min(1.0, alpha));
}

void update_follow_camera(const mighty_mapper::Pose& latest,
                          double dt_sec,
                          FollowCameraState* follow,
                          pangolin::OpenGlRenderState* camera) {
  if (!follow || !camera) return;
  const Eigen::Vector3d latest_target = render_from_mapper(latest.t_odom_from_camera);
  const Eigen::Vector3d latest_heading = horizontal_heading_from_pose(latest);
  const double pos_alpha = 1.0 - std::exp(-std::max(0.0, dt_sec) * kFollowPosSmoothRate);
  const double heading_alpha = 1.0 - std::exp(-std::max(0.0, dt_sec) * kFollowHeadingSmoothRate);
  const double cam_alpha = 1.0 - std::exp(-std::max(0.0, dt_sec) * kFollowCamSmoothRate);
  const double target_alpha = 1.0 - std::exp(-std::max(0.0, dt_sec) * kFollowTargetSmoothRate);

  if (!follow->has_target) {
    follow->target = latest_target;
    follow->has_target = true;
  } else {
    follow->target = lerp_vec3(follow->target, latest_target, pos_alpha);
  }

  if (!follow->has_heading) {
    follow->heading = latest_heading;
    follow->has_heading = true;
  } else {
    follow->heading = lerp_vec3(follow->heading, latest_heading, heading_alpha);
    if (follow->heading.squaredNorm() > 1e-9) follow->heading.normalize();
    else follow->heading = latest_heading;
  }

  const Eigen::Vector3d world_up = Eigen::Vector3d::UnitY();
  Eigen::Vector3d right = follow->heading.cross(world_up);
  if (right.squaredNorm() < 1e-9) right = Eigen::Vector3d::UnitZ();
  else right.normalize();

  const Eigen::Vector3d desired_camera =
      follow->target -
      follow->heading * (kFollowBackDistance * follow->zoom_scale) +
      right * (kFollowSideOffset * follow->zoom_scale) +
      world_up * (kFollowUpOffset * follow->zoom_scale);

  if (!follow->has_camera_position ||
      (follow->camera_position - desired_camera).squaredNorm() > 100.0) {
    follow->camera_position = desired_camera;
    follow->has_camera_position = true;
  } else {
    follow->camera_position = lerp_vec3(follow->camera_position, desired_camera, cam_alpha);
  }

  const Eigen::Vector3d look_target = follow->has_target
      ? lerp_vec3(follow->target, latest_target, target_alpha)
      : latest_target;
  camera->SetModelViewMatrix(pangolin::ModelViewLookAt(
      follow->camera_position.x(), follow->camera_position.y(), follow->camera_position.z(),
      look_target.x(), look_target.y(), look_target.z(),
      pangolin::AxisY));
}

void render_loop(SharedState* state, const Options& opts) {
  pangolin::CreateWindowAndBind("Mighty Mapper Live", 1600, 1000);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(kUiPanelWidthPx));
  pangolin::Var<bool> follow_pose("ui.Follow Pose", opts.follow_pose, true);
  pangolin::Var<bool> reset_view("ui.Reset View", false, false);
  pangolin::Var<int> point_stride("ui.Point Stride", opts.point_stride, 1, 50, false);
  pangolin::Var<int> trajectory_poses("ui.Trajectory Poses", 0);
  pangolin::Var<int> map_points("ui.Map Points", 0);

  pangolin::OpenGlRenderState camera(
      pangolin::ProjectionMatrix(1600, 1000, 900, 900, 800, 500, 0.05, 5000),
      default_view_matrix());
  pangolin::Handler3D handler(camera);
  pangolin::View& view = pangolin::CreateDisplay()
                             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(kUiPanelWidthPx), 1.0,
                                        -static_cast<float>(1600 - kUiPanelWidthPx) / 1000.0f)
                             .SetHandler(&handler);
  pangolin::View& preview_view =
      pangolin::CreateDisplay()
          .SetBounds(pangolin::Attach::ReversePix(kPreviewMarginPx + kPreviewHeightPx),
                     pangolin::Attach::ReversePix(kPreviewMarginPx),
                     pangolin::Attach::ReversePix(kPreviewMarginPx + kPreviewWidthPx),
                     pangolin::Attach::ReversePix(kPreviewMarginPx),
                     static_cast<float>(kPreviewWidthPx) / static_cast<float>(kPreviewHeightPx));
  RenderProfileWindow profile;
  FollowCameraState follow_state;
  std::unique_ptr<pangolin::GlTexture> preview_texture;
  int preview_tex_width = 0;
  int preview_tex_height = 0;
  auto last_render_at = Clock::now();

  while (!pangolin::ShouldQuit()) {
    const auto render_start = Clock::now();
    const double dt_sec = std::min(0.1, elapsed_ms(last_render_at, render_start) * 0.001);
    last_render_at = render_start;
    std::shared_ptr<mighty_mapper::Mapper> mapper;
    std::string status;
    cv::Mat preview_bgr;
    bool should_exit_idle = false;
    {
      std::lock_guard<std::mutex> lock(state->mu);
      mapper = state->mapper;
      status = state->status;
      if (!state->latest_preview_bgr.empty()) state->latest_preview_bgr.copyTo(preview_bgr);
      if (opts.auto_exit_on_idle && state->saw_stream_data && state->images.empty() &&
          !state->mapping_active) {
        const double idle_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                Clock::now() - state->last_stream_at)
                .count();
        if (idle_sec >= opts.auto_exit_idle_sec) {
          state->status = "stream idle; exiting";
          should_exit_idle = true;
        }
      }
    }
    if (should_exit_idle) break;

    mighty_mapper::MapSnapshot snapshot;
    const auto snapshot_start = Clock::now();
    if (mapper) snapshot = mapper->snapshot();
    if (opts.profile) profile.snapshot.add(elapsed_ms(snapshot_start));
    map_points = static_cast<int>(std::min<size_t>(
        snapshot.points.size(), static_cast<size_t>(std::numeric_limits<int>::max())));
    trajectory_poses = static_cast<int>(std::min<size_t>(
        snapshot.trajectory.size(), static_cast<size_t>(std::numeric_limits<int>::max())));

    if (pangolin::Pushed(reset_view)) {
      camera.SetModelViewMatrix(default_view_matrix());
      follow_state.reset();
      follow_pose = false;
    }

    if (follow_pose && !snapshot.trajectory.empty()) {
      update_follow_camera(snapshot.trajectory.back(), dt_sec, &follow_state, &camera);
    } else if (!follow_pose) {
      follow_state.reset();
    }

    const auto clear_start = Clock::now();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    view.Activate(camera);
    if (opts.profile) profile.clear_activate.add(elapsed_ms(clear_start));

    const auto grid_start = Clock::now();
    draw_grid(20.0f, 0.25f);
    if (opts.profile) profile.grid.add(elapsed_ms(grid_start));

    const auto points_start = Clock::now();
    draw_points(snapshot.points, point_stride);
    if (opts.profile) profile.points.add(elapsed_ms(points_start));

    const auto traj_start = Clock::now();
    draw_trajectory(snapshot.trajectory, kBrandRedR, kBrandRedG, kBrandRedB, 0.0f);
    draw_trajectory(snapshot.trajectory, kBrandRedR, kBrandRedG, kBrandRedB, -0.03f);
    draw_latest_pose_dot(snapshot.trajectory);
    if (opts.profile) profile.trajectory.add(elapsed_ms(traj_start));

    const auto text_start = Clock::now();
    pangolin::default_font().Text("%s follow=%s", status.c_str(), follow_pose ? "on" : "off")
        .DrawWindow(kUiPanelWidthPx + 12, 18);
    if (opts.profile) profile.text.add(elapsed_ms(text_start));

    if (!preview_bgr.empty()) {
      if (!preview_texture || preview_tex_width != preview_bgr.cols ||
          preview_tex_height != preview_bgr.rows) {
        preview_texture = std::make_unique<pangolin::GlTexture>(
            preview_bgr.cols,
            preview_bgr.rows,
            GL_RGB,
            false,
            0,
            GL_BGR,
            GL_UNSIGNED_BYTE);
        preview_tex_width = preview_bgr.cols;
        preview_tex_height = preview_bgr.rows;
      }
      preview_texture->Upload(preview_bgr.data, GL_BGR, GL_UNSIGNED_BYTE);
      glDisable(GL_DEPTH_TEST);
      preview_view.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      preview_texture->RenderToViewportFlipY();
      glEnable(GL_DEPTH_TEST);
    }

    const auto finish_start = Clock::now();
    pangolin::FinishFrame();
    if (opts.profile) {
      profile.finish.add(elapsed_ms(finish_start));
      profile.total.add(elapsed_ms(render_start));
      profile.maybe_report(snapshot.points.size());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  {
    std::lock_guard<std::mutex> lock(state->mu);
    state->stop = true;
  }
  state->cv.notify_all();
}

}  // namespace

int main(int argc, char** argv) {
  Options opts;
  if (!parse_args(argc, argv, &opts)) {
    print_usage();
    return 2;
  }

  SharedState state;
  MightyWebDeviceOptions device_opts;
  if (!opts.base_url.empty()) device_opts.base_url = opts.base_url;
  device_opts.read_timeout_ms = 1000;

  auto device = std::make_shared<MightyWebDevice>(device_opts);
  auto client = std::make_shared<MightyClient>(device, MightyClientOptions());

  auto image_sub = client->on_image([&](const ImageFrame& image_frame) {
    const RawImageFrame* raw = pick_render_frame(image_frame);
    if (!raw || raw->timestamp_ns == 0) return;
    cv::Mat bgr;
    if (!decode_raw_to_bgr(*raw, &bgr)) return;

    std::lock_guard<std::mutex> lock(state.mu);
    ImageItem item;
    item.timestamp_ns = raw->timestamp_ns;
    item.frame_id = static_cast<int>(state.received_images++);
    state.latest_preview_bgr = bgr.clone();
    state.latest_preview_timestamp_ns = raw->timestamp_ns;
    item.bgr = std::move(bgr);
    state.images.push_back(std::move(item));
    state.saw_stream_data = true;
    state.last_stream_at = Clock::now();
    while (state.images.size() > opts.max_queued_images) {
      state.images.pop_front();
      ++state.dropped_images;
    }
    state.cv.notify_all();
  });

  auto pose_sub = client->on_pose([&](const PoseFrame& pose) {
    if (!pose_can_drive_mapper(pose)) return;
    std::lock_guard<std::mutex> lock(state.mu);
    state.poses.push_back(PoseItem{pose});
    state.saw_stream_data = true;
    state.last_stream_at = Clock::now();
    while (state.poses.size() > kMaxPoseHistory) state.poses.pop_front();
    state.cv.notify_all();
  });

  auto status_sub = client->on_status([&](const auto& ev) {
    std::lock_guard<std::mutex> lock(state.mu);
    state.status = ev.text;
  });

  auto error_sub = client->on_error([&](const auto& ev) {
    std::lock_guard<std::mutex> lock(state.mu);
    state.status = ev.scope + ": " + ev.message;
  });

  client->connect();

  std::thread calib_thread([&]() {
    while (true) {
      {
        std::lock_guard<std::mutex> lock(state.mu);
        if (state.stop || state.calib.ready) break;
        state.status = "fetching calib";
      }
      auto result = client->config_get_text("calib");
      if (result.ok && result.found) {
        Calibration calib = parse_calibration_text(result.value);
        if (calib.ready) {
          std::lock_guard<std::mutex> lock(state.mu);
          state.calib = calib;
          std::ostringstream ss;
          ss << "calib ready fx=" << calib.intrinsics[0] << " fy=" << calib.intrinsics[1]
             << " model=" << calib.camera_model;
          state.status = ss.str();
          state.cv.notify_all();
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  std::thread worker(mapping_thread, &state, opts);
  render_loop(&state, opts);

  {
    std::lock_guard<std::mutex> lock(state.mu);
    state.stop = true;
  }
  state.cv.notify_all();
  client->disconnect();
  if (calib_thread.joinable()) calib_thread.join();
  if (worker.joinable()) worker.join();

  (void)image_sub;
  (void)pose_sub;
  (void)status_sub;
  (void)error_sub;
  return 0;
}
