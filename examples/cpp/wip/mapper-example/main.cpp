#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "mighty_loopclosure/mighty_loopclosure_device_c.h"
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

constexpr double kAutoExitIdleSecDefault = 5.0;
constexpr int kUiPanelWidthPx = 180;
constexpr double kFollowDefaultZoomScale = 3.0;
constexpr double kFollowBackDistance = 2.4;
constexpr double kFollowUpOffset = 0.9;
constexpr double kFollowSideOffset = 0.35;
constexpr double kFollowPosSmoothRate = 2.5;
constexpr double kFollowHeadingSmoothRate = 1.8;
constexpr double kFollowCamSmoothRate = 2.0;
constexpr double kFollowTargetSmoothRate = 2.5;
constexpr int kPreviewWidthPx = 1280;
constexpr int kPreviewHeightPx = 720;
constexpr int kPreviewMarginPx = 12;
constexpr float kBrandBlueR = 0.0f;
constexpr float kBrandBlueG = 153.0f / 255.0f;
constexpr float kBrandBlueB = 1.0f;
constexpr float kBrandRedR = 1.0f;
constexpr float kBrandRedG = 0.0f;
constexpr float kBrandRedB = 85.0f / 255.0f;

struct Options {
  std::string base_url;
  double auto_exit_idle_sec = kAutoExitIdleSecDefault;
  int point_stride = 1;
  bool auto_exit_on_idle = false;
  bool profile = false;
  bool quiet = false;
  bool follow_pose = true;
  std::string dump_map_path;
  size_t dump_after_processed = 350;
};

struct SharedState {
  std::mutex mu;
  std::condition_variable cv;
  bool stop = false;
  bool stream_connected = false;
  size_t accepted_frames = 0;
  size_t received_images = 0;
  size_t dropped_images = 0;
  size_t pushed_points = 0;
  bool saw_stream_data = false;
  Clock::time_point last_stream_at = Clock::now();
  std::string status = "starting";
  mmp_device_mapper_t* mapper = nullptr;
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

uint32_t fnv1a32(const uint8_t* data, size_t size) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < size; ++i) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
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
  StageStats map_update;
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
              << " map_update.avg/max=" << map_update.avg() << "/" << map_update.max_ms
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
    map_update.reset();
    status_update.reset();
    total.reset();
  }
};

struct RenderProfileWindow {
  Clock::time_point window_at = Clock::now();
  size_t window_frames = 0;
  StageStats map_update;
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
              << " map_update.avg/max=" << map_update.avg() << "/" << map_update.max_ms
              << " clear_activate.avg/max=" << clear_activate.avg() << "/" << clear_activate.max_ms
              << " grid.avg/max=" << grid.avg() << "/" << grid.max_ms
              << " points.avg/max=" << points.avg() << "/" << points.max_ms
              << " trajectory.avg/max=" << trajectory.avg() << "/" << trajectory.max_ms
              << " text.avg/max=" << text.avg() << "/" << text.max_ms
              << " finish.avg/max=" << finish.avg() << "/" << finish.max_ms
              << "\n";
    window_frames = 0;
    window_at = Clock::now();
    map_update.reset();
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
  double zoom_scale = kFollowDefaultZoomScale;
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  Eigen::Vector3d heading = Eigen::Vector3d::UnitX();
  Eigen::Vector3d camera_position = Eigen::Vector3d::Zero();

  void reset() {
    has_target = false;
    has_heading = false;
    has_camera_position = false;
    zoom_scale = kFollowDefaultZoomScale;
    target.setZero();
    heading = Eigen::Vector3d::UnitX();
    camera_position.setZero();
  }
};

struct RenderMap {
  std::map<int, std::vector<mmp_map_point_t>> frames;
  std::vector<mmp_pose_sample_t> trajectory;
  uint64_t revision = 0;
  size_t point_count = 0;

  void reset() {
    frames.clear();
    trajectory.clear();
    revision = 0;
    point_count = 0;
  }

  void replace_frame(int frame_id, const mmp_map_point_t* points, size_t count) {
    auto& dst = frames[frame_id];
    point_count -= dst.size();
    dst.assign(points, points + count);
    point_count += dst.size();
  }

  void remove_frame(int frame_id) {
    auto it = frames.find(frame_id);
    if (it == frames.end()) return;
    point_count -= it->second.size();
    frames.erase(it);
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
    } else if (starts_with(arg, "--idle-exit-sec=")) {
      opts->auto_exit_idle_sec =
          std::max(0.5, std::stod(arg.substr(std::string("--idle-exit-sec=").size())));
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
    } else if (arg == "--no-follow") {
      opts->follow_pose = false;
    } else if (starts_with(arg, "--dump-map=")) {
      opts->dump_map_path = arg.substr(std::string("--dump-map=").size());
    } else if (arg == "--dump-map" && i + 1 < argc) {
      opts->dump_map_path = argv[++i];
    } else if (starts_with(arg, "--dump-after-processed=")) {
      opts->dump_after_processed = std::max<size_t>(
          1, static_cast<size_t>(std::stoull(
                 arg.substr(std::string("--dump-after-processed=").size()))));
    } else if (arg == "--dump-after-processed" && i + 1 < argc) {
      opts->dump_after_processed =
          std::max<size_t>(1, static_cast<size_t>(std::stoull(argv[++i])));
    } else if (starts_with(arg, "--point-stride=")) {
      opts->point_stride = std::max(1, std::stoi(arg.substr(std::string("--point-stride=").size())));
    } else {
      std::cerr << "unknown option: " << arg << "\n";
      return false;
    }
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
      << "  --profile          print mapper/render timing windows to stderr\n"
      << "  --quiet            suppress most core mapper logs\n"
      << "  --follow           start viewer with trajectory follow mode enabled (default)\n"
      << "  --no-follow        start viewer with trajectory follow mode disabled\n"
      << "  --point-stride=N   render every Nth map point (default 1)\n"
      << "  --dump-map=PATH    run headless and write final raw map points CSV\n"
      << "  --dump-after-processed=N\n"
      << "                     processed-frame cutoff for --dump-map (default 350)\n"
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

uint8_t to_mlc_raw_format(uint8_t raw_format) {
  switch (static_cast<RawFormat>(raw_format)) {
    case RawFormat::kGray8: return MLC_RAW_GRAY8;
    case RawFormat::kRGB24: return MLC_RAW_RGB24;
    case RawFormat::kBGR24: return MLC_RAW_BGR24;
    case RawFormat::kRGBA32: return MLC_RAW_RGBA32;
    case RawFormat::kBGRA32: return MLC_RAW_BGRA32;
    case RawFormat::kYUV420SP: return MLC_RAW_YUV420SP;
    case RawFormat::kYUV420P: return MLC_RAW_YUV420P;
    case RawFormat::kUnknown:
    default:
      return MLC_RAW_UNKNOWN;
  }
}

mlc_pose_t to_mlc_pose(const PoseFrame& pose) {
  mlc_pose_t out{};
  out.timestamp_ns = pose.timestamp_ns.value_or(0);
  out.px = pose.position_m[0];
  out.py = pose.position_m[1];
  out.pz = pose.position_m[2];
  if (pose.orientation_xyzw.has_value()) {
    const auto& q = pose.orientation_xyzw.value();
    out.qx = q[0];
    out.qy = q[1];
    out.qz = q[2];
    out.qw = q[3];
  } else {
    out.qw = 1.0;
  }
  out.frame = pose_is_camera_frame(pose) ? MLC_POSE_FRAME_CAMERA : MLC_POSE_FRAME_BODY;
  out.confidence = pose.confidence;
  return out;
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

void apply_map_update(mmp_device_mapper_t* mapper, RenderMap* render_map) {
  if (!mapper || !render_map) return;
  mmp_map_update_t update{};
  const mmp_status_t status =
      mmp_map_update(mapper, render_map->revision, render_map->trajectory.size(), &update);
  if (status != MMP_STATUS_OK) {
    mmp_map_update_destroy(&update);
    return;
  }

  if (update.reset) render_map->reset();
  if (update.frames && update.frame_count > 0) {
    for (size_t i = 0; i < update.frame_count; ++i) {
      const mmp_map_frame_update_t& frame = update.frames[i];
      if (frame.remove) {
        render_map->remove_frame(frame.frame_id);
      } else {
        render_map->replace_frame(frame.frame_id, frame.points, frame.point_count);
      }
    }
  }
  if (update.trajectory && update.trajectory_count > 0) {
    if (update.trajectory_start == 0) {
      render_map->trajectory.clear();
    } else if (update.trajectory_start != render_map->trajectory.size()) {
      render_map->trajectory.clear();
      render_map->revision = 0;
      mmp_map_update_destroy(&update);
      return;
    }
    render_map->trajectory.insert(render_map->trajectory.end(),
                                  update.trajectory,
                                  update.trajectory + update.trajectory_count);
  }
  render_map->revision = update.revision;
  mmp_map_update_destroy(&update);
}

bool write_map_csv(const RenderMap& map, const std::string& path) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) return false;
  out << "frame_id,point_index,x,y,z\n";
  out << std::setprecision(9);
  for (const auto& kv : map.frames) {
    const int frame_id = kv.first;
    const auto& points = kv.second;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto& p = points[i];
      out << frame_id << "," << i << ","
          << p.x << "," << p.y << "," << p.z << "\n";
    }
  }
  return true;
}

bool collect_headless(SharedState* state, const Options& opts, RenderMap* out_map) {
  if (!state || !out_map) return false;
  const auto start = Clock::now();
  Clock::time_point last_report = start;
  bool reached_target = false;
  while (true) {
    mmp_device_mapper_t* mapper = nullptr;
    size_t accepted = 0;
    bool saw_stream_data = false;
    bool stop = false;
    Clock::time_point last_stream_at = Clock::now();
    {
      std::lock_guard<std::mutex> lock(state->mu);
      mapper = state->mapper;
      accepted = state->accepted_frames;
      saw_stream_data = state->saw_stream_data;
      stop = state->stop;
      last_stream_at = state->last_stream_at;
    }
    apply_map_update(mapper, out_map);
    if (accepted >= opts.dump_after_processed) {
      reached_target = true;
      break;
    }
    if (stop) break;

    const auto now = Clock::now();
    const double elapsed_sec =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
    if (saw_stream_data &&
        std::chrono::duration_cast<std::chrono::duration<double>>(
            now - last_stream_at).count() > opts.auto_exit_idle_sec) {
      break;
    }
    if (elapsed_sec > 60.0) break;
    if (opts.profile && elapsed_ms(last_report, now) > 1000.0) {
      std::cerr << "[dump-map] processed=" << accepted
                << " points=" << out_map->point_count
                << " frames=" << out_map->frames.size() << "\n";
      last_report = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (int i = 0; i < 10; ++i) {
    apply_map_update(state->mapper, out_map);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return reached_target;
}

void draw_trajectory(const RenderMap& map,
                     float r,
                     float g,
                     float b,
                     float y_offset) {
  if (map.trajectory.size() < 2) return;
  glColor3f(r, g, b);
  glLineWidth(4.0f);
  glBegin(GL_LINE_STRIP);
  for (const auto& pose : map.trajectory) {
    Eigen::Vector3d t = render_from_mapper(Eigen::Vector3d(pose.px, pose.py, pose.pz));
    t.y() += y_offset;
    glVertex3d(t.x(), t.y(), t.z());
  }
  glEnd();
}

void draw_points(const RenderMap& map, int stride) {
  glColor3f(kBrandBlueR, kBrandBlueG, kBrandBlueB);
  glPointSize(2.0f);
  glBegin(GL_POINTS);
  const size_t step = static_cast<size_t>(std::max(1, stride));
  for (const auto& kv : map.frames) {
    const auto& points = kv.second;
    for (size_t i = 0; i < points.size(); i += step) {
      const auto& p = points[i];
      const Eigen::Vector3d q = render_from_mapper(Eigen::Vector3d(p.x, p.y, p.z));
      glVertex3f(q.x(), q.y(), q.z());
    }
  }
  glEnd();
}

void draw_latest_pose_dot(const RenderMap& map) {
  if (map.trajectory.empty()) return;
  const auto& latest = map.trajectory.back();
  const Eigen::Vector3d p = render_from_mapper(Eigen::Vector3d(latest.px, latest.py, latest.pz));

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

Eigen::Vector3d horizontal_heading_from_pose(const mmp_pose_sample_t& pose) {
  const Eigen::Quaterniond q_odom_from_camera(pose.qw, pose.qx, pose.qy, pose.qz);
  const Eigen::Vector3d camera_forward_odom =
      q_odom_from_camera.normalized() * Eigen::Vector3d(0.0, 0.0, -1.0);
  Eigen::Vector3d heading = -render_from_mapper(camera_forward_odom);
  heading.y() = 0.0;
  if (heading.squaredNorm() < 1e-9) return Eigen::Vector3d::UnitX();
  return heading.normalized();
}

Eigen::Vector3d lerp_vec3(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double alpha) {
  return a + (b - a) * std::max(0.0, std::min(1.0, alpha));
}

void update_follow_camera(const mmp_pose_sample_t& latest,
                          double dt_sec,
                          FollowCameraState* follow,
                          pangolin::OpenGlRenderState* camera) {
  if (!follow || !camera) return;
  const Eigen::Vector3d latest_target = render_from_mapper(
      Eigen::Vector3d(latest.px, latest.py, latest.pz));
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
  RenderMap render_map;
  std::unique_ptr<pangolin::GlTexture> preview_texture;
  int preview_tex_width = 0;
  int preview_tex_height = 0;
  auto last_render_at = Clock::now();

  while (!pangolin::ShouldQuit()) {
    const auto render_start = Clock::now();
    const double dt_sec = std::min(0.1, elapsed_ms(last_render_at, render_start) * 0.001);
    last_render_at = render_start;
    mmp_device_mapper_t* mapper = nullptr;
    std::string status;
    cv::Mat preview_bgr;
    bool should_exit_idle = false;
    {
      std::lock_guard<std::mutex> lock(state->mu);
      mapper = state->mapper;
      status = state->status;
      if (!state->latest_preview_bgr.empty()) state->latest_preview_bgr.copyTo(preview_bgr);
      if (opts.auto_exit_on_idle && state->saw_stream_data) {
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

    const auto map_update_start = Clock::now();
    apply_map_update(mapper, &render_map);
    if (opts.profile) profile.map_update.add(elapsed_ms(map_update_start));
    map_points = static_cast<int>(std::min<size_t>(
        render_map.point_count, static_cast<size_t>(std::numeric_limits<int>::max())));
    trajectory_poses = static_cast<int>(std::min<size_t>(
        render_map.trajectory.size(), static_cast<size_t>(std::numeric_limits<int>::max())));

    if (pangolin::Pushed(reset_view)) {
      camera.SetModelViewMatrix(default_view_matrix());
      follow_state.reset();
      follow_pose = false;
    }

    if (follow_pose && !render_map.trajectory.empty()) {
      update_follow_camera(
          render_map.trajectory.back(), dt_sec, &follow_state, &camera);
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
    draw_points(render_map, point_stride);
    if (opts.profile) profile.points.add(elapsed_ms(points_start));

    const auto traj_start = Clock::now();
    draw_trajectory(render_map, kBrandRedR, kBrandRedG, kBrandRedB, 0.0f);
    draw_trajectory(render_map, kBrandRedR, kBrandRedG, kBrandRedB, -0.03f);
    draw_latest_pose_dot(render_map);
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
      profile.maybe_report(render_map.point_count);
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

  mmp_options_t mapper_options{};
  mmp_options_default(&mapper_options);
  mapper_options.quiet = opts.quiet ? 1 : 0;
  mmp_device_mapper_t* mapper = nullptr;
  mmp_status_t mapper_status = mmp_create(&mapper_options, &mapper);
  if (mapper_status != MMP_STATUS_OK || !mapper) {
    std::cerr << "failed to create mapper: " << mmp_status_message(mapper_status) << "\n";
    return 1;
  }
  mapper_status = mmp_initialize(mapper);
  if (mapper_status != MMP_STATUS_OK) {
    std::cerr << "failed to initialize mapper: " << mmp_status_message(mapper_status) << "\n";
    mmp_destroy(mapper);
    return 1;
  }
  state.mapper = mapper;

  std::mutex trace_mu;
  std::ofstream trace_out;
  uint64_t trace_image_seq = 0;
  uint64_t trace_pose_seq = 0;
  if (const char* trace_path = std::getenv("MIGHTY_MAPPER_INPUT_TRACE")) {
    if (*trace_path) {
      trace_out.open(trace_path);
      if (trace_out) {
        trace_out << "event,seq,frame_id,timestamp_ns,width,height,format,channel,size,hash,"
                  << "px,py,pz,qx,qy,qz,qw,pose_frame,confidence\n";
      }
    }
  }

  auto image_sub = client->on_image([&](const ImageFrame& image_frame) {
    const RawImageFrame* raw = pick_render_frame(image_frame);
    if (!raw || raw->timestamp_ns == 0) return;
    cv::Mat bgr;
    if (!decode_raw_to_bgr(*raw, &bgr)) return;

    int frame_id = 0;
    {
      std::lock_guard<std::mutex> lock(state.mu);
      frame_id = static_cast<int>(state.received_images++);
      state.latest_preview_bgr = bgr.clone();
      state.latest_preview_timestamp_ns = raw->timestamp_ns;
      state.saw_stream_data = true;
      state.last_stream_at = Clock::now();
    }

    {
      std::lock_guard<std::mutex> lock(trace_mu);
      if (trace_out) {
        const std::string channel =
            raw->channel_alias.empty() ? raw->channel : raw->channel_alias;
        trace_out << "image," << trace_image_seq++ << "," << frame_id << ","
                  << raw->timestamp_ns << "," << raw->width << "," << raw->height << ","
                  << static_cast<int>(raw->format) << "," << channel << ","
                  << raw->data.size() << ","
                  << fnv1a32(raw->data.data(), raw->data.size())
                  << ",,,,,,,,,\n";
      }
    }

    mlc_raw_image_t input{};
    input.timestamp_ns = raw->timestamp_ns;
    input.frame_id = frame_id;
    input.width = raw->width;
    input.height = raw->height;
    input.format = to_mlc_raw_format(raw->format);
    input.data = raw->data.data();
    input.size_bytes = raw->data.size();
    mmp_push_result_t result{};
    const mmp_status_t status = mmp_push_image(mapper, &input, &result);
    if (status != MMP_STATUS_OK && status != MMP_STATUS_NOT_READY) {
      std::lock_guard<std::mutex> lock(state.mu);
      state.status = std::string("mapper image: ") + mmp_status_message(status);
    } else if (result.version != 0) {
      std::lock_guard<std::mutex> lock(state.mu);
      state.accepted_frames = result.frames_processed;
      state.dropped_images = result.frames_dropped;
      state.pushed_points = result.point_count;
      std::ostringstream ss;
      ss << "frames=" << result.frames_processed << " pts=" << result.point_count;
      if (result.frames_dropped > 0) ss << " dropped=" << result.frames_dropped;
      if (result.initialized) ss << " init=yes";
      if (result.lost) ss << " lost";
      state.status = ss.str();
    }
  });

  auto pose_sub = client->on_pose([&](const PoseFrame& pose) {
    if (!pose_can_drive_mapper(pose)) return;
    {
      std::lock_guard<std::mutex> lock(state.mu);
      state.saw_stream_data = true;
      state.last_stream_at = Clock::now();
    }
    const mlc_pose_t input = to_mlc_pose(pose);
    {
      std::lock_guard<std::mutex> lock(trace_mu);
      if (trace_out) {
        trace_out << std::setprecision(17)
                  << "pose," << trace_pose_seq++ << ",,"
                  << input.timestamp_ns << ",,,,,,,"
                  << input.px << "," << input.py << "," << input.pz << ","
                  << input.qx << "," << input.qy << "," << input.qz << ","
                  << input.qw << "," << static_cast<int>(input.frame) << ","
                  << input.confidence << "\n";
      }
    }
    mmp_push_result_t result{};
    const mmp_status_t status = mmp_push_pose(mapper, &input, &result);
    if (status != MMP_STATUS_OK && status != MMP_STATUS_NOT_READY) {
      std::lock_guard<std::mutex> lock(state.mu);
      state.status = std::string("mapper pose: ") + mmp_status_message(status);
    } else if (result.version != 0) {
      std::lock_guard<std::mutex> lock(state.mu);
      state.accepted_frames = result.frames_processed;
      state.dropped_images = result.frames_dropped;
      state.pushed_points = result.point_count;
    }
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
        if (state.stop) break;
        state.status = "fetching calib";
      }
      auto result = client->config_get_text("calib");
      if (result.ok && result.found) {
        const mmp_status_t status = mmp_set_calibration_yaml(mapper, result.value.c_str());
        if (status == MMP_STATUS_OK) {
          std::lock_guard<std::mutex> lock(state.mu);
          state.status = "calib ready";
          state.cv.notify_all();
          break;
        } else {
          std::lock_guard<std::mutex> lock(state.mu);
          state.status = std::string("calib failed: ") + mmp_status_message(status);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  int exit_code = 0;
  if (!opts.dump_map_path.empty()) {
    RenderMap render_map;
    const bool reached_target = collect_headless(&state, opts, &render_map);
    if (!opts.dump_map_path.empty() && !write_map_csv(render_map, opts.dump_map_path)) {
      std::cerr << "failed to write map CSV: " << opts.dump_map_path << "\n";
      exit_code = 1;
    } else if (!opts.dump_map_path.empty()) {
      std::cerr << "[dump-map] wrote " << render_map.point_count
                << " points across " << render_map.frames.size()
                << " frames to " << opts.dump_map_path
                << " reached_target=" << (reached_target ? "yes" : "no") << "\n";
    }
  } else {
    render_loop(&state, opts);
  }

  {
    std::lock_guard<std::mutex> lock(state.mu);
    state.stop = true;
  }
  state.cv.notify_all();
  client->disconnect();
  if (calib_thread.joinable()) calib_thread.join();
  mmp_finish(mapper);
  mmp_destroy(mapper);
  state.mapper = nullptr;

  (void)image_sub;
  (void)pose_sub;
  (void)status_sub;
  (void)error_sub;
  return exit_code;
}
