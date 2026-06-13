#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "mighty_sdk.h"

namespace {

using Clock = std::chrono::steady_clock;
using mighty_protocol::RawFormat;
using mighty_protocol::sdk::ImageFrame;
using mighty_protocol::sdk::ImuBatch;
using mighty_protocol::sdk::KeyframeEvent;
using mighty_protocol::sdk::MightyClient;
using mighty_protocol::sdk::MightyClientOptions;
using mighty_protocol::sdk::MightyWebDevice;
using mighty_protocol::sdk::PoseFrame;
using mighty_protocol::sdk::RawImageFrame;
using mighty_protocol::sdk::StatusEvent;
using mighty_protocol::sdk::VioStateFrame;

constexpr int kCanvasW = 1540;
constexpr int kCanvasH = 900;
constexpr double kLiveTimeoutSec = 2.5;
constexpr size_t kMaxPosePoints = 3000;
constexpr size_t kMaxImuPoints = 8000;
constexpr double kImuWindowSec = 10.0;

const cv::Scalar kPaper(240, 244, 244);      // #f4f4f0
const cv::Scalar kPanelBg(248, 248, 245);
const cv::Scalar kInk(17, 17, 17);           // #111111
const cv::Scalar kCyan(255, 153, 0);         // #0099ff
const cv::Scalar kMagenta(85, 0, 255);       // #ff0055
const cv::Scalar kYellow(0, 204, 255);       // #ffcc00
const cv::Scalar kLineSoft(210, 210, 210);
const cv::Scalar kBorder(28, 28, 28);

struct ImuPoint {
  double t = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;
};

struct DashboardState {
  std::mutex mu;
  cv::Mat image_bgr;
  std::string image_info = "No frame";
  std::string status_text = "Waiting for data...";
  std::string keyframe_info = "No keyframe";
  std::string host_version = "Unknown";
  std::string last_error;
  int vio_state_code = -1;
  std::string vio_state_text = "STATE_NA";
  double fps_current = std::numeric_limits<double>::quiet_NaN();
  int num_features = -1;
  double pose_confidence = std::numeric_limits<double>::quiet_NaN();
  std::deque<ImuPoint> imu_points;
  bool imu_offset_valid = false;
  double imu_time_offset = 0.0;
  double imu_last_t = 0.0;
  std::vector<cv::Point3f> pose_path;
  cv::Point3d pose_latest{0.0, 0.0, 0.0};
  bool has_pose = false;
  std::array<double, 4> quat_latest{0.0, 0.0, 0.0, 1.0};  // xyzw
  bool has_quat = false;
  bool vio_command_in_flight = false;
  bool saw_data = false;
  Clock::time_point last_data_at = Clock::now();
};

struct DashboardSnapshot {
  bool connected = false;
  bool live = false;
  std::string source;
  std::string image_info;
  cv::Mat image_bgr;
  std::string status_text;
  std::string keyframe_info;
  std::string host_version;
  std::string last_error;
  int vio_state_code = -1;
  std::string vio_state_text;
  double fps_current = std::numeric_limits<double>::quiet_NaN();
  int num_features = -1;
  double pose_confidence = std::numeric_limits<double>::quiet_NaN();
  std::deque<ImuPoint> imu_points;
  std::vector<cv::Point3f> pose_path;
  cv::Point3d pose_latest{0.0, 0.0, 0.0};
  bool has_pose = false;
  std::array<double, 4> quat_latest{0.0, 0.0, 0.0, 1.0};
  bool has_quat = false;
  bool vio_command_in_flight = false;
};

struct UiRuntime {
  std::shared_ptr<MightyClient> client;
  DashboardState* state = nullptr;
  std::atomic<bool> request_toggle_vio{false};
  cv::Rect vio_button_rect;
};

double steady_seconds() {
  const auto now = Clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
}

std::string to_vio_label(int code) {
  switch (code) {
    case 0: return "OFF";
    case 1: return "INITIALIZING";
    case 2: return "TRACKING";
    case 3: return "DEGRADED";
    case 4: return "LOST";
    case 5: return "LOW_LIGHT";
    default: break;
  }
  if (code < 0) return "STATE_NA";
  return "STATE_" + std::to_string(code);
}

bool is_primary_channel(const std::string& channel_or_alias) {
  std::string s = channel_or_alias;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s == "cam0" || s == "preview" || s == "left";
}

const RawImageFrame* pick_render_frame(const ImageFrame& image) {
  if (image.kind == ImageFrame::Kind::kRaw) return &image.left;

  const RawImageFrame* left = &image.left;
  const RawImageFrame* right = image.right ? &image.right.value() : nullptr;
  const auto pick_name = [](const RawImageFrame* f) -> std::string {
    if (!f) return "";
    if (!f->channel_alias.empty()) return f->channel_alias;
    return f->channel;
  };

  if (left && is_primary_channel(pick_name(left))) return left;
  if (right && is_primary_channel(pick_name(right))) return right;
  if (left) return left;
  return right;
}

bool decode_raw_to_bgr(const RawImageFrame& raw, cv::Mat* out) {
  if (!out) return false;
  if (raw.width == 0 || raw.height == 0) return false;

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

cv::Matx33d quat_xyzw_to_rot(const std::array<double, 4>& q) {
  const double x = q[0];
  const double y = q[1];
  const double z = q[2];
  const double w = q[3];
  const double n = x * x + y * y + z * z + w * w;
  if (n < 1e-12) return cv::Matx33d::eye();
  const double s = 2.0 / n;
  const double xx = x * x * s;
  const double yy = y * y * s;
  const double zz = z * z * s;
  const double xy = x * y * s;
  const double xz = x * z * s;
  const double yz = y * z * s;
  const double wx = w * x * s;
  const double wy = w * y * s;
  const double wz = w * z * s;
  return cv::Matx33d(
      1.0 - (yy + zz), xy - wz,         xz + wy,
      xy + wz,         1.0 - (xx + zz), yz - wx,
      xz - wy,         yz + wx,         1.0 - (xx + yy));
}

std::array<double, 4> rot_to_quat_xyzw(const cv::Matx33d& R) {
  const double trace = R(0, 0) + R(1, 1) + R(2, 2);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (R(2, 1) - R(1, 2)) / s;
    y = (R(0, 2) - R(2, 0)) / s;
    z = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
    const double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
    w = (R(2, 1) - R(1, 2)) / s;
    x = 0.25 * s;
    y = (R(0, 1) + R(1, 0)) / s;
    z = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    const double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
    w = (R(0, 2) - R(2, 0)) / s;
    x = (R(0, 1) + R(1, 0)) / s;
    y = 0.25 * s;
    z = (R(1, 2) + R(2, 1)) / s;
  } else {
    const double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
    w = (R(1, 0) - R(0, 1)) / s;
    x = (R(0, 2) + R(2, 0)) / s;
    y = (R(1, 2) + R(2, 1)) / s;
    z = 0.25 * s;
  }
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n > 1e-12) {
    x /= n;
    y /= n;
    z /= n;
    w /= n;
  } else {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    w = 1.0;
  }
  return {x, y, z, w};
}

const cv::Matx33d kRvizFromOdom(
    0.0, -1.0, 0.0,
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0);

cv::Point3d map_pose_position_odom_to_viz(const std::array<double, 3>& p) {
  const cv::Vec3d v(p[0], p[1], p[2]);
  const cv::Vec3d out = kRvizFromOdom * v;
  return cv::Point3d(out[0], out[1], out[2]);
}

std::array<double, 4> map_pose_quat_odom_to_viz(const std::array<double, 4>& q_xyzw) {
  const cv::Matx33d r_odom_body = quat_xyzw_to_rot(q_xyzw);
  const cv::Matx33d r_viz_body = kRvizFromOdom * r_odom_body;
  return rot_to_quat_xyzw(r_viz_body);
}

void mark_data(DashboardState* state) {
  if (!state) return;
  state->saw_data = true;
  state->last_data_at = Clock::now();
}

DashboardSnapshot make_snapshot(DashboardState* state,
                                bool connected,
                                const std::string& source) {
  DashboardSnapshot s;
  s.connected = connected;
  s.source = source;
  const auto now = Clock::now();

  std::lock_guard<std::mutex> lock(state->mu);
  s.image_info = state->image_info;
  s.image_bgr = state->image_bgr.empty() ? cv::Mat() : state->image_bgr.clone();
  s.status_text = state->status_text;
  s.keyframe_info = state->keyframe_info;
  s.host_version = state->host_version;
  s.last_error = state->last_error;
  s.vio_state_code = state->vio_state_code;
  s.vio_state_text = state->vio_state_text;
  s.fps_current = state->fps_current;
  s.num_features = state->num_features;
  s.pose_confidence = state->pose_confidence;
  s.imu_points = state->imu_points;
  s.pose_path = state->pose_path;
  s.pose_latest = state->pose_latest;
  s.has_pose = state->has_pose;
  s.quat_latest = state->quat_latest;
  s.has_quat = state->has_quat;
  s.vio_command_in_flight = state->vio_command_in_flight;
  const double age_s = std::chrono::duration_cast<std::chrono::duration<double>>(now - state->last_data_at).count();
  s.live = connected && state->saw_data && age_s <= kLiveTimeoutSec;
  return s;
}

void draw_panel(cv::Mat& canvas, const cv::Rect& rect) {
  cv::rectangle(canvas, rect, kPanelBg, cv::FILLED);
  cv::rectangle(canvas, rect, kBorder, 2);
}

void draw_status_panel(cv::Mat& canvas,
                       const cv::Rect& rect,
                       const DashboardSnapshot& snap,
                       cv::Rect* out_vio_button_rect) {
  draw_panel(canvas, rect);
  const int x0 = rect.x + 14;
  const int y0 = rect.y + 26;
  const int line_h = 24;
  const int col2 = rect.x + rect.width / 2 + 8;

  std::string conn = "disconnected";
  if (snap.connected && snap.live) conn = "connected";
  else if (snap.connected) conn = "connecting";

  const bool can_stop = snap.vio_state_code > 0;
  const std::string btn_text = snap.vio_command_in_flight
      ? (can_stop ? "Stopping..." : "Starting...")
      : (can_stop ? "Stop VIO" : "Start VIO");

  cv::Rect btn(rect.x + rect.width - 164, rect.y + 10, 150, 34);
  if (out_vio_button_rect) *out_vio_button_rect = btn;
  cv::rectangle(canvas, btn, can_stop ? kMagenta : kCyan, cv::FILLED);
  cv::rectangle(canvas, btn, kInk, 1);
  cv::putText(canvas, btn_text, cv::Point(btn.x + 16, btn.y + 23),
              cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  cv::putText(canvas, "VIO STATUS", cv::Point(x0, y0),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, kInk, 2, cv::LINE_AA);

  cv::putText(canvas, "Connection:", cv::Point(x0, y0 + line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, conn, cv::Point(x0 + 108, y0 + line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, snap.live ? cv::Scalar(20, 140, 20) : kInk, 1, cv::LINE_AA);

  cv::putText(canvas, "Source:", cv::Point(x0, y0 + 2 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, snap.source.empty() ? "(none)" : snap.source.substr(0, 32),
              cv::Point(x0 + 72, y0 + 2 * line_h), cv::FONT_HERSHEY_SIMPLEX, 0.44, kInk, 1, cv::LINE_AA);

  cv::putText(canvas, "VIO:", cv::Point(x0, y0 + 3 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, snap.vio_state_text, cv::Point(x0 + 40, y0 + 3 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);

  cv::putText(canvas, "Status:", cv::Point(x0, y0 + 4 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, snap.status_text.substr(0, 30), cv::Point(x0 + 64, y0 + 4 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.44, kInk, 1, cv::LINE_AA);

  cv::putText(canvas, "Keyframe:", cv::Point(x0, y0 + 5 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, snap.keyframe_info.substr(0, 30), cv::Point(x0 + 86, y0 + 5 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.44, kInk, 1, cv::LINE_AA);

  cv::putText(canvas, "Host:", cv::Point(col2, y0 + line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, snap.host_version.substr(0, 24), cv::Point(col2 + 54, y0 + line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.44, kInk, 1, cv::LINE_AA);

  std::ostringstream fps_ss;
  fps_ss << (std::isfinite(snap.fps_current) ? std::to_string(static_cast<int>(snap.fps_current + 0.5)) : "NA");
  cv::putText(canvas, "FPS:", cv::Point(col2, y0 + 2 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, fps_ss.str(), cv::Point(col2 + 44, y0 + 2 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);

  cv::putText(canvas, "Features:", cv::Point(col2, y0 + 3 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, snap.num_features >= 0 ? std::to_string(snap.num_features) : "NA",
              cv::Point(col2 + 80, y0 + 3 * line_h), cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);

  std::ostringstream conf_ss;
  if (std::isfinite(snap.pose_confidence)) {
    conf_ss << std::fixed << std::setprecision(3) << snap.pose_confidence;
  } else {
    conf_ss << "NA";
  }
  cv::putText(canvas, "Pose Conf:", cv::Point(col2, y0 + 4 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);
  cv::putText(canvas, conf_ss.str(), cv::Point(col2 + 86, y0 + 4 * line_h),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, kInk, 1, cv::LINE_AA);

  if (!snap.last_error.empty()) {
    cv::putText(canvas, "Error: " + snap.last_error.substr(0, 54),
                cv::Point(x0, rect.y + rect.height - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, kMagenta, 1, cv::LINE_AA);
  }
}

void draw_image_panel(cv::Mat& canvas, const cv::Rect& rect, const DashboardSnapshot& snap) {
  draw_panel(canvas, rect);
  cv::putText(canvas, "CAMERA", cv::Point(rect.x + 12, rect.y + 24),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, kInk, 2, cv::LINE_AA);
  cv::putText(canvas, snap.image_info.substr(0, 54), cv::Point(rect.x + 12, rect.y + 48),
              cv::FONT_HERSHEY_SIMPLEX, 0.42, kInk, 1, cv::LINE_AA);

  cv::Rect draw_area(rect.x + 10, rect.y + 58, rect.width - 20, rect.height - 68);
  cv::rectangle(canvas, draw_area, cv::Scalar(232, 232, 232), cv::FILLED);
  cv::rectangle(canvas, draw_area, kLineSoft, 1);

  if (snap.image_bgr.empty()) {
    cv::putText(canvas, "Waiting for image frames...",
                cv::Point(draw_area.x + 14, draw_area.y + draw_area.height / 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, kInk, 1, cv::LINE_AA);
    return;
  }

  const double sx = static_cast<double>(draw_area.width) / static_cast<double>(snap.image_bgr.cols);
  const double sy = static_cast<double>(draw_area.height) / static_cast<double>(snap.image_bgr.rows);
  const double s = std::min(sx, sy);
  const int rw = std::max(1, static_cast<int>(snap.image_bgr.cols * s));
  const int rh = std::max(1, static_cast<int>(snap.image_bgr.rows * s));

  cv::Mat resized;
  cv::resize(snap.image_bgr, resized, cv::Size(rw, rh), 0.0, 0.0, cv::INTER_NEAREST);
  const int ox = draw_area.x + (draw_area.width - rw) / 2;
  const int oy = draw_area.y + (draw_area.height - rh) / 2;
  resized.copyTo(canvas(cv::Rect(ox, oy, rw, rh)));
}

void draw_imu_subplot(cv::Mat& panel,
                      const std::deque<ImuPoint>& samples,
                      bool accel,
                      const std::string& label) {
  panel.setTo(kPanelBg);

  const int width = panel.cols;
  const int height = panel.rows;
  const int header_h = 26;
  const cv::Rect plot(0, header_h, width, std::max(8, height - header_h));

  const double now = steady_seconds();
  const double t_min = now - kImuWindowSec;
  std::vector<const ImuPoint*> visible;
  visible.reserve(samples.size());
  for (const auto& s : samples) {
    if (s.t >= t_min) visible.push_back(&s);
  }

  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  for (const ImuPoint* s : visible) {
    const double a = accel ? s->ax : s->gx;
    const double b = accel ? s->ay : s->gy;
    const double c = accel ? s->az : s->gz;
    min_v = std::min(min_v, std::min(a, std::min(b, c)));
    max_v = std::max(max_v, std::max(a, std::max(b, c)));
  }
  if (!std::isfinite(min_v) || !std::isfinite(max_v)) {
    min_v = -1.0;
    max_v = 1.0;
  }
  if (std::abs(max_v - min_v) < 1e-6) {
    min_v -= 1.0;
    max_v += 1.0;
  }
  const double margin = std::max((max_v - min_v) * 0.1, 1e-3);
  min_v -= margin;
  max_v += margin;
  const double range = std::max(1e-6, max_v - min_v);

  for (int i = 0; i <= 5; ++i) {
    const int x = plot.x + (i * plot.width) / 5;
    cv::line(panel, cv::Point(x, plot.y), cv::Point(x, plot.y + plot.height), kLineSoft, 1, cv::LINE_AA);
  }
  for (int i = 0; i <= 4; ++i) {
    const int y = plot.y + (i * plot.height) / 4;
    cv::line(panel, cv::Point(plot.x, y), cv::Point(plot.x + plot.width, y), kLineSoft, 1, cv::LINE_AA);
  }
  if (min_v < 0.0 && max_v > 0.0) {
    const int zy = plot.y + static_cast<int>((1.0 - ((0.0 - min_v) / range)) * plot.height);
    cv::line(panel, cv::Point(plot.x, zy), cv::Point(plot.x + plot.width, zy), cv::Scalar(150, 150, 150), 2, cv::LINE_AA);
  }

  const std::array<cv::Scalar, 3> colors = {kMagenta, kCyan, kYellow};
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<cv::Point> pts;
    pts.reserve(visible.size());
    for (const ImuPoint* s : visible) {
      const double t_norm = std::clamp((s->t - t_min) / kImuWindowSec, 0.0, 1.0);
      double v = 0.0;
      if (accel) v = axis == 0 ? s->ax : (axis == 1 ? s->ay : s->az);
      else v = axis == 0 ? s->gx : (axis == 1 ? s->gy : s->gz);
      const double y_norm = (v - min_v) / range;
      const int px = plot.x + static_cast<int>(t_norm * plot.width);
      const int py = plot.y + static_cast<int>((1.0 - y_norm) * plot.height);
      pts.emplace_back(px, py);
    }
    if (pts.size() >= 2) {
      cv::polylines(panel, pts, false, colors[axis], 2, cv::LINE_AA);
    }
  }

  cv::putText(panel, label, cv::Point(8, 14), cv::FONT_HERSHEY_SIMPLEX, 0.45, kInk, 1, cv::LINE_AA);
  std::ostringstream range_ss;
  range_ss << std::fixed << std::setprecision(2) << min_v << " to " << max_v
           << (accel ? " m/s^2" : " rad/s");
  cv::putText(panel, range_ss.str(), cv::Point(8, 23), cv::FONT_HERSHEY_SIMPLEX, 0.38, kInk, 1, cv::LINE_AA);
}

void draw_imu_panel(cv::Mat& canvas, const cv::Rect& rect, const DashboardSnapshot& snap) {
  draw_panel(canvas, rect);
  const int split = (rect.height - 32) / 2;
  cv::Rect acc(rect.x + 10, rect.y + 10, rect.width - 20, split - 4);
  cv::Rect gyr(rect.x + 10, rect.y + 18 + split, rect.width - 20, rect.height - split - 28);

  cv::Mat acc_view = canvas(acc);
  cv::Mat gyr_view = canvas(gyr);
  draw_imu_subplot(acc_view, snap.imu_points, true, "ACC");
  draw_imu_subplot(gyr_view, snap.imu_points, false, "GYRO");

  cv::line(canvas,
           cv::Point(rect.x + 10, rect.y + 14 + split),
           cv::Point(rect.x + rect.width - 10, rect.y + 14 + split),
           cv::Scalar(170, 170, 170),
           2,
           cv::LINE_AA);
}

cv::Point2f project_pose_point(const cv::Point3d& p,
                               const cv::Point3d& center,
                               double radius,
                               const cv::Rect& rect) {
  const double yaw = 0.82;    // around Z
  const double pitch = -0.52; // around X
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);

  const cv::Vec3d d(p.x - center.x, p.y - center.y, p.z - center.z);
  const double x1 = cy * d[0] - sy * d[1];
  const double y1 = sy * d[0] + cy * d[1];
  const double z1 = d[2];
  const double y2 = cp * y1 - sp * z1;
  const double z2 = sp * y1 + cp * z1;

  const double r = std::max(radius, 0.5);
  const double px_per_meter = 0.42 * static_cast<double>(std::min(rect.width, rect.height)) / r;
  const double depth = std::max(0.30, 1.0 + 0.18 * (z2 / r));
  const double sx = rect.x + rect.width * 0.52 + (x1 * px_per_meter) / depth;
  const double sy2 = rect.y + rect.height * 0.56 - (y2 * px_per_meter) / depth;
  return cv::Point2f(static_cast<float>(sx), static_cast<float>(sy2));
}

void draw_pose_panel(cv::Mat& canvas, const cv::Rect& rect, const DashboardSnapshot& snap) {
  draw_panel(canvas, rect);
  cv::putText(canvas, "POSE (3D VIEW)", cv::Point(rect.x + 14, rect.y + 26),
              cv::FONT_HERSHEY_SIMPLEX, 0.62, kInk, 2, cv::LINE_AA);

  cv::Rect plot(rect.x + 10, rect.y + 40, rect.width - 20, rect.height - 50);
  cv::rectangle(canvas, plot, cv::Scalar(252, 252, 252), cv::FILLED);
  cv::rectangle(canvas, plot, kLineSoft, 1);

  if (snap.pose_path.empty()) {
    cv::putText(canvas, "Waiting for pose data...", cv::Point(plot.x + 16, plot.y + plot.height / 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, kInk, 1, cv::LINE_AA);
    return;
  }

  cv::Point3d mn(snap.pose_path.front().x, snap.pose_path.front().y, snap.pose_path.front().z);
  cv::Point3d mx = mn;
  for (const auto& p : snap.pose_path) {
    mn.x = std::min<double>(mn.x, p.x);
    mn.y = std::min<double>(mn.y, p.y);
    mn.z = std::min<double>(mn.z, p.z);
    mx.x = std::max<double>(mx.x, p.x);
    mx.y = std::max<double>(mx.y, p.y);
    mx.z = std::max<double>(mx.z, p.z);
  }
  const cv::Point3d center((mn.x + mx.x) * 0.5, (mn.y + mx.y) * 0.5, (mn.z + mx.z) * 0.5);
  const double radius = std::max({0.5, (mx.x - mn.x) * 0.6, (mx.y - mn.y) * 0.6, (mx.z - mn.z) * 0.6});

  auto draw_axis = [&](const cv::Point3d& a, const cv::Point3d& b, const cv::Scalar& color) {
    const cv::Point2f p0 = project_pose_point(a, center, radius, plot);
    const cv::Point2f p1 = project_pose_point(b, center, radius, plot);
    cv::line(canvas, p0, p1, color, 2, cv::LINE_AA);
  };
  draw_axis(center, center + cv::Point3d(radius * 0.7, 0.0, 0.0), kMagenta);
  draw_axis(center, center + cv::Point3d(0.0, radius * 0.7, 0.0), kCyan);
  draw_axis(center, center + cv::Point3d(0.0, 0.0, radius * 0.7), kYellow);

  std::vector<cv::Point> path_2d;
  path_2d.reserve(snap.pose_path.size());
  for (const auto& p : snap.pose_path) {
    const cv::Point2f s = project_pose_point(cv::Point3d(p.x, p.y, p.z), center, radius, plot);
    path_2d.emplace_back(static_cast<int>(s.x), static_cast<int>(s.y));
  }
  if (path_2d.size() >= 2) {
    cv::polylines(canvas, path_2d, false, kCyan, 2, cv::LINE_AA);
  }

  if (snap.has_pose) {
    const cv::Point2f cur = project_pose_point(snap.pose_latest, center, radius, plot);
    cv::circle(canvas, cur, 5, kMagenta, cv::FILLED, cv::LINE_AA);

    if (snap.has_quat) {
      const cv::Matx33d R = quat_xyzw_to_rot(snap.quat_latest);
      const cv::Point3d o = snap.pose_latest;
      const std::array<cv::Point3d, 5> local = {
          cv::Point3d(0.0, 0.0, 0.0),
          cv::Point3d(0.09, 0.06, 0.18),
          cv::Point3d(-0.09, 0.06, 0.18),
          cv::Point3d(-0.09, -0.06, 0.18),
          cv::Point3d(0.09, -0.06, 0.18),
      };
      std::array<cv::Point2f, 5> proj{};
      for (size_t i = 0; i < local.size(); ++i) {
        const cv::Vec3d rv = R * cv::Vec3d(local[i].x, local[i].y, local[i].z);
        const cv::Point3d w(o.x + rv[0], o.y + rv[1], o.z + rv[2]);
        proj[i] = project_pose_point(w, center, radius, plot);
      }
      for (int i = 1; i <= 4; ++i) {
        cv::line(canvas, proj[0], proj[i], kMagenta, 1, cv::LINE_AA);
      }
      for (int i = 1; i <= 4; ++i) {
        const int j = (i == 4) ? 1 : (i + 1);
        cv::line(canvas, proj[i], proj[j], kMagenta, 1, cv::LINE_AA);
      }
    }
  }

  std::ostringstream pose_ss;
  if (snap.has_pose) {
    pose_ss << std::fixed << std::setprecision(2)
            << "pose: " << snap.pose_latest.x << ", " << snap.pose_latest.y << ", " << snap.pose_latest.z;
  } else {
    pose_ss << "pose: NA";
  }
  cv::putText(canvas, pose_ss.str(), cv::Point(rect.x + 14, rect.y + rect.height - 12),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, kInk, 1, cv::LINE_AA);
}

void render_dashboard(cv::Mat& canvas,
                      const DashboardSnapshot& snap,
                      cv::Rect* out_vio_button_rect) {
  canvas.setTo(kPaper);
  const int margin = 12;
  const int left_w = 500;
  const int left_x = margin;
  const int right_x = left_x + left_w + margin;
  const int right_w = canvas.cols - right_x - margin;

  const cv::Rect status(left_x, margin, left_w, 200);
  const cv::Rect image(left_x, status.y + status.height + margin, left_w, 300);
  const int imu_h = canvas.rows - image.y - image.height - 3 * margin;
  const cv::Rect imu(left_x, image.y + image.height + margin, left_w, std::max(220, imu_h));
  const cv::Rect pose(right_x, margin, right_w, canvas.rows - 2 * margin);

  draw_status_panel(canvas, status, snap, out_vio_button_rect);
  draw_image_panel(canvas, image, snap);
  draw_imu_panel(canvas, imu, snap);
  draw_pose_panel(canvas, pose, snap);

  cv::putText(canvas, "Keys: q/esc quit, v toggle VIO",
              cv::Point(16, canvas.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.45, kInk, 1, cv::LINE_AA);
}

void request_vio_toggle(UiRuntime* ui) {
  if (!ui || !ui->client || !ui->state) return;
  int state_code = -1;
  {
    std::lock_guard<std::mutex> lock(ui->state->mu);
    if (ui->state->vio_command_in_flight) return;
    ui->state->vio_command_in_flight = true;
    state_code = ui->state->vio_state_code;
    ui->state->last_error.clear();
  }

  const bool should_stop = state_code > 0;
  const auto result = should_stop ? ui->client->stop_vio() : ui->client->start_vio();

  {
    std::lock_guard<std::mutex> lock(ui->state->mu);
    ui->state->vio_command_in_flight = false;
    if (result.ok) {
      ui->state->status_text = should_stop ? "stop_vio sent" : "start_vio sent";
    } else {
      ui->state->last_error = (should_stop ? "stop_vio failed: " : "start_vio failed: ") + result.message;
    }
  }
}

void on_mouse(int event, int x, int y, int flags, void* userdata) {
  (void)flags;
  auto* ui = static_cast<UiRuntime*>(userdata);
  if (!ui) return;
  if (event == cv::EVENT_LBUTTONUP) {
    if (ui->vio_button_rect.contains(cv::Point(x, y))) {
      ui->request_toggle_vio.store(true);
    }
  }
}

}  // namespace

int main() {
  auto device = std::make_shared<MightyWebDevice>();
  MightyClientOptions opts;
  opts.auto_reconnect = true;
  opts.reconnect_delay_ms = 1000;
  auto client = std::make_shared<MightyClient>(device, opts);

  DashboardState state;

  client->on_image([&state](const ImageFrame& evt) {
    const RawImageFrame* frame = pick_render_frame(evt);
    if (!frame) return;
    cv::Mat bgr;
    if (!decode_raw_to_bgr(*frame, &bgr)) return;
    std::lock_guard<std::mutex> lock(state.mu);
    state.image_bgr = std::move(bgr);
    const std::string channel = !frame->channel_alias.empty() ? frame->channel_alias : frame->channel;
    std::ostringstream ss;
    ss << "raw " << frame->width << "x" << frame->height << " " << channel
       << " " << frame->timestamp_ns;
    state.image_info = ss.str();
    mark_data(&state);
  });

  client->on_pose([&state](const PoseFrame& evt) {
    std::lock_guard<std::mutex> lock(state.mu);
    const cv::Point3d p_viz = map_pose_position_odom_to_viz(evt.position_m);
    state.pose_latest = p_viz;
    state.has_pose = true;
    if (evt.orientation_xyzw.has_value()) {
      state.quat_latest = map_pose_quat_odom_to_viz(evt.orientation_xyzw.value());
      state.has_quat = true;
    } else {
      state.has_quat = false;
    }
    state.pose_path.emplace_back(static_cast<float>(p_viz.x),
                                 static_cast<float>(p_viz.y),
                                 static_cast<float>(p_viz.z));
    if (state.pose_path.size() > kMaxPosePoints) {
      const size_t drop = state.pose_path.size() - kMaxPosePoints;
      state.pose_path.erase(state.pose_path.begin(), state.pose_path.begin() + static_cast<long>(drop));
    }
    if (evt.confidence == evt.confidence) state.pose_confidence = evt.confidence;
    mark_data(&state);
  });

  client->on_keyframe([&state](const KeyframeEvent& evt) {
    std::lock_guard<std::mutex> lock(state.mu);
    std::ostringstream ss;
    ss << evt.timestamp_ns << " dim=" << evt.descriptor.size();
    state.keyframe_info = ss.str();
    mark_data(&state);
  });

  client->on_imu([&state](const ImuBatch& evt) {
    const double now_sec = steady_seconds();
    std::lock_guard<std::mutex> lock(state.mu);
    for (const auto& s : evt.samples) {
      double t = now_sec;
      if (s.timestamp_ns > 0) {
        const double raw_t = static_cast<double>(s.timestamp_ns) * 1e-9;
        if (!state.imu_offset_valid) {
          state.imu_time_offset = now_sec - raw_t;
          state.imu_offset_valid = true;
        }
        t = raw_t + state.imu_time_offset;
      }
      if (t < state.imu_last_t) t = state.imu_last_t;
      state.imu_last_t = t;
      state.imu_points.push_back(ImuPoint{
          t, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
      });
    }
    while (!state.imu_points.empty() && state.imu_points.size() > kMaxImuPoints) {
      state.imu_points.pop_front();
    }
    mark_data(&state);
  });

  client->on_vio_state([&state](const VioStateFrame& evt) {
    std::lock_guard<std::mutex> lock(state.mu);
    state.vio_state_code = static_cast<int>(evt.state);
    state.vio_state_text = to_vio_label(static_cast<int>(evt.state));
    if (evt.light_level01.has_value() && std::isfinite(*evt.light_level01)) {
      const float required = evt.light_required01.value_or(0.0f);
      std::ostringstream light_ss;
      light_ss << std::fixed << std::setprecision(4) << *evt.light_level01 << "/" << required;
      state.vio_state_text += " darkness " + light_ss.str();
    }
    state.fps_current = evt.fps_current;
    state.num_features = static_cast<int>(evt.num_features);
    state.pose_confidence = evt.pose_confidence;
    if (!evt.build_version.empty()) state.host_version = evt.build_version;
    mark_data(&state);
  });

  client->on_status([&state](const StatusEvent& evt) {
    std::string text = evt.text;
    if (text.rfind("HOST_VERSION:", 0) == 0) {
      std::string hv = text.substr(std::string("HOST_VERSION:").size());
      if (!hv.empty() && hv[0] == ' ') hv.erase(hv.begin());
      std::lock_guard<std::mutex> lock(state.mu);
      state.host_version = hv.empty() ? state.host_version : hv;
      mark_data(&state);
      return;
    }
    if (text.rfind("STATUS:", 0) == 0 || text.rfind("status:", 0) == 0) {
      text = text.substr(text.find(':') + 1);
      if (!text.empty() && text[0] == ' ') text.erase(text.begin());
    }
    if (text == "Reset" || text == "reset") return;
    std::lock_guard<std::mutex> lock(state.mu);
    state.status_text = text.empty() ? state.status_text : text;
    mark_data(&state);
  });

  client->on_reset([&state](const mighty_protocol::sdk::ResetEvent&) {
    std::lock_guard<std::mutex> lock(state.mu);
    state.status_text = "Reset";
    mark_data(&state);
  });

  client->on_error([&state](const mighty_protocol::sdk::MightyErrorEvent& err) {
    std::lock_guard<std::mutex> lock(state.mu);
    state.last_error = err.scope + ":" + err.code + " " + err.message;
  });

  std::cout << "Connecting with MightyWebDevice default host fallback..." << std::endl;
  client->connect();

  UiRuntime ui;
  ui.client = client;
  ui.state = &state;

  const std::string window = "Mighty SDK C++ Demo";
  cv::namedWindow(window, cv::WINDOW_NORMAL);
  cv::resizeWindow(window, kCanvasW, kCanvasH);
  cv::setMouseCallback(window, on_mouse, &ui);

  cv::Mat canvas(kCanvasH, kCanvasW, CV_8UC3);
  bool running = true;
  while (running) {
    if (ui.request_toggle_vio.exchange(false)) {
      request_vio_toggle(&ui);
    }

    const bool connected = client->is_connected();
    const std::string source = device->get_info().source;
    const DashboardSnapshot snap = make_snapshot(&state, connected, source);

    render_dashboard(canvas, snap, &ui.vio_button_rect);
    cv::imshow(window, canvas);

    const int key = cv::waitKey(16);
    if (key == 27 || key == 'q' || key == 'Q') {
      running = false;
    } else if (key == 'v' || key == 'V') {
      request_vio_toggle(&ui);
    }
  }

  client->disconnect();
  cv::destroyAllWindows();
  return 0;
}
