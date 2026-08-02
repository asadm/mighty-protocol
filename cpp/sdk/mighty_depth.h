#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mighty_client.h"

namespace mighty_protocol {
namespace sdk {

struct RectifiedRgbaFrame {
  uint64_t timestamp_ns = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  std::string frame_id;
  std::vector<uint8_t> rgba;
};

inline std::optional<float> depth_at_meters(const DepthFrame& frame,
                                            uint32_t x,
                                            uint32_t y) {
  if (x >= frame.width || y >= frame.height ||
      frame.depth_mm.size() != static_cast<size_t>(frame.width) * frame.height ||
      !std::isfinite(frame.depth_scale_m) || frame.depth_scale_m <= 0.0f) {
    return std::nullopt;
  }
  const uint16_t sample = frame.depth_mm[static_cast<size_t>(y) * frame.width + x];
  if (sample == frame.invalid_value) return std::nullopt;
  return static_cast<float>(sample) * frame.depth_scale_m;
}

namespace depth_detail {

inline std::string canonical_channel(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (value == "preview" || value == "left") return "cam0";
  if (value == "right") return "cam1";
  return value;
}

inline std::string rgbd_key(const std::string& channel, uint64_t timestamp_ns) {
  return canonical_channel(channel) + ":" + std::to_string(timestamp_ns);
}

inline bool raw_to_rgba(const RawImageFrame& image,
                        std::vector<uint8_t>* rgba) {
  if (!rgba || image.width == 0 || image.height == 0) return false;
  const size_t pixels = static_cast<size_t>(image.width) * image.height;
  rgba->assign(pixels * 4u, 0);
  const auto format = static_cast<RawFormat>(image.format);
  if (format == RawFormat::kGray8 ||
      format == RawFormat::kYUV420SP || format == RawFormat::kYUV420P) {
    if (image.data.size() < pixels) return false;
    for (size_t i = 0; i < pixels; ++i) {
      (*rgba)[i * 4u] = image.data[i];
      (*rgba)[i * 4u + 1] = image.data[i];
      (*rgba)[i * 4u + 2] = image.data[i];
      (*rgba)[i * 4u + 3] = 255;
    }
    return true;
  }
  if (format == RawFormat::kRGB24 || format == RawFormat::kBGR24) {
    if (image.data.size() < pixels * 3u) return false;
    const bool bgr = format == RawFormat::kBGR24;
    for (size_t i = 0; i < pixels; ++i) {
      const size_t src = i * 3u;
      (*rgba)[i * 4u] = image.data[src + (bgr ? 2u : 0u)];
      (*rgba)[i * 4u + 1] = image.data[src + 1u];
      (*rgba)[i * 4u + 2] = image.data[src + (bgr ? 0u : 2u)];
      (*rgba)[i * 4u + 3] = 255;
    }
    return true;
  }
  if (format == RawFormat::kRGBA32 || format == RawFormat::kBGRA32) {
    if (image.data.size() < pixels * 4u) return false;
    const bool bgra = format == RawFormat::kBGRA32;
    for (size_t i = 0; i < pixels; ++i) {
      const size_t src = i * 4u;
      (*rgba)[i * 4u] = image.data[src + (bgra ? 2u : 0u)];
      (*rgba)[i * 4u + 1] = image.data[src + 1u];
      (*rgba)[i * 4u + 2] = image.data[src + (bgra ? 0u : 2u)];
      (*rgba)[i * 4u + 3] = image.data[src + 3u];
    }
    return true;
  }
  return false;
}

inline std::array<double, 2> distort(double x,
                                     double y,
                                     const DepthFrame& frame) {
  const auto& d = frame.distortion;
  if (frame.camera_model == static_cast<uint8_t>(DepthCameraModel::kDoubleSphere)) {
    const double radius2 = x * x + y * y;
    if (radius2 < 1e-16) return {x, y};
    const double d1 = std::sqrt(radius2 + 1.0);
    const double shifted = static_cast<double>(d[0]) * d1 + 1.0;
    const double d2 = std::sqrt(radius2 + shifted * shifted);
    const double denominator = static_cast<double>(d[1]) * d2 +
        (1.0 - static_cast<double>(d[1])) * shifted;
    if (!std::isfinite(denominator) || std::abs(denominator) <= 1e-12) {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      return {nan, nan};
    }
    return {x / denominator, y / denominator};
  }
  if (frame.distortion_model == static_cast<uint8_t>(DepthDistortionModel::kNone)) {
    return {x, y};
  }
  if (frame.distortion_model == static_cast<uint8_t>(DepthDistortionModel::kEquidistant)) {
    const double radius = std::hypot(x, y);
    if (radius < 1e-8) return {x, y};
    const double theta = std::atan(radius);
    const double theta2 = theta * theta;
    const double theta_d = theta * (1.0 + d[0] * theta2 + d[1] * theta2 * theta2 +
        d[2] * theta2 * theta2 * theta2 + d[3] * theta2 * theta2 * theta2 * theta2);
    const double scale = theta_d / radius;
    return {x * scale, y * scale};
  }
  const double x2 = x * x;
  const double y2 = y * y;
  const double xy = x * y;
  const double radius2 = x2 + y2;
  const double radial = 1.0 + d[0] * radius2 + d[1] * radius2 * radius2;
  return {
      x * radial + 2.0 * d[2] * xy + d[3] * (radius2 + 2.0 * x2),
      y * radial + d[2] * (radius2 + 2.0 * y2) + 2.0 * d[3] * xy,
  };
}

inline void sample_rgba(const std::vector<uint8_t>& source,
                        uint32_t width,
                        uint32_t height,
                        double x,
                        double y,
                        uint8_t* output) {
  if (!output) return;
  output[0] = output[1] = output[2] = 0;
  output[3] = 255;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0 ||
      x > static_cast<double>(width - 1u) || y > static_cast<double>(height - 1u)) return;
  const uint32_t x0 = static_cast<uint32_t>(std::floor(x));
  const uint32_t y0 = static_cast<uint32_t>(std::floor(y));
  const uint32_t x1 = std::min(width - 1u, x0 + 1u);
  const uint32_t y1 = std::min(height - 1u, y0 + 1u);
  const double ax = x - x0;
  const double ay = y - y0;
  const size_t indexes[4] = {
      (static_cast<size_t>(y0) * width + x0) * 4u,
      (static_cast<size_t>(y0) * width + x1) * 4u,
      (static_cast<size_t>(y1) * width + x0) * 4u,
      (static_cast<size_t>(y1) * width + x1) * 4u,
  };
  for (size_t channel = 0; channel < 4; ++channel) {
    const double top = source[indexes[0] + channel] +
        (source[indexes[1] + channel] - source[indexes[0] + channel]) * ax;
    const double bottom = source[indexes[2] + channel] +
        (source[indexes[3] + channel] - source[indexes[2] + channel]) * ax;
    output[channel] = static_cast<uint8_t>(std::lround(top + (bottom - top) * ay));
  }
}

}  // namespace depth_detail

inline bool rectify_image_to_depth(const RawImageFrame& image,
                                   const DepthFrame& depth,
                                   RectifiedRgbaFrame* output,
                                   bool require_matching_timestamp = true) {
  if (!output || depth.width == 0 || depth.height == 0 ||
      (require_matching_timestamp && image.timestamp_ns != 0 && depth.timestamp_ns != 0 &&
       image.timestamp_ns != depth.timestamp_ns)) {
    return false;
  }
  std::vector<uint8_t> source;
  if (!depth_detail::raw_to_rgba(image, &source)) return false;
  const auto& dk = depth.depth_intrinsics;
  const auto& sk = depth.source_intrinsics;
  if (!(dk[0] > 0.0f) || !(dk[1] > 0.0f) || !(sk[0] > 0.0f) || !(sk[1] > 0.0f)) {
    return false;
  }
  const uint32_t calibration_width = depth.source_width > 0 ? depth.source_width : image.width;
  const uint32_t calibration_height = depth.source_height > 0 ? depth.source_height : image.height;
  if (calibration_width == 0 || calibration_height == 0) return false;
  const double scale_x = static_cast<double>(image.width) / calibration_width;
  const double scale_y = static_cast<double>(image.height) / calibration_height;
  const double sfx = sk[0] * scale_x;
  const double sfy = sk[1] * scale_y;
  const double scx = sk[2] * scale_x;
  const double scy = sk[3] * scale_y;

  RectifiedRgbaFrame rectified;
  rectified.timestamp_ns = depth.timestamp_ns;
  rectified.width = depth.width;
  rectified.height = depth.height;
  rectified.frame_id = depth.frame_id;
  rectified.rgba.resize(static_cast<size_t>(depth.width) * depth.height * 4u);
  for (uint32_t y = 0; y < depth.height; ++y) {
    for (uint32_t x = 0; x < depth.width; ++x) {
      const double ray_x = (static_cast<double>(x) - dk[2]) / dk[0];
      const double ray_y = (static_cast<double>(y) - dk[3]) / dk[1];
      const auto distorted = depth_detail::distort(ray_x, ray_y, depth);
      depth_detail::sample_rgba(
          source, image.width, image.height,
          sfx * distorted[0] + scx, sfy * distorted[1] + scy,
          rectified.rgba.data() + (static_cast<size_t>(y) * depth.width + x) * 4u);
    }
  }
  *output = std::move(rectified);
  return true;
}

class RgbdSynchronizer {
 public:
  using Handler = std::function<void(const RawImageFrame&, const DepthFrame&)>;

  explicit RgbdSynchronizer(Handler handler, size_t max_entries = 64)
      : handler_(std::move(handler)), max_entries_(std::max<size_t>(2, max_entries)) {}

  void push_image(const RawImageFrame& image) {
    if (image.timestamp_ns == 0) return;
    std::optional<DepthFrame> matched;
    const std::string key = depth_detail::rgbd_key(
        image.channel_alias.empty() ? image.channel : image.channel_alias,
        image.timestamp_ns);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = depths_.find(key);
      if (it != depths_.end()) {
        matched = std::move(it->second);
        depths_.erase(it);
        erase_key(depth_order_, key);
      } else {
        erase_key(image_order_, key);
        images_[key] = image;
        image_order_.push_back(key);
        trim(images_, image_order_);
      }
    }
    if (matched && handler_) handler_(image, *matched);
  }

  void push_depth(const DepthFrame& depth) {
    if (depth.timestamp_ns == 0) return;
    std::optional<RawImageFrame> matched;
    const std::string key = depth_detail::rgbd_key(depth.source_channel, depth.timestamp_ns);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = images_.find(key);
      if (it != images_.end()) {
        matched = std::move(it->second);
        images_.erase(it);
        erase_key(image_order_, key);
      } else {
        erase_key(depth_order_, key);
        depths_[key] = depth;
        depth_order_.push_back(key);
        trim(depths_, depth_order_);
      }
    }
    if (matched && handler_) handler_(*matched, depth);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    images_.clear();
    depths_.clear();
    image_order_.clear();
    depth_order_.clear();
  }

 private:
  static void erase_key(std::deque<std::string>& order,
                        const std::string& key) {
    order.erase(std::remove(order.begin(), order.end(), key), order.end());
  }

  template <typename Map>
  void trim(Map& map, std::deque<std::string>& order) {
    while (map.size() > max_entries_ && !order.empty()) {
      const std::string key = std::move(order.front());
      order.pop_front();
      map.erase(key);
    }
  }

  Handler handler_;
  size_t max_entries_;
  std::mutex mutex_;
  std::unordered_map<std::string, RawImageFrame> images_;
  std::unordered_map<std::string, DepthFrame> depths_;
  std::deque<std::string> image_order_;
  std::deque<std::string> depth_order_;
};

}  // namespace sdk
}  // namespace mighty_protocol
