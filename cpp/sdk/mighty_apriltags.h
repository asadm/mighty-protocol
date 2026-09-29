#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <locale>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../third_party/picojson.h"

namespace mighty_protocol {
namespace sdk {

inline constexpr double FRC_TAG_SIZE_M = 0.1651;

struct AprilTagPose3d {
  int tag_id = 0;
  double size_m = FRC_TAG_SIZE_M;
  std::array<double, 3> position_m{};
  // World-from-tag, WPILib tag axes: +X normal, +Y left, +Z up.
  std::array<double, 4> orientation_xyzw{0, 0, 0, 1};
};

struct AprilTagMap {
  std::vector<AprilTagPose3d> tags;
  // Zero means the optional dimension was absent; coordinates are unchanged.
  double field_length_m = 0;
  double field_width_m = 0;
};

namespace apriltag_detail {
inline const picojson::value& member(const picojson::value& value,
                                   const std::string& key) {
  if (!value.is<picojson::object>() || !value.contains(key)) {
    throw std::invalid_argument("missing WPILib layout field: " + key);
  }
  return value.get(key);
}

inline double number(const picojson::value& value) {
  if (!value.is<double>() || !std::isfinite(value.get<double>())) {
    throw std::invalid_argument("layout values must be finite numbers");
  }
  return value.get<double>();
}

inline std::array<double, 4> normalize(std::array<double, 4> q) {
  double scale = 0;
  for (double v : q) {
    if (!std::isfinite(v)) throw std::invalid_argument("quaternion must be finite");
    scale = std::max(scale, std::abs(v));
  }
  if (scale == 0) throw std::invalid_argument("quaternion must be nonzero");
  double norm = 0;
  for (double& v : q) { v /= scale; norm += v * v; }
  norm = std::sqrt(norm);
  for (double& v : q) v /= norm;
  return q;
}

inline void validate(const AprilTagPose3d& tag, std::set<int>& seen) {
  if (tag.tag_id < 0) throw std::invalid_argument("tag ID must be non-negative");
  if (!seen.insert(tag.tag_id).second) throw std::invalid_argument("duplicate tag ID");
  if (!std::isfinite(tag.size_m) || tag.size_m <= 0) {
    throw std::invalid_argument("tag size must be positive and finite");
  }
  for (double v : tag.position_m) {
    if (!std::isfinite(v)) throw std::invalid_argument("tag position must be finite");
  }
  normalize(tag.orientation_xyzw);
}
}  // namespace apriltag_detail

inline bool parse_wpilib_field_layout_json(const std::string& json,
                                           AprilTagMap* out,
                                           std::string* error = nullptr,
                                           std::optional<double> tag_size_m = std::nullopt) {
  if (error) error->clear();
  try {
    if (!out) throw std::invalid_argument("missing output map");
    picojson::value layout;
    std::string parse_error;
    auto end = picojson::parse(layout, json.begin(), json.end(), &parse_error);
    if (!parse_error.empty()) throw std::invalid_argument("invalid JSON: " + parse_error);
    for (; end != json.end(); ++end) {
      if (*end != ' ' && *end != '\t' && *end != '\r' && *end != '\n') {
        throw std::invalid_argument("invalid JSON: trailing content");
      }
    }
    double size_m = tag_size_m.value_or(FRC_TAG_SIZE_M);
    if (layout.contains("mighty")) {
      const auto& metadata = layout.get("mighty");
      if (!metadata.is<picojson::object>()) throw std::invalid_argument("mighty metadata must be an object");
      if (metadata.contains("tagFamily") && (!metadata.get("tagFamily").is<std::string>() ||
          metadata.get("tagFamily").get<std::string>() != "tag36h11")) {
        throw std::invalid_argument("Mighty requires tag36h11 tags");
      }
      if (!tag_size_m && metadata.contains("tagSizeM")) size_m = apriltag_detail::number(metadata.get("tagSizeM"));
    }
    if (!std::isfinite(size_m) || size_m <= 0) throw std::invalid_argument("tag size must be positive and finite");
    const auto& rows = apriltag_detail::member(layout, "tags");
    if (!rows.is<picojson::array>() || rows.get<picojson::array>().empty()) {
      throw std::invalid_argument("WPILib layout must contain a nonempty tags array");
    }
    AprilTagMap result;
    std::set<int> seen;
    for (const auto& row : rows.get<picojson::array>()) {
      AprilTagPose3d tag;
      const double id = apriltag_detail::number(apriltag_detail::member(row, "ID"));
      if (id < 0 || id > 2147483647.0 || id != std::floor(id)) {
        throw std::invalid_argument("tag ID must be a non-negative 32-bit integer");
      }
      tag.tag_id = static_cast<int>(id);
      tag.size_m = size_m;
      const auto& pose = apriltag_detail::member(row, "pose");
      const auto& p = apriltag_detail::member(pose, "translation");
      const auto& q = apriltag_detail::member(apriltag_detail::member(pose, "rotation"), "quaternion");
      for (int i = 0; i < 3; ++i) {
        tag.position_m[i] = apriltag_detail::number(apriltag_detail::member(p, std::string(1, "xyz"[i])));
      }
      for (int i = 0; i < 4; ++i) {
        tag.orientation_xyzw[i] = apriltag_detail::number(apriltag_detail::member(q, std::string(1, "XYZW"[i])));
      }
      tag.orientation_xyzw = apriltag_detail::normalize(tag.orientation_xyzw);
      apriltag_detail::validate(tag, seen);
      result.tags.push_back(tag);
    }
    if (layout.contains("field")) {
      const auto& field = layout.get("field");
      if (!field.is<picojson::object>()) throw std::invalid_argument("field must be an object");
      for (const auto& key : {"length", "width"}) {
        if (!field.contains(key)) continue;
        double dimension = apriltag_detail::number(field.get(key));
        if (dimension <= 0) throw std::invalid_argument("field dimensions must be positive");
        if (std::string(key) == "length") result.field_length_m = dimension;
        else result.field_width_m = dimension;
      }
    }
    *out = std::move(result);  // Leave caller's map untouched on failure.
    return true;
  } catch (const std::exception& exc) {
    if (error) *error = exc.what();
    return false;
  }
}

inline std::string serialize_apriltag_map_yaml(const AprilTagMap& map) {
  if (map.tags.empty()) throw std::invalid_argument("tag map must not be empty");
  std::ostringstream yaml;
  yaml.imbue(std::locale::classic());
  yaml << std::setprecision(17) << "%YAML:1.0\n---\napriltags:\n";
  std::set<int> seen;
  for (const auto& tag : map.tags) {
    apriltag_detail::validate(tag, seen);
    const auto q = apriltag_detail::normalize(tag.orientation_xyzw);
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    // q_field_wpilibTag * q_wpilibTag_pnpTag; field coordinates unchanged.
    const std::array<double, 4> native{(-w+x-y-z)/2, (w+x+y-z)/2,
                                      (-w+x+y+z)/2, (w+x-y+z)/2};
    yaml << "  - tag_id: " << tag.tag_id << "\n    size_m: " << tag.size_m
         << "\n    position_m: [" << tag.position_m[0] << ", " << tag.position_m[1]
         << ", " << tag.position_m[2] << "]\n    orientation: quaternion\n"
         << "    orientation_xyzw: [" << native[0] << ", " << native[1] << ", "
         << native[2] << ", " << native[3] << "]\n";
  }
  return yaml.str();
}

}  // namespace sdk
}  // namespace mighty_protocol
