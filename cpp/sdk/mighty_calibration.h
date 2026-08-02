#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace mighty_protocol {
namespace sdk {

struct RigidTransform3d {
  // Row-major target_from_source homogeneous matrix.
  std::array<double, 16> matrix{
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0};
  bool valid = false;
};

struct PinholeIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  bool valid() const {
    return std::isfinite(fx) && std::isfinite(fy) &&
           std::isfinite(cx) && std::isfinite(cy) && fx > 0.0 && fy > 0.0;
  }
};

struct CameraCalibration {
  std::string id = "cam0";
  // Canonical tokens: pinhole | double_sphere | unknown.
  std::string camera_model = "unknown";
  // Canonical tokens: none | radtan | equidistant | unknown.
  std::string distortion_model = "unknown";
  PinholeIntrinsics intrinsics;
  // Camera-model parameters not included in fx/fy/cx/cy. For double-sphere
  // this is [xi, alpha].
  std::vector<double> projection_parameters;
  std::vector<double> distortion_coefficients;
  std::uint32_t resolution_width = 0;
  std::uint32_t resolution_height = 0;
  std::string topic;
  double time_shift_camera_imu_s = 0.0;

  // T_camera_imu from calibration: p_camera = T_camera_imu * p_imu.
  RigidTransform3d camera_from_imu;
  // Derived against the public pose contract: base_link is FLU at the IMU
  // origin, and camera coordinates are optical right/down/forward.
  RigidTransform3d body_from_camera;

  bool valid() const { return intrinsics.valid(); }
};

struct Calibration {
  // kalibr | opencv
  std::string source_format;
  std::vector<CameraCalibration> cameras;

  const CameraCalibration* camera(const std::string& id = "cam0") const {
    for (const CameraCalibration& value : cameras) {
      if (value.id == id) return &value;
    }
    return nullptr;
  }

  CameraCalibration* camera(const std::string& id = "cam0") {
    for (CameraCalibration& value : cameras) {
      if (value.id == id) return &value;
    }
    return nullptr;
  }
};

struct CalibrationGetResult {
  bool ok = false;
  bool found = false;
  Calibration value;
  std::string message;
};

namespace calibration_detail {

inline std::string trim(std::string value) {
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
              value.end());
  return value;
}

inline std::string lower_token(std::string value) {
  value = trim(std::move(value));
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

inline std::string canonical_camera_model(const std::string& raw) {
  std::string token = lower_token(raw);
  token.erase(std::remove_if(token.begin(), token.end(),
                             [](unsigned char ch) {
                               return ch == '-' || ch == '_' ||
                                      std::isspace(ch);
                             }),
              token.end());
  if (token == "ds" || token == "doublesphere") return "double_sphere";
  if (token == "pinhole") return "pinhole";
  return "unknown";
}

inline std::string canonical_distortion_model(const std::string& raw) {
  std::string token = lower_token(raw);
  token.erase(std::remove_if(token.begin(), token.end(),
                             [](unsigned char ch) {
                               return ch == '-' || ch == '_' ||
                                      std::isspace(ch);
                             }),
              token.end());
  if (token.empty() || token == "none" || token == "off" ||
      token == "disabled") {
    return "none";
  }
  if (token == "radtan" || token == "radialtangential") return "radtan";
  if (token == "equidistant" || token == "fisheye") return "equidistant";
  return "unknown";
}

inline bool is_camera_id(const std::string& value) {
  if (value.size() < 4 || value.rfind("cam", 0) != 0) return false;
  return std::all_of(value.begin() + 3, value.end(),
                     [](unsigned char ch) { return std::isdigit(ch); });
}

struct Section {
  std::string id;
  std::size_t begin = 0;
  std::size_t end = 0;
};

inline std::vector<Section> camera_sections(const std::string& yaml) {
  std::vector<Section> sections;
  std::size_t line_start = 0;
  while (line_start < yaml.size()) {
    const std::size_t line_end = yaml.find('\n', line_start);
    const std::size_t end =
        line_end == std::string::npos ? yaml.size() : line_end;
    std::size_t first = line_start;
    while (first < end &&
           std::isspace(static_cast<unsigned char>(yaml[first])) &&
           yaml[first] != '\n' && yaml[first] != '\r') {
      ++first;
    }
    if (first == line_start) {
      const std::string line = trim(yaml.substr(line_start, end - line_start));
      if (!line.empty() && line.back() == ':') {
        const std::string id = trim(line.substr(0, line.size() - 1));
        if (is_camera_id(id)) {
          if (!sections.empty()) sections.back().end = line_start;
          sections.push_back(Section{id, line_start, yaml.size()});
        }
      }
    }
    if (line_end == std::string::npos) break;
    line_start = line_end + 1;
  }
  return sections;
}

inline bool find_key_value(const std::string& text,
                           std::size_t begin,
                           std::size_t end,
                           const std::string& key,
                           std::size_t* value_start,
                           std::size_t* line_end_out = nullptr) {
  std::size_t line_start = begin;
  while (line_start < end) {
    const std::size_t newline = text.find('\n', line_start);
    const std::size_t line_end =
        newline == std::string::npos ? end : std::min(end, newline);
    std::size_t cursor = line_start;
    while (cursor < line_end &&
           std::isspace(static_cast<unsigned char>(text[cursor]))) {
      ++cursor;
    }
    if (cursor + key.size() <= line_end &&
        text.compare(cursor, key.size(), key) == 0) {
      std::size_t separator = cursor + key.size();
      while (separator < line_end &&
             std::isspace(static_cast<unsigned char>(text[separator]))) {
        ++separator;
      }
      if (separator < line_end && text[separator] == ':') {
        if (value_start) *value_start = separator + 1;
        if (line_end_out) *line_end_out = line_end;
        return true;
      }
    }
    if (newline == std::string::npos || newline >= end) break;
    line_start = newline + 1;
  }
  return false;
}

inline bool scalar_text(const std::string& text,
                        std::size_t begin,
                        std::size_t end,
                        const std::string& key,
                        std::string* value) {
  std::size_t value_start = 0;
  std::size_t line_end = 0;
  if (!find_key_value(text, begin, end, key, &value_start, &line_end)) {
    return false;
  }
  std::string parsed = trim(text.substr(value_start, line_end - value_start));
  const std::size_t comment = parsed.find('#');
  if (comment != std::string::npos) parsed = trim(parsed.substr(0, comment));
  if (parsed.size() >= 2 &&
      ((parsed.front() == '"' && parsed.back() == '"') ||
       (parsed.front() == '\'' && parsed.back() == '\''))) {
    parsed = parsed.substr(1, parsed.size() - 2);
  }
  if (value) *value = parsed;
  return !parsed.empty();
}

inline bool scalar_double(const std::string& text,
                          std::size_t begin,
                          std::size_t end,
                          const std::string& key,
                          double* value) {
  std::string raw;
  if (!scalar_text(text, begin, end, key, &raw)) return false;
  char* parsed_end = nullptr;
  const double parsed = std::strtod(raw.c_str(), &parsed_end);
  if (parsed_end == raw.c_str() || !std::isfinite(parsed)) return false;
  while (*parsed_end != '\0' &&
         std::isspace(static_cast<unsigned char>(*parsed_end))) {
    ++parsed_end;
  }
  if (*parsed_end != '\0') return false;
  if (value) *value = parsed;
  return true;
}

inline bool append_numbers(const char* cursor,
                           const char* end,
                           std::vector<double>* values) {
  if (!values) return false;
  while (cursor < end) {
    char* parsed_end = nullptr;
    const double value = std::strtod(cursor, &parsed_end);
    if (parsed_end != cursor) {
      if (!std::isfinite(value)) return false;
      values->push_back(value);
      cursor = parsed_end;
    } else {
      ++cursor;
    }
  }
  return true;
}

inline bool bracket_numbers(const std::string& text,
                            std::size_t begin,
                            std::size_t end,
                            const std::string& key,
                            std::vector<double>* values,
                            std::size_t adjacent_row_target = 0) {
  if (!values) return false;
  values->clear();
  std::size_t value_start = 0;
  if (!find_key_value(text, begin, end, key, &value_start)) return false;
  std::size_t group_open = text.find('[', value_start);
  if (group_open == std::string::npos || group_open >= end ||
      group_open - value_start > 512) {
    return false;
  }
  const std::string prefix = text.substr(value_start, group_open - value_start);
  const std::size_t prefix_line_end = prefix.find('\n');
  const std::string first_prefix_line =
      prefix.substr(0, prefix_line_end == std::string::npos
                           ? prefix.size()
                           : prefix_line_end);
  if (first_prefix_line.find("!!opencv-matrix") == std::string::npos) {
    for (unsigned char ch : prefix) {
      if (!std::isspace(ch) && ch != '-') return false;
    }
  }

  while (group_open < end) {
    std::size_t group_close = group_open;
    int depth = 0;
    for (; group_close < end; ++group_close) {
      if (text[group_close] == '[') {
        ++depth;
      } else if (text[group_close] == ']') {
        --depth;
        if (depth == 0) break;
      }
    }
    if (group_close >= end) return false;
    if (!append_numbers(text.c_str() + group_open + 1,
                        text.c_str() + group_close, values)) {
      return false;
    }
    if (adjacent_row_target == 0 || values->size() >= adjacent_row_target) {
      return true;
    }

    const std::size_t next_open = text.find('[', group_close + 1);
    if (next_open == std::string::npos || next_open >= end ||
        next_open - group_close > 128) {
      return false;
    }
    for (std::size_t index = group_close + 1; index < next_open; ++index) {
      const unsigned char ch = static_cast<unsigned char>(text[index]);
      if (!std::isspace(ch) && ch != '-') return false;
    }
    group_open = next_open;
  }
  return false;
}

inline double determinant3(const std::array<double, 16>& matrix) {
  return matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
         matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
         matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
}

inline bool rigid_matrix_valid(const std::array<double, 16>& matrix) {
  for (double value : matrix) {
    if (!std::isfinite(value)) return false;
  }
  if (std::abs(matrix[12]) > 1e-6 || std::abs(matrix[13]) > 1e-6 ||
      std::abs(matrix[14]) > 1e-6 || std::abs(matrix[15] - 1.0) > 1e-6 ||
      std::abs(determinant3(matrix) - 1.0) > 0.1) {
    return false;
  }
  double orthogonality_error = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double dot = 0.0;
      for (int index = 0; index < 3; ++index) {
        dot += matrix[row * 4 + index] * matrix[col * 4 + index];
      }
      const double expected = row == col ? 1.0 : 0.0;
      orthogonality_error += (dot - expected) * (dot - expected);
    }
  }
  return std::sqrt(orthogonality_error) <= 0.1;
}

inline RigidTransform3d inverse_rigid(
    const std::array<double, 16>& matrix) {
  RigidTransform3d result;
  if (!rigid_matrix_valid(matrix)) return result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result.matrix[row * 4 + col] = matrix[col * 4 + row];
    }
    result.matrix[row * 4 + 3] =
        -(result.matrix[row * 4] * matrix[3] +
          result.matrix[row * 4 + 1] * matrix[7] +
          result.matrix[row * 4 + 2] * matrix[11]);
  }
  result.valid = true;
  return result;
}

inline RigidTransform3d derive_body_from_camera(
    const std::array<double, 16>& camera_from_imu) {
  RigidTransform3d result;
  if (!rigid_matrix_valid(camera_from_imu)) return result;
  // Optical camera right/down/forward -> base_link forward/left/up.
  result.matrix = {
      0.0, 0.0, 1.0, -camera_from_imu[11],
      -1.0, 0.0, 0.0, camera_from_imu[3],
      0.0, -1.0, 0.0, camera_from_imu[7],
      0.0, 0.0, 0.0, 1.0};
  result.valid = true;
  return result;
}

inline bool matrix_for_key(const std::string& text,
                           std::size_t begin,
                           std::size_t end,
                           const std::string& key,
                           std::array<double, 16>* matrix) {
  std::vector<double> values;
  if (!bracket_numbers(text, begin, end, key, &values, 16) ||
      values.size() != 16) {
    return false;
  }
  std::copy(values.begin(), values.end(), matrix->begin());
  return rigid_matrix_valid(*matrix);
}

inline std::uint32_t positive_u32(double value) {
  if (!std::isfinite(value) || value <= 0.0 ||
      value > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    return 0;
  }
  return static_cast<std::uint32_t>(std::llround(value));
}

inline CameraCalibration parse_camera(const std::string& yaml,
                                      const Section& section,
                                      bool kalibr_layout) {
  CameraCalibration camera;
  camera.id = section.id;

  std::string raw;
  if (scalar_text(yaml, section.begin, section.end, "camera_model", &raw)) {
    camera.camera_model = canonical_camera_model(raw);
  }
  if (scalar_text(yaml, section.begin, section.end,
                  "distortion_model", &raw)) {
    camera.distortion_model = canonical_distortion_model(raw);
  }
  if (scalar_text(yaml, section.begin, section.end, "rostopic", &raw)) {
    camera.topic = raw;
  }

  std::vector<double> intrinsics;
  if (bracket_numbers(yaml, section.begin, section.end,
                      "intrinsics", &intrinsics)) {
    if (camera.camera_model == "double_sphere" && intrinsics.size() >= 6) {
      camera.projection_parameters.assign(intrinsics.begin(),
                                          intrinsics.end() - 4);
      camera.intrinsics.fx = intrinsics[intrinsics.size() - 4];
      camera.intrinsics.fy = intrinsics[intrinsics.size() - 3];
      camera.intrinsics.cx = intrinsics[intrinsics.size() - 2];
      camera.intrinsics.cy = intrinsics[intrinsics.size() - 1];
    } else if (intrinsics.size() >= 4) {
      camera.intrinsics.fx = intrinsics[intrinsics.size() - 4];
      camera.intrinsics.fy = intrinsics[intrinsics.size() - 3];
      camera.intrinsics.cx = intrinsics[intrinsics.size() - 2];
      camera.intrinsics.cy = intrinsics[intrinsics.size() - 1];
      if (intrinsics.size() > 4) {
        camera.projection_parameters.assign(intrinsics.begin(),
                                            intrinsics.end() - 4);
      }
    }
  } else {
    scalar_double(yaml, section.begin, section.end,
                  "fx", &camera.intrinsics.fx);
    scalar_double(yaml, section.begin, section.end,
                  "fy", &camera.intrinsics.fy);
    scalar_double(yaml, section.begin, section.end,
                  "cx", &camera.intrinsics.cx);
    scalar_double(yaml, section.begin, section.end,
                  "cy", &camera.intrinsics.cy);
    if (camera.camera_model == "double_sphere") {
      double xi = 0.0;
      double alpha = 0.0;
      if (scalar_double(yaml, section.begin, section.end, "xi", &xi) &&
          scalar_double(yaml, section.begin, section.end, "alpha", &alpha)) {
        camera.projection_parameters = {xi, alpha};
      }
    }
  }

  bracket_numbers(yaml, section.begin, section.end,
                  "distortion_coeffs", &camera.distortion_coefficients);
  if (camera.distortion_coefficients.empty() &&
      camera.distortion_model != "none" &&
      camera.camera_model != "double_sphere") {
    const char* names[] = {"k1", "k2", "p1", "p2"};
    for (const char* name : names) {
      double coefficient = 0.0;
      if (scalar_double(yaml, section.begin, section.end,
                        name, &coefficient)) {
        camera.distortion_coefficients.push_back(coefficient);
      }
    }
  }

  std::vector<double> resolution;
  if (bracket_numbers(yaml, section.begin, section.end,
                      "resolution", &resolution) && resolution.size() >= 2) {
    camera.resolution_width = positive_u32(resolution[0]);
    camera.resolution_height = positive_u32(resolution[1]);
  } else {
    double width = 0.0;
    double height = 0.0;
    scalar_double(yaml, section.begin, section.end,
                  "resolution_width", &width);
    scalar_double(yaml, section.begin, section.end,
                  "resolution_height", &height);
    camera.resolution_width = positive_u32(width);
    camera.resolution_height = positive_u32(height);
  }

  if (!scalar_double(yaml, section.begin, section.end,
                     "timeshift_cam_imu", &camera.time_shift_camera_imu_s)) {
    scalar_double(yaml, section.begin, section.end,
                  "td", &camera.time_shift_camera_imu_s);
  }

  std::array<double, 16> matrix{};
  if (matrix_for_key(yaml, section.begin, section.end,
                     "T_cam_imu", &matrix)) {
    camera.camera_from_imu.matrix = matrix;
    camera.camera_from_imu.valid = true;
    camera.body_from_camera = derive_body_from_camera(matrix);
  } else if (matrix_for_key(yaml, section.begin, section.end,
                            "T_imu_cam", &matrix)) {
    camera.camera_from_imu = inverse_rigid(matrix);
    if (camera.camera_from_imu.valid) {
      camera.body_from_camera =
          derive_body_from_camera(camera.camera_from_imu.matrix);
    }
  } else if (matrix_for_key(yaml, section.begin, section.end,
                            "T_cam_body", &matrix)) {
    camera.body_from_camera = inverse_rigid(matrix);
  }

  (void)kalibr_layout;
  return camera;
}

}  // namespace calibration_detail

// Parses the two calibration layouts produced and consumed by Mighty:
// Kalibr's cam0/cam1 row-list YAML and the flat OpenCV FileStorage layout.
inline bool parse_calibration_yaml(const std::string& yaml,
                                   Calibration* calibration,
                                   std::string* error = nullptr) {
  if (!calibration) {
    if (error) *error = "calibration destination is null";
    return false;
  }
  *calibration = Calibration{};
  if (yaml.empty()) {
    if (error) *error = "calibration YAML is empty";
    return false;
  }

  std::vector<calibration_detail::Section> sections =
      calibration_detail::camera_sections(yaml);
  const bool kalibr_layout = !sections.empty();
  if (!kalibr_layout) {
    sections.push_back(calibration_detail::Section{
        "cam0", 0, yaml.size()});
  }
  calibration->source_format = kalibr_layout ? "kalibr" : "opencv";

  for (const calibration_detail::Section& section : sections) {
    CameraCalibration camera =
        calibration_detail::parse_camera(yaml, section, kalibr_layout);
    if (camera.valid()) calibration->cameras.push_back(std::move(camera));
  }
  if (calibration->cameras.empty()) {
    if (error) *error = "calibration has no camera with valid intrinsics";
    return false;
  }
  return true;
}

}  // namespace sdk
}  // namespace mighty_protocol
