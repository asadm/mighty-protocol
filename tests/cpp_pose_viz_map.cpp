#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Case {
  std::string name;
  std::array<double, 3> position{};
  std::array<double, 4> quat{};
};

std::array<double, 3> map_pos_odom_to_viz(const std::array<double, 3>& p) {
  return {
      0.0 * p[0] + -1.0 * p[1] + 0.0 * p[2],
      0.0 * p[0] + 0.0 * p[1] + 1.0 * p[2],
      -1.0 * p[0] + 0.0 * p[1] + 0.0 * p[2],
  };
}

std::array<double, 4> q_mul_xyzw(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  const double ax = a[0], ay = a[1], az = a[2], aw = a[3];
  const double bx = b[0], by = b[1], bz = b[2], bw = b[3];
  return {
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
      aw * bw - ax * bx - ay * by - az * bz,
  };
}

std::array<double, 4> q_norm(const std::array<double, 4>& q) {
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(n) || n <= 1e-12) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

std::string json_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 4);
  for (char c : s) {
    if (c == '"' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

}  // namespace

int main() {
  const std::array<double, 4> q_viz_from_odom{{0.5, -0.5, 0.5, 0.5}};
  const std::vector<Case> cases = {
      {"forward_identity", {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
      {"left_yaw90", {0.0, 1.0, 0.0}, {0.0, 0.0, 0.7071067811865475, 0.7071067811865476}},
      {"mixed_nonunit", {-2.5, 4.0, 0.25}, {0.1, -0.2, 0.3, 0.9}},
  };

  std::cout << "{\"language\":\"cpp\",\"cases\":[";
  for (size_t i = 0; i < cases.size(); ++i) {
    const auto& c = cases[i];
    const auto p_viz = map_pos_odom_to_viz(c.position);
    const auto q_viz = q_norm(q_mul_xyzw(q_viz_from_odom, q_norm(c.quat)));
    if (i > 0) std::cout << ",";
    std::cout << "{\"name\":\"" << json_escape(c.name) << "\",";
    std::cout << "\"position\":[" << p_viz[0] << "," << p_viz[1] << "," << p_viz[2] << "],";
    std::cout << "\"quat\":[" << q_viz[0] << "," << q_viz[1] << "," << q_viz[2] << "," << q_viz[3] << "]}";
  }
  std::cout << "]}" << std::endl;
  return 0;
}
