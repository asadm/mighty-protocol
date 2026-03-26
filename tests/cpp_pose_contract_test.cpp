#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <vector>

#include "../cpp/mighty_protocol.h"

using namespace mighty_protocol;

namespace {

bool approx(double a, double b, double eps = 1e-9) {
  return std::abs(a - b) <= eps;
}

void append_u32_be(std::vector<uint8_t>& out, uint32_t v) {
  out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(v & 0xFF));
}

void append_u64_be(std::vector<uint8_t>& out, uint64_t v) {
  for (int i = 7; i >= 0; --i) {
    out.push_back(static_cast<uint8_t>((v >> (i * 8)) & 0xFF));
  }
}

void append_f64_be(std::vector<uint8_t>& out, double value) {
  static_assert(sizeof(double) == sizeof(uint64_t), "double size");
  uint64_t raw = 0;
  std::memcpy(&raw, &value, sizeof(double));
  append_u64_be(out, raw);
}

void append_f32_be(std::vector<uint8_t>& out, float value) {
  static_assert(sizeof(float) == sizeof(uint32_t), "float size");
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(float));
  append_u32_be(out, raw);
}

std::array<double, 3> map_position_odom_to_viz(const std::array<double, 3>& p) {
  return {
      0.0 * p[0] + -1.0 * p[1] + 0.0 * p[2],
      0.0 * p[0] + 0.0 * p[1] + 1.0 * p[2],
      -1.0 * p[0] + 0.0 * p[1] + 0.0 * p[2],
  };
}

}  // namespace

int main() {
  const uint32_t pose_type = 0;
  const uint32_t flags = 0x1u | (1u << 2) | (1u << 3) | (1u << 6);
  const std::array<double, 3> position{{4.25, -2.5, 0.75}};
  const std::array<double, 4> quat{{0.11, -0.22, 0.33, -0.44}};  // xyzw
  const std::array<double, 3> linvel{{1.0, 2.0, 3.0}};
  const std::array<double, 3> angvel{{-1.0, -2.0, -3.0}};
  const uint64_t timestamp_ns = 123456789ull;

  std::vector<uint8_t> payload;
  payload.reserve(4 + 4 + 24 + 32 + 4 + 24 + 24 + 8);
  append_u32_be(payload, pose_type);
  append_u32_be(payload, flags);
  append_f64_be(payload, position[0]);
  append_f64_be(payload, position[1]);
  append_f64_be(payload, position[2]);
  append_f64_be(payload, quat[0]);
  append_f64_be(payload, quat[1]);
  append_f64_be(payload, quat[2]);
  append_f64_be(payload, quat[3]);
  append_f32_be(payload, 1.25f);  // clamp to 1.0
  append_f64_be(payload, linvel[0]);
  append_f64_be(payload, linvel[1]);
  append_f64_be(payload, linvel[2]);
  append_f64_be(payload, angvel[0]);
  append_f64_be(payload, angvel[1]);
  append_f64_be(payload, angvel[2]);
  append_u64_be(payload, timestamp_ns);

  uint32_t out_pose_type = 99;
  uint32_t out_flags = 0;
  double x = 0.0, y = 0.0, z = 0.0;
  std::optional<std::array<double, 4>> out_quat;
  std::optional<std::array<double, 3>> out_linvel;
  std::optional<std::array<double, 3>> out_angvel;
  std::optional<uint64_t> out_ts;
  float confidence = 0.0f;

  const bool ok = decode_pose_payload(payload,
                                      out_pose_type,
                                      out_flags,
                                      x,
                                      y,
                                      z,
                                      out_quat,
                                      &confidence,
                                      &out_linvel,
                                      &out_angvel,
                                      nullptr,
                                      nullptr,
                                      &out_ts);
  assert(ok);
  assert(out_pose_type == pose_type);
  assert((out_flags & 0x1u) != 0u);
  assert(out_quat.has_value());
  assert(out_linvel.has_value());
  assert(out_angvel.has_value());
  assert(out_ts.has_value());
  assert(out_ts.value() == timestamp_ns);

  // Explicit XYZW wire-order regression check.
  assert(approx(out_quat.value()[0], quat[0]));
  assert(approx(out_quat.value()[1], quat[1]));
  assert(approx(out_quat.value()[2], quat[2]));
  assert(approx(out_quat.value()[3], quat[3]));
  assert(approx(x, position[0]));
  assert(approx(y, position[1]));
  assert(approx(z, position[2]));
  assert(approx(out_linvel.value()[0], linvel[0]));
  assert(approx(out_angvel.value()[2], angvel[2]));
  assert(approx(static_cast<double>(confidence), 1.0));

  // Signed-axis sanity for canonical->viz basis.
  const auto ex = map_position_odom_to_viz({{1.0, 0.0, 0.0}});
  const auto ey = map_position_odom_to_viz({{0.0, 1.0, 0.0}});
  const auto ez = map_position_odom_to_viz({{0.0, 0.0, 1.0}});
  assert(static_cast<int>(std::llround(ex[0])) == 0);
  assert(static_cast<int>(std::llround(ex[1])) == 0);
  assert(static_cast<int>(std::llround(ex[2])) == -1);
  assert(static_cast<int>(std::llround(ey[0])) == -1);
  assert(static_cast<int>(std::llround(ey[1])) == 0);
  assert(static_cast<int>(std::llround(ey[2])) == 0);
  assert(static_cast<int>(std::llround(ez[0])) == 0);
  assert(static_cast<int>(std::llround(ez[1])) == 1);
  assert(static_cast<int>(std::llround(ez[2])) == 0);

  std::cout << "C++ pose contract test passed" << std::endl;
  return 0;
}
