#include "../cpp/mighty_protocol.h"
#include <cassert>
#include <fstream>
#include <string>

using namespace mighty_protocol;

int main(int argc, char** argv) {
  assert(argc == 2);
  std::vector<ImuSample> samples;
  for (uint64_t i = 0; i < 5; ++i) {
    samples.push_back({1000000000 + i * 1250000, double(i), 2, 3, 4, 5, 6});
  }
  auto legacy = build_imu_payload(samples);
  assert(legacy.size() == 4 + 5 * 56);
  samples[0].temperature_c = -12.5;
  samples[1].temperature_c = 0;
  samples[3].temperature_c = 46.5;
  samples[4].temperature_c = std::numeric_limits<double>::infinity();
  auto modern = build_imu_payload(samples);
  assert(modern.size() == legacy.size() + 12 + 5 * 4);
  assert(std::equal(legacy.begin(), legacy.end(), modern.begin()));
  std::vector<ImuSample> decoded;
  assert(decode_imu_payload(modern, decoded));
  for (size_t i = 0; i < samples.size(); ++i) {
    assert(decoded[i].timestamp_ns == samples[i].timestamp_ns);
    assert(decoded[i].ax == samples[i].ax && decoded[i].gz == samples[i].gz);
    assert(decoded[i].temperature_c.has_value() == (i == 0 || i == 1 || i == 3));
    if (decoded[i].temperature_c) assert(*decoded[i].temperature_c == *samples[i].temperature_c);
  }
  for (auto& s : samples) s.temperature_c = std::numeric_limits<double>::quiet_NaN();
  assert(build_imu_payload(samples) == legacy);
  assert(decode_imu_payload(legacy, decoded));
  for (const auto& s : decoded) assert(!s.temperature_c);
  auto truncated = modern;
  truncated.pop_back();
  auto unknown = modern;
  unknown[legacy.size() + 5] = 2;
  auto bad_count = modern;
  write_u32_be(bad_count.data() + legacy.size() + 8, 1);
  for (const auto& bytes : {truncated, unknown, bad_count}) {
    assert(decode_imu_payload(bytes, decoded));
    assert(decoded.size() == 5);
    for (const auto& s : decoded) assert(!s.temperature_c);
  }
  auto invalid = legacy;
  write_u32_be(invalid.data(), 0xffffffff);
  assert(!decode_imu_payload(invalid, decoded));
  for (const auto& entry : std::vector<std::pair<std::string, std::vector<uint8_t>>>{
      {"legacy", legacy}, {"modern", modern}, {"truncated", truncated},
      {"unknown", unknown}, {"bad-count", bad_count}}) {
    std::ofstream out(std::string(argv[1]) + "/" + entry.first + ".bin", std::ios::binary);
    out.write(reinterpret_cast<const char*>(entry.second.data()), entry.second.size());
  }
}
