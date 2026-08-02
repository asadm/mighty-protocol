#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <vector>

#include "../cpp/mighty_protocol_consumer.h"
#include "../cpp/mighty_sdk.h"

using namespace mighty_protocol;
using namespace mighty_protocol::sdk;

int main() {
  static_assert(std::is_member_function_pointer<decltype(&MightyClient::on_depth)>::value,
                "C++ SDK must expose on_depth");
  DepthFrame depth;
  depth.timestamp_ns = 123456789;
  depth.width = 2;
  depth.height = 2;
  depth.source_width = 2;
  depth.source_height = 2;
  depth.depth_intrinsics = {1.0f, 1.0f, 0.0f, 0.0f};
  depth.source_intrinsics = {1.0f, 1.0f, 0.0f, 0.0f};
  depth.depth_mm = {0, 1250, 2500, 10000};

  const std::vector<uint8_t> payload = build_depth_payload(depth);
  assert(!payload.empty());
  DepthFrame decoded;
  assert(decode_depth_payload(payload, decoded));
  assert(decoded.depth_mm == depth.depth_mm);
  assert(depth_at_meters(decoded, 1, 0).has_value());
  assert(std::abs(*depth_at_meters(decoded, 1, 0) - 1.25f) < 1e-6f);
  assert(!depth_at_meters(decoded, 0, 0).has_value());

  RawImageFrame raw;
  raw.timestamp_ns = depth.timestamp_ns;
  raw.width = 2;
  raw.height = 2;
  raw.format = static_cast<uint8_t>(RawFormat::kGray8);
  raw.channel = "preview";
  raw.channel_alias = "cam0";
  raw.data = {10, 20, 30, 40};
  RectifiedRgbaFrame rectified;
  assert(rectify_image_to_depth(raw, decoded, &rectified));
  const std::vector<uint8_t> expected = {
      10, 10, 10, 255, 20, 20, 20, 255,
      30, 30, 30, 255, 40, 40, 40, 255};
  assert(rectified.rgba == expected);

  int pair_count = 0;
  RgbdSynchronizer synchronizer(
      [&](const RawImageFrame& image, const DepthFrame& paired_depth) {
        ++pair_count;
        assert(image.timestamp_ns == paired_depth.timestamp_ns);
      });
  synchronizer.push_image(raw);
  synchronizer.push_depth(decoded);
  assert(pair_count == 1);

  int dispatch_count = 0;
  DecodedDispatcher dispatcher;
  dispatcher.on_depth([&](const DepthFrame& frame) {
    ++dispatch_count;
    assert(frame.frame_id == "cam0_rectified");
  });
  const std::vector<uint8_t> packet = make_packet(payload, TYPE_DPT);
  dispatcher.feed(packet.data(), packet.size());
  assert(dispatch_count == 1);

  std::cout << "cpp depth tests passed\n";
  return 0;
}
