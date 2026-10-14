#include "../cpp/mighty_protocol_consumer.h"
#include <cassert>
#include <cstdio>

int main() {
  using namespace mighty_protocol;
  const std::vector<uint8_t> left{0xff, 0xd8, 1, 0xff, 0xd9};
  const std::vector<uint8_t> right{0xff, 0xd8, 2, 3, 0xff, 0xd9};
  auto payload = build_stereo_jpg_payload(123, 456, left.data(), left.size(), right.data(), right.size());
  uint64_t lts, rts; std::string lch, rch; std::vector<uint8_t> ldata, rdata;
  auto decode = [&](const auto& bytes) {
    return decode_stereo_jpg_payload(bytes, lts, lch, ldata, rts, rch, rdata);
  };
  assert(decode(payload));
  assert(lts == 123 && rts == 456 && lch == "cam0" && rch == "cam1");
  assert(ldata == left && rdata == right);
  auto bad = payload; bad.pop_back(); assert(!decode(bad));
  bad = payload; bad.push_back(0); assert(!decode(bad));
  bad = payload; write_u32_be(bad.data(), 0xffffffff); assert(!decode(bad));
  DecodedDispatcher dispatcher;
  int members = 0, pairs = 0;
  dispatcher.on_jpg([&](auto, const auto&, const auto&, bool ref) { assert(!ref); ++members; });
  auto packet = make_packet(payload, TYPE_SJPG);
  // Fragmented transport must not dispatch one side before the complete pair.
  dispatcher.feed(packet.data(), packet.size() - 1); assert(members == 0);
  dispatcher.feed(packet.data() + packet.size() - 1, 1); assert(members == 2);
  dispatcher.on_stereo_jpg([&](const auto& l, const auto& r) {
    assert(l.timestamp_ns == 123 && r.timestamp_ns == 456); ++pairs;
  });
  dispatcher.feed(packet.data(), packet.size()); assert(pairs == 1 && members == 2);
  for (auto byte : payload) std::printf("%02x", byte);
  std::puts("");
}
