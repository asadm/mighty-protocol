#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "../cpp/mighty_protocol.h"
#include "../cpp/mighty_protocol_consumer.h"

using namespace mighty_protocol;

namespace {

constexpr int kExpectedFrames = 15; // RSET + JPG + RJPG + RAW + SRAW + POSE + UPOSE + LCON + 3xVIZ + IMU + STAT + FEA3 + PCLD

struct SampleData {
  uint64_t jpg_ts = 111;
  std::string jpg_channel = "preview";
  std::vector<uint8_t> jpg_data = {0x01, 0x02, 0x03};

  uint64_t rjpg_ts = 222;
  std::vector<uint8_t> rjpg_data = {0xAA, 0xBB};

  uint64_t raw_ts = 333;
  uint32_t raw_width = 4;
  uint32_t raw_height = 2;
  uint8_t raw_format = static_cast<uint8_t>(RawFormat::kGray8);
  std::string raw_channel = "cam0";
  std::vector<uint8_t> raw_data = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

  uint64_t sraw_left_ts = 444;
  uint64_t sraw_right_ts = 445;
  uint32_t sraw_left_width = 2;
  uint32_t sraw_left_height = 1;
  uint8_t sraw_left_format = static_cast<uint8_t>(RawFormat::kGray8);
  std::string sraw_left_channel = "cam0";
  std::vector<uint8_t> sraw_left_data = {0x21, 0x22};
  uint32_t sraw_right_width = 2;
  uint32_t sraw_right_height = 1;
  uint8_t sraw_right_format = static_cast<uint8_t>(RawFormat::kGray8);
  std::string sraw_right_channel = "cam1";
  std::vector<uint8_t> sraw_right_data = {0x31, 0x32};

  uint32_t pose_type = 0;
  double pose_xyz[3] = {1.1, 2.2, 3.3};
  double pose_quat[4] = {0.1, 0.2, 0.3, 0.9};
  float pose_confidence01 = 0.82f;

  uint32_t upose_type = 0;
  double upose_xyz[3] = {4.4, 5.5, 6.6};
  double upose_quat[4] = {0.4, 0.5, 0.6, 0.7};
  float upose_confidence01 = 0.41f;

  std::vector<PoseConstraintSegment> constraints = {
    PoseConstraintSegment{{0.1f, 0.2f, 0.3f}, {0.4f, 0.5f, 0.6f}, 0},
    PoseConstraintSegment{{1.0f, 1.1f, 1.2f}, {1.3f, 1.4f, 1.5f}, 1},
  };

  VizPayload viz0 {
    .subtype = 0,
    .features = {
      VizFeature{10, 20, 1, 7},
      VizFeature{30, 40, 4, 8},
    }
  };

  VizPayload viz1 {
    .subtype = 1,
    .detections = {
      VizDetection{5, 6, 25, 26, "car"},
    }
  };

  VizPayload viz2 {
    .subtype = 2,
    .matches = {
      VizMatch{100, 110, 120, 130, 200},
    }
  };

  std::vector<ImuSample> imu = {
    {1000, 0.1, 0.2, 0.3, 1.1, 1.2, 1.3},
    {2000, 0.4, 0.5, 0.6, 1.4, 1.5, 1.6},
  };

  std::string status = "STATUS_OK";

  std::vector<Feature3D> fea3 = {
    {1, 0.1, 0.2, 0.3},
    {2, 1.1, 1.2, 1.3},
  };

  std::vector<Point3DColor> pcld = {
    {1.f, 2.f, 3.f, 10, 20, 30},
    {4.f, 5.f, 6.f, 40, 50, 60},
  };
  float pcld_size = 1.5f;
};

bool approx(double a, double b, double eps = 1e-6) {
  return std::abs(a - b) < eps;
}

bool verify_frame(const Frame& f, int index, const SampleData& s) {
  auto type_str = std::string(f.type.begin(), f.type.end());
  if (type_str == "RSET") {
    return index == 0 && f.payload.empty();
  }
  if (type_str == "JPG ") {
    uint64_t ts; std::string ch; std::vector<uint8_t> data;
    if (!decode_jpg_payload(f.payload, false, ts, ch, data)) return false;
    return ts == s.jpg_ts && ch == s.jpg_channel && data == s.jpg_data;
  }
  if (type_str == "RJPG") {
    uint64_t ts; std::string ch; std::vector<uint8_t> data;
    if (!decode_jpg_payload(f.payload, true, ts, ch, data)) return false;
    return ts == s.rjpg_ts && data == s.rjpg_data;
  }
  if (type_str == "RAW ") {
    uint64_t ts; uint32_t w, h; uint8_t fmt; std::string ch; std::vector<uint8_t> data;
    if (!decode_raw_payload(f.payload, ts, w, h, fmt, ch, data)) return false;
    return ts == s.raw_ts && w == s.raw_width && h == s.raw_height &&
           fmt == s.raw_format && ch == s.raw_channel && data == s.raw_data;
  }
  if (type_str == "SRAW") {
    uint64_t lts, rts; uint32_t lw, lh, rw, rh; uint8_t lfmt, rfmt;
    std::string lch, rch; std::vector<uint8_t> ldata, rdata;
    if (!decode_stereo_raw_payload(f.payload, lts, rts, lw, lh, lfmt, lch, ldata, rw, rh, rfmt, rch, rdata)) return false;
    return lts == s.sraw_left_ts && rts == s.sraw_right_ts &&
           lw == s.sraw_left_width && lh == s.sraw_left_height && lfmt == s.sraw_left_format &&
           lch == s.sraw_left_channel && ldata == s.sraw_left_data &&
           rw == s.sraw_right_width && rh == s.sraw_right_height && rfmt == s.sraw_right_format &&
           rch == s.sraw_right_channel && rdata == s.sraw_right_data;
  }
  if (type_str == "POSE") {
    uint32_t pt, pf; double x, y, z; std::optional<std::array<double,4>> q;
    float conf = 1.0f;
    if (!decode_pose_payload(f.payload, pt, pf, x, y, z, q, &conf)) return false;
    return pt == s.pose_type && (pf & 0x3) == 0x3 &&
           approx(x, s.pose_xyz[0]) && approx(y, s.pose_xyz[1]) && approx(z, s.pose_xyz[2]) &&
           q.has_value() && approx((*q)[0], s.pose_quat[0]) && approx((*q)[3], s.pose_quat[3]) &&
           approx(conf, s.pose_confidence01, 1e-3);
  }
  if (type_str == "UPOS") {
    uint32_t pt, pf; double x, y, z; std::optional<std::array<double,4>> q;
    float conf = 1.0f;
    if (!decode_pose_payload(f.payload, pt, pf, x, y, z, q, &conf)) return false;
    return pt == s.upose_type && (pf & 0x1) == 0x1 &&
           approx(x, s.upose_xyz[0]) && approx(z, s.upose_xyz[2]) &&
           q.has_value() && approx((*q)[1], s.upose_quat[1]) &&
           approx(conf, s.upose_confidence01, 1e-3);
  }
  if (type_str == "LCON") {
    std::vector<PoseConstraintSegment> segs;
    if (!decode_constraints_payload(f.payload, segs)) return false;
    return segs.size() == s.constraints.size() && segs[1].type == 1;
  }
  if (type_str == "VIZ ") {
    VizPayload vp;
    if (!decode_viz_payload(f.payload, vp)) return false;
    if (vp.subtype == 0) return vp.features.size() == s.viz0.features.size();
    if (vp.subtype == 1) return !vp.detections.empty() && vp.detections[0].label == "car";
    if (vp.subtype == 2) return !vp.matches.empty() && vp.matches[0].confidence == 200;
    return false;
  }
  if (type_str == "IMU ") {
    std::vector<ImuSample> out;
    if (!decode_imu_payload(f.payload, out)) return false;
    return out.size() == s.imu.size() && approx(out[0].gx, s.imu[0].gx);
  }
  if (type_str == "STAT") {
    std::string text;
    if (!decode_status_payload(f.payload, text)) return false;
    return text == s.status;
  }
  if (type_str == "FEA3") {
    std::vector<Feature3D> out;
    if (!decode_fea3_payload(f.payload, out)) return false;
    return out.size() == s.fea3.size() && out[1].id == 2;
  }
  if (type_str == "PCLD") {
    std::vector<Point3DColor> pts; std::optional<float> ps;
    if (!decode_pcld_payload(f.payload, pts, ps)) return false;
    return pts.size() == s.pcld.size() && ps.has_value() && approx(ps.value(), s.pcld_size, 1e-5);
  }
  return false;
}

std::vector<std::vector<uint8_t>> build_sample_packets(const SampleData& s) {
  std::vector<std::vector<uint8_t>> packets;
  packets.reserve(kExpectedFrames);
  packets.push_back(make_packet(nullptr, 0, TYPE_RSET));
  packets.push_back(make_packet(build_jpg_payload(s.jpg_ts, false, s.jpg_channel, s.jpg_data.data(), s.jpg_data.size()), TYPE_JPG));
  packets.push_back(make_packet(build_jpg_payload(s.rjpg_ts, true, "", s.rjpg_data.data(), s.rjpg_data.size()), TYPE_RJPG));
  packets.push_back(make_packet(build_raw_payload(s.raw_ts, s.raw_channel, s.raw_width, s.raw_height,
                                                 s.raw_format, s.raw_data.data(), s.raw_data.size()), TYPE_RAW));
  packets.push_back(make_packet(build_stereo_raw_payload(s.sraw_left_ts,
                                                         s.sraw_right_ts,
                                                         s.sraw_left_channel,
                                                         s.sraw_left_width,
                                                         s.sraw_left_height,
                                                         s.sraw_left_format,
                                                         s.sraw_left_data.data(),
                                                         s.sraw_left_data.size(),
                                                         s.sraw_right_channel,
                                                         s.sraw_right_width,
                                                         s.sraw_right_height,
                                                         s.sraw_right_format,
                                                         s.sraw_right_data.data(),
                                                         s.sraw_right_data.size()), TYPE_SRAW));
  packets.push_back(make_packet(build_pose_payload(s.pose_type, true, true,
                                                   s.pose_xyz[0], s.pose_xyz[1], s.pose_xyz[2],
                                                   s.pose_quat, s.pose_confidence01), TYPE_POSE));
  packets.push_back(make_packet(build_pose_payload(s.upose_type, true, false,
                                                   s.upose_xyz[0], s.upose_xyz[1], s.upose_xyz[2],
                                                   s.upose_quat, s.upose_confidence01), TYPE_UPOSE));
  packets.push_back(make_packet(build_constraints_payload(s.constraints), TYPE_LCON));
  packets.push_back(make_packet(build_viz_payload(s.viz0), TYPE_VIZ));
  packets.push_back(make_packet(build_viz_payload(s.viz1), TYPE_VIZ));
  packets.push_back(make_packet(build_viz_payload(s.viz2), TYPE_VIZ));
  packets.push_back(make_packet(build_imu_payload(s.imu), TYPE_IMU));
  packets.push_back(make_packet(build_status_payload(s.status), TYPE_STAT));
  packets.push_back(make_packet(build_fea3_payload(s.fea3), TYPE_FEA3));
  packets.push_back(make_packet(build_pcld_payload(s.pcld, s.pcld_size), TYPE_PCLD));
  return packets;
}

int create_server(uint16_t port) {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return -1;
  int opt = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }
  if (listen(fd, 1) < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

bool send_all(int sock, const std::vector<uint8_t>& data) {
  size_t sent = 0;
  while (sent < data.size()) {
    ssize_t n = ::send(sock, data.data() + sent, data.size() - sent, 0);
    if (n <= 0) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}

} // namespace

int main(int argc, char** argv) {
  if (argc < 3 || std::string(argv[1]) != "--port") {
    std::cerr << "Usage: cpp_roundtrip --port <port>\n";
    return 1;
  }
  uint16_t port = static_cast<uint16_t>(std::atoi(argv[2]));
  SampleData sample;
  const auto outbound = build_sample_packets(sample);

  int server_fd = create_server(port);
  if (server_fd < 0) {
    std::cerr << "Failed to create server\n";
    return 2;
  }
  sockaddr_in cli{}; socklen_t cl = sizeof(cli);
  int client = accept(server_fd, reinterpret_cast<sockaddr*>(&cli), &cl);
  if (client < 0) {
    std::cerr << "accept failed\n";
    return 3;
  }

  std::vector<uint8_t> buffer;
  mighty_protocol::FrameConsumer consumer;
  int received_frames = 0;
  while (received_frames < kExpectedFrames) {
    uint8_t tmp[4096];
    ssize_t n = recv(client, tmp, sizeof(tmp), 0);
    if (n <= 0) {
      std::cerr << "recv failed\n";
      return 4;
    }
    consumer.feed(tmp, static_cast<size_t>(n));
    auto parsed = consumer.drain();
    for (auto& f : parsed) {
      if (!verify_frame(f, received_frames, sample)) {
        std::cerr << "Frame verification failed at index " << received_frames << "\n";
        return 5;
      }
      received_frames++;
    }
  }

  // Send our frames back to the Node client
  for (const auto& pkt : outbound) {
    if (!send_all(client, pkt)) {
      std::cerr << "send failed\n";
      return 6;
    }
  }

  close(client);
  close(server_fd);
  return 0;
}
