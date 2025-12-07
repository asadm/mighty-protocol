#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace mighty_protocol {

// Type codes
inline constexpr char TYPE_JPG[4]  = {'J','P','G',' '};
inline constexpr char TYPE_RJPG[4] = {'R','J','P','G'};
inline constexpr char TYPE_POSE[4] = {'P','O','S','E'};
inline constexpr char TYPE_UPOSE[4]= {'U','P','O','S'};
inline constexpr char TYPE_LCON[4] = {'L','C','O','N'};
inline constexpr char TYPE_VIZ[4]  = {'V','I','Z',' '};
inline constexpr char TYPE_IMU[4]  = {'I','M','U',' '};
inline constexpr char TYPE_STAT[4] = {'S','T','A','T'};
inline constexpr char TYPE_RSET[4] = {'R','S','E','T'};
inline constexpr char TYPE_FEA3[4] = {'F','E','A','3'};
inline constexpr char TYPE_PCLD[4] = {'P','C','L','D'};

inline constexpr uint8_t HEADER_MAGIC[4] = {0xDE, 0xAD, 0xBE, 0xEF};
inline constexpr uint8_t FOOTER_MAGIC[4] = {0xFE, 0xED, 0xFA, 0xCE};

// --------------------------------------------------------------------------
// Byte helpers
// --------------------------------------------------------------------------
inline void write_u32_be(uint8_t* dst, uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    dst[i] = static_cast<uint8_t>((value >> (24 - 8 * i)) & 0xFF);
  }
}
inline void write_u64_be(uint8_t* dst, uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    dst[i] = static_cast<uint8_t>((value >> (56 - 8 * i)) & 0xFF);
  }
}
inline void write_f32_be(uint8_t* dst, float value) {
  uint32_t raw = 0;
  static_assert(sizeof(float) == sizeof(uint32_t), "float must be 4 bytes");
  std::memcpy(&raw, &value, sizeof(float));
  write_u32_be(dst, raw);
}
inline void write_f64_be(uint8_t* dst, double value) {
  uint64_t raw = 0;
  static_assert(sizeof(double) == sizeof(uint64_t), "double must be 8 bytes");
  std::memcpy(&raw, &value, sizeof(double));
  write_u64_be(dst, raw);
}

inline uint32_t crc32_be(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) {
      crc = (crc & 1) ? (0xEDB88320 ^ (crc >> 1)) : (crc >> 1);
    }
  }
  return crc ^ 0xFFFFFFFF;
}

// --------------------------------------------------------------------------
// Framed packet builder (reuses a per-thread buffer)
// --------------------------------------------------------------------------
inline std::vector<uint8_t>& make_packet(const uint8_t* data,
                                         size_t len,
                                         const char type[4]) {
  thread_local std::vector<uint8_t> pkt;

  const size_t kFixedOverhead = 4 + 4 + 4 + 4 + 4; // header + type + len + crc + footer
  const size_t required = kFixedOverhead + len;
  if (pkt.capacity() < required) pkt.reserve(required);
  pkt.clear();

  pkt.insert(pkt.end(), HEADER_MAGIC, HEADER_MAGIC + 4);
  pkt.insert(pkt.end(), type, type + 4);
  pkt.push_back((len >> 24) & 0xFF);
  pkt.push_back((len >> 16) & 0xFF);
  pkt.push_back((len >> 8) & 0xFF);
  pkt.push_back(len & 0xFF);
  if (len > 0) {
    assert(data != nullptr && "make_packet requires non-null data when len > 0");
    pkt.insert(pkt.end(), data, data + len);
  }
  const uint32_t crc = len > 0 ? crc32_be(data, len) : 0;
  pkt.push_back((crc >> 24) & 0xFF);
  pkt.push_back((crc >> 16) & 0xFF);
  pkt.push_back((crc >> 8) & 0xFF);
  pkt.push_back(crc & 0xFF);
  pkt.insert(pkt.end(), FOOTER_MAGIC, FOOTER_MAGIC + 4);
  return pkt;
}

inline std::vector<uint8_t>& make_packet(const std::vector<uint8_t>& payload,
                                         const char type[4]) {
  const uint8_t* ptr = payload.empty() ? nullptr : payload.data();
  return make_packet(ptr, payload.size(), type);
}

// --------------------------------------------------------------------------
// Frame parser
// --------------------------------------------------------------------------
struct Frame {
  std::array<char, 4> type{};
  std::vector<uint8_t> payload;
};

// Attempts to parse a single frame from buffer. Returns true if a full frame
// was consumed; `consumed` indicates bytes eaten (0 if need more data).
inline bool parse_frame(const std::vector<uint8_t>& buffer,
                        Frame& out,
                        size_t& consumed) {
  consumed = 0;
  const size_t min_size = 4 + 4 + 4 + 4 + 4;
  if (buffer.size() < min_size) return false;
  if (!std::equal(std::begin(HEADER_MAGIC), std::end(HEADER_MAGIC), buffer.begin())) {
    return false;
  }
  const uint32_t len = (static_cast<uint32_t>(buffer[8]) << 24) |
                       (static_cast<uint32_t>(buffer[9]) << 16) |
                       (static_cast<uint32_t>(buffer[10]) << 8) |
                       (static_cast<uint32_t>(buffer[11]));
  const size_t pkt_size = min_size + len;
  if (buffer.size() < pkt_size) return false; // need more data
  const size_t payload_start = 12;
  const size_t crc_start = payload_start + len;
  const size_t footer_start = crc_start + 4;
  if (!std::equal(std::begin(FOOTER_MAGIC), std::end(FOOTER_MAGIC), buffer.begin() + footer_start)) {
    consumed = pkt_size;
    return false;
  }
  const uint32_t recv_crc = (static_cast<uint32_t>(buffer[crc_start]) << 24) |
                            (static_cast<uint32_t>(buffer[crc_start + 1]) << 16) |
                            (static_cast<uint32_t>(buffer[crc_start + 2]) << 8) |
                            (static_cast<uint32_t>(buffer[crc_start + 3]));
  if (len > 0) {
    const uint32_t calc_crc = crc32_be(buffer.data() + payload_start, len);
    if (calc_crc != recv_crc) {
      consumed = pkt_size;
      return false;
    }
  }

  out.type[0] = static_cast<char>(buffer[4]);
  out.type[1] = static_cast<char>(buffer[5]);
  out.type[2] = static_cast<char>(buffer[6]);
  out.type[3] = static_cast<char>(buffer[7]);
  out.payload.assign(buffer.begin() + payload_start, buffer.begin() + payload_start + len);
  consumed = pkt_size;
  return true;
}

// --------------------------------------------------------------------------
// Payload structures
// --------------------------------------------------------------------------
struct ImuSample {
  uint64_t timestamp_ns;
  double ax;
  double ay;
  double az;
  double gx;
  double gy;
  double gz;
};

struct PoseConstraintSegment {
  float start[3];
  float end[3];
  uint8_t type = 0; // 0 = odometry, 1 = loop closure
};

struct Feature3D {
  uint16_t id;
  double x, y, z;
};

struct Point3DColor {
  float x, y, z;
  uint8_t r, g, b;
};

struct VizFeature {
  uint16_t x;
  uint16_t y;
  uint8_t status;
  uint16_t id;
};

struct VizDetection {
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
  std::string label;
};

struct VizMatch {
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
  uint8_t confidence;
};

struct VizPayload {
  uint8_t subtype = 0; // 0=features,1=detections,2=matches
  std::vector<VizFeature> features;
  std::vector<VizDetection> detections;
  std::vector<VizMatch> matches;
};

// --------------------------------------------------------------------------
// Payload builders
// --------------------------------------------------------------------------
inline std::vector<uint8_t> build_jpg_payload(uint64_t timestamp_ns,
                                              bool is_ref,
                                              const std::string& channel,
                                              const uint8_t* data,
                                              size_t len) {
  std::vector<uint8_t> payload;
  if (is_ref) {
    payload.resize(8 + len);
    write_u64_be(payload.data(), timestamp_ns);
    if (len > 0 && data) {
      std::memcpy(payload.data() + 8, data, len);
    }
  } else {
    const uint8_t channel_len = static_cast<uint8_t>(std::min<size_t>(255, channel.size()));
    payload.resize(8 + 1 + channel_len + len);
    size_t offset = 0;
    write_u64_be(payload.data() + offset, timestamp_ns);
    offset += 8;
    payload[offset] = channel_len; offset += 1;
    if (channel_len > 0) {
      std::memcpy(payload.data() + offset, channel.data(), channel_len);
      offset += channel_len;
    }
    if (len > 0 && data) {
      std::memcpy(payload.data() + offset, data, len);
    }
  }
  return payload;
}

inline std::vector<uint8_t> build_pose_payload(uint32_t pose_type,
                                               bool has_quat,
                                               bool is_keyframe,
                                               double x,
                                               double y,
                                               double z,
                                               const double* quat_or_null) {
  std::vector<uint8_t> payload;
  payload.reserve(4 + 4 + 8 * 3 + (has_quat ? 8 * 4 : 0));
  uint32_t flags = has_quat ? 1u : 0u;
  if (is_keyframe) flags |= (1u << 1);

  payload.resize(0);
  uint8_t buf[8];
  write_u32_be(buf, pose_type); payload.insert(payload.end(), buf, buf + 4);
  write_u32_be(buf, flags); payload.insert(payload.end(), buf, buf + 4);
  write_f64_be(buf, x); payload.insert(payload.end(), buf, buf + 8);
  write_f64_be(buf, y); payload.insert(payload.end(), buf, buf + 8);
  write_f64_be(buf, z); payload.insert(payload.end(), buf, buf + 8);
  if (has_quat && quat_or_null) {
    write_f64_be(buf, quat_or_null[0]); payload.insert(payload.end(), buf, buf + 8);
    write_f64_be(buf, quat_or_null[1]); payload.insert(payload.end(), buf, buf + 8);
    write_f64_be(buf, quat_or_null[2]); payload.insert(payload.end(), buf, buf + 8);
    write_f64_be(buf, quat_or_null[3]); payload.insert(payload.end(), buf, buf + 8);
  }
  return payload;
}

inline std::vector<uint8_t> build_constraints_payload(const std::vector<PoseConstraintSegment>& segments) {
  std::vector<uint8_t> payload;
  payload.reserve(4 + segments.size() * (1 + 6 * sizeof(float)));
  uint8_t buf[4];
  write_u32_be(buf, static_cast<uint32_t>(segments.size()));
  payload.insert(payload.end(), buf, buf + 4);
  for (const auto& seg : segments) {
    payload.push_back(seg.type);
    for (int i = 0; i < 3; ++i) { write_f32_be(buf, seg.start[i]); payload.insert(payload.end(), buf, buf + 4); }
    for (int i = 0; i < 3; ++i) { write_f32_be(buf, seg.end[i]);   payload.insert(payload.end(), buf, buf + 4); }
  }
  return payload;
}

inline std::vector<uint8_t> build_viz_payload(const VizPayload& in) {
  std::vector<uint8_t> payload;
  payload.reserve(3 + 8 * (in.features.size() + in.detections.size() + in.matches.size()));
  payload.push_back(in.subtype);
  uint8_t buf[8];
  const uint16_t count = (in.subtype == 0) ? static_cast<uint16_t>(in.features.size()) :
                        (in.subtype == 1) ? static_cast<uint16_t>(in.detections.size()) :
                                            static_cast<uint16_t>(in.matches.size());
  payload.push_back(static_cast<uint8_t>((count >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>(count & 0xFF));

  if (in.subtype == 0) {
    for (const auto& f : in.features) {
      payload.push_back(static_cast<uint8_t>((f.x >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(f.x & 0xFF));
      payload.push_back(static_cast<uint8_t>((f.y >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(f.y & 0xFF));
      payload.push_back(f.status);
      payload.push_back(static_cast<uint8_t>((f.id >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(f.id & 0xFF));
    }
  } else if (in.subtype == 1) {
    for (const auto& d : in.detections) {
      payload.push_back(static_cast<uint8_t>((d.x1 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(d.x1 & 0xFF));
      payload.push_back(static_cast<uint8_t>((d.y1 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(d.y1 & 0xFF));
      payload.push_back(static_cast<uint8_t>((d.x2 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(d.x2 & 0xFF));
      payload.push_back(static_cast<uint8_t>((d.y2 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(d.y2 & 0xFF));
      const uint8_t ll = static_cast<uint8_t>(std::min<size_t>(255, d.label.size()));
      payload.push_back(ll);
      payload.insert(payload.end(), d.label.data(), d.label.data() + ll);
    }
  } else if (in.subtype == 2) {
    for (const auto& m : in.matches) {
      payload.push_back(static_cast<uint8_t>((m.x1 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(m.x1 & 0xFF));
      payload.push_back(static_cast<uint8_t>((m.y1 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(m.y1 & 0xFF));
      payload.push_back(static_cast<uint8_t>((m.x2 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(m.x2 & 0xFF));
      payload.push_back(static_cast<uint8_t>((m.y2 >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(m.y2 & 0xFF));
      payload.push_back(m.confidence);
    }
  }
  return payload;
}

inline std::vector<uint8_t> build_imu_payload(const std::vector<ImuSample>& batch) {
  if (batch.empty()) return {};
  const uint32_t count = static_cast<uint32_t>(batch.size());
  const size_t sample_stride = 8 + 6 * sizeof(double);
  std::vector<uint8_t> payload;
  payload.resize(4 + static_cast<size_t>(count) * sample_stride);
  payload[0] = (count >> 24) & 0xFF;
  payload[1] = (count >> 16) & 0xFF;
  payload[2] = (count >> 8) & 0xFF;
  payload[3] = count & 0xFF;
  size_t offset = 4;
  uint8_t buf[8];
  for (const auto& s : batch) {
    write_u64_be(payload.data() + offset, s.timestamp_ns); offset += 8;
    write_f64_be(buf, s.ax); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
    write_f64_be(buf, s.ay); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
    write_f64_be(buf, s.az); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
    write_f64_be(buf, s.gx); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
    write_f64_be(buf, s.gy); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
    write_f64_be(buf, s.gz); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
  }
  return payload;
}

inline std::vector<uint8_t> build_status_payload(const std::string& text) {
  return std::vector<uint8_t>(text.begin(), text.end());
}

inline std::vector<uint8_t> build_fea3_payload(const std::vector<Feature3D>& features) {
  if (features.empty()) return std::vector<uint8_t>(2, 0);
  uint16_t count = static_cast<uint16_t>(features.size());
  std::vector<uint8_t> payload;
  payload.resize(2 + count * (sizeof(uint16_t) + 3 * sizeof(double)));
  payload[0] = (count >> 8) & 0xFF;
  payload[1] = count & 0xFF;
  size_t offset = 2;
  uint8_t buf[8];
  for (const auto& f : features) {
    payload[offset] = (f.id >> 8) & 0xFF;
    payload[offset + 1] = f.id & 0xFF;
    offset += 2;
    write_f64_be(buf, f.x); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
    write_f64_be(buf, f.y); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
    write_f64_be(buf, f.z); std::memcpy(payload.data() + offset, buf, 8); offset += 8;
  }
  return payload;
}

inline std::vector<uint8_t> build_pcld_payload(const std::vector<Point3DColor>& points,
                                               std::optional<float> point_size = std::nullopt) {
  if (points.empty()) return {};
  const uint32_t count = static_cast<uint32_t>(points.size());
  const bool include_size = point_size.has_value() && point_size.value() > 0.0f;
  std::vector<uint8_t> payload;
  payload.resize((include_size ? 8 : 4) + count * (sizeof(float) * 3 + 3));
  write_u32_be(payload.data(), count);
  size_t offset = 4;
  if (include_size) {
    write_f32_be(payload.data() + offset, point_size.value());
    offset += 4;
  }
  uint8_t buf[4];
  for (const auto& p : points) {
    write_f32_be(buf, p.x); std::memcpy(payload.data() + offset, buf, 4); offset += 4;
    write_f32_be(buf, p.y); std::memcpy(payload.data() + offset, buf, 4); offset += 4;
    write_f32_be(buf, p.z); std::memcpy(payload.data() + offset, buf, 4); offset += 4;
    payload[offset++] = p.r;
    payload[offset++] = p.g;
    payload[offset++] = p.b;
  }
  return payload;
}

// --------------------------------------------------------------------------
// Payload decoders (basic validation)
// --------------------------------------------------------------------------
inline bool decode_jpg_payload(const std::vector<uint8_t>& payload,
                               bool is_ref,
                               uint64_t& timestamp_ns,
                               std::string& channel,
                               std::vector<uint8_t>& data) {
  if (payload.size() < 8) return false;
  timestamp_ns = 0;
  for (int i = 0; i < 8; ++i) {
    timestamp_ns = (timestamp_ns << 8) | payload[i];
  }
  size_t offset = 8;
  if (is_ref) {
    data.assign(payload.begin() + offset, payload.end());
    channel.clear();
    return true;
  }
  if (payload.size() < offset + 1) return false;
  uint8_t clen = payload[offset]; offset += 1;
  if (payload.size() < offset + clen) return false;
  channel.assign(reinterpret_cast<const char*>(payload.data() + offset), clen);
  offset += clen;
  data.assign(payload.begin() + offset, payload.end());
  return true;
}

inline bool decode_pose_payload(const std::vector<uint8_t>& payload,
                                uint32_t& pose_type,
                                uint32_t& pose_flags,
                                double& x,
                                double& y,
                                double& z,
                                std::optional<std::array<double,4>>& quat) {
  if (payload.size() < 4 + 4 + 8 * 3) return false;
  auto rd_u32 = [&](size_t idx) -> uint32_t {
    return (static_cast<uint32_t>(payload[idx]) << 24) |
           (static_cast<uint32_t>(payload[idx + 1]) << 16) |
           (static_cast<uint32_t>(payload[idx + 2]) << 8) |
           (static_cast<uint32_t>(payload[idx + 3]));
  };
  auto rd_f64 = [&](size_t idx, double& out) {
    uint64_t tmp = 0;
    for (int i = 0; i < 8; ++i) tmp = (tmp << 8) | payload[idx + i];
    std::memcpy(&out, &tmp, 8);
  };
  size_t off = 0;
  pose_type = rd_u32(off); off += 4;
  pose_flags = rd_u32(off); off += 4;
  rd_f64(off, x); off += 8;
  rd_f64(off, y); off += 8;
  rd_f64(off, z); off += 8;
  if (pose_flags & 0x1) {
    if (payload.size() < off + 32) return false;
    std::array<double,4> q{};
    rd_f64(off, q[0]); off += 8;
    rd_f64(off, q[1]); off += 8;
    rd_f64(off, q[2]); off += 8;
    rd_f64(off, q[3]); off += 8;
    quat = q;
  } else {
    quat.reset();
  }
  return true;
}

inline bool decode_constraints_payload(const std::vector<uint8_t>& payload,
                                       std::vector<PoseConstraintSegment>& out) {
  if (payload.size() < 4) return false;
  uint32_t count = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3];
  size_t off = 4;
  const size_t bytes_per = 1 + 6 * 4;
  if (payload.size() < off + static_cast<size_t>(count) * bytes_per) return false;
  auto rd_f32 = [&](size_t idx, float& v) {
    uint32_t tmp = (payload[idx] << 24) | (payload[idx + 1] << 16) | (payload[idx + 2] << 8) | payload[idx + 3];
    std::memcpy(&v, &tmp, 4);
  };
  out.clear();
  out.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    PoseConstraintSegment s{};
    s.type = payload[off]; off += 1;
    for (int k = 0; k < 3; ++k) { rd_f32(off, s.start[k]); off += 4; }
    for (int k = 0; k < 3; ++k) { rd_f32(off, s.end[k]); off += 4; }
    out.push_back(s);
  }
  return true;
}

inline bool decode_viz_payload(const std::vector<uint8_t>& payload, VizPayload& out) {
  if (payload.size() < 3) return false;
  size_t off = 0;
  out.subtype = payload[off++];
  uint16_t count = (payload[off] << 8) | payload[off + 1]; off += 2;
  if (out.subtype == 0) {
    const size_t bytes_per = 2 + 2 + 1 + 2;
    if (payload.size() < off + bytes_per * count) return false;
    out.features.clear();
    out.features.reserve(count);
    for (uint16_t i = 0; i < count; ++i) {
      VizFeature vf{};
      vf.x = (payload[off] << 8) | payload[off + 1]; off += 2;
      vf.y = (payload[off] << 8) | payload[off + 1]; off += 2;
      vf.status = payload[off++]; 
      vf.id = (payload[off] << 8) | payload[off + 1]; off += 2;
      out.features.push_back(vf);
    }
  } else if (out.subtype == 1) {
    out.detections.clear();
    for (uint16_t i = 0; i < count; ++i) {
      if (off + 9 > payload.size()) return false;
      VizDetection det{};
      det.x1 = (payload[off] << 8) | payload[off + 1]; off += 2;
      det.y1 = (payload[off] << 8) | payload[off + 1]; off += 2;
      det.x2 = (payload[off] << 8) | payload[off + 1]; off += 2;
      det.y2 = (payload[off] << 8) | payload[off + 1]; off += 2;
      uint8_t ll = payload[off++];
      if (off + ll > payload.size()) return false;
      det.label.assign(reinterpret_cast<const char*>(payload.data() + off), ll);
      off += ll;
      out.detections.push_back(det);
    }
  } else if (out.subtype == 2) {
    const size_t bytes_per = 2 + 2 + 2 + 2 + 1;
    if (payload.size() < off + bytes_per * count) return false;
    out.matches.clear();
    out.matches.reserve(count);
    for (uint16_t i = 0; i < count; ++i) {
      VizMatch m{};
      m.x1 = (payload[off] << 8) | payload[off + 1]; off += 2;
      m.y1 = (payload[off] << 8) | payload[off + 1]; off += 2;
      m.x2 = (payload[off] << 8) | payload[off + 1]; off += 2;
      m.y2 = (payload[off] << 8) | payload[off + 1]; off += 2;
      m.confidence = payload[off++];
      out.matches.push_back(m);
    }
  } else {
    return false;
  }
  return true;
}

inline bool decode_imu_payload(const std::vector<uint8_t>& payload,
                               std::vector<ImuSample>& out) {
  if (payload.size() < 4) return false;
  uint32_t count = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3];
  const size_t stride = 8 + 6 * sizeof(double);
  if (payload.size() < 4 + stride * static_cast<size_t>(count)) return false;
  out.clear();
  out.reserve(count);
  size_t off = 4;
  for (uint32_t i = 0; i < count; ++i) {
    ImuSample s{};
    s.timestamp_ns = 0;
    for (int k = 0; k < 8; ++k) s.timestamp_ns = (s.timestamp_ns << 8) | payload[off++];
    auto rd_f64 = [&](double& v) {
      uint64_t tmp = 0;
      for (int k = 0; k < 8; ++k) tmp = (tmp << 8) | payload[off + k];
      std::memcpy(&v, &tmp, 8);
      off += 8;
    };
    rd_f64(s.ax); rd_f64(s.ay); rd_f64(s.az);
    rd_f64(s.gx); rd_f64(s.gy); rd_f64(s.gz);
    out.push_back(s);
  }
  return true;
}

inline bool decode_status_payload(const std::vector<uint8_t>& payload, std::string& text) {
  text.assign(payload.begin(), payload.end());
  return true;
}

inline bool decode_fea3_payload(const std::vector<uint8_t>& payload, std::vector<Feature3D>& out) {
  if (payload.size() < 2) return false;
  uint16_t count = (payload[0] << 8) | payload[1];
  const size_t bytes_per = 2 + 3 * sizeof(double);
  if (payload.size() < 2 + bytes_per * count) return false;
  out.clear();
  out.reserve(count);
  size_t off = 2;
  for (uint16_t i = 0; i < count; ++i) {
    Feature3D f{};
    f.id = (payload[off] << 8) | payload[off + 1]; off += 2;
    auto rd_f64 = [&](double& v) {
      uint64_t tmp = 0;
      for (int k = 0; k < 8; ++k) tmp = (tmp << 8) | payload[off + k];
      std::memcpy(&v, &tmp, 8);
      off += 8;
    };
    rd_f64(f.x); rd_f64(f.y); rd_f64(f.z);
    out.push_back(f);
  }
  return true;
}

inline bool decode_pcld_payload(const std::vector<uint8_t>& payload,
                                std::vector<Point3DColor>& out,
                                std::optional<float>& point_size) {
  if (payload.size() < 4) return false;
  uint32_t count = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3];
  size_t off = 4;
  point_size.reset();
  if (payload.size() >= 8 + count * (3 * sizeof(float) + 3)) {
    float ps;
    uint32_t tmp = (payload[off] << 24) | (payload[off + 1] << 16) | (payload[off + 2] << 8) | payload[off + 3];
    std::memcpy(&ps, &tmp, 4);
    point_size = ps;
    off += 4;
  }
  const size_t needed_without_size = 4 + count * (3 * sizeof(float) + 3);
  if (payload.size() < needed_without_size) return false;
  out.clear();
  out.reserve(count);
  auto rd_f32 = [&](float& v) {
    uint32_t tmp = (payload[off] << 24) | (payload[off + 1] << 16) | (payload[off + 2] << 8) | payload[off + 3];
    std::memcpy(&v, &tmp, 4);
    off += 4;
  };
  for (uint32_t i = 0; i < count; ++i) {
    Point3DColor p{};
    rd_f32(p.x); rd_f32(p.y); rd_f32(p.z);
    p.r = payload[off++]; p.g = payload[off++]; p.b = payload[off++];
    out.push_back(p);
  }
  return true;
}

} // namespace mighty_protocol
