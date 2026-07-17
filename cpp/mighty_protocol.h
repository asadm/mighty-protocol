#pragma once

#include <array>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mighty_protocol {

// Type codes
inline constexpr char TYPE_JPG[4]  = {'J','P','G',' '};
inline constexpr char TYPE_RJPG[4] = {'R','J','P','G'};
inline constexpr char TYPE_RAW[4]  = {'R','A','W',' '};
inline constexpr char TYPE_SRAW[4] = {'S','R','A','W'};
inline constexpr char TYPE_POSE[4] = {'P','O','S','E'};
inline constexpr char TYPE_UPOSE[4]= {'U','P','O','S'};
inline constexpr char TYPE_LCON[4] = {'L','C','O','N'};
inline constexpr char TYPE_VIZ[4]  = {'V','I','Z',' '};
inline constexpr char TYPE_IMU[4]  = {'I','M','U',' '};
inline constexpr char TYPE_STAT[4] = {'S','T','A','T'};
inline constexpr char TYPE_RSET[4] = {'R','S','E','T'};
inline constexpr char TYPE_FEA3[4] = {'F','E','A','3'};
inline constexpr char TYPE_PCLD[4] = {'P','C','L','D'};
inline constexpr char TYPE_VSTA[4] = {'V','S','T','A'};
inline constexpr char TYPE_LLOG[4] = {'L','L','O','G'};
inline constexpr char TYPE_KEYF[4] = {'K','E','Y','F'};
inline constexpr char TYPE_CMD[4]  = {'C','M','D',' '};
inline constexpr char TYPE_CRES[4] = {'C','R','E','S'};
inline constexpr char TYPE_CFGQ[4] = {'C','F','G','Q'};
inline constexpr char TYPE_CFGR[4] = {'C','F','G','R'};
inline constexpr char TYPE_EVNT[4] = {'E','V','N','T'};

// Raw frame formats (payload data is tightly packed)
enum class RawFormat : uint8_t {
  kUnknown = 0,
  kGray8 = 1,
  kRGB24 = 2,
  kBGR24 = 3,
  kRGBA32 = 4,
  kBGRA32 = 5,
  kYUV420SP = 6,
  kYUV420P = 7,
};

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
inline uint16_t read_u16_be(const uint8_t* src) {
  return static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 8) |
                               static_cast<uint16_t>(src[1]));
}
inline uint32_t read_u32_be(const uint8_t* src) {
  return (static_cast<uint32_t>(src[0]) << 24) |
         (static_cast<uint32_t>(src[1]) << 16) |
         (static_cast<uint32_t>(src[2]) << 8) |
         static_cast<uint32_t>(src[3]);
}
inline uint64_t read_u64_be(const uint8_t* src) {
  uint64_t value = 0;
  for (int i = 0; i < 8; ++i) {
    value = (value << 8) | static_cast<uint64_t>(src[i]);
  }
  return value;
}
inline float read_f32_be(const uint8_t* src) {
  const uint32_t raw = read_u32_be(src);
  float value = 0.0f;
  static_assert(sizeof(float) == sizeof(uint32_t), "float must be 4 bytes");
  std::memcpy(&value, &raw, sizeof(float));
  return value;
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
  const bool is_raw = (buffer[4] == 'R' && buffer[5] == 'A' && buffer[6] == 'W' && buffer[7] == ' ') ||
                      (buffer[4] == 'S' && buffer[5] == 'R' && buffer[6] == 'A' && buffer[7] == 'W');
  const uint32_t recv_crc = (static_cast<uint32_t>(buffer[crc_start]) << 24) |
                            (static_cast<uint32_t>(buffer[crc_start + 1]) << 16) |
                            (static_cast<uint32_t>(buffer[crc_start + 2]) << 8) |
                            (static_cast<uint32_t>(buffer[crc_start + 3]));
  if (len > 0) {
    const bool skip_crc = is_raw && recv_crc == 0;
    if (!skip_crc) {
      const uint32_t calc_crc = crc32_be(buffer.data() + payload_start, len);
      if (calc_crc != recv_crc) {
        consumed = pkt_size;
        return false;
      }
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

inline constexpr uint16_t KEYFRAME_FLAG_LOCAL_FEATURES = 1u << 0;

struct KeyframeFeature {
  float x = 0.0f;
  float y = 0.0f;
  float score = 0.0f;
  std::vector<float> descriptor;
};

struct KeyframeDescriptor {
  uint8_t version = 1;
  uint8_t descriptor_type = 1; // 1 = float32 descriptor
  uint16_t flags = 0;
  uint64_t timestamp_ns = 0;
  std::vector<float> descriptor;
  // Version 2 fields. Coordinates are pixels in the original image, not the
  // model's resized input image.
  uint32_t image_width = 0;
  uint32_t image_height = 0;
  uint16_t feature_descriptor_dim = 0;
  uint8_t feature_descriptor_type = 1; // 1 = float32 descriptor
  std::vector<KeyframeFeature> features;
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

struct VizAprilTag {
  uint32_t id = 0;
  uint8_t hamming = 0;
  float center_x = 0.0f;
  float center_y = 0.0f;
  std::array<float, 8> corners{};
};

struct VizPayload {
  uint8_t subtype = 0; // 0=features,1=detections,2=matches,3=apriltags,4=timestamped tracker
  uint64_t timestamp_ns = 0;
  std::vector<VizFeature> features;
  std::vector<VizDetection> detections;
  std::vector<VizMatch> matches;
  std::vector<VizAprilTag> apriltags;
};

struct CommandRequest {
  uint32_t req_id = 0;
  std::string name;
  std::vector<uint8_t> data;
};

struct CommandResponse {
  uint32_t req_id = 0;
  uint8_t status = 0; // 0=ok, 1=error
  std::string message;
  std::vector<uint8_t> data;
};

struct EventPayload {
  uint8_t version = 1;
  std::string kind;
  std::string json;
};

struct ResetVioPoseRequest {
  uint32_t pose_type = 0;
  uint32_t pose_flags = 0;
  std::array<double, 3> position_m{};
  std::optional<std::array<double, 4>> orientation_xyzw;
};

// Pixel coordinates in the primary camera image for the start_tracker command.
struct TrackerRectRequest {
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t width = 0;
  uint16_t height = 0;
};

enum class ConfigOp : uint8_t {
  kGet = 0,
  kSet = 1,
};

enum class VioStateCode : uint8_t {
  kOff = 0,
  kInitializing = 1,
  kTracking = 2,
  kDegraded = 3,
  kLost = 4,
  kLowLight = 5,
};

enum VioDegradedReason : uint32_t {
  kDegradedLowTracking = 1u << 0,
  kDegradedLowTranslationObservability = 1u << 1,
  kDegradedLowParallaxPoseHold = 1u << 2,
  kDegradedStationaryPoseHold = 1u << 3,
  kDegradedHighVelocityLowParallax = 1u << 4,
  kDegradedInitUncertain = 1u << 5,
  kStaticTranslationConstrained = 1u << 6,
  kRotationOnly3Dof = 1u << 7,
};

enum class VioInitReasonCode : uint8_t {
  kNone = 0,
  kWaitingForFirstImu = 1,
  kWaitingForInitFrames = 2,
  kWaitingForParallax = 3,
  kWaitingForImuExcitation = 4,
  kStaticInsufficientFeatures = 5,
  kStaticSceneMotionTooHigh = 6,
  kRelativePoseUnavailable = 7,
  kGlobalSfmFailed = 8,
  kPnpInsufficientPoints = 9,
  kPnpRansacFailed = 10,
  kVisualImuAlignmentFailed = 11,
  kUnknown = 12,
  kWaitingForFirstImuNoSamples = 13,
  kWaitingForFirstImuNotYetAligned = 14,
  kWaitingForFirstImuTimeOffsetInvalid = 15,
};

struct ConfigRequest {
  uint8_t version = 1;
  uint8_t op = static_cast<uint8_t>(ConfigOp::kGet);
  std::string key;
  std::vector<uint8_t> value;
};

struct ConfigResponse {
  uint8_t version = 1;
  uint8_t op = static_cast<uint8_t>(ConfigOp::kGet);
  uint8_t success = 0;
  bool has_value = false;
  std::string key;
  std::string message;
  std::vector<uint8_t> value;
};

struct VioState {
  // Payload layout (big-endian):
  //   u8  version
  //   u8  state   (enum chosen by producer)
  //   u16 flags   (bitfield chosen by producer)
  //   u64 timestamp_ns (typically pose/image timestamp; 0 if unknown)
  //   f32 fps_current
  //   f32 fps_average
  //   f32 pose_confidence01
  //   f32 tracking_rate
  //   u32 num_features
  //   u32 loop_closures
  //
  // Version 2+ appends:
  //   u8  build_version_len
  //   u8[build_version_len] build_version (ASCII)
  //
  // Version 3+ appends (after optional build_version):
  //   f32 imu_hz_current
  //   f32 imu_hz_average_5s
  //
  // Version 4+ appends:
  //   u8  init_reason_code
  //
  // Version 5+ appends:
  //   u8  static_init_reason_code
  //   u8  dynamic_init_reason_code
  //
  // Version 6+ appends:
  //   u64 memory_total_bytes
  //   u64 memory_used_bytes
  //   u64 memory_free_bytes
  //
  // Version 7+ appends:
  //   f32 light_level01     (producer-defined raw image-quality actual value)
  //   f32 light_required01  (producer-defined raw requirement/limit value)
  //
  // Version 8+ appends:
  //   f32 translation_confidence01
  //   f32 translation_observability01
  //   u32 degraded_reason_flags
  uint8_t version = 1;
  uint8_t state = 0;
  uint16_t flags = 0;
  uint64_t timestamp_ns = 0;
  float fps_current = 0.0f;
  float fps_average = 0.0f;
  float pose_confidence01 = 0.0f;
  float tracking_rate = 0.0f;
  uint32_t num_features = 0;
  uint32_t loop_closures = 0;
  std::string build_version;
  float imu_hz_current = 0.0f;
  float imu_hz_average_5s = 0.0f;
  uint8_t init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  uint8_t static_init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  uint8_t dynamic_init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  uint64_t memory_total_bytes = 0;
  uint64_t memory_used_bytes = 0;
  uint64_t memory_free_bytes = 0;
  float light_level01 = 1.0f;
  float light_required01 = 1.0f;
  float translation_confidence01 = 1.0f;
  float translation_observability01 = 1.0f;
  uint32_t degraded_reason_flags = 0;
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

inline std::vector<uint8_t> build_raw_payload(uint64_t timestamp_ns,
                                              const std::string& channel,
                                              uint32_t width,
                                              uint32_t height,
                                              uint8_t format,
                                              const uint8_t* data,
                                              size_t len) {
  const uint8_t channel_len = static_cast<uint8_t>(std::min<size_t>(255, channel.size()));
  std::vector<uint8_t> payload;
  payload.resize(8 + 4 + 4 + 1 + 1 + channel_len + len);
  size_t offset = 0;
  write_u64_be(payload.data() + offset, timestamp_ns); offset += 8;
  write_u32_be(payload.data() + offset, width); offset += 4;
  write_u32_be(payload.data() + offset, height); offset += 4;
  payload[offset++] = format;
  payload[offset++] = channel_len;
  if (channel_len > 0) {
    std::memcpy(payload.data() + offset, channel.data(), channel_len);
    offset += channel_len;
  }
  if (len > 0 && data) {
    std::memcpy(payload.data() + offset, data, len);
  }
  return payload;
}

inline std::vector<uint8_t> build_stereo_raw_payload(uint64_t left_timestamp_ns,
                                                     uint64_t right_timestamp_ns,
                                                     const std::string& left_channel,
                                                     uint32_t left_width,
                                                     uint32_t left_height,
                                                     uint8_t left_format,
                                                     const uint8_t* left_data,
                                                     size_t left_len,
                                                     const std::string& right_channel,
                                                     uint32_t right_width,
                                                     uint32_t right_height,
                                                     uint8_t right_format,
                                                     const uint8_t* right_data,
                                                     size_t right_len) {
  const uint8_t left_channel_len = static_cast<uint8_t>(std::min<size_t>(255, left_channel.size()));
  const uint8_t right_channel_len = static_cast<uint8_t>(std::min<size_t>(255, right_channel.size()));
  const uint32_t left_len_u32 = static_cast<uint32_t>(left_len);
  const uint32_t right_len_u32 = static_cast<uint32_t>(right_len);

  std::vector<uint8_t> payload;
  payload.resize(8 + 8 +
                 4 + 4 + 1 + 1 + left_channel_len +
                 4 + 4 + 1 + 1 + right_channel_len +
                 4 + 4 +
                 left_len_u32 + right_len_u32);
  size_t offset = 0;
  write_u64_be(payload.data() + offset, left_timestamp_ns); offset += 8;
  write_u64_be(payload.data() + offset, right_timestamp_ns); offset += 8;

  write_u32_be(payload.data() + offset, left_width); offset += 4;
  write_u32_be(payload.data() + offset, left_height); offset += 4;
  payload[offset++] = left_format;
  payload[offset++] = left_channel_len;
  if (left_channel_len > 0) {
    std::memcpy(payload.data() + offset, left_channel.data(), left_channel_len);
    offset += left_channel_len;
  }

  write_u32_be(payload.data() + offset, right_width); offset += 4;
  write_u32_be(payload.data() + offset, right_height); offset += 4;
  payload[offset++] = right_format;
  payload[offset++] = right_channel_len;
  if (right_channel_len > 0) {
    std::memcpy(payload.data() + offset, right_channel.data(), right_channel_len);
    offset += right_channel_len;
  }

  write_u32_be(payload.data() + offset, left_len_u32); offset += 4;
  write_u32_be(payload.data() + offset, right_len_u32); offset += 4;
  if (left_len_u32 > 0 && left_data) {
    std::memcpy(payload.data() + offset, left_data, left_len_u32);
    offset += left_len_u32;
  }
  if (right_len_u32 > 0 && right_data) {
    std::memcpy(payload.data() + offset, right_data, right_len_u32);
  }
  return payload;
}

inline std::vector<uint8_t> build_pose_payload(uint32_t pose_type,
                                               bool has_quat,
                                               bool is_keyframe,
                                               double x,
                                               double y,
                                               double z,
                                               const double* orientation_xyzw_or_null,
                                               float confidence01 = 1.0f,
                                               const double* linear_velocity_body_mps_or_null = nullptr,
                                               const double* angular_velocity_body_rps_or_null = nullptr,
                                               const double* linear_acceleration_body_mps2_or_null = nullptr,
                                               const double* angular_acceleration_body_rps2_or_null = nullptr,
                                               std::optional<uint64_t> timestamp_ns = std::nullopt) {
  std::vector<uint8_t> payload;
  // NOTE: payload is intentionally append-only for backward compatibility.
  // New fields must be added at the end so older decoders can ignore them.
  const bool has_timestamp = timestamp_ns.has_value() && timestamp_ns.value() > 0;
  payload.reserve(4 + 4 + 8 * 3 + (has_quat ? 8 * 4 : 0) + 4 +
                  (linear_velocity_body_mps_or_null ? 8 * 3 : 0) +
                  (angular_velocity_body_rps_or_null ? 8 * 3 : 0) +
                  (linear_acceleration_body_mps2_or_null ? 8 * 3 : 0) +
                  (angular_acceleration_body_rps2_or_null ? 8 * 3 : 0) +
                  (has_timestamp ? 8 : 0));

  // Pose flags (uint32 big-endian)
  // Bit 0: has_quat
  // Bit 1: is_keyframe
  // Bit 2: has_linear_velocity_body_mps (vx,vy,vz) float64[3]
  // Bit 3: has_angular_velocity_body_rps (wx,wy,wz) float64[3]
  // Bit 4: has_linear_acceleration_body_mps2 (ax,ay,az) float64[3]
  // Bit 5: has_angular_acceleration_body_rps2 (alphax,alphay,alphaz) float64[3]
  // Bit 6: has_timestamp (timestamp_ns) uint64 appended at the end
  uint32_t flags = has_quat ? 1u : 0u;
  if (is_keyframe) flags |= (1u << 1);
  if (linear_velocity_body_mps_or_null) flags |= (1u << 2);
  if (angular_velocity_body_rps_or_null) flags |= (1u << 3);
  if (linear_acceleration_body_mps2_or_null) flags |= (1u << 4);
  if (angular_acceleration_body_rps2_or_null) flags |= (1u << 5);
  if (has_timestamp) flags |= (1u << 6);

  payload.resize(0);
  uint8_t buf8[8];
  uint8_t buf4[4];
  write_u32_be(buf4, pose_type); payload.insert(payload.end(), buf4, buf4 + 4);
  write_u32_be(buf4, flags); payload.insert(payload.end(), buf4, buf4 + 4);
  write_f64_be(buf8, x); payload.insert(payload.end(), buf8, buf8 + 8);
  write_f64_be(buf8, y); payload.insert(payload.end(), buf8, buf8 + 8);
  write_f64_be(buf8, z); payload.insert(payload.end(), buf8, buf8 + 8);
  if (has_quat && orientation_xyzw_or_null) {
    write_f64_be(buf8, orientation_xyzw_or_null[0]); payload.insert(payload.end(), buf8, buf8 + 8);
    write_f64_be(buf8, orientation_xyzw_or_null[1]); payload.insert(payload.end(), buf8, buf8 + 8);
    write_f64_be(buf8, orientation_xyzw_or_null[2]); payload.insert(payload.end(), buf8, buf8 + 8);
    write_f64_be(buf8, orientation_xyzw_or_null[3]); payload.insert(payload.end(), buf8, buf8 + 8);
  }
  // Confidence in [0,1]. (float32 big-endian)
  if (!std::isfinite(confidence01)) confidence01 = 0.0f;
  confidence01 = std::min(1.0f, std::max(0.0f, confidence01));
  write_f32_be(buf4, confidence01);
  payload.insert(payload.end(), buf4, buf4 + 4);

  auto append_f64x3 = [&](const double* v) {
    write_f64_be(buf8, v[0]); payload.insert(payload.end(), buf8, buf8 + 8);
    write_f64_be(buf8, v[1]); payload.insert(payload.end(), buf8, buf8 + 8);
    write_f64_be(buf8, v[2]); payload.insert(payload.end(), buf8, buf8 + 8);
  };
  if (linear_velocity_body_mps_or_null) append_f64x3(linear_velocity_body_mps_or_null);
  if (angular_velocity_body_rps_or_null) append_f64x3(angular_velocity_body_rps_or_null);
  if (linear_acceleration_body_mps2_or_null) append_f64x3(linear_acceleration_body_mps2_or_null);
  if (angular_acceleration_body_rps2_or_null) append_f64x3(angular_acceleration_body_rps2_or_null);
  if (has_timestamp) {
    write_u64_be(buf8, timestamp_ns.value());
    payload.insert(payload.end(), buf8, buf8 + 8);
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
  payload.reserve(3 + (in.subtype == 4 ? 8 : 0) +
                  8 * (in.features.size() + in.detections.size() + in.matches.size()) +
                  45 * in.apriltags.size());
  payload.push_back(in.subtype);
  uint8_t buf[8];
  const uint16_t count = (in.subtype == 0) ? static_cast<uint16_t>(in.features.size()) :
                        (in.subtype == 1 || in.subtype == 4) ? static_cast<uint16_t>(in.detections.size()) :
                        (in.subtype == 2) ? static_cast<uint16_t>(in.matches.size()) :
                                            static_cast<uint16_t>(in.apriltags.size());
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
  } else if (in.subtype == 1 || in.subtype == 4) {
    if (in.subtype == 4) {
      write_u64_be(buf, in.timestamp_ns);
      payload.insert(payload.end(), buf, buf + 8);
    }
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
  } else if (in.subtype == 3) {
    for (const auto& tag : in.apriltags) {
      write_u32_be(buf, tag.id);
      payload.insert(payload.end(), buf, buf + 4);
      payload.push_back(tag.hamming);
      write_f32_be(buf, tag.center_x);
      payload.insert(payload.end(), buf, buf + 4);
      write_f32_be(buf, tag.center_y);
      payload.insert(payload.end(), buf, buf + 4);
      for (float coord : tag.corners) {
        write_f32_be(buf, coord);
        payload.insert(payload.end(), buf, buf + 4);
      }
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

inline std::vector<uint8_t> build_event_payload(const std::string& kind,
                                                const std::string& json) {
  const uint8_t kind_len = static_cast<uint8_t>(std::min<size_t>(255, kind.size()));
  const uint32_t json_len = static_cast<uint32_t>(
      std::min<size_t>(std::numeric_limits<uint32_t>::max(), json.size()));
  std::vector<uint8_t> payload;
  payload.resize(1 + 1 + kind_len + 4 + json_len);
  size_t off = 0;
  payload[off++] = 1;
  payload[off++] = kind_len;
  if (kind_len > 0) {
    std::memcpy(payload.data() + off, kind.data(), kind_len);
    off += kind_len;
  }
  write_u32_be(payload.data() + off, json_len);
  off += 4;
  if (json_len > 0) {
    std::memcpy(payload.data() + off, json.data(), json_len);
  }
  return payload;
}

inline std::vector<uint8_t> build_lua_log_payload(uint32_t seq, const std::string& text) {
  std::vector<uint8_t> payload;
  payload.resize(4 + text.size());
  write_u32_be(payload.data(), seq);
  if (!text.empty()) {
    std::memcpy(payload.data() + 4, text.data(), text.size());
  }
  return payload;
}

inline std::vector<uint8_t> build_keyframe_payload(uint64_t timestamp_ns,
                                                   const std::vector<float>& descriptor,
                                                   uint16_t flags = 0) {
  const uint32_t dim = static_cast<uint32_t>(descriptor.size());
  std::vector<uint8_t> payload;
  payload.resize(1 + 1 + 2 + 8 + 4 + static_cast<size_t>(dim) * 4);
  size_t off = 0;
  payload[off++] = 1; // version
  payload[off++] = 1; // descriptor_type: float32
  payload[off++] = static_cast<uint8_t>((flags >> 8) & 0xFF);
  payload[off++] = static_cast<uint8_t>(flags & 0xFF);
  write_u64_be(payload.data() + off, timestamp_ns); off += 8;
  write_u32_be(payload.data() + off, dim); off += 4;
  for (float value : descriptor) {
    write_f32_be(payload.data() + off, value);
    off += 4;
  }
  return payload;
}

inline std::vector<uint8_t> build_keyframe_payload(const KeyframeDescriptor& keyframe) {
  if (keyframe.version == 1) {
    return build_keyframe_payload(
        keyframe.timestamp_ns, keyframe.descriptor, keyframe.flags);
  }
  if (keyframe.version != 2 || keyframe.descriptor_type != 1 ||
      keyframe.feature_descriptor_type != 1 ||
      keyframe.descriptor.size() > std::numeric_limits<uint32_t>::max() ||
      keyframe.features.size() > std::numeric_limits<uint32_t>::max()) {
    return {};
  }

  uint16_t feature_dim = keyframe.feature_descriptor_dim;
  if (feature_dim == 0 && !keyframe.features.empty()) {
    feature_dim = static_cast<uint16_t>(std::min<size_t>(
        std::numeric_limits<uint16_t>::max(),
        keyframe.features.front().descriptor.size()));
  }

  if (keyframe.descriptor.size() >
      std::numeric_limits<size_t>::max() / sizeof(float)) {
    return {};
  }
  const size_t descriptor_bytes = keyframe.descriptor.size() * sizeof(float);
  const size_t feature_stride = 3 * sizeof(float) +
                                static_cast<size_t>(feature_dim) * sizeof(float);
  constexpr size_t kV1HeaderBytes = 1 + 1 + 2 + 8 + 4;
  constexpr size_t kV2ExtensionHeaderBytes = 4 + 4 + 4 + 2 + 1 + 1;
  if (descriptor_bytes > std::numeric_limits<size_t>::max() -
                             kV1HeaderBytes - kV2ExtensionHeaderBytes) {
    return {};
  }
  if (keyframe.features.size() >
      (std::numeric_limits<size_t>::max() - kV1HeaderBytes -
       kV2ExtensionHeaderBytes - descriptor_bytes) / feature_stride) {
    return {};
  }

  std::vector<uint8_t> payload;
  payload.resize(kV1HeaderBytes + descriptor_bytes + kV2ExtensionHeaderBytes +
                 keyframe.features.size() * feature_stride);
  size_t off = 0;
  payload[off++] = 2;
  payload[off++] = keyframe.descriptor_type;
  payload[off++] = static_cast<uint8_t>((keyframe.flags >> 8) & 0xFF);
  payload[off++] = static_cast<uint8_t>(keyframe.flags & 0xFF);
  write_u64_be(payload.data() + off, keyframe.timestamp_ns); off += 8;
  write_u32_be(payload.data() + off,
               static_cast<uint32_t>(keyframe.descriptor.size()));
  off += 4;
  for (float value : keyframe.descriptor) {
    write_f32_be(payload.data() + off, value);
    off += 4;
  }
  write_u32_be(payload.data() + off, keyframe.image_width); off += 4;
  write_u32_be(payload.data() + off, keyframe.image_height); off += 4;
  write_u32_be(payload.data() + off,
               static_cast<uint32_t>(keyframe.features.size()));
  off += 4;
  payload[off++] = static_cast<uint8_t>((feature_dim >> 8) & 0xFF);
  payload[off++] = static_cast<uint8_t>(feature_dim & 0xFF);
  payload[off++] = keyframe.feature_descriptor_type;
  payload[off++] = 0; // reserved
  for (const KeyframeFeature& feature : keyframe.features) {
    write_f32_be(payload.data() + off, feature.x); off += 4;
    write_f32_be(payload.data() + off, feature.y); off += 4;
    write_f32_be(payload.data() + off, feature.score); off += 4;
    for (size_t i = 0; i < feature_dim; ++i) {
      const float value = i < feature.descriptor.size()
                              ? feature.descriptor[i]
                              : 0.0f;
      write_f32_be(payload.data() + off, value);
      off += 4;
    }
  }
  return payload;
}

inline std::vector<uint8_t> build_vio_state_payload(const VioState& s) {
  const bool include_build = (s.version >= 2);
  const bool include_imu_hz = (s.version >= 3);
  const bool include_init_reason = (s.version >= 4);
  const bool include_split_init_reasons = (s.version >= 5);
  const bool include_memory = (s.version >= 6);
  const bool include_light = (s.version >= 7);
  const bool include_translation_confidence = (s.version >= 8);
  const uint8_t build_len = static_cast<uint8_t>(std::min<size_t>(255, s.build_version.size()));
  std::vector<uint8_t> payload;
  payload.resize(1 + 1 + 2 + 8 + 4 + 4 + 4 + 4 + 4 + 4 +
                 (include_build ? (1 + build_len) : 0) +
                 (include_imu_hz ? (4 + 4) : 0) +
                 (include_init_reason ? 1 : 0) +
                 (include_split_init_reasons ? 2 : 0) +
                 (include_memory ? (8 + 8 + 8) : 0) +
                 (include_light ? (4 + 4) : 0) +
                 (include_translation_confidence ? (4 + 4 + 4) : 0));
  size_t off = 0;
  payload[off++] = s.version;
  payload[off++] = s.state;
  payload[off++] = static_cast<uint8_t>((s.flags >> 8) & 0xFF);
  payload[off++] = static_cast<uint8_t>(s.flags & 0xFF);
  write_u64_be(payload.data() + off, s.timestamp_ns); off += 8;
  write_f32_be(payload.data() + off, s.fps_current); off += 4;
  write_f32_be(payload.data() + off, s.fps_average); off += 4;
  write_f32_be(payload.data() + off, s.pose_confidence01); off += 4;
  write_f32_be(payload.data() + off, s.tracking_rate); off += 4;
  write_u32_be(payload.data() + off, s.num_features); off += 4;
  write_u32_be(payload.data() + off, s.loop_closures); off += 4;
  if (include_build) {
    payload[off++] = build_len;
    if (build_len > 0) {
      std::memcpy(payload.data() + off, s.build_version.data(), build_len);
      off += build_len;
    }
  }
  if (include_imu_hz) {
    write_f32_be(payload.data() + off, s.imu_hz_current); off += 4;
    write_f32_be(payload.data() + off, s.imu_hz_average_5s); off += 4;
  }
  if (include_init_reason) {
    payload[off++] = s.init_reason_code;
  }
  if (include_split_init_reasons) {
    payload[off++] = s.static_init_reason_code;
    payload[off++] = s.dynamic_init_reason_code;
  }
  if (include_memory) {
    write_u64_be(payload.data() + off, s.memory_total_bytes); off += 8;
    write_u64_be(payload.data() + off, s.memory_used_bytes); off += 8;
    write_u64_be(payload.data() + off, s.memory_free_bytes); off += 8;
  }
  if (include_light) {
    write_f32_be(payload.data() + off, s.light_level01); off += 4;
    write_f32_be(payload.data() + off, s.light_required01); off += 4;
  }
  if (include_translation_confidence) {
    write_f32_be(payload.data() + off, s.translation_confidence01); off += 4;
    write_f32_be(payload.data() + off, s.translation_observability01); off += 4;
    write_u32_be(payload.data() + off, s.degraded_reason_flags); off += 4;
  }
  return payload;
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

inline std::vector<uint8_t> build_command_payload(uint32_t req_id,
                                                  const std::string& name,
                                                  const uint8_t* data,
                                                  size_t len) {
  const uint8_t name_len = static_cast<uint8_t>(std::min<size_t>(255, name.size()));
  std::vector<uint8_t> payload;
  payload.reserve(4 + 1 + name_len + 4 + len);
  uint8_t buf[8];
  write_u32_be(buf, req_id); payload.insert(payload.end(), buf, buf + 4);
  payload.push_back(name_len);
  payload.insert(payload.end(), name.data(), name.data() + name_len);
  write_u32_be(buf, static_cast<uint32_t>(len)); payload.insert(payload.end(), buf, buf + 4);
  if (len > 0 && data) {
    payload.insert(payload.end(), data, data + len);
  }
  return payload;
}

inline std::vector<uint8_t> build_command_payload(uint32_t req_id,
                                                  const std::string& name,
                                                  const std::vector<uint8_t>& data) {
  const uint8_t* ptr = data.empty() ? nullptr : data.data();
  return build_command_payload(req_id, name, ptr, data.size());
}

inline std::vector<uint8_t> build_reset_vio_pose_payload(
    const std::array<double, 3>& position_m,
    const std::optional<std::array<double, 4>>& orientation_xyzw = std::nullopt) {
  const double* orientation_ptr =
      orientation_xyzw.has_value() ? orientation_xyzw->data() : nullptr;
  return build_pose_payload(/*pose_type=*/0,
                            /*has_quat=*/orientation_ptr != nullptr,
                            /*is_keyframe=*/false,
                            position_m[0], position_m[1], position_m[2],
                            orientation_ptr,
                            /*confidence01=*/1.0f);
}

inline std::vector<uint8_t> build_tracker_rect_payload(const TrackerRectRequest& rect) {
  std::vector<uint8_t> payload(8);
  payload[0] = static_cast<uint8_t>((rect.x >> 8) & 0xFF);
  payload[1] = static_cast<uint8_t>(rect.x & 0xFF);
  payload[2] = static_cast<uint8_t>((rect.y >> 8) & 0xFF);
  payload[3] = static_cast<uint8_t>(rect.y & 0xFF);
  payload[4] = static_cast<uint8_t>((rect.width >> 8) & 0xFF);
  payload[5] = static_cast<uint8_t>(rect.width & 0xFF);
  payload[6] = static_cast<uint8_t>((rect.height >> 8) & 0xFF);
  payload[7] = static_cast<uint8_t>(rect.height & 0xFF);
  return payload;
}

inline std::vector<uint8_t> build_command_response_payload(const CommandResponse& res) {
  const uint16_t msg_len = static_cast<uint16_t>(std::min<size_t>(65535, res.message.size()));
  const uint32_t data_len = static_cast<uint32_t>(res.data.size());
  std::vector<uint8_t> payload;
  payload.reserve(4 + 1 + 2 + msg_len + 4 + data_len);
  uint8_t buf[8];
  write_u32_be(buf, res.req_id); payload.insert(payload.end(), buf, buf + 4);
  payload.push_back(res.status);
  payload.push_back(static_cast<uint8_t>((msg_len >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>(msg_len & 0xFF));
  payload.insert(payload.end(), res.message.data(), res.message.data() + msg_len);
  write_u32_be(buf, data_len); payload.insert(payload.end(), buf, buf + 4);
  if (data_len > 0) {
    payload.insert(payload.end(), res.data.begin(), res.data.end());
  }
  return payload;
}

inline std::vector<uint8_t> build_config_request_payload(const ConfigRequest& req) {
  const uint8_t key_len = static_cast<uint8_t>(std::min<size_t>(255, req.key.size()));
  const uint32_t value_len = static_cast<uint32_t>(req.value.size());
  std::vector<uint8_t> payload;
  payload.reserve(1 + 1 + 1 + key_len + 4 + value_len);
  uint8_t buf[8];
  payload.push_back(req.version);
  payload.push_back(req.op);
  payload.push_back(key_len);
  payload.insert(payload.end(), req.key.data(), req.key.data() + key_len);
  write_u32_be(buf, value_len); payload.insert(payload.end(), buf, buf + 4);
  if (value_len > 0) {
    payload.insert(payload.end(), req.value.begin(), req.value.end());
  }
  return payload;
}

inline std::vector<uint8_t> build_config_response_payload(const ConfigResponse& res) {
  const uint8_t key_len = static_cast<uint8_t>(std::min<size_t>(255, res.key.size()));
  const uint16_t msg_len = static_cast<uint16_t>(std::min<size_t>(65535, res.message.size()));
  const uint32_t value_len = static_cast<uint32_t>(res.value.size());
  std::vector<uint8_t> payload;
  payload.reserve(1 + 1 + 1 + 1 + 1 + key_len + 2 + msg_len + 4 + value_len);
  uint8_t buf[8];
  payload.push_back(res.version);
  payload.push_back(res.op);
  payload.push_back(res.success);
  payload.push_back(res.has_value ? 1 : 0);
  payload.push_back(key_len);
  payload.insert(payload.end(), res.key.data(), res.key.data() + key_len);
  payload.push_back(static_cast<uint8_t>((msg_len >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>(msg_len & 0xFF));
  payload.insert(payload.end(), res.message.data(), res.message.data() + msg_len);
  write_u32_be(buf, value_len); payload.insert(payload.end(), buf, buf + 4);
  if (value_len > 0) {
    payload.insert(payload.end(), res.value.begin(), res.value.end());
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

inline bool decode_raw_payload(const std::vector<uint8_t>& payload,
                               uint64_t& timestamp_ns,
                               uint32_t& width,
                               uint32_t& height,
                               uint8_t& format,
                               std::string& channel,
                               std::vector<uint8_t>& data) {
  if (payload.size() < 8 + 4 + 4 + 1 + 1) return false;
  timestamp_ns = 0;
  for (int i = 0; i < 8; ++i) {
    timestamp_ns = (timestamp_ns << 8) | payload[i];
  }
  size_t offset = 8;
  width = (static_cast<uint32_t>(payload[offset]) << 24) |
          (static_cast<uint32_t>(payload[offset + 1]) << 16) |
          (static_cast<uint32_t>(payload[offset + 2]) << 8) |
          (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  height = (static_cast<uint32_t>(payload[offset]) << 24) |
           (static_cast<uint32_t>(payload[offset + 1]) << 16) |
           (static_cast<uint32_t>(payload[offset + 2]) << 8) |
           (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  format = payload[offset]; offset += 1;
  uint8_t clen = payload[offset]; offset += 1;
  if (payload.size() < offset + clen) return false;
  channel.assign(reinterpret_cast<const char*>(payload.data() + offset), clen);
  offset += clen;
  data.assign(payload.begin() + offset, payload.end());
  return true;
}

inline bool decode_stereo_raw_payload(const std::vector<uint8_t>& payload,
                                      uint64_t& left_timestamp_ns,
                                      uint64_t& right_timestamp_ns,
                                      uint32_t& left_width,
                                      uint32_t& left_height,
                                      uint8_t& left_format,
                                      std::string& left_channel,
                                      std::vector<uint8_t>& left_data,
                                      uint32_t& right_width,
                                      uint32_t& right_height,
                                      uint8_t& right_format,
                                      std::string& right_channel,
                                      std::vector<uint8_t>& right_data) {
  if (payload.size() < 8 + 8 + 4 + 4 + 1 + 1 + 4 + 4 + 1 + 1 + 4 + 4) return false;
  size_t offset = 0;
  left_timestamp_ns = 0;
  for (int i = 0; i < 8; ++i) {
    left_timestamp_ns = (left_timestamp_ns << 8) | payload[offset + i];
  }
  offset += 8;
  right_timestamp_ns = 0;
  for (int i = 0; i < 8; ++i) {
    right_timestamp_ns = (right_timestamp_ns << 8) | payload[offset + i];
  }
  offset += 8;

  left_width = (static_cast<uint32_t>(payload[offset]) << 24) |
               (static_cast<uint32_t>(payload[offset + 1]) << 16) |
               (static_cast<uint32_t>(payload[offset + 2]) << 8) |
               (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  left_height = (static_cast<uint32_t>(payload[offset]) << 24) |
                (static_cast<uint32_t>(payload[offset + 1]) << 16) |
                (static_cast<uint32_t>(payload[offset + 2]) << 8) |
                (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  left_format = payload[offset++];
  uint8_t left_clen = payload[offset++];
  if (payload.size() < offset + left_clen) return false;
  left_channel.assign(reinterpret_cast<const char*>(payload.data() + offset), left_clen);
  offset += left_clen;

  if (payload.size() < offset + 4 + 4 + 1 + 1) return false;
  right_width = (static_cast<uint32_t>(payload[offset]) << 24) |
                (static_cast<uint32_t>(payload[offset + 1]) << 16) |
                (static_cast<uint32_t>(payload[offset + 2]) << 8) |
                (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  right_height = (static_cast<uint32_t>(payload[offset]) << 24) |
                 (static_cast<uint32_t>(payload[offset + 1]) << 16) |
                 (static_cast<uint32_t>(payload[offset + 2]) << 8) |
                 (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  right_format = payload[offset++];
  uint8_t right_clen = payload[offset++];
  if (payload.size() < offset + right_clen) return false;
  right_channel.assign(reinterpret_cast<const char*>(payload.data() + offset), right_clen);
  offset += right_clen;

  if (payload.size() < offset + 8) return false;
  uint32_t left_len = (static_cast<uint32_t>(payload[offset]) << 24) |
                      (static_cast<uint32_t>(payload[offset + 1]) << 16) |
                      (static_cast<uint32_t>(payload[offset + 2]) << 8) |
                      (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  uint32_t right_len = (static_cast<uint32_t>(payload[offset]) << 24) |
                       (static_cast<uint32_t>(payload[offset + 1]) << 16) |
                       (static_cast<uint32_t>(payload[offset + 2]) << 8) |
                       (static_cast<uint32_t>(payload[offset + 3]));
  offset += 4;
  if (payload.size() < offset + left_len + right_len) return false;
  left_data.assign(payload.begin() + offset, payload.begin() + offset + left_len);
  offset += left_len;
  right_data.assign(payload.begin() + offset, payload.begin() + offset + right_len);
  return true;
}

inline bool decode_pose_payload(const std::vector<uint8_t>& payload,
                                uint32_t& pose_type,
                                uint32_t& pose_flags,
                                double& x,
                                double& y,
                                double& z,
                                std::optional<std::array<double,4>>& orientation_xyzw,
                                float* confidence01_or_null = nullptr,
                                std::optional<std::array<double,3>>* linear_velocity_body_mps_or_null = nullptr,
                                std::optional<std::array<double,3>>* angular_velocity_body_rps_or_null = nullptr,
                                std::optional<std::array<double,3>>* linear_acceleration_body_mps2_or_null = nullptr,
                                std::optional<std::array<double,3>>* angular_acceleration_body_rps2_or_null = nullptr,
                                std::optional<uint64_t>* timestamp_ns_or_null = nullptr) {
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
  auto rd_f32 = [&](size_t idx, float& out) {
    uint32_t tmp = (static_cast<uint32_t>(payload[idx]) << 24) |
                   (static_cast<uint32_t>(payload[idx + 1]) << 16) |
                   (static_cast<uint32_t>(payload[idx + 2]) << 8) |
                   (static_cast<uint32_t>(payload[idx + 3]));
    std::memcpy(&out, &tmp, 4);
  };
  auto rd_f64x3 = [&](size_t idx, std::array<double,3>& out) {
    rd_f64(idx + 0, out[0]);
    rd_f64(idx + 8, out[1]);
    rd_f64(idx + 16, out[2]);
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
    orientation_xyzw = q;
  } else {
    orientation_xyzw.reset();
  }
  float conf = 1.0f;
  if (payload.size() >= off + 4) {
    rd_f32(off, conf);
  }
  if (!std::isfinite(conf)) conf = 0.0f;
  conf = std::min(1.0f, std::max(0.0f, conf));
  if (confidence01_or_null) *confidence01_or_null = conf;
  off += 4;

  auto decode_vec3_if_present = [&](uint32_t flag_bit,
                                    std::optional<std::array<double,3>>* out_opt) {
    if (!out_opt) return;
    if ((pose_flags & (1u << flag_bit)) == 0) {
      out_opt->reset();
      return;
    }
    if (payload.size() < off + 24) {
      out_opt->reset();
      return;
    }
    std::array<double,3> v{};
    rd_f64x3(off, v);
    *out_opt = v;
    off += 24;
  };

  decode_vec3_if_present(/*flag_bit=*/2, linear_velocity_body_mps_or_null);
  decode_vec3_if_present(/*flag_bit=*/3, angular_velocity_body_rps_or_null);
  decode_vec3_if_present(/*flag_bit=*/4, linear_acceleration_body_mps2_or_null);
  decode_vec3_if_present(/*flag_bit=*/5, angular_acceleration_body_rps2_or_null);

  if (timestamp_ns_or_null) {
    if ((pose_flags & (1u << 6)) == 0) {
      timestamp_ns_or_null->reset();
    } else if (payload.size() < off + 8) {
      timestamp_ns_or_null->reset();
    } else {
      uint64_t ts = 0;
      for (int i = 0; i < 8; ++i) ts = (ts << 8) | payload[off + i];
      *timestamp_ns_or_null = ts;
      off += 8;
    }
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
  out.timestamp_ns = 0;
  uint16_t count = read_u16_be(payload.data() + off); off += 2;
  if (out.subtype == 0) {
    const size_t bytes_per = 2 + 2 + 1 + 2;
    if (payload.size() < off + bytes_per * count) return false;
    out.features.clear();
    out.features.reserve(count);
    for (uint16_t i = 0; i < count; ++i) {
      VizFeature vf{};
      vf.x = read_u16_be(payload.data() + off); off += 2;
      vf.y = read_u16_be(payload.data() + off); off += 2;
      vf.status = payload[off++]; 
      vf.id = read_u16_be(payload.data() + off); off += 2;
      out.features.push_back(vf);
    }
  } else if (out.subtype == 1 || out.subtype == 4) {
    if (out.subtype == 4) {
      if (off + 8 > payload.size()) return false;
      out.timestamp_ns = read_u64_be(payload.data() + off);
      off += 8;
    }
    out.detections.clear();
    for (uint16_t i = 0; i < count; ++i) {
      if (off + 9 > payload.size()) return false;
      VizDetection det{};
      det.x1 = read_u16_be(payload.data() + off); off += 2;
      det.y1 = read_u16_be(payload.data() + off); off += 2;
      det.x2 = read_u16_be(payload.data() + off); off += 2;
      det.y2 = read_u16_be(payload.data() + off); off += 2;
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
      m.x1 = read_u16_be(payload.data() + off); off += 2;
      m.y1 = read_u16_be(payload.data() + off); off += 2;
      m.x2 = read_u16_be(payload.data() + off); off += 2;
      m.y2 = read_u16_be(payload.data() + off); off += 2;
      m.confidence = payload[off++];
      out.matches.push_back(m);
    }
  } else if (out.subtype == 3) {
    const size_t bytes_per = 4 + 1 + 2 * 4 + 8 * 4;
    if (payload.size() < off + bytes_per * count) return false;
    out.apriltags.clear();
    out.apriltags.reserve(count);
    for (uint16_t i = 0; i < count; ++i) {
      VizAprilTag tag{};
      tag.id = read_u32_be(payload.data() + off); off += 4;
      tag.hamming = payload[off++];
      tag.center_x = read_f32_be(payload.data() + off); off += 4;
      tag.center_y = read_f32_be(payload.data() + off); off += 4;
      for (float& coord : tag.corners) {
        coord = read_f32_be(payload.data() + off);
        off += 4;
      }
      out.apriltags.push_back(tag);
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

inline bool decode_event_payload(const std::vector<uint8_t>& payload, EventPayload& out) {
  if (payload.size() < 1 + 1 + 4) return false;
  size_t off = 0;
  out.version = payload[off++];
  const uint8_t kind_len = payload[off++];
  if (payload.size() < off + kind_len + 4) return false;
  out.kind.assign(reinterpret_cast<const char*>(payload.data() + off), kind_len);
  off += kind_len;
  const uint32_t json_len = read_u32_be(payload.data() + off);
  off += 4;
  if (payload.size() < off + json_len) return false;
  out.json.assign(reinterpret_cast<const char*>(payload.data() + off), json_len);
  return true;
}

inline bool decode_keyframe_payload(const std::vector<uint8_t>& payload,
                                    KeyframeDescriptor& out) {
  if (payload.size() < 16) return false;
  size_t off = 0;
  KeyframeDescriptor decoded;
  decoded.version = payload[off++];
  decoded.descriptor_type = payload[off++];
  decoded.flags = (static_cast<uint16_t>(payload[off]) << 8) | payload[off + 1];
  off += 2;
  decoded.timestamp_ns = 0;
  for (int k = 0; k < 8; ++k) {
    decoded.timestamp_ns = (decoded.timestamp_ns << 8) | payload[off++];
  }
  const uint32_t dim = read_u32_be(payload.data() + off);
  off += 4;
  if ((decoded.version != 1 && decoded.version != 2) ||
      decoded.descriptor_type != 1) {
    return false;
  }
  if (static_cast<size_t>(dim) > (payload.size() - off) / sizeof(float)) {
    return false;
  }
  decoded.descriptor.reserve(dim);
  for (uint32_t i = 0; i < dim; ++i) {
    decoded.descriptor.push_back(read_f32_be(payload.data() + off));
    off += 4;
  }

  if (decoded.version == 1) {
    out = std::move(decoded);
    return true;
  }

  constexpr size_t kV2ExtensionHeaderBytes = 4 + 4 + 4 + 2 + 1 + 1;
  if (payload.size() - off < kV2ExtensionHeaderBytes) return false;
  decoded.image_width = read_u32_be(payload.data() + off); off += 4;
  decoded.image_height = read_u32_be(payload.data() + off); off += 4;
  const uint32_t feature_count = read_u32_be(payload.data() + off); off += 4;
  decoded.feature_descriptor_dim = read_u16_be(payload.data() + off); off += 2;
  decoded.feature_descriptor_type = payload[off++];
  ++off; // reserved
  if (decoded.feature_descriptor_type != 1) return false;

  const size_t descriptor_bytes =
      static_cast<size_t>(decoded.feature_descriptor_dim) * sizeof(float);
  const size_t feature_stride = 3 * sizeof(float) + descriptor_bytes;
  if (static_cast<size_t>(feature_count) >
      (payload.size() - off) / feature_stride) {
    return false;
  }
  decoded.features.reserve(feature_count);
  for (uint32_t i = 0; i < feature_count; ++i) {
    KeyframeFeature feature;
    feature.x = read_f32_be(payload.data() + off); off += 4;
    feature.y = read_f32_be(payload.data() + off); off += 4;
    feature.score = read_f32_be(payload.data() + off); off += 4;
    feature.descriptor.reserve(decoded.feature_descriptor_dim);
    for (uint16_t j = 0; j < decoded.feature_descriptor_dim; ++j) {
      feature.descriptor.push_back(read_f32_be(payload.data() + off));
      off += 4;
    }
    decoded.features.push_back(std::move(feature));
  }
  out = std::move(decoded);
  return true;
}

inline bool decode_vio_state_payload(const std::vector<uint8_t>& payload, VioState& out) {
  const size_t need = 1 + 1 + 2 + 8 + 4 + 4 + 4 + 4 + 4 + 4;
  if (payload.size() < need) return false;
  size_t off = 0;
  out.version = payload[off++];
  out.state = payload[off++];
  out.flags = static_cast<uint16_t>((payload[off] << 8) | payload[off + 1]); off += 2;
  out.timestamp_ns = 0;
  for (int i = 0; i < 8; ++i) out.timestamp_ns = (out.timestamp_ns << 8) | payload[off + i];
  off += 8;
  auto rd_f32 = [&](float& v) {
    uint32_t tmp = (static_cast<uint32_t>(payload[off]) << 24) |
                   (static_cast<uint32_t>(payload[off + 1]) << 16) |
                   (static_cast<uint32_t>(payload[off + 2]) << 8) |
                   (static_cast<uint32_t>(payload[off + 3]));
    std::memcpy(&v, &tmp, 4);
    off += 4;
  };
  rd_f32(out.fps_current);
  rd_f32(out.fps_average);
  rd_f32(out.pose_confidence01);
  rd_f32(out.tracking_rate);
  out.num_features = (static_cast<uint32_t>(payload[off]) << 24) |
                     (static_cast<uint32_t>(payload[off + 1]) << 16) |
                     (static_cast<uint32_t>(payload[off + 2]) << 8) |
                     (static_cast<uint32_t>(payload[off + 3]));
  off += 4;
  out.loop_closures = (static_cast<uint32_t>(payload[off]) << 24) |
                      (static_cast<uint32_t>(payload[off + 1]) << 16) |
                      (static_cast<uint32_t>(payload[off + 2]) << 8) |
                      (static_cast<uint32_t>(payload[off + 3]));
  off += 4;

  out.build_version.clear();
  out.imu_hz_current = 0.0f;
  out.imu_hz_average_5s = 0.0f;
  out.init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  out.static_init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  out.dynamic_init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  out.memory_total_bytes = 0;
  out.memory_used_bytes = 0;
  out.memory_free_bytes = 0;
  out.light_level01 = 1.0f;
  out.light_required01 = 1.0f;
  if (out.version >= 2) {
    if (off < payload.size()) {
      const uint8_t ll = payload[off++];
      if (ll > 0) {
        if (payload.size() < off + ll) return false;
        out.build_version.assign(reinterpret_cast<const char*>(payload.data() + off),
                                 reinterpret_cast<const char*>(payload.data() + off + ll));
        off += ll;
      }
    }
  }
  if (out.version >= 3 && payload.size() >= off + 8) {
    rd_f32(out.imu_hz_current);
    rd_f32(out.imu_hz_average_5s);
  }
  if (out.version >= 4 && payload.size() > off) {
    out.init_reason_code = payload[off++];
  }
  if (out.version >= 5 && payload.size() >= off + 2) {
    out.static_init_reason_code = payload[off++];
    out.dynamic_init_reason_code = payload[off++];
  }
  if (out.version >= 6 && payload.size() >= off + 24) {
    out.memory_total_bytes = 0;
    out.memory_used_bytes = 0;
    out.memory_free_bytes = 0;
    for (int i = 0; i < 8; ++i) out.memory_total_bytes = (out.memory_total_bytes << 8) | payload[off + i];
    off += 8;
    for (int i = 0; i < 8; ++i) out.memory_used_bytes = (out.memory_used_bytes << 8) | payload[off + i];
    off += 8;
    for (int i = 0; i < 8; ++i) out.memory_free_bytes = (out.memory_free_bytes << 8) | payload[off + i];
    off += 8;
  }
  if (out.version >= 7 && payload.size() >= off + 8) {
    rd_f32(out.light_level01);
    rd_f32(out.light_required01);
  }
  if (out.version >= 8 && payload.size() >= off + 12) {
    rd_f32(out.translation_confidence01);
    rd_f32(out.translation_observability01);
    out.degraded_reason_flags = 0;
    for (int i = 0; i < 4; ++i) out.degraded_reason_flags = (out.degraded_reason_flags << 8) | payload[off + i];
    off += 4;
  }
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

inline bool decode_reset_vio_pose_payload(const std::vector<uint8_t>& payload,
                                          ResetVioPoseRequest& out) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  float confidence = 0.0f;
  std::optional<std::array<double, 4>> orientation;
  if (!decode_pose_payload(payload, out.pose_type, out.pose_flags,
                           x, y, z, orientation, &confidence)) {
    return false;
  }
  if (out.pose_type != 0) {
    return false;
  }
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  out.position_m = {x, y, z};
  if (orientation.has_value()) {
    for (double v : *orientation) {
      if (!std::isfinite(v)) {
        return false;
      }
    }
  }
  out.orientation_xyzw = orientation;
  return true;
}

inline bool decode_tracker_rect_payload(const std::vector<uint8_t>& payload,
                                        TrackerRectRequest& out) {
  if (payload.size() != 8) return false;
  out.x = read_u16_be(payload.data());
  out.y = read_u16_be(payload.data() + 2);
  out.width = read_u16_be(payload.data() + 4);
  out.height = read_u16_be(payload.data() + 6);
  return out.width > 0 && out.height > 0;
}

inline bool decode_command_payload(const std::vector<uint8_t>& payload,
                                   CommandRequest& out) {
  const size_t min_size = 4 + 1 + 4;
  if (payload.size() < min_size) return false;
  size_t off = 0;
  out.req_id = (static_cast<uint32_t>(payload[off]) << 24) |
               (static_cast<uint32_t>(payload[off + 1]) << 16) |
               (static_cast<uint32_t>(payload[off + 2]) << 8) |
               (static_cast<uint32_t>(payload[off + 3]));
  off += 4;
  const uint8_t name_len = payload[off++];
  if (payload.size() < off + name_len + 4) return false;
  out.name.assign(reinterpret_cast<const char*>(payload.data() + off), name_len);
  off += name_len;
  const uint32_t data_len = (static_cast<uint32_t>(payload[off]) << 24) |
                            (static_cast<uint32_t>(payload[off + 1]) << 16) |
                            (static_cast<uint32_t>(payload[off + 2]) << 8) |
                            (static_cast<uint32_t>(payload[off + 3]));
  off += 4;
  if (payload.size() < off + data_len) return false;
  out.data.assign(payload.begin() + off, payload.begin() + off + data_len);
  return true;
}

inline bool decode_config_request_payload(const std::vector<uint8_t>& payload,
                                          ConfigRequest& out) {
  const size_t min_size = 1 + 1 + 1 + 4;
  if (payload.size() < min_size) return false;
  size_t off = 0;
  out.version = payload[off++];
  out.op = payload[off++];
  const uint8_t key_len = payload[off++];
  if (payload.size() < off + key_len + 4) return false;
  out.key.assign(reinterpret_cast<const char*>(payload.data() + off), key_len);
  off += key_len;
  const uint32_t value_len = (static_cast<uint32_t>(payload[off]) << 24) |
                             (static_cast<uint32_t>(payload[off + 1]) << 16) |
                             (static_cast<uint32_t>(payload[off + 2]) << 8) |
                             (static_cast<uint32_t>(payload[off + 3]));
  off += 4;
  if (payload.size() < off + value_len) return false;
  out.value.assign(payload.begin() + off, payload.begin() + off + value_len);
  return true;
}

inline bool decode_command_response_payload(const std::vector<uint8_t>& payload,
                                            CommandResponse& out) {
  const size_t min_size = 4 + 1 + 2 + 4;
  if (payload.size() < min_size) return false;
  size_t off = 0;
  out.req_id = (static_cast<uint32_t>(payload[off]) << 24) |
               (static_cast<uint32_t>(payload[off + 1]) << 16) |
               (static_cast<uint32_t>(payload[off + 2]) << 8) |
               (static_cast<uint32_t>(payload[off + 3]));
  off += 4;
  out.status = payload[off++];
  const uint16_t msg_len = static_cast<uint16_t>((payload[off] << 8) | payload[off + 1]);
  off += 2;
  if (payload.size() < off + msg_len + 4) return false;
  out.message.assign(reinterpret_cast<const char*>(payload.data() + off), msg_len);
  off += msg_len;
  const uint32_t data_len = (static_cast<uint32_t>(payload[off]) << 24) |
                            (static_cast<uint32_t>(payload[off + 1]) << 16) |
                            (static_cast<uint32_t>(payload[off + 2]) << 8) |
                            (static_cast<uint32_t>(payload[off + 3]));
  off += 4;
  if (payload.size() < off + data_len) return false;
  out.data.assign(payload.begin() + off, payload.begin() + off + data_len);
  return true;
}

inline bool decode_config_response_payload(const std::vector<uint8_t>& payload,
                                           ConfigResponse& out) {
  const size_t min_size = 1 + 1 + 1 + 1 + 1 + 2 + 4;
  if (payload.size() < min_size) return false;
  size_t off = 0;
  out.version = payload[off++];
  out.op = payload[off++];
  out.success = payload[off++];
  out.has_value = payload[off++] != 0;
  const uint8_t key_len = payload[off++];
  if (payload.size() < off + key_len + 2 + 4) return false;
  out.key.assign(reinterpret_cast<const char*>(payload.data() + off), key_len);
  off += key_len;
  const uint16_t msg_len = static_cast<uint16_t>((payload[off] << 8) | payload[off + 1]);
  off += 2;
  if (payload.size() < off + msg_len + 4) return false;
  out.message.assign(reinterpret_cast<const char*>(payload.data() + off), msg_len);
  off += msg_len;
  const uint32_t value_len = (static_cast<uint32_t>(payload[off]) << 24) |
                             (static_cast<uint32_t>(payload[off + 1]) << 16) |
                             (static_cast<uint32_t>(payload[off + 2]) << 8) |
                             (static_cast<uint32_t>(payload[off + 3]));
  off += 4;
  if (payload.size() < off + value_len) return false;
  out.value.assign(payload.begin() + off, payload.begin() + off + value_len);
  return true;
}

} // namespace mighty_protocol
