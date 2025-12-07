#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

#include "mighty_protocol.h"

namespace mighty_protocol {

// Stream-oriented consumer that buffers incoming bytes and emits validated frames.
class FrameConsumer {
 public:
  // Feed raw bytes; parsed frames are returned via try_pop().
  void feed(const uint8_t* data, size_t len) {
    buffer_.insert(buffer_.end(), data, data + len);
  }

  // Attempt to parse the next frame from the internal buffer.
  // Returns std::nullopt if there is not yet a complete frame.
  std::optional<Frame> try_pop() {
    while (true) {
      Frame f; size_t consumed = 0;
      bool ok = parse_frame(buffer_, f, consumed);
      if (!ok) {
        if (consumed > 0) {
          // Drop malformed data and continue.
          buffer_.erase(buffer_.begin(), buffer_.begin() + consumed);
          continue;
        }
        return std::nullopt; // need more data
      }
      buffer_.erase(buffer_.begin(), buffer_.begin() + consumed);
      return f;
    }
  }

  // Parse as many frames as possible; returns vector of parsed frames.
  std::vector<Frame> drain() {
    std::vector<Frame> out;
    while (true) {
      auto maybe = try_pop();
      if (!maybe.has_value()) break;
      out.push_back(std::move(*maybe));
    }
    return out;
  }

  size_t buffered_bytes() const { return buffer_.size(); }

 private:
  std::vector<uint8_t> buffer_;
};

// Convenience dispatcher: feed raw bytes, invoke a handler per complete frame.
class FrameDispatcher {
 public:
  using Handler = std::function<void(const Frame&)>;

  explicit FrameDispatcher(Handler h = nullptr) : handler_(std::move(h)) {}

  void set_handler(Handler h) { handler_ = std::move(h); }

  void feed(const uint8_t* data, size_t len) {
    consumer_.feed(data, len);
    auto frames = consumer_.drain();
    for (auto &f : frames) {
      if (handler_) handler_(f);
    }
  }

  size_t buffered_bytes() const { return consumer_.buffered_bytes(); }

 private:
  FrameConsumer consumer_;
  Handler handler_;
};

// --------------------------------------------------------------------------
// Decoded dispatcher: register per-type handlers receiving decoded structs.
// --------------------------------------------------------------------------
class DecodedDispatcher {
 public:
  struct Pose {
    uint32_t pose_type;
    uint32_t pose_flags;
    double x, y, z;
    std::optional<std::array<double,4>> quat;
  };
  struct Constraints {
    std::vector<PoseConstraintSegment> segments;
  };
  struct PointCloud {
    std::vector<Point3DColor> points;
    std::optional<float> point_size;
  };
  using JpgHandler = std::function<void(uint64_t timestamp_ns, const std::string& channel, const std::vector<uint8_t>& data, bool is_ref)>;
  using PoseHandler = std::function<void(const Pose&, bool is_unoptimized)>;
  using ConstraintsHandler = std::function<void(const Constraints&)>;
  using FeaturesHandler = std::function<void(const std::vector<Feature3D>&)>;
  using PointCloudHandler = std::function<void(const PointCloud&)>;
  using VizHandler = std::function<void(const std::vector<uint8_t>&)>;
  using ImuHandler = std::function<void(const std::vector<ImuSample>&)>;
  using StatusHandler = std::function<void(const std::string&)>;
  using ResetHandler = std::function<void()>;

  void on_jpg(JpgHandler h) { jpg_handler_ = std::move(h); }
  void on_pose(PoseHandler h) { pose_handler_ = std::move(h); }
  void on_constraints(ConstraintsHandler h) { constraints_handler_ = std::move(h); }
  void on_features(FeaturesHandler h) { fea_handler_ = std::move(h); }
  void on_pointcloud(PointCloudHandler h) { pcl_handler_ = std::move(h); }
  void on_viz(VizHandler h) { viz_handler_ = std::move(h); }
  void on_imu(ImuHandler h) { imu_handler_ = std::move(h); }
  void on_status(StatusHandler h) { status_handler_ = std::move(h); }
  void on_reset(ResetHandler h) { reset_handler_ = std::move(h); }

  void feed(const uint8_t* data, size_t len) {
    consumer_.feed(data, len);
    auto frames = consumer_.drain();
    for (auto &f : frames) {
      dispatch(f);
    }
  }

 private:
  void dispatch(const Frame& f) {
    const std::string type(f.type.begin(), f.type.end());
    if (type == "JPG ") {
      if (!jpg_handler_) return;
      uint64_t ts; std::string ch; std::vector<uint8_t> data;
      if (decode_jpg_payload(f.payload, false, ts, ch, data)) {
        jpg_handler_(ts, ch, data, false);
      }
    } else if (type == "RJPG") {
      if (!jpg_handler_) return;
      uint64_t ts; std::string ch; std::vector<uint8_t> data;
      if (decode_jpg_payload(f.payload, true, ts, ch, data)) {
        jpg_handler_(ts, ch, data, true);
      }
    } else if (type == "POSE" || type == "UPOS") {
      if (!pose_handler_) return;
      uint32_t pt, pf; double x,y,z; std::optional<std::array<double,4>> q;
      if (decode_pose_payload(f.payload, pt, pf, x, y, z, q)) {
        Pose p{pt, pf, x, y, z, q};
        pose_handler_(p, type == "UPOS");
      }
    } else if (type == "LCON") {
      if (!constraints_handler_) return;
      std::vector<PoseConstraintSegment> segs;
      if (decode_constraints_payload(f.payload, segs)) {
        constraints_handler_(Constraints{segs});
      }
    } else if (type == "FEA3") {
      if (!fea_handler_) return;
      std::vector<Feature3D> feats;
      if (decode_fea3_payload(f.payload, feats)) {
        fea_handler_(feats);
      }
    } else if (type == "PCLD") {
      if (!pcl_handler_) return;
      std::vector<Point3DColor> pts; std::optional<float> ps;
      if (decode_pcld_payload(f.payload, pts, ps)) {
        pcl_handler_(PointCloud{pts, ps});
      }
    } else if (type == "VIZ ") {
      if (viz_handler_) viz_handler_(f.payload);
    } else if (type == "IMU ") {
      if (!imu_handler_) return;
      std::vector<ImuSample> batch;
      if (decode_imu_payload(f.payload, batch)) {
        imu_handler_(batch);
      }
    } else if (type == "STAT") {
      if (!status_handler_) return;
      std::string text;
      if (decode_status_payload(f.payload, text)) status_handler_(text);
    } else if (type == "RSET") {
      if (reset_handler_) reset_handler_();
    }
  }

  FrameConsumer consumer_;
  JpgHandler jpg_handler_;
  PoseHandler pose_handler_;
  ConstraintsHandler constraints_handler_;
  FeaturesHandler fea_handler_;
  PointCloudHandler pcl_handler_;
  VizHandler viz_handler_;
  ImuHandler imu_handler_;
  StatusHandler status_handler_;
  ResetHandler reset_handler_;
};

} // namespace mighty_protocol
