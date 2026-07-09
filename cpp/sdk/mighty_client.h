#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mighty_device_io.h"
#include "../mighty_protocol.h"
#include "../mighty_protocol_consumer.h"

#if defined(MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE)
#include "mighty_loopclosure/mighty_loopclosure_device_c.h"
#endif

namespace mighty_protocol {
namespace sdk {

struct MightyClientOptions {
  int command_timeout_ms = 2000;  // Reserved for transports that support cancelable command requests.
  bool auto_reconnect = true;
  int reconnect_delay_ms = 300;
  bool emit_stat_as_status = true;
  bool normalize_channel_aliases = true;
  bool loopclosure = false;
  std::string loopclosure_calibration_yaml;
};

struct MightyClientStats {
  uint64_t rx_frames = 0;
  uint64_t rx_bytes = 0;
  uint64_t decode_errors = 0;
  uint64_t reconnects = 0;
  uint64_t command_timeouts = 0;
};

struct RawImageFrame {
  uint64_t timestamp_ns = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint8_t format = 0;
  std::string channel;
  std::string channel_alias;
  std::vector<uint8_t> data;
};

struct ImageFrame {
  enum class Kind {
    kRaw,
    kStereoRaw,
  };

  Kind kind = Kind::kRaw;
  RawImageFrame left;
  std::optional<RawImageFrame> right;
};

struct PoseFrame {
  bool is_public = true;  // true for POSE, false for UPOS
  std::string packet_type; // POSE | UPOS
  std::string pose_type; // body | camera | other
  uint32_t pose_type_raw = 0;
  uint32_t pose_flags = 0;
  std::string frame_id = "odom";
  std::string child_frame_id = "base_link";
  std::array<double, 3> position_m{0.0, 0.0, 0.0};
  std::optional<std::array<double, 3>> raw_position_m;
  std::optional<std::array<double, 4>> orientation_xyzw;
  float confidence = 0.0f;
  bool is_keyframe = false;
  bool loopclosure_corrected = false;
  std::optional<std::array<double, 3>> linear_velocity_body_mps;
  std::optional<std::array<double, 3>> angular_velocity_body_rps;
  std::optional<std::array<double, 3>> linear_acceleration_body_mps2;
  std::optional<std::array<double, 3>> angular_acceleration_body_rps2;
  std::optional<uint64_t> timestamp_ns;
};

struct ImuBatch {
  std::vector<ImuSample> samples;
};

struct VioStateFrame {
  uint8_t version = 0;
  uint8_t state = 0;
  uint16_t flags = 0;
  uint64_t timestamp_ns = 0;
  float fps_current = 0.0f;
  float fps_average = 0.0f;
  float pose_confidence = 0.0f;
  float tracking_rate = 0.0f;
  uint32_t num_features = 0;
  uint32_t loop_closures = 0;
  std::string build_version;
  std::optional<float> imu_hz_current;
  std::optional<float> imu_hz_average_5s;
  uint8_t init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  uint8_t static_init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  uint8_t dynamic_init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  std::optional<uint64_t> memory_total_bytes;
  std::optional<uint64_t> memory_used_bytes;
  std::optional<uint64_t> memory_free_bytes;
  std::optional<float> light_level01;
  std::optional<float> light_required01;
  std::optional<float> translation_confidence01;
  std::optional<float> translation_observability01;
  std::optional<uint32_t> degraded_reason_flags;
  VioInitReasonCode init_reason = VioInitReasonCode::kNone;
};

struct VizFrame {
  std::string subtype;  // features | detections | matches | unknown
  uint8_t raw_subtype = 255;
  std::vector<VizFeature> features;
  std::vector<VizDetection> detections;
  std::vector<VizMatch> matches;
  std::vector<uint8_t> raw;
};

struct LconFrame {
  std::vector<PoseConstraintSegment> segments;
};

struct KeyframeEvent {
  uint8_t version = 1;
  uint8_t descriptor_type = 1;
  uint16_t flags = 0;
  uint64_t timestamp_ns = 0;
  std::vector<float> descriptor;
};

struct LoopClosureEvent {
  std::string type;
  uint64_t timestamp_ns = 0;
  std::size_t current_keyframe = 0;
  std::size_t matched_keyframe = 0;
  std::array<double, 3> pose_translation_correction_m{0.0, 0.0, 0.0};
};

struct StatusEvent {
  std::string text;
};

struct ResetEvent {
  uint64_t received_at_ms = 0;
};

struct AnyEvent {
  std::string type;      // image|pose|imu|vio_state|viz|lcon|status|reset|unknown
  std::string raw_type;  // only for unknown
  std::vector<uint8_t> payload; // only for unknown
};

struct MightyErrorEvent {
  std::string scope;   // transport|protocol|command|config
  std::string code;
  std::string message;
};

struct CommandResult {
  bool ok = false;
  uint32_t req_id = 0;
  uint8_t status = 1;
  std::string message;
  std::vector<uint8_t> data;
};

struct ConfigGetResult {
  bool ok = false;
  bool found = false;
  std::string key;
  std::vector<uint8_t> value;
  std::string message;
};

struct ConfigGetTextResult {
  bool ok = false;
  bool found = false;
  std::string key;
  std::string value;
  std::string message;
};

struct ConfigSetResult {
  bool ok = false;
  std::string key;
  std::vector<uint8_t> value;
  std::string message;
};

class MightyClient {
 public:
  using ImageHandler = std::function<void(const ImageFrame&)>;
  using PoseHandler = std::function<void(const PoseFrame&)>;
  using ImuHandler = std::function<void(const ImuBatch&)>;
  using VioStateHandler = std::function<void(const VioStateFrame&)>;
  using VizHandler = std::function<void(const VizFrame&)>;
  using LconHandler = std::function<void(const LconFrame&)>;
  using KeyframeHandler = std::function<void(const KeyframeEvent&)>;
  using LoopClosureHandler = std::function<void(const LoopClosureEvent&)>;
  using StatusHandler = std::function<void(const StatusEvent&)>;
  using ResetHandler = std::function<void(const ResetEvent&)>;
  using AnyHandler = std::function<void(const AnyEvent&)>;
  using ErrorHandler = std::function<void(const MightyErrorEvent&)>;

  struct Subscription {
    enum class Kind {
      kInvalid,
      kImage,
      kPose,
      kImu,
      kVioState,
      kViz,
      kLcon,
      kKeyframe,
      kLoopClosure,
      kStatus,
      kReset,
      kAny,
      kError,
    };

    Kind kind = Kind::kInvalid;
    uint64_t id = 0;

    bool valid() const { return kind != Kind::kInvalid && id != 0; }
  };

  explicit MightyClient(std::shared_ptr<MightyDeviceIO> device,
                        const MightyClientOptions& opts = {})
      : device_(std::move(device)), opts_(opts) {
    frame_dispatcher_.set_handler([this](const Frame& frame) {
      this->handle_frame(frame);
    });
  }

  ~MightyClient() {
    disconnect();
#if defined(MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE)
    if (loopclosure_) {
      mlc_destroy(loopclosure_);
      loopclosure_ = nullptr;
    }
#endif
  }

  void connect() {
    if (!device_) {
      emit_error("transport", "missing_device", "MightyClient requires a non-null device");
      return;
    }

    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;

    transport_thread_ = std::thread([this]() {
      this->transport_loop();
    });

    if (opts_.loopclosure) {
      initialize_loopclosure();
      set_keyframes_enabled(true);
    }
  }

  void disconnect() {
    const bool was_running = running_.exchange(false);
    if (was_running && opts_.loopclosure) {
      set_keyframes_enabled(false);
    }
    if (device_) {
      device_->disconnect();
    }
    if (transport_thread_.joinable()) {
      transport_thread_.join();
    }
    if (was_running) {
      stream_active_.store(false);
    }
  }

  bool is_connected() const {
    return stream_active_.load();
  }

  MightyClientStats stats() const {
    std::lock_guard<std::mutex> lock(stats_mu_);
    return stats_;
  }

  Subscription on_image(ImageHandler cb) { return subscribe(image_handlers_, Subscription::Kind::kImage, std::move(cb)); }
  Subscription on_pose(PoseHandler cb) { return subscribe(pose_handlers_, Subscription::Kind::kPose, std::move(cb)); }
  Subscription on_imu(ImuHandler cb) { return subscribe(imu_handlers_, Subscription::Kind::kImu, std::move(cb)); }
  Subscription on_vio_state(VioStateHandler cb) { return subscribe(vio_state_handlers_, Subscription::Kind::kVioState, std::move(cb)); }
  Subscription on_viz(VizHandler cb) { return subscribe(viz_handlers_, Subscription::Kind::kViz, std::move(cb)); }
  Subscription on_lcon(LconHandler cb) { return subscribe(lcon_handlers_, Subscription::Kind::kLcon, std::move(cb)); }
  Subscription on_constraints(LconHandler cb) { return on_lcon(std::move(cb)); }
  Subscription on_keyframe(KeyframeHandler cb) { return subscribe(keyframe_handlers_, Subscription::Kind::kKeyframe, std::move(cb)); }
  Subscription on_loopclosure(LoopClosureHandler cb) { return subscribe(loopclosure_handlers_, Subscription::Kind::kLoopClosure, std::move(cb)); }
  Subscription on_status(StatusHandler cb) { return subscribe(status_handlers_, Subscription::Kind::kStatus, std::move(cb)); }
  Subscription on_reset(ResetHandler cb) { return subscribe(reset_handlers_, Subscription::Kind::kReset, std::move(cb)); }
  Subscription on_any(AnyHandler cb) { return subscribe(any_handlers_, Subscription::Kind::kAny, std::move(cb)); }
  Subscription on_error(ErrorHandler cb) { return subscribe(error_handlers_, Subscription::Kind::kError, std::move(cb)); }

  void unsubscribe(const Subscription& sub) {
    if (!sub.valid()) return;
    switch (sub.kind) {
      case Subscription::Kind::kImage: image_handlers_.remove(sub.id); break;
      case Subscription::Kind::kPose: pose_handlers_.remove(sub.id); break;
      case Subscription::Kind::kImu: imu_handlers_.remove(sub.id); break;
      case Subscription::Kind::kVioState: vio_state_handlers_.remove(sub.id); break;
      case Subscription::Kind::kViz: viz_handlers_.remove(sub.id); break;
      case Subscription::Kind::kLcon: lcon_handlers_.remove(sub.id); break;
      case Subscription::Kind::kKeyframe: keyframe_handlers_.remove(sub.id); break;
      case Subscription::Kind::kLoopClosure: loopclosure_handlers_.remove(sub.id); break;
      case Subscription::Kind::kStatus: status_handlers_.remove(sub.id); break;
      case Subscription::Kind::kReset: reset_handlers_.remove(sub.id); break;
      case Subscription::Kind::kAny: any_handlers_.remove(sub.id); break;
      case Subscription::Kind::kError: error_handlers_.remove(sub.id); break;
      case Subscription::Kind::kInvalid: break;
    }
  }

  CommandResult command(const std::string& name,
                        const std::vector<uint8_t>& data = std::vector<uint8_t>()) {
    CommandResult out;
    out.req_id = alloc_req_id();

    if (name.empty()) {
      out.message = "command name must be a non-empty string";
      return out;
    }

    if (!device_) {
      out.message = "missing device";
      return out;
    }

    std::vector<uint8_t> cmd_payload = build_command_payload(out.req_id, name, data);
    std::vector<uint8_t> response_payload;
    std::string transport_error;

    const bool ok = device_->send_command_payload(cmd_payload, &response_payload, &transport_error);
    if (!ok) {
      out.message = transport_error.empty() ? "command failed" : transport_error;
      emit_error("command", "command_failed", out.message);
      return out;
    }

    CommandResponse decoded;
    if (!decode_command_response_payload(response_payload, decoded)) {
      out.message = "invalid command response";
      emit_error("command", "decode_failed", out.message);
      return out;
    }

    out.ok = decoded.status == 0;
    out.req_id = decoded.req_id;
    out.status = decoded.status;
    out.message = decoded.message;
    out.data = std::move(decoded.data);
    return out;
  }

  ConfigGetResult config_get(const std::string& key) {
    ConfigGetResult out;
    out.key = key;

    ConfigRequest req;
    req.version = 1;
    req.op = static_cast<uint8_t>(ConfigOp::kGet);
    req.key = key;
    req.value.clear();

    const auto cfgq = build_config_request_payload(req);
    CommandResult cmd = command("config", cfgq);
    if (!cmd.ok) {
      out.message = cmd.message;
      return out;
    }

    ConfigResponse cfgr;
    if (!decode_config_response_payload(cmd.data, cfgr)) {
      out.message = "invalid config response";
      emit_error("config", "decode_failed", out.message);
      return out;
    }

    out.ok = cfgr.success != 0;
    out.found = cfgr.has_value;
    out.key = cfgr.key.empty() ? key : cfgr.key;
    out.value = std::move(cfgr.value);
    out.message = cfgr.message;
    return out;
  }

  ConfigGetTextResult config_get_text(const std::string& key) {
    ConfigGetResult bytes = config_get(key);
    ConfigGetTextResult out;
    out.ok = bytes.ok;
    out.found = bytes.found;
    out.key = bytes.key;
    out.value.assign(bytes.value.begin(), bytes.value.end());
    out.message = bytes.message;
    return out;
  }

  ConfigSetResult config_set(const std::string& key, const std::vector<uint8_t>& value) {
    ConfigSetResult out;
    out.key = key;
    out.value = value;

    ConfigRequest req;
    req.version = 1;
    req.op = static_cast<uint8_t>(ConfigOp::kSet);
    req.key = key;
    req.value = value;

    const auto cfgq = build_config_request_payload(req);
    CommandResult cmd = command("config", cfgq);
    if (!cmd.ok) {
      out.message = cmd.message;
      return out;
    }

    ConfigResponse cfgr;
    if (!decode_config_response_payload(cmd.data, cfgr)) {
      out.message = "invalid config response";
      emit_error("config", "decode_failed", out.message);
      return out;
    }

    out.ok = cfgr.success != 0;
    out.key = cfgr.key.empty() ? key : cfgr.key;
    out.value = std::move(cfgr.value);
    out.message = cfgr.message;
    return out;
  }

  ConfigSetResult config_set_text(const std::string& key, const std::string& value) {
    return config_set(key, std::vector<uint8_t>(value.begin(), value.end()));
  }

  CommandResult start_vio() {
    return command("start_vio");
  }

  CommandResult stop_vio() {
    return command("stop_vio");
  }

  CommandResult reset_vio_pose(
      const std::array<double, 3>& position_m,
      const std::optional<std::array<double, 4>>& orientation_xyzw = std::nullopt) {
    return command("reset_vio_pose",
                   build_reset_vio_pose_payload(position_m, orientation_xyzw));
  }

  CommandResult set_keyframes_enabled(bool enabled) {
    const std::string action = enabled ? "on" : "off";
    return command("keyframes", std::vector<uint8_t>(action.begin(), action.end()));
  }

  CommandResult keyframes_status() {
    const std::string action = "status";
    return command("keyframes", std::vector<uint8_t>(action.begin(), action.end()));
  }

  bool set_loopclosure_calibration_yaml(const std::string& yaml) {
    opts_.loopclosure_calibration_yaml = yaml;
#if defined(MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE)
    if (!opts_.loopclosure) {
      emit_error("loopclosure", "disabled", "loopclosure option is disabled");
      return false;
    }
    if (yaml.empty()) {
      emit_error("loopclosure", "calibration_failed", "empty calibration yaml");
      return false;
    }
    initialize_loopclosure();
    if (!loopclosure_) return false;
    const mlc_status_t status = mlc_set_calibration_yaml(loopclosure_, yaml.c_str());
    if (status != MLC_STATUS_OK) {
      emit_error("loopclosure", "calibration_failed", mlc_status_message(status));
      return false;
    }
    return true;
#else
    (void)yaml;
    emit_error("loopclosure",
               "not_compiled",
               "MightyClient loopclosure option requires MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE and the native loop-closure library");
    return false;
#endif
  }

 private:
  template <typename Fn>
  class ListenerSet {
   public:
    uint64_t add(Fn cb) {
      if (!cb) return 0;
      std::lock_guard<std::mutex> lock(mu_);
      const uint64_t id = next_id_++;
      listeners_[id] = std::move(cb);
      return id;
    }

    void remove(uint64_t id) {
      std::lock_guard<std::mutex> lock(mu_);
      listeners_.erase(id);
    }

    bool empty() const {
      std::lock_guard<std::mutex> lock(mu_);
      return listeners_.empty();
    }

    std::vector<Fn> snapshot() const {
      std::lock_guard<std::mutex> lock(mu_);
      std::vector<Fn> out;
      out.reserve(listeners_.size());
      for (const auto& kv : listeners_) {
        out.push_back(kv.second);
      }
      return out;
    }

   private:
    mutable std::mutex mu_;
    std::unordered_map<uint64_t, Fn> listeners_;
    uint64_t next_id_ = 1;
  };

  template <typename Fn>
  Subscription subscribe(ListenerSet<Fn>& set, Subscription::Kind kind, Fn cb) {
    const uint64_t id = set.add(std::move(cb));
    Subscription out;
    out.kind = id == 0 ? Subscription::Kind::kInvalid : kind;
    out.id = id;
    return out;
  }

  template <typename Fn, typename Event>
  void emit(const ListenerSet<Fn>& set, const Event& evt, bool forward_listener_errors = true) {
    const auto listeners = set.snapshot();
    for (const auto& cb : listeners) {
      try {
        cb(evt);
      } catch (...) {
        if (forward_listener_errors) {
          emit_error("protocol", "listener_threw", "listener callback threw");
        }
      }
    }
  }

  void emit_any(const AnyEvent& evt) {
    emit(any_handlers_, evt);
  }

  void emit_error(const std::string& scope, const std::string& code, const std::string& message) {
    MightyErrorEvent evt;
    evt.scope = scope;
    evt.code = code;
    evt.message = message;
    emit(error_handlers_, evt, /*forward_listener_errors=*/false);
  }

  uint32_t alloc_req_id() {
    uint32_t out = req_id_.fetch_add(1);
    if (out == 0) {
      out = req_id_.fetch_add(1);
    }
    return out;
  }

  static float clamp01(float v) {
    if (!(v == v)) return 0.0f;
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
  }

  std::string map_channel_alias(const std::string& channel) const {
    if (!opts_.normalize_channel_aliases) return "";
    std::string c = channel;
    for (char& ch : c) {
      if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
    }
    if (c == "cam0" || c == "preview" || c == "left") return "cam0";
    if (c == "cam1" || c == "right") return "cam1";
    return "";
  }

  bool is_primary_channel(const RawImageFrame& raw) const {
    std::string channel = raw.channel_alias.empty() ? raw.channel : raw.channel_alias;
    for (char& ch : channel) {
      if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
    }
    return channel == "cam0" || channel == "preview" || channel == "left";
  }

  void initialize_loopclosure() {
#if defined(MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE)
    if (loopclosure_initialized_) return;
    mlc_options_t options{};
    mlc_options_default(&options);
    mlc_device_loopclosure_t* next = nullptr;
    mlc_status_t status = mlc_create(&options, &next);
    if (status != MLC_STATUS_OK || !next) {
      emit_error("loopclosure", "create_failed", mlc_status_message(status));
      return;
    }
    loopclosure_ = next;
    mlc_set_event_callback(loopclosure_, &MightyClient::loopclosure_event_trampoline, this);
    status = mlc_initialize(loopclosure_);
    if (status != MLC_STATUS_OK) {
      emit_error("loopclosure", "initialize_failed", mlc_status_message(status));
      return;
    }
    if (!opts_.loopclosure_calibration_yaml.empty()) {
      status = mlc_set_calibration_yaml(loopclosure_, opts_.loopclosure_calibration_yaml.c_str());
      if (status != MLC_STATUS_OK) {
        emit_error("loopclosure", "calibration_failed", mlc_status_message(status));
      }
    }
    loopclosure_initialized_ = true;
#else
    emit_error("loopclosure",
               "not_compiled",
               "MightyClient loopclosure option requires MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE and the native loop-closure library");
#endif
  }

#if defined(MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE)
  static void loopclosure_event_trampoline(const mlc_event_t* event, void* user) {
    if (!event || !user) return;
    static_cast<MightyClient*>(user)->handle_loopclosure_event(*event);
  }

  static std::string loopclosure_event_type_name(uint8_t type) {
    switch (static_cast<mlc_event_type_t>(type)) {
      case MLC_EVENT_LOOP_CLOSURE: return "loop_closure";
    }
    return "unknown";
  }

  void handle_loopclosure_event(const mlc_event_t& event) {
    LoopClosureEvent out;
    out.type = loopclosure_event_type_name(event.type);
    out.timestamp_ns = event.timestamp_ns;
    out.current_keyframe = event.current_keyframe;
    out.matched_keyframe = event.matched_keyframe;
    out.pose_translation_correction_m = {
        event.correction_tx,
        event.correction_ty,
        event.correction_tz,
    };
    {
      std::lock_guard<std::mutex> lock(loopclosure_mu_);
      loopclosure_translation_correction_ = out.pose_translation_correction_m;
      loopclosure_has_correction_ = true;
    }
    emit(loopclosure_handlers_, out);
  }

  void push_loopclosure_image(const RawImageFrame& raw) {
    if (!opts_.loopclosure || !loopclosure_initialized_ || !loopclosure_) return;
    if (!is_primary_channel(raw)) return;
    mlc_raw_image_t image{};
    image.timestamp_ns = raw.timestamp_ns;
    image.frame_id = loopclosure_next_frame_id_.fetch_add(1);
    image.width = raw.width;
    image.height = raw.height;
    image.format = raw.format;
    image.data = raw.data.data();
    image.size_bytes = raw.data.size();
    const mlc_status_t status = mlc_push_image(loopclosure_, &image);
    if (status != MLC_STATUS_OK) {
      emit_error("loopclosure", "push_image_failed", mlc_status_message(status));
    }
  }

  void push_loopclosure_pose(const PoseFrame& pose) {
    if (!opts_.loopclosure || !loopclosure_initialized_ || !loopclosure_) return;
    if (!pose.is_public || !pose.timestamp_ns || !pose.orientation_xyzw) return;
    if (!(pose.pose_type == "body" || pose.pose_type == "camera" ||
          pose.pose_type_raw == 0 || pose.pose_type_raw == 1)) {
      return;
    }
    const auto& q = *pose.orientation_xyzw;
    mlc_pose_t raw{};
    raw.timestamp_ns = *pose.timestamp_ns;
    raw.px = pose.raw_position_m ? (*pose.raw_position_m)[0] : pose.position_m[0];
    raw.py = pose.raw_position_m ? (*pose.raw_position_m)[1] : pose.position_m[1];
    raw.pz = pose.raw_position_m ? (*pose.raw_position_m)[2] : pose.position_m[2];
    raw.qw = q[3];
    raw.qx = q[0];
    raw.qy = q[1];
    raw.qz = q[2];
    raw.frame = (pose.pose_type == "camera" || pose.pose_type_raw == 1)
        ? MLC_POSE_FRAME_CAMERA
        : MLC_POSE_FRAME_BODY;
    raw.confidence = pose.confidence;
    const mlc_status_t status = mlc_push_pose(loopclosure_, &raw);
    if (status != MLC_STATUS_OK) {
      emit_error("loopclosure", "push_pose_failed", mlc_status_message(status));
    }
  }

  void push_loopclosure_keyframe(const KeyframeEvent& keyframe) {
    if (!opts_.loopclosure || !loopclosure_initialized_ || !loopclosure_) return;
    mlc_device_keyframe_t raw{};
    raw.timestamp_ns = keyframe.timestamp_ns;
    raw.frame_id = -1;
    raw.descriptor_type = keyframe.descriptor_type;
    raw.flags = keyframe.flags;
    raw.descriptor = keyframe.descriptor.data();
    raw.descriptor_count = keyframe.descriptor.size();
    const mlc_status_t status = mlc_push_keyframe(loopclosure_, &raw);
    if (status != MLC_STATUS_OK) {
      emit_error("loopclosure", "push_keyframe_failed", mlc_status_message(status));
    }
  }
#else
  void push_loopclosure_image(const RawImageFrame&) {}
  void push_loopclosure_pose(const PoseFrame&) {}
  void push_loopclosure_keyframe(const KeyframeEvent&) {}
#endif

  void apply_loopclosure_correction(PoseFrame* pose) {
    if (!pose || !opts_.loopclosure || !pose->is_public) return;
    if (!(pose->pose_type == "body" || pose->pose_type == "camera" ||
          pose->pose_type_raw == 0 || pose->pose_type_raw == 1)) {
      return;
    }
    std::lock_guard<std::mutex> lock(loopclosure_mu_);
    if (!loopclosure_has_correction_) return;
    pose->raw_position_m = pose->position_m;
    for (size_t i = 0; i < 3; ++i) {
      pose->position_m[i] += loopclosure_translation_correction_[i];
    }
    pose->loopclosure_corrected = true;
  }

  static std::string map_pose_type(uint32_t pose_type) {
    if (pose_type == 0) return "body";
    if (pose_type == 1) return "camera";
    return "other";
  }

  void add_rx_bytes(size_t len) {
    std::lock_guard<std::mutex> lock(stats_mu_);
    stats_.rx_bytes += static_cast<uint64_t>(len);
  }

  bool has_any_listener() const {
    return !any_handlers_.empty();
  }

  void handle_bytes(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    add_rx_bytes(len);
    frame_dispatcher_.feed(data, len);
  }

  void handle_frame(const Frame& frame) {
    {
      std::lock_guard<std::mutex> lock(stats_mu_);
      stats_.rx_frames += 1;
    }

    const std::string type(frame.type.begin(), frame.type.end());
    const bool wants_any = has_any_listener();

    try {
      if (type == "RAW ") {
        if (image_handlers_.empty() && !wants_any && !opts_.loopclosure) return;
        RawImageFrame raw;
        if (!decode_raw_payload(frame.payload, raw.timestamp_ns, raw.width, raw.height,
                                raw.format, raw.channel, raw.data)) {
          throw std::runtime_error("RAW decode failed");
        }
        raw.channel_alias = map_channel_alias(raw.channel);
        push_loopclosure_image(raw);
        ImageFrame evt;
        evt.kind = ImageFrame::Kind::kRaw;
        evt.left = std::move(raw);
        emit(image_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"image", "", {}});
        return;
      }

      if (type == "SRAW") {
        if (image_handlers_.empty() && !wants_any && !opts_.loopclosure) return;
        RawImageFrame left;
        RawImageFrame right;
        if (!decode_stereo_raw_payload(frame.payload,
                                       left.timestamp_ns,
                                       right.timestamp_ns,
                                       left.width,
                                       left.height,
                                       left.format,
                                       left.channel,
                                       left.data,
                                       right.width,
                                       right.height,
                                       right.format,
                                       right.channel,
                                       right.data)) {
          throw std::runtime_error("SRAW decode failed");
        }
        left.channel_alias = map_channel_alias(left.channel);
        right.channel_alias = map_channel_alias(right.channel);
        if (is_primary_channel(left)) {
          push_loopclosure_image(left);
        } else if (is_primary_channel(right)) {
          push_loopclosure_image(right);
        } else {
          push_loopclosure_image(left);
        }

        ImageFrame evt;
        evt.kind = ImageFrame::Kind::kStereoRaw;
        evt.left = std::move(left);
        evt.right = std::move(right);
        emit(image_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"image", "", {}});
        return;
      }

      if (type == "POSE" || type == "UPOS") {
        if (pose_handlers_.empty() && !wants_any && !opts_.loopclosure) return;
        uint32_t pose_type = 0;
        PoseFrame evt;
        evt.is_public = (type == "POSE");
        evt.packet_type = type;
        if (!decode_pose_payload(frame.payload,
                                 pose_type,
                                 evt.pose_flags,
                                 evt.position_m[0],
                                 evt.position_m[1],
                                 evt.position_m[2],
                                 evt.orientation_xyzw,
                                 &evt.confidence,
                                 &evt.linear_velocity_body_mps,
                                 &evt.angular_velocity_body_rps,
                                 &evt.linear_acceleration_body_mps2,
                                 &evt.angular_acceleration_body_rps2,
                                 &evt.timestamp_ns)) {
          throw std::runtime_error("POSE decode failed");
        }
        evt.pose_type_raw = pose_type;
        evt.pose_type = map_pose_type(pose_type);
        evt.confidence = clamp01(evt.confidence);
        evt.is_keyframe = (evt.pose_flags & (1u << 1)) != 0;
        push_loopclosure_pose(evt);
        apply_loopclosure_correction(&evt);
        emit(pose_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"pose", "", {}});
        return;
      }

      if (type == "IMU ") {
        if (imu_handlers_.empty() && !wants_any) return;
        ImuBatch evt;
        if (!decode_imu_payload(frame.payload, evt.samples)) {
          throw std::runtime_error("IMU decode failed");
        }
        emit(imu_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"imu", "", {}});
        return;
      }

      if (type == "VSTA") {
        if (vio_state_handlers_.empty() && !wants_any) return;
        VioState raw;
        if (!decode_vio_state_payload(frame.payload, raw)) {
          throw std::runtime_error("VSTA decode failed");
        }
        VioStateFrame evt;
        evt.version = raw.version;
        evt.state = raw.state;
        evt.flags = raw.flags;
        evt.timestamp_ns = raw.timestamp_ns;
        evt.fps_current = raw.fps_current;
        evt.fps_average = raw.fps_average;
        evt.pose_confidence = raw.pose_confidence01;
        evt.tracking_rate = raw.tracking_rate;
        evt.num_features = raw.num_features;
        evt.loop_closures = raw.loop_closures;
        evt.build_version = raw.build_version;
        if (raw.version >= 3) {
          evt.imu_hz_current = raw.imu_hz_current;
          evt.imu_hz_average_5s = raw.imu_hz_average_5s;
        }
        evt.init_reason_code = raw.init_reason_code;
        evt.static_init_reason_code = raw.static_init_reason_code;
        evt.dynamic_init_reason_code = raw.dynamic_init_reason_code;
        if (raw.version >= 6) {
          evt.memory_total_bytes = raw.memory_total_bytes;
          evt.memory_used_bytes = raw.memory_used_bytes;
          evt.memory_free_bytes = raw.memory_free_bytes;
        }
        if (raw.version >= 7) {
          if (std::isfinite(raw.light_level01)) {
            evt.light_level01 = raw.light_level01;
          }
          if (std::isfinite(raw.light_required01)) {
            evt.light_required01 = raw.light_required01;
          }
        }
        if (raw.version >= 8) {
          if (std::isfinite(raw.translation_confidence01)) {
            evt.translation_confidence01 = raw.translation_confidence01;
          }
          if (std::isfinite(raw.translation_observability01)) {
            evt.translation_observability01 = raw.translation_observability01;
          }
          evt.degraded_reason_flags = raw.degraded_reason_flags;
        }
        evt.init_reason = static_cast<VioInitReasonCode>(raw.init_reason_code);
        emit(vio_state_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"vio_state", "", {}});
        return;
      }

      if (type == "VIZ ") {
        if (viz_handlers_.empty() && !wants_any) return;
        VizFrame evt;
        VizPayload decoded;
        if (decode_viz_payload(frame.payload, decoded)) {
          evt.raw_subtype = decoded.subtype;
          if (decoded.subtype == 0) {
            evt.subtype = "features";
            evt.features = std::move(decoded.features);
          } else if (decoded.subtype == 1) {
            evt.subtype = "detections";
            evt.detections = std::move(decoded.detections);
          } else if (decoded.subtype == 2) {
            evt.subtype = "matches";
            evt.matches = std::move(decoded.matches);
          } else {
            evt.subtype = "unknown";
            evt.raw = frame.payload;
          }
        } else {
          evt.subtype = "unknown";
          evt.raw_subtype = frame.payload.empty() ? 255 : frame.payload[0];
          evt.raw = frame.payload;
        }
        emit(viz_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"viz", "", {}});
        return;
      }

      if (type == "LCON") {
        if (lcon_handlers_.empty() && !wants_any) return;
        LconFrame evt;
        if (!decode_constraints_payload(frame.payload, evt.segments)) {
          throw std::runtime_error("LCON decode failed");
        }
        emit(lcon_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"lcon", "", {}});
        return;
      }

      if (type == "KEYF") {
        if (keyframe_handlers_.empty() && !wants_any && !opts_.loopclosure) return;
        KeyframeDescriptor decoded;
        if (!decode_keyframe_payload(frame.payload, decoded)) {
          throw std::runtime_error("KEYF decode failed");
        }
        KeyframeEvent evt;
        evt.version = decoded.version;
        evt.descriptor_type = decoded.descriptor_type;
        evt.flags = decoded.flags;
        evt.timestamp_ns = decoded.timestamp_ns;
        evt.descriptor = std::move(decoded.descriptor);
        push_loopclosure_keyframe(evt);
        emit(keyframe_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"keyframe", "", {}});
        return;
      }

      if (type == "STAT") {
        if (!opts_.emit_stat_as_status) return;
        if (status_handlers_.empty() && !wants_any) return;
        StatusEvent evt;
        if (!decode_status_payload(frame.payload, evt.text)) {
          throw std::runtime_error("STAT decode failed");
        }
        emit(status_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"status", "", {}});
        return;
      }

      if (type == "RSET") {
        if (reset_handlers_.empty() && !wants_any) return;
        ResetEvent evt;
        const auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now());
        evt.received_at_ms = static_cast<uint64_t>(now.time_since_epoch().count());
        emit(reset_handlers_, evt);
        if (wants_any) emit_any(AnyEvent{"reset", "", {}});
        return;
      }

      if (wants_any) {
        emit_any(AnyEvent{"unknown", type, frame.payload});
      }
    } catch (const std::exception& ex) {
      {
        std::lock_guard<std::mutex> lock(stats_mu_);
        stats_.decode_errors += 1;
      }
      emit_error("protocol", "decode_failed", ex.what());
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(stats_mu_);
        stats_.decode_errors += 1;
      }
      emit_error("protocol", "decode_failed", "unknown decode error");
    }
  }

  void transport_loop() {
    while (running_.load()) {
      stream_active_.store(true);

      std::string err;
      const bool ok = device_->connect(
          [this](const uint8_t* data, size_t len) { this->handle_bytes(data, len); },
          &err);

      stream_active_.store(false);

      if (!running_.load()) break;

      if (ok) {
        emit_error("transport", "stream_closed", "stream closed");
      } else {
        emit_error("transport", "stream_error", err.empty() ? "stream error" : err);
      }

      if (!running_.load() || !opts_.auto_reconnect) break;

      {
        std::lock_guard<std::mutex> lock(stats_mu_);
        stats_.reconnects += 1;
      }

      const int delay_ms = opts_.reconnect_delay_ms > 0 ? opts_.reconnect_delay_ms : 1;
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(delay_ms);
      while (running_.load() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    stream_active_.store(false);
  }

  std::shared_ptr<MightyDeviceIO> device_;
  MightyClientOptions opts_;

  std::atomic<bool> running_{false};
  std::atomic<bool> stream_active_{false};
  std::atomic<uint32_t> req_id_{1};

  mutable std::mutex stats_mu_;
  MightyClientStats stats_;

  std::thread transport_thread_;
  FrameDispatcher frame_dispatcher_;

  ListenerSet<ImageHandler> image_handlers_;
  ListenerSet<PoseHandler> pose_handlers_;
  ListenerSet<ImuHandler> imu_handlers_;
  ListenerSet<VioStateHandler> vio_state_handlers_;
  ListenerSet<VizHandler> viz_handlers_;
  ListenerSet<LconHandler> lcon_handlers_;
  ListenerSet<KeyframeHandler> keyframe_handlers_;
  ListenerSet<LoopClosureHandler> loopclosure_handlers_;
  ListenerSet<StatusHandler> status_handlers_;
  ListenerSet<ResetHandler> reset_handlers_;
  ListenerSet<AnyHandler> any_handlers_;
  ListenerSet<ErrorHandler> error_handlers_;

  mutable std::mutex loopclosure_mu_;
  bool loopclosure_has_correction_ = false;
  std::array<double, 3> loopclosure_translation_correction_{0.0, 0.0, 0.0};
#if defined(MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE)
  mlc_device_loopclosure_t* loopclosure_ = nullptr;
  bool loopclosure_initialized_ = false;
  std::atomic<int> loopclosure_next_frame_id_{0};
#endif
};

}  // namespace sdk
}  // namespace mighty_protocol
