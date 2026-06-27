#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "../cpp/mighty_sdk.h"

using namespace mighty_protocol;
using namespace mighty_protocol::sdk;

namespace {

class MockDevice : public MightyDeviceIO {
 public:
  DeviceInfo get_info() const override {
    return DeviceInfo{"mock", "mock://device"};
  }

  bool connect(BytesCallback on_bytes, std::string* error) override {
    if (!on_bytes) {
      if (error) *error = "missing callback";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mu_);
      if (connected_) {
        if (error) *error = "already connected";
        return false;
      }
      connected_ = true;
      on_bytes_ = std::move(on_bytes);
      stop_ = false;
    }

    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait(lock, [this]() { return stop_; });
    connected_ = false;
    on_bytes_ = nullptr;
    return true;
  }

  void disconnect() override {
    {
      std::lock_guard<std::mutex> lock(mu_);
      stop_ = true;
    }
    cv_.notify_all();
  }

  bool send_command_payload(const std::vector<uint8_t>& cmd_payload,
                            std::vector<uint8_t>* response_payload,
                            std::string* error) override {
    CommandRequest cmd;
    if (!decode_command_payload(cmd_payload, cmd)) {
      if (error) *error = "bad command payload";
      return false;
    }

    if (cmd.name == "start_vio" || cmd.name == "stop_vio") {
      CommandResponse ok;
      ok.req_id = cmd.req_id;
      ok.status = 0;
      ok.message = "ok";
      if (response_payload) *response_payload = build_command_response_payload(ok);
      return true;
    }

    if (cmd.name == "reset_vio_pose") {
      ResetVioPoseRequest reset_pose{};
      if (!decode_reset_vio_pose_payload(cmd.data, reset_pose)) {
        if (error) *error = "bad reset pose payload";
        return false;
      }
      last_reset_pose = reset_pose;
      CommandResponse ok;
      ok.req_id = cmd.req_id;
      ok.status = 0;
      ok.message = "pose reset";
      if (response_payload) *response_payload = build_command_response_payload(ok);
      return true;
    }

    if (cmd.name == "keyframes") {
      const std::string action(cmd.data.begin(), cmd.data.end());
      CommandResponse ok;
      ok.req_id = cmd.req_id;
      ok.status = 0;
      ok.message = action == "status" ? "keyframes disabled" : "keyframes " + action;
      if (response_payload) *response_payload = build_command_response_payload(ok);
      return true;
    }

    if (cmd.name == "config") {
      ConfigRequest cfgq;
      if (!decode_config_request_payload(cmd.data, cfgq)) {
        if (error) *error = "bad config request payload";
        return false;
      }

      if (cfgq.key != "calib") {
        ConfigResponse cfgr;
        cfgr.version = cfgq.version;
        cfgr.op = cfgq.op;
        cfgr.success = 0;
        cfgr.has_value = false;
        cfgr.key = cfgq.key;
        cfgr.message = "unknown key";

        CommandResponse cres;
        cres.req_id = cmd.req_id;
        cres.status = 1;
        cres.message = "config failed";
        cres.data = build_config_response_payload(cfgr);
        if (response_payload) *response_payload = build_command_response_payload(cres);
        return true;
      }

      if (cfgq.op == static_cast<uint8_t>(ConfigOp::kGet)) {
        ConfigResponse cfgr;
        cfgr.version = cfgq.version;
        cfgr.op = cfgq.op;
        cfgr.success = 1;
        cfgr.has_value = true;
        cfgr.key = "calib";
        cfgr.message = "loaded";
        cfgr.value.assign(calib_.begin(), calib_.end());

        CommandResponse cres;
        cres.req_id = cmd.req_id;
        cres.status = 0;
        cres.message = "ok";
        cres.data = build_config_response_payload(cfgr);
        if (response_payload) *response_payload = build_command_response_payload(cres);
        return true;
      }

      if (cfgq.op == static_cast<uint8_t>(ConfigOp::kSet)) {
        calib_.assign(cfgq.value.begin(), cfgq.value.end());

        ConfigResponse cfgr;
        cfgr.version = cfgq.version;
        cfgr.op = cfgq.op;
        cfgr.success = 1;
        cfgr.has_value = true;
        cfgr.key = "calib";
        cfgr.message = "saved";
        cfgr.value.assign(calib_.begin(), calib_.end());

        CommandResponse cres;
        cres.req_id = cmd.req_id;
        cres.status = 0;
        cres.message = "ok";
        cres.data = build_config_response_payload(cfgr);
        if (response_payload) *response_payload = build_command_response_payload(cres);
        return true;
      }
    }

    CommandResponse fail;
    fail.req_id = cmd.req_id;
    fail.status = 1;
    fail.message = "unknown command";
    if (response_payload) *response_payload = build_command_response_payload(fail);
    return true;
  }

  void emit_packet(const std::vector<uint8_t>& pkt) {
    BytesCallback cb;
    {
      std::lock_guard<std::mutex> lock(mu_);
      cb = on_bytes_;
    }
    if (cb) cb(pkt.data(), pkt.size());
  }

  std::optional<ResetVioPoseRequest> last_reset_pose;

 private:
  mutable std::mutex mu_;
  std::condition_variable cv_;
  bool connected_ = false;
  bool stop_ = false;
  BytesCallback on_bytes_;
  std::string calib_ = "%YAML:1.0\ncam0:\n  intrinsics: [1,2,3,4]\n";
};

bool wait_until(const std::function<bool()>& pred, int timeout_ms = 2000) {
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool approx(double a, double b, double eps = 1e-6) {
  return std::abs(a - b) < eps;
}

}  // namespace

int main() {
  auto device = std::make_shared<MockDevice>();

  MightyClientOptions opts;
  opts.auto_reconnect = false;

  MightyClient client(device, opts);

  struct Seen {
    std::atomic<int> image{0};
    std::atomic<int> pose{0};
    std::atomic<int> imu{0};
    std::atomic<int> vsta{0};
    std::atomic<int> lcon{0};
    std::atomic<int> keyframe{0};
    std::atomic<int> status{0};
    std::atomic<int> reset{0};
    std::atomic<int> any{0};
  };

  Seen seen;
  std::optional<ImageFrame> last_image;
  std::optional<PoseFrame> last_pose;
  std::optional<VioStateFrame> last_vsta;
  std::optional<KeyframeEvent> last_keyframe;

  client.on_image([&](const ImageFrame& f) {
    seen.image.fetch_add(1);
    last_image = f;
  });
  client.on_pose([&](const PoseFrame& p) {
    seen.pose.fetch_add(1);
    last_pose = p;
  });
  client.on_imu([&](const ImuBatch&) { seen.imu.fetch_add(1); });
  client.on_vio_state([&](const VioStateFrame& v) {
    seen.vsta.fetch_add(1);
    last_vsta = v;
  });
  client.on_lcon([&](const LconFrame&) { seen.lcon.fetch_add(1); });
  client.on_keyframe([&](const KeyframeEvent& k) {
    seen.keyframe.fetch_add(1);
    last_keyframe = k;
  });
  client.on_status([&](const StatusEvent&) { seen.status.fetch_add(1); });
  client.on_reset([&](const ResetEvent&) { seen.reset.fetch_add(1); });
  client.on_any([&](const AnyEvent&) { seen.any.fetch_add(1); });

  client.connect();
  assert(wait_until([&]() { return client.is_connected(); }, 1000));

  const std::vector<uint8_t> raw_data = {0x01, 0x02};
  auto raw_payload = build_raw_payload(/*timestamp_ns=*/10,
                                       /*channel=*/"cam0",
                                       /*width=*/2,
                                       /*height=*/1,
                                       /*format=*/static_cast<uint8_t>(RawFormat::kGray8),
                                       raw_data.data(),
                                       raw_data.size());
  device->emit_packet(make_packet(raw_payload, TYPE_RAW));

  const std::array<double, 4> pose_quat{{0.1, 0.2, 0.3, 0.9}};
  const std::array<double, 3> pose_linvel{{4.0, 5.0, 6.0}};
  const std::array<double, 3> pose_angvel{{0.4, 0.5, 0.6}};
  const std::array<double, 3> pose_linacc{{7.0, 8.0, 9.0}};
  const std::array<double, 3> pose_angacc{{0.7, 0.8, 0.9}};
  auto pose_payload = build_pose_payload(/*pose_type=*/0,
                                         /*has_quat=*/true,
                                         /*is_keyframe=*/true,
                                         /*x=*/1.0,
                                         /*y=*/2.0,
                                         /*z=*/3.0,
                                         /*quat_or_null=*/pose_quat.data(),
                                         /*confidence=*/0.5f,
                                         /*linvel=*/pose_linvel.data(),
                                         /*angvel=*/pose_angvel.data(),
                                         /*linacc=*/pose_linacc.data(),
                                         /*angacc=*/pose_angacc.data(),
                                         /*timestamp_ns=*/std::optional<uint64_t>(11));
  device->emit_packet(make_packet(pose_payload, TYPE_POSE));

  std::vector<ImuSample> imu = {
      {12, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
  };
  device->emit_packet(make_packet(build_imu_payload(imu), TYPE_IMU));

  VioState vsta;
  vsta.version = 4;
  vsta.state = 2;
  vsta.flags = 3;
  vsta.timestamp_ns = 13;
  vsta.fps_current = 30.0f;
  vsta.fps_average = 29.0f;
  vsta.pose_confidence01 = 0.8f;
  vsta.tracking_rate = 0.9f;
  vsta.num_features = 100;
  vsta.loop_closures = 2;
  vsta.build_version = "test";
  vsta.imu_hz_current = 200.0f;
  vsta.imu_hz_average_5s = 199.0f;
  vsta.init_reason_code = static_cast<uint8_t>(VioInitReasonCode::kNone);
  device->emit_packet(make_packet(build_vio_state_payload(vsta), TYPE_VSTA));

  PoseConstraintSegment seg;
  seg.type = 1;
  seg.start[0] = 0.0f; seg.start[1] = 0.0f; seg.start[2] = 0.0f;
  seg.end[0] = 1.0f; seg.end[1] = 1.0f; seg.end[2] = 1.0f;
  std::vector<PoseConstraintSegment> segs = {seg};
  device->emit_packet(make_packet(build_constraints_payload(segs), TYPE_LCON));

  device->emit_packet(make_packet(build_keyframe_payload(14, std::vector<float>{0.25f, -0.5f, 1.0f}), TYPE_KEYF));

  device->emit_packet(make_packet(build_status_payload("hello"), TYPE_STAT));
  device->emit_packet(make_packet(nullptr, 0, TYPE_RSET));
  device->emit_packet(make_packet(std::vector<uint8_t>{0xAA}, "ZZZZ"));

  assert(wait_until([&]() { return seen.any.load() >= 9; }, 2000));

  assert(seen.image.load() == 1);
  assert(last_image.has_value());
  assert(last_image->kind == ImageFrame::Kind::kRaw);
  assert(last_image->left.channel == "cam0");
  assert(last_image->left.channel_alias == "cam0");

  assert(seen.pose.load() == 1);
  assert(last_pose.has_value());
  assert(last_pose->is_public);
  assert(last_pose->packet_type == "POSE");
  assert(last_pose->pose_type == "body");
  assert(last_pose->pose_type_raw == 0);
  assert(last_pose->frame_id == "odom");
  assert(last_pose->child_frame_id == "base_link");
  assert(last_pose->is_keyframe);
  assert((last_pose->pose_flags & 0x1u) != 0u);
  assert((last_pose->pose_flags & (1u << 2)) != 0u);
  assert((last_pose->pose_flags & (1u << 3)) != 0u);
  assert((last_pose->pose_flags & (1u << 4)) != 0u);
  assert((last_pose->pose_flags & (1u << 5)) != 0u);
  assert((last_pose->pose_flags & (1u << 6)) != 0u);
  assert(last_pose->orientation_xyzw.has_value());
  assert(last_pose->linear_velocity_body_mps.has_value());
  assert(last_pose->angular_velocity_body_rps.has_value());
  assert(last_pose->linear_acceleration_body_mps2.has_value());
  assert(last_pose->angular_acceleration_body_rps2.has_value());
  assert(last_pose->timestamp_ns.has_value());
  assert(last_pose->timestamp_ns.value() == 11);
  assert(approx(last_pose->orientation_xyzw.value()[0], pose_quat[0]));
  assert(approx(last_pose->orientation_xyzw.value()[3], pose_quat[3]));
  assert(approx(last_pose->linear_velocity_body_mps.value()[2], pose_linvel[2]));
  assert(approx(last_pose->angular_velocity_body_rps.value()[1], pose_angvel[1]));
  assert(approx(last_pose->linear_acceleration_body_mps2.value()[0], pose_linacc[0]));
  assert(approx(last_pose->angular_acceleration_body_rps2.value()[2], pose_angacc[2]));

  assert(seen.imu.load() == 1);
  assert(seen.vsta.load() == 1);
  assert(last_vsta.has_value());
  assert(last_vsta->init_reason_code == static_cast<uint8_t>(VioInitReasonCode::kNone));
  assert(last_vsta->init_reason == VioInitReasonCode::kNone);
  assert(seen.lcon.load() == 1);
  assert(seen.keyframe.load() == 1);
  assert(last_keyframe.has_value());
  assert(last_keyframe->timestamp_ns == 14);
  assert(last_keyframe->descriptor.size() == 3);
  assert(approx(last_keyframe->descriptor[1], -0.5));
  assert(seen.status.load() == 1);
  assert(seen.reset.load() == 1);

  CommandResult cmd = client.start_vio();
  assert(cmd.ok);

  CommandResult reset_pose = client.reset_vio_pose({0.0, 0.0, 0.0});
  assert(reset_pose.ok);
  assert(device->last_reset_pose.has_value());
  assert(approx(device->last_reset_pose->position_m[0], 0.0));
  assert(!device->last_reset_pose->orientation_xyzw.has_value());

  CommandResult reset_pose_quat =
      client.reset_vio_pose({1.0, 2.0, 3.0}, std::array<double, 4>{0.0, 0.0, 0.0, 1.0});
  assert(reset_pose_quat.ok);
  assert(device->last_reset_pose.has_value());
  assert(approx(device->last_reset_pose->position_m[2], 3.0));
  assert(device->last_reset_pose->orientation_xyzw.has_value());
  assert(approx(device->last_reset_pose->orientation_xyzw->at(3), 1.0));

  CommandResult keyframes_on = client.set_keyframes_enabled(true);
  assert(keyframes_on.ok);
  assert(keyframes_on.message == "keyframes on");

  CommandResult keyframes_status = client.keyframes_status();
  assert(keyframes_status.ok);
  assert(keyframes_status.message == "keyframes disabled");

  ConfigGetTextResult cfg_get = client.config_get_text("calib");
  assert(cfg_get.ok);
  assert(cfg_get.found);
  assert(cfg_get.value.find("intrinsics") != std::string::npos);

  ConfigSetResult cfg_set = client.config_set_text("calib", "%YAML:1.0\nfoo: 1\n");
  assert(cfg_set.ok);

  ConfigGetTextResult cfg_get2 = client.config_get_text("calib");
  assert(cfg_get2.ok);
  assert(cfg_get2.value == "%YAML:1.0\nfoo: 1\n");

  MightyClientStats st = client.stats();
  assert(st.rx_frames >= 8);
  assert(st.rx_bytes > 0);

  client.disconnect();
  assert(!client.is_connected());

  std::cout << "C++ SDK unit test passed" << std::endl;
  return 0;
}
