#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
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

struct IntegrationState {
  std::mutex mu;
  std::string calib = "%YAML:1.0\ncam0:\n  intrinsics: [9,8,7,6]\n";
  std::vector<std::vector<uint8_t>> stream_chunks;
};

bool wait_until(const std::function<bool()>& pred, int timeout_ms = 3000) {
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
  IntegrationState state;

  auto raw_payload = build_raw_payload(/*timestamp_ns=*/10,
                                       /*channel=*/"cam0",
                                       /*width=*/2,
                                       /*height=*/1,
                                       /*format=*/static_cast<uint8_t>(RawFormat::kGray8),
                                       reinterpret_cast<const uint8_t*>("\x01\x02"),
                                       2);

  const std::array<double, 4> pose_quat{{0.1, 0.2, 0.3, 0.9}};
  const std::array<double, 3> pose_linvel{{4.0, 5.0, 6.0}};
  const std::array<double, 3> pose_angvel{{0.4, 0.5, 0.6}};
  const std::array<double, 3> pose_linacc{{7.0, 8.0, 9.0}};
  const std::array<double, 3> pose_angacc{{0.7, 0.8, 0.9}};
  auto pose_payload = build_pose_payload(/*pose_type=*/0,
                                         /*has_quat=*/true,
                                         /*is_keyframe=*/false,
                                         /*x=*/1.0,
                                         /*y=*/2.0,
                                         /*z=*/3.0,
                                         /*quat_or_null=*/pose_quat.data(),
                                         /*confidence=*/0.9f,
                                         /*linvel=*/pose_linvel.data(),
                                         /*angvel=*/pose_angvel.data(),
                                         /*linacc=*/pose_linacc.data(),
                                         /*angacc=*/pose_angacc.data(),
                                         /*timestamp_ns=*/std::optional<uint64_t>(11));

  std::vector<ImuSample> imu = {
      {12, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
  };

  VioState vsta;
  vsta.version = 3;
  vsta.state = 2;
  vsta.flags = 1;
  vsta.timestamp_ns = 123;
  vsta.fps_current = 30.0f;
  vsta.fps_average = 29.0f;
  vsta.pose_confidence01 = 0.7f;
  vsta.tracking_rate = 0.8f;
  vsta.num_features = 120;
  vsta.loop_closures = 3;
  vsta.build_version = "test";
  vsta.imu_hz_current = 200.0f;
  vsta.imu_hz_average_5s = 199.0f;

  std::vector<std::vector<uint8_t>> packets;
  packets.push_back(make_packet(nullptr, 0, TYPE_RSET));
  packets.push_back(make_packet(raw_payload, TYPE_RAW));
  packets.push_back(make_packet(pose_payload, TYPE_POSE));
  packets.push_back(make_packet(build_imu_payload(imu), TYPE_IMU));
  packets.push_back(make_packet(build_vio_state_payload(vsta), TYPE_VSTA));
  packets.push_back(make_packet(build_status_payload("hello"), TYPE_STAT));

  for (const auto& pkt : packets) {
    const size_t mid = pkt.size() / 2;
    if (mid > 0) {
      state.stream_chunks.emplace_back(pkt.begin(), pkt.begin() + static_cast<std::ptrdiff_t>(mid));
      state.stream_chunks.emplace_back(pkt.begin() + static_cast<std::ptrdiff_t>(mid), pkt.end());
    } else {
      state.stream_chunks.push_back(pkt);
    }
  }

  httplib::Server server;

  server.Get("/stream", [&](const httplib::Request&, httplib::Response& res) {
    std::vector<uint8_t> body;
    for (const auto& part : state.stream_chunks) {
      body.insert(body.end(), part.begin(), part.end());
    }
    res.set_content(reinterpret_cast<const char*>(body.data()), body.size(), "application/octet-stream");
  });

  server.Post("/command", [&](const httplib::Request& req, httplib::Response& res) {
    std::vector<uint8_t> cmd_payload(req.body.begin(), req.body.end());

    CommandRequest cmd;
    if (!decode_command_payload(cmd_payload, cmd)) {
      res.status = 400;
      res.set_content("", "application/octet-stream");
      return;
    }

    if (cmd.name == "start_vio" || cmd.name == "stop_vio") {
      CommandResponse cres;
      cres.req_id = cmd.req_id;
      cres.status = 0;
      cres.message = "ok";
      const auto body = build_command_response_payload(cres);
      res.set_content(reinterpret_cast<const char*>(body.data()), body.size(), "application/octet-stream");
      return;
    }

    if (cmd.name == "config") {
      ConfigRequest cfgq;
      if (!decode_config_request_payload(cmd.data, cfgq)) {
        res.status = 400;
        res.set_content("", "application/octet-stream");
        return;
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

        const auto body = build_command_response_payload(cres);
        res.set_content(reinterpret_cast<const char*>(body.data()), body.size(), "application/octet-stream");
        return;
      }

      if (cfgq.op == static_cast<uint8_t>(ConfigOp::kGet)) {
        ConfigResponse cfgr;
        cfgr.version = cfgq.version;
        cfgr.op = cfgq.op;
        cfgr.success = 1;
        cfgr.has_value = true;
        cfgr.key = "calib";
        cfgr.message = "loaded";
        {
          std::lock_guard<std::mutex> lock(state.mu);
          cfgr.value.assign(state.calib.begin(), state.calib.end());
        }

        CommandResponse cres;
        cres.req_id = cmd.req_id;
        cres.status = 0;
        cres.message = "ok";
        cres.data = build_config_response_payload(cfgr);

        const auto body = build_command_response_payload(cres);
        res.set_content(reinterpret_cast<const char*>(body.data()), body.size(), "application/octet-stream");
        return;
      }

      if (cfgq.op == static_cast<uint8_t>(ConfigOp::kSet)) {
        {
          std::lock_guard<std::mutex> lock(state.mu);
          state.calib.assign(cfgq.value.begin(), cfgq.value.end());
        }

        ConfigResponse cfgr;
        cfgr.version = cfgq.version;
        cfgr.op = cfgq.op;
        cfgr.success = 1;
        cfgr.has_value = true;
        cfgr.key = "calib";
        cfgr.message = "saved";
        {
          std::lock_guard<std::mutex> lock(state.mu);
          cfgr.value.assign(state.calib.begin(), state.calib.end());
        }

        CommandResponse cres;
        cres.req_id = cmd.req_id;
        cres.status = 0;
        cres.message = "ok";
        cres.data = build_config_response_payload(cfgr);

        const auto body = build_command_response_payload(cres);
        res.set_content(reinterpret_cast<const char*>(body.data()), body.size(), "application/octet-stream");
        return;
      }
    }

    CommandResponse fail;
    fail.req_id = cmd.req_id;
    fail.status = 1;
    fail.message = "unknown command";
    const auto body = build_command_response_payload(fail);
    res.set_content(reinterpret_cast<const char*>(body.data()), body.size(), "application/octet-stream");
  });

  const int port = server.bind_to_any_port("127.0.0.1");
  assert(port > 0);

  std::thread server_thread([&]() {
    server.listen_after_bind();
  });
  server.wait_until_ready();

  try {
    MightyWebDeviceOptions dev_opts;
    dev_opts.base_url = "http://127.0.0.1:" + std::to_string(port);
    dev_opts.read_timeout_ms = 200;

    auto device = std::make_shared<MightyWebDevice>(dev_opts);

    MightyClientOptions client_opts;
    client_opts.auto_reconnect = false;

    MightyClient client(device, client_opts);

    struct Seen {
      std::atomic<int> image{0};
      std::atomic<int> pose{0};
      std::atomic<int> imu{0};
      std::atomic<int> vsta{0};
      std::atomic<int> status{0};
      std::atomic<int> reset{0};
      std::atomic<int> any{0};
      std::atomic<int> error{0};
    };
    Seen seen;
    std::optional<PoseFrame> last_pose;

    client.on_image([&](const ImageFrame&) { seen.image.fetch_add(1); });
    client.on_pose([&](const PoseFrame& p) {
      seen.pose.fetch_add(1);
      last_pose = p;
    });
    client.on_imu([&](const ImuBatch&) { seen.imu.fetch_add(1); });
    client.on_vio_state([&](const VioStateFrame&) { seen.vsta.fetch_add(1); });
    client.on_status([&](const StatusEvent&) { seen.status.fetch_add(1); });
    client.on_reset([&](const ResetEvent&) { seen.reset.fetch_add(1); });
    client.on_any([&](const AnyEvent&) { seen.any.fetch_add(1); });
    client.on_error([&](const MightyErrorEvent& e) {
      seen.error.fetch_add(1);
      (void)e;
    });

    client.connect();
    wait_until([&]() { return seen.any.load() >= 6 || seen.error.load() > 0; }, 3000);
    assert(seen.any.load() >= 6);

    assert(seen.image.load() >= 1);
    assert(seen.pose.load() >= 1);
    assert(seen.imu.load() >= 1);
    assert(seen.vsta.load() >= 1);
    assert(seen.status.load() >= 1);
    assert(seen.reset.load() >= 1);
    assert(last_pose.has_value());
    assert(last_pose->is_public);
    assert(last_pose->packet_type == "POSE");
    assert(last_pose->pose_type == "body");
    assert(last_pose->pose_type_raw == 0);
    assert(last_pose->frame_id == "odom");
    assert(last_pose->child_frame_id == "base_link");
    assert((last_pose->pose_flags & 0x1u) != 0u);
    assert((last_pose->pose_flags & (1u << 2)) != 0u);
    assert((last_pose->pose_flags & (1u << 3)) != 0u);
    assert((last_pose->pose_flags & (1u << 4)) != 0u);
    assert((last_pose->pose_flags & (1u << 5)) != 0u);
    assert((last_pose->pose_flags & (1u << 6)) != 0u);
    assert(last_pose->orientation_xyzw.has_value());
    assert(last_pose->linear_velocity_body_mps.has_value());
    assert(last_pose->timestamp_ns.has_value());
    assert(last_pose->timestamp_ns.value() == 11);
    assert(approx(last_pose->orientation_xyzw.value()[3], 0.9));
    assert(approx(last_pose->linear_velocity_body_mps.value()[2], 6.0));

    CommandResult cmd = client.start_vio();
    assert(cmd.ok);

    ConfigGetTextResult cfg_get = client.config_get_text("calib");
    assert(cfg_get.ok);
    assert(cfg_get.found);
    assert(cfg_get.value.find("intrinsics") != std::string::npos);

    ConfigSetResult cfg_set = client.config_set_text("calib", "%YAML:1.0\nfoo: 9\n");
    assert(cfg_set.ok);

    ConfigGetTextResult cfg_get2 = client.config_get_text("calib");
    assert(cfg_get2.ok);
    assert(cfg_get2.value == "%YAML:1.0\nfoo: 9\n");

    client.disconnect();

    std::cout << "C++ SDK HTTP integration test passed" << std::endl;
  } catch (...) {
    server.stop();
    if (server_thread.joinable()) server_thread.join();
    throw;
  }

  server.stop();
  if (server_thread.joinable()) server_thread.join();
  return 0;
}
