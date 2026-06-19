#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "cpp/mighty_sdk.h"
#include "examples/cpp/wip/loopclosure_sdk/utils.h"

namespace {

using loopclosure_sdk_example::Options;
using loopclosure_sdk_example::State;
using loopclosure_sdk_example::parse_args;
using loopclosure_sdk_example::print_loop_event;
using loopclosure_sdk_example::print_usage;
using loopclosure_sdk_example::record_keyframe_pose;
using loopclosure_sdk_example::record_loop_event;
using loopclosure_sdk_example::set_error;
using loopclosure_sdk_example::set_source;
using loopclosure_sdk_example::set_status;
using loopclosure_sdk_example::write_svg;

using mighty_protocol::sdk::LoopClosureEvent;
using mighty_protocol::sdk::MightyClient;
using mighty_protocol::sdk::MightyClientOptions;
using mighty_protocol::sdk::MightyErrorEvent;
using mighty_protocol::sdk::MightyWebDevice;
using mighty_protocol::sdk::MightyWebDeviceOptions;
using mighty_protocol::sdk::PoseFrame;
using mighty_protocol::sdk::StatusEvent;

std::atomic<bool> g_stop{false};

void handle_signal(int) {
  g_stop.store(true);
}

}  // namespace

int main(int argc, char** argv) {
#if !defined(MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE)
  std::cerr << "This example must be compiled with MIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE.\n";
  return 2;
#else
  Options opts;
  if (!parse_args(argc, argv, &opts)) {
    print_usage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  MightyWebDeviceOptions device_opts;
  if (!opts.host.empty()) device_opts.base_url = opts.host;
  auto device = std::make_shared<MightyWebDevice>(device_opts);

  MightyClientOptions client_opts;
  client_opts.loopclosure = true;
  client_opts.loopclosure_calibration_yaml = opts.calibration_yaml;

  State state;
  auto client = std::make_shared<MightyClient>(device, client_opts);

  client->on_error([&](const MightyErrorEvent& e) {
    const std::string text = e.scope + " " + e.code + ": " + e.message;
    set_error(&state, text);
    std::cerr << "[error] " << text << std::endl;
  });

  client->on_status([&](const StatusEvent& s) {
    set_status(&state, s.text);
    std::cout << "[status] " << s.text << std::endl;
  });

  client->on_pose([&](const PoseFrame& p) {
    record_keyframe_pose(&state, p);
  });

  client->on_loopclosure([&](const LoopClosureEvent& e) {
    print_loop_event(e);
    record_loop_event(&state, e);
    write_svg(opts, state, opts.output_svg);
  });

  client->connect();
  const auto info = device->get_info();
  set_source(&state, info.source);
  std::cout << "transport=http source=" << info.source << std::endl;

  bool calibration_loaded = false;
  if (!opts.calibration_yaml.empty()) {
    calibration_loaded = client->set_loopclosure_calibration_yaml(opts.calibration_yaml);
  } else {
    for (int attempt = 0; attempt < 40 && !g_stop.load(); ++attempt) {
      const auto result = client->config_get_text("calib");
      if (result.ok && result.found && !result.value.empty()) {
        calibration_loaded = client->set_loopclosure_calibration_yaml(result.value);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  std::cout << "loop-closure calibration loaded="
            << (calibration_loaded ? "true" : "false") << std::endl;

  if (opts.start_vio) {
    const auto res = client->start_vio();
    std::cout << "start_vio ok=" << (res.ok ? "true" : "false")
              << " status=" << static_cast<int>(res.status)
              << " message=\"" << res.message << "\"" << std::endl;
  }

  write_svg(opts, state, opts.output_svg);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(opts.seconds);
  auto next_render = std::chrono::steady_clock::now();
  while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::steady_clock::now() >= next_render) {
      write_svg(opts, state, opts.output_svg);
      next_render = std::chrono::steady_clock::now() + std::chrono::milliseconds(opts.update_ms);
    }
  }

  if (opts.start_vio) {
    const auto res = client->stop_vio();
    std::cout << "stop_vio ok=" << (res.ok ? "true" : "false")
              << " status=" << static_cast<int>(res.status)
              << " message=\"" << res.message << "\"" << std::endl;
  }
  client->disconnect();
  write_svg(opts, state, opts.output_svg);

  const auto stats = client->stats();
  std::cout << "wrote " << opts.output_svg << "\n";
  std::cout << "stats frames=" << stats.rx_frames
            << " bytes=" << stats.rx_bytes
            << " decode_errors=" << stats.decode_errors
            << " reconnects=" << stats.reconnects
            << " command_timeouts=" << stats.command_timeouts << std::endl;
  return 0;
#endif
}
