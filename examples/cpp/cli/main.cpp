#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "cpp/mighty_sdk.h"

namespace {

std::atomic<bool> g_stop{false};

void handle_signal(int) {
  g_stop.store(true);
}

struct Options {
  std::string host;
  int seconds = 10;
  bool start_vio = true;
};

void print_usage(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [--host URL] [--seconds N] [--no-start]\n"
            << "\n"
            << "Examples:\n"
            << "  " << argv0 << "\n"
            << "  " << argv0 << " --host http://192.168.7.1 --seconds 30\n";
}

bool parse_args(int argc, char** argv, Options* opts) {
  if (!opts) return false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? argv[i] : "";
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (arg == "--host") {
      if (i + 1 >= argc) {
        std::cerr << "--host requires a URL\n";
        return false;
      }
      opts->host = argv[++i];
    } else if (arg == "--seconds") {
      if (i + 1 >= argc) {
        std::cerr << "--seconds requires a number\n";
        return false;
      }
      opts->seconds = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--no-start") {
      opts->start_vio = false;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Options opts;
  if (!parse_args(argc, argv, &opts)) {
    print_usage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  using namespace mighty_protocol::sdk;

  MightyWebDeviceOptions device_opts;
  if (!opts.host.empty()) {
    device_opts.base_url = opts.host;
  }

  auto device = std::make_shared<MightyWebDevice>(device_opts);
  MightyClient client(device);

  client.on_error([](const MightyErrorEvent& e) {
    std::cerr << "[error] " << e.scope << " " << e.code << ": "
              << e.message << std::endl;
  });

  client.on_status([](const StatusEvent& s) {
    std::cout << "[status] " << s.text << std::endl;
  });

  client.on_vio_state([](const VioStateFrame& s) {
    std::cout << "[vio] state=" << static_cast<int>(s.state)
              << " confidence=" << s.pose_confidence
              << " features=" << s.num_features
              << " init_reason=" << static_cast<int>(s.init_reason_code)
              << std::endl;
  });

  client.on_pose([](const PoseFrame& p) {
    std::cout << "[pose] t=";
    if (p.timestamp_ns.has_value()) {
      std::cout << *p.timestamp_ns;
    } else {
      std::cout << "unknown";
    }
    std::cout << " p=(" << p.position_m[0] << ", "
              << p.position_m[1] << ", " << p.position_m[2] << ")"
              << " confidence=" << p.confidence << std::endl;
  });

  client.connect();

  const auto info = device->get_info();
  std::cout << "transport=" << info.transport << " source=" << info.source << std::endl;

  if (opts.start_vio) {
    const auto res = client.start_vio();
    std::cout << "start_vio ok=" << (res.ok ? "true" : "false")
              << " status=" << static_cast<int>(res.status)
              << " message=\"" << res.message << "\"" << std::endl;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(opts.seconds);
  while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (opts.start_vio) {
    const auto res = client.stop_vio();
    std::cout << "stop_vio ok=" << (res.ok ? "true" : "false")
              << " status=" << static_cast<int>(res.status)
              << " message=\"" << res.message << "\"" << std::endl;
  }

  client.disconnect();

  const auto stats = client.stats();
  std::cout << "stats frames=" << stats.rx_frames
            << " bytes=" << stats.rx_bytes
            << " decode_errors=" << stats.decode_errors
            << " reconnects=" << stats.reconnects
            << " command_timeouts=" << stats.command_timeouts
            << std::endl;

  return 0;
}
