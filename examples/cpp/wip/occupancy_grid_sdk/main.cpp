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

std::atomic<bool> stop_requested{false};

void handleSignal(int) { stop_requested.store(true); }

struct Options {
  std::string host = "http://192.168.7.1";
  int seconds = 60;
  float resolution_m = 0.05f;
};

bool parseOptions(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i] ? argv[i] : "";
    if (argument == "--host" && i + 1 < argc) {
      options->host = argv[++i];
    } else if (argument == "--seconds" && i + 1 < argc) {
      options->seconds = std::max(1, std::atoi(argv[++i]));
    } else if (argument == "--resolution" && i + 1 < argc) {
      options->resolution_m = std::stof(argv[++i]);
    } else if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--host URL] [--seconds N] [--resolution METERS]\n";
      return false;
    } else {
      std::cerr << "Unknown or incomplete argument: " << argument << '\n';
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
#if !defined(MIGHTY_PROTOCOL_ENABLE_OCCUPANCY_GRID)
  std::cerr << "This example requires occupancy-grid support.\n";
  return 2;
#else
  Options options;
  if (!parseOptions(argc, argv, &options)) return 2;

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  using namespace mighty_protocol::sdk;

  MightyWebDeviceOptions device_options;
  device_options.base_url = options.host;
  auto device = std::make_shared<MightyWebDevice>(device_options);
  auto client = std::make_shared<MightyClient>(device);
  client->on_error([](const MightyErrorEvent& error) {
    std::cerr << "[mighty] " << error.scope << '/' << error.code << ": "
              << error.message << '\n';
  });

  client->connect();
  const auto connection_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!stop_requested.load() && !client->is_connected() &&
         std::chrono::steady_clock::now() < connection_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!client->is_connected()) {
    std::cerr << "Camera did not connect.\n";
    client->disconnect();
    return 1;
  }

  MightyOccupancyGridOptions grid_options;
  grid_options.resolution_m = options.resolution_m;
  MightyOccupancyGrid grid(client, grid_options);
  grid.setUpdateCallback([](const OccupancyGridUpdate& update) {
    std::size_t occupied_changes = 0;
    for (const OccupancyCell& cell : update.changes) {
      if (cell.state == OccupancyState::kOccupied) ++occupied_changes;
    }
    std::cout << "[grid] revision=" << update.revision
              << " changes=" << update.changes.size()
              << " occupied=" << occupied_changes
              << (update.reset ? " reset" : "") << '\n';
  });

  std::thread processing_thread([&grid]() {
    while (grid.process()) {
    }
  });

  const CommandResult start = client->start_vio();
  std::cout << "start_vio: " << (start.ok ? "ok" : "failed") << '\n';
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(options.seconds);
  while (!stop_requested.load() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  grid.close();
  processing_thread.join();
  client->stop_vio();
  client->disconnect();

  const MightyOccupancyGridStats stats = grid.stats();
  std::cout << "frames=" << stats.received_frames
            << " processed=" << stats.processed_frames
            << " dropped=" << stats.dropped_frames
            << " cells=" << stats.cell_count
            << " occupied=" << stats.occupied_cell_count << '\n';
  return 0;
#endif
}
