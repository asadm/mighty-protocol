#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "mighty_loopclosure/mighty_loopclosure_device_c.h"

namespace {

struct Options {
  std::filesystem::path replay_dir;
  std::string dump_map_path = "tmp/native-replay-map.csv";
  size_t dump_after_processed = 180;
  bool quiet = true;
};

struct RenderMap {
  std::map<int, std::vector<mmp_map_point_t>> frames;
  std::vector<mmp_pose_sample_t> trajectory;
  uint64_t revision = 0;
  size_t point_count = 0;

  void reset() {
    frames.clear();
    trajectory.clear();
    revision = 0;
    point_count = 0;
  }

  void replace_frame(int frame_id, const mmp_map_point_t* points, size_t count) {
    auto& dst = frames[frame_id];
    point_count -= dst.size();
    dst.assign(points, points + count);
    point_count += dst.size();
  }

  void remove_frame(int frame_id) {
    auto it = frames.find(frame_id);
    if (it == frames.end()) return;
    point_count -= it->second.size();
    frames.erase(it);
  }
};

bool starts_with(const std::string& s, const std::string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> out;
  std::string cell;
  std::istringstream in(line);
  while (std::getline(in, cell, ',')) out.push_back(cell);
  if (!line.empty() && line.back() == ',') out.emplace_back();
  return out;
}

uint64_t u64_or_zero(const std::string& s) {
  return s.empty() ? 0 : static_cast<uint64_t>(std::stoull(s));
}

int int_or_zero(const std::string& s) {
  return s.empty() ? 0 : std::stoi(s);
}

double double_or_zero(const std::string& s) {
  return s.empty() ? 0.0 : std::stod(s);
}

float float_or_one(const std::string& s) {
  return s.empty() ? 1.0f : std::stof(s);
}

std::vector<uint8_t> read_binary(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  in.seekg(0, std::ios::end);
  const std::streamoff size = in.tellg();
  in.seekg(0, std::ios::beg);
  std::vector<uint8_t> data(static_cast<size_t>(std::max<std::streamoff>(0, size)));
  if (!data.empty()) in.read(reinterpret_cast<char*>(data.data()), data.size());
  return data;
}

void apply_map_update(mmp_device_mapper_t* mapper, RenderMap* render_map) {
  if (!mapper || !render_map) return;
  mmp_map_update_t update{};
  const mmp_status_t status =
      mmp_map_update(mapper, render_map->revision, render_map->trajectory.size(), &update);
  if (status != MMP_STATUS_OK) {
    mmp_map_update_destroy(&update);
    return;
  }
  if (update.reset) render_map->reset();
  if (update.frames && update.frame_count > 0) {
    for (size_t i = 0; i < update.frame_count; ++i) {
      const mmp_map_frame_update_t& frame = update.frames[i];
      if (frame.remove) {
        render_map->remove_frame(frame.frame_id);
      } else {
        render_map->replace_frame(frame.frame_id, frame.points, frame.point_count);
      }
    }
  }
  if (update.trajectory && update.trajectory_count > 0) {
    if (update.trajectory_start == 0) {
      render_map->trajectory.clear();
    } else if (update.trajectory_start != render_map->trajectory.size()) {
      render_map->trajectory.clear();
      render_map->revision = 0;
      mmp_map_update_destroy(&update);
      return;
    }
    render_map->trajectory.insert(render_map->trajectory.end(),
                                  update.trajectory,
                                  update.trajectory + update.trajectory_count);
  }
  render_map->revision = update.revision;
  mmp_map_update_destroy(&update);
}

bool write_map_csv(const RenderMap& map, const std::string& path) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) return false;
  out << "frame_id,point_index,x,y,z\n";
  out << std::setprecision(9);
  for (const auto& kv : map.frames) {
    for (size_t i = 0; i < kv.second.size(); ++i) {
      const auto& p = kv.second[i];
      out << kv.first << "," << i << "," << p.x << "," << p.y << "," << p.z << "\n";
    }
  }
  return true;
}

bool parse_args(int argc, char** argv, Options* opts) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (starts_with(arg, "--replay=")) {
      opts->replay_dir = arg.substr(std::string("--replay=").size());
    } else if (arg == "--replay" && i + 1 < argc) {
      opts->replay_dir = argv[++i];
    } else if (starts_with(arg, "--dump-map=")) {
      opts->dump_map_path = arg.substr(std::string("--dump-map=").size());
    } else if (arg == "--dump-map" && i + 1 < argc) {
      opts->dump_map_path = argv[++i];
    } else if (starts_with(arg, "--dump-after-processed=")) {
      opts->dump_after_processed =
          std::max<size_t>(1, static_cast<size_t>(std::stoull(
                                  arg.substr(std::string("--dump-after-processed=").size()))));
    } else if (arg == "--dump-after-processed" && i + 1 < argc) {
      opts->dump_after_processed =
          std::max<size_t>(1, static_cast<size_t>(std::stoull(argv[++i])));
    } else if (arg == "--quiet") {
      opts->quiet = true;
    } else if (arg == "--verbose") {
      opts->quiet = false;
    } else {
      std::cerr << "unknown option: " << arg << "\n";
      return false;
    }
  }
  if (opts->replay_dir.empty()) {
    std::cerr << "--replay DIR is required\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Options opts;
  if (!parse_args(argc, argv, &opts)) {
    std::cerr << "Usage: mighty_mapper_replay --replay DIR [--dump-after-processed N]"
              << " [--dump-map CSV]\n";
    return 2;
  }

  mmp_options_t options{};
  mmp_options_default(&options);
  options.quiet = opts.quiet ? 1 : 0;
  mmp_device_mapper_t* mapper = nullptr;
  mmp_status_t status = mmp_create(&options, &mapper);
  if (status != MMP_STATUS_OK) {
    std::cerr << "mmp_create failed: " << mmp_status_message(status) << "\n";
    return 1;
  }
  status = mmp_initialize(mapper);
  if (status != MMP_STATUS_OK) {
    std::cerr << "mmp_initialize failed: " << mmp_status_message(status) << "\n";
    mmp_destroy(mapper);
    return 1;
  }
  const auto calib_path = opts.replay_dir / "calib.yaml";
  status = mmp_set_calibration_yaml(mapper, calib_path.string().c_str());
  if (status != MMP_STATUS_OK) {
    std::cerr << "mmp_set_calibration_yaml failed: " << mmp_status_message(status) << "\n";
    mmp_finish(mapper);
    mmp_destroy(mapper);
    return 1;
  }

  std::ifstream events(opts.replay_dir / "events.csv");
  if (!events) {
    std::cerr << "failed to open events.csv\n";
    mmp_finish(mapper);
    mmp_destroy(mapper);
    return 1;
  }
  std::string line;
  std::getline(events, line);

  RenderMap map;
  mmp_push_result_t last_result{};
  size_t pushed_events = 0;
  while (std::getline(events, line)) {
    if (line.empty()) continue;
    const auto row = split_csv_line(line);
    if (row.size() < 17) continue;
    mmp_push_result_t result{};
    if (row[0] == "image") {
      const std::vector<uint8_t> data = read_binary(opts.replay_dir / row[7]);
      mlc_raw_image_t image{};
      image.frame_id = int_or_zero(row[2]);
      image.timestamp_ns = u64_or_zero(row[3]);
      image.width = static_cast<uint32_t>(int_or_zero(row[4]));
      image.height = static_cast<uint32_t>(int_or_zero(row[5]));
      image.format = static_cast<uint8_t>(int_or_zero(row[6]));
      image.data = data.data();
      image.size_bytes = data.size();
      status = mmp_push_image(mapper, &image, &result);
    } else if (row[0] == "pose") {
      mlc_pose_t pose{};
      pose.timestamp_ns = u64_or_zero(row[3]);
      pose.px = double_or_zero(row[8]);
      pose.py = double_or_zero(row[9]);
      pose.pz = double_or_zero(row[10]);
      pose.qx = double_or_zero(row[11]);
      pose.qy = double_or_zero(row[12]);
      pose.qz = double_or_zero(row[13]);
      pose.qw = row[14].empty() ? 1.0 : std::stod(row[14]);
      pose.frame = static_cast<uint8_t>(int_or_zero(row[15]));
      pose.confidence = float_or_one(row[16]);
      status = mmp_push_pose(mapper, &pose, &result);
    } else {
      continue;
    }
    pushed_events += 1;
    if (status != MMP_STATUS_OK && status != MMP_STATUS_NOT_READY && status != MMP_STATUS_LOST) {
      std::cerr << "push failed at event " << pushed_events << ": " << mmp_status_message(status) << "\n";
      break;
    }
    if (result.version != 0) last_result = result;
    apply_map_update(mapper, &map);
    if (last_result.frames_processed >= opts.dump_after_processed) break;
  }
  apply_map_update(mapper, &map);

  const bool wrote_map = write_map_csv(map, opts.dump_map_path);
  mmp_finish(mapper);
  mmp_destroy(mapper);

  std::cout << "native replay events=" << pushed_events
            << " processed=" << last_result.frames_processed
            << " map_points=" << map.point_count
            << " wrote_map=" << (wrote_map ? "yes" : "no")
            << "\n";
  return wrote_map ? 0 : 1;
}
