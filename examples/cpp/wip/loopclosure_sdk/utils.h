#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "cpp/mighty_sdk.h"

namespace loopclosure_sdk_example {

using mighty_protocol::sdk::LoopClosureEvent;
using mighty_protocol::sdk::PoseFrame;

struct Options {
  std::string host;
  std::string calibration_yaml;
  std::string output_svg = "loopclosure_topdown.svg";
  std::string axes = "xy";
  int seconds = 60;
  int update_ms = 1000;
  bool start_vio = true;
};

struct LoopEdge {
  size_t current_keyframe = 0;
  size_t matched_keyframe = 0;
  std::string type;
};

struct KeyframePose {
  std::array<double, 3> raw_position_m{0.0, 0.0, 0.0};
  std::array<double, 3> position_m{0.0, 0.0, 0.0};
};

struct State {
  mutable std::mutex mu;
  std::string source;
  std::string last_status;
  std::string last_error;
  uint64_t loops_accepted = 0;
  std::vector<KeyframePose> keyframes;
  std::vector<LoopEdge> loop_edges;
};

inline std::string read_text_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

inline void print_usage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [--host URL] [--seconds N] [--out FILE]\n"
      << "       [--calibration FILE] [--axes xy|xz|yz] [--update-ms N] [--no-start]\n"
      << "\n"
      << "Example:\n"
      << "  " << argv0 << " --host http://192.168.7.1 --seconds 120 --out loopclosure.svg\n"
      << "\n"
      << "Build example from mighty-protocol root on macOS arm64:\n"
      << "  c++ -arch arm64 -std=c++17 -DMIGHTY_PROTOCOL_ENABLE_LOOPCLOSURE -I. \\\n"
      << "    -Ilib/loopclosure/macos-arm64-static/include \\\n"
      << "    examples/cpp/wip/loopclosure_sdk/main.cpp \\\n"
      << "    lib/loopclosure/macos-arm64-static/lib/libmighty_loopclosure_device.dylib \\\n"
      << "    -Wl,-rpath,$PWD/lib/loopclosure/macos-arm64-static/lib \\\n"
      << "    -o /tmp/loopclosure_sdk_example\n";
}

inline bool parse_args(int argc, char** argv, Options* opts) {
  if (!opts) return false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? argv[i] : "";
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << name << " requires a value\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (arg == "--host") {
      const char* v = need_value("--host");
      if (!v) return false;
      opts->host = v;
    } else if (arg == "--seconds") {
      const char* v = need_value("--seconds");
      if (!v) return false;
      opts->seconds = std::max(1, std::atoi(v));
    } else if (arg == "--out") {
      const char* v = need_value("--out");
      if (!v) return false;
      opts->output_svg = v;
    } else if (arg == "--calibration") {
      const char* v = need_value("--calibration");
      if (!v) return false;
      opts->calibration_yaml = read_text_file(v);
      if (opts->calibration_yaml.empty()) {
        std::cerr << "failed to read calibration file: " << v << "\n";
        return false;
      }
    } else if (arg == "--axes") {
      const char* v = need_value("--axes");
      if (!v) return false;
      opts->axes = v;
      if (opts->axes != "xy" && opts->axes != "xz" && opts->axes != "yz") {
        std::cerr << "--axes must be xy, xz, or yz\n";
        return false;
      }
    } else if (arg == "--update-ms") {
      const char* v = need_value("--update-ms");
      if (!v) return false;
      opts->update_ms = std::max(100, std::atoi(v));
    } else if (arg == "--no-start") {
      opts->start_vio = false;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }
  return true;
}

inline void set_source(State* state, const std::string& source) {
  if (!state) return;
  std::lock_guard<std::mutex> lock(state->mu);
  state->source = source;
}

inline void set_status(State* state, const std::string& status) {
  if (!state) return;
  std::lock_guard<std::mutex> lock(state->mu);
  state->last_status = status;
}

inline void set_error(State* state, const std::string& error) {
  if (!state) return;
  std::lock_guard<std::mutex> lock(state->mu);
  state->last_error = error;
}

inline void record_keyframe_pose(State* state, const PoseFrame& pose) {
  if (!state || !pose.is_keyframe) return;
  std::lock_guard<std::mutex> lock(state->mu);
  state->keyframes.push_back(KeyframePose{
      pose.raw_position_m.value_or(pose.position_m),
      pose.position_m,
  });
}

inline void record_loop_event(State* state, const LoopClosureEvent& event) {
  if (!state) return;
  std::lock_guard<std::mutex> lock(state->mu);
  state->loops_accepted += 1;
  state->loop_edges.push_back(
      LoopEdge{event.current_keyframe, event.matched_keyframe, event.type});
}

inline std::string fmt(double v, int precision = 3) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << v;
  return ss.str();
}

inline void print_loop_event(const LoopClosureEvent& e) {
  std::cout << "[loop] type=" << e.type
            << " current=" << e.current_keyframe
            << " matched=" << e.matched_keyframe;
  std::cout << std::endl;
}

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

inline Point2 project(const std::array<double, 3>& p, const std::string& axes) {
  if (axes == "xz") return {p[0], p[2]};
  if (axes == "yz") return {p[1], p[2]};
  return {p[0], p[1]};
}

inline std::string xml_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char ch : s) {
    switch (ch) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out.push_back(ch); break;
    }
  }
  return out;
}

inline std::string polyline_points(const std::vector<Point2>& points,
                                   double min_x,
                                   double min_y,
                                   double scale,
                                   double x0,
                                   double y0,
                                   double plot_h,
                                   double x_offset,
                                   double y_offset) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  for (const auto& p : points) {
    const double sx = x0 + x_offset + (p.x - min_x) * scale;
    const double sy = y0 + plot_h - y_offset - (p.y - min_y) * scale;
    ss << sx << "," << sy << " ";
  }
  return ss.str();
}

inline void expand_bounds(const Point2& p, double* min_x, double* min_y, double* max_x, double* max_y) {
  *min_x = std::min(*min_x, p.x);
  *min_y = std::min(*min_y, p.y);
  *max_x = std::max(*max_x, p.x);
  *max_y = std::max(*max_y, p.y);
}

inline bool write_svg(const Options& opts, const State& state, const std::string& path) {
  State snapshot;
  {
    std::lock_guard<std::mutex> lock(state.mu);
    snapshot.source = state.source;
    snapshot.last_status = state.last_status;
    snapshot.last_error = state.last_error;
    snapshot.loops_accepted = state.loops_accepted;
    snapshot.keyframes = state.keyframes;
    snapshot.loop_edges = state.loop_edges;
  }

  std::vector<Point2> kf_raw;
  std::vector<Point2> kf_opt;
  kf_raw.reserve(snapshot.keyframes.size());
  kf_opt.reserve(snapshot.keyframes.size());

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto& kf : snapshot.keyframes) {
    const Point2 raw = project(kf.raw_position_m, opts.axes);
    const Point2 opt = project(kf.position_m, opts.axes);
    kf_raw.push_back(raw);
    kf_opt.push_back(opt);
    expand_bounds(raw, &min_x, &min_y, &max_x, &max_y);
    expand_bounds(opt, &min_x, &min_y, &max_x, &max_y);
  }

  if (!std::isfinite(min_x) || !std::isfinite(min_y)) {
    min_x = min_y = -1.0;
    max_x = max_y = 1.0;
  }
  if (std::abs(max_x - min_x) < 1e-6) {
    min_x -= 1.0;
    max_x += 1.0;
  }
  if (std::abs(max_y - min_y) < 1e-6) {
    min_y -= 1.0;
    max_y += 1.0;
  }

  constexpr double width = 1320.0;
  constexpr double height = 760.0;
  constexpr double margin = 48.0;
  constexpr double header_h = 128.0;
  constexpr double gap = 42.0;
  constexpr double plot_w = (width - 2.0 * margin - gap) / 2.0;
  constexpr double plot_h = height - header_h - margin;
  const double scale = std::min(plot_w / (max_x - min_x),
                                plot_h / (max_y - min_y));
  const double scaled_w = (max_x - min_x) * scale;
  const double scaled_h = (max_y - min_y) * scale;
  const double x_offset = (plot_w - scaled_w) * 0.5;
  const double y_offset = (plot_h - scaled_h) * 0.5;
  auto sx = [&](const Point2& p, double x0) {
    return x0 + x_offset + (p.x - min_x) * scale;
  };
  auto sy = [&](const Point2& p, double y0) {
    return y0 + plot_h - y_offset - (p.y - min_y) * scale;
  };

  std::ofstream out(path);
  if (!out) {
    std::cerr << "failed to open " << path << " for writing\n";
    return false;
  }

  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 "
      << width << " " << height << "\" width=\"" << width
      << "\" height=\"" << height
      << "\" role=\"img\" font-family=\"system-ui, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif\">\n";
  out << "<title>Mighty Loop Closure SDK Top-Down View</title>\n";
  out << "<desc>Raw and optimized top-down trajectories with accepted loop closure linkages.</desc>\n";
  out << "<rect x=\"0\" y=\"0\" width=\"" << width << "\" height=\"" << height << "\" fill=\"#f8f8f4\"/>";
  out << "<g font-size=\"13\" fill=\"#0f172a\">"
      << "<text x=\"48\" y=\"40\" font-size=\"18\" font-weight=\"700\">Mighty Loop Closure SDK</text>"
      << "<text x=\"48\" y=\"66\">source: " << xml_escape(snapshot.source.empty() ? "unknown" : snapshot.source)
      << " | axes: " << xml_escape(opts.axes)
      << " | keyframes: " << snapshot.keyframes.size()
      << " | loops: " << snapshot.loops_accepted << "</text>"
      << "<line x1=\"48\" y1=\"94\" x2=\"100\" y2=\"94\" stroke=\"#666666\" stroke-width=\"4\" opacity=\"0.55\" stroke-dasharray=\"10 8\"/><text x=\"112\" y=\"98\">raw keyframes</text>"
      << "<line x1=\"258\" y1=\"94\" x2=\"310\" y2=\"94\" stroke=\"#ff0055\" stroke-width=\"4\"/><text x=\"322\" y=\"98\">optimized keyframes</text>"
      << "<line x1=\"512\" y1=\"94\" x2=\"564\" y2=\"94\" stroke=\"#0099ff\" stroke-width=\"2.5\" stroke-dasharray=\"8 5\"/><text x=\"576\" y=\"98\">accepted loop links</text>"
      << "</g>";

  const double raw_x = margin;
  const double opt_x = margin + plot_w + gap;
  const double plot_y = header_h;
  auto draw_grid = [&](double x0) {
    out << "<rect x=\"" << x0 << "\" y=\"" << plot_y << "\" width=\"" << plot_w
        << "\" height=\"" << plot_h << "\" fill=\"#ffffff\" stroke=\"#d4d4ce\"/>";
    for (int i = 1; i < 10; ++i) {
      const double x = x0 + plot_w * i / 10.0;
      const double y = plot_y + plot_h * i / 10.0;
      out << "<line x1=\"" << x << "\" y1=\"" << plot_y << "\" x2=\"" << x
          << "\" y2=\"" << plot_y + plot_h << "\" stroke=\"#e4e4df\"/>";
      out << "<line x1=\"" << x0 << "\" y1=\"" << y << "\" x2=\"" << x0 + plot_w
          << "\" y2=\"" << y << "\" stroke=\"#e4e4df\"/>";
    }
  };
  auto draw_loop_edges = [&](const std::vector<Point2>& points, double x0) {
    for (const auto& edge : snapshot.loop_edges) {
      if (edge.current_keyframe >= points.size() || edge.matched_keyframe >= points.size()) continue;
      const Point2 a = points[edge.current_keyframe];
      const Point2 b = points[edge.matched_keyframe];
      out << "<line x1=\"" << sx(a, x0) << "\" y1=\"" << sy(a, plot_y)
          << "\" x2=\"" << sx(b, x0) << "\" y2=\"" << sy(b, plot_y)
          << "\" stroke=\"#0099ff\" stroke-width=\"2.5\" stroke-dasharray=\"8 5\" opacity=\"0.85\"/>";
    }
  };
  auto draw_keyframe_points = [&](const std::vector<Point2>& points, double x0, const char* stroke) {
    for (size_t i = 0; i < points.size(); ++i) {
      const Point2 p = points[i];
      out << "<circle cx=\"" << sx(p, x0) << "\" cy=\"" << sy(p, plot_y)
          << "\" r=\"3.5\" fill=\"#ffcc00\" stroke=\"" << stroke
          << "\" stroke-width=\"1\"><title>keyframe " << i << "</title></circle>";
    }
  };

  draw_grid(raw_x);
  draw_grid(opt_x);
  out << "<text x=\"" << raw_x << "\" y=\"" << plot_y - 14
      << "\" font-size=\"14\" font-weight=\"700\" fill=\"#0f172a\">Unoptimized keyframes</text>";
  out << "<text x=\"" << opt_x << "\" y=\"" << plot_y - 14
      << "\" font-size=\"14\" font-weight=\"700\" fill=\"#0f172a\">Optimized keyframes</text>";

  draw_loop_edges(kf_raw, raw_x);
  draw_loop_edges(kf_opt, opt_x);
  if (kf_raw.size() >= 2) {
    out << "<polyline points=\"" << polyline_points(kf_raw, min_x, min_y, scale, raw_x, plot_y, plot_h, x_offset, y_offset)
        << "\" fill=\"none\" stroke=\"#666666\" stroke-width=\"4\" opacity=\"0.65\" stroke-dasharray=\"10 8\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/>";
  }
  if (kf_opt.size() >= 2) {
    out << "<polyline points=\"" << polyline_points(kf_opt, min_x, min_y, scale, opt_x, plot_y, plot_h, x_offset, y_offset)
        << "\" fill=\"none\" stroke=\"#ff0055\" stroke-width=\"4\" opacity=\"0.95\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/>";
  }
  draw_keyframe_points(kf_raw, raw_x, "#666666");
  draw_keyframe_points(kf_opt, opt_x, "#ff0055");
  out << "</svg>\n";
  return true;
}

}  // namespace loopclosure_sdk_example
