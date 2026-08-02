#include "voxblox_viewer.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pangolin/display/default_font.h>
#include <pangolin/pangolin.h>

namespace mighty_voxblox {
namespace {

using Clock = std::chrono::steady_clock;

constexpr int kWindowWidth = 1600;
constexpr int kWindowHeight = 1000;
constexpr int kUiPanelWidth = 250;
constexpr int kDepthPreviewWidth = 400;
constexpr int kDepthPreviewHeight = 300;
constexpr int kPreviewMargin = 16;

constexpr float kBrandRedR = 1.0f;
constexpr float kBrandRedG = 0.0f;
constexpr float kBrandRedB = 85.0f / 255.0f;

std::array<std::uint8_t, 3> depth_color(float depth_m,
                                        float min_depth_m,
                                        float max_depth_m) {
  const float denominator = std::max(1e-6f, max_depth_m - min_depth_m);
  const float t = std::clamp(
      (depth_m - min_depth_m) / denominator, 0.0f, 1.0f);
  const float middle = 1.0f - std::abs(2.0f * t - 1.0f);
  return {
      static_cast<std::uint8_t>(std::lround(255.0f * t)),
      static_cast<std::uint8_t>(std::lround(35.0f + 190.0f * middle)),
      static_cast<std::uint8_t>(std::lround(255.0f * (1.0f - t)))};
}

Eigen::Vector3d render_from_odom(double x, double y, double z) {
  // Mighty odom is FLU (Z up); Pangolin's ground plane below uses Y up.
  return Eigen::Vector3d(x, z, -y);
}

Eigen::Vector3d render_from_odom(const MeshVertex& vertex) {
  return render_from_odom(vertex.x, vertex.y, vertex.z);
}

pangolin::OpenGlMatrix default_view_matrix() {
  return pangolin::ModelViewLookAt(
      4.5, 3.5, 4.5, 0.0, 0.0, 0.0, pangolin::AxisY);
}

class MapNavigationHandler final : public pangolin::Handler3D {
 public:
  MapNavigationHandler(pangolin::OpenGlRenderState& camera,
                       std::function<void()> on_manual_navigation)
      : pangolin::Handler3D(camera, pangolin::AxisY),
        on_manual_navigation_(std::move(on_manual_navigation)) {}

  void Keyboard(pangolin::View& view, unsigned char key, int x, int y,
                bool pressed) override {
    const unsigned char normalized = static_cast<unsigned char>(
        std::tolower(static_cast<unsigned char>(key)));
    if (normalized == 'w' || normalized == 'a' || normalized == 's' ||
        normalized == 'd' || normalized == 'q' || normalized == 'e') {
      movement_keys_[normalized] = pressed;
      if (pressed) manual_navigation();
      return;
    }
    pangolin::Handler3D::Keyboard(view, key, x, y, pressed);
  }

  void Mouse(pangolin::View& view, pangolin::MouseButton button, int x, int y,
             bool pressed, int button_state) override {
    if (pressed) manual_navigation();
    pangolin::Handler3D::Mouse(
        view, button, x, y, pressed, button_state);
  }

  void MouseMotion(pangolin::View& view, int x, int y,
                   int button_state) override {
    if (button_state != 0) manual_navigation();
    pangolin::Handler3D::MouseMotion(view, x, y, button_state);
  }

  void Special(pangolin::View& view, pangolin::InputSpecial input_type,
               float x, float y, float p1, float p2, float p3, float p4,
               int button_state) override {
    manual_navigation();
    pangolin::Handler3D::Special(
        view, input_type, x, y, p1, p2, p3, p4, button_state);
  }

  Eigen::Vector3d local_fly_direction() const {
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    if (key_down('a')) direction.x() -= 1.0;
    if (key_down('d')) direction.x() += 1.0;
    if (key_down('q')) direction.y() -= 1.0;
    if (key_down('e')) direction.y() += 1.0;
    // OpenGL cameras look along local -Z.
    if (key_down('w')) direction.z() -= 1.0;
    if (key_down('s')) direction.z() += 1.0;
    const double norm = direction.norm();
    return norm > 1.0 ? direction / norm : direction;
  }

 private:
  bool key_down(unsigned char key) const {
    return movement_keys_[key];
  }

  void manual_navigation() {
    if (on_manual_navigation_) on_manual_navigation_();
  }

  std::array<bool, 256> movement_keys_{};
  std::function<void()> on_manual_navigation_;
};

void apply_ghost_fly(pangolin::OpenGlRenderState* camera,
                     const MapNavigationHandler& navigation,
                     double speed_mps,
                     double elapsed_s) {
  if (!camera || !std::isfinite(speed_mps) || speed_mps <= 0.0 ||
      !std::isfinite(elapsed_s) || elapsed_s <= 0.0) {
    return;
  }
  const Eigen::Vector3d local_direction = navigation.local_fly_direction();
  if (local_direction.squaredNorm() < 1e-12) return;

  pangolin::OpenGlMatrix& world_to_camera = camera->GetModelViewMatrix();
  Eigen::Matrix3d rotation_camera_from_world;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation_camera_from_world(row, column) =
          world_to_camera(row, column);
    }
  }
  Eigen::Vector3d translation_camera_from_world(
      world_to_camera(0, 3), world_to_camera(1, 3),
      world_to_camera(2, 3));
  Eigen::Vector3d camera_position_world =
      -rotation_camera_from_world.transpose() * translation_camera_from_world;
  camera_position_world += rotation_camera_from_world.transpose() *
      local_direction * (speed_mps * elapsed_s);
  translation_camera_from_world =
      -rotation_camera_from_world * camera_position_world;
  for (int row = 0; row < 3; ++row) {
    world_to_camera(row, 3) = translation_camera_from_world[row];
  }
}

void draw_grid(float extent, float step) {
  glLineWidth(1.0f);
  glColor3f(0.78f, 0.82f, 0.84f);
  glBegin(GL_LINES);
  for (float value = -extent; value <= extent + 1e-5f; value += step) {
    glVertex3f(-extent, 0.0f, value);
    glVertex3f(extent, 0.0f, value);
    glVertex3f(value, 0.0f, -extent);
    glVertex3f(value, 0.0f, extent);
  }
  glEnd();

  glLineWidth(3.0f);
  glBegin(GL_LINES);
  glColor3f(0.88f, 0.18f, 0.18f);
  glVertex3f(0.0f, 0.002f, 0.0f);
  glVertex3f(1.0f, 0.002f, 0.0f);
  glColor3f(0.18f, 0.72f, 0.28f);
  glVertex3f(0.0f, 0.002f, 0.0f);
  glVertex3f(0.0f, 1.0f, 0.0f);
  glColor3f(0.16f, 0.38f, 0.92f);
  glVertex3f(0.0f, 0.002f, 0.0f);
  glVertex3f(0.0f, 0.002f, 1.0f);
  glEnd();
}

void draw_mesh(const MeshSnapshot& mesh, bool wireframe) {
  if (mesh.vertices.size() < 3) return;

  glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);
  glLineWidth(wireframe ? 1.0f : 1.0f);
  glBegin(GL_TRIANGLES);
  const Eigen::Vector3d light = Eigen::Vector3d(0.25, 0.9, 0.35).normalized();
  for (std::size_t index = 0; index + 2 < mesh.vertices.size(); index += 3) {
    const Eigen::Vector3d p0 = render_from_odom(mesh.vertices[index]);
    const Eigen::Vector3d p1 = render_from_odom(mesh.vertices[index + 1]);
    const Eigen::Vector3d p2 = render_from_odom(mesh.vertices[index + 2]);
    Eigen::Vector3d normal = (p1 - p0).cross(p2 - p0);
    const double norm = normal.norm();
    if (norm > 1e-9) normal /= norm;
    const float shade = static_cast<float>(
        0.48 + 0.52 * std::abs(normal.dot(light)));

    const Eigen::Vector3d points[3] = {p0, p1, p2};
    for (int vertex_index = 0; vertex_index < 3; ++vertex_index) {
      const MeshVertex& vertex = mesh.vertices[index + vertex_index];
      if (wireframe) {
        glColor3f(0.05f, 0.23f, 0.34f);
      } else {
        glColor3f(shade * static_cast<float>(vertex.r) / 255.0f,
                  shade * static_cast<float>(vertex.g) / 255.0f,
                  shade * static_cast<float>(vertex.b) / 255.0f);
      }
      glVertex3d(points[vertex_index].x(), points[vertex_index].y(),
                 points[vertex_index].z());
    }
  }
  glEnd();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void draw_trajectory(const std::vector<ViewerPose>& trajectory) {
  if (trajectory.empty()) return;

  if (trajectory.size() > 1) {
    glColor3f(kBrandRedR, kBrandRedG, kBrandRedB);
    glLineWidth(4.0f);
    glBegin(GL_LINE_STRIP);
    for (const ViewerPose& pose : trajectory) {
      const Eigen::Vector3d point = render_from_odom(
          pose.position_m[0], pose.position_m[1], pose.position_m[2]);
      glVertex3d(point.x(), point.y(), point.z());
    }
    glEnd();
  }

  const ViewerPose& latest = trajectory.back();
  const Eigen::Vector3d origin = render_from_odom(
      latest.position_m[0], latest.position_m[1], latest.position_m[2]);
  glPointSize(12.0f);
  glColor3f(kBrandRedR, kBrandRedG, kBrandRedB);
  glBegin(GL_POINTS);
  glVertex3d(origin.x(), origin.y(), origin.z());
  glEnd();

  Eigen::Quaterniond rotation(
      latest.orientation_xyzw[3], latest.orientation_xyzw[0],
      latest.orientation_xyzw[1], latest.orientation_xyzw[2]);
  if (rotation.norm() < 1e-9) return;
  rotation.normalize();

  const Eigen::Vector3d axes_odom[3] = {
      rotation * Eigen::Vector3d::UnitX(),
      rotation * Eigen::Vector3d::UnitY(),
      rotation * Eigen::Vector3d::UnitZ()};
  const float colors[3][3] = {
      {0.95f, 0.20f, 0.20f},
      {0.20f, 0.78f, 0.30f},
      {0.18f, 0.42f, 0.95f}};
  glLineWidth(4.0f);
  glBegin(GL_LINES);
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d endpoint_odom =
        Eigen::Vector3d(latest.position_m[0], latest.position_m[1],
                        latest.position_m[2]) +
        axes_odom[axis] * 0.25;
    const Eigen::Vector3d endpoint = render_from_odom(
        endpoint_odom.x(), endpoint_odom.y(), endpoint_odom.z());
    glColor3fv(colors[axis]);
    glVertex3d(origin.x(), origin.y(), origin.z());
    glVertex3d(endpoint.x(), endpoint.y(), endpoint.z());
  }
  glEnd();
}

std::string count_text(std::uint64_t value) {
  std::ostringstream stream;
  if (value >= 1'000'000) {
    stream << std::fixed << std::setprecision(1)
           << static_cast<double>(value) / 1'000'000.0 << "M";
  } else if (value >= 1'000) {
    stream << std::fixed << std::setprecision(1)
           << static_cast<double>(value) / 1'000.0 << "k";
  } else {
    stream << value;
  }
  return stream.str();
}

}  // namespace

void ViewerState::set_status(std::string status) {
  std::lock_guard<std::mutex> lock(mutex_);
  status_ = std::move(status);
}

void ViewerState::set_mesh(std::shared_ptr<const MeshSnapshot> mesh) {
  if (!mesh) return;
  std::lock_guard<std::mutex> lock(mutex_);
  mesh_ = std::move(mesh);
}

void ViewerState::update_depth(const mighty_protocol::DepthFrame& depth,
                               float min_depth_m,
                               float max_depth_m) {
  const std::uint64_t expected =
      static_cast<std::uint64_t>(depth.width) * depth.height;
  if (depth.width == 0 || depth.height == 0 ||
      expected != depth.depth_mm.size() ||
      !std::isfinite(depth.depth_scale_m) || depth.depth_scale_m <= 0.0f) {
    return;
  }

  auto preview = std::make_shared<DepthPreview>();
  preview->timestamp_ns = depth.timestamp_ns;
  preview->width = depth.width;
  preview->height = depth.height;
  preview->rgb.resize(static_cast<std::size_t>(expected) * 3);
  for (std::size_t index = 0; index < depth.depth_mm.size(); ++index) {
    const std::uint16_t sample = depth.depth_mm[index];
    std::array<std::uint8_t, 3> color{18, 20, 22};
    if (sample != depth.invalid_value) {
      const float depth_m = sample * depth.depth_scale_m;
      if (std::isfinite(depth_m) && depth_m >= min_depth_m &&
          depth_m <= max_depth_m) {
        color = depth_color(depth_m, min_depth_m, max_depth_m);
      }
    }
    preview->rgb[index * 3] = color[0];
    preview->rgb[index * 3 + 1] = color[1];
    preview->rgb[index * 3 + 2] = color[2];
  }

  std::lock_guard<std::mutex> lock(mutex_);
  depth_ = std::move(preview);
}

void ViewerState::add_pose(const ViewerPose& pose) {
  for (double value : pose.position_m) {
    if (!std::isfinite(value)) return;
  }
  for (double value : pose.orientation_xyzw) {
    if (!std::isfinite(value)) return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!trajectory_.empty() &&
      trajectory_.back().timestamp_ns == pose.timestamp_ns) {
    trajectory_.back() = pose;
    return;
  }
  trajectory_.push_back(pose);
  if (trajectory_.size() > kMaxTrajectoryPoses) {
    trajectory_.erase(
        trajectory_.begin(),
        trajectory_.begin() + static_cast<std::ptrdiff_t>(
            kMaxTrajectoryPoses / 4));
  }
}

void ViewerState::clear_map_visuals() {
  std::lock_guard<std::mutex> lock(mutex_);
  mesh_ = std::make_shared<MeshSnapshot>();
  trajectory_.clear();
}

ViewerState::Snapshot ViewerState::snapshot() const {
  Snapshot result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.mesh = mesh_;
  result.depth = depth_;
  result.trajectory = trajectory_;
  result.status = status_;
  return result;
}

void run_voxblox_viewer(ViewerState* state,
                        const ViewerActions& actions,
                        const ViewerOptions& options,
                        const std::function<bool()>& should_stop) {
  if (!state) return;

  pangolin::CreateWindowAndBind(
      "Mighty Voxblox — Live Depth Fusion", kWindowWidth, kWindowHeight);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glClearColor(0.96f, 0.97f, 0.97f, 1.0f);

  pangolin::CreatePanel("ui").SetBounds(
      0.0, 1.0, 0.0, pangolin::Attach::Pix(kUiPanelWidth));
  pangolin::Var<bool> start_vio("ui.Start VIO", false, false);
  pangolin::Var<bool> stop_vio("ui.Stop VIO", false, false);
  pangolin::Var<bool> clear_map("ui.Clear Map", false, false);
  pangolin::Var<bool> save_outputs("ui.Save Mesh + Map", false, false);
  pangolin::Var<bool> save_mesh("ui.Save Mesh", false, false);
  pangolin::Var<bool> save_map("ui.Save Map", false, false);
  pangolin::Var<bool> follow_pose(
      "ui.Follow Pose", options.follow_pose, true);
  pangolin::Var<bool> show_mesh("ui.Show Mesh", true, true);
  pangolin::Var<bool> wireframe("ui.Wireframe", false, true);
  pangolin::Var<bool> show_trajectory("ui.Show Trajectory", true, true);
  pangolin::Var<bool> show_depth("ui.Show Depth Preview", true, true);
  pangolin::Var<bool> reset_view("ui.Reset View", false, false);
  pangolin::Var<double> fly_speed(
      "ui.Fly Speed m/s", 1.5, 0.1, 10.0, false);
  pangolin::Var<std::string> mouse_controls(
      "ui.Mouse", "RMB orbit | LMB pan | wheel zoom",
      pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> fly_controls(
      "ui.Fly", "MMB look | WASD move | Q/E down/up",
      pangolin::META_FLAG_READONLY);

  pangolin::Var<std::string> connection(
      "ui.Connection", "waiting", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> source(
      "ui.Source", "", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> vio_state(
      "ui.VIO", "UNKNOWN", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> depth_frames(
      "ui.Depth Frames", "0", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> matched_frames(
      "ui.Matched Frames", "0", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> integrated_frames(
      "ui.Integrated Frames", "0", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> blocks(
      "ui.TSDF Blocks", "0", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> triangles(
      "ui.Mesh Triangles", "0", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> queue_drops(
      "ui.Queue Drops", "0", pangolin::META_FLAG_READONLY);
  pangolin::Var<std::string> sync_drops(
      "ui.Sync Drops", "0", pangolin::META_FLAG_READONLY);

  pangolin::OpenGlRenderState camera(
      pangolin::ProjectionMatrix(
          kWindowWidth, kWindowHeight, 900, 900,
          kWindowWidth / 2.0, kWindowHeight / 2.0, 0.05, 1000.0),
      default_view_matrix());
  MapNavigationHandler handler(camera, [&follow_pose]() {
    follow_pose = false;
  });
  pangolin::View& world_view = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(kUiPanelWidth), 1.0,
                 -static_cast<float>(kWindowWidth - kUiPanelWidth) /
                     static_cast<float>(kWindowHeight))
      .SetHandler(&handler);
  pangolin::View& depth_view = pangolin::CreateDisplay()
      .SetBounds(
          pangolin::Attach::ReversePix(kPreviewMargin + kDepthPreviewHeight),
          pangolin::Attach::ReversePix(kPreviewMargin),
          pangolin::Attach::ReversePix(kPreviewMargin + kDepthPreviewWidth),
          pangolin::Attach::ReversePix(kPreviewMargin),
          static_cast<float>(kDepthPreviewWidth) / kDepthPreviewHeight);

  std::unique_ptr<pangolin::GlTexture> depth_texture;
  std::uint32_t texture_width = 0;
  std::uint32_t texture_height = 0;
  std::uint64_t uploaded_depth_timestamp =
      std::numeric_limits<std::uint64_t>::max();
  Eigen::Vector3d follow_target = Eigen::Vector3d::Zero();
  bool has_follow_target = false;
  Clock::time_point last_frame = Clock::now();

  while (!pangolin::ShouldQuit() &&
         !(should_stop && should_stop())) {
    if (actions.tick) actions.tick();

    if (pangolin::Pushed(start_vio) && actions.start_vio) {
      actions.start_vio();
    }
    if (pangolin::Pushed(stop_vio) && actions.stop_vio) {
      actions.stop_vio();
    }
    if (pangolin::Pushed(clear_map) && actions.clear_map) {
      actions.clear_map();
    }
    if (pangolin::Pushed(save_outputs) && actions.save_outputs) {
      actions.save_outputs();
    }
    if (pangolin::Pushed(save_mesh) && actions.save_mesh) {
      actions.save_mesh();
    }
    if (pangolin::Pushed(save_map) && actions.save_map) {
      actions.save_map();
    }
    if (pangolin::Pushed(reset_view)) {
      camera.SetModelViewMatrix(default_view_matrix());
      follow_pose = false;
      has_follow_target = false;
    }

    const ViewerState::Snapshot snapshot = state->snapshot();
    const ViewerStats stats = actions.read_stats
        ? actions.read_stats()
        : ViewerStats{};
    connection = stats.inspection_paused
        ? "paused — inspecting map"
        : (stats.connected ? "connected" : "reconnecting");
    source = stats.source;
    vio_state = stats.vio_state;
    depth_frames = count_text(stats.depth_received);
    matched_frames = count_text(stats.matched_frames);
    integrated_frames = count_text(stats.integrated_frames);
    blocks = count_text(stats.allocated_blocks);
    triangles = snapshot.mesh
        ? count_text(snapshot.mesh->triangle_count())
        : "0";
    queue_drops = count_text(stats.queue_drops);
    sync_drops = count_text(stats.unmatched_depth_drops);

    const Clock::time_point now = Clock::now();
    const double dt = std::min(
        0.1,
        std::chrono::duration_cast<std::chrono::duration<double>>(
            now - last_frame).count());
    last_frame = now;
    if (follow_pose && !snapshot.trajectory.empty()) {
      const ViewerPose& latest = snapshot.trajectory.back();
      const Eigen::Vector3d latest_target = render_from_odom(
          latest.position_m[0], latest.position_m[1], latest.position_m[2]);
      const double alpha = 1.0 - std::exp(-dt * 3.0);
      if (!has_follow_target) {
        follow_target = latest_target;
        has_follow_target = true;
      } else {
        follow_target += (latest_target - follow_target) * alpha;
      }
      const Eigen::Vector3d eye =
          follow_target + Eigen::Vector3d(3.2, 2.3, 3.2);
      camera.SetModelViewMatrix(pangolin::ModelViewLookAt(
          eye.x(), eye.y(), eye.z(), follow_target.x(), follow_target.y(),
          follow_target.z(), pangolin::AxisY));
    } else if (!follow_pose) {
      has_follow_target = false;
    }
    if (!follow_pose) {
      apply_ghost_fly(&camera, handler, fly_speed, dt);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    world_view.Activate(camera);
    draw_grid(20.0f, 0.5f);
    if (show_mesh && snapshot.mesh) {
      draw_mesh(*snapshot.mesh, wireframe);
    }
    if (show_trajectory) draw_trajectory(snapshot.trajectory);

    pangolin::default_font()
        .Text("%s  |  VIO %s  |  depth %s  |  mesh %s triangles",
              stats.inspection_paused
                  ? "PAUSED — MAP FROZEN"
                  : (stats.connected ? "CONNECTED" : "RECONNECTING"),
              stats.vio_state.c_str(), count_text(stats.depth_received).c_str(),
              snapshot.mesh
                  ? count_text(snapshot.mesh->triangle_count()).c_str()
                  : "0")
        .DrawWindow(kUiPanelWidth + 14, 20);
    if (!snapshot.status.empty()) {
      pangolin::default_font().Text("%s", snapshot.status.c_str())
          .DrawWindow(kUiPanelWidth + 14, 42);
    }

    if (show_depth && snapshot.depth && !snapshot.depth->rgb.empty()) {
      if (!depth_texture || texture_width != snapshot.depth->width ||
          texture_height != snapshot.depth->height) {
        depth_texture = std::make_unique<pangolin::GlTexture>(
            snapshot.depth->width, snapshot.depth->height, GL_RGB, false, 0,
            GL_RGB, GL_UNSIGNED_BYTE);
        texture_width = snapshot.depth->width;
        texture_height = snapshot.depth->height;
        uploaded_depth_timestamp =
            std::numeric_limits<std::uint64_t>::max();
      }
      if (uploaded_depth_timestamp != snapshot.depth->timestamp_ns) {
        depth_texture->Upload(
            snapshot.depth->rgb.data(), GL_RGB, GL_UNSIGNED_BYTE);
        uploaded_depth_timestamp = snapshot.depth->timestamp_ns;
      }
      glDisable(GL_DEPTH_TEST);
      depth_view.Activate();
      glColor3f(1.0f, 1.0f, 1.0f);
      depth_texture->RenderToViewportFlipY();
      glEnable(GL_DEPTH_TEST);
    }

    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(8));
  }
}

}  // namespace mighty_voxblox
