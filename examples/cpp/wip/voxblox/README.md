# Mighty Protocol + voxblox

This native GUI application builds and displays a live voxblox TSDF from
Mighty metric-depth frames and timestamp-matched VIO poses. Its Pangolin window
contains a navigable 3D mesh, VIO trajectory, colorized depth preview, live
fusion statistics, and VIO/map controls. All device interaction uses the
Mighty Protocol C++ SDK:

- `set_depth_estimation_enabled(true)` starts depth automatically.
- `start_vio()` and `stop_vio()` back the interactive VIO controls.
- `on_depth`, `on_pose`, and `on_vio_state` provide the fusion inputs and
  tracking gate.
- `get_calibration()` supplies the normalized camera-to-IMU and derived
  body-to-camera transforms needed to turn the public `odom -> base_link` pose
  into `odom -> cam0_rectified`.

No ROS or catkin components are used at build time or runtime. Plain CMake
fetches pinned voxblox and minkindr sources, generates the upstream protobuf
messages, and builds only the TSDF, meshing, and serialization code needed by
this application. Pangolin provides the same native 3D UI used by the Mighty
mapper C++ example.

## Prerequisites

Install CMake, Git, Eigen, protobuf, glog, gflags, and Pangolin. On macOS with
Homebrew, install the non-GUI dependencies with:

```bash
brew install cmake eigen protobuf glog gflags
```

On Ubuntu/Debian:

```bash
sudo apt install cmake git libeigen3-dev libprotobuf-dev protobuf-compiler \
  libgoogle-glog-dev libgflags-dev
```

This Mighty checkout already provides Pangolin under
`external/pangolin-install`, which CMake detects automatically. With a separate
Pangolin installation, provide its prefix when configuring:

```bash
cmake -S . -B build \
  -DMIGHTY_PANGOLIN_ROOT=/path/to/pangolin/install
```

Then configure and build directly:

```bash
cd mighty-web/mighty-protocol/examples/cpp/wip/voxblox
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The first configure downloads the pinned
[voxblox](https://github.com/ethz-asl/voxblox) and
[minkindr](https://github.com/ethz-asl/minkindr) revisions into `build/_deps`.
For offline or pre-fetched builds, pass repository roots explicitly:

```bash
cmake -S . -B build \
  -DMIGHTY_VOXBLOX_SOURCE_DIR=/path/to/voxblox \
  -DMIGHTY_MINKINDR_SOURCE_DIR=/path/to/minkindr
```

## Run

Connect to a Mighty endpoint and optionally start VIO immediately:

```bash
./build/mighty_voxblox --host http://192.168.7.1
./build/mighty_voxblox --host http://127.0.0.1:8084 --start-vio
```

Depth estimation is enabled as soon as the protocol stream connects. VIO is
left under user control unless `--start-vio` is passed. The native side panel
provides Start VIO, Stop VIO, Clear Map, Save Mesh, Save Map, and combined save
buttons. It also has follow-camera, mesh, wireframe, trajectory, and depth
preview toggles. The viewer starts in free-camera mode: right-drag orbits,
left-drag pans, and the scroll wheel zooms. For ghost-fly navigation, click the
map, use middle-drag to look, `W/A/S/D` to move, and `Q/E` to move down/up.
`Fly Speed m/s` controls translation speed. Enabling Follow Pose tracks the
latest pose; any mouse or fly-key interaction automatically turns follow off so
manual camera control is never overwritten.

Stopping VIO or reaching the end of a replay freezes the last map and leaves
the window in `PAUSED — MAP FROZEN` inspection mode. The process exits only
when you close the Pangolin window or send an interrupt, so orbit/fly and save
controls remain available after the stream has ended.

The displayed mesh is extracted incrementally on a background thread every
500 ms by default. Change that interval with `--mesh-update-ms N`.

By default, exit writes `mighty_voxblox_mesh.ply` and
`mighty_voxblox_map.voxblox` if at least one frame was integrated. It also
disables device depth estimation; pass `--leave-depth-on` to retain it.

Useful fusion controls include:

```bash
./build/mighty_voxblox \
  --host http://192.168.7.1 \
  --voxel-size 0.05 \
  --truncation-distance 0.15 \
  --min-depth 0.20 \
  --max-depth 5.0 \
  --stride 2 \
  --integrator fast
```

Run `./build/mighty_voxblox --help` for all options.

## Fusion contract

The example only fuses public poses with an orientation, a protocol timestamp,
confidence at or above `0.5` (configurable), and VIO state `TRACKING`. A depth
frame is paired with the nearest pose after the pose stream reaches its
timestamp; the default maximum delta is 15 ms. A small bounded worker queue
drops old frames if CPU integration falls behind.

Depth is reconstructed with the intrinsics and depth convention carried by
each `DepthFrame`. Both z-depth and ray-range conventions are supported, but
the frame must be rectified. Body poses are converted to optical-camera poses
using the calibration fetched over Mighty Protocol. Camera-type poses can be
used directly.

The calibration fetch is part of the SDK, not this example. It normalizes both
Kalibr `cam0` row-list YAML (including rosbag overrides) and Mighty's flat
OpenCV FileStorage YAML into the same `CameraCalibration` object. The example
exits with an explicit error if `cam0.body_from_camera` is unavailable rather
than accepting depth frames it cannot place in the map.

When the protocol emits `RSET`, both pending synchronization data and the local
TSDF are cleared. This prevents maps from two different VIO `odom` frames from
being combined after a stop/restart or reset.

The default `fast` integrator is intended for dense, small-voxel real-time
fusion. `merged` and `simple` are available for comparison.
