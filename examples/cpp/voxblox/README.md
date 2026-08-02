# Mighty Protocol + voxblox

This example builds a live voxblox TSDF from Mighty metric-depth frames and
timestamp-matched VIO poses. All device interaction uses the Mighty Protocol
C++ SDK:

- `set_depth_estimation_enabled(true)` starts depth automatically.
- `start_vio()` and `stop_vio()` back the interactive VIO controls.
- `on_depth`, `on_pose`, and `on_vio_state` provide the fusion inputs and
  tracking gate.
- `config_get_text("calib")` supplies the camera-to-IMU calibration needed to
  turn the public `odom -> base_link` pose into `odom -> cam0_rectified`.

No ROS or catkin components are used at build time or runtime. Plain CMake
fetches pinned voxblox and minkindr sources, generates the upstream protobuf
messages, and builds only the TSDF, meshing, and serialization code needed by
this example.

## Prerequisites

Install CMake, Git, Eigen, protobuf, glog, and gflags. On macOS with Homebrew:

```bash
brew install cmake eigen protobuf glog gflags
```

On Ubuntu/Debian:

```bash
sudo apt install cmake git libeigen3-dev libprotobuf-dev protobuf-compiler \
  libgoogle-glog-dev libgflags-dev
```

Then configure and build directly:

```bash
cd mighty-web/mighty-protocol/examples/cpp/voxblox
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
left under user control unless `--start-vio` is passed. At the prompt, use:

```text
start                 start VIO
stop                  stop VIO
toggle                toggle VIO using the last VIO state
status                show synchronization and TSDF statistics
depth                 query the device depth state
mesh [path]           export a PLY mesh
map [path]            serialize the TSDF layer
save                  export both configured outputs
clear                 clear only the local TSDF
quit                  disable depth and exit
```

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

When the protocol emits `RSET`, both pending synchronization data and the local
TSDF are cleared. This prevents maps from two different VIO `odom` frames from
being combined after a stop/restart or reset.

The default `fast` integrator is intended for dense, small-voxel real-time
fusion. `merged` and `simple` are available for comparison.
