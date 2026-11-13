# MightyOccupancyGrid C++ Example

This example requires an installed native algorithms SDK at
`lib/algorithms/current`. From the protocol repository root, build and run:

```bash
cmake -S examples/cpp/wip/occupancy_grid_sdk \
  -B examples/cpp/wip/occupancy_grid_sdk/build
cmake --build examples/cpp/wip/occupancy_grid_sdk/build -j
examples/cpp/wip/occupancy_grid_sdk/build/mighty_occupancy_grid_example \
  --host http://192.168.7.1
```

The example links OpenCV `core`, `imgcodecs`, and `imgproc` so
`MightyOccupancyGrid` can decode either mono JPEG or RAW input before passing
pixels to the native occupancy runtime.

The example owns the processing thread. Incoming client callbacks only enqueue
data; `MightyOccupancyGrid::process()` waits for work and processes one finite
batch. Applications can call `tryProcess()` from an existing event loop instead.

Without `MIGHTY_PROTOCOL_ENABLE_OCCUPANCY_GRID`, including `cpp/mighty_sdk.h`
does not include or link the optional runtime.
