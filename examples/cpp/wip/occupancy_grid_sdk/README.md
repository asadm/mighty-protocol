# MightyOccupancyGrid C++ Example

Build the optional native runtime once from the protocol repository root:

```bash
scripts/rebuild_algorithms_sdk.sh macos-arm64
```

Then build and run the example:

```bash
cmake -S examples/cpp/wip/occupancy_grid_sdk \
  -B examples/cpp/wip/occupancy_grid_sdk/build
cmake --build examples/cpp/wip/occupancy_grid_sdk/build -j
examples/cpp/wip/occupancy_grid_sdk/build/mighty_occupancy_grid_example \
  --host http://192.168.7.1
```

The example owns the processing thread. Incoming client callbacks only enqueue
data; `MightyOccupancyGrid::process()` waits for work and processes one finite
batch. Applications can call `tryProcess()` from an existing event loop instead.

Without `MIGHTY_PROTOCOL_ENABLE_OCCUPANCY_GRID`, including `cpp/mighty_sdk.h`
does not include or link the optional runtime.
