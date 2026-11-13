# Mighty Mapper C++ Example

Native live mapper example using:

- Mighty protocol C++ SDK for image, pose, and calibration
- An installed `libmighty_algorithms` runtime
- Pangolin for a simple viewer

The viewer displays map updates on a white background with blue map points
and a red trajectory.

## Build

From this directory:

```bash
cmake -S . -B build
cmake --build build --target mighty_mapper_live -j
```

This example requires the native algorithms SDK. CMake looks in
`../../../../lib/algorithms/current` by default. To use an installed SDK, set
`MIGHTY_ALGORITHMS_SDK=/path/to/mighty-algorithms-sdk`.

On Apple Silicon, the CMake file defaults to an `arm64` target when using
Homebrew dependencies from `/opt/homebrew`, including when the invoking shell is
running under Rosetta.

## Run

Connect Mighty Camera and start VIO in the visualizer, then run:

```bash
./build/mighty_mapper_live --base-url http://192.168.7.1
```

The example uses the mapper defaults from `libmighty_algorithms`.
The viewer window stays open after the stream ends so the final map can be
inspected. Map points use Mighty blue (`#0099ff`) and the pose trajectory uses
Mighty red (`#ff0055`).

Useful options:

```bash
./build/mighty_mapper_live --point-stride=2
./build/mighty_mapper_live --auto-exit
```
