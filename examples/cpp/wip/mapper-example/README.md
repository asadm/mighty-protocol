# Mighty Mapper C++ Example

Native live mapper example using:

- Mighty protocol C++ SDK for image, pose, and calibration
- `libmighty_loopclosure_device` from the packaged native SDK for mapping
- Pangolin for a simple viewer

The viewer intentionally draws its own snapshot view so we can debug coordinate
scale/orientation before returning to the WASM/Three.js path. It uses a white
background, blue map points, and a red trajectory.

## Build

From this directory:

```bash
cmake -S . -B build
cmake --build build --target mighty_mapper_live -j
```

By default CMake uses the bundled native SDK at
`../../../../lib/loopclosure/macos-arm64-static`. To test another package, set
`MIGHTY_LOOPCLOSURE_SDK=/path/to/mighty-loopclosure-device-sdk`.

On Apple Silicon, the CMake file defaults to an `arm64` target when using
Homebrew dependencies from `/opt/homebrew`, including when the invoking shell is
running under Rosetta.

## Run

Start VIO first, for example:

```bash
./build_app/app ~/datasets/mighty-small-ov9281/calabazas-stairs2.bag \
  main/polaris/config/rockchip1_ov9281_halfres.yaml \
  --vio --wait_for_web_client --fps=60
```

Then run:

```bash
./build/mighty_mapper_live --base-url http://127.0.0.1:8084
```

The example uses the mapper defaults from `libmighty_loopclosure_device`.
The viewer window stays open after the stream ends so the final map can be
inspected. Map points use Mighty blue (`#0099ff`) and the pose trajectory uses
Mighty red (`#ff0055`).

Useful options:

```bash
./build/mighty_mapper_live --point-stride=2
./build/mighty_mapper_live --auto-exit
```
