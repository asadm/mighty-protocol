# Mighty Mapper C++ Example

Native live mapper example using:

- Mighty protocol C++ SDK for image, pose, and calibration
- `mighty_mapper::Mapper` for mapping
- Pangolin for a simple viewer

The viewer intentionally draws its own snapshot view so we can debug coordinate
scale/orientation before returning to the WASM/Three.js path. It uses a white
background, black map points, and cyan/magenta trajectory lines.

## Build

From this directory:

```bash
cmake -S . -B build
cmake --build build --target mighty_mapper_live -j
```

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

By default the example mirrors the offline mapper runner for bag playback:
`start=12`, `pose-max-dt-ms=5`, and a large image queue so the mapper does not
drop most of the finite dataset while it catches up.

Useful options:

```bash
./build/mighty_mapper_live --base-url http://127.0.0.1:8084 --pose-max-dt-ms=80
./build/mighty_mapper_live --base-url http://127.0.0.1:8084 --start-frame=0
./build/mighty_mapper_live --base-url http://127.0.0.1:8084 --max-queued-images=120
./build/mighty_mapper_live --point-stride=2
```
