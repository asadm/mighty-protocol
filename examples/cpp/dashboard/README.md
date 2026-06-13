# C++ Example (OpenCV + CMake)

This example shows a lightweight C++ dashboard built on the Mighty SDK using OpenCV HighGUI.

Main file:
- `main.cpp`

## Preview

![C++ SDK Dashboard Preview](./screen.png)

Features:
- Uses SDK defaults via `MightyWebDevice()` (built-in host fallback / retry path)
- Streams RAW/SRAW image preview
- Shows VIO status + host info
- Plots IMU accel/gyro traces
- Plots pose trajectory in a simple 3D-style view
- Start/Stop VIO control (button click or `v` key)

## Dependencies

- C++17 compiler
- CMake >= 3.16
- OpenCV 4 (`core`, `imgproc`, `highgui`)

## Build

From `examples/cpp/dashboard`:

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/mightyapp
```

Controls:
- `q` or `Esc`: quit
- `v`: start/stop VIO
- Mouse click on Start/Stop button: start/stop VIO
